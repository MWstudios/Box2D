using Box2D;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.Intrinsics.Arm;
using System.Runtime.Intrinsics.X86;
using System.Threading;

namespace Box2D;

public struct Softness
{
    public float biasRate;
    public float massScale;
    public float impulseScale;
    public Softness(float hertz, float zeta, float h)
    {
        if (hertz == 0) { biasRate = 0; massScale = 0; impulseScale = 0; return; }
        float omega = MathF.Tau * hertz;
        float a1 = 2 * zeta + h * omega;
        float a2 = h * omega * a1;
        float a3 = 1 / (1 + a2);
        biasRate = omega / a1;
        massScale = a2 * a3;
        impulseScale = a3;
    }
}

public enum SolverStageType
{
    PrepareJoints, PrepareContacts, IntegrateVelocities,
    WarmStart, Solve, IntegratePositions, Relax, Restitution, StoreImpulses
}
public enum SolverBlockType : byte
{
    Body, Joint, Contact, GraphJoint, GraphContact
}
/// <summary>b2_graphContactBlock
/// Each block of work has a sync index that gets incremented when a worker claims the block. This ensures only a single worker
/// claims a block, yet lets work be distributed dynamically across multiple workers (work stealing). This also reduces contention
/// on a single block index atomic. For non-iterative stages the sync index is simply set to one. For iterative stages (solver
/// iteration) the same block of work is executed once per iteration and the atomic sync index is shared across iterations, so it
/// increases monotonically.
/// Solver work is partitioned into fixed-size blocks that worker threads claim
/// in parallel via atomic CAS on each block's own syncIndex. Three properties
/// of this design matter for performance:
///
/// 1. Distributed contention. Per-block atomic syncIndex avoids the cache line stampede
///    that a single shared fetch_add counter would cause. Once a worker
///    settles into a block range, its CAS targets live in its own L1.
///
/// 2. Monotonic syncIndex across iterations. Iterative stages (warm start,
///    solve, relax) reuse the same block array every sub-step iteration.
///    syncIndex grows each iteration; workers CAS (prev, prev+1), so the
///    main thread never touches any per-block state between iterations.
///    Non-iterative stages simply use syncIndex 1.
///
/// 3. L2 affinity across iterations. Each worker picks a start offset from
///    its workerIndex, then scans forward and (after wrap) backward:
///
///<br/>      blocks:   [0] [1] [2] [3] [4] [5] [6] [7]
///<br/>                 ^           ^           ^   ^
///<br/>                 W0          W1          W2  W3   <- start offsets
///<br/>
///    W0 claims 0,1,2,3 (forward), W1 claims 4,5, etc. Under balanced load
///    each worker re-hits the same block range every iteration, keeping that
///    range's hot data resident in its L2. A failed CAS means a neighbour
///    already claimed the block, so the stealing worker stops -- preserving
///    locality under mild imbalance while still draining the queue.
///
/// A graph color stage lays out joint blocks first, then contact blocks:
///<br/>
///<br/>      stage->blocks ->
///<br/>        +------+------+------+------+------+------+------+
///<br/>        |  J0  |  J1  |  J2  |  C0  |  C1  |  C2  |  C3  |
///<br/>        +------+------+------+------+------+------+------+
///<br/>        &lt;-- graphJointBlocks --&gt;&lt;---- graphContactBlocks ----&gt;
///<br/>
/// Each block carries its type so the dispatcher routes J-blocks to the joint
/// solver and C-blocks to the SIMD contact solver; both kinds run concurrently
/// within the stage -- no barrier between them. The type tag lives on the
/// block (not the stage) so that mixed-type stages can keep the concurrency.</summary>
public struct SolverBlock
{
    public int startIndex;
    public ushort count;
    public SolverBlockType blockType;
    public byte colorIndex;
}
/// <summary>A unit of multithreaded work along with atomic synchronization. The syncIndex grows
/// monotonically allowing the solver block to be re-used across sub-steps.</summary>
public struct SyncBlock
{
    public SolverBlock block;
    public int syncIndex;
}
public unsafe class SolverStage
{
    public SyncBlock* blocks;
    public SolverStageType type;
    public int blockCount;
    public byte colorIndex;
    public int completionCount;
}
/// <summary>Prepare/store run as a flat parallel-for over the whole wide-constraint
/// range. Each span maps a slice of that range back to the owning color's
/// contacts so workers can decode flat wide-slot indices without touching
/// graph state. The spans array has one entry per active color plus a sentinel
/// whose start == wideContactCount.</summary>
public struct ContactPrepareSpan
{
    public int start, count;
    public List<ContactSim> contacts;
}
/// <summary>Similar for joints</summary>
public struct JointPrepareSpan
{
    public int start, count;
    public List<JointSim> joints;
}
public partial class StepContext
{
    /// <summary>time step</summary>
    public float dt;

    /// <summary>inverse time step (0 if dt == 0).</summary>
    public float inv_dt;

    /// <summary>sub-step</summary>
    public float h;
    public float inv_h;

    public int subStepCount;

    public Softness contactSoftness;
    public Softness staticSoftness;

    public float restitutionThreshold;
    public float maxLinearVelocity;

    public World world;
    public ConstraintGraph graph;

    /// <summary>shortcut to body states from awake set</summary>
    public PtrArray<BodyState> states;

    /// <summary>shortcut to body sims from awake set</summary>
    public List<BodySim> sims;

    /// <summary>array of all shape ids for shapes that have enlarged AABBs</summary>
    public int[] enlargedShapes;

    /// <summary>Array of bullet bodies that need continuous collision handling</summary>
    public int[] bulletBodies;
    public int bulletBodyCount;

    /// <summary>contact pointers for simplified parallel-for access.<br/>
    /// - parallel-for collide with no gaps, includes touching and non-touching</summary>
    public ContactSim[] contactSims;

    /// <summary>Flat view of the wide contact constraint array used by prepare and store.
    /// prepareSpans has activeColorCount + 1 entries, the last being a sentinel
    /// at wideContactCount. wideContactConstraints is the contiguous base
    /// pointer; per-color slices live at colors[i].wideConstraints.</summary>
    public IContactConstraintsSIMD wideContactConstraints;
    public ContactPrepareSpan[] contactPrepareSpans;
    public int wideContactCount;

    public JointPrepareSpan[] jointPrepareSpans;
    public int jointCount;

    public int activeColorCount;
    public int workerCount;

    public SolverStage[] stages;
    public bool enableWarmStarting;

    /// <summary>sync index (16-bits) | stage type (16-bits)</summary>
    public uint atomicSyncBits;

    /// <summary>Race flag claimed by whichever runner reaches b2SolverTask with workerIndex 0 first.
    /// The calling thread of b2World_Step also races for this slot so the orchestrator can
    /// always make progress, regardless of how the user's task system schedules tasks (out
    /// of order, fewer threads than workers, or synchronously inside enqueueTaskFcn). The
    /// loser of the race no-ops as workerIndex 0.</summary>
    public int mainClaimed;
}
public unsafe partial class World
{
    public class WorkerContext
    {
        public StepContext context;
        public int workerIndex;
        public object userTask;
    }
    public static void IntegrateVelocitiesTask(ref SolverBlock block, StepContext context)
    {
        Vector2 gravity = context.world.gravity;
        float h = context.h;
        for (int i = block.startIndex; i < block.startIndex + block.count; i++)
        {
            BodySim sim = context.sims[i];
            BodyState* state = context.states.Data + i;
            Vector2 v = state->linearVelocity;
            float w = state->angularVelocity;
            float linearDamping = 1 / (1 + h * sim.linearDamping);
            float angularDamping = 1 / (1 + h * sim.angularDamping);
            float gravityScale = sim.invMass > 0 ? sim.gravityScale : 0;
            Vector2 linearVelocityDelta = h * sim.invMass * sim.force + h * gravityScale * gravity;
            float angularVelocityDelta = h * sim.invInertia * sim.torque;
            v = Vector2.MulAdd(linearVelocityDelta, linearDamping, v);
            w = angularVelocityDelta + angularDamping * w;
            state->linearVelocity = v;
            state->angularVelocity = w;
         }
    }
    public static void IntegratePositionsTask(ref SolverBlock block, StepContext context)
    {
        float h = context.h;
        float maxLinearSpeed = context.maxLinearVelocity;
        float maxAngularSpeed = Box2D.MaxRotation * context.inv_dt;
        float maxLinearSpeedSquared = maxLinearSpeed * maxLinearSpeed;
        float maxAngularSpeedSquared = maxAngularSpeed * maxAngularSpeed;
        for (int i = block.startIndex; i < block.startIndex + block.count; i++)
        {
            BodyState* state = context.states.Data + i;
            Vector2 v = state->linearVelocity;
            float w = state->angularVelocity;
            v.x = state->flags.HasFlag(BodyFlags.LockLinearX) ? 0 : v.x;
            v.y = state->flags.HasFlag(BodyFlags.LockLinearY) ? 0 : v.y;
            w = state->flags.HasFlag(BodyFlags.LockAngularZ) ? 0 : w;
            if (Vector2.Dot(v, v) > maxLinearSpeedSquared)
            {
                float ratio = maxLinearSpeed / v.Length();
                v *= ratio;
                state->flags |= BodyFlags.IsSpeedCapped;
            }
            if (w * w > maxAngularSpeedSquared && !state->flags.HasFlag(BodyFlags.AllowFastRotation))
            {
                float ratio = maxAngularSpeed / Math.Abs(w);
                w *= ratio;
                state->flags |= BodyFlags.IsSpeedCapped;
            }
            state->linearVelocity = v;
            state->angularVelocity = w;
            state->deltaPosition = Vector2.MulAdd(state->deltaPosition, h, state->linearVelocity);
            state->deltaRotation = state->deltaRotation.Integrate(h * state->angularVelocity);
        }
    }
    public class ContinuousContext
    {
        public World world;
        public BodySim fastBodySim;
        public Shape fastShape;
        public Vector2 centroid1, centroid2;
        public Sweep sweep;
        public Position base_;
        public float fraction;
        public List<SensorHit> sensorHits = new(8);
        public List<float> sensorFractions = new(8);
    }
    public static bool ContinuousQueryCallback(int proxyId, ulong userData, object context)
    {
        int shapeId = (int)userData;
        ContinuousContext continuousContext = (ContinuousContext)context;
        Shape fastShape = continuousContext.fastShape;
        Debug.Assert(fastShape.sensorIndex == -1);
        if (shapeId == fastShape.id) return false;
        World world = continuousContext.world;
        Shape shape = world.shapes[shapeId];
        if (shape.bodyId == fastShape.bodyId) return true;
        bool isSensor = shape.sensorIndex != -1;
        if (isSensor && (!shape.enableSensorEvents || !fastShape.enableSensorEvents))
            return true;
        if (!Shape.ShouldShapesCollide(fastShape.filter, shape.filter)) return true;
        Body body = world.bodies[shape.bodyId];
        BodySim bodySim = world.GetBodySim(body);
        Debug.Assert(body.type == BodyType.Static || continuousContext.fastBodySim.flags.HasFlag(BodyFlags.IsBullet));
        if (bodySim.flags.HasFlag(BodyFlags.IsBullet)) return true;
        Body fastBody = world.bodies[continuousContext.fastBodySim.bodyId];
        if (!world.ShouldBodiesCollide(fastBody, body)) return true;
        if (shape.enableCustomFiltering || fastShape.enableCustomFiltering)
        {
            if (world.customFilterFcn != null)
            {
                if (!world.customFilterFcn(new() { index1 = shapeId + 1, world0 = world, generation = shape.generation },
                    new() { index1 = fastShape.id + 1, world0 = world, generation = fastShape.generation },
                    world.customFilterContext))
                    return true;
            }
        }
        if (shape.shape is ChainSegment chainSegment)
        {
            Transform transform = bodySim.transform.ToRelativeTransform(continuousContext.base_);
            Vector2 p1 = transform.TransformPoint(chainSegment.segment.point1);
            Vector2 p2 = transform.TransformPoint(chainSegment.segment.point2);
            Vector2 e = p2 - p1;
            e = e.GetLengthAndNormalize(out float length);
            if (length > Box2D.LinearSlop)
            {
                Vector2 c1 = continuousContext.centroid1; float separation1 = Vector2.Cross(c1 - p1, e);
                Vector2 c2 = continuousContext.centroid2; float separation2 = Vector2.Cross(c2 - p1, e); // ???
                float coreDistance = 0.25f * continuousContext.fastBodySim.minExtent;
                if (separation1 < 0 || (separation1 - separation2 < coreDistance && separation2 > coreDistance))
                    return true;
            }
        }
        TOIInput input = new()
        {
            proxyA = shape.MakeDistanceProxy(),
            proxyB = fastShape.MakeDistanceProxy(),
            sweepA = bodySim.MakeRelativeSweep(continuousContext.base_),
            sweepB = continuousContext.sweep,
            maxFraction = continuousContext.fraction
        };
        TOIOutput output = input.TimeOfImpact();
        if (isSensor)
        {
            if (output.fraction <= continuousContext.fraction)
            {
                continuousContext.sensorHits.Add(new() { sensorId = shape.id, visitorId = fastShape.id });
                continuousContext.sensorFractions.Add(output.fraction);
            }
        }
        else
        {
            float hitFraction = continuousContext.fraction;
            bool didHit = false;
            if (0 < output.fraction && output.fraction < continuousContext.fraction)
            { hitFraction = output.fraction; didHit = true; }
            else if (0 == output.fraction)
            {
                Vector2 centroid = fastShape.GetCentroid();
                ShapeExtent extent = fastShape.ComputeExtent(centroid);
                float radius = 0.25f * extent.minExtent;
                input.proxyB = Distance.MakeProxy([centroid], radius);
                output = input.TimeOfImpact();
                if (0 < output.fraction && output.fraction < continuousContext.fraction)
                { hitFraction = output.fraction; didHit = true; }
            }
            if (didHit && (shape.enablePreSolveEvents || fastShape.enablePreSolveEvents) && world.preSolveFcn != null)
                didHit = world.preSolveFcn(new() { index1 = shape.id + 1, world0 = world, generation = shape.generation },
                    new() { index1 = fastShape.id + 1, world0 = world, generation = fastShape.generation },
                    continuousContext.base_ + output.point, output.normal, world.preSolveContext);
            if (didHit)
            {
                continuousContext.fastBodySim.flags |= BodyFlags.HadTimeOfImpact;
                continuousContext.fraction = hitFraction;
            }
        }
        return true;
    }
    public void SolveContinuous(int bodySimIndex, TaskContext taskContext)
    {
        SolverSet awakeSet = solverSets[(int)SetType.Awake];
        BodySim fastBodySim = awakeSet.bodySims[bodySimIndex];
        Debug.Assert(fastBodySim.flags.HasFlag(BodyFlags.IsFast));
        Position base_ = fastBodySim.center0;
        Sweep sweep = fastBodySim.MakeRelativeSweep(base_);
        Transform xf1 = new(sweep.c1 - sweep.q1 * sweep.localCenter, sweep.q1);
        Transform xf2 = new(sweep.c2 - sweep.q2 * sweep.localCenter, sweep.q2);
        DynamicTree staticTree = broadPhase.trees[(int)BodyType.Static];
        DynamicTree kinematicTree = broadPhase.trees[(int)BodyType.Kinematic];
        DynamicTree dynamicTree = broadPhase.trees[(int)BodyType.Dynamic];
        Body fastBody = bodies[fastBodySim.bodyId];
        ContinuousContext context = new() { world = this, sweep = sweep, base_ = base_, fastBodySim = fastBodySim, fraction = 1 };
        bool isBullet = fastBodySim.flags.HasFlag(BodyFlags.IsBullet);
        int shapeId = fastBody.headShapeId;
        while (shapeId != -1)
        {
            Shape fastShape = shapes[shapeId];
            shapeId = fastShape.nextShapeId;
            context.fastShape = fastShape;
            context.centroid1 = xf1.TransformPoint(fastShape.localCentroid);
            context.centroid2 = xf2.TransformPoint(fastShape.localCentroid);
            AABB box1 = fastShape.aabb;
            AABB box2 = fastShape.ComputeAABB(xf2).Offset(base_);
            fastShape.aabb = box2;
            if (fastShape.sensorIndex != -1) continue;
            staticTree.Query(AABB.Union(box1, box2), Box2D.DEFAULT_MASK_BITS, ContinuousQueryCallback, context);
            if (isBullet)
            {
                kinematicTree.Query(AABB.Union(box1, box2), Box2D.DEFAULT_MASK_BITS, ContinuousQueryCallback, context);
                dynamicTree.Query(AABB.Union(box1, box2), Box2D.DEFAULT_MASK_BITS, ContinuousQueryCallback, context);
            }
        }
        if (context.fraction < 1)
        {
            Rotation q = Rotation.NLerp(sweep.q1, sweep.q2, context.fraction);
            Vector2 c = Vector2.Lerp(sweep.c1, sweep.c2, context.fraction);
            Vector2 origin = c - q * sweep.localCenter;
            Transform transform = new(origin, q);
            fastBodySim.transform.q = q;
            fastBodySim.transform.p = base_ + origin;
            fastBodySim.center = base_ + c;
            fastBodySim.rotation0 = q;
            fastBodySim.center0 = fastBodySim.center;
            ref BodyMoveEvent event_ = ref System.Runtime.InteropServices.CollectionsMarshal.AsSpan(bodyMoveEvents)[bodySimIndex];
            event_.transform = fastBodySim.transform;
            shapeId = fastBody.headShapeId;
            while (shapeId != -1)
            {
                Shape shape = shapes[shapeId];
                AABB aabb = shape.ComputeFatAABB(fastBodySim.transform, Box2D.SpeculativeDistance);
                if (!shape.fatAABB.Contains(aabb))
                {
                    float margin = shape.aabbMargin;
                    shape.fatAABB = new(new(aabb.lowerBound.x - margin, aabb.lowerBound.y - margin),
                        new(aabb.upperBound.x + margin, aabb.upperBound.y + margin));
                    shape.enlargedAABB = true;
                    fastBodySim.flags |= BodyFlags.EnlargeBounds;
                }
                shapeId = shape.nextShapeId;
            }
        }
        else
        {
            fastBodySim.rotation0 = fastBodySim.transform.q;
            fastBodySim.center0 = fastBodySim.center;
            shapeId = fastBody.headShapeId;
            while (shapeId != -1)
            {
                Shape shape = shapes[shapeId];
                if (!shape.fatAABB.Contains(shape.aabb))
                {
                    float margin = shape.aabbMargin;
                    shape.fatAABB = new(new(shape.aabb.lowerBound.x - margin, shape.aabb.lowerBound.y - margin),
                        new(shape.aabb.upperBound.x + margin, shape.aabb.upperBound.y + margin));
                    shape.enlargedAABB = true;
                    fastBodySim.flags |= BodyFlags.EnlargeBounds;
                }
                shapeId = shape.nextShapeId;
            }
        }
        for (int i = 0; i < context.sensorHits.Count; i++)
            if (context.sensorFractions[i] < context.fraction)
                taskContext.sensorHits.Add(context.sensorHits[i]);
    }
    public static void FinalizeBodiesTask(int startIndex, int endIndex, int workerIndex, object context)
    {
        StepContext stepContext = (StepContext)context;
        World world = stepContext.world;
        Debug.Assert(endIndex <= world.bodyMoveEvents.Count);
        TaskContext taskContext = world.taskContexts[workerIndex];
        ref BitSet enlargedSimBitSet = ref taskContext.enlargedSimBitSet;
        ref BitSet awakeIslandBitSet = ref taskContext.awakeIslandBitSet;
        for (int simIndex = startIndex; simIndex < endIndex; simIndex++)
        {
            BodyState* state = stepContext.states.Data + simIndex;
            BodySim sim = stepContext.sims[simIndex];
            Vector2 v = state->linearVelocity;
            float w = state->angularVelocity;
            Debug.Assert(v.IsValid() && float.IsFinite(w));
            Debug.Assert(float.IsFinite(w));
            sim.center += state->deltaPosition;
            sim.transform.q = (state->deltaRotation * sim.transform.q).Normalize();
            float maxVelocity = v.Length() + Math.Abs(w) * sim.maxExtent;
            float maxDeltaPosition = state->deltaPosition.Length() + Math.Abs(state->deltaRotation.s) * sim.maxExtent;
            float sleepVelocity = Math.Max(maxVelocity, 0.5f * stepContext.inv_dt * maxDeltaPosition);
            state->deltaPosition = Vector2.Zero;
            state->deltaRotation = Rotation.Identity;
            sim.transform.p = sim.center - sim.transform.q * sim.localCenter;
            Body body = world.bodies[sim.bodyId];
            body.bodyMoveIndex = simIndex;
            world.bodyMoveEvents[simIndex] = new()
            {
                transform = sim.transform,
                bodyId = new() { index1 = sim.bodyId + 1, world0 = world, generation = body.generation },
                userData = body.userData,
                fellAsleep = false
            };
            sim.force = Vector2.Zero;
            sim.torque = 0;
            Debug.Assert(!body.flags.HasFlag(BodyFlags.DirtyMass));
            body.flags &= ~BodyFlags.TransientFlags;
            body.flags |= sim.flags & (BodyFlags.IsSpeedCapped | BodyFlags.HadTimeOfImpact);
            body.flags |= state->flags & (BodyFlags.IsSpeedCapped | BodyFlags.HadTimeOfImpact);
            sim.flags &= ~BodyFlags.TransientFlags;
            state->flags &= ~BodyFlags.TransientFlags;
            if (!world.enableSleep || !body.flags.HasFlag(BodyFlags.EnableSleep) || sleepVelocity > body.sleepThreshold)
            {
                body.sleepTime = 0;
                if (body.type == BodyType.Dynamic && world.enableContinuous && Math.Max(maxDeltaPosition, maxVelocity * stepContext.dt) > 0.5f * sim.minExtent)
                {
                    sim.flags |= BodyFlags.IsFast;
                    if (sim.flags.HasFlag(BodyFlags.IsBullet))
                        stepContext.bulletBodies[Interlocked.Increment(ref stepContext.bulletBodyCount) - 1] = simIndex;
                    else world.SolveContinuous(simIndex, taskContext);
                }
                else
                {
                    sim.center0 = sim.center;
                    sim.rotation0 = sim.transform.q;
                }
            }
            else
            {
                sim.center0 = sim.center;
                sim.rotation0 = sim.transform.q;
                body.sleepTime += stepContext.dt;
            }
            Island island = world.islands[body.islandId];
            if (body.sleepTime < Box2D.TimeToSleep) awakeIslandBitSet.SetBit(island.localIndex);
            else if (island.constraintRemoveCount > 0)
            {
                if (body.sleepTime > taskContext.splitSleepTime)
                {
                    taskContext.splitIslandId = body.islandId;
                    taskContext.splitSleepTime = body.sleepTime;
                }
            }
            WorldTransform transform = sim.transform;
            bool isFast = sim.flags.HasFlag(BodyFlags.IsFast);
            int shapeId = body.headShapeId;
            while (shapeId != -1)
            {
                Shape shape = world.shapes[shapeId];
                if (isFast) enlargedSimBitSet.SetBit(simIndex);
                else
                {
                    AABB aabb = shape.ComputeFatAABB(transform, Box2D.SpeculativeDistance);
                    shape.aabb = aabb;
                    Debug.Assert(!shape.enlargedAABB);
                    if (!shape.fatAABB.Contains(aabb))
                    {
                        float margin = shape.aabbMargin;
                        shape.fatAABB = new(new(aabb.lowerBound.x - margin, aabb.lowerBound.y - margin),
                            new(aabb.upperBound.x + margin, aabb.upperBound.y + margin));
                        shape.enlargedAABB = true;
                        enlargedSimBitSet.SetBit(simIndex);
                    }
                }
                shapeId = shape.nextShapeId;
            }
        }
    }
    public struct BlockDim
    {
        /// <summary>number of items per block (except last block)</summary>
        public int size;
        /// <summary>total number of blocks</summary>
        public int count;
    }
    /// <summary>A block is a range of tasks, a start index and count as a sub-array. Each worker receives at
    /// most M blocks of work. The workers may receive less blocks if there is not sufficient work.
    /// Each block of work has a minimum number of elements (block size). This in turn may limit the
    /// number of blocks. If there are many elements then the block size is increased so there are
    /// still at most M blocks of work per worker. M is a tunable number that has two goals:
    /// 1. keep M small to reduce overhead
    /// 2. keep M large enough for other workers to be able to steal work
    /// The block size is a power of two to make math efficient.</summary>
    public static BlockDim ComputeBlockCount(int itemCount, int minSize, int maxBlockCount)
    {
        BlockDim dim = new();
        if (itemCount == 0) return dim;
        dim.size = itemCount <= minSize * maxBlockCount ? minSize : (itemCount + maxBlockCount - 1) / maxBlockCount;
        dim.count = (itemCount + dim.size - 1) / dim.size;
        Debug.Assert(dim.count >= 1);
        Debug.Assert(dim.size * dim.count >= itemCount);
        return dim;
    }
    /// <summary>Initialize solver blocks for a contiguous range of items. Computes block size internally
    /// from the same parameters used by b2ComputeBlockCount.</summary>
    public static void InitBlocks(SyncBlock* blocks, BlockDim dim, int itemCount, SolverBlockType blockType, byte colorIndex)
    {
        if (dim.count == 0) return;
        Debug.Assert(itemCount >= dim.count);
        int blockSize = dim.size;
        Debug.Assert(blockSize <= short.MaxValue);
        for (int i = 0; i < dim.count; i++)
        {
            blocks[i].block.startIndex = i * blockSize;
            blocks[i].block.count = (ushort)blockSize;
            blocks[i].block.blockType = blockType;
            blocks[i].block.colorIndex = colorIndex;
            Interlocked.Exchange(ref blocks[i].syncIndex, 0);
        }
        blocks[dim.count - 1].block.count = (ushort)(itemCount - (dim.count - 1) * blockSize);
    }
    public static void InitStage(SolverStage[] stages, ref int stageIndex, SolverStageType type, SyncBlock* blocks, int blockCount, byte colorIndex)
    {
        stages[stageIndex] = new() { type = type, blocks = blocks, blockCount = blockCount, colorIndex = colorIndex };
        Interlocked.Exchange(ref stages[stageIndex].completionCount, 0);
        stageIndex++;
    }
    /// <summary>Initialize one stage per color for each iteration. Used for warm start, solve, relax, and restitution.
    /// All iterations of a given color share the same b2SyncBlock array so the per-block syncIndex
    /// grows monotonically across stages within that color.</summary>
    public static void InitColorStages(SolverStage[] stages, ref int stageIndex, SolverStageType type, int iterations, int activeColorCount, SyncBlock*[] colorBlocks, int[] colorBlockCounts, int[] activeColorIndices)
    {
        for (int j = 0; j < iterations; j++) for (int i = 0; i < activeColorCount; i++)
            InitStage(stages, ref stageIndex, type, colorBlocks[i], colorBlockCounts[i], (byte)activeColorIndices[i]);
    }
    public static void ExecuteBlock(SolverStage stage, StepContext context, ref SolverBlock block, int workerIndex)
    {
        IContactSolverW contactSolver = IContactSolverW.Instance();
        switch (stage.type)
        {
            case SolverStageType.PrepareJoints:
                PrepareJointsTask(ref block, context);
                break;
            case SolverStageType.PrepareContacts:
                contactSolver.PrepareContactsTask(ref block, context);
                break;
            case SolverStageType.IntegrateVelocities:
                IntegrateVelocitiesTask(ref block, context);
                break;
            case SolverStageType.WarmStart:
                if (block.blockType == SolverBlockType.GraphContact) contactSolver.WarmStartContactsTask(ref block, context);
                else if (block.blockType == SolverBlockType.GraphJoint) WarmStartJointsTask(ref block, context, stage.colorIndex);
                break;
            case SolverStageType.Solve:
                if (block.blockType == SolverBlockType.GraphContact) contactSolver.SolveContactsTask(ref block, context, true);
                else if (block.blockType == SolverBlockType.GraphJoint) SolveJointsTask(ref block, context, stage.colorIndex, true, workerIndex);
                break;
            case SolverStageType.IntegratePositions:
                IntegratePositionsTask(ref block, context);
                break;
            case SolverStageType.Relax:
                if (block.blockType == SolverBlockType.GraphContact) contactSolver.SolveContactsTask(ref block, context, false);
                else if (block.blockType == SolverBlockType.GraphJoint) SolveJointsTask(ref block, context, stage.colorIndex, false, workerIndex);
                break;
            case SolverStageType.Restitution:
                if (block.blockType == SolverBlockType.GraphContact) contactSolver.ApplyRestitutionTask(ref block, context);
                break;
            case SolverStageType.StoreImpulses:
                contactSolver.StoreImpulsesTask(ref block, context, workerIndex);
                break;
            default:
                break;
        }
    }
    /// <summary>This staggers the worker start indices so they avoid touching the same solver blocks</summary>
    public static int GetWorkerStartIndex(int workerIndex, int blockCount, int workerCount)
    {
        if (blockCount <= workerCount) return workerIndex < blockCount ? workerIndex : -1;
        int blocksPerWorker = blockCount / workerCount, remainder = blockCount - blocksPerWorker * workerCount;
        return blocksPerWorker * workerIndex + Math.Min(remainder, workerIndex);
    }
    /// <summary>Execute a stage, which is an array of solver blocks, each controlled with an atomic sync index.
    /// Each worker starts at its home index and sweeps the ring, CAS-claiming any unclaimed blocks.</summary>
    public static void ExecuteStage(SolverStage stage, StepContext context, int previousSyncIndex, int syncIndex, int workerIndex)
    {
        int completedCount = 0, blockCount = stage.blockCount;
        int startIndex = GetWorkerStartIndex(workerIndex, blockCount, context.workerCount);
        if (startIndex == -1) return;
        Debug.Assert(0 <= startIndex && startIndex < blockCount);
        int blockIndex = startIndex;
        for (int i = 0; i < blockCount; i++)
        {
            if (Interlocked.CompareExchange(ref stage.blocks[blockIndex].syncIndex, syncIndex, previousSyncIndex) == previousSyncIndex)
            {
                Debug.Assert(stage.type != SolverStageType.PrepareContacts || syncIndex < 2);
                Debug.Assert(completedCount < blockCount);
                ExecuteBlock(stage, context, ref stage.blocks[blockIndex].block, workerIndex);
                completedCount++;
            }
            blockIndex++;
            if (blockIndex >= blockCount) blockIndex = 0;
        }
        Interlocked.Add(ref stage.completionCount, completedCount);
    }
    public static void ExecuteMainStage(SolverStage stage, StepContext context, uint syncBits)
    {
        int blockCount = stage.blockCount;
        if (blockCount == 0) return;
        int workerIndex = 0;
        if (blockCount == 1) ExecuteBlock(stage, context, ref stage.blocks[0].block, workerIndex);
        else
        {
            Interlocked.Exchange(ref context.atomicSyncBits, syncBits);
            int syncIndex = (int)((syncBits >> 16) & 0xFFFF);
            Debug.Assert(syncIndex > 0);
            int previousSyncIndex = syncIndex - 1;
            ExecuteStage(stage, context, previousSyncIndex, syncIndex, workerIndex);
            while (Interlocked.Add(ref stage.completionCount, 0) != blockCount)
                X86Base.Pause();
            Interlocked.Exchange(ref stage.completionCount, 0);
        }
    }
    public static void SolverTask(object taskContext)
    {
        WorkerContext workerContext = (WorkerContext)taskContext;
        StepContext context = workerContext.context;
        SolverStage[] stages = context.stages;
        Profile profile = context.world.profile;
        const int ITERATIONS = 1, RELAX_ITERATIONS = 1;
        if (workerContext.workerIndex == 0)
        {
            if (Interlocked.CompareExchange(ref context.mainClaimed, 1, 0) != 0) return;
            Stopwatch ticks = new(); ticks.Start();
            int bodySyncIndex = 1, stageIndex = 0;
            uint jointSyncIndex = 1, syncBits = (jointSyncIndex << 16) | (uint)stageIndex;
            Debug.Assert(stages[stageIndex].type == SolverStageType.PrepareJoints);
            ExecuteMainStage(stages[stageIndex], context, syncBits);
            stageIndex++; jointSyncIndex++;
            uint contactSyncIndex = 1;
            syncBits = (contactSyncIndex << 16) | (uint)stageIndex;
            Debug.Assert(stages[stageIndex].type == SolverStageType.PrepareContacts);
            ExecuteMainStage(stages[stageIndex], context, syncBits);
            stageIndex++; contactSyncIndex++;
            context.PrepareJoints_Overflow();
            context.PrepareContacts_Overflow();
            profile.prepareConstraints += (float)ticks.Elapsed.TotalMilliseconds;
            int graphSyncIndex = 1;
            ticks.Restart();
            for (int subStepIndex = 0; subStepIndex < context.subStepCount; subStepIndex++)
            {
                int iterationStageIndex = stageIndex;
                syncBits = ((uint)bodySyncIndex << 16) | (uint)iterationStageIndex;
                Debug.Assert(stages[iterationStageIndex].type == SolverStageType.IntegrateVelocities);
                ExecuteMainStage(stages[iterationStageIndex], context, syncBits);
                iterationStageIndex++; bodySyncIndex++;
                profile.integrateVelocities += (float)ticks.Elapsed.TotalMilliseconds;
                ticks.Restart();
                context.WarmStartJoints_Overflow();
                context.WarmStartContacts_Overflow();
                for (int colorIndex = 0; colorIndex < context.activeColorCount; colorIndex++)
                {
                    syncBits = ((uint)graphSyncIndex << 16) | (uint)iterationStageIndex;
                    Debug.Assert(stages[iterationStageIndex].type == SolverStageType.WarmStart);
                    ExecuteMainStage(stages[iterationStageIndex], context, syncBits);
                    iterationStageIndex++;
                }
                graphSyncIndex++;
                profile.warmStart += (float)ticks.Elapsed.TotalMilliseconds;
                ticks.Restart();
                bool useBias = true;
                for (int j = 0; j < ITERATIONS; j++)
                {
                    context.SolveJoints_Overflow(useBias);
                    context.SolveContacts_Overflow(useBias);
                    for (int colorIndex = 0; colorIndex < context.activeColorCount; colorIndex++)
                    {
                        syncBits = ((uint)graphSyncIndex << 16) | (uint)iterationStageIndex;
                        Debug.Assert(stages[iterationStageIndex].type == SolverStageType.Solve);
                        ExecuteMainStage(stages[iterationStageIndex], context, syncBits);
                        iterationStageIndex++;
                    }
                    graphSyncIndex++;
                }
                profile.solveImpulses += (float)ticks.Elapsed.TotalMilliseconds;
                ticks.Restart();
                Debug.Assert(stages[iterationStageIndex].type == SolverStageType.IntegratePositions);
                syncBits = ((uint)bodySyncIndex << 16) | (uint)iterationStageIndex;
                ExecuteMainStage(stages[iterationStageIndex], context, syncBits);
                iterationStageIndex++; bodySyncIndex++;
                profile.integratePositions += (float)ticks.Elapsed.TotalMilliseconds;
                ticks.Restart();
                for (int j = 0; j < RELAX_ITERATIONS; j++)
                {
                    context.SolveJoints_Overflow(useBias);
                    context.SolveContacts_Overflow(useBias);
                    for (int colorIndex = 0; colorIndex < context.activeColorCount; colorIndex++)
                    {
                        syncBits = ((uint)graphSyncIndex << 16) | (uint)iterationStageIndex;
                        Debug.Assert(stages[iterationStageIndex].type == SolverStageType.Relax);
                        ExecuteMainStage(stages[iterationStageIndex], context, syncBits);
                        iterationStageIndex++;
                    }
                    graphSyncIndex++;
                }
                profile.relaxImpulses += (float)ticks.Elapsed.TotalMilliseconds;
                ticks.Restart();
            }
            stageIndex += 1 + context.activeColorCount + ITERATIONS * context.activeColorCount + 1 + RELAX_ITERATIONS * context.activeColorCount;
            {
                context.ApplyRestitution_Overflow();
                int iterStageIndex = stageIndex;
                for (int colorIndex = 0; colorIndex < context.activeColorCount; colorIndex++)
                {
                    syncBits = ((uint)graphSyncIndex << 16) | (uint)iterStageIndex;
                    Debug.Assert(stages[iterStageIndex].type == SolverStageType.Restitution);
                    ExecuteMainStage(stages[iterStageIndex], context, syncBits);
                    iterStageIndex++;
                }
                stageIndex += context.activeColorCount;
            }
            profile.applyRestitution += (float)ticks.Elapsed.TotalMilliseconds;
            ticks.Restart();
            context.StoreImpulses_Overflow();
            syncBits = (contactSyncIndex << 16) | (uint)stageIndex;
            Debug.Assert(stages[stageIndex].type == SolverStageType.StoreImpulses);
            ExecuteMainStage(stages[stageIndex], context, syncBits);
            profile.storeImpulses += (float)ticks.Elapsed.TotalMilliseconds;
            ticks.Stop();
            Interlocked.Exchange(ref context.atomicSyncBits, uint.MaxValue);
            Debug.Assert(stageIndex + 1 == context.stages.Length);
            return;
        }
        uint lastSyncBits = 0;
        while (true)
        {
            uint syncBits;
            int spinCount = 0;
            while ((syncBits = Interlocked.Add(ref context.atomicSyncBits, 0)) == lastSyncBits)
            {
                if (spinCount < 5) { Thread.Sleep(0); spinCount = 0; }
                else { X86Base.Pause(); X86Base.Pause(); spinCount++; }
            }
            if (syncBits == uint.MaxValue) break;
            int stageIndex = (int)(syncBits & 0xFFFF);
            Debug.Assert(stageIndex < context.stages.Length);
            int syncIndex = (int)((syncBits >> 16) & 0xFFFF);
            Debug.Assert(syncIndex > 0);
            int previousSyncIndex = syncIndex - 1;
            ExecuteStage(stages[stageIndex], context, previousSyncIndex, syncIndex, workerContext.workerIndex);
            lastSyncBits = syncBits;
        }
    }
    public static void BulletBodyTask(int startIndex, int endIndex, int workerIndex, object context)
    {
        StepContext stepContext = (StepContext)context;
        TaskContext taskContext = stepContext.world.taskContexts[workerIndex];
        Debug.Assert(startIndex <= endIndex);
        for (int i = startIndex; i < endIndex; i++)
        {
            int simIndex = stepContext.bulletBodies[i];
            stepContext.world.SolveContinuous(simIndex, taskContext);
        }
    }

    int[] solve_activeColorIndices = new int[Box2D.GraphColorCount],
        solve_colorContactCounts = new int[Box2D.GraphColorCount],
        solve_colorJointCounts = new int[Box2D.GraphColorCount];
    BlockDim[] solve_graphContactDims = new BlockDim[Box2D.GraphColorCount],
        solve_graphJointDims = new BlockDim[Box2D.GraphColorCount];
    ContactPrepareSpan[] solve_contactPrepareSpans = new ContactPrepareSpan[Box2D.GraphColorCount + 1];
    JointPrepareSpan[] solve_jointPrepareSpans = new JointPrepareSpan[Box2D.GraphColorCount + 1];
    SyncBlock*[] solve_graphColorBlocks = new SyncBlock*[Box2D.GraphColorCount];
    int[] solve_graphBlockCounts = new int[Box2D.GraphColorCount];
    WorkerContext[] solve_workerContext = new WorkerContext[Box2D.MaxWorkers];
    public void Solve(StepContext stepContext)
    {
        int SIMD_SHIFT = Avx.IsSupported ? 3 : AdvSimd.IsSupported ? 2 : Sse.IsSupported ? 2 : 0;
        int SIMD_WIDTH = Avx.IsSupported ? 8 : AdvSimd.IsSupported ? 4 : Sse.IsSupported ? 4 : 1;
        const int ITERATIONS = 1, RELAX_ITERATIONS = 1;
        stepIndex++;
        SolverSet awakeSet = solverSets[(int)SetType.Awake];
        int awakeBodyCount = awakeSet.bodySims.Count;
        if (awakeBodyCount == 0)
        {
            if (userTreeTask != null)
            {
                finishTaskFcn(userTreeTask, userTaskContext);
                userTreeTask = null;
                activeTaskCount--;
            }
            broadPhase.ValidateNoEnlarged();
            return;
        }
        {
            Stopwatch setupTicks = new(); setupTicks.Start();
            Interlocked.Exchange(ref stepContext.bulletBodyCount, 0);
            stepContext.bulletBodies = new int[awakeBodyCount];
            stepContext.sims = awakeSet.bodySims;
            stepContext.states = awakeSet.bodyStates;
            int activeColorCount = 0;
            for (int i = 0; i < Box2D.GraphColorCount - 1; i++)
            {
                int perColorContactCount = constraintGraph.colors[i].contactSims.Count;
                int perColorJointCount = constraintGraph.colors[i].jointSims.Count;
                activeColorCount += perColorContactCount + perColorJointCount > 0 ? 1 : 0;
            }
            for (int i = bodyMoveEvents.Count; i < awakeBodyCount; i++) bodyMoveEvents.Add(new());
            int maxBlockCount = 4 * workerCount;
            int minBodiesPerBlock = 32;
            BlockDim bodyDim = ComputeBlockCount(awakeBodyCount, minBodiesPerBlock, maxBlockCount);
            int minContactsPerBlock = 4, minJointsPerBlock = 4;
            //int[] activeColorIndices = new int[Box2D.GraphColorCount],
            //    colorContactCounts = new int[Box2D.GraphColorCount],
            //    colorJointCounts = new int[Box2D.GraphColorCount];
            //BlockDim[] graphContactDims = new BlockDim[Box2D.GraphColorCount],
            //    graphJointDims = new BlockDim[Box2D.GraphColorCount];
            int graphBlockCount = 0;
            int wideContactCount = 0;
            int jointCount = 0;
            int c = 0;
            for (int i = 0; i < Box2D.GraphColorCount - 1; i++)
            {
                int colorContactCount = constraintGraph.colors[i].contactSims.Count;
                int colorJointCount = constraintGraph.colors[i].jointSims.Count;
                if (colorContactCount + colorJointCount == 0) continue;
                solve_activeColorIndices[c] = i;
                int colorContactCountW = colorContactCount > 0 ? ((colorContactCount - 1) >> SIMD_SHIFT) + 1 : 0;
                wideContactCount += colorContactCountW;
                solve_colorContactCounts[c] = colorContactCountW;
                solve_colorJointCounts[c] = colorJointCount;
                jointCount += colorJointCount;
                solve_graphContactDims[c] = ComputeBlockCount(colorContactCountW, minContactsPerBlock, maxBlockCount);
                solve_graphJointDims[c] = ComputeBlockCount(colorJointCount, minJointsPerBlock, maxBlockCount);
                graphBlockCount += solve_graphContactDims[c].count + solve_graphJointDims[c].count;
                c++;
            }
            activeColorCount = c;
            BlockDim contactPrepareDim = ComputeBlockCount(wideContactCount, minContactsPerBlock, maxBlockCount);
            BlockDim jointPrepareDim = ComputeBlockCount(jointCount, minJointsPerBlock, maxBlockCount);
            IContactConstraintsSIMD wideContactConstraints = IContactConstraintsSIMD.Alloc(wideContactCount);
            GraphColor overflow = constraintGraph.colors[Box2D.GraphColorCount - 1];
            int overflowCount = overflow.contactSims.Count;
            //var contactPrepareSpans = new ContactPrepareSpan[Box2D.GraphColorCount + 1];
            //var jointPrepareSpans = new JointPrepareSpan[Box2D.GraphColorCount + 1];
            {
                int wideBase = 0;
                int jointBase = 0;
                for (int i = 0; i < activeColorCount; i++)
                {
                    int j = solve_activeColorIndices[i];
                    GraphColor color = constraintGraph.colors[j];
                    int colorContactCount = color.contactSims.Count;
                    solve_contactPrepareSpans[i].start = wideBase;
                    solve_contactPrepareSpans[i].count = colorContactCount;
                    solve_contactPrepareSpans[i].contacts = color.contactSims;
                    if (colorContactCount == 0)
                    {
                        color.wideConstraints = null;
                        color.wideConstraintCount = 0;
                    }
                    else
                    {
                        color.wideConstraints = wideContactConstraints.PointTo(wideBase);
                        int colorContactCountW = ((colorContactCount - 1) >> SIMD_SHIFT) + 1;
                        color.wideConstraintCount = colorContactCountW;
                        if ((colorContactCount & (SIMD_WIDTH - 1)) != 0) //wipes full lane, even necessary data
                            color.wideConstraints.Clear(colorContactCountW - 1, 1);
                        wideBase += colorContactCountW;
                    }
                    solve_jointPrepareSpans[i].start = jointBase;
                    solve_jointPrepareSpans[i].count = color.jointSims.Count;
                    solve_jointPrepareSpans[i].joints = color.jointSims;
                    jointBase += color.jointSims.Count;
                }
                solve_contactPrepareSpans[activeColorCount].start = wideContactCount;
                solve_contactPrepareSpans[activeColorCount].count = 0;
                solve_contactPrepareSpans[activeColorCount].contacts = null;
                Debug.Assert(wideBase == wideContactCount);
                solve_jointPrepareSpans[activeColorCount].start = jointCount;
                solve_jointPrepareSpans[activeColorCount].count = 0;
                solve_jointPrepareSpans[activeColorCount].joints = null;
                Debug.Assert(jointBase == jointCount);
            }
            int stageCount = 1; // b2_stagePrepareJoints
            stageCount += 1; // b2_stagePrepareContacts
            stageCount += 1; // b2_stageIntegrateVelocities
            stageCount += activeColorCount; // b2_stageWarmStart
            stageCount += ITERATIONS * activeColorCount; // b2_stageSolve
            stageCount += 1; // b2_stageIntegratePositions
            stageCount += RELAX_ITERATIONS * activeColorCount;  // b2_stageRelax
            stageCount += activeColorCount; // b2_stageRestitution
            stageCount += 1; // b2_stageStoreImpulses
            SolverStage[] stages = new SolverStage[stageCount];
            SyncBlock* bodyBlocks = (SyncBlock*)stack.Alloc(bodyDim.count * sizeof(SyncBlock), "body blocks"),
                contactBlocks = (SyncBlock*)stack.Alloc(contactPrepareDim.count * sizeof(SyncBlock), "contact blocks"),
                jointBlocks = (SyncBlock*)stack.Alloc(jointPrepareDim.count * sizeof(SyncBlock), "joint blocks"),
                graphBlocks = (SyncBlock*)stack.Alloc(graphBlockCount * sizeof(SyncBlock), "graph blocks");
            object splitIslandTask = null;
            if (splitIslandId != -1)
            {
                if (taskCount < Box2D.MaxTasks)
                {
                    splitIslandTask = enqueueTaskFcn(SplitIslandTask, this, userTaskContext);
                    taskCount++;
                    activeTaskCount += splitIslandTask == null ? 0 : 1;
                }
                else SplitIslandTask(this);
            }
            InitBlocks(bodyBlocks, bodyDim, awakeBodyCount, SolverBlockType.Body, byte.MaxValue);
            InitBlocks(contactBlocks, contactPrepareDim, wideContactCount, SolverBlockType.Contact, byte.MaxValue);
            InitBlocks(jointBlocks, jointPrepareDim, jointCount, SolverBlockType.Joint, byte.MaxValue);
            //SyncBlock*[] graphColorBlocks = new SyncBlock*[Box2D.GraphColorCount];
            SyncBlock* baseGraphBlock = graphBlocks;
            //int[] graphBlockCounts = new int[Box2D.GraphColorCount];
            for (int i = 0; i < activeColorCount; i++)
            {
                solve_graphColorBlocks[i] = baseGraphBlock;
                byte colorIndex = (byte)solve_activeColorIndices[i];
                InitBlocks(baseGraphBlock, solve_graphJointDims[i], solve_colorJointCounts[i], SolverBlockType.GraphJoint, colorIndex);
                baseGraphBlock += solve_graphJointDims[i].count;
                InitBlocks(baseGraphBlock, solve_graphContactDims[i], solve_colorContactCounts[i], SolverBlockType.GraphContact, colorIndex);
                baseGraphBlock += solve_graphContactDims[i].count;
                solve_graphBlockCounts[i] = solve_graphJointDims[i].count + solve_graphContactDims[i].count;
            }
            Debug.Assert((baseGraphBlock - graphBlocks) == graphBlockCount);
            int stageIndex = 0;
            InitStage(stages, ref stageIndex, SolverStageType.PrepareJoints, jointBlocks, jointPrepareDim.count, byte.MaxValue);
            InitStage(stages, ref stageIndex, SolverStageType.PrepareContacts, contactBlocks, contactPrepareDim.count, byte.MaxValue);
            InitStage(stages, ref stageIndex, SolverStageType.IntegrateVelocities, bodyBlocks, bodyDim.count, byte.MaxValue);
            InitColorStages(stages, ref stageIndex, SolverStageType.WarmStart, 1, activeColorCount, solve_graphColorBlocks, solve_graphBlockCounts, solve_activeColorIndices);
            InitColorStages(stages, ref stageIndex, SolverStageType.Solve, ITERATIONS, activeColorCount, solve_graphColorBlocks, solve_graphBlockCounts, solve_activeColorIndices);
            InitStage(stages, ref stageIndex, SolverStageType.IntegratePositions, bodyBlocks, bodyDim.count, byte.MaxValue);
            InitColorStages(stages, ref stageIndex, SolverStageType.Relax, RELAX_ITERATIONS, activeColorCount, solve_graphColorBlocks, solve_graphBlockCounts, solve_activeColorIndices);
            InitColorStages(stages, ref stageIndex, SolverStageType.Restitution, 1, activeColorCount, solve_graphColorBlocks, solve_graphBlockCounts, solve_activeColorIndices);
            InitStage(stages, ref stageIndex, SolverStageType.StoreImpulses, contactBlocks, contactPrepareDim.count, byte.MaxValue);
            Debug.Assert(stageIndex == stageCount);
            Debug.Assert(workerCount <= Box2D.MaxWorkers);
            //WorkerContext[] workerContext = new WorkerContext[Box2D.MaxWorkers];
            stepContext.graph = constraintGraph;
            stepContext.activeColorCount = activeColorCount;
            stepContext.workerCount = workerCount;
            stepContext.stages = stages;
            stepContext.wideContactConstraints = wideContactConstraints;
            stepContext.contactPrepareSpans = solve_contactPrepareSpans;
            stepContext.wideContactCount = wideContactCount;
            stepContext.jointPrepareSpans = solve_jointPrepareSpans;
            Interlocked.Exchange(ref stepContext.atomicSyncBits, 0);
            profile.solverSetup = (float)setupTicks.Elapsed.TotalMilliseconds;
            setupTicks.Stop();
            Stopwatch constraintTicks = new(); constraintTicks.Start();
            int jointIdCapacity = jointIdPool.GetIdCapacity();
            int contactIdCapacity = contactIdPool.GetIdCapacity();
            for (int i = 0; i < workerCount; i++)
            {
                TaskContext taskContext = taskContexts[i];
                taskContext.jointStateBitSet.SetBitCountAndClear(jointIdCapacity);
                taskContext.hitEventBitSet.SetBitCountAndClear(contactIdCapacity);
                taskContext.hasHitEvents = false;
                solve_workerContext[i] = new()
                {
                    context = stepContext,
                    workerIndex = i
                };
                if (taskCount < Box2D.MaxTasks)
                {
                    solve_workerContext[i].userTask = enqueueTaskFcn(SolverTask, solve_workerContext[i], userTaskContext);
                    taskCount++;
                    activeTaskCount += solve_workerContext[i].userTask == null ? 0 : 1;
                }
                else
                {
                    solve_workerContext[i].userTask = null;
                    SolverTask(solve_workerContext[i]);
                }
            }
            SolverTask(new WorkerContext { context = stepContext });
            for (int i = 0; i < workerCount; i++) if (solve_workerContext[i].userTask != null)
            {
                finishTaskFcn(solve_workerContext[i].userTask, userTaskContext);
                activeTaskCount--;
            }
            if (splitIslandTask != null)
            {
                finishTaskFcn(splitIslandTask, userTaskContext);
                activeTaskCount--;
            }
            splitIslandId = -1;
            profile.constraints = (float)constraintTicks.Elapsed.TotalMilliseconds;
            constraintTicks.Stop();
            Stopwatch transformTicks = new(); transformTicks.Start();
            int awakeIslandCount = awakeSet.islandSims.Count;
            for (int i = 0; i < workerCount; i++)
            {
                TaskContext taskContext = taskContexts[i];
                taskContext.sensorHits.Clear();
                taskContext.enlargedSimBitSet.SetBitCountAndClear(awakeBodyCount);
                taskContext.awakeIslandBitSet.SetBitCountAndClear(awakeIslandCount);
                taskContext.splitIslandId = -1;
                taskContext.splitSleepTime = 0;
            }
            ParallelFor(FinalizeBodiesTask, awakeBodyCount, 64, stepContext);
            stack.Free(graphBlocks);
            stack.Free(jointBlocks);
            stack.Free(contactBlocks);
            stack.Free(bodyBlocks);
            wideContactConstraints.Free();
            profile.transforms = (float)transformTicks.Elapsed.TotalMilliseconds;
        }
        {
            Stopwatch jointEventTicks = new(); jointEventTicks.Start();
            BitSet jointStateBitSet = taskContexts[0].jointStateBitSet;
            for (int i = 1; i < workerCount; i++)
                jointStateBitSet.InPlaceUnion(taskContexts[i].jointStateBitSet);
            for (uint k = 0; k < jointStateBitSet.blockCount; k++)
            {
                ulong word = jointStateBitSet.bits[k];
                while (word != 0)
                {
                    uint ctz = CTZ.CTZ64(word);
                    int jointId = (int)(64 * k + ctz);
                    Debug.Assert(jointId < joints.Count);
                    Joint joint = joints[jointId];
                    Debug.Assert(joint.setIndex == (int)SetType.Awake);
                    jointEvents.Add(new()
                    {
                        jointId = new() { index1 = jointId + 1, world0 = this, generation = joint.generation },
                        userData = joint.userData
                    });
                    word &= word - 1;
                }
            }
            profile.jointEvents = (float)jointEventTicks.Elapsed.TotalMilliseconds;
            jointEventTicks.Stop();
        }
        {
            Stopwatch hitTicks = new(); hitTicks.Start();
            Debug.Assert(contactHitEvents.Count == 0);
            bool anyHitEvents = false;
            for (int i = 0; i < workerCount; i++) if (taskContexts[i].hasHitEvents)
            { anyHitEvents = true; break; }
            if (anyHitEvents)
            {
                BitSet hitEventBitSet = taskContexts[0].hitEventBitSet;
                for (int i = 1; i < workerCount; i++) if (taskContexts[i].hasHitEvents)
                        hitEventBitSet.InPlaceUnion(taskContexts[i].hitEventBitSet);
                for (int k = 0; k < hitEventBitSet.blockCount; k++)
                {
                    ulong word = hitEventBitSet.bits[k];
                    while (word != 0)
                    {
                        uint ctz = CTZ.CTZ64(word);
                        int contactId = (int)(64 * k + ctz);
                        Contact contact = contacts[contactId];
                        Debug.Assert(contact.setIndex == (int)SetType.Awake && contact.colorIndex != -1);
                        GraphColor color = constraintGraph.colors[contact.colorIndex];
                        ContactSim contactSim = constraintGraph.colors[contact.colorIndex].contactSims[contact.localIndex];
                        ContactHitEvent event_ = new() { approachSpeed = hitEventThreshold };
                        int bestPoint = -1;
                        if (contactSim.manifold.pointCount > 0)
                        {
                            float approachSpeed = -contactSim.manifold.point0.normalVelocity;
                            if (approachSpeed > event_.approachSpeed && contactSim.manifold.point0.totalNormalImpulse > 0)
                            {
                                event_.approachSpeed = approachSpeed;
                                bestPoint = 0;
                            }
                        }
                        if (contactSim.manifold.pointCount > 1)
                        {
                            float approachSpeed = -contactSim.manifold.point1.normalVelocity;
                            if (approachSpeed > event_.approachSpeed && contactSim.manifold.point1.totalNormalImpulse > 0)
                            {
                                event_.approachSpeed = approachSpeed;
                                bestPoint = 1;
                            }
                        }
                        if (bestPoint != -1)
                        {
                            event_.normal = contactSim.manifold.normal;
                            Shape shapeA = shapes[contactSim.shapeIdA], shapeB = shapes[contactSim.shapeIdB];
                            Body bodyA = bodies[shapeA.bodyId], bodyB = bodies[shapeB.bodyId];
                            if (bodyA.type != BodyType.Static && bodyB.type == BodyType.Static)
                            {
                                BodySim bodySimB = GetBodySim(bodyB);
                                event_.point = bodySimB.center + (bestPoint == 1 ? contactSim.manifold.point1.anchorB : contactSim.manifold.point0.anchorB);
                            }
                            else
                            {
                                BodySim bodySimA = GetBodySim(bodyA);
                                event_.point = bodySimA.center + (bestPoint == 1 ? contactSim.manifold.point1.anchorA : contactSim.manifold.point0.anchorA);
                            }
                            event_.shapeIdA = new() { index1 = shapeA.id + 1, world0 = this, generation = shapeA.generation };
                            event_.shapeIdB = new() { index1 = shapeB.id + 1, world0 = this, generation = shapeB.generation };
                            event_.contactId = new()
                            {
                                index1 = contact.contactId + 1,
                                world0 = this,
                                generation = contact.generation,
                            };
                            contactHitEvents.Add(event_);
                        }
                    }
                    word &= word - 1;
                }
            }
            profile.hitEvents = (float)hitTicks.Elapsed.TotalMilliseconds;
            hitTicks.Stop();
        }
        {
            Stopwatch refitTicks = new(); refitTicks.Start();
            if (userTreeTask != null)
            {
                finishTaskFcn(userTreeTask, userTaskContext);
                userTreeTask = null;
                activeTaskCount--;
            }
            broadPhase.ValidateNoEnlarged();
            BitSet enlargedBodyBitSet = taskContexts[0].enlargedSimBitSet;
            for (int i = 1; i < workerCount; i++) enlargedBodyBitSet.InPlaceUnion(taskContexts[i].enlargedSimBitSet);
            for (uint k = 0; k < enlargedBodyBitSet.blockCount; k++)
            {
                ulong word = enlargedBodyBitSet.bits[k];
                while (word != 0)
                {
                    uint ctz = CTZ.CTZ64(word);
                    int bodySimIndex = (int)(64 * k + ctz);
                    BodySim bodySim = awakeSet.bodySims[bodySimIndex];
                    Body body = bodies[bodySim.bodyId];
                    int shapeId = body.headShapeId;
                    if ((bodySim.flags & (BodyFlags.IsBullet | BodyFlags.IsFast)) == (BodyFlags.IsBullet | BodyFlags.IsFast))
                    {
                        while (shapeId != -1)
                        {
                            Shape shape = shapes[shapeId];
                            broadPhase.BufferMove(shape.proxyKey);
                            shapeId = shape.nextShapeId;
                        }
                    }
                    else
                    {
                        while (shapeId != -1)
                        {
                            Shape shape = shapes[shapeId];
                            if (shape.enlargedAABB)
                            { broadPhase.EnlargeProxy(shape.proxyKey, shape.fatAABB); shape.enlargedAABB = false; }
                            shapeId = shape.nextShapeId;
                        }
                    }
                    word &= word - 1;
                }
            }
            broadPhase.ValidateBroadphase();
            profile.refit = (float)refitTicks.Elapsed.TotalMilliseconds;
            refitTicks.Stop();
        }
        int bulletBodyCount = Interlocked.Add(ref stepContext.bulletBodyCount, 0);
        if (bulletBodyCount > 0)
        {
            Stopwatch bulletTicks = new(); bulletTicks.Start();
            int minRange = 8;
            ParallelFor(BulletBodyTask, bulletBodyCount, minRange, stepContext);
            DynamicTree dynamicTree = broadPhase.trees[(int)BodyType.Dynamic];
            var bulletBodySimIndices = stepContext.bulletBodies;
            for (int i = 0; i < bulletBodyCount; i++)
            {
                BodySim bulletBodySim = awakeSet.bodySims[bulletBodySimIndices[i]];
                if (!bulletBodySim.flags.HasFlag(BodyFlags.EnlargeBounds)) continue;
                bulletBodySim.flags &= ~BodyFlags.EnlargeBounds;
                int bodyId = bulletBodySim.bodyId;
                Debug.Assert(0 <= bodyId && bodyId < bodies.Count);
                Body bulletBody = bodies[bodyId];
                int shapeId = bulletBody.headShapeId;
                while (shapeId != -1)
                {
                    Shape shape = shapes[shapeId];
                    if (!shape.enlargedAABB) { shapeId = shape.nextShapeId; continue; }
                    shape.enlargedAABB = false;
                    int proxyKey = shape.proxyKey;
                    int proxyId = B2_PROXY_ID(proxyKey);
                    Debug.Assert(B2_PROXY_TYPE(proxyKey) == BodyType.Dynamic);
                    Debug.Assert(broadPhase.movedProxies[(int)BodyType.Dynamic].GetBit(proxyId));
                    dynamicTree.EnlargeProxy(proxyKey, shape.fatAABB);
                    shapeId = shape.nextShapeId;
                }
            }
            profile.bullets = (float)bulletTicks.Elapsed.TotalMilliseconds;
            bulletTicks.Stop();
        }
        stepContext.bulletBodies = null;
        Interlocked.Exchange(ref stepContext.bulletBodyCount, 0);
        {
            Stopwatch sensorHitTicks = new(); sensorHitTicks.Start();
            Debug.Assert(workerCount == taskContexts.Count);
            for (int i = 0; i < workerCount; i++)
            {
                TaskContext taskContext = taskContexts[i];
                int hitCount = taskContext.sensorHits.Count;
                for (int j = 0; j < hitCount; j++)
                {
                    ref SensorHit hit = ref System.Runtime.InteropServices.CollectionsMarshal.AsSpan(taskContext.sensorHits)[j];
                    Shape sensorShape = shapes[hit.sensorId], visitor = shapes[hit.visitorId];
                    sensors[sensorShape.sensorIndex].hits.Add(new()
                    { shapeId = hit.visitorId, generation = visitor.generation });
                }
            }
            profile.bullets = (float)sensorHitTicks.Elapsed.TotalMilliseconds;
            sensorHitTicks.Stop();
        }
        if (enableSleep)
        {
            Stopwatch sleepTicks = new(); sleepTicks.Start();
            Debug.Assert(splitIslandId == -1);
            float splitSleepTimer = 0;
            for (int i = 0; i < workerCount; i++)
            {
                TaskContext taskContext = taskContexts[i];
                if (taskContext.splitIslandId != -1 && taskContext.splitSleepTime >= splitSleepTimer)
                {
                    Debug.Assert(taskContext.splitSleepTime > 0);
                    if (taskContext.splitSleepTime == splitSleepTimer && taskContext.splitIslandId < splitIslandId) continue;
                    splitIslandId = taskContext.splitIslandId;
                    splitSleepTimer = taskContext.splitSleepTime;
                }
            }
            BitSet awakeIslandBitSet = taskContexts[0].awakeIslandBitSet;
            for (int i = 0; i < workerCount; i++) awakeIslandBitSet.InPlaceUnion(taskContexts[i].awakeIslandBitSet);
            int count = awakeSet.islandSims.Count;
            for (int islandIndex = count - 1; islandIndex >= 0; islandIndex--)
            {
                if (awakeIslandBitSet.GetBit(islandIndex)) continue;
                IslandSim island = awakeSet.islandSims[islandIndex];
                TrySleepIsland(island.islandId);
            }
            ValidateSolverSets();
            profile.sleepIslands = (float)sleepTicks.Elapsed.TotalMilliseconds;
            sleepTicks.Stop();
        }
    }
}