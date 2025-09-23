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
public enum SolverBlockType : short
{
    Body, Joint, Contact, GraphJoint, GraphContact
}
/// <summary>Each block of work has a sync index that gets incremented when a worker claims the block. This ensures only a single worker
/// claims a block, yet lets work be distributed dynamically across multiple workers (work stealing). This also reduces contention
/// on a single block index atomic. For non-iterative stages the sync index is simply set to one. For iterative stages (solver
/// iteration) the same block of work is executed once per iteration and the atomic sync index is shared across iterations, so it
/// increases monotonically.</summary>
public struct SolverBlock
{
    public int startIndex;
    public short count;
    public SolverBlockType blockType;
    public int syncIndex;
}
public unsafe class SolverStage
{
    public SolverStageType type;
    public SolverBlock* blocks;
    public int blockCount;
    public int colorIndex;
    public int completionCount;
}
public unsafe partial class StepContext
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

    /// <summary>joint pointers for simplified parallel-for access.</summary>
    public JointSim[] joints;

    /// <summary>contact pointers for simplified parallel-for access.<br/>
    /// - parallel-for collide with no gaps<br/>
    /// - parallel-for prepare and store contacts with NULL gaps for SIMD remainders
    /// despite being an array of pointers, these are contiguous sub-arrays corresponding
    /// to constraint graph colors</summary>
    public ContactSim[] contacts;

    public IContactConstraintsSIMD simdContactConstraints;
    public int activeColorCount;
    public int workerCount;

    public SolverStage[] stages;
    public bool enableWarmStarting;

    /// <summary>sync index (16-bits) | stage type (16-bits)</summary>
    public uint atomicSyncBits;
}
public unsafe partial class World
{
    public class WorkerContext
    {
        public StepContext context;
        public int workerIndex;
        public object userTask;
    }
    public static void IntegrateVelocitiesTask(int startIndex, int endIndex, StepContext context)
    {
        Vector2 gravity = context.world.gravity;
        float h = context.h;
        float maxLinearSpeed = context.maxLinearVelocity;
        float maxAngularSpeed = Box2D.MaxRotation * context.inv_dt;
        float maxLinearSpeedSquared = maxLinearSpeed * maxLinearSpeed;
        float maxAngularSpeedSquared = maxAngularSpeed * maxAngularSpeed;
        for (int i = startIndex; i < endIndex; i++)
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
            if (Vector2.Dot(v, v) > maxLinearSpeedSquared)
            {
                float ratio = maxLinearSpeed / v.Length();
                v = ratio * v;
                sim.flags |= BodyFlags.IsSpeedCapped;
            }
            if (w * w > maxAngularSpeedSquared && !sim.flags.HasFlag(BodyFlags.AllowFastRotation))
            {
                float ratio = maxAngularSpeed / Math.Abs(w);
                w *= ratio;
                sim.flags |= BodyFlags.IsSpeedCapped;
            }
            if (state->flags.HasFlag(BodyFlags.LockLinearX)) v.x = 0;
            if (state->flags.HasFlag(BodyFlags.LockLinearY)) v.y = 0;
            if (state->flags.HasFlag(BodyFlags.LockAngularZ)) w = 0;
            state->linearVelocity = v;
            state->angularVelocity = w;
         }
    }
    public static void PrepareJointsTask(int startIndex, int endIndex, StepContext context)
    {
        for (int i = startIndex; i < endIndex; i++)
        {
            JointSim joint = context.joints[i];
            joint.PrepareJoint(context);
        }
    }
    public static void WarmStartJointsTask(int startIndex, int endIndex, StepContext context, int colorIndex)
    {
        GraphColor color = context.graph.colors[colorIndex];
        Debug.Assert(0 <= startIndex && startIndex < color.jointSims.Count);
        Debug.Assert(startIndex <= endIndex && endIndex <= color.jointSims.Count);
        for (int i = startIndex; i < endIndex; i++)
        {
            JointSim joint = color.jointSims[i];
            joint.WarmStart(context);
        }
    }
    public static void SolveJointsTask(int startIndex, int endIndex, StepContext context, int colorIndex, bool useBias, int workerIndex)
    {
        GraphColor color = context.graph.colors[colorIndex];
        Debug.Assert(0 <= startIndex && startIndex < color.jointSims.Count);
        Debug.Assert(startIndex <= endIndex && endIndex <= color.jointSims.Count);
        BitSet jointStateBitSet = context.world.taskContexts[workerIndex].jointStateBitSet;
        for (int i = startIndex; i < endIndex; i++)
        {
            JointSim joint = color.jointSims[i];
            joint.Solve(context, useBias);
            if (useBias && (joint.forceThreshold < float.MaxValue || joint.torqueThreshold < float.MaxValue)
                && !jointStateBitSet.GetBit(joint.jointId))
            {
                joint.GetJointReaction(context.inv_h, out float force, out float torque);
                if (force >= joint.forceThreshold || torque >= joint.torqueThreshold)
                    jointStateBitSet.SetBit(joint.jointId);
            }
        }
    }
    public static void IntegratePositionsTask(int startIndex, int endIndex, StepContext context)
    {
        float h = context.h;
        Debug.Assert(startIndex <= endIndex);
        for (int i = startIndex; i < endIndex; i++)
        {
            BodyState* state = context.states.Data + i;
            if (state->flags.HasFlag(BodyFlags.LockLinearX)) state->linearVelocity.x = 0;
            if (state->flags.HasFlag(BodyFlags.LockLinearY)) state->linearVelocity.y = 0;
            if (state->flags.HasFlag(BodyFlags.LockAngularZ)) state->angularVelocity = 0;
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
            Transform transform = bodySim.transform;
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
            sweepA = bodySim.MakeSweep(),
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
                    output.point, output.normal, world.preSolveContext);
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
        Sweep sweep = fastBodySim.MakeSweep();
        Transform xf1 = new(sweep.c1 - sweep.q1 * sweep.localCenter, sweep.q1);
        Transform xf2 = new(sweep.c2 - sweep.q2 * sweep.localCenter, sweep.q2);
        DynamicTree staticTree = broadPhase.trees[(int)BodyType.Static];
        DynamicTree kinematicTree = broadPhase.trees[(int)BodyType.Kinematic];
        DynamicTree dynamicTree = broadPhase.trees[(int)BodyType.Dynamic];
        Body fastBody = bodies[fastBodySim.bodyId];
        ContinuousContext context = new() { world = this, sweep = sweep, fastBodySim = fastBodySim, fraction = 1 };
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
            AABB box2 = fastShape.ComputeAABB(xf2);
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
            fastBodySim.transform = transform;
            fastBodySim.center = c;
            fastBodySim.rotation0 = q;
            fastBodySim.center0 = c;
            ref BodyMoveEvent event_ = ref System.Runtime.InteropServices.CollectionsMarshal.AsSpan(bodyMoveEvents)[bodySimIndex];
            event_.transform = transform;
            shapeId = fastBody.headShapeId;
            while (shapeId != -1)
            {
                Shape shape = shapes[shapeId];
                AABB aabb = shape.ComputeAABB(transform);
                aabb.lowerBound.x -= Box2D.SpeculativeDistance;
                aabb.lowerBound.y -= Box2D.SpeculativeDistance;
                aabb.upperBound.x += Box2D.SpeculativeDistance;
                aabb.upperBound.y += Box2D.SpeculativeDistance;
                shape.aabb = aabb;
                if (!shape.fatAABB.Contains(aabb))
                {
                    shape.fatAABB = new(new(aabb.lowerBound.x - Box2D.AABBMargin, aabb.lowerBound.y - Box2D.AABBMargin),
                        new(aabb.upperBound.x + Box2D.AABBMargin, aabb.upperBound.y + Box2D.AABBMargin));
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
                    shape.fatAABB = new(new(shape.aabb.lowerBound.x - Box2D.AABBMargin, shape.aabb.lowerBound.y - Box2D.AABBMargin),
                        new(shape.aabb.upperBound.x + Box2D.AABBMargin, shape.aabb.upperBound.y + Box2D.AABBMargin));
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
    public static void FinalizeBodiesTask(int startIndex, int endIndex, uint threadIndex, object context)
    {
        StepContext stepContext = (StepContext)context;
        World world = stepContext.world;
        Debug.Assert(endIndex <= world.bodyMoveEvents.Count);
        TaskContext taskContext = world.taskContexts[(int)threadIndex];
        ref BitSet enlargedSimBitSet = ref taskContext.enlargedSimBitSet;
        ref BitSet awakeIslandBitSet = ref taskContext.awakeIslandBitSet;
        Debug.Assert(startIndex <= endIndex);
        for (int simIndex = startIndex; simIndex < endIndex; simIndex++)
        {
            BodyState* state = stepContext.states.Data + simIndex;
            BodySim sim = stepContext.sims[simIndex];
            if (state->flags.HasFlag(BodyFlags.LockLinearX)) state->linearVelocity.x = 0;
            if (state->flags.HasFlag(BodyFlags.LockLinearY)) state->linearVelocity.y = 0;
            if (state->flags.HasFlag(BodyFlags.LockAngularZ)) state->angularVelocity = 0;
            Vector2 v = state->linearVelocity;
            float w = state->angularVelocity;
            Debug.Assert(v.IsValid());
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
            body.flags &= ~(BodyFlags.IsFast | BodyFlags.IsSpeedCapped | BodyFlags.HadTimeOfImpact);
            body.flags |= (sim.flags & (BodyFlags.IsSpeedCapped | BodyFlags.HadTimeOfImpact));
            sim.flags &= ~(BodyFlags.IsFast | BodyFlags.IsSpeedCapped | BodyFlags.HadTimeOfImpact);
            if (!world.enableSleep || !body.enableSleep || sleepVelocity > body.sleepThreshold)
            {
                body.sleepTime = 0;
                if (body.type == BodyType.Dynamic && world.enableContinuous && maxVelocity * stepContext.dt > 0.5f * sim.minExtent)
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
            Transform transform = sim.transform;
            bool isFast = sim.flags.HasFlag(BodyFlags.IsFast);
            int shapeId = body.headShapeId;
            while (shapeId != -1)
            {
                Shape shape = world.shapes[shapeId];
                if (isFast) enlargedSimBitSet.SetBit(simIndex);
                else
                {
                    AABB aabb = shape.ComputeAABB(transform);
                    aabb.lowerBound.x -= Box2D.SpeculativeDistance;
                    aabb.lowerBound.y -= Box2D.SpeculativeDistance;
                    aabb.upperBound.x += Box2D.SpeculativeDistance;
                    aabb.upperBound.y += Box2D.SpeculativeDistance;
                    shape.aabb = aabb;
                    Debug.Assert(!shape.enlargedAABB);
                    if (!shape.fatAABB.Contains(aabb))
                    {
                        shape.fatAABB = new(new(aabb.lowerBound.x - Box2D.AABBMargin, aabb.lowerBound.y - Box2D.AABBMargin),
                            new(aabb.upperBound.x + Box2D.AABBMargin, aabb.upperBound.y + Box2D.AABBMargin));
                        shape.enlargedAABB = true;
                        enlargedSimBitSet.SetBit(simIndex);
                    }
                }
                shapeId = shape.nextShapeId;
            }
        }
    }
    public static void ExecuteBlock(SolverStage stage, StepContext context, ref SolverBlock block, int workerIndex)
    {
        int startIndex = block.startIndex, endIndex = startIndex + block.count;
        IContactSolverW contactSolver = IContactSolverW.Instance();
        switch (stage.type)
        {
            case SolverStageType.PrepareJoints:
                PrepareJointsTask(startIndex, endIndex, context);
                break;
            case SolverStageType.PrepareContacts:
                contactSolver.PrepareContactsTask(startIndex, endIndex, context);
                break;
            case SolverStageType.IntegrateVelocities:
                IntegrateVelocitiesTask(startIndex, endIndex, context);
                break;
            case SolverStageType.WarmStart:
                if (block.blockType == SolverBlockType.GraphContact) contactSolver.WarmStartContactsTask(startIndex, endIndex, context, stage.colorIndex);
                else if (block.blockType == SolverBlockType.GraphJoint) WarmStartJointsTask(startIndex, endIndex, context, stage.colorIndex);
                break;
            case SolverStageType.Solve:
                if (block.blockType == SolverBlockType.GraphContact) contactSolver.SolveContactsTask(startIndex, endIndex, context, stage.colorIndex, true);
                else if (block.blockType == SolverBlockType.GraphJoint) SolveJointsTask(startIndex, endIndex, context, stage.colorIndex, true, workerIndex);
                break;
            case SolverStageType.IntegratePositions:
                IntegratePositionsTask(startIndex, endIndex, context);
                break;
            case SolverStageType.Relax:
                if (block.blockType == SolverBlockType.GraphContact) contactSolver.SolveContactsTask(startIndex, endIndex, context, stage.colorIndex, false);
                else if (block.blockType == SolverBlockType.GraphJoint) SolveJointsTask(startIndex, endIndex, context, stage.colorIndex, false, workerIndex);
                break;
            case SolverStageType.Restitution:
                if (block.blockType == SolverBlockType.GraphContact) contactSolver.ApplyRestitutionTask(startIndex, endIndex, context, stage.colorIndex);
                break;
            case SolverStageType.StoreImpulses:
                contactSolver.StoreImpulsesTask(startIndex, endIndex, context);
                break;
            default:
                break;
        }
    }
    public static int GetWorkerStartIndex(int workerIndex, int blockCount, int workerCount)
    {
        if (blockCount <= workerCount) return workerIndex < blockCount ? workerIndex : -1;
        int blocksPerWorker = blockCount / workerCount, remainder = blockCount - blocksPerWorker * workerCount;
        return blocksPerWorker * workerIndex + Math.Min(remainder, workerIndex);
    }
    public static void ExecuteStage(SolverStage stage, StepContext context, int previousSyncIndex, int syncIndex, int workerIndex)
    {
        int completedCount = 0, blockCount = stage.blockCount;
        int expectedSyncIndex = previousSyncIndex;
        int startIndex = GetWorkerStartIndex(workerIndex, blockCount, context.workerCount);
        if (startIndex == -1) return;
        Debug.Assert(0 <= startIndex && startIndex < blockCount);
        int blockIndex = startIndex;
        while (Interlocked.CompareExchange(ref stage.blocks[blockIndex].syncIndex, syncIndex, expectedSyncIndex) == expectedSyncIndex)
        {
            Debug.Assert(stage.type != SolverStageType.PrepareContacts || syncIndex < 2);
            Debug.Assert(completedCount < blockCount);
            ExecuteBlock(stage, context, ref stage.blocks[blockIndex], workerIndex);
            completedCount++;
            blockIndex++;
            if (blockIndex >= blockCount) blockIndex = 0;
            expectedSyncIndex = previousSyncIndex;
        }
        blockIndex = startIndex - 1;
        while (true)
        {
            if (blockIndex < 0) blockIndex = blockCount - 1;
            expectedSyncIndex = previousSyncIndex;
            if (Interlocked.CompareExchange(ref stage.blocks[blockIndex].syncIndex, syncIndex, expectedSyncIndex) != expectedSyncIndex)
                break;
            ExecuteBlock(stage, context, ref stage.blocks[blockIndex], workerIndex);
            completedCount++;
            blockIndex--;
        }
        Interlocked.Add(ref stage.completionCount, completedCount);
    }
    public static void ExecuteMainStage(SolverStage stage, StepContext context, uint syncBits)
    {
        int blockCount = stage.blockCount;
        if (blockCount == 0) return;
        int workerIndex = 0;
        if (blockCount == 1) ExecuteBlock(stage, context, ref stage.blocks[0], workerIndex);
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
    public static void SolverTask(int startIndex, int endIndex, uint threadIndexIgnore, object taskContext)
    {
        WorkerContext workerContext = (WorkerContext)taskContext;
        StepContext context = workerContext.context;
        SolverStage[] stages = context.stages;
        Profile profile = context.world.profile;
        const int ITERATIONS = 1, RELAX_ITERATIONS = 1;
        if (workerContext.workerIndex == 0)
        {
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
            int graphSyncIndex = 1;
            context.PrepareOverflowJoints();
            context.PrepareOverflowContacts();
            profile.prepareConstraints += (float)ticks.Elapsed.TotalMilliseconds;
            ticks.Restart();
            for (int i = 0; i < context.subStepCount; i++)
            {
                int iterStageIndex = stageIndex;
                syncBits = ((uint)bodySyncIndex << 16) | (uint)iterStageIndex;
                Debug.Assert(stages[iterStageIndex].type == SolverStageType.IntegrateVelocities);
                ExecuteMainStage(stages[iterStageIndex], context, syncBits);
                iterStageIndex++; bodySyncIndex++;
                profile.integrateVelocities += (float)ticks.Elapsed.TotalMilliseconds;
                ticks.Restart();
                context.WarmStartOverflowJoints();
                context.WarmStartOverflowContacts();
                for (int colorIndex = 0; colorIndex < context.activeColorCount; colorIndex++)
                {
                    syncBits = ((uint)graphSyncIndex << 16) | (uint)iterStageIndex;
                    Debug.Assert(stages[iterStageIndex].type == SolverStageType.WarmStart);
                    ExecuteMainStage(stages[iterStageIndex], context, syncBits);
                    iterStageIndex++;
                }
                graphSyncIndex++;
                profile.warmStart += (float)ticks.Elapsed.TotalMilliseconds;
                ticks.Restart();
                bool useBias = true;
                for (int j = 0; j < ITERATIONS; j++)
                {
                    context.SolveOverflowJoints(useBias);
                    context.SolveOverflowContacts(useBias);
                    for (int colorIndex = 0; colorIndex < context.activeColorCount; colorIndex++)
                    {
                        syncBits = ((uint)graphSyncIndex << 16) | (uint)iterStageIndex;
                        Debug.Assert(stages[iterStageIndex].type == SolverStageType.Solve);
                        ExecuteMainStage(stages[iterStageIndex], context, syncBits);
                        iterStageIndex++;
                    }
                    graphSyncIndex++;
                }
                profile.solveImpulses += (float)ticks.Elapsed.TotalMilliseconds;
                ticks.Restart();
                Debug.Assert(stages[iterStageIndex].type == SolverStageType.IntegratePositions);
                syncBits = ((uint)bodySyncIndex << 16) | (uint)iterStageIndex;
                ExecuteMainStage(stages[iterStageIndex], context, syncBits);
                iterStageIndex++; bodySyncIndex++;
                profile.integratePositions += (float)ticks.Elapsed.TotalMilliseconds;
                ticks.Restart();
                for (int j = 0; j < RELAX_ITERATIONS; j++)
                {
                    context.SolveOverflowJoints(useBias);
                    context.SolveOverflowContacts(useBias);
                    for (int colorIndex = 0; colorIndex < context.activeColorCount; colorIndex++)
                    {
                        syncBits = ((uint)graphSyncIndex << 16) | (uint)iterStageIndex;
                        Debug.Assert(stages[iterStageIndex].type == SolverStageType.Relax);
                        ExecuteMainStage(stages[iterStageIndex], context, syncBits);
                        iterStageIndex++;
                    }
                    graphSyncIndex++;
                }
                profile.relaxImpulses += (float)ticks.Elapsed.TotalMilliseconds;
                ticks.Restart();
            }
            stageIndex += 1 + context.activeColorCount + ITERATIONS * context.activeColorCount + 1 + RELAX_ITERATIONS * context.activeColorCount;
            {
                context.ApplyOverflowRestitution();
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
            context.StoreOverflowImpulses();
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
    public static void BulletBodyTask(int startIndex, int endIndex, uint threadIndex, object context)
    {
        StepContext stepContext = (StepContext)context;
        TaskContext taskContext = stepContext.world.taskContexts[(int)threadIndex];
        Debug.Assert(startIndex <= endIndex);
        for (int i = startIndex; i < endIndex; i++)
        {
            int simIndex = stepContext.bulletBodies[i];
            stepContext.world.SolveContinuous(simIndex, taskContext);
        }
    }
    public unsafe void Solve(StepContext stepContext)
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
            Interlocked.Exchange(ref stepContext.bulletBodyCount, 0);
            stepContext.bulletBodies = new int[awakeBodyCount];
            Stopwatch prepareTicks = new(); prepareTicks.Start();
            stepContext.sims = awakeSet.bodySims;
            stepContext.states = awakeSet.bodyStates;
            int awakeJointCount = 0, activeColorCount = 0;
            for (int i = 0; i < Box2D.GraphColorCount - 1; i++)
            {
                int perColorContactCount = constraintGraph.colors[i].contactSims.Count;
                int perColorJointCount = constraintGraph.colors[i].jointSims.Count;
                activeColorCount += perColorContactCount + perColorJointCount > 0 ? 1 : 0;
                awakeJointCount += perColorJointCount;
            }
            for (int i = bodyMoveEvents.Count; i < awakeBodyCount; i++) bodyMoveEvents.Add(new());
            int blocksPerWorker = 4;
            int maxBlockCount = blocksPerWorker * workerCount;
            int bodyBlockSize = 1 << 5;
            int bodyBlockCount;
            if (awakeBodyCount > bodyBlockSize * maxBlockCount)
            {
                bodyBlockSize = awakeBodyCount / maxBlockCount;
                bodyBlockCount = maxBlockCount;
            }
            else bodyBlockCount = ((awakeBodyCount - 1) >> 5) + 1;
            int[] activeColorIndices = new int[Box2D.GraphColorCount],
                colorContactCounts = new int[Box2D.GraphColorCount],
                colorContactBlockSizes = new int[Box2D.GraphColorCount],
                colorContactBlockCounts = new int[Box2D.GraphColorCount],
                colorJointCounts = new int[Box2D.GraphColorCount],
                colorJointBlockSizes = new int[Box2D.GraphColorCount],
                colorJointBlockCounts = new int[Box2D.GraphColorCount];
            int graphBlockCount = 0;
            int simdContactCount = 0;
            int c = 0;
            for (int i = 0; i < Box2D.GraphColorCount - 1; i++)
            {
                int colorContactCount = constraintGraph.colors[i].contactSims.Count;
                int colorJointCount = constraintGraph.colors[i].jointSims.Count;
                if (colorContactCount + colorJointCount > 0)
                {
                    activeColorIndices[c] = i;
                    int colorContactCountSIMD = colorContactCount > 0 ? ((colorContactCount - 1) >> SIMD_SHIFT) + 1 : 0;
                    colorContactCounts[c] = colorContactCountSIMD;
                    if (colorContactCountSIMD > blocksPerWorker * maxBlockCount)
                    {
                        colorContactBlockSizes[c] = colorContactCountSIMD / maxBlockCount;
                        colorContactBlockCounts[c] = maxBlockCount;
                    }
                    else if (colorContactCountSIMD > 0)
                    {
                        colorContactBlockSizes[c] = blocksPerWorker;
                        colorContactBlockCounts[c] = ((colorContactCountSIMD - 1) >> 2) + 1;
                    }
                    else
                    {
                        colorContactBlockSizes[c] = 0;
                        colorContactBlockCounts[c] = 0;
                    }
                    colorJointCounts[c] = colorJointCount;
                    if (colorJointCount > blocksPerWorker * maxBlockCount)
                    {
                        colorJointBlockSizes[c] = colorJointCount / maxBlockCount;
                        colorJointBlockCounts[c] = maxBlockCount;
                    }
                    else if (colorJointCount > 0)
                    {
                        colorJointBlockSizes[c] = blocksPerWorker;
                        colorJointBlockCounts[c] = ((colorJointCount - 1) >> 2) + 1;
                    }
                    else
                    {
                        colorJointBlockSizes[c] = 0;
                        colorJointBlockCounts[c] = 0;
                    }
                    graphBlockCount += colorContactBlockCounts[c] + colorJointBlockCounts[c];
                    simdContactCount += colorContactCountSIMD;
                    c++;
                }
            }
            activeColorCount = c;
            ContactSim[] contacts = new ContactSim[SIMD_WIDTH * simdContactCount];
            JointSim[] joints = new JointSim[awakeJointCount];
            IContactConstraintsSIMD simdContactConstraints = IContactConstraintsSIMD.Alloc(simdContactCount);
            int overflowContactCount = constraintGraph.colors[Box2D.GraphColorCount - 1].contactSims.Count;
            ContactConstraint[] overflowContactConstraints = new ContactConstraint[overflowContactCount];
            constraintGraph.colors[Box2D.GraphColorCount - 1].overflowConstraints = overflowContactConstraints;
            {
                int contactBase = 0;
                int jointBase = 0;
                for (int i = 0; i < activeColorCount; i++)
                {
                    int j = activeColorIndices[i];
                    GraphColor color = constraintGraph.colors[j];
                    int colorContactCount = color.contactSims.Count;
                    if (colorContactCount == 0) color.simdConstraints = null;
                    else
                    {
                        color.simdConstraints = simdContactConstraints.PointTo(contactBase);
                        for (int k = 0; k < colorContactCount; ++k)
                            contacts[SIMD_WIDTH * contactBase + k] = color.contactSims[k];
                        int colorContactCountSIMD = ((colorContactCount - 1) >> SIMD_SHIFT) + 1;
                        for (int k = colorContactCount; k < SIMD_WIDTH * colorContactCountSIMD; ++k)
                            contacts[SIMD_WIDTH * contactBase + k] = null;
                        contactBase += colorContactCountSIMD;
                    }

                    int colorJointCount = color.jointSims.Count;
                    for (int k = 0; k < colorJointCount; ++k) joints[jointBase + k] = color.jointSims[k];
                    jointBase += colorJointCount;
                }
                Debug.Assert(contactBase == simdContactCount);
                Debug.Assert(jointBase == awakeJointCount);
            }
            int contactBlockSize = blocksPerWorker;
            int contactBlockCount = simdContactCount > 0 ? ((simdContactCount - 1) >> 2) + 1 : 0;
            if (simdContactCount > contactBlockSize * maxBlockCount)
            {
                contactBlockSize = simdContactCount / maxBlockCount;
                contactBlockCount = maxBlockCount;
            }
            int jointBlockSize = blocksPerWorker;
            int jointBlockCount = awakeJointCount > 0 ? ((awakeJointCount - 1) >> 2) + 1 : 0;
            if (awakeJointCount > jointBlockSize * maxBlockCount)
            {
                jointBlockSize = awakeJointCount / maxBlockCount;
                jointBlockCount = maxBlockCount;
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
            SolverBlock* bodyBlocks = (SolverBlock*)arena.AllocateArenaItem(bodyBlockCount * sizeof(SolverBlock), "body blocks"),
                contactBlocks = (SolverBlock*)arena.AllocateArenaItem(contactBlockCount * sizeof(SolverBlock), "contact blocks"),
                jointBlocks = (SolverBlock*)arena.AllocateArenaItem(jointBlockCount * sizeof(SolverBlock), "joint blocks"),
                graphBlocks = (SolverBlock*)arena.AllocateArenaItem(graphBlockCount * sizeof(SolverBlock), "graph blocks");
            object splitIslandTask = null;
            if (splitIslandId != -1)
            {
                splitIslandTask = enqueueTaskFcn(SplitIslandTask, 1, 1, this, userTaskContext);
                taskCount++;
                activeTaskCount += splitIslandTask == null ? 0 : 1;
            }
            for (int i = 0; i < bodyBlockCount; i++)
            {
                SolverBlock* block = bodyBlocks + i;
                block->startIndex = i * bodyBlockSize;
                block->count = (short)bodyBlockSize;
                block->blockType = SolverBlockType.Body;
                Interlocked.Exchange(ref block->syncIndex, 0);
            }
            bodyBlocks[bodyBlockCount - 1].count = (short)(awakeBodyCount - (bodyBlockCount - 1) * bodyBlockSize);
            for (int i = 0; i < jointBlockCount; i++)
            {
                SolverBlock* block = jointBlocks + i;
                block->startIndex = i * jointBlockSize;
                block->count = (short)jointBlockSize;
                block->blockType = SolverBlockType.Joint;
                Interlocked.Exchange(ref block->syncIndex, 0);
            }
            if (jointBlockCount > 0)
                jointBlocks[jointBlockCount - 1].count = (short)(awakeJointCount - (jointBlockCount - 1) * jointBlockSize);
            for (int i = 0; i < contactBlockCount; i++)
            {
                SolverBlock* block = contactBlocks + i;
                block->startIndex = i * contactBlockSize;
                block->count = (short)contactBlockSize;
                block->blockType = SolverBlockType.Contact;
                Interlocked.Exchange(ref block->syncIndex, 0);
            }
            if (contactBlockCount > 0)
                contactBlocks[contactBlockCount - 1].count = (short)(simdContactCount - (contactBlockCount - 1) * contactBlockSize);
            SolverBlock*[] graphColorBlocks = new SolverBlock*[Box2D.GraphColorCount];
            int baseGraphBlock = 0;
            for (int i = 0; i < activeColorCount; i++)
            {
                graphColorBlocks[i] = graphBlocks + baseGraphBlock;
                int colorJointBlockCount = colorJointBlockCounts[i];
                int colorJointBlockSize = colorJointBlockSizes[i];
                for (int j = 0; j < colorJointBlockCount; ++j)
                {
                    ref SolverBlock block = ref graphBlocks[baseGraphBlock + j];
                    block.startIndex = j * colorJointBlockSize;
                    block.count = (short)colorJointBlockSize;
                    block.blockType = SolverBlockType.GraphJoint;
                    Interlocked.Exchange(ref block.syncIndex, 0);
                }
                if (colorJointBlockCount > 0)
                {
                    graphBlocks[baseGraphBlock + colorJointBlockCount - 1].count =
                        (short)(colorJointCounts[i] - (colorJointBlockCount - 1) * colorJointBlockSize);
                    baseGraphBlock += colorJointBlockCount;
                }
                int colorContactBlockCount = colorContactBlockCounts[i];
                int colorContactBlockSize = colorContactBlockSizes[i];
                for (int j = 0; j < colorContactBlockCount; ++j)
                {
                    ref SolverBlock block = ref graphBlocks[baseGraphBlock + j];
                    block.startIndex = j * colorContactBlockSize;
                    block.count = (short)colorContactBlockSize;
                    block.blockType = SolverBlockType.GraphContact;
                    Interlocked.Exchange(ref block.syncIndex, 0);
                }

                if (colorContactBlockCount > 0)
                {
                    graphBlocks[baseGraphBlock + colorContactBlockCount - 1].count =
                        (short)(colorContactCounts[i] - (colorContactBlockCount - 1) * colorContactBlockSize);
                    baseGraphBlock += colorContactBlockCount;
                }
            }
            Debug.Assert(baseGraphBlock == graphBlockCount);
            int stageIndex = 0;
            ref SolverStage stage = ref stages[stageIndex];
            stage = new()
            {
                type = SolverStageType.PrepareJoints,
                blocks = jointBlocks,
                blockCount = jointBlockCount,
                colorIndex = -1
            };
            Interlocked.Exchange(ref stages[stageIndex].completionCount, 0);
            stages[++stageIndex] = new()
            {
                type = SolverStageType.PrepareContacts,
                blocks = contactBlocks,
                blockCount = contactBlockCount,
                colorIndex = -1
            };
            Interlocked.Exchange(ref stages[stageIndex].completionCount, 0);
            stages[++stageIndex] = new()
            {
                type = SolverStageType.IntegrateVelocities,
                blocks = bodyBlocks,
                blockCount = bodyBlockCount,
                colorIndex = -1
            };
            Interlocked.Exchange(ref stages[stageIndex].completionCount, 0);
            for (int i = 0; i < activeColorCount; i++)
            {
                stages[++stageIndex] = new()
                {
                    type = SolverStageType.WarmStart,
                    blocks = graphColorBlocks[i],
                    blockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i],
                    colorIndex = activeColorIndices[i]
                };
                Interlocked.Exchange(ref stages[stageIndex].completionCount, 0);
            }
            for (int j = 0; j < ITERATIONS; j++) for (int i = 0; i < activeColorCount; i++)
                {
                    stages[++stageIndex] = new()
                    {
                        type = SolverStageType.Solve,
                        blocks = graphColorBlocks[i],
                        blockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i],
                        colorIndex = activeColorIndices[i]
                    };
                    Interlocked.Exchange(ref stages[stageIndex].completionCount, 0);
                }
            stages[++stageIndex] = new()
            {
                type = SolverStageType.IntegratePositions,
                blocks = bodyBlocks,
                blockCount = bodyBlockCount,
                colorIndex = -1
            };
            Interlocked.Exchange(ref stages[stageIndex].completionCount, 0);
            for (int j = 0; j < RELAX_ITERATIONS; j++) for (int i = 0; i < activeColorCount; i++)
                {
                    stages[++stageIndex] = new()
                    {
                        type = SolverStageType.Relax,
                        blocks = graphColorBlocks[i],
                        blockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i],
                        colorIndex = activeColorIndices[i]
                    };
                    Interlocked.Exchange(ref stages[stageIndex].completionCount, 0);
                }
            for (int i = 0; i < activeColorCount; i++)
            {
                stages[++stageIndex] = new()
                {
                    type = SolverStageType.Restitution,
                    blocks = graphColorBlocks[i],
                    blockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i],
                    colorIndex = activeColorIndices[i]
                };
                Interlocked.Exchange(ref stages[stageIndex].completionCount, 0);
            }
            stages[++stageIndex] = new()
            {
                type = SolverStageType.StoreImpulses,
                blocks = contactBlocks,
                blockCount = contactBlockCount,
                colorIndex = -1
            };
            Interlocked.Exchange(ref stages[stageIndex].completionCount, 0);
            stageIndex++;
            Debug.Assert(stageIndex == stageCount);
            Debug.Assert(workerCount <= Box2D.MaxWorkers);
            WorkerContext[] workerContext = new WorkerContext[Box2D.MaxWorkers];
            stepContext.graph = constraintGraph;
            stepContext.joints = joints;
            stepContext.contacts = contacts;
            stepContext.simdContactConstraints = simdContactConstraints;
            stepContext.activeColorCount = activeColorCount;
            stepContext.workerCount = workerCount;
            stepContext.stages = stages;
            Interlocked.Exchange(ref stepContext.atomicSyncBits, 0);
            profile.prepareStages = (float)prepareTicks.Elapsed.TotalMilliseconds;
            prepareTicks.Stop();
            Stopwatch constraintTicks = new(); constraintTicks.Start();
            int jointIdCapacity = jointIdPool.GetIdCapacity();
            for (int i = 0; i < workerCount; i++)
            {
                TaskContext taskContext = taskContexts[i];
                taskContext.jointStateBitSet.SetBitCountAndClear(jointIdCapacity);
                workerContext[i] = new()
                {
                    context = stepContext,
                    workerIndex = i
                };
                workerContext[i].userTask = enqueueTaskFcn(SolverTask, 1, 1, workerContext[i], userTaskContext);
                taskCount++;
                activeTaskCount += workerContext[i].userTask == null ? 0 : 1;
            }
            if (splitIslandTask != null)
            { finishTaskFcn(splitIslandTask, userTaskContext); activeTaskCount--; }
            splitIslandId = -1;
            for (int i = 0; i < workerCount; i++)
            {
                if (workerContext[i].userTask != null)
                {
                    finishTaskFcn(workerContext[i].userTask, userTaskContext);
                    activeTaskCount--;
                }
            }
            profile.solveConstraints = (float)constraintTicks.Elapsed.TotalMilliseconds;
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
            object finalizeBodiesTask = enqueueTaskFcn(FinalizeBodiesTask, awakeBodyCount, 64, stepContext, userTaskContext);
            taskCount++;
            if (finalizeBodiesTask != null) finishTaskFcn(finalizeBodiesTask, userTaskContext);
            arena.FreeArenaItem(graphBlocks);
            arena.FreeArenaItem(jointBlocks);
            arena.FreeArenaItem(contactBlocks);
            arena.FreeArenaItem(bodyBlocks);
            simdContactConstraints.Free();
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
            for (int i = 0; i < Box2D.GraphColorCount; i++)
            {
                GraphColor color = constraintGraph.colors[i];
                int contactCount = color.contactSims.Count;
                for (int j = 0; j < contactCount; j++)
                {
                    ContactSim contactSim = color.contactSims[j];
                    if (!contactSim.simFlags.HasFlag(ContactSimFlags.EnableHitEvent)) continue;
                    ContactHitEvent event_ = new() { approachSpeed = hitEventThreshold };
                    bool hit = false;
                    if (contactSim.manifold.pointCount > 0)
                    {
                        ref ManifoldPoint mp = ref contactSim.manifold.point0;
                        float approachSpeed = -mp.normalVelocity;
                        if (approachSpeed > event_.approachSpeed && mp.totalNormalImpulse > 0)
                        { event_.approachSpeed = approachSpeed; event_.point = mp.point; hit = true; }
                    }
                    if (contactSim.manifold.pointCount > 1)
                    {
                        ref ManifoldPoint mp = ref contactSim.manifold.point1;
                        float approachSpeed = -mp.normalVelocity;
                        if (approachSpeed > event_.approachSpeed && mp.totalNormalImpulse > 0)
                        { event_.approachSpeed = approachSpeed; event_.point = mp.point; hit = true; }
                    }
                    if (hit)
                    {
                        event_.normal = contactSim.manifold.normal;
                        Shape shapeA = shapes[contactSim.shapeIdA], shapeB = shapes[contactSim.shapeIdB];
                        event_.shapeIdA = new() { index1 = shapeA.id + 1, world0 = this, generation = shapeA.generation };
                        event_.shapeIdB = new() { index1 = shapeB.id + 1, world0 = this, generation = shapeB.generation };
                        contactHitEvents.Add(event_);
                    }
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
            object userBulletBodyTask = enqueueTaskFcn(BulletBodyTask, bulletBodyCount, minRange, stepContext, userTaskContext);
            taskCount++;
            if (userBulletBodyTask != null) finishTaskFcn(userBulletBodyTask, userTaskContext);
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
                    Debug.Assert(broadPhase.moveSet.Contains(proxyKey));
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