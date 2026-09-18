using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace Box2D;

enum SetType
{
    Static = 0,
    Disabled = 1,
    Awake = 2,
    FirstSleeping = 3,
};

/// <summary>Per thread task storage</summary>
public class TaskContext
{
    ///<summary> Collect per thread sensor continuous hit events.</summary>
    public List<SensorHit> sensorHits = new();

    /// <summary>Broad-phase pairs.</summary>
    public List<ulong> pairKeys = new();

    ///<summary> These bits align with the contact id capacity and signal a change in contact status</summary>
    public BitSet contactStateBitSet;

    /// <summary>These bits align with the contact id capacity and signal a hit event.</summary>
    public BitSet hitEventBitSet;

    /// <summary>Fast-path flag: true when this worker set at least one bit in hitEventBitSet this step.</summary>
    public bool hasHitEvents;

    ///<summary> These bits align with the joint id capacity and signal a change in contact status</summary>
    public BitSet jointStateBitSet;

    ///<summary> Used to track bodies with shapes that have enlarged AABBs. This avoids having a bit array
    /// that is very large when there are many static shapes.</summary>
    public BitSet enlargedSimBitSet;

    ///<summary> Used to put islands to sleep</summary>
    public BitSet awakeIslandBitSet;

    ///<summary> Per worker split island candidate</summary>
    public float splitSleepTime;
    public int splitIslandId;

    /// <summary>Number of contacts recycled this step (collide pass).</summary>
    public int recycledContactCount;
}

/// <summary>The world struct manages all physics entities, dynamic simulation,  and asynchronous queries.
/// The world also contains efficient memory management facilities.</summary>
public partial class World
{
    public B2Stack stack = new(2048);
    public BroadPhase broadPhase;
    public ConstraintGraph constraintGraph;

    ///<summary> The body id pool is used to allocate and recycle body ids. Body ids
    /// provide a stable identifier for users, but incur caches misses when used
    /// to access body data. Aligns with Body.</summary>
    public IDPool bodyIdPool = new();

    ///<summary> This is a sparse array that maps body ids to the body data
    /// stored in solver sets. As sims move within a set or across set.
    /// Indices come from id pool.</summary>
    public List<Body> bodies = new(16);

    ///<summary> Provides free list for solver sets.</summary>
    public IDPool solverSetIdPool = new();

    ///<summary> Solvers sets allow sims to be stored in contiguous arrays. The first
    /// set is all static sims. The second set is active sims. The third set is disabled
    /// sims. The remaining sets are sleeping islands.</summary>
    public List<SolverSet> solverSets = new(8);

    ///<summary> Used to create stable ids for joints</summary>
    public IDPool jointIdPool = new();

    ///<summary> This is a sparse array that maps joint ids to the joint data stored in the constraint graph
    /// or in the solver sets.</summary>
    public List<Joint> joints = new(16);

    ///<summary> Used to create stable ids for contacts</summary>
    public IDPool contactIdPool = new();

    ///<summary> This is a sparse array that maps contact ids to the contact data stored in the constraint graph
    /// or in the solver sets.</summary>
    public List<Contact> contacts = new(16);

    ///<summary> Used to create stable ids for islands</summary>
    public IDPool islandIdPool = new();

    ///<summary> This is a sparse array that maps island ids to the island data stored in the solver sets.</summary>
    public List<Island> islands = new(8);

    public IDPool shapeIdPool = new();
    public IDPool chainIdPool = new();

    ///<summary> These are sparse arrays that point into the pools above</summary>
    public List<Shape> shapes = new(16);
    public List<AABB> fatAABBs = new(4);
    public List<ChainShape> chainShapes = new(4);

    ///<summary> This is a dense array of sensor data.</summary>
    public List<Sensor> sensors = new(4);

    ///<summary> Per thread storage</summary>
    public List<TaskContext> taskContexts = new();
    public List<SensorTaskContext> sensorTaskContexts = new();

    public List<BodyMoveEvent> bodyMoveEvents = new(4);
    public List<SensorBeginTouchEvent> sensorBeginEvents = new(4);
    public List<ContactBeginTouchEvent> contactBeginEvents = new(4);

    ///<summary> End events are double buffered so that the user doesn't need to flush events</summary>
    public List<SensorEndTouchEvent> sensorEndEvents0 = new(4), sensorEndEvents1 = new(4);
    public List<ContactEndTouchEvent> contactEndEvents0 = new(4), contactEndEvents1 = new(4);
    public int endEventArrayIndex;

    public List<ContactHitEvent> contactHitEvents = new(4);
    public List<JointEvent> jointEvents = new(4);

    ///<summary> Used to track debug draw</summary>
    public BitSet debugBodySet = new(256);
    public BitSet debugJointSet = new(256);
    public BitSet debugContactSet = new(256);
    public BitSet debugIslandSet = new(256);

    ///<summary> Id that is incremented every time step</summary>
    public ulong stepIndex = 0;

    ///<summary> Identify islands for splitting as follows:<br/>
    /// - I want to split islands so smaller islands can sleep<br/>
    /// - when a body comes to rest and its sleep timer trips, I can look at the island and flag it for splitting<br/>
    ///   if it has removed constraints
    /// - islands that have removed constraints must be put split first because I don't want to wake bodies incorrectly<br/>
    /// - otherwise I can use the awake islands that have bodies wanting to sleep as the splitting candidates<br/>
    /// - if no bodies want to sleep then there is no reason to perform island splitting</summary>
    public int splitIslandId = -1;

    public Vector2 gravity;
    public float hitEventThreshold;
    public float restitutionThreshold;
    public float maxLinearSpeed;
    public float contactSpeed;
    public float contactHertz;
    public float contactDampingRatio;
    public float contactRecycleDistance;

    public FrictionCallback frictionCallback = (frictionA, _, frictionB, _) => MathF.Sqrt(frictionA * frictionB);
    public RestitutionCallback restitutionCallback = (restitutionA, _, restitutionB, _) => Math.Max(restitutionA, restitutionB);

    public ushort generation;

    public Profile profile;

    public Capacity maxCapacity;

    public PreSolveFcn preSolveFcn;
    public PreContinuousFcn preContinuousFcn;
    public object preSolveContext;

    public CustomFilterFcn customFilterFcn;
    public object customFilterContext;

    public int workerCount = 1;
    public EnqueueTaskCallback enqueueTaskFcn = (task, taskContext, _) => { task(taskContext); return null!; };
    public FinishTaskCallback finishTaskFcn = (_, _) => { };
    public object userTaskContext = null;
    public object userTreeTask = null;

    public Scheduler scheduler;

    public object userData;

    ///<summary> Remember type step used for reporting forces and torques</summary>
    public float inv_h;

    /// <summary>inverse full-step</summary>
    public float inv_dt;

    public int activeTaskCount = 0;
    public int taskCount = 0;

    public bool enableSleep;
    public bool locked = false;
    public bool enableWarmStarting = true;
    public bool enableContinuous;
    public bool inUse = true;

    public List<Particle.ParticleSystem> particleSystemList = new();
    public static World GetUnlockedWorldFromId(WorldID id)
    {
        if (id.index1.locked) { Debug.Assert(false); return null; }
        return id.index1;
    }
    public static World GetWorldLocked(World world)
    {
        if (world.locked) throw new ArgumentException("World is already locked");
        return world;
    }
    public void CreateWorkerContexts()
    {
        taskContexts = new(workerCount);
        sensorTaskContexts = new(workerCount);
        for (int i = 0; i < workerCount; i++)
        {
            taskContexts.Add(new()
            {
                sensorHits = new(8),
                pairKeys = new(),
                contactStateBitSet = new(1024),
                hitEventBitSet = new(1024),
                jointStateBitSet = new(1024),
                enlargedSimBitSet = new(256),
                awakeIslandBitSet = new(256)
            });
            sensorTaskContexts.Add(new() { eventBits = new(128) });
        }
    }
    public World(ref WorldDef def)
    {
        broadPhase = new(ref def.capacity);
        constraintGraph = new(ref def.capacity);

        Debug.Assert(def.internalValue == Box2D.SECRET_COOKIE);
        solverSets.Add(new() { setIndex = solverSetIdPool.AllocId() });
        Debug.Assert(solverSets[(int)SetType.Static].setIndex == (int)SetType.Static);
        solverSets.Add(new() { setIndex = solverSetIdPool.AllocId() });
        Debug.Assert(solverSets[(int)SetType.Disabled].setIndex == (int)SetType.Disabled);
        solverSets.Add(new() { setIndex = solverSetIdPool.AllocId() });
        Debug.Assert(solverSets[(int)SetType.Awake].setIndex == (int)SetType.Awake);

        gravity = def.gravity;
        hitEventThreshold = def.hitEventThreshold;
        maxLinearSpeed = def.maximumLinearSpeed;
        contactSpeed = def.contactSpeed;
        contactHertz = def.contactHertz;
        contactDampingRatio = def.contactDampingRatio;
        contactRecycleDistance = Box2D.ContactRecycleDistance;
        if (def.frictionCallback != null) frictionCallback = def.frictionCallback;
        if (def.restitutionCallback != null) restitutionCallback = def.restitutionCallback;
        enableSleep = def.enableSleep;
        enableContinuous = def.enableContinuous;
        userData = def.userData;
        if (def.WorkerCount > 0 && def.enqueueTask != null && def.finishTask != null)
        {
            workerCount = Math.Min(def.WorkerCount, Box2D.MaxWorkers);
            enqueueTaskFcn = def.enqueueTask;
            finishTaskFcn = def.finishTask;
            userTaskContext = def.userTaskContext ?? new DefaultTaskContext(workerCount);
            scheduler = null;
        }
        else if (def.WorkerCount > 1)
        {
            workerCount = Math.Min(def.WorkerCount, Box2D.MaxWorkers);
            scheduler = new(workerCount);
            enqueueTaskFcn = SchedulerEnqueueTask;
            finishTaskFcn = SchedulerFinishTask;
            userTaskContext = scheduler;
        }
        CreateWorkerContexts();
    }
    public void Destroy()
    {
        scheduler?.Destroy();
        debugBodySet.Destroy();
        debugJointSet.Destroy();
        debugContactSet.Destroy();
        debugIslandSet.Destroy();
        for (int i = 0; i < workerCount; i++)
        {
            taskContexts[i].contactStateBitSet.Destroy();
            taskContexts[i].jointStateBitSet.Destroy();
            taskContexts[i].enlargedSimBitSet.Destroy();
            taskContexts[i].awakeIslandBitSet.Destroy();
            sensorTaskContexts[i].eventBits.Destroy();
        }
        int chainCapacity = chainShapes.Count;
        for (int i = 0; i < chainCapacity; i++)
        {
            ChainShape chain = chainShapes[i];
            if (chain.id != -1) { }
            else
            {
                Debug.Assert(chain.shapeIndices == null);
            }
        }
        int setCapacity = solverSets.Count;
        for (int i = 0; i < setCapacity; i++)
        {
            SolverSet set = solverSets[i];
            if (set.setIndex != -1) DestroySolverSet(i);
        }
        stack.Destroy();
        generation++;
    }
    unsafe BodySim ResolveContactBodySim(List<BodySim> awakeSims, List<BodySim> staticSims, int encodedIndex, int bodyId)
    {
        if (encodedIndex >= 0) return awakeSims[encodedIndex];
        if (Body.IsStaticSimIndex(encodedIndex)) return staticSims[-encodedIndex - 2];
        return GetBodySim(bodies[bodyId]);
    }
    public static unsafe void CollideTask(int startIndex, int endIndex, int threadIndex, object context)
    {
        StepContext stepContext = (StepContext)context;
        World world = stepContext.world;
        TaskContext taskContext = world.taskContexts[threadIndex];
        var spans = stepContext.collideSpans;
        List<Shape> shapes = world.shapes;
        var fatAABBs = System.Runtime.InteropServices.CollectionsMarshal.AsSpan(world.fatAABBs);
        var awakeSims = world.solverSets[(int)SetType.Awake].bodySims;
        var staticSims = world.solverSets[(int)SetType.Static].bodySims;
        var states = world.solverSets[(int)SetType.Awake].bodyStates.Data;
        Debug.Assert(startIndex < endIndex);
        int spanIndex = 0;
        while (spans[spanIndex + 1].start <= startIndex)
            spanIndex++;
        int spanStart = spans[spanIndex].start;
        int spanEnd = spans[spanIndex + 1].start;
        List<ContactSim> spanBase = spans[spanIndex].contacts;
        float speculativeDistance = Box2D.SpeculativeDistance;
        float recycleDistanceNonTouching = Math.Min(world.contactRecycleDistance, speculativeDistance);
        for (int contactIndex = startIndex; contactIndex < endIndex; contactIndex++)
        {
            if (contactIndex == spanEnd)
            {
                spanIndex++;
                spanStart = spans[spanIndex].start;
                spanEnd = spans[spanIndex + 1].start;
                spanBase = spans[spanIndex].contacts;
            }
            ContactSim contactSim = spanBase[contactIndex - spanStart];
            int contactId = contactSim.contactId;
            bool overlap = AABBV.OverlapV(ref fatAABBs[contactSim.shapeIdA], ref fatAABBs[contactSim.shapeIdB]);
            if (!overlap)
            {
                contactSim.simFlags |= ContactFlags.SimDisjoint;
                contactSim.simFlags &= ~ContactFlags.SimTouching;
                taskContext.contactStateBitSet.SetBit(contactId);
            }
            else
            {
                Shape shapeA = shapes[contactSim.shapeIdA], shapeB = shapes[contactSim.shapeIdB];
                bool wasTouching = contactSim.simFlags.HasFlag(ContactFlags.SimTouching);
                int encodedA = contactSim.encodedBodySimA, encodedB = contactSim.encodedBodySimB;
                BodySim bodySimA = world.ResolveContactBodySim(awakeSims, staticSims, encodedA, shapeA.bodyId),
                    bodySimB = world.ResolveContactBodySim(awakeSims, staticSims, encodedB, shapeB.bodyId);
                WorldTransform transformA = bodySimA.transform, transformB = bodySimB.transform;
                contactSim.invMassA = bodySimA.invMass;
                contactSim.invIA = bodySimA.invInertia;
                contactSim.invMassB = bodySimB.invMass;
                contactSim.invIB = bodySimB.invInertia;
                bool isFast = ((bodySimA.flags | bodySimB.flags) & BodyFlags.IsFast) != 0;
                if (!isFast && world.contactRecycleDistance > 0 && contactSim.simFlags.HasFlag(ContactFlags.SimRelativeTransformValid) && contactSim.simFlags.HasFlag(ContactFlags.Recycle))
                {
                    Rotation cachedQA = contactSim.cachedRotationA, cachedQB = contactSim.cachedRotationB;
                    Transform xfc = contactSim.cachedRelativePose, xf = WorldTransform.InvMulWorldTransforms(transformA, transformB);
                    float cosA = Rotation.RelativeCos(transformA.q, cachedQA);
                    float cosB = Rotation.RelativeCos(transformB.q, cachedQB);
                    float minCos = Math.Min(cosA, cosB);
                    float maxExtentA = Body.IsStaticSimIndex(encodedA) ? 0 : bodySimA.maxExtent;
                    float maxExtentB = Body.IsStaticSimIndex(encodedB) ? 0 : bodySimB.maxExtent;
                    float maxExtent = Math.Max(maxExtentA, maxExtentB);
                    float distance = Vector2.Distance(xf.p, xfc.p);
                    Rotation qr = Rotation.InvMulRot(xf.q, xfc.q);
                    float tolerance = wasTouching ? world.contactRecycleDistance : recycleDistanceNonTouching;
                    if (minCos > Box2D.ContactRecycleCosAngle && distance + maxExtent * Math.Abs(qr.s) < tolerance)
                    {
                        Rotation dqA = transformA.q * cachedQA.Invert();
                        Rotation dqB = transformB.q * cachedQB.Invert();
                        Vector2 normal = contactSim.manifold.normal;
                        Vector2 dc = bodySimB.center - bodySimA.center;
                        if (contactSim.manifold.pointCount > 0)
                        {
                            ref ManifoldPoint mp = ref contactSim.manifold.point0;
                            Vector2 rA = dqA * mp.anchorA;
                            Vector2 rB = dqB * mp.anchorB;
                            Vector2 dp = dc + (rB - rA);
                            mp.separation = mp.baseSeparation + Vector2.Dot(dp, normal);
                            mp.restitutionVelocity = mp.totalNormalImpulse > 0 && mp.normalVelocity < -world.restitutionThreshold ? -contactSim.restitution * mp.normalVelocity : 0;
                            int indexA = Body.DecodeAwakeIndex(encodedA);
                            Vector2 vrA = Vector2.Zero;
                            if (indexA != -1)
                            {
                                ref BodyState stateA = ref states[indexA];
                                vrA = stateA.linearVelocity + Vector2.CrossSV(stateA.angularVelocity, mp.anchorA);
                            }
                            int indexB = Body.DecodeAwakeIndex(encodedB);
                            Vector2 vrB = Vector2.Zero;
                            if (indexB != -1)
                            {
                                ref BodyState stateB = ref states[indexB];
                                vrB = stateB.linearVelocity + Vector2.CrossSV(stateB.angularVelocity, mp.anchorB);
                            }
                            mp.normalVelocity = Vector2.Dot(contactSim.manifold.normal, vrB - vrA);
                            mp.persisted = true;
                            if (contactSim.manifold.pointCount > 1)
                            {
                                mp = ref contactSim.manifold.point1;
                                rA = dqA * mp.anchorA;
                                rB = dqB * mp.anchorB;
                                dp = dc + (rB - rA);
                                mp.separation = mp.baseSeparation + Vector2.Dot(dp, normal);
                                mp.restitutionVelocity = mp.totalNormalImpulse > 0 && mp.normalVelocity < -world.restitutionThreshold ? -contactSim.restitution * mp.normalVelocity : 0;
                                indexA = Body.DecodeAwakeIndex(encodedA);
                                vrA = Vector2.Zero;
                                if (indexA != -1)
                                {
                                    ref BodyState stateA = ref states[indexA];
                                    vrA = stateA.linearVelocity + Vector2.CrossSV(stateA.angularVelocity, mp.anchorA);
                                }
                                indexB = Body.DecodeAwakeIndex(encodedB);
                                vrB = Vector2.Zero;
                                if (indexB != -1)
                                {
                                    ref BodyState stateB = ref states[indexB];
                                    vrB = stateB.linearVelocity + Vector2.CrossSV(stateB.angularVelocity, mp.anchorB);
                                }
                                mp.normalVelocity = Vector2.Dot(contactSim.manifold.normal, vrB - vrA);
                                mp.persisted = true;
                            }
                        }
                        taskContext.recycledContactCount++;
                        continue;
                    }
                }
                contactSim.cachedRotationA = transformA.q;
                contactSim.cachedRotationB = transformB.q;
                contactSim.cachedRelativePose = WorldTransform.InvMulWorldTransforms(transformA, transformB);
                contactSim.simFlags |= ContactFlags.SimRelativeTransformValid;
                Vector2 centerOffsetA = transformA.q * bodySimA.localCenter;
                Vector2 centerOffsetB = transformB.q * bodySimB.localCenter;
                bool touching = world.UpdateContact(contactSim, shapeA, transformA, centerOffsetA, shapeB, transformB, centerOffsetB);
                if (touching && !wasTouching)
                {
                    contactSim.simFlags |= ContactFlags.SimStartedTouching;
                    taskContext.contactStateBitSet.SetBit(contactId);
                }
                else if (!touching && wasTouching)
                {
                    contactSim.simFlags |= ContactFlags.SimStoppedTouching;
                    taskContext.contactStateBitSet.SetBit(contactId);
                }
                if (touching) if (contactSim.manifold.pointCount > 0)
                {
                    ref ManifoldPoint mp = ref contactSim.manifold.point0;
                    mp.baseSeparation = mp.separation;
                    int indexA = contactSim.encodedBodySimA;
                    Vector2 vrA = Vector2.Zero;
                    if (indexA != -1)
                    {
                        ref BodyState stateA = ref states[indexA];
                        vrA = stateA.linearVelocity + Vector2.CrossSV(stateA.angularVelocity, mp.anchorA);
                    }
                    int indexB = contactSim.encodedBodySimB;
                    Vector2 vrB = Vector2.Zero;
                    if (indexB != -1)
                    {
                        ref BodyState stateB = ref states[indexB];
                        vrB = stateB.linearVelocity + Vector2.CrossSV(stateB.angularVelocity, mp.anchorB);
                    }
                    mp.normalVelocity = Vector2.Dot(contactSim.manifold.normal, vrB - vrA);
                    if (contactSim.manifold.pointCount > 1)
                    {
                        mp = ref contactSim.manifold.point1;
                        mp.baseSeparation = mp.separation;
                        indexA = contactSim.encodedBodySimA;
                        vrA = Vector2.Zero;
                        if (indexA != -1)
                        {
                            ref BodyState stateA = ref states[indexA];
                            vrA = stateA.linearVelocity + Vector2.CrossSV(stateA.angularVelocity, mp.anchorA);
                        }
                        indexB = contactSim.encodedBodySimB;
                        vrB = Vector2.Zero;
                        if (indexB != -1)
                        {
                            ref BodyState stateB = ref states[indexB];
                            vrB = stateB.linearVelocity + Vector2.CrossSV(stateB.angularVelocity, mp.anchorB);
                        }
                        mp.normalVelocity = Vector2.Dot(contactSim.manifold.normal, vrB - vrA);
                    }
                }
            }
        }
    }
    public void AddNonTouchingContact(Contact contact, ContactSim contactSim)
    {
        Debug.Assert(contact.setIndex == (int)SetType.Awake);
        SolverSet set = solverSets[(int)SetType.Awake];
        contact.colorIndex = -1;
        contact.localIndex = set.contactSims.Count;
        set.contactSims.Add(contactSim with { });
    }
    public void RemoveNonTouchingContact(int setIndex, int localIndex)
    {
        SolverSet set = solverSets[setIndex];
        int movedIndex = set.contactSims.RemoveSwap(localIndex);
        if (movedIndex != -1)
        {
            ContactSim movedContactSim = set.contactSims[localIndex];
            Contact movedContact = contacts[movedContactSim.contactId];
            Debug.Assert(movedContact.setIndex == setIndex);
            Debug.Assert(movedContact.localIndex == movedIndex);
            Debug.Assert(movedContact.colorIndex == -1);
            movedContact.localIndex = localIndex;
        }
    }
    public static void Collide(StepContext context)
    {
        World world = context.world;
        Debug.Assert(world.workerCount > 0);
        int contactCount = 0;
        GraphColor[] graphColors = world.constraintGraph.colors;
        for (int i = 0; i < Box2D.GraphColorCount; i++)
            contactCount += graphColors[i].contactSims.Count;
        int nonTouchingCount = world.solverSets[(int)SetType.Awake].contactSims.Count;
        contactCount += nonTouchingCount;
        if (contactCount == 0) return;
        ContactCollideSpan collideSpans = new();
        int spanCount = 0;
        int contactIndex = 0;
        for (int i = 0; i < Box2D.GraphColorCount; i++)
        {
            GraphColor color = graphColors[i];
            int count = color.contactSims.Count;
            if (count > 0)
            {
                collideSpans[spanCount].start = contactIndex;
                collideSpans[spanCount].contacts = color.contactSims;
                spanCount++;
                contactIndex += count;
            }
        }
        if (nonTouchingCount > 0)
        {
            collideSpans[spanCount].start = contactIndex;
            collideSpans[spanCount].contacts = world.solverSets[(int)SetType.Awake].contactSims;
            spanCount++;
            contactIndex += nonTouchingCount;
        }
        Debug.Assert(contactIndex == contactCount);
        Debug.Assert(spanCount <= Box2D.GraphColorCount + 1);
        collideSpans[spanCount].start = contactCount;
        collideSpans[spanCount].contacts = null;
        context.collideSpans = collideSpans;
        int contactIdCapacity = world.contactIdPool.GetIdCapacity();
        for (int i = 0; i < world.workerCount; i++)
        {
            world.taskContexts[i].contactStateBitSet.SetBitCountAndClear(contactIdCapacity);
            world.taskContexts[i].recycledContactCount = 0;
        }
        int minRange = 64;
        world.ParallelFor(CollideTask, contactCount, minRange, context);
        context.collideSpans = new();
        ref BitSet bitSet = ref world.taskContexts[0].contactStateBitSet;
        for (int i = 1; i < world.workerCount; i++)
            bitSet.InPlaceUnion(world.taskContexts[i].contactStateBitSet);
        SolverSet awakeSet = world.solverSets[(int)SetType.Awake];
        int endEventArrayIndex = world.endEventArrayIndex;
        List<Shape> shapes = world.shapes;
        for (uint k = 0; k < bitSet.blockCount; k++)
        {
            ulong bits = bitSet.bits[k];
            while (bits != 0)
            {
                uint ctz = CTZ.CTZ64(bits);
                int contactId = (int)(64 * k + ctz);
                Contact contact = world.contacts[contactId];
                Debug.Assert(contact.setIndex == (int)SetType.Awake);
                int colorIndex = contact.colorIndex;
                int localIndex = contact.localIndex;
                ContactSim contactSim = null;
                if (colorIndex != -1)
                {
                    Debug.Assert(0 <= colorIndex && colorIndex < Box2D.GraphColorCount);
                    contactSim = graphColors[colorIndex].contactSims[localIndex];
                }
                else contactSim = awakeSet.contactSims[localIndex];
                ContactFlags flags = contact.flags;
                ContactFlags simFlags = contactSim.simFlags;
                if (simFlags.HasFlag(ContactFlags.SimDisjoint))
                {
                    world.DestroyContact(contact);
                    contact = null;
                    contactSim = null;
                }
                else if (simFlags.HasFlag(ContactFlags.SimStartedTouching))
                {
                    Debug.Assert(contact.islandId == -1);
                    if (flags.HasFlag(ContactFlags.EnableContactEvents))
                    {
                        Shape shapeA = shapes[contact.shapeIdA];
                        Shape shapeB = shapes[contact.shapeIdB];
                        ShapeID shapeIdA = new() { index1 = shapeA.id + 1, world0 = world, generation = shapeA.generation };
                        ShapeID shapeIdB = new() { index1 = shapeB.id + 1, world0 = world, generation = shapeB.generation };
                        ContactID contactFullId = new() { index1 = contactId + 1, world0 = world, generation = contact.generation };
                        world.contactBeginEvents.Add(new() { shapeIdA = shapeIdA, shapeIdB = shapeIdB, contactId = contactFullId });
                    }
                    Debug.Assert(contactSim.manifold.pointCount > 0);
                    Debug.Assert(contact.setIndex == (int)SetType.Awake);
                    contact.flags |= ContactFlags.Touching;
                    world.LinkContact(contact);
                    Debug.Assert(contact.colorIndex == -1);
                    Debug.Assert(contact.localIndex == localIndex);
                    contactSim = awakeSet.contactSims[localIndex];
                    contactSim.simFlags &= ~ContactFlags.SimStartedTouching;
                    world.AddContactToGraph(contactSim, contact);
                    world.RemoveNonTouchingContact((int)SetType.Awake, localIndex);
                }
                else if (simFlags.HasFlag(ContactFlags.SimStoppedTouching))
                {
                    contactSim.simFlags &= ~ContactFlags.SimStoppedTouching;
                    contact.flags &= ~ContactFlags.Touching;
                    if (contact.flags.HasFlag(ContactFlags.EnableContactEvents))
                    {
                        Shape shapeA = shapes[contact.shapeIdA];
                        Shape shapeB = shapes[contact.shapeIdB];
                        ShapeID shapeIdA = new() { index1 = shapeA.id + 1, world0 = world, generation = shapeA.generation };
                        ShapeID shapeIdB = new() { index1 = shapeB.id + 1, world0 = world, generation = shapeB.generation };
                        ContactID contactFullId = new() { index1 = contactId + 1, world0 = world, generation = contact.generation };
                        if (endEventArrayIndex == 1) world.contactEndEvents1.Add(new() { shapeIdA = shapeIdA, shapeIdB = shapeIdB, contactId = contactFullId });
                        else world.contactEndEvents0.Add(new() { shapeIdA = shapeIdA, shapeIdB = shapeIdB, contactId = contactFullId });
                    }
                    Debug.Assert(contactSim.manifold.pointCount == 0);
                    world.UnlinkContact(contact);
                    int bodyIdA = contact.edge0.bodyId, bodyIdB = contact.edge1.bodyId;
                    world.AddNonTouchingContact(contact, contactSim);
                    world.RemoveContactFromGraph(bodyIdA, bodyIdB, colorIndex, localIndex);
                    contact = null;
                    contactSim = null;
                }
                bits &= bits - 1;
            }
        }
        world.ValidateSolverSets();
        world.ValidateContacts();
    }
    public void DrawWithBounds(DebugDraw draw)
    {
    }
    public void ValidateConnectivity()
    {
#if B2_VALIDATE
        for (int bodyIndex = 0; bodyIndex < bodies.Count; bodyIndex++)
        {
            Body body = bodies[bodyIndex];
            if (body.id == -1) { bodyIdPool.ValidateFreeID(bodyIndex); continue; }
            bodyIdPool.ValidateUsedID(bodyIndex);
            Debug.Assert(bodyIndex == body.id);
            int bodyIslandId = body.islandId;
            int bodySetIndex = body.setIndex;
            int contactKey = body.headContactKey;
            while (contactKey != -1)
            {
                int contactId = contactKey >> 1;
                int edgeIndex = contactKey & 1;
                Contact contact = contacts[contactId];
                if (((ContactFlags)contact.flags).HasFlag(ContactFlags.Touching))
                {
                    if (bodySetIndex != (int)SetType.Static)
                    {
                        int contactIslandId = contact.islandId;
                        Debug.Assert(contactIslandId == bodyIslandId);
                    }
                    else Debug.Assert(contact.islandId == -1);
                }
                contactKey = edgeIndex == 1 ? contact.edge1.nextKey : contact.edge0.nextKey;
            }
            int jointKey = body.headJointKey;
            while (jointKey != -1)
            {
                int jointId = jointKey >> 1;
                int edgeIndex = jointKey & 1;
                Joint joint = joints[jointId];
                int otherEdgeIndex = edgeIndex ^ 1;
                Body otherBody = bodies[otherEdgeIndex == 1 ? joint.edge1.bodyId : joint.edge0.bodyId];
                if (bodySetIndex == (int)SetType.Disabled || otherBody.setIndex == (int)SetType.Disabled)
                    Debug.Assert(joint.islandId == -1);
                else if (bodySetIndex == (int)SetType.Static)
                {
                    if (otherBody.setIndex == (int)SetType.Static) Debug.Assert(joint.islandId == -1);
                }
                else if (body.type != BodyType.Dynamic && otherBody.type != BodyType.Dynamic)
                    Debug.Assert(joint.islandId == -1);
                else
                {
                    int jointIslandId = joint.islandId;
                    Debug.Assert(jointIslandId == bodyIslandId);
                }
                jointKey = edgeIndex == 1 ? joint.edge1.nextKey : joint.edge0.nextKey;
            }
        }
#endif
    }
    public unsafe void ValidateSolverSets()
    {
#if B2_VALIDATE
        Debug.Assert(bodyIdPool.GetIdCapacity() == bodies.Count);
        Debug.Assert(contactIdPool.GetIdCapacity() == contacts.Count);
        Debug.Assert(jointIdPool.GetIdCapacity() == joints.Count);
        Debug.Assert(islandIdPool.GetIdCapacity() == islands.Count);
        Debug.Assert(solverSetIdPool.GetIdCapacity() == solverSets.Count);
        int activeSetCount = 0;
        int totalBodyCount = 0;
        int totalJointCount = 0;
        int totalContactCount = 0;
        int totalIslandCount = 0;
        int setCount = solverSets.Count;
        for (int setIndex = 0; setIndex < setCount; setIndex++)
        {
            SolverSet set = solverSets[setIndex];
            if (set.setIndex != -1)
            {
                activeSetCount++;
                switch ((SetType)setIndex)
                {
                    case SetType.Static:
                        Debug.Assert(set.contactSims.Count == 0);
                        Debug.Assert(set.islandSims.Count == 0);
                        Debug.Assert(set.bodyStates.Count == 0);
                        break;
                    case SetType.Disabled:
                        Debug.Assert(set.bodySims.Count == set.bodyStates.Count);
                        Debug.Assert(set.jointSims.Count == 0);
                        break;
                    case SetType.Awake:
                        Debug.Assert(set.bodySims.Count == set.bodyStates.Count);
                        Debug.Assert(set.jointSims.Count == 0);
                        break;
                    default:
                        Debug.Assert(set.bodyStates.Count == 0);
                        break;
                }
                {
                    Debug.Assert(set.bodyStates.Count >= 0);
                    totalBodyCount += set.bodySims.Count;
                    for (int i = 0; i < set.bodySims.Count; i++)
                    {
                        BodySim bodySim = set.bodySims[i];
                        int bodyId = bodySim.bodyId;
                        Debug.Assert(0 <= bodyId && bodyId < bodies.Count);
                        Body body = bodies[bodyId];
                        Debug.Assert(body.setIndex == setIndex);
                        Debug.Assert(body.localIndex == i);
                        var syncedFlags = body.flags & ~BodyFlags.TransientFlags;
                        Debug.Assert((body.flags & syncedFlags) == syncedFlags);
                        Debug.Assert(bodySim.flags.HasFlag(BodyFlags.IsFast) == bodySim.flags.HasFlag(BodyFlags.IsFast));
                        BodyState* bodyState = GetBodyState(body);
                        if (bodyState != null) Debug.Assert((bodyState->flags & syncedFlags) == syncedFlags);
                        if (body.type == BodyType.Dynamic) Debug.Assert(body.flags.HasFlag(BodyFlags.Dynamic));
                        if (setIndex == (int)SetType.Disabled) Debug.Assert(body.headContactKey == -1);
                        int prevShapeId = -1;
                        int shapeId = body.headShapeId;
                        while (shapeId != -1)
                        {
                            Shape shape = shapes[shapeId];
                            Debug.Assert(shape.id == shapeId);
                            Debug.Assert(shape.prevShapeId == prevShapeId);
                            switch ((SetType)setIndex)
                            {
                                case SetType.Static: Debug.Assert(B2_PROXY_TYPE(shape.proxyKey) == BodyType.Static); break;
                                case SetType.Disabled: Debug.Assert(shape.proxyKey == -1); break;
                                default:
                                    BodyType proxyType = B2_PROXY_TYPE(shape.proxyKey);
                                    Debug.Assert(proxyType == BodyType.Kinematic || proxyType == BodyType.Dynamic);
                                    break;
                            }
                            prevShapeId = shapeId;
                            shapeId = shape.nextShapeId;
                        }
                        int contactKey = body.headContactKey;
                        while (contactKey != -1)
                        {
                            int contactId = contactKey >> 1;
                            int edgeIndex = contactKey & 1;
                            Contact contact = contacts[contactId];
                            Debug.Assert(contact.setIndex != (int)SetType.Static);
                            Debug.Assert(contact.edge0.bodyId == bodyId || contact.edge1.bodyId == bodyId);
                            contactKey = edgeIndex == 1 ? contact.edge1.nextKey : contact.edge0.nextKey;
                        }
                        int jointKey = body.headJointKey;
                        while (jointKey != -1)
                        {
                            int jointId = jointKey >> 1;
                            int edgeIndex = jointKey & 1;
                            Joint joint = joints[jointId];
                            int otherEdgeIndex = edgeIndex ^ 1;
                            Body otherBody = bodies[otherEdgeIndex == 1 ? joint.edge1.bodyId : joint.edge0.bodyId];
                            if (setIndex == (int)SetType.Disabled || otherBody.setIndex == (int)SetType.Disabled)
                                Debug.Assert(joint.setIndex == (int)SetType.Disabled);
                            else if (setIndex == (int)SetType.Static && otherBody.setIndex == (int)SetType.Static)
                                Debug.Assert(joint.setIndex == (int)SetType.Static);
                            else if (body.type != BodyType.Dynamic && otherBody.type != BodyType.Dynamic)
                                Debug.Assert(joint.setIndex == (int)SetType.Static);
                            else if (setIndex == (int)SetType.Awake)
                                Debug.Assert(joint.setIndex == (int)SetType.Awake);
                            else if (setIndex >= (int)SetType.FirstSleeping)
                                Debug.Assert(joint.setIndex == setIndex);
                            JointSim jointSim = GetJointSim(joint);
                            Debug.Assert(jointSim.jointId == jointId);
                            Debug.Assert(jointSim.bodyIdA == joint.edge0.bodyId);
                            Debug.Assert(jointSim.bodyIdB == joint.edge1.bodyId);
                            jointKey = edgeIndex == 1 ? joint.edge1.nextKey : joint.edge0.nextKey;
                        }
                    }
                }
                {
                    Debug.Assert(set.contactSims.Count >= 0);
                    totalContactCount += set.contactSims.Count;
                    for (int i = 0; i < set.contactSims.Count; i++)
                    {
                        ContactSim contactSim = set.contactSims[i];
                        Contact contact = contacts[contactSim.contactId];
                        if (setIndex == (int)SetType.Awake)
                            Debug.Assert(contactSim.manifold.pointCount == 0 ||
                                contactSim.simFlags.HasFlag(ContactFlags.SimStartedTouching));
                        Debug.Assert(contact.setIndex == setIndex);
                        Debug.Assert(contact.colorIndex == -1);
                        Debug.Assert(contact.localIndex == i);
                        Debug.Assert(contactSim.encodedBodySimA == bodies[contact.edge0.bodyId].EncodeBodySimIndex());
                        Debug.Assert(contactSim.encodedBodySimB == bodies[contact.edge1.bodyId].EncodeBodySimIndex());
                    }
                }
                {
                    Debug.Assert(set.jointSims.Count >= 0);
                    totalJointCount += set.jointSims.Count;
                    for (int i = 0; i < set.jointSims.Count; i++)
                    {
                        JointSim jointSim = set.jointSims[i];
                        Joint joint = joints[jointSim.jointId];
                        Debug.Assert(joint.setIndex == setIndex);
                        Debug.Assert(joint.colorIndex == -1);
                        Debug.Assert(joint.localIndex == i);
                    }
                }
                {
                    Debug.Assert(set.islandSims.Count >= 0);
                    totalIslandCount += set.islandSims.Count;
                    for (int i = 0; i < set.islandSims.Count; i++)
                    {
                        IslandSim islandSim = set.islandSims[i];
                        Island island = islands[islandSim.islandId];
                        Debug.Assert(island.setIndex == setIndex);
                        Debug.Assert(island.localIndex == i);
                    }
                }
            }
            else
            {
                Debug.Assert(set.bodySims.Count == 0);
                Debug.Assert(set.contactSims.Count == 0);
                Debug.Assert(set.jointSims.Count == 0);
                Debug.Assert(set.islandSims.Count == 0);
                Debug.Assert(set.bodyStates.Count == 0);
            }
        }
        Debug.Assert(activeSetCount == solverSetIdPool.GetIdCount());
        Debug.Assert(totalBodyCount == bodyIdPool.GetIdCount());
        Debug.Assert(totalIslandCount == islandIdPool.GetIdCount());
        for (int colorIndex = 0; colorIndex < Box2D.GraphColorCount; colorIndex++)
        {
            GraphColor color = constraintGraph.colors[colorIndex];
            {
                int bitCount = 0;
                Debug.Assert(color.contactSims.Count >= 0);
                totalContactCount += color.contactSims.Count;
                for (int i = 0; i < color.contactSims.Count; i++)
                {
                    ContactSim contactSim = color.contactSims[i];
                    Contact contact = contacts[contactSim.contactId];
                    Debug.Assert(contactSim.manifold.pointCount > 0 ||
                        contactSim.simFlags.HasFlag(ContactFlags.SimStoppedTouching) ||
                        contactSim.simFlags.HasFlag(ContactFlags.SimDisjoint));
                    Debug.Assert(contact.setIndex == (int)SetType.Awake);
                    Debug.Assert(contact.colorIndex == colorIndex);
                    Debug.Assert(contact.localIndex == i);
                    int bodyIdA = contact.edge0.bodyId, bodyIdB = contact.edge1.bodyId;
                    Body edgeBodyA = bodies[bodyIdA], edgeBodyB = bodies[bodyIdB];
                    Debug.Assert(contactSim.encodedBodySimA == edgeBodyA.EncodeBodySimIndex());
                    Debug.Assert(contactSim.encodedBodySimB == edgeBodyB.EncodeBodySimIndex());
                    if (colorIndex < Box2D.GraphColorCount - 1)
                    {
                        Debug.Assert(color.bodySet.GetBit(bodyIdA) == (edgeBodyA.type == BodyType.Dynamic));
                        Debug.Assert(color.bodySet.GetBit(bodyIdB) == (edgeBodyB.type == BodyType.Dynamic));
                        bitCount += edgeBodyA.type == BodyType.Dynamic ? 1 : 0;
                        bitCount += edgeBodyB.type == BodyType.Dynamic ? 1 : 0;
                    }
                }
                Debug.Assert(color.jointSims.Count >= 0);
                totalJointCount += color.jointSims.Count;
                for (int i = 0; i < color.jointSims.Count; i++)
                {
                    JointSim jointSim = color.jointSims[i];
                    Joint joint = joints[jointSim.jointId];
                    Debug.Assert(joint.setIndex == (int)SetType.Awake);
                    Debug.Assert(joint.colorIndex == colorIndex);
                    Debug.Assert(joint.localIndex == i);
                    int bodyIdA = joint.edge0.bodyId, bodyIdB = joint.edge1.bodyId;
                    if (colorIndex < Box2D.GraphColorCount - 1)
                    {
                        Body bodyA = bodies[bodyIdA], bodyB = bodies[bodyIdB];
                        Debug.Assert(color.bodySet.GetBit(bodyIdA) == (bodyA.type == BodyType.Dynamic));
                        Debug.Assert(color.bodySet.GetBit(bodyIdB) == (bodyB.type == BodyType.Dynamic));
                        bitCount += bodyA.type == BodyType.Static ? 1 : 0;
                        bitCount += bodyB.type == BodyType.Static ? 1 : 0;
                    }
                }
                Debug.Assert(bitCount == color.bodySet.CountSetBits());
            }
        }
        Debug.Assert(totalContactCount == contactIdPool.GetIdCount());
        Debug.Assert(totalContactCount == broadPhase.pairSet.Count);
        Debug.Assert(totalJointCount == jointIdPool.GetIdCount());
#endif
    }
    public void ValidateContacts()
    {
#if B2_VALIDATE
        int contactCount = contacts.Count;
        Debug.Assert(contactCount == contactIdPool.GetIdCapacity());
        int allocatedContactCount = 0;
        for (int contactIndex = 0; contactIndex < contactCount; contactIndex++)
        {
            Contact contact = contacts[contactIndex];
            if (contact.contactId == -1) continue;
            Debug.Assert(contact.contactId == contactIndex);
            allocatedContactCount++;
            bool touching = ((ContactFlags)contact.flags).HasFlag(ContactFlags.Touching);
            int setId = contact.setIndex;
            if (setId == (int)SetType.Awake)
            {
                if (touching) Debug.Assert(0 <= contact.colorIndex && contact.colorIndex < Box2D.GraphColorCount);
                else Debug.Assert(contact.colorIndex == -1);
            }
            else if (setId == (int)SetType.FirstSleeping) Debug.Assert(touching);
            else Debug.Assert(!touching && setId == (int)SetType.Disabled);
            ContactSim contactSim = GetContactSim(contact);
            Debug.Assert(contactSim.contactId == contactIndex);
            Debug.Assert(contactSim.bodyIdA == contact.edge0.bodyId);
            Debug.Assert(contactSim.bodyIdB == contact.edge1.bodyId);
            Debug.Assert(touching == ((ContactSimFlags)contactSim.simFlags).HasFlag(ContactSimFlags.Touching));
            Debug.Assert(0 <= contactSim.manifold.pointCount && contactSim.manifold.pointCount <= 2);
        }
        Debug.Assert(allocatedContactCount == contactIdPool.GetIdCount());
#endif
    }
    public unsafe ulong HashWorldStateDeep()
    {
        ulong hash = 14695981039346656037ul;
        for (int i = 0; i < bodies.Count; i++)
        {
            Body body = bodies[i];
            if (body.id != i) continue;
            BodySim sim = GetBodySim(body);
            hash = Box2D.FnvMixPosition(hash, sim.transform.p);
            hash = Box2D.FnvMixFloat(hash, sim.transform.q.c);
            hash = Box2D.FnvMixFloat(hash, sim.transform.q.s);
            BodyState* state = GetBodyState(body);
            if (state != null)
            {
                hash = Box2D.FnvMixFloat(hash, state->linearVelocity.x);
                hash = Box2D.FnvMixFloat(hash, state->linearVelocity.y);
                hash = Box2D.FnvMixFloat(hash, state->angularVelocity);
            }
            hash = Box2D.FnvMixInt(hash, body.setIndex);
            hash = Box2D.FnvMixInt(hash, body.localIndex);
        }
        for (int i = 0; i < contacts.Count; i++)
        {
            Contact contact = contacts[i];
            if (contact.contactId != i) continue;
            hash = Box2D.FnvMixInt(hash, contact.setIndex);
            hash = Box2D.FnvMixInt(hash, contact.colorIndex);
            hash = Box2D.FnvMixInt(hash, contact.localIndex);
            ContactSim sim = GetContactSim(contact);
            if (sim != null)
            {
                ref Manifold m = ref sim.manifold;
                hash = Box2D.FnvMixInt(hash, m.pointCount);
                if (m.pointCount > 0)
                {
                    hash = Box2D.FnvMixFloat(hash, m.point0.normalImpulse);
                    hash = Box2D.FnvMixFloat(hash, m.point0.tangentImpulse);
                    hash = Box2D.FnvMixFloat(hash, m.point0.totalNormalImpulse);
                }
                if (m.pointCount > 1)
                {
                    hash = Box2D.FnvMixFloat(hash, m.point1.normalImpulse);
                    hash = Box2D.FnvMixFloat(hash, m.point1.tangentImpulse);
                    hash = Box2D.FnvMixFloat(hash, m.point1.totalNormalImpulse);
                }
            }
        }
        for (int i = 0; i < joints.Count; i++)
        {
            Joint joint = joints[i];
            if (joint.jointId != i) continue;
            hash = Box2D.FnvMixInt(hash, joint.setIndex);
            hash = Box2D.FnvMixInt(hash, joint.colorIndex);
            hash = Box2D.FnvMixInt(hash, joint.localIndex);
            JointSim sim = GetJointSim(joint);
            sim?.joint.HashStateDeep(ref hash);
        }
        hash = Box2D.FnvMixInt(hash, bodyIdPool.GetIdCapacity());
        hash = Box2D.FnvMixInt(hash, bodyIdPool.GetIdCount());
        hash = Box2D.FnvMixInt(hash, shapeIdPool.GetIdCapacity());
        hash = Box2D.FnvMixInt(hash, shapeIdPool.GetIdCount());
        hash = Box2D.FnvMixInt(hash, chainIdPool.GetIdCapacity());
        hash = Box2D.FnvMixInt(hash, chainIdPool.GetIdCount());
        hash = Box2D.FnvMixInt(hash, contactIdPool.GetIdCapacity());
        hash = Box2D.FnvMixInt(hash, contactIdPool.GetIdCount());
        hash = Box2D.FnvMixInt(hash, jointIdPool.GetIdCapacity());
        hash = Box2D.FnvMixInt(hash, jointIdPool.GetIdCount());
        hash = Box2D.FnvMixInt(hash, islandIdPool.GetIdCapacity());
        hash = Box2D.FnvMixInt(hash, islandIdPool.GetIdCount());
        hash = Box2D.FnvMixInt(hash, solverSetIdPool.GetIdCapacity());
        hash = Box2D.FnvMixInt(hash, solverSetIdPool.GetIdCount());
        hash = Box2D.FnvMixInt(hash, solverSets.Count);
        return hash;
    }
}
public static partial class Box2D
{
    const ulong SnapFnvPrime = 1099511628211ul;
    public static unsafe ulong FnvMixPosition(ulong hash, Position p)
    {
#if BOX2D_DOUBLE_PRECISION
        ulong bx = *(ulong*)&p.x, by = *(ulong*)&p.y;
#else
        uint bx = *(uint*)&p.x, by = *(uint*)&p.y;
#endif
        hash = (hash ^ bx) * SnapFnvPrime;
        hash = (hash ^ by) * SnapFnvPrime;
        return hash;
    }
    public static unsafe ulong FnvMixBytes(ulong hash, nint data, int n)
    {
        for (int i = 0; i < n; i++) hash = (hash ^ ((byte*)data)[i]) * SnapFnvPrime;
        return hash;
    }
    public static unsafe ulong FnvMixFloat(ulong hash, float f) => (hash ^ *(uint*)&f) * SnapFnvPrime;
    public static ulong FnvMixInt(ulong hash, int v) => (hash ^ (uint)v) * SnapFnvPrime;
}
public partial class DebugDraw
{
    public void DrawShape(Shape shape, WorldTransform transform, HexColor color, bool drawChainNormals)
    {
        switch (shape.type)
        {
            case ShapeType.Capsule:
                {
                    Capsule capsule = (Capsule)shape.shape;
                    Position p1 = transform.TransformWorldPoint(capsule.center1), p2 = transform.TransformWorldPoint(capsule.center2);
                    DrawSolidCapsuleFcn(p1, p2, capsule.radius, color, context);
                    break;
                }
            case ShapeType.Circle:
                Circle circle = (Circle)shape.shape;
                DrawSolidCircleFcn(transform, circle.center, circle.radius, color, context);
                break;
            case ShapeType.Polygon:
                Polygon poly = (Polygon)shape.shape;
                DrawSolidPolygonFcn(transform, poly.vertices, poly.radius, color, context);
                break;
            case ShapeType.Segment:
                {
                    Segment segment = (Segment)shape.shape;
                    Position p1 = transform.TransformWorldPoint(segment.point1), p2 = transform.TransformWorldPoint(segment.point2);
                    DrawLineFcn(p1, p2, color, context);
                    break;
                }
            case ShapeType.ChainSegment:
                {
                    Segment segment = ((ChainSegment)shape.shape).segment;
                    Position p1 = transform.TransformWorldPoint(segment.point1), p2 = transform.TransformWorldPoint(segment.point2);
                    DrawLineFcn(p1, p2, color, context);
                    DrawPointFcn(p2, 4, color, context);
                    if (drawChainNormals)
                    {
                        Vector2 c = Vector2.Lerp(p1, p2, 0.5f);
                        DrawLineFcn(c, c + 0.2f * Box2D.LengthUnitsPerMeter * (p2 - p1).Normalize().RightPerp(), HexColor.PaleGreen, context);
                    }
                    break;
                }
            default:
                break;
        }
    }
    public struct DrawContext
    {
        public World world;
        public DebugDraw draw;
    }
    public static bool DrawQueryCallback(int proxyId, ulong userData, object context)
    {
        int shapeId = (int)userData;
        DrawContext drawContext = (DrawContext)context;
        World world = drawContext.world;
        DebugDraw draw = drawContext.draw;
        Shape shape = world.shapes[shapeId];
        Debug.Assert(shape.id == shapeId);
        world.debugBodySet.SetBit(shape.bodyId);
        Body body = world.bodies[shape.bodyId];
        BodySim bodySim = world.GetBodySim(body);
        if (draw.drawShapes)
        {
            HexColor color;
            if (shape.material.customColor != 0) color = (HexColor)shape.material.customColor;
            else if (body.type == BodyType.Dynamic && body.mass == 0) color = HexColor.Red;
            else if (body.setIndex == (int)SetType.Disabled) color = HexColor.SlateGray;
            else if (shape.sensorIndex != -1) color = HexColor.Wheat;
            else if (body.flags.HasFlag(BodyFlags.HadTimeOfImpact)) color = HexColor.Lime;
            else if (bodySim.flags.HasFlag(BodyFlags.IsBullet) && body.setIndex == (int)SetType.Awake) color = HexColor.Turquoise;
            else if (body.flags.HasFlag(BodyFlags.IsSpeedCapped)) color = HexColor.Yellow;
            else if (body.flags.HasFlag(BodyFlags.IsFast)) color = HexColor.Salmon;
            else if (body.type == BodyType.Static) color = HexColor.PaleGreen;
            else if (body.type == BodyType.Kinematic) color = HexColor.RoyalBlue;
            else if (body.setIndex == (int)SetType.Awake) color = HexColor.Pink;
            else color = HexColor.Gray;
            draw.DrawShape(shape, bodySim.transform, color, draw.drawChainNormals);
        }
        if (draw.drawBounds)
        {
            draw.DrawBoundsFcn(world.fatAABBs[shapeId], HexColor.Gold, draw.context);
        }
        return true;
    }
}
