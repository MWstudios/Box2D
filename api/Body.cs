using System;
using System.Diagnostics;

namespace Box2D.API;

public unsafe static class BodyAPI
{
    /// <summary>Create a rigid body given a definition. No reference to the definition is retained. So you can create the definition
    /// on the stack and pass it as a pointer.
    /// <code>
    /// BodyDef bodyDef = DefaultBodyDef();
    /// BodyID myBodyID = CreateBody(myWorldId, &bodyDef);
    /// </code>
    /// @warning This function is locked during callbacks.</summary>
    public static BodyID CreateBody(WorldID worldId, ref BodyDef def)
    {
        Debug.Assert(def.internalValue == Box2D.SECRET_COOKIE);
        Debug.Assert(def.position.IsValid());
        Debug.Assert(def.rotation.IsValid());
        Debug.Assert(def.linearVelocity.IsValid());
        Debug.Assert(float.IsFinite(def.angularVelocity));
        Debug.Assert(float.IsFinite(def.linearDamping) && def.linearDamping >= 0);
        Debug.Assert(float.IsFinite(def.angularVelocity) && def.angularDamping >= 0);
        Debug.Assert(float.IsFinite(def.sleepThreshold) && def.sleepThreshold >= 0);
        Debug.Assert(float.IsFinite(def.gravityScale));
        World world = worldId.index1; Debug.Assert(!world.locked); if (world.locked) return new();
        bool isAwake = (def.isAwake || !def.enableSleep) && def.isEnabled;
        int setId;
        if (!def.isEnabled) setId = (int)SetType.Disabled;
        else if (def.type == BodyType.Static) setId = (int)SetType.Static;
        else if (isAwake) setId = (int)SetType.Awake;
        else
        {
            setId = world.solverSetIdPool.AllocId();
            if (setId == world.solverSets.Count) world.solverSets.Add(new());
            else Debug.Assert(world.solverSets[setId].setIndex == -1);
            world.solverSets[setId].setIndex = setId;
        }
        Debug.Assert(0 <= setId && setId < world.solverSets.Count);
        int bodyId = world.bodyIdPool.AllocId();
        BodyFlags lockFlags = (def.motionLocks.linearX ? BodyFlags.LockLinearX : 0)
            | (def.motionLocks.linearY ? BodyFlags.LockLinearY : 0)
            | (def.motionLocks.angularZ ? BodyFlags.LockAngularZ : 0)
            | (def.isBullet ? BodyFlags.IsBullet : 0)
            | (def.allowFastRotation ? BodyFlags.AllowFastRotation : 0);
        SolverSet set = world.solverSets[setId];
        BodySim bodySim = new()
        {
            transform = new(def.position, def.rotation),
            center = def.position,
            rotation0 = def.rotation,
            center0 = def.position,
            minExtent = Box2D.Huge,
            maxExtent = 0,
            linearDamping = def.linearDamping,
            angularDamping = def.angularDamping,
            gravityScale = def.gravityScale,
            bodyId = bodyId,
            flags = lockFlags | (def.isBullet ? BodyFlags.IsBullet : 0)
            | (def.allowFastRotation ? BodyFlags.AllowFastRotation : 0)
            | (def.type == BodyType.Dynamic ? BodyFlags.Dynamic : 0),
        };
        set.bodySims.Add(bodySim);
        if (setId == (int)SetType.Awake)
        {
            set.bodyStates.Add(new()
            {
                linearVelocity = def.linearVelocity,
                angularVelocity = def.angularVelocity,
                deltaRotation = Rotation.Identity,
                flags = bodySim.flags,
            });
        }
        if (bodyId == world.bodies.Count) world.bodies.Add(new());
        else Debug.Assert(world.bodies[bodyId].id == -1);
        Body body = world.bodies[bodyId];
        if (def.name != null) body.name = def.name;
        body.userData = def.userData;
        body.setIndex = setId;
        body.localIndex = set.bodySims.Count - 1;
        body.generation++;
        body.headShapeId = -1;
        body.shapeCount = 0;
        body.headChainId = -1;
        body.headContactKey = -1;
        body.contactCount = 0;
        body.headJointKey = -1;
        body.jointCount = 0;
        body.islandId = -1;
        body.islandPrev = -1;
        body.islandNext = -1;
        body.bodyMoveIndex = -1;
        body.id = bodyId;
        body.mass = 0;
        body.inertia = 0;
        body.sleepThreshold = def.sleepThreshold;
        body.sleepTime = 0;
        body.type = def.type;
        body.flags = bodySim.flags;
        body.enableSleep = def.enableSleep;
        if (setId >= (int)SetType.Awake) world.CreateIslandForBody(setId, body);
        world.ValidateSolverSets();
        return new() { index1 = bodyId + 1, world0 = world, generation = body.generation };
    }

    ///<summary>Destroy a rigid body given an id. This destroys all shapes and joints attached to the body.
    /// Do not keep references to the associated shapes and joints.</summary>
    public static void DestroyBody(BodyID bodyId)
    {
        World world = World.GetWorldLocked(bodyId.world0); if (world == null) return;
        Body body = world.GetBodyFullID(bodyId);
        int edgeKey = body.headJointKey;
        while (edgeKey != -1)
        {
            int jointId = edgeKey >> 1;
            int edgeIndex = edgeKey & 1;
            Joint joint = world.joints[jointId];
            edgeKey = edgeIndex == 1 ? joint.edge1.nextKey : joint.edge0.nextKey;
            world.DestroyJointInternal(joint, true);
        }
        int shapeId = body.headShapeId;
        while (shapeId != -1)
        {
            Shape shape = world.shapes[shapeId];
            if (shape.sensorIndex != -1) world.DestroySensor(shape);
            shape.DestroyProxy(world.broadPhase);
            world.shapeIdPool.FreeId(shapeId);
            shape.id = -1;
            shapeId = shape.nextShapeId;
        }
        int chainId = body.headChainId;
        while (chainId != -1)
        {
            ChainShape chain = world.chainShapes[chainId];
            world.chainIdPool.FreeId(chainId);
            chain.id = -1;
            chainId = chain.nextChainId;
        }
        world.RemoveBodyFromIsland(body);
        SolverSet set = world.solverSets[body.setIndex];
        int movedIndex = set.bodySims.RemoveSwap(body.localIndex);
        if (movedIndex != -1)
        {
            BodySim movedSim = set.bodySims[body.localIndex];
            int movedId = movedSim.bodyId;
            Body movedBody = world.bodies[movedId];
            Debug.Assert(movedBody.localIndex == movedIndex);
            movedBody.localIndex = body.localIndex;
        }
        if (body.setIndex == (int)SetType.Awake)
        {
            int result = set.bodyStates.RemoveSwap(body.localIndex);
            Debug.Assert(result == movedIndex);
        }
        else if (set.setIndex >= (int)SetType.FirstSleeping && set.bodySims.Count == 0)
            world.DestroySolverSet(set.setIndex);
        world.bodyIdPool.FreeId(body.id);
        body.setIndex = -1;
        body.localIndex = -1;
        body.id = -1;
        world.ValidateSolverSets();
    }

    ///<summary> Body identifier validation. Can be used to detect orphaned ids. Provides validation for up to 64K allocations.
    ///This can be used to detect orphaned ids. Provides validation for up to 64K allocations.</summary>
    public static bool Body_IsValid(BodyID id)
    {
        World world = id.world0;
        if (world == null) return false;
        if (id.index1 < 1 || world.bodies.Count < id.index1) return false;
        Body body = world.bodies[id.index1 - 1];
        if (body.setIndex == -1) return false;
        Debug.Assert(body.localIndex != -1);
        if (body.generation != id.generation) return false;
        return true;
    }

    ///<summary> Get the body type: static, kinematic, or dynamic</summary>
    public static BodyType Body_GetType(BodyID bodyId) => bodyId.world0.GetBodyFullID(bodyId).type;

    ///<summary>Change the body type. This is an expensive operation. This automatically updates the mass
    /// properties regardless of the automatic mass setting.</summary>
    public static void Body_SetType(BodyID bodyId, BodyType type)
    {
        World world = bodyId.world0; Body body = world.GetBodyFullID(bodyId);
        BodyType originalType = body.type;
        if (originalType == type) return;
        if (body.setIndex == (int)SetType.Disabled)
        {
            body.type = type;
            if (type == BodyType.Dynamic) body.flags |= BodyFlags.Dynamic;
            else body.flags &= ~BodyFlags.Dynamic;
            world.UpdateBodyMassData(body);
            return;
        }
        world.DestroyBodyContacts(body, false);
        world.WakeBody(body);
        SolverSet staticSet = world.solverSets[(int)SetType.Static];
        int jointKey = body.headJointKey;
        while (jointKey != -1)
        {
            int jointId = jointKey >> 1;
            int edgeIndex = jointKey & 1;
            Joint joint = world.joints[jointId];
            jointKey = edgeIndex == 1 ? joint.edge1.nextKey : joint.edge0.nextKey;
            if (joint.setIndex == (int)SetType.Disabled) continue;
            Body bodyA = world.bodies[joint.edge0.bodyId], bodyB = world.bodies[joint.edge1.bodyId];
            world.WakeBody(bodyA);
            world.WakeBody(bodyB);
            world.UnlinkJoint(joint);
            SolverSet jointSourceSet = world.solverSets[joint.setIndex];
            world.TransferJoint(staticSet, jointSourceSet, joint);
        }
        body.type = type;
        if (type == BodyType.Dynamic) body.flags |= BodyFlags.Dynamic;
        else body.flags &= ~BodyFlags.Dynamic;
        SolverSet awakeSet = world.solverSets[(int)SetType.Awake];
        SolverSet sourceSet = world.solverSets[body.setIndex];
        SolverSet targetSet = type == BodyType.Static ? staticSet : awakeSet;
        world.TransferBody(targetSet, sourceSet, body);
        if (originalType == BodyType.Static) world.CreateIslandForBody((int)SetType.Awake, body);
        else if (type == BodyType.Static) world.RemoveBodyFromIsland(body);
        jointKey = body.headJointKey;
        while (jointKey != -1)
        {
            int jointId = jointKey >> 1;
            int edgeIndex = jointKey & 1;
            Joint joint = world.joints[jointId];
            jointKey = edgeIndex == 1 ? joint.edge1.nextKey : joint.edge0.nextKey;
            if (joint.setIndex == (int)SetType.Disabled) continue;
            Debug.Assert(joint.setIndex == (int)SetType.Static);
            Body bodyA = world.bodies[joint.edge0.bodyId], bodyB = world.bodies[joint.edge1.bodyId];
            Debug.Assert(bodyA.setIndex == (int)SetType.Static || bodyA.setIndex == (int)SetType.Awake);
            Debug.Assert(bodyB.setIndex == (int)SetType.Static || bodyB.setIndex == (int)SetType.Awake);
            if (bodyA.type == BodyType.Dynamic || bodyB.type == BodyType.Dynamic)
                world.TransferJoint(awakeSet, staticSet, joint);
        }
        Transform transform = world.GetBodyTransformQuick(body);
        int shapeId = body.headShapeId;
        while (shapeId != -1)
        {
            Shape shape = world.shapes[shapeId];
            shapeId = shape.nextShapeId;
            shape.DestroyProxy(world.broadPhase);
            shape.CreateProxy(world.broadPhase, type, transform, true);
        }
        jointKey = body.headJointKey;
        while (jointKey != -1)
        {
            int jointId = jointKey >> 1;
            int edgeIndex = jointKey & 1;
            Joint joint = world.joints[jointId];
            jointKey = edgeIndex == 1 ? joint.edge1.nextKey : joint.edge0.nextKey;
            int otherEdgeIndex = edgeIndex ^ 1;
            Body otherBody = world.bodies[otherEdgeIndex == 1 ? joint.edge1.bodyId : joint.edge0.bodyId];
            if (otherBody.setIndex == (int)SetType.Disabled) continue;
            if (body.type != BodyType.Dynamic && otherBody.type != BodyType.Dynamic) continue;
            world.LinkJoint(joint);
        }
        world.UpdateBodyMassData(body);
        BodyState* state = world.GetBodyState(body);
        if (state != null) state->flags = body.flags;
        world.ValidateSolverSets();
        world.ValidateIsland(body.islandId);
    }

    ///<summary> Set the body name. Up to 31 characters excluding 0 termination.</summary>
    public static void Body_SetName(BodyID bodyId, string name) => bodyId.world0.GetBodyFullID(bodyId).name = name;

    ///<summary> Get the body name.</summary>
    public static string Body_GetName(BodyID bodyId) => bodyId.world0.GetBodyFullID(bodyId).name;

    ///<summary> Set the user data for a body</summary>
    public static void Body_SetUserData(BodyID bodyId, object userData) => bodyId.world0.GetBodyFullID(bodyId).userData = userData;

    ///<summary> Get the user data stored in a body</summary>
    public static object Body_GetUserData(BodyID bodyId) => bodyId.world0.GetBodyFullID(bodyId).userData;

    ///<summary> Get the world position of a body. This is the location of the body origin.</summary>
    public static Vector2 Body_GetPosition(BodyID bodyId) => bodyId.world0.GetBodyTransformQuick(bodyId.world0.GetBodyFullID(bodyId)).p;

    ///<summary> Get the world rotation of a body as a cosine/sine pair (complex number)</summary>
    public static Rotation Body_GetRotation(BodyID bodyId) => bodyId.world0.GetBodyTransformQuick(bodyId.world0.GetBodyFullID(bodyId)).q;

    ///<summary> Get the world transform of a body.</summary>
    public static Transform Body_GetTransform(BodyID bodyId) => bodyId.world0.GetBodyTransformQuick(bodyId.world0.GetBodyFullID(bodyId));

    /// <summary>Set the world transform of a body. This acts as a teleport and is fairly expensive.
    /// @see BodyDef::position and BodyDef::rotation</summary>
    /// <remarks>Generally you should create a body with then intended transform.</remarks>
    public static void Body_SetTransform(BodyID bodyId, Vector2 position, Rotation rotation)
    {
        Debug.Assert(position.IsValid());
        Debug.Assert(rotation.IsValid());
        Debug.Assert(Body_IsValid(bodyId));
        World world = bodyId.world0; Debug.Assert(!world.locked);
        Body body = world.GetBodyFullID(bodyId);
        BodySim bodySim = world.GetBodySim(body);
        bodySim.transform = new(position, rotation);
        bodySim.center = bodySim.transform.TransformPoint(bodySim.localCenter);
        bodySim.rotation0 = bodySim.transform.q;
        bodySim.center0 = bodySim.center;
        BroadPhase broadPhase = world.broadPhase;
        Transform transform = bodySim.transform;
        int shapeId = body.headShapeId;
        while (shapeId != -1)
        {
            Shape shape = world.shapes[shapeId];
            AABB aabb = shape.ComputeAABB(transform);
            aabb.lowerBound.x -= Box2D.SpeculativeDistance;
            aabb.lowerBound.y -= Box2D.SpeculativeDistance;
            aabb.upperBound.x += Box2D.SpeculativeDistance;
            aabb.upperBound.y += Box2D.SpeculativeDistance;
            shape.aabb = aabb;
            if (!shape.fatAABB.Contains(aabb))
            {
                AABB fatAABB = new(new(aabb.lowerBound.x - Box2D.AABBMargin, aabb.lowerBound.y - Box2D.AABBMargin),
                    new(aabb.upperBound.x + Box2D.AABBMargin, aabb.upperBound.y + Box2D.AABBMargin));
                shape.fatAABB = fatAABB;
                if (shape.proxyKey != -1) broadPhase.MoveProxy(shape.proxyKey, fatAABB);
            }
            shapeId = shape.nextShapeId;
        }
    }

    ///<summary> Get a local point on a body given a world point</summary>
    public static Vector2 Body_GetLocalPoint(BodyID bodyId, Vector2 worldPoint) =>
        bodyId.world0.GetBodyTransformQuick(bodyId.world0.GetBodyFullID(bodyId)).InvTransformPoint(worldPoint);

    ///<summary> Get a world point on a body given a local point</summary>
    public static Vector2 Body_GetWorldPoint(BodyID bodyId, Vector2 localPoint) =>
        bodyId.world0.GetBodyTransformQuick(bodyId.world0.GetBodyFullID(bodyId)).TransformPoint(localPoint);

    ///<summary> Get a local vector on a body given a world vector</summary>
    public static Vector2 Body_GetLocalVector(BodyID bodyId, Vector2 worldVector) =>
        bodyId.world0.GetBodyTransformQuick(bodyId.world0.GetBodyFullID(bodyId)).q.InvRotateVector(worldVector);

    ///<summary> Get a world vector on a body given a local vector</summary>
    public static Vector2 Body_GetWorldVector(BodyID bodyId, Vector2 localVector) =>
        bodyId.world0.GetBodyTransformQuick(bodyId.world0.GetBodyFullID(bodyId)).q * localVector;

    ///<summary> Get the linear velocity of a body's center of mass. Usually in meters per second.</summary>
    public static Vector2 Body_GetLinearVelocity(BodyID bodyId)
    {
        BodyState* state = bodyId.world0.GetBodyState(bodyId.world0.GetBodyFullID(bodyId));
        return state != null ? state->linearVelocity : Vector2.Zero;
    }

    ///<summary> Get the angular velocity of a body in radians per second</summary>
    public static float Body_GetAngularVelocity(BodyID bodyId)
    {
        BodyState* state = bodyId.world0.GetBodyState(bodyId.world0.GetBodyFullID(bodyId));
        return state != null ? state->angularVelocity : 0;
    }

    ///<summary> Set the linear velocity of a body. Usually in meters per second.</summary>
    public static void Body_SetLinearVelocity(BodyID bodyId, Vector2 linearVelocity)
    {
        World world = bodyId.world0; Body body = world.GetBodyFullID(bodyId);
        if (body.type == BodyType.Static) return;
        if (linearVelocity.LengthSquared() > 0) world.WakeBody(body);
        BodyState* state = world.GetBodyState(body);
        if (state != null) state->linearVelocity = linearVelocity;
    }

    ///<summary> Set the angular velocity of a body in radians per second</summary>
    public static void Body_SetAngularVelocity(BodyID bodyId, float angularVelocity)
    {
        World world = bodyId.world0; Body body = world.GetBodyFullID(bodyId);
        if (body.type == BodyType.Static) return;
        if (angularVelocity != 0) world.WakeBody(body);
        BodyState* state = world.GetBodyState(body);
        if (state != null) state->angularVelocity = angularVelocity;
    }

    ///<summary>Set the velocity to reach the given transform after a given time step.
    /// The result will be close but maybe not exact. This is meant for kinematic bodies.
    /// The target is not applied if the velocity would be below the sleep threshold.
    /// This will automatically wake the body if asleep.</summary>
    public static void Body_SetTargetTransform(BodyID bodyId, Transform target, float timeStep)
    {
        World world = bodyId.world0; Body body = world.GetBodyFullID(bodyId);
        if (body.setIndex == (int)SetType.Disabled) return;
        if (body.type == (int)SetType.Static || timeStep <= 0) return;
        BodySim sim = world.GetBodySim(body);
        Vector2 center1 = sim.center, center2 = target.TransformPoint(sim.localCenter);
        float invTimeStep = 1 / timeStep;
        Vector2 linearVelocity = invTimeStep * (center2 - center1);
        Rotation q1 = sim.transform.q, q2 = target.q;
        float deltaAngle = Rotation.RelativeAngle(q1, q2);
        float angularVelocity = invTimeStep * deltaAngle;
        if (body.setIndex != (int)SetType.Awake)
        {
            float maxVelocity = linearVelocity.Length() + Math.Abs(angularVelocity) * sim.maxExtent;
            if (maxVelocity < body.sleepThreshold) return;
            world.WakeBody(body);
        }
        Debug.Assert(body.setIndex == (int)SetType.Awake);
        BodyState* state = world.GetBodyState(body);
        if (state != null)
        {
            state->linearVelocity = linearVelocity;state->angularVelocity = angularVelocity;
        }
    }

    ///<summary> Get the linear velocity of a local point attached to a body. Usually in meters per second.</summary>
    public static Vector2 Body_GetLocalPointVelocity(BodyID bodyId, Vector2 localPoint)
    {
        World world = bodyId.world0; Body body = world.GetBodyFullID(bodyId);
        BodyState* state = world.GetBodyState(body);
        if (state == null) return Vector2.Zero;
        SolverSet set = world.solverSets[body.setIndex];
        BodySim bodySim = set.bodySims[body.localIndex];
        Vector2 r = bodySim.transform.q * (localPoint - bodySim.localCenter);
        return state->linearVelocity + Vector2.CrossSV(state->angularVelocity, r);
    }

    ///<summary> Get the linear velocity of a world point attached to a body. Usually in meters per second.</summary>
    public static Vector2 Body_GetWorldPointVelocity(BodyID bodyId, Vector2 worldPoint)
    {
        World world = bodyId.world0; Body body = world.GetBodyFullID(bodyId);
        BodyState* state = world.GetBodyState(body);
        if (state == null) return Vector2.Zero;
        SolverSet set = world.solverSets[body.setIndex];
        BodySim bodySim = set.bodySims[body.localIndex];
        Vector2 r = worldPoint - bodySim.center;
        return state->linearVelocity + Vector2.CrossSV(state->angularVelocity, r);
    }

    ///<summary> Apply a force at a world point. If the force is not applied at the center of mass,
    /// it will generate a torque and affect the angular velocity. This optionally wakes up the body.
    /// The force is ignored if the body is not awake.</summary>
    /// <param name="bodyID">The body id</param>
    /// <param name="force">The world force vector, usually in newtons (N)</param>
    /// <param name="point">The world position of the point of application</param>
    /// <param name="wake">Option to wake up the body</param>
    public static void Body_ApplyForce(BodyID bodyId, Vector2 force, Vector2 point, bool wake)
    {
        World world = bodyId.world0; Body body = world.GetBodyFullID(bodyId);
        if (body.type != BodyType.Dynamic || body.setIndex == (int)SetType.Disabled) return;
        if (wake && body.setIndex >= (int)SetType.FirstSleeping) world.WakeBody(body);
        if (body.setIndex == (int)SetType.Awake)
        {
            BodySim bodySim = world.GetBodySim(body);
            bodySim.force += force;
            bodySim.torque += Vector2.Cross(point - bodySim.center, force);
        }
    }

    ///<summary> Apply a force to the center of mass. This optionally wakes up the body.
    /// The force is ignored if the body is not awake.</summary>
    /// <param name="bodyID">The body id</param>
    /// <param name="force">the world force vector, usually in newtons (N).</param>
    /// <param name="wake">also wake up the body</param>
    public static void Body_ApplyForceToCenter(BodyID bodyId, Vector2 force, bool wake)
    {
        World world = bodyId.world0; Body body = world.GetBodyFullID(bodyId);
        if (body.type != BodyType.Dynamic || body.setIndex == (int)SetType.Disabled) return;
        if (wake && body.setIndex >= (int)SetType.FirstSleeping) world.WakeBody(body);
        if (body.setIndex == (int)SetType.Awake)
        {
            BodySim bodySim = world.GetBodySim(body);
            bodySim.force += force;
        }
    }

    /// <summary>Apply a torque. This affects the angular velocity without affecting the linear velocity.
    /// This optionally wakes the body. The torque is ignored if the body is not awake.</summary>
    /// <param name="bodyID">The body id</param>
    /// <param name="torque">about the z-axis (out of the screen), usually in ref Nm.</param>
    /// <param name="wake">also wake up the body</param>
    public static void Body_ApplyTorque(BodyID bodyId, float torque, bool wake)
    {
        World world = bodyId.world0; Body body = world.GetBodyFullID(bodyId);
        if (body.type != BodyType.Dynamic || body.setIndex == (int)SetType.Disabled) return;
        if (wake && body.setIndex >= (int)SetType.FirstSleeping) world.WakeBody(body);
        if (body.setIndex == (int)SetType.Awake)
        {
            BodySim bodySim = world.GetBodySim(body);
            bodySim.torque = torque;
        }
    }

    /// <summary>Apply an impulse at a point. This immediately modifies the velocity.
    /// It also modifies the angular velocity if the point of application
    /// is not at the center of mass. This optionally wakes the body.
    /// The impulse is ignored if the body is not awake.</summary>
    /// <param name="bodyID">The body id</param>
    /// <param name="impulse">the world impulse vector, usually in ref Ns or ref kgm/s.</param>
    /// <param name="point">the world position of the point of application.</param>
    /// <param name="wake">also wake up the body</param>
    /// <remarks>This should be used for one-shot impulses. If you need a steady force,
    /// use a force instead, which will work better with the sub-stepping solver.</remarks>
    public static void Body_ApplyLinearImpulse(BodyID bodyId, Vector2 impulse, Vector2 point, bool wake)
    {
        World world = bodyId.world0; Body body = world.GetBodyFullID(bodyId);
        if (body.type != BodyType.Dynamic || body.setIndex == (int)SetType.Disabled) return;
        if (wake && body.setIndex >= (int)SetType.FirstSleeping) world.WakeBody(body);
        if (body.setIndex == (int)SetType.Awake)
        {
            int localIndex = body.localIndex;
            SolverSet set = world.solverSets[(int)SetType.Awake];
            BodyState* state = set.bodyStates.Data + localIndex;
            BodySim bodySim = set.bodySims[localIndex];
            state->linearVelocity = Vector2.MulAdd(state->linearVelocity, bodySim.invMass, impulse);
            state->angularVelocity += bodySim.invInertia * Vector2.Cross(point - bodySim.center, impulse);
            state->LimitVelocity(world.maxLinearSpeed);
        }
    }

    /// <summary>Apply an impulse to the center of mass. This immediately modifies the velocity.
    /// The impulse is ignored if the body is not awake. This optionally wakes the body.</summary>
    /// <param name="bodyID">The body id</param>
    /// <param name="impulse">the world impulse vector, usually in ref Ns or ref kgm/s.</param>
    /// <param name="wake">also wake up the body</param>
    /// <remarks>This should be used for one-shot impulses. If you need a steady force,
    /// use a force instead, which will work better with the sub-stepping solver.</remarks>
    public static void Body_ApplyLinearImpulseToCenter(BodyID bodyId, Vector2 impulse, bool wake)
    {
        World world = bodyId.world0; Body body = world.GetBodyFullID(bodyId);
        if (body.type != BodyType.Dynamic || body.setIndex == (int)SetType.Disabled) return;
        if (wake && body.setIndex >= (int)SetType.FirstSleeping) world.WakeBody(body);
        if (body.setIndex == (int)SetType.Awake)
        {
            int localIndex = body.localIndex;
            SolverSet set = world.solverSets[(int)SetType.Awake];
            BodyState* state = set.bodyStates.Data + localIndex;
            BodySim bodySim = set.bodySims[localIndex];
            state->linearVelocity = Vector2.MulAdd(state->linearVelocity, bodySim.invMass, impulse);
            state->LimitVelocity(world.maxLinearSpeed);
        }
    }

    /// <summary>Apply an angular impulse. The impulse is ignored if the body is not awake.
    /// This optionally wakes the body.</summary>
    /// <param name="bodyID">The body id</param>
    /// <param name="impulse">the angular impulse, usually in units of ref kg*mm/s</param>
    /// <param name="wake">also wake up the body</param>
    /// <remarks>This should be used for one-shot impulses. If you need a steady torque,
    /// use a torque instead, which will work better with the sub-stepping solver.</remarks>
    public static void Body_ApplyAngularImpulse(BodyID bodyId, float impulse, bool wake)
    {
        Debug.Assert(Body_IsValid(bodyId));
        World world = bodyId.world0; Body body = world.GetBodyFullID(bodyId);
        if (body.type != BodyType.Dynamic || body.setIndex == (int)SetType.Disabled) return;
        if (wake && body.setIndex >= (int)SetType.FirstSleeping) world.WakeBody(body);
        if (body.setIndex == (int)SetType.Awake)
        {
            int localIndex = body.localIndex;
            SolverSet set = world.solverSets[(int)SetType.Awake];
            BodyState* state = set.bodyStates.Data + localIndex;
            BodySim bodySim = set.bodySims[localIndex];
            state->angularVelocity += bodySim.invInertia * impulse;
        }
    }

    ///<summary> Get the mass of the body, usually in kilograms</summary>
    public static float Body_GetMass(BodyID bodyId) => bodyId.world0.GetBodyFullID(bodyId).mass;

    ///<summary> Get the rotational inertia of the body, usually in ref kgm^2</summary>
    public static float Body_GetRotationalInertia(BodyID bodyId) => bodyId.world0.GetBodyFullID(bodyId).inertia;

    ///<summary> Get the center of mass position of the body in local space</summary>
    public static Vector2 Body_GetLocalCenterOfMass(BodyID bodyId) => bodyId.world0.GetBodySim(bodyId.world0.GetBodyFullID(bodyId)).localCenter;

    ///<summary> Get the center of mass position of the body in world space</summary>
    public static Vector2 Body_GetWorldCenterOfMass(BodyID bodyId) => bodyId.world0.GetBodySim(bodyId.world0.GetBodyFullID(bodyId)).center;

    ///<summary>Override the body's mass properties. Normally this is computed automatically using the
    /// shape geometry and density. This information is lost if a shape is added or removed or if the
    /// body type changes.</summary>
    public static void Body_SetMassData(BodyID bodyId, MassData massData)
    {
        Debug.Assert(float.IsFinite(massData.mass) && massData.mass >= 0);
        Debug.Assert(float.IsFinite(massData.rotationalInertia) && massData.rotationalInertia >= 0);
        Debug.Assert(massData.center.IsValid());
        World world = World.GetWorldLocked(bodyId.world0); if (world == null) return;
        Body body = world.GetBodyFullID(bodyId);
        BodySim bodySim = world.GetBodySim(body);
        body.mass = massData.mass;
        body.inertia = massData.rotationalInertia;
        bodySim.localCenter = massData.center;
        Vector2 center = bodySim.transform.TransformPoint(massData.center);
        bodySim.center = center;
        bodySim.center0 = center;
        bodySim.invMass = body.mass > 0 ? 1 / body.mass : 0;
        bodySim.invInertia = body.inertia > 0 ? 1 / body.inertia : 0;
    }

    ///<summary> Get the mass data for a body</summary>
    public static MassData Body_GetMassData(BodyID bodyId)
    {
        Body body = bodyId.world0.GetBodyFullID(bodyId);
        BodySim bodySim = bodyId.world0.GetBodySim(body);
        return new() { mass = body.mass, center = bodySim.localCenter, rotationalInertia = body.inertia };
    }

    ///<summary>This update the mass properties to the sum of the mass properties of the shapes.
    /// This normally does not need to be called unless you called SetMassData to override
    /// the mass and you later want to reset the mass.
    /// You may also use this when automatic mass computation has been disabled.
    /// You should call this regardless of body type.
    /// Note that sensor shapes may have mass.</summary>
    public static void Body_ApplyMassFromShapes(BodyID bodyId)
    {
        World world = World.GetWorldLocked(bodyId.world0); if (world == null) return;
        world.UpdateBodyMassData(world.GetBodyFullID(bodyId));
    }

    ///<summary> Adjust the linear damping. Normally this is set in BodyDef before creation.</summary>
    public static void Body_SetLinearDamping(BodyID bodyId, float linearDamping)
    {
        Debug.Assert(float.IsFinite(linearDamping) && linearDamping >= 0);
        World world = World.GetWorldLocked(bodyId.world0); if (world == null) return;
        world.GetBodySim(world.GetBodyFullID(bodyId)).linearDamping = linearDamping;
    }

    ///<summary> Get the current linear damping.</summary>
    public static float Body_GetLinearDamping(BodyID bodyId) => bodyId.world0.GetBodySim(bodyId.world0.GetBodyFullID(bodyId)).linearDamping;

    ///<summary> Adjust the angular damping. Normally this is set in BodyDef before creation.</summary>
    public static void Body_SetAngularDamping(BodyID bodyId, float angularDamping)
    {
        Debug.Assert(float.IsFinite(angularDamping) && angularDamping >= 0);
        World world = World.GetWorldLocked(bodyId.world0); if (world == null) return;
        world.GetBodySim(world.GetBodyFullID(bodyId)).angularDamping = angularDamping;
    }

    ///<summary> Get the current angular damping.</summary>
    public static float Body_GetAngularDamping(BodyID bodyId) => bodyId.world0.GetBodySim(bodyId.world0.GetBodyFullID(bodyId)).angularDamping;

    ///<summary>Adjust the gravity scale. Normally this is set in BodyDef before creation.
    /// @see BodyDef::gravityScale</summary>
    public static void Body_SetGravityScale(BodyID bodyId, float gravityScale)
    {
        Debug.Assert(Body_IsValid(bodyId));
        Debug.Assert(float.IsFinite(gravityScale));
        World world = World.GetWorldLocked(bodyId.world0); if (world == null) return;
        world.GetBodySim(world.GetBodyFullID(bodyId)).gravityScale = gravityScale;
    }

    ///<summary> Get the current gravity scale</summary>
    public static float Body_GetGravityScale(BodyID bodyId)
    {
        Debug.Assert(Body_IsValid(bodyId));
        return bodyId.world0.GetBodySim(bodyId.world0.GetBodyFullID(bodyId)).gravityScale;
    }

    ///<summary> <returns>true if this body is awake</returns></summary>
    public static bool Body_IsAwake(BodyID bodyId) => bodyId.world0.GetBodyFullID(bodyId).setIndex == (int)SetType.Awake;

    ///<summary>Wake a body from sleep. This wakes the entire island the body is touching.</summary>
    ///<remarks> Putting a body to sleep will put the entire island of bodies touching this body to sleep,
    /// which can be expensive and possibly unintuitive.</remarks>
    public static void Body_SetAwake(BodyID bodyId, bool awake)
    {
        World world = World.GetWorldLocked(bodyId.world0); if (world == null) return;
        Body body = world.GetBodyFullID(bodyId);
        if (awake && body.setIndex >= (int)SetType.FirstSleeping) world.WakeBody(body);
        else if (!awake && body.setIndex == (int)SetType.Awake)
        {
            Island island = world.islands[body.islandId];
            if (island.constraintRemoveCount > 0) world.SplitIsland(body.islandId);
            world.TrySleepIsland(body.islandId);
        }
    }

    ///<summary> Enable or disable sleeping for this body. If sleeping is disabled the body will wake.</summary>
    public static void Body_EnableSleep(BodyID bodyId, bool enableSleep)
    {
        World world = World.GetWorldLocked(bodyId.world0); if (world == null) return;
        Body body = world.GetBodyFullID(bodyId);
        body.enableSleep = enableSleep;
        if (!enableSleep) world.WakeBody(body);
    }

    ///<summary> Returns true if sleeping is enabled for this body</summary>
    public static bool Body_IsSleepEnabled(BodyID bodyId) => bodyId.world0.GetBodyFullID(bodyId).enableSleep;

    ///<summary> Set the sleep threshold, usually in meters per second</summary>
    public static void Body_SetSleepThreshold(BodyID bodyId, float sleepThreshold) => bodyId.world0.GetBodyFullID(bodyId).sleepThreshold = sleepThreshold;

    ///<summary> Get the sleep threshold, usually in meters per second.</summary>
    public static float Body_GetSleepThreshold(BodyID bodyId) => bodyId.world0.GetBodyFullID(bodyId).sleepThreshold;

    ///<summary> Returns true if this body is enabled</summary>
    public static bool Body_IsEnabled(BodyID bodyId) => bodyId.world0.GetBodyFullID(bodyId).setIndex != (int)SetType.Disabled;

    ///<summary> Disable a body by removing it completely from the simulation. This is expensive.</summary>
    public static void Body_Disable(BodyID bodyId)
    {
        World world = World.GetWorldLocked(bodyId.world0); if (world == null) return;
        Body body = world.GetBodyFullID(bodyId);
        if (body.setIndex == (int)SetType.Disabled) return;
        world.DestroyBodyContacts(body, true);
        SolverSet set = world.solverSets[body.setIndex];
        SolverSet disabledSet = world.solverSets[(int)SetType.Disabled];
        int jointKey = body.headJointKey;
        while (jointKey != -1)
        {
            int jointId = jointKey >> 1;
            int edgeIndex = jointKey & 1;
            Joint joint = world.joints[jointId];
            jointKey = edgeIndex == 1 ? joint.edge1.nextKey : joint.edge0.nextKey;
            if (joint.setIndex == (int)SetType.Disabled) continue;
            Debug.Assert(joint.setIndex == set.setIndex || set.setIndex == (int)SetType.Static);
            world.UnlinkJoint(joint);
            SolverSet jointSet = world.solverSets[joint.setIndex];
            world.TransferJoint(disabledSet, jointSet, joint);
        }
        int shapeId = body.headShapeId;
        while (shapeId != -1)
        {
            Shape shape = world.shapes[shapeId];
            shapeId = shape.nextShapeId;
            shape.DestroyProxy(world.broadPhase);
        }
        world.ValidateConnectivity();
        world.ValidateSolverSets();
    }

    ///<summary> Enable a body by adding it to the simulation. This is expensive.</summary>
    public static void Body_Enable(BodyID bodyId)
    {
        World world = World.GetWorldLocked(bodyId.world0); if (world == null) return;
        Body body = world.GetBodyFullID(bodyId);
        if (body.setIndex != (int)SetType.Disabled) return;
        SolverSet disabledSet = world.solverSets[(int)SetType.Disabled];
        int setId = body.type == BodyType.Static ? (int)SetType.Static : (int)SetType.Awake;
        SolverSet targetSet = world.solverSets[setId];
        world.TransferBody(targetSet, disabledSet, body);
        Transform transform = world.GetBodyTransformQuick(body);
        int shapeId = body.headShapeId;
        while (shapeId != -1)
        {
            Shape shape = world.shapes[shapeId];
            shapeId = shape.nextShapeId;
            shape.CreateProxy(world.broadPhase, body.type, transform, true);
        }
        if (setId != (int)SetType.Static) world.CreateIslandForBody(setId, body);
        int jointKey = body.headJointKey;
        while (jointKey != -1)
        {
            int jointId = jointKey >> 1;
            int edgeIndex = jointKey & 1;
            Joint joint = world.joints[jointId];
            Debug.Assert(joint.setIndex == (int)SetType.Disabled);
            Debug.Assert(joint.islandId == -1);
            jointKey = edgeIndex == 1 ? joint.edge1.nextKey : joint.edge0.nextKey;
            Body bodyA = world.bodies[joint.edge0.bodyId], bodyB = world.bodies[joint.edge1.bodyId];
            if (bodyA.setIndex == (int)SetType.Disabled || bodyB.setIndex == (int)SetType.Disabled) continue;
            int jointSetId;
            if (bodyA.setIndex == (int)SetType.Static && bodyB.setIndex == (int)SetType.Static)
                jointSetId = (int)SetType.Static;
            else if (bodyA.setIndex == (int)SetType.Static) jointSetId = bodyB.setIndex;
            else jointSetId = bodyA.setIndex;
            SolverSet jointSet = world.solverSets[jointSetId];
            world.TransferJoint(jointSet, disabledSet, joint);
            if (jointSetId != (int)SetType.Static) world.LinkJoint(joint);
        }
        world.ValidateSolverSets();
    }

    ///<summary> Set the motion locks on this body.</summary>
    public static void Body_SetMotionLocks(BodyID bodyId, MotionLocks locks)
    {
        World world = World.GetWorldLocked(bodyId.world0); if (world == null) return;
        BodyFlags newFlags = (locks.linearX ? BodyFlags.LockLinearX : 0)
            | (locks.linearY ? BodyFlags.LockLinearY : 0)
            | (locks.angularZ ? BodyFlags.LockAngularZ : 0);
        Body body = world.GetBodyFullID(bodyId);
        BodyFlags allLocks = BodyFlags.LockLinearX | BodyFlags.LockLinearY | BodyFlags.LockAngularZ;
        if ((body.flags & allLocks) != newFlags)
        {
            body.flags &= ~allLocks;
            body.flags |= allLocks;
            BodySim bodySim = world.GetBodySim(body);
            bodySim.flags &= ~allLocks;
            bodySim.flags |= newFlags;
            BodyState* state = world.GetBodyState(body);
            if (state != null)
            {
                state->flags = bodySim.flags;
                state->linearVelocity = new(locks.linearX ? 0 : state->linearVelocity.x, locks.linearY ? 0 : state->linearVelocity.y);
                state->angularVelocity = locks.angularZ ? 0 : state->angularVelocity;
            }
        }
    }

    ///<summary> Get the motion locks for this body.</summary>
    public static MotionLocks Body_GetMotionLocks(BodyID bodyId)
    {
        Body body = bodyId.world0.GetBodyFullID(bodyId);
        return new()
        {
            linearX = body.flags.HasFlag(BodyFlags.LockLinearX),
            linearY = body.flags.HasFlag(BodyFlags.LockLinearY),
            angularZ = body.flags.HasFlag(BodyFlags.LockAngularZ)
        };
    }

    ///<summary>Set this body to be a bullet. A bullet does continuous collision detection
    /// against dynamic bodies (but not other bullets).</summary>
    public static void Body_SetBullet(BodyID bodyId, bool flag)
    {
        World world = World.GetWorldLocked(bodyId.world0); if (world == null) return;
        BodySim bodySim = world.GetBodySim(world.GetBodyFullID(bodyId));
        if (flag) bodySim.flags |= BodyFlags.IsBullet;
        else bodySim.flags &= ~BodyFlags.IsBullet;
    }

    ///<summary> Is this body a bullet?</summary>
    public static bool Body_IsBullet(BodyID bodyId) => bodyId.world0.GetBodySim(bodyId.world0.GetBodyFullID(bodyId)).flags.HasFlag(BodyFlags.IsBullet);

    ///<summary>Enable/disable contact events on all shapes.
    /// @see ShapeDef::enableContactEvents</summary>
    ///<remarks> changing this at runtime may cause mismatched begin/end touch events</remarks>
    public static void Body_EnableContactEvents(BodyID bodyId, bool flag)
    {
        World world = bodyId.world0; Body body = world.GetBodyFullID(bodyId);
        int shapeId = body.headShapeId;
        while (shapeId != -1)
        {
            Shape shape = world.shapes[shapeId];
            shape.enableContactEvents = flag;
            shapeId = shape.nextShapeId;
        }
    }

    ///<summary>Enable/disable hit events on all shapes
    /// @see ShapeDef::enableHitEvents</summary>
    public static void Body_EnableHitEvents(BodyID bodyId, bool flag)
    {
        World world = bodyId.world0; Body body = world.GetBodyFullID(bodyId);
        int shapeId = body.headShapeId;
        while (shapeId != -1)
        {
            Shape shape = world.shapes[shapeId];
            shape.enableHitEvents = flag;
            shapeId = shape.nextShapeId;
        }
    }

    ///<summary> Get the world that owns this body</summary>
    public static WorldID Body_GetWorld(BodyID bodyId) => new() { index1 = bodyId.world0, generation = bodyId.world0.generation };

    ///<summary> Get the number of shapes on this body</summary>
    public static int Body_GetShapeCount(BodyID bodyId) => bodyId.world0.GetBodyFullID(bodyId).shapeCount;

    ///<summary>Get the shape ids for all shapes on this body, up to the provided capacity.</summary>
    ///<returns> the number of shape ids stored in the user array</returns>
    public static int Body_GetShapes(BodyID bodyId, ShapeID[] shapeArray)
    {
        World world = bodyId.world0; Body body = world.GetBodyFullID(bodyId);
        int shapeId = body.headShapeId, shapeCount = 0;
        while (shapeId != -1 && shapeCount < shapeArray.Length)
        {
            Shape shape = world.shapes[shapeId];
            shapeArray[shapeCount++] = new() { index1 = shape.id + 1, world0 = bodyId.world0, generation = shape.generation };
            shapeId = shape.nextShapeId;
        }
        return shapeCount;
    }

    ///<summary> Get the number of joints on this body</summary>
    public static int Body_GetJointCount(BodyID bodyId) => bodyId.world0.GetBodyFullID(bodyId).jointCount;
        
    ///<summary>Get the joint ids for all joints on this body, up to the provided capacity</summary>
    ///<returns>the number of joint ids stored in the user array</returns>
    public static int Body_GetJoints(BodyID bodyId, JointID[] jointArray)
    {
        World world = bodyId.world0; Body body = world.GetBodyFullID(bodyId);
        int jointKey = body.headJointKey, jointCount = 0;
        while (jointKey != -1)
        {
            int jointId = jointKey >> 1;
            int edgeIndex = jointKey & 1;
            Joint joint = world.joints[jointId];
            jointArray[jointCount++] = new() { index1 = jointId + 1, world0 = bodyId.world0, generation = joint.generation };
            jointKey = edgeIndex == 1 ? joint.edge1.nextKey : joint.edge0.nextKey;
        }
        return jointCount;
    }

    ///<summary> Get the maximum capacity required for retrieving all the touching contacts on a body</summary>
    public static int Body_GetContactCapacity(BodyID bodyId)
    {
        World world = World.GetWorldLocked(bodyId.world0); if (world == null) return 0;
        return world.GetBodyFullID(bodyId).contactCount;
    }

    /// <summary>Get the touching contact data for a body.</summary>
    /// <remarks>Box2D uses speculative collision so some contact points may be separated.</remarks>
    /// <returns>the number of elements filled in the provided array<br/>
    /// do not ignore the return value, it specifies the valid number of elements</returns>
    public static int Body_GetContactData(BodyID bodyId, ContactData[] contactData)
    {
        World world = World.GetWorldLocked(bodyId.world0); if (world == null) return 0;
        Body body = world.GetBodyFullID(bodyId);
        int contactKey = body.headContactKey;
        int index = 0;
        while (contactKey != -1 && index < contactData.Length)
        {
            int contactId = contactKey >> 1;
            int edgeIndex = contactKey & 1;
            Contact contact = world.contacts[contactId];
            if (contact.flags.HasFlag(ContactFlags.Touching))
            {
                Shape shapeA = world.shapes[contact.shapeIdA], shapeB = world.shapes[contact.shapeIdB];
                contactData[index] = new()
                {
                    contactId = new() { index1 = contact.contactId + 1, world0 = bodyId.world0, generation = contact.generation },
                    shapeIdA = new() { index1 = shapeA.id + 1, world0 = bodyId.world0, generation = shapeA.generation },
                    shapeIdB = new() { index1 = shapeB.id + 1, world0 = bodyId.world0, generation = shapeB.generation },
                    manifold = world.GetContactSim(contact).manifold
                };
                index++;
            }
            contactKey = edgeIndex == 1 ? contact.edge1.nextKey : contact.edge0.nextKey;
        }
        Debug.Assert(index <= contactData.Length);
        return index;
    }

    ///<summary>Get the current world AABB that contains all the attached shapes. Note that this may not encompass the body origin.
    /// If there are no shapes attached then the returned AABB is empty and centered on the body origin.</summary>
    public static AABB Body_ComputeAABB(BodyID bodyId)
    {
        World world = World.GetWorldLocked(bodyId.world0); if (world == null) return new();
        Body body = world.GetBodyFullID(bodyId);
        if (body.headShapeId == -1)
        {
            Transform transform = world.GetBodyTransform(body.id);
            return new(transform.p, transform.p);
        }
        Shape shape = world.shapes[body.headShapeId];
        AABB aabb = shape.aabb;
        while (shape.nextShapeId != -1) aabb = AABB.Union(aabb, world.shapes[shape.nextShapeId].aabb);
        return aabb;
    }

}
