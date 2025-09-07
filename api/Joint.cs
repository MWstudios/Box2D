using System;
using System.Diagnostics;

namespace Box2D.API;

public static class JointAPI
{
    ///<summary> Destroy a joint</summary>
    public static void DestroyJoint(JointID jointId)
    {
        World world = jointId.world0;
        Debug.Assert(!world.locked);
        if (world.locked) return;
        Joint joint = world.GetJointFullID(jointId);
        world.DestroyJointInternal(joint, true);
    }

    ///<summary> Joint identifier validation. Provides validation for up to 64K allocations.</summary>
    public static bool IsValid(JointID id)
    {
        World world = id.world0;
        if (world == null) return false;
        int jointId = id.index1 - 1;
        if (jointId < 0 || world.joints.Count <= jointId) return false;
        Joint joint = world.joints[jointId];
        if (joint.jointId == -1) return false;
        Debug.Assert(joint.jointId == jointId);
        return id.generation == joint.generation;
    }

    ///<summary> Get the joint type</summary>
    public static JointType GetType(JointID jointId) => jointId.world0.GetJointFullID(jointId).type;

    ///<summary> Get body A id on a joint</summary>
    public static BodyID GetBodyA(JointID jointId) => jointId.world0.MakeBodyID(jointId.world0.GetJointFullID(jointId).edge0.bodyId);

    ///<summary> Get body B id on a joint</summary>
    public static BodyID GetBodyB(JointID jointId) => jointId.world0.MakeBodyID(jointId.world0.GetJointFullID(jointId).edge1.bodyId);

    ///<summary> Get the world that owns this joint</summary>
    public static WorldID GetWorld(JointID jointId) => new() { index1 = jointId.world0, generation = jointId.world0.generation };

    ///<summary> Set the local frame on bodyA</summary>
    public static void SetLocalFrameA(JointID jointId, Transform localFrame) => jointId.world0.GetJointSim(jointId.world0.GetJointFullID(jointId)).localFrameA = localFrame;

    ///<summary> Get the local frame on bodyA</summary>
    public static Transform GetLocalFrameA(JointID jointId) => jointId.world0.GetJointSim(jointId.world0.GetJointFullID(jointId)).localFrameA;

    ///<summary> Set the local frame on bodyB</summary>
    public static void SetLocalFrameB(JointID jointId, Transform localFrame) => jointId.world0.GetJointSim(jointId.world0.GetJointFullID(jointId)).localFrameB = localFrame;

    ///<summary> Get the local frame on bodyB</summary>
    public static Transform GetLocalFrameB(JointID jointId) => jointId.world0.GetJointSim(jointId.world0.GetJointFullID(jointId)).localFrameB;

    ///<summary> Toggle collision between connected bodies</summary>
    public static void SetCollideConnected(JointID jointId, bool shouldCollide)
    {
        World world = World.GetWorldLocked(jointId.world0);
        if (world == null) return;
        Joint joint = world.GetJointFullID(jointId);
        if (joint.collideConnected == shouldCollide) return;
        joint.collideConnected = shouldCollide;
        Body bodyA = world.bodies[joint.edge0.bodyId], bodyB = world.bodies[joint.edge1.bodyId];
        if (shouldCollide)
        {
            int shapeCountA = bodyA.shapeCount, shapeCountB = bodyB.shapeCount;
            int shapeId = shapeCountA < shapeCountB ? bodyA.headShapeId : bodyB.headShapeId;
            while (shapeId != -1)
            {
                Shape shape = world.shapes[shapeId];
                if (shape.proxyKey != -1) world.broadPhase.BufferMove(shape.proxyKey);
                shapeId = shape.nextShapeId;
            }
        }
        else world.DestroyContactsBetweeenBodies(bodyA, bodyB);
    }

    ///<summary> Is collision allowed between connected bodies?</summary>
    public static bool GetCollideConnected(JointID jointId) => jointId.world0.GetJointFullID(jointId).collideConnected;

    ///<summary> Set the user data on a joint</summary>
    public static void SetUserData(JointID jointId, object userData) => jointId.world0.GetJointFullID(jointId).userData = userData;

    ///<summary> Get the user data on a joint</summary>
    public static object GetUserData(JointID jointId) => jointId.world0.GetJointFullID(jointId).userData;

    ///<summary> Wake the bodies connect to this joint</summary>
    public static void WakeBodies(JointID jointId)
    {
        World world = World.GetWorldLocked(jointId.world0);
        if (world == null) return;
        Joint joint = world.GetJointFullID(jointId);
        Body bodyA = world.bodies[joint.edge0.bodyId], bodyB = world.bodies[joint.edge1.bodyId];
        world.WakeBody(bodyA); world.WakeBody(bodyB);
    }

    ///<summary> Get the current constraint force for this joint. Usually in Newtons.</summary>
    public static Vector2 GetConstraintForce(JointID jointId) => jointId.world0.GetJointConstraintForce(jointId.world0.GetJointFullID(jointId));

    ///<summary> Get the current constraint torque for this joint. Usually in Newton * meters.</summary>
    public static float GetConstraintTorque(JointID jointId) => jointId.world0.GetJointConstraintTorque(jointId.world0.GetJointFullID(jointId));

    ///<summary> Get the current linear separation error for this joint. Does not consider admissible movement. Usually in meters.</summary>
    public static float GetLinearSeparation(JointID jointId)
    {
        World world = jointId.world0;
        Joint joint = world.GetJointFullID(jointId);
        JointSim base_ = world.GetJointSim(joint);
        Transform xfA = world.GetBodyTransform(joint.edge0.bodyId), xfB = world.GetBodyTransform(joint.edge1.bodyId);
        Vector2 pA = xfA.TransformPoint(base_.localFrameA.p), pB = xfB.TransformPoint(base_.localFrameB.p);
        Vector2 dp = pB - pA;
        return base_.joint.GetLinearSeparation(xfA, xfB, dp);
    }

    ///<summary> Get the current angular separation error for this joint. Does not consider admissible movement. Usually in meters.</summary>
    public static float GetAngularSeparation(JointID jointId)
    {
        World world = jointId.world0;
        Joint joint = world.GetJointFullID(jointId);
        JointSim base_ = world.GetJointSim(joint);
        Transform xfA = world.GetBodyTransform(joint.edge0.bodyId), xfB = world.GetBodyTransform(joint.edge1.bodyId);
        float relativeAngle = Rotation.RelativeAngle(xfA.q, xfB.q);
        return base_.joint.GetAngularSeparation(relativeAngle);
    }

    /// <summary>Set the joint constraint tuning. Advanced feature.</summary>
    /// <param name="jointID">the joint</param>
    /// <param name="hertz">the stiffness in Hertz (cycles per second)</param>
    /// <param name="dampingRatio">the non-dimensional damping ratio (one for critical damping)</param>
    public static void SetConstraintTuning(JointID jointId, float hertz, float dampingRatio)
    {
        Debug.Assert(float.IsFinite(hertz) && hertz >= 0);
        Debug.Assert(float.IsFinite(dampingRatio) && dampingRatio >= 0);
        JointSim base_ = jointId.world0.GetJointSim(jointId.world0.GetJointFullID(jointId));
        base_.constraintHertz = hertz;
        base_.constraintDampingRatio = dampingRatio;
    }

    ///<summary> Get the joint constraint tuning. Advanced feature.</summary>
    public static void GetConstraintTuning(JointID jointId, out float hertz, out float dampingRatio)
    {
        JointSim base_ = jointId.world0.GetJointSim(jointId.world0.GetJointFullID(jointId));
        hertz = base_.constraintHertz;
        dampingRatio = base_.constraintDampingRatio;
    }

    ///<summary> Set the force threshold for joint events (Newtons)</summary>
    public static void SetForceThreshold(JointID jointId, float threshold)
    {
        Debug.Assert(float.IsFinite(threshold) && threshold >= 0);
        jointId.world0.GetJointSim(jointId.world0.GetJointFullID(jointId)).forceThreshold = threshold;
    }

    ///<summary> Get the force threshold for joint events (Newtons)</summary>
    public static float GetForceThreshold(JointID jointId) =>
        jointId.world0.GetJointSim(jointId.world0.GetJointFullID(jointId)).forceThreshold;

    ///<summary> Set the torque threshold for joint events (N-m)</summary>
    public static void SetTorqueThreshold(JointID jointId, float threshold)
    {
        Debug.Assert(float.IsFinite(threshold) && threshold >= 0);
        jointId.world0.GetJointSim(jointId.world0.GetJointFullID(jointId)).torqueThreshold = threshold;
    }

    ///<summary> Get the torque threshold for joint events (N-m)</summary>
    public static float GetTorqueThreshold(JointID jointId) =>
        jointId.world0.GetJointSim(jointId.world0.GetJointFullID(jointId)).torqueThreshold;
    static JointSim GetJointSimCheckType(JointID jointId, JointType type)
    {
        World world = jointId.world0;
        Debug.Assert(!world.locked);
        if (world.locked) return null;
        Joint joint = world.GetJointFullID(jointId);
        Debug.Assert(joint.type == type);
        JointSim jointSim = world.GetJointSim(joint);
        Debug.Assert(jointSim.type == type);
        return jointSim;
    }

    /// <summary>Create a distance joint
    /// @see DistanceJointDef for details</summary>
    public static JointID CreateDistanceJoint(WorldID worldId, ref DistanceJointDef def) => DistanceJoint.Create(worldId, ref def);

    /// <summary>Set the rest length of a distance joint</summary>
    /// <param name="jointID">The id for a distance joint</param>
    /// <param name="length">The new distance joint length</param>
    public static void DistanceJoint_SetLength(JointID jointId, float length)
    {
        JointSim base_ = GetJointSimCheckType(jointId, JointType.Distance);
        DistanceJoint joint = (DistanceJoint)base_.joint;
        joint.length = Math.Clamp(length, Box2D.LinearSlop, Box2D.Huge);
        joint.lowerImpulse = 0;
        joint.upperImpulse = 0;
    }

    ///<summary> Get the rest length of a distance joint</summary>
    public static float DistanceJoint_GetLength(JointID jointId) =>
        ((DistanceJoint)GetJointSimCheckType(jointId, JointType.Distance).joint).length;

    ///<summary> Enable/disable the distance joint spring. When disabled the distance joint is rigid.</summary>
    public static void DistanceJoint_EnableSpring(JointID jointId, bool enableSpring)=>
        ((DistanceJoint) GetJointSimCheckType(jointId, JointType.Distance).joint).enableSpring = enableSpring;

    ///<summary> Is the distance joint spring enabled?</summary>
    public static bool DistanceJoint_IsSpringEnabled(JointID jointId) =>
        ((DistanceJoint)GetJointSimCheckType(jointId, JointType.Distance).joint).enableSpring;

    ///<summary> Set the force range for the spring.</summary>
    public static void DistanceJoint_SetSpringForceRange(JointID jointId, float lowerForce, float upperForce)
    {
        Debug.Assert(lowerForce <= upperForce);
        DistanceJoint base_ = (DistanceJoint)GetJointSimCheckType(jointId, JointType.Distance).joint;
        base_.lowerSpringForce = lowerForce;
        base_.upperSpringForce = upperForce;
    }

    ///<summary> Get the force range for the spring.</summary>
    public static void DistanceJoint_GetSpringForceRange(JointID jointId, out float lowerForce, out float upperForce)
    {
        DistanceJoint base_ = (DistanceJoint)GetJointSimCheckType(jointId, JointType.Distance).joint;
        lowerForce = base_.lowerSpringForce;
        upperForce = base_.upperSpringForce;
    }

    ///<summary> Set the spring stiffness in Hertz</summary>
    public static void DistanceJoint_SetSpringHertz(JointID jointId, float hertz) =>
        ((DistanceJoint)GetJointSimCheckType(jointId, JointType.Distance).joint).hertz = hertz;

    ///<summary> Set the spring damping ratio, non-dimensional</summary>
    public static void DistanceJoint_SetSpringDampingRatio(JointID jointId, float dampingRatio) =>
        ((DistanceJoint)GetJointSimCheckType(jointId, JointType.Distance).joint).dampingRatio = dampingRatio;

    ///<summary> Get the spring Hertz</summary>
    public static float DistanceJoint_GetSpringHertz(JointID jointId) =>
        ((DistanceJoint)GetJointSimCheckType(jointId, JointType.Distance).joint).hertz;

    ///<summary> Get the spring damping ratio</summary>
    public static float DistanceJoint_GetSpringDampingRatio(JointID jointId) =>
        ((DistanceJoint)GetJointSimCheckType(jointId, JointType.Distance).joint).dampingRatio;

    ///<summary>Enable joint limit. The limit only works if the joint spring is enabled. Otherwise the joint is rigid
    /// and the limit has no effect.</summary>
    public static void DistanceJoint_EnableLimit(JointID jointId, bool enableLimit) =>
        ((DistanceJoint)GetJointSimCheckType(jointId, JointType.Distance).joint).enableLimit = enableLimit;

    ///<summary> Is the distance joint limit enabled?</summary>
    public static bool DistanceJoint_IsLimitEnabled(JointID jointId) =>
        ((DistanceJoint)GetJointSimCheckType(jointId, JointType.Distance).joint).enableLimit;

    ///<summary> Set the minimum and maximum length parameters of a distance joint</summary>
    public static void DistanceJoint_SetLengthRange(JointID jointId, float minLength, float maxLength)
    {
        JointSim base_ = GetJointSimCheckType(jointId, JointType.Distance);
        DistanceJoint joint = (DistanceJoint)base_.joint;
        minLength = Math.Clamp(minLength, Box2D.LinearSlop, Box2D.Huge);
        maxLength = Math.Clamp(maxLength, Box2D.LinearSlop, Box2D.Huge);
        joint.minLength = Math.Min(minLength, maxLength);
        joint.maxLength = Math.Max(minLength, maxLength);
        joint.impulse = 0;
        joint.lowerImpulse = 0;
        joint.upperImpulse = 0;
    }

    ///<summary> Get the distance joint minimum length</summary>
    public static float DistanceJoint_GetMinLength(JointID jointId) =>
        ((DistanceJoint)GetJointSimCheckType(jointId, JointType.Distance).joint).minLength;

    ///<summary> Get the distance joint maximum length</summary>
    public static float DistanceJoint_GetMaxLength(JointID jointId) =>
        ((DistanceJoint)GetJointSimCheckType(jointId, JointType.Distance).joint).maxLength;

    ///<summary> Get the current length of a distance joint</summary>
    public static float DistanceJoint_GetCurrentLength(JointID jointId)
    {
        JointSim base_ = GetJointSimCheckType(jointId, JointType.Distance);
        World world = jointId.world0;
        Debug.Assert(!world.locked);
        if (world.locked) return 0;
        Transform transformA = world.GetBodyTransform(base_.bodyIdA);
        Transform transformB = world.GetBodyTransform(base_.bodyIdB);
        Vector2 pA = transformA.TransformPoint(base_.localFrameA.p);
        Vector2 pB = transformB.TransformPoint(base_.localFrameB.p);
        return (pB - pA).Length();
    }

    ///<summary> Enable/disable the distance joint motor</summary>
    public static void DistanceJoint_EnableMotor(JointID jointId, bool enableMotor)
    {
        JointSim base_ = GetJointSimCheckType(jointId, JointType.Distance);
        DistanceJoint distanceJoint = (DistanceJoint)base_.joint;
        if (enableMotor != distanceJoint.enableMotor)
        {
            distanceJoint.enableMotor = enableMotor;
            distanceJoint.motorImpulse = 0.0f;
        }
    }

    ///<summary> Is the distance joint motor enabled?</summary>
    public static bool DistanceJoint_IsMotorEnabled(JointID jointId) =>
        ((DistanceJoint)GetJointSimCheckType(jointId, JointType.Distance).joint).enableMotor;

    ///<summary> Set the distance joint motor speed, usually in meters per second</summary>
    public static void DistanceJoint_SetMotorSpeed(JointID jointId, float motorSpeed) =>
        ((DistanceJoint)GetJointSimCheckType(jointId, JointType.Distance).joint).motorSpeed = motorSpeed;

    ///<summary> Get the distance joint motor speed, usually in meters per second</summary>
    public static float DistanceJoint_GetMotorSpeed(JointID jointId) =>
        ((DistanceJoint)GetJointSimCheckType(jointId, JointType.Distance).joint).motorSpeed;

    ///<summary> Set the distance joint maximum motor force, usually in newtons</summary>
    public static void DistanceJoint_SetMaxMotorForce(JointID jointId, float force) =>
        ((DistanceJoint)GetJointSimCheckType(jointId, JointType.Distance).joint).maxMotorForce = force;

    ///<summary> Get the distance joint maximum motor force, usually in newtons</summary>
    public static float DistanceJoint_GetMaxMotorForce(JointID jointId) =>
        ((DistanceJoint)GetJointSimCheckType(jointId, JointType.Distance).joint).maxMotorForce;

    ///<summary> Get the distance joint current motor force, usually in newtons</summary>
    public static float DistanceJoint_GetMotorForce(JointID jointId) =>
        ((DistanceJoint)GetJointSimCheckType(jointId, JointType.Distance).joint).motorImpulse * jointId.world0.inv_h;

    ///<summary> Create a motor joint
    /// @see MotorJointDef for details</summary>
    public static JointID CreateMotorJoint(WorldID worldId, ref MotorJointDef def) => MotorJoint.Create(worldId, ref def);

    ///<summary> Set the desired relative linear velocity in meters per second</summary>
    public static void MotorJoint_SetLinearVelocity(JointID jointId, Vector2 velocity) =>
        ((MotorJoint)GetJointSimCheckType(jointId, JointType.Motor).joint).linearVelocity = velocity;

    ///<summary> Get the desired relative linear velocity in meters per second</summary>
    public static Vector2 MotorJoint_GetLinearVelocity(JointID jointId) =>
        ((MotorJoint)GetJointSimCheckType(jointId, JointType.Motor).joint).linearVelocity;

    ///<summary> Set the desired relative angular velocity in radians per second</summary>
    public static void MotorJoint_SetAngularVelocity(JointID jointId, float velocity) =>
        ((MotorJoint)GetJointSimCheckType(jointId, JointType.Motor).joint).angularVelocity = velocity;

    ///<summary> Get the desired relative angular velocity in radians per second</summary>
    public static float MotorJoint_GetAngularVelocity(JointID jointId) =>
        ((MotorJoint)GetJointSimCheckType(jointId, JointType.Motor).joint).angularVelocity;

    ///<summary> Set the motor joint maximum force, usually in newtons</summary>
    public static void MotorJoint_SetMaxVelocityForce(JointID jointId, float maxForce) =>
        ((MotorJoint)GetJointSimCheckType(jointId, JointType.Motor).joint).maxVelocityForce = Math.Max(0, maxForce);

    ///<summary> Get the motor joint maximum force, usually in newtons</summary>
    public static float MotorJoint_GetMaxVelocityForce(JointID jointId) =>
        ((MotorJoint)GetJointSimCheckType(jointId, JointType.Motor).joint).maxVelocityForce;

    ///<summary> Set the motor joint maximum torque, usually in newton-meters</summary>
    public static void MotorJoint_SetMaxVelocityTorque(JointID jointId, float maxTorque) =>
        ((MotorJoint)GetJointSimCheckType(jointId, JointType.Motor).joint).maxVelocityTorque = maxTorque;

    ///<summary> Get the motor joint maximum torque, usually in newton-meters</summary>
    public static float MotorJoint_GetMaxTorque(JointID jointId) =>
        ((MotorJoint)GetJointSimCheckType(jointId, JointType.Motor).joint).maxVelocityTorque;

    ///<summary> Set the spring linear hertz stiffness</summary>
    public static void MotorJoint_SetLinearHertz(JointID jointId, float hertz) =>
        ((MotorJoint)GetJointSimCheckType(jointId, JointType.Motor).joint).linearHertz = hertz;

    ///<summary> Get the spring linear hertz stiffness</summary>
    public static float MotorJoint_GetLinearHertz(JointID jointId) =>
        ((MotorJoint)GetJointSimCheckType(jointId, JointType.Motor).joint).linearHertz;

    ///<summary> Set the spring linear damping ratio. Use 1.0 for critical damping.</summary>
    public static void MotorJoint_SetLinearDampingRatio(JointID jointId, float damping) =>
        ((MotorJoint)GetJointSimCheckType(jointId, JointType.Motor).joint).linearDampingRatio = damping;

    ///<summary> Get the spring linear damping ratio.</summary>
    public static float MotorJoint_GetLinearDampingRatio(JointID jointId) =>
        ((MotorJoint)GetJointSimCheckType(jointId, JointType.Motor).joint).linearDampingRatio;

    ///<summary> Set the spring angular hertz stiffness</summary>
    public static void MotorJoint_SetAngularHertz(JointID jointId, float hertz) =>
        ((MotorJoint)GetJointSimCheckType(jointId, JointType.Motor).joint).angularHertz = hertz;

    ///<summary> Get the spring angular hertz stiffness</summary>
    public static float MotorJoint_GetAngularHertz(JointID jointId) =>
        ((MotorJoint)GetJointSimCheckType(jointId, JointType.Motor).joint).angularHertz;

    ///<summary> Set the spring angular damping ratio. Use 1.0 for critical damping.</summary>
    public static void MotorJoint_SetAngularDampingRatio(JointID jointId, float damping) =>
        ((MotorJoint)GetJointSimCheckType(jointId, JointType.Motor).joint).angularDampingRatio = damping;

    ///<summary> Get the spring angular damping ratio.</summary>
    public static float MotorJoint_GetAngularDampingRatio(JointID jointId) =>
        ((MotorJoint)GetJointSimCheckType(jointId, JointType.Motor).joint).angularDampingRatio;

    ///<summary> Set the maximum spring force in newtons.</summary>
    public static void MotorJoint_SetMaxSpringForce(JointID jointId, float maxForce) =>
        ((MotorJoint)GetJointSimCheckType(jointId, JointType.Motor).joint).maxSpringForce = Math.Max(0, maxForce);

    ///<summary> Get the maximum spring force in newtons.</summary>
    public static float MotorJoint_GetMaxSpringForce(JointID jointId) =>
        ((MotorJoint)GetJointSimCheckType(jointId, JointType.Motor).joint).maxSpringForce;

    ///<summary> Set the maximum spring torque in newtons * meters</summary>
    public static void MotorJoint_SetMaxSpringTorque(JointID jointId, float maxTorque) =>
        ((MotorJoint)GetJointSimCheckType(jointId, JointType.Motor).joint).maxSpringTorque = Math.Max(0, maxTorque);

    ///<summary> Get the maximum spring torque in newtons * meters</summary>
    public static float MotorJoint_GetMaxSpringTorque(JointID jointId) =>
        ((MotorJoint)GetJointSimCheckType(jointId, JointType.Motor).joint).maxSpringTorque;

    ///<summary>Create a filter joint.
    /// @see FilterJointDef for details</summary>
    public static JointID CreateFilterJoint(WorldID worldId, ref FilterJointDef def) => FilterJoint.Create(worldId, ref def);

    ///<summary>Create a prismatic (slider) joint.
    /// @see PrismaticJointDef for details</summary>
    public static JointID CreatePrismaticJoint(WorldID worldId, ref PrismaticJointDef def) => PrismaticJoint.Create(worldId, ref def);

    ///<summary> Enable/disable the joint spring.</summary>
    public static void PrismaticJoint_EnableSpring(JointID jointId, bool enableSpring)
    {
        PrismaticJoint joint = (PrismaticJoint)GetJointSimCheckType(jointId, JointType.Prismatic).joint;
        if (enableSpring != joint.enableSpring) { joint.enableSpring = enableSpring; joint.springImpulse = 0; }
    }

    ///<summary> Is the prismatic joint spring enabled or not?</summary>
    public static bool PrismaticJoint_IsSpringEnabled(JointID jointId) =>
        ((PrismaticJoint)GetJointSimCheckType(jointId, JointType.Prismatic).joint).enableSpring;

    ///<summary>Set the prismatic joint stiffness in Hertz.
    /// This should usually be less than a quarter of the simulation rate. For example, if the simulation
    /// runs at 60Hz then the joint stiffness should be 15Hz or less.</summary>
    public static void PrismaticJoint_SetSpringHertz(JointID jointId, float hertz) =>
        ((PrismaticJoint)GetJointSimCheckType(jointId, JointType.Prismatic).joint).hertz = hertz;

    ///<summary> Get the prismatic joint stiffness in Hertz</summary>
    public static float PrismaticJoint_GetSpringHertz(JointID jointId) =>
        ((PrismaticJoint)GetJointSimCheckType(jointId, JointType.Prismatic).joint).hertz;

    ///<summary> Set the prismatic joint damping ratio (non-dimensional)</summary>
    public static void PrismaticJoint_SetSpringDampingRatio(JointID jointId, float dampingRatio) =>
        ((PrismaticJoint)GetJointSimCheckType(jointId, JointType.Prismatic).joint).dampingRatio = dampingRatio;

    ///<summary> Get the prismatic spring damping ratio (non-dimensional)</summary>
    public static float PrismaticJoint_GetSpringDampingRatio(JointID jointId) =>
        ((PrismaticJoint)GetJointSimCheckType(jointId, JointType.Prismatic).joint).dampingRatio;

    ///<summary> Set the prismatic joint spring target angle, usually in meters</summary>
    public static void PrismaticJoint_SetTargetTranslation(JointID jointId, float translation) =>
        ((PrismaticJoint)GetJointSimCheckType(jointId, JointType.Prismatic).joint).targetTranslation = translation;

    ///<summary> Get the prismatic joint spring target translation, usually in meters</summary>
    public static float PrismaticJoint_GetTargetTranslation(JointID jointId) =>
        ((PrismaticJoint)GetJointSimCheckType(jointId, JointType.Prismatic).joint).targetTranslation;

    ///<summary> Enable/disable a prismatic joint limit</summary>
    public static void PrismaticJoint_EnableLimit(JointID jointId, bool enableLimit)
    {
        PrismaticJoint joint = (PrismaticJoint)GetJointSimCheckType(jointId, JointType.Prismatic).joint;
        if (enableLimit != joint.enableLimit) { joint.enableLimit = enableLimit; joint.lowerImpulse = 0; joint.upperImpulse = 0; }
    }

    ///<summary> Is the prismatic joint limit enabled?</summary>
    public static bool PrismaticJoint_IsLimitEnabled(JointID jointId) =>
        ((PrismaticJoint)GetJointSimCheckType(jointId, JointType.Prismatic).joint).enableLimit;

    ///<summary> Get the prismatic joint lower limit</summary>
    public static float PrismaticJoint_GetLowerLimit(JointID jointId) =>
        ((PrismaticJoint)GetJointSimCheckType(jointId, JointType.Prismatic).joint).lowerTranslation;

    ///<summary> Get the prismatic joint upper limit</summary>
    public static float PrismaticJoint_GetUpperLimit(JointID jointId) =>
        ((PrismaticJoint)GetJointSimCheckType(jointId, JointType.Prismatic).joint).upperTranslation;

    ///<summary> Set the prismatic joint limits</summary>
    public static void PrismaticJoint_SetLimits(JointID jointId, float lower, float upper)
    {
        Debug.Assert(lower <= upper);
        PrismaticJoint joint = (PrismaticJoint)GetJointSimCheckType(jointId, JointType.Prismatic).joint;
        if (lower != joint.lowerTranslation || upper != joint.upperTranslation)
        {
            joint.lowerTranslation = Math.Min(lower, upper);
            joint.upperTranslation = Math.Max(lower, upper);
            joint.lowerImpulse = 0; joint.upperImpulse = 0;
        }
    }

    ///<summary> Enable/disable a prismatic joint motor</summary>
    public static void PrismaticJoint_EnableMotor(JointID jointId, bool enableMotor)
    {
        PrismaticJoint joint = (PrismaticJoint)GetJointSimCheckType(jointId, JointType.Prismatic).joint;
        if (enableMotor != joint.enableMotor) { joint.enableMotor = enableMotor; joint.motorImpulse = 0; }
    }

    ///<summary> Is the prismatic joint motor enabled?</summary>
    public static bool PrismaticJoint_IsMotorEnabled(JointID jointId) =>
        ((PrismaticJoint)GetJointSimCheckType(jointId, JointType.Prismatic).joint).enableMotor;

    ///<summary> Set the prismatic joint motor speed, usually in meters per second</summary>
    public static void PrismaticJoint_SetMotorSpeed(JointID jointId, float motorSpeed) =>
        ((PrismaticJoint)GetJointSimCheckType(jointId, JointType.Prismatic).joint).motorSpeed = motorSpeed;

    ///<summary> Get the prismatic joint motor speed, usually in meters per second</summary>
    public static float PrismaticJoint_GetMotorSpeed(JointID jointId) =>
        ((PrismaticJoint)GetJointSimCheckType(jointId, JointType.Prismatic).joint).motorSpeed;

    ///<summary> Set the prismatic joint maximum motor force, usually in newtons</summary>
    public static void PrismaticJoint_SetMaxMotorForce(JointID jointId, float force) =>
        ((PrismaticJoint)GetJointSimCheckType(jointId, JointType.Prismatic).joint).maxMotorForce = force;

    ///<summary> Get the prismatic joint maximum motor force, usually in newtons</summary>
    public static float PrismaticJoint_GetMaxMotorForce(JointID jointId) =>
        ((PrismaticJoint)GetJointSimCheckType(jointId, JointType.Prismatic).joint).maxMotorForce;

    ///<summary> Get the prismatic joint current motor force, usually in newtons</summary>
    public static float PrismaticJoint_GetMotorForce(JointID jointId) =>
        ((PrismaticJoint)GetJointSimCheckType(jointId, JointType.Prismatic).joint).motorImpulse * jointId.world0.inv_h;

    ///<summary> Get the current joint translation, usually in meters.</summary>
    public static float PrismaticJoint_GetTranslation(JointID jointId)
    {
        World world = jointId.world0;
        JointSim jointSim = GetJointSimCheckType(jointId, JointType.Prismatic);
        Transform transformA = world.GetBodyTransform(jointSim.bodyIdA);
        Transform transformB = world.GetBodyTransform(jointSim.bodyIdB);
        Vector2 localAxisA = jointSim.localFrameA.q * new Vector2(1, 0);
        Vector2 axisA = transformA.q * localAxisA;
        Vector2 pA = transformA.TransformPoint(jointSim.localFrameA.p);
        Vector2 pB = transformB.TransformPoint(jointSim.localFrameB.p);
        Vector2 d = pB - pA;
        return Vector2.Dot(d, axisA);
    }

    ///<summary> Get the current joint translation speed, usually in meters per second.</summary>
    public unsafe static float PrismaticJoint_GetSpeed(JointID jointId)
    {
        World world = jointId.world0;
        Joint joint = world.GetJointFullID(jointId); Debug.Assert(joint.type == JointType.Prismatic);
        JointSim base_ = world.GetJointSim(joint); Debug.Assert(base_.type == JointType.Prismatic);
        Body bodyA = world.bodies[base_.bodyIdA], bodyB = world.bodies[base_.bodyIdB];
        BodySim bodySimA = world.GetBodySim(bodyA), bodySimB = world.GetBodySim(bodyB);
        BodyState* bodyStateA = world.GetBodyState(bodyA), bodyStateB = world.GetBodyState(bodyB);
        Transform transformA = bodySimA.transform, transformB = bodySimB.transform;
        Vector2 localAxisA = base_.localFrameA.q * new Vector2(1, 0);
        Vector2 axisA = transformA.q * localAxisA;
        Vector2 cA = bodySimA.center, cB = bodySimB.center;
        Vector2 rA = transformA.q * (base_.localFrameA.p - bodySimA.localCenter);
        Vector2 rB = transformB.q * (base_.localFrameB.p - bodySimB.localCenter);
        Vector2 d = cB - cA + (rB - rA);
        Vector2 vA = bodyStateA != null ? bodyStateA->linearVelocity : Vector2.Zero;
        Vector2 vB = bodyStateB != null ? bodyStateB->linearVelocity : Vector2.Zero;
        float wA = bodyStateA != null ? bodyStateA->angularVelocity : 0;
        float wB = bodyStateB != null ? bodyStateB->angularVelocity : 0;
        Vector2 vRel = vB + Vector2.CrossSV(wB, rB) - (vA + Vector2.CrossSV(wA, rA));
        return Vector2.Dot(d, Vector2.CrossSV(wA, axisA)) + Vector2.Dot(axisA, vRel);
    }

    ///<summary>Create a revolute joint
    /// @see RevoluteJointDef for details</summary>
    public static JointID CreateRevoluteJoint(WorldID worldId, ref RevoluteJointDef def) => RevoluteJoint.Create(worldId, ref def);

    ///<summary> Enable/disable the revolute joint spring</summary>
    public static void RevoluteJoint_EnableSpring(JointID jointId, bool enableSpring)
    {
        RevoluteJoint joint = (RevoluteJoint)GetJointSimCheckType(jointId, JointType.Revolute).joint;
        if (enableSpring != joint.enableSpring) { joint.enableSpring = enableSpring; joint.springImpulse = 0; }
    }

    ///<summary> It the revolute angular spring enabled?</summary>
    public static bool RevoluteJoint_IsSpringEnabled(JointID jointId) =>
        ((RevoluteJoint)GetJointSimCheckType(jointId, JointType.Revolute).joint).enableSpring;

    ///<summary> Set the revolute joint spring stiffness in Hertz</summary>
    public static void RevoluteJoint_SetSpringHertz(JointID jointId, float hertz) =>
        ((RevoluteJoint)GetJointSimCheckType(jointId, JointType.Revolute).joint).hertz = hertz;

    ///<summary> Get the revolute joint spring stiffness in Hertz</summary>
    public static float RevoluteJoint_GetSpringHertz(JointID jointId) =>
        ((RevoluteJoint)GetJointSimCheckType(jointId, JointType.Revolute).joint).hertz;

    ///<summary> Set the revolute joint spring damping ratio, non-dimensional</summary>
    public static void RevoluteJoint_SetSpringDampingRatio(JointID jointId, float dampingRatio) =>
        ((RevoluteJoint)GetJointSimCheckType(jointId, JointType.Revolute).joint).dampingRatio = dampingRatio;

    ///<summary> Get the revolute joint spring damping ratio, non-dimensional</summary>
    public static float RevoluteJoint_GetSpringDampingRatio(JointID jointId) =>
        ((RevoluteJoint)GetJointSimCheckType(jointId, JointType.Revolute).joint).dampingRatio;

    ///<summary> Set the revolute joint spring target angle, radians</summary>
    public static void RevoluteJoint_SetTargetAngle(JointID jointId, float angle) =>
        ((RevoluteJoint)GetJointSimCheckType(jointId, JointType.Revolute).joint).targetAngle = angle;

    ///<summary> Get the revolute joint spring target angle, radians</summary>
    public static float RevoluteJoint_GetTargetAngle(JointID jointId) =>
        ((RevoluteJoint)GetJointSimCheckType(jointId, JointType.Revolute).joint).targetAngle;

    ///<summary>Get the revolute joint current angle in radians relative to the reference angle
    /// @see RevoluteJointDef::referenceAngle</summary>
    public static float RevoluteJoint_GetAngle(JointID jointId)
    {
        World world = jointId.world0;
        JointSim jointSim = GetJointSimCheckType(jointId, JointType.Revolute);
        Transform transformA = world.GetBodyTransform(jointSim.bodyIdA);
        Transform transformB = world.GetBodyTransform(jointSim.bodyIdB);
        Rotation qA = transformA.q * jointSim.localFrameA.q;
        Rotation qB = transformB.q * jointSim.localFrameB.q;
        return Rotation.RelativeAngle(qA, qB);
    }

    ///<summary> Enable/disable the revolute joint limit</summary>
    public static void RevoluteJoint_EnableLimit(JointID jointId, bool enableLimit)
    {
        RevoluteJoint joint = (RevoluteJoint)GetJointSimCheckType(jointId, JointType.Revolute).joint;
        if (enableLimit != joint.enableLimit) { joint.enableLimit = enableLimit; joint.lowerImpulse = 0; joint.upperImpulse = 0; }
    }

    ///<summary> Is the revolute joint limit enabled?</summary>
    public static bool RevoluteJoint_IsLimitEnabled(JointID jointId) =>
        ((RevoluteJoint)GetJointSimCheckType(jointId, JointType.Revolute).joint).enableLimit;

    ///<summary> Get the revolute joint lower limit in radians</summary>
    public static float RevoluteJoint_GetLowerLimit(JointID jointId) =>
        ((RevoluteJoint)GetJointSimCheckType(jointId, JointType.Revolute).joint).lowerAngle;

    ///<summary> Get the revolute joint upper limit in radians</summary>
    public static float RevoluteJoint_GetUpperLimit(JointID jointId) =>
        ((RevoluteJoint)GetJointSimCheckType(jointId, JointType.Revolute).joint).upperAngle;

    /// <summary>Set the revolute joint limits in radians. It is expected that lower <= upper
    /// and that -0.99 * B2_PI <= lower && upper <= -0.99 * B2_PI.</summary>
    public static void RevoluteJoint_SetLimits(JointID jointId, float lower, float upper)
    {
        Debug.Assert(lower <= upper);
        Debug.Assert(lower >= -0.99f * MathF.PI);
        Debug.Assert(upper <= 0.99f * MathF.PI);
        RevoluteJoint joint = (RevoluteJoint)GetJointSimCheckType(jointId, JointType.Revolute).joint;
        if (lower != joint.lowerAngle || upper != joint.upperAngle)
        {
            joint.lowerAngle = Math.Min(lower, upper); joint.upperAngle = Math.Max(lower, upper);
            joint.lowerImpulse = 0; joint.upperImpulse = 0;
        }
    }

    ///<summary> Enable/disable a revolute joint motor</summary>
    public static void RevoluteJoint_EnableMotor(JointID jointId, bool enableMotor)
    {
        RevoluteJoint joint = (RevoluteJoint)GetJointSimCheckType(jointId, JointType.Revolute).joint;
        if (enableMotor != joint.enableMotor) { joint.enableMotor = enableMotor; joint.motorImpulse = 0; }
    }

    ///<summary> Is the revolute joint motor enabled?</summary>
    public static bool RevoluteJoint_IsMotorEnabled(JointID jointId) =>
        ((RevoluteJoint)GetJointSimCheckType(jointId, JointType.Revolute).joint).enableMotor;

    ///<summary> Set the revolute joint motor speed in radians per second</summary>
    public static void RevoluteJoint_SetMotorSpeed(JointID jointId, float motorSpeed) =>
        ((RevoluteJoint)GetJointSimCheckType(jointId, JointType.Revolute).joint).motorSpeed = motorSpeed;

    ///<summary> Get the revolute joint motor speed in radians per second</summary>
    public static float RevoluteJoint_GetMotorSpeed(JointID jointId) =>
        ((RevoluteJoint)GetJointSimCheckType(jointId, JointType.Revolute).joint).motorSpeed;

    ///<summary> Get the revolute joint current motor torque, usually in newton-meters</summary>
    public static float RevoluteJoint_GetMotorTorque(JointID jointId) =>
        ((RevoluteJoint)GetJointSimCheckType(jointId, JointType.Revolute).joint).motorImpulse * jointId.world0.inv_h;

    ///<summary> Set the revolute joint maximum motor torque, usually in newton-meters</summary>
    public static void RevoluteJoint_SetMaxMotorTorque(JointID jointId, float torque) =>
        ((RevoluteJoint)GetJointSimCheckType(jointId, JointType.Revolute).joint).maxMotorTorque = torque;

    ///<summary> Get the revolute joint maximum motor torque, usually in newton-meters</summary>
    public static float RevoluteJoint_GetMaxMotorTorque(JointID jointId) =>
        ((RevoluteJoint)GetJointSimCheckType(jointId, JointType.Revolute).joint).maxMotorTorque;

    ///<summary>Create a weld joint
    /// @see WeldJointDef for details</summary>
    public static JointID CreateWeldJoint(WorldID worldId, ref WeldJointDef def) => WeldJoint.Create(worldId, ref def);

    ///<summary> Set the weld joint linear stiffness in Hertz. 0 is rigid.</summary>
    public static void WeldJoint_SetLinearHertz(JointID jointId, float hertz)
    {
        Debug.Assert(float.IsFinite(hertz) && hertz >= 0);
        ((WeldJoint)GetJointSimCheckType(jointId, JointType.Weld).joint).linearHertz = hertz;
    }

    ///<summary> Get the weld joint linear stiffness in Hertz</summary>
    public static float WeldJoint_GetLinearHertz(JointID jointId) =>
        ((WeldJoint)GetJointSimCheckType(jointId, JointType.Weld).joint).linearHertz;

    ///<summary> Set the weld joint linear damping ratio (non-dimensional)</summary>
    public static void WeldJoint_SetLinearDampingRatio(JointID jointId, float dampingRatio)
    {
        Debug.Assert(float.IsFinite(dampingRatio) && dampingRatio >= 0);
        ((WeldJoint)GetJointSimCheckType(jointId, JointType.Weld).joint).linearDampingRatio = dampingRatio;
    }

    ///<summary> Get the weld joint linear damping ratio (non-dimensional)</summary>
    public static float WeldJoint_GetLinearDampingRatio(JointID jointId) =>
        ((WeldJoint)GetJointSimCheckType(jointId, JointType.Weld).joint).linearDampingRatio;

    ///<summary> Set the weld joint angular stiffness in Hertz. 0 is rigid.</summary>
    public static void WeldJoint_SetAngularHertz(JointID jointId, float hertz)
    {
        Debug.Assert(float.IsFinite(hertz) && hertz >= 0);
        ((WeldJoint)GetJointSimCheckType(jointId, JointType.Weld).joint).angularHertz = hertz;
    }

    ///<summary> Get the weld joint angular stiffness in Hertz</summary>
    public static float WeldJoint_GetAngularHertz(JointID jointId) =>
        ((WeldJoint)GetJointSimCheckType(jointId, JointType.Weld).joint).angularHertz;

    ///<summary> Set weld joint angular damping ratio, non-dimensional</summary>
    public static void WeldJoint_SetAngularDampingRatio(JointID jointId, float dampingRatio)
    {
        Debug.Assert(float.IsFinite(dampingRatio) && dampingRatio >= 0);
        ((WeldJoint)GetJointSimCheckType(jointId, JointType.Weld).joint).angularDampingRatio = dampingRatio;
    }

    ///<summary> Get the weld joint angular damping ratio, non-dimensional</summary>
    public static float WeldJoint_GetAngularDampingRatio(JointID jointId) =>
        ((WeldJoint)GetJointSimCheckType(jointId, JointType.Weld).joint).angularDampingRatio;

    ///<summary>Create a wheel joint
    /// @see WheelJointDef for details</summary>
    public static JointID CreateWheelJoint(WorldID worldId, ref WheelJointDef def) => WheelJoint.Create(worldId, ref def);

    ///<summary> Enable/disable the wheel joint spring</summary>
    public static void WheelJoint_EnableSpring(JointID jointId, bool enableSpring)
    {
        WheelJoint joint = (WheelJoint)GetJointSimCheckType(jointId, JointType.Wheel).joint;
        if (enableSpring != joint.enableSpring) { joint.enableSpring = enableSpring; joint.springImpulse = 0; }
    }

    ///<summary> Is the wheel joint spring enabled?</summary>
    public static bool WheelJoint_IsSpringEnabled(JointID jointId) =>
        ((WheelJoint)GetJointSimCheckType(jointId, JointType.Wheel).joint).enableSpring;

    ///<summary> Set the wheel joint stiffness in Hertz</summary>
    public static void WheelJoint_SetSpringHertz(JointID jointId, float hertz) =>
        ((WheelJoint)GetJointSimCheckType(jointId, JointType.Wheel).joint).hertz = hertz;

    ///<summary> Get the wheel joint stiffness in Hertz</summary>
    public static float WheelJoint_GetSpringHertz(JointID jointId) =>
        ((WheelJoint)GetJointSimCheckType(jointId, JointType.Wheel).joint).hertz;

    ///<summary> Set the wheel joint damping ratio, non-dimensional</summary>
    public static void WheelJoint_SetSpringDampingRatio(JointID jointId, float dampingRatio) =>
        ((WheelJoint)GetJointSimCheckType(jointId, JointType.Wheel).joint).dampingRatio = dampingRatio;

    ///<summary> Get the wheel joint damping ratio, non-dimensional</summary>
    public static float WheelJoint_GetSpringDampingRatio(JointID jointId) =>
        ((WheelJoint)GetJointSimCheckType(jointId, JointType.Wheel).joint).dampingRatio;

    ///<summary> Enable/disable the wheel joint limit</summary>
    public static void WheelJoint_EnableLimit(JointID jointId, bool enableLimit)
    {
        WheelJoint joint = (WheelJoint)GetJointSimCheckType(jointId, JointType.Wheel).joint;
        if (joint.enableLimit != enableLimit) { joint.lowerImpulse = 0; joint.upperImpulse = 0; joint.enableLimit = false; }
    }

    ///<summary> Is the wheel joint limit enabled?</summary>
    public static bool WheelJoint_IsLimitEnabled(JointID jointId) =>
        ((WheelJoint)GetJointSimCheckType(jointId, JointType.Wheel).joint).enableLimit;

    ///<summary> Get the wheel joint lower limit</summary>
    public static float WheelJoint_GetLowerLimit(JointID jointId) =>
        ((WheelJoint)GetJointSimCheckType(jointId, JointType.Wheel).joint).lowerTranslation;

    ///<summary> Get the wheel joint upper limit</summary>
    public static float WheelJoint_GetUpperLimit(JointID jointId) =>
        ((WheelJoint)GetJointSimCheckType(jointId, JointType.Wheel).joint).upperTranslation;

    ///<summary> Set the wheel joint limits</summary>
    public static void WheelJoint_SetLimits(JointID jointId, float lower, float upper)
    {
        Debug.Assert(lower <= upper);
        WheelJoint joint = (WheelJoint)GetJointSimCheckType(jointId, JointType.Wheel).joint;
        if (lower != joint.lowerTranslation || upper != joint.upperTranslation)
        {
            joint.lowerTranslation = Math.Min(lower, upper);
            joint.upperTranslation = Math.Max(lower, upper);
            joint.lowerImpulse = 0;
            joint.upperImpulse = 0;
        }
    }

    ///<summary> Enable/disable the wheel joint motor</summary>
    public static void WheelJoint_EnableMotor(JointID jointId, bool enableMotor)
    {
        WheelJoint joint = (WheelJoint)GetJointSimCheckType(jointId, JointType.Wheel).joint;
        if (joint.enableMotor != enableMotor) { joint.motorImpulse = 0; joint.enableMotor = enableMotor; }
    }

    ///<summary> Is the wheel joint motor enabled?</summary>
    public static bool WheelJoint_IsMotorEnabled(JointID jointId) =>
        ((WheelJoint)GetJointSimCheckType(jointId, JointType.Wheel).joint).enableMotor;

    ///<summary> Set the wheel joint motor speed in radians per second</summary>
    public static void WheelJoint_SetMotorSpeed(JointID jointId, float motorSpeed) =>
        ((WheelJoint)GetJointSimCheckType(jointId, JointType.Wheel).joint).motorSpeed = motorSpeed;

    ///<summary> Get the wheel joint motor speed in radians per second</summary>
    public static float WheelJoint_GetMotorSpeed(JointID jointId) =>
        ((WheelJoint)GetJointSimCheckType(jointId, JointType.Wheel).joint).motorSpeed;

    ///<summary> Set the wheel joint maximum motor torque, usually in newton-meters</summary>
    public static void WheelJoint_SetMaxMotorTorque(JointID jointId, float torque) =>
        ((WheelJoint)GetJointSimCheckType(jointId, JointType.Wheel).joint).maxMotorTorque = torque;

    ///<summary> Get the wheel joint maximum motor torque, usually in newton-meters</summary>
    public static float WheelJoint_GetMaxMotorTorque(JointID jointId) =>
        ((WheelJoint)GetJointSimCheckType(jointId, JointType.Wheel).joint).maxMotorTorque;

    ///<summary> Get the wheel joint current motor torque, usually in newton-meters</summary>
    public static float WheelJoint_GetMotorTorque(JointID jointId) =>
        jointId.world0.inv_h * ((WheelJoint)GetJointSimCheckType(jointId, JointType.Wheel).joint).motorImpulse;
}
