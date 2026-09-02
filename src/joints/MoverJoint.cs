using System;
using System.Diagnostics;

namespace Box2D;

public unsafe record class MoverJoint : IJoint
{
    public Vector2 linearVelocity;
    public Vector2 maxVelocityForce;
    public Vector2 linearVelocityImpulse;
    public int indexA;
    public int indexB;
    public Transform frameA;
    public Transform frameB;
    public float linearMass;
    public static JointID Create(WorldID worldId, ref MoverJointDef def)
    {
        Debug.Assert(def.internalValue == Box2D.SECRET_COOKIE);
        World world = worldId.index1;
        Debug.Assert(!world.locked);
        if (world.locked) return new();
        JointPair pair = world.CreateJoint(ref def.base_, JointType.Mover);
        pair.jointSim.joint = new MoverJoint
        {
            linearVelocity = def.linearVelocity,
            maxVelocityForce = def.maxVelocityForce
        };
        return new() { index1 = pair.jointSim.jointId + 1, world0 = world, generation = pair.joint.generation };
    }
    public void GetReaction(out float linearImpulse, out float angularImpulse)
    { linearImpulse = linearVelocityImpulse.Length(); angularImpulse = 0; }
    public Vector2 GetForce(World world, JointSim base_) => world.inv_h * linearVelocityImpulse;
    public void Prepare(JointSim joint, StepContext context)
    {
        Debug.Assert(joint.type == JointType.Mover);
        int idA = joint.bodyIdA, idB = joint.bodyIdB;
        World world = context.world;
        Body bodyA = world.bodies[idA], bodyB = world.bodies[idB];
        Debug.Assert(bodyA.setIndex == (int)SetType.Awake || bodyB.setIndex == (int)SetType.Awake);
        SolverSet setA = world.solverSets[bodyA.setIndex];
        SolverSet setB = world.solverSets[bodyB.setIndex];
        int localIndexA = bodyA.localIndex, localIndexB = bodyB.localIndex;
        BodySim bodySimA = setA.bodySims[localIndexA], bodySimB = setB.bodySims[localIndexB];
        float mA = bodySimA.invMass;
        float mB = bodySimB.invMass;
        joint.invMassA = mA; joint.invMassB = mB;
        joint.invIA = 0; joint.invIB = 0;
        indexA = bodyA.setIndex == (int)SetType.Awake ? localIndexA : -1;
        indexB = bodyB.setIndex == (int)SetType.Awake ? localIndexB : -1;
        float k = mA + mB;
        linearMass = k > 0 ? 1 / k : 0;
        if (!context.enableWarmStarting) linearVelocityImpulse = Vector2.Zero;
    }
    public void WarmStart(JointSim joint, StepContext context)
    {
        Debug.Assert(joint.type == JointType.Mover);
        float mA = joint.invMassA, mB = joint.invMassB;
        BodyState* stateA = BodyState.IdentityPtr; if (indexA != -1) stateA = context.states.Data + indexA;
        BodyState* stateB = BodyState.IdentityPtr; if (indexB != -1) stateB = context.states.Data + indexB;
        if (stateA->flags.HasFlag(BodyFlags.Dynamic)) stateA->linearVelocity = Vector2.MulSub(stateA->linearVelocity, mA, linearVelocityImpulse);
        if (stateB->flags.HasFlag(BodyFlags.Dynamic)) stateB->linearVelocity = Vector2.MulSub(stateB->linearVelocity, mB, linearVelocityImpulse);
    }
    public void Solve(JointSim joint, StepContext context, bool useBias)
    {
        Debug.Assert(joint.type == JointType.Mover);
        float mA = joint.invMassA, mB = joint.invMassB;

        BodyState* stateA = BodyState.IdentityPtr; if (indexA != -1) stateA = context.states.Data + indexA;
        BodyState* stateB = BodyState.IdentityPtr; if (indexB != -1) stateB = context.states.Data + indexB;
        Vector2 vA = stateA->linearVelocity;
        Vector2 vB = stateB->linearVelocity;
        if (maxVelocityForce.x > 0 || maxVelocityForce.y > 0)
        {
            Vector2 cdot = vB - vA - linearVelocity;
            Vector2 b = linearMass * cdot;
            Vector2 impulse = -b;
            Vector2 oldImpulse = linearVelocityImpulse;
            linearVelocityImpulse += impulse;
            Vector2 maxImpulse = context.h * maxVelocityForce;
            linearVelocityImpulse = Vector2.Clamp(linearVelocityImpulse, -maxImpulse, maxImpulse);
            impulse = linearVelocityImpulse - oldImpulse;
            vA = Vector2.MulSub(vA, mA, impulse);
            vB = Vector2.MulAdd(vB, mB, impulse);
        }
        else linearVelocityImpulse = Vector2.Zero;
        if (stateA->flags.HasFlag(BodyFlags.Dynamic)) stateA->linearVelocity = vA;
        if (stateB->flags.HasFlag(BodyFlags.Dynamic)) stateB->linearVelocity = vB;
    }
    public void Draw(DebugDraw draw, JointSim jointSim, WorldTransform transformA, WorldTransform transformB,
        Position pA, Position pB, float drawSize, HexColor color)
    {
        draw.DrawPointFcn(pB, 8, HexColor.Plum, draw.context);
    }
    public void HashStateDeep(ref ulong hash)
    {
        hash = Box2D.FnvMixFloat(hash, linearVelocityImpulse.x);
        hash = Box2D.FnvMixFloat(hash, linearVelocityImpulse.y);
    }
    public IJoint Copy() => new MoverJoint(this);
}
