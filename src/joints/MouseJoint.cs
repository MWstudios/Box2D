using System;
using System.Diagnostics;

namespace Box2D;

public unsafe record class MouseJoint : IJoint
{
    public float hertz;
    public float dampingRatio;
    public float maxForce;

    public Vector2 linearImpulse;
    public float angularImpulse;

    public Softness linearSoftness;
    public Softness angularSoftness;
    public int indexA;
    public int indexB;
    public Transform frameA;
    public Transform frameB;
    public Vector2 deltaCenter;
    public Mat22 linearMass;
    public float angularMass;
    public static JointID Create(WorldID worldId, ref MouseJointDef def)
    {
        Debug.Assert(def.internalValue == Box2D.SECRET_COOKIE);
        World world = worldId.index1;
        Debug.Assert(!world.locked);
        if (world.locked) return new();
        JointPair pair = world.CreateJoint(ref def.base_, JointType.Mouse);
        pair.jointSim.joint = new MouseJoint
        {
            hertz = def.hertz,
            dampingRatio = def.dampingRatio,
            maxForce = def.maxForce,
        };
        return new() { index1 = pair.jointSim.jointId + 1, world0 = world, generation = pair.joint.generation };
    }
    public void GetReaction(out float linearImpulse, out float angularImpulse)
    { linearImpulse = this.linearImpulse.Length(); angularImpulse = Math.Abs(this.angularImpulse); }
    public Vector2 GetForce(World world, JointSim base_) => world.inv_h * linearImpulse;
    public float GetTorque(World world, JointSim base_) => world.inv_h * angularImpulse;
    public void Prepare(JointSim joint, StepContext context)
    {
        Debug.Assert(joint.type == JointType.Mouse);
        int idA = joint.bodyIdA, idB = joint.bodyIdB;
        World world = context.world;
        Body bodyA = world.bodies[idA], bodyB = world.bodies[idB];
        Debug.Assert(bodyA.setIndex == (int)SetType.Awake || bodyB.setIndex == (int)SetType.Awake);
        SolverSet setA = world.solverSets[bodyA.setIndex], setB = world.solverSets[bodyB.setIndex];
        int localIndexA = bodyA.localIndex, localIndexB = bodyB.localIndex;
        BodySim bodySimA = setA.bodySims[localIndexA], bodySimB = setB.bodySims[localIndexB];
        float mA = bodySimA.invMass, iA = bodySimA.invInertia;
        float mB = bodySimB.invMass, iB = bodySimB.invInertia;
        joint.invMassA = mA; joint.invMassB = mB;
        joint.invIA = iA; joint.invIB = iB;
        indexA = bodyA.setIndex == (int)SetType.Awake ? localIndexA : -1;
        indexB = bodyB.setIndex == (int)SetType.Awake ? localIndexB : -1;
        frameA.q = bodySimA.transform.q * joint.localFrameA.q;
        frameA.p = bodySimA.transform.q * (joint.localFrameA.p - bodySimA.localCenter);
        frameB.q = bodySimB.transform.q * joint.localFrameB.q;
        frameB.p = bodySimB.transform.q * (joint.localFrameB.p - bodySimB.localCenter);
        deltaCenter = bodySimB.center - bodySimA.center;
        linearSoftness = new(hertz, dampingRatio, context.h);
        float angularHertz = 0.5f, angularDampingRatio = 1;
        angularSoftness = new(angularHertz, angularDampingRatio, context.h);
        Vector2 rA = frameA.p, rB = frameB.p;
        Mat22 K = new(new(mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB, -rA.y * rA.x * iA - rB.y * rB.x * iB),
            new(0, mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB));
        K.cy.x = K.cx.y;
        linearMass = K.Inverse();
        float ka = iA + iB;
        angularMass = ka > 0 ? 1 / ka : 0;
        if (!context.enableWarmStarting) { linearImpulse = Vector2.Zero; angularImpulse = 0; }
    }
    public void WarmStart(JointSim joint, StepContext context)
    {
        Debug.Assert(joint.type == JointType.Mouse);
        float mA = joint.invMassA, mB = joint.invMassB;
        float iA = joint.invIA, iB = joint.invIB;
        BodyState dummyState = new();
        BodyState* stateA = &dummyState; if (indexA != -1) stateA = context.states.Data + indexA;
        BodyState* stateB = &dummyState; if (indexB != -1) stateB = context.states.Data + indexB;
        Vector2 rA = stateA->deltaRotation * frameA.p, rB = stateB->deltaRotation * frameB.p;
        stateA->linearVelocity = Vector2.MulSub(stateA->linearVelocity, mA, linearImpulse);
        stateA->angularVelocity -= iA * (Vector2.Cross(rA, linearImpulse) + angularImpulse);
        stateB->linearVelocity = Vector2.MulAdd(stateB->linearVelocity, mB, linearImpulse);
        stateB->angularVelocity += iB * (Vector2.Cross(rB, linearImpulse) + angularImpulse);
    }
    public void Solve(JointSim joint, StepContext context, bool useBias)
    {
        Debug.Assert(joint.type == JointType.Mouse);
        float mA = joint.invMassA, mB = joint.invMassB;
        float iA = joint.invIA, iB = joint.invIB;
        BodyState dummyState = new();
        BodyState* stateA = &dummyState; if (indexA != -1) stateA = context.states.Data + indexA;
        BodyState* stateB = &dummyState; if (indexB != -1) stateB = context.states.Data + indexB;
        Vector2 vA = stateA->linearVelocity, vB = stateB->linearVelocity;
        float wA = stateA->angularVelocity, wB = stateB->angularVelocity;
        {
            float massScale = angularSoftness.massScale;
            float impulseScale = angularSoftness.impulseScale;
            float Cdot = wB - wA;
            float impulse = -massScale * angularMass * Cdot - impulseScale * angularImpulse;
            angularImpulse += impulse;
            wA -= iA * impulse;
            wB += iB * impulse;
        }
        float maxImpulse = maxForce * context.h;
        {
            Vector2 rA = stateA->deltaRotation * frameA.p, rB = stateB->deltaRotation * frameB.p;
            Vector2 Cdot = (vB + Vector2.CrossSV(wB, rB)) - (vA + Vector2.CrossSV(wA, rA));
            Vector2 dcA = stateA->deltaPosition, dcB = stateB->deltaPosition;
            Vector2 C = dcB - dcA + (rB - rA) + deltaCenter;
            Vector2 bias = linearSoftness.biasRate * C;
            float massScale = linearSoftness.massScale;
            float impulseScale = linearSoftness.impulseScale;
            Vector2 b = linearMass * (Cdot + bias);
            Vector2 impulse = -massScale * b - impulseScale * linearImpulse;
            Vector2 oldImpulse = linearImpulse;
            linearImpulse += impulse;
            if (linearImpulse.LengthSquared() > maxImpulse * maxImpulse)
                linearImpulse = maxImpulse * linearImpulse.Normalize();
            impulse = linearImpulse - oldImpulse;
            vA = Vector2.MulSub(vA, mA, impulse);
            wA -= iA * Vector2.Cross(rA, impulse);
            vB = Vector2.MulAdd(vB, mB, impulse);
            wB += iB * Vector2.Cross(rB, impulse);
        }
        stateA->linearVelocity = vA;
        stateA->angularVelocity = wA;
        stateB->linearVelocity = vB;
        stateB->angularVelocity = wB;
    }
    public void Draw(DebugDraw draw, JointSim jointSim, Transform transformA, Transform transformB,
        Vector2 pA, Vector2 pB, float drawSize, HexColor color)
    {
        draw.DrawPointFcn(pA, 8, HexColor.YellowGreen, draw.context);
        draw.DrawPointFcn(pB, 8, HexColor.YellowGreen, draw.context);
        draw.DrawSegmentFcn(pA, pB, HexColor.LightGray, draw.context);
    }
    public IJoint Copy() => new MouseJoint(this);
}
