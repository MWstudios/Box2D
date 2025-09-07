using System;
using System.Diagnostics;

namespace Box2D;

public unsafe record class MotorJoint : IJoint
{
    public Vector2 linearVelocity;
    public float maxVelocityForce;
    public float angularVelocity;
    public float maxVelocityTorque;
    public float linearHertz;
    public float linearDampingRatio;
    public float maxSpringForce;
    public float angularHertz;
    public float angularDampingRatio;
    public float maxSpringTorque;
    public Vector2 linearVelocityImpulse;
    public float angularVelocityImpulse;
    public Vector2 linearSpringImpulse;
    public float angularSpringImpulse;
    public Softness linearSpring;
    public Softness angularSpring;

    public int indexA;
    public int indexB;
    public Transform frameA;
    public Transform frameB;
    public Vector2 deltaCenter;
    public Mat22 linearMass;
    public float angularMass;
    public static JointID Create(WorldID worldId, ref MotorJointDef def)
    {
        Debug.Assert(def.internalValue == Box2D.SECRET_COOKIE);
        World world = worldId.index1;
        Debug.Assert(!world.locked);
        if (world.locked) return new();
        JointPair pair = world.CreateJoint(ref def.base_, JointType.Motor);
        pair.jointSim.joint = new MotorJoint
        {
            linearVelocity = def.linearVelocity,
            maxVelocityForce = def.maxVelocityForce,
            angularVelocity = def.angularVelocity,
            maxVelocityTorque = def.maxVelocityTorque,
            linearHertz = def.linearHertz,
            linearDampingRatio = def.linearDampingRatio,
            maxSpringForce = def.maxSpringForce,
            angularHertz = def.angularHertz,
            angularDampingRatio = def.angularDampingRatio,
            maxSpringTorque = def.maxSpringTorque
        };
        return new() { index1 = pair.jointSim.jointId + 1, world0 = world, generation = pair.joint.generation };
    }
    public void GetReaction(out float linearImpulse, out float angularImpulse)
    {
        linearImpulse = (linearVelocityImpulse + linearSpringImpulse).Length();
        angularImpulse = Math.Abs(angularVelocityImpulse + angularSpringImpulse);
    }
    public Vector2 GetForce(World world, JointSim base_) => world.inv_h * (linearVelocityImpulse + linearSpringImpulse);
    public float GetTorque(World world, JointSim base_) => world.inv_h * (angularVelocityImpulse + angularSpringImpulse);
    public void Prepare(JointSim joint, StepContext context)
    {
        Debug.Assert(joint.type == JointType.Motor);
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
        Vector2 rA = frameA.p, rB = frameB.p;
        linearSpring = new(linearHertz, linearDampingRatio, context.h);
        angularSpring = new(angularHertz, angularDampingRatio, context.h);
        Mat22 kl = new(new(mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB, -rA.y * rA.x * iA - rB.y * rB.x * iB),
            new(0, mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB));
        kl.cy.x = kl.cx.y;
        linearMass = kl.Inverse();
        float ka = iA + iB;
        angularMass = ka > 0 ? 1 / ka : 0;
        if (!context.enableWarmStarting)
        {
            linearVelocityImpulse = Vector2.Zero;
            angularVelocityImpulse = 0;
            linearSpringImpulse = Vector2.Zero;
            angularSpringImpulse = 0;
        }
    }
    public void WarmStart(JointSim joint, StepContext context)
    {
        Debug.Assert(joint.type == JointType.Motor);
        float mA = joint.invMassA, mB = joint.invMassB;
        float iA = joint.invIA, iB = joint.invIB;
        BodyState dummyState = new();
        BodyState* stateA = &dummyState; if (indexA != -1) stateA = context.states.Data + indexA;
        BodyState* stateB = &dummyState; if (indexB != -1) stateB = context.states.Data + indexB;
        Vector2 rA = stateA->deltaRotation * frameA.p, rB = stateB->deltaRotation * frameB.p;
        Vector2 linearImpulse = linearVelocityImpulse + linearSpringImpulse;
        float angularImpulse = angularVelocityImpulse + angularSpringImpulse;
        if (stateA->flags.HasFlag(BodyFlags.Dynamic))
        {
            stateA->linearVelocity = Vector2.MulSub(stateA->linearVelocity, mA, linearImpulse);
            stateA->angularVelocity -= iA * (Vector2.Cross(rA, linearImpulse) + angularImpulse);
        }
        if (stateB->flags.HasFlag(BodyFlags.Dynamic))
        {
            stateB->linearVelocity = Vector2.MulAdd(stateB->linearVelocity, mB, linearImpulse);
            stateB->angularVelocity += iB * (Vector2.Cross(rB, linearImpulse) + angularImpulse);
        }
    }
    public void Solve(JointSim joint, StepContext context, bool useBias)
    {
        Debug.Assert(joint.type == JointType.Motor);
        float mA = joint.invMassA, mB = joint.invMassB;
        float iA = joint.invIA, iB = joint.invIB;
        BodyState dummyState = new();
        BodyState* stateA = &dummyState; if (indexA != -1) stateA = context.states.Data + indexA;
        BodyState* stateB = &dummyState; if (indexB != -1) stateB = context.states.Data + indexB;
        Vector2 vA = stateA->linearVelocity, vB = stateB->linearVelocity;
        float wA = stateA->angularVelocity, wB = stateB->angularVelocity;
        if (maxSpringTorque > 0 && angularHertz > 0)
        {
            Rotation qA = stateA->deltaRotation * frameA.q, qB = stateB->deltaRotation * frameB.q;
            Rotation relQ = Rotation.InvMulRot(qA, qB);
            float c = relQ.GetAngle();
            float bias = angularSpring.biasRate * c;
            float cdot = wB - wA;
            float maxImpulse = context.h * maxSpringTorque;
            float oldImpulse = angularSpringImpulse;
            float impulse = -angularSpring.massScale * angularMass * (cdot + bias) - angularSpring.impulseScale * oldImpulse;
            angularSpringImpulse = Math.Clamp(oldImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = angularSpringImpulse - oldImpulse;
            wA -= iA * impulse;
            wB += iB * impulse;
        }
        if (maxVelocityTorque > 0)
        {
            float cdot = wB - wA - angularVelocity;
            float impulse = -angularMass * cdot;
            float maxImpulse = context.h * maxVelocityTorque;
            float oldImpulse = angularVelocityImpulse;
            angularVelocityImpulse = Math.Clamp(oldImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = angularVelocityImpulse - oldImpulse;
            wA -= iA * impulse;
            wB += iB * impulse;
        }
        Vector2 rA = stateA->deltaRotation * frameA.p;
        Vector2 rB = stateB->deltaRotation * frameB.p;
        if (maxSpringForce > 0 && linearHertz > 0)
        {
            Vector2 dcA = stateA->deltaPosition;
            Vector2 dcB = stateB->deltaPosition;
            Vector2 c = (dcB - dcA) + (rB - rA) + deltaCenter;
            Vector2 bias = linearSpring.biasRate * c;
            Vector2 cdot = vB + Vector2.CrossSV(wB, rB) - (vA + Vector2.CrossSV(wA, rA));
            cdot += bias;
            Mat22 kl = new(new(mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB, -rA.y * rA.x * iA - rB.y * rB.x * iB),
                new(0, mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB));
            kl.cy.x = kl.cx.y;
            linearMass = kl.Inverse();
            Vector2 b = linearMass * cdot;
            Vector2 oldImpulse = linearSpringImpulse;
            Vector2 impulse = -linearSpring.massScale * b - linearSpring.impulseScale * oldImpulse;
            float maxImpulse = context.h * maxSpringForce;
            linearSpringImpulse += impulse;
            if (linearSpringImpulse.LengthSquared() > maxImpulse * maxImpulse)
            {
                linearSpringImpulse = linearSpringImpulse.Normalize();
                linearSpringImpulse *= maxImpulse;
            }
            impulse = linearSpringImpulse - oldImpulse;
            vA = Vector2.MulSub(vA, mA, impulse);
            wA -= iA * Vector2.Cross(rA, impulse);
            vB = Vector2.MulAdd(vB, mB, impulse);
            wB += iB * Vector2.Cross(rB, impulse);
        }
        if (maxVelocityForce > 0)
        {
            Vector2 cdot = vB + Vector2.CrossSV(wB, rB) - (vA + Vector2.CrossSV(wA, rA));
            cdot -= linearVelocity;
            Vector2 b = linearMass * cdot;
            Vector2 impulse = -b;
            Vector2 oldImpulse = linearVelocityImpulse;
            float maxImpulse = context.h * maxVelocityForce;
            linearVelocityImpulse += impulse;
            if (linearVelocityImpulse.LengthSquared() > maxImpulse * maxImpulse)
            {
                linearVelocityImpulse = linearVelocityImpulse.Normalize();
                linearVelocityImpulse *= maxImpulse;
            }
            impulse = linearVelocityImpulse - oldImpulse;
            vA = Vector2.MulSub(vA, mA, impulse);
            wA -= iA * Vector2.Cross(rA, impulse);
            vB = Vector2.MulAdd(vB, mB, impulse);
            wB += iB * Vector2.Cross(rB, impulse);
        }
        if (stateA->flags.HasFlag(BodyFlags.Dynamic))
        {
            stateA->linearVelocity = vA;
            stateA->angularVelocity = wA;
        }
        if (stateB->flags.HasFlag(BodyFlags.Dynamic))
        {
            stateB->linearVelocity = vB;
            stateB->angularVelocity = wB;
        }
    }
    public void Draw(DebugDraw draw, JointSim jointSim, Transform transformA, Transform transformB,
        Vector2 pA, Vector2 pB, float drawScale, HexColor color)
    {
        draw.DrawPointFcn(pA, 8, HexColor.YellowGreen, draw.context);
        draw.DrawPointFcn(pB, 8, HexColor.Plum, draw.context);
    }
    public IJoint Copy() => new MotorJoint(this);
}