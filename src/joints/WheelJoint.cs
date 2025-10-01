using System;
using System.Diagnostics;

namespace Box2D;

public unsafe record class WheelJoint : IJoint
{
    public float perpImpulse;
    public float motorImpulse;
    public float springImpulse;
    public float lowerImpulse;
    public float upperImpulse;
    public float maxMotorTorque;
    public float motorSpeed;
    public float lowerTranslation;
    public float upperTranslation;
    public float hertz;
    public float dampingRatio;

    public int indexA;
    public int indexB;
    public Transform frameA;
    public Transform frameB;
    public Vector2 deltaCenter;
    public float perpMass;
    public float motorMass;
    public float axialMass;
    public Softness springSoftness;

    public bool enableSpring;
    public bool enableMotor;
    public bool enableLimit;
    public static JointID Create(WorldID worldId, ref WheelJointDef def)
    {
        Debug.Assert(def.internalValue == Box2D.SECRET_COOKIE);
        Debug.Assert(def.lowerTranslation <= def.upperTranslation);
        World world = worldId.index1;
        Debug.Assert(!world.locked);
        if (world.locked) return new();
        JointPair pair = world.CreateJoint(ref def.base_, JointType.Wheel);
        pair.jointSim.joint = new WheelJoint
        {
            perpMass = 0,
            axialMass = 0,
            motorImpulse = 0,
            lowerImpulse = 0,
            upperImpulse = 0,
            lowerTranslation = def.lowerTranslation,
            upperTranslation = def.upperTranslation,
            maxMotorTorque = def.maxMotorTorque,
            motorSpeed = def.motorSpeed,
            hertz = def.hertz,
            dampingRatio = def.dampingRatio,
            enableSpring = def.enableSpring,
            enableLimit = def.enableLimit,
            enableMotor = def.enableMotor,
        };
        return new() { index1 = pair.jointSim.jointId + 1, world0 = world, generation = pair.joint.generation };
    }
    public void GetReaction(out float linearImpulse, out float angularImpulse)
    {
        float perpImpulse = this.perpImpulse;
        float axialImpulse = springImpulse + lowerImpulse - upperImpulse;
        linearImpulse = MathF.Sqrt(perpImpulse * perpImpulse + axialImpulse * axialImpulse);
        angularImpulse = Math.Abs(motorImpulse);
    }
    public float GetLinearSeparation(Transform xfA, Transform xfB, Vector2 dp)
    {
        Vector2 axisA = xfA.q * new Vector2(1, 0), perpA = axisA.LeftPerp();
        float perpendicularSeparation = Math.Abs(Vector2.Dot(perpA, dp));
        float limitSeparation = 0;
        if (enableLimit)
        {
            float translation = Vector2.Dot(axisA, dp);
            if (translation < lowerTranslation) limitSeparation = lowerTranslation - translation;
            if (upperTranslation < translation) limitSeparation = translation - upperTranslation;
        }
        return MathF.Sqrt(perpendicularSeparation * perpendicularSeparation + limitSeparation * limitSeparation);
    }
    public Vector2 GetForce(World world, JointSim base_)
    {
        int idA = base_.bodyIdA;
        Transform transformA = world.GetBodyTransform(idA);
        Vector2 localAxisA = base_.localFrameA.q * new Vector2(1, 0);
        Vector2 axisA = transformA.q * localAxisA;
        Vector2 perpA = axisA.LeftPerp();
        float perpForce = world.inv_h * perpImpulse;
        float axialForce = world.inv_h * (springImpulse + lowerImpulse - upperImpulse);
        return perpForce * perpA + axialForce * axisA;
    }
    public float GetTorque(World world, JointSim base_) => world.inv_h * motorImpulse;
    public void Prepare(JointSim joint, StepContext context)
    {
        Debug.Assert(joint.type == JointType.Wheel);
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
        Vector2 d = deltaCenter + (rB - rA);
        Vector2 axisA = frameA.q * new Vector2(1, 0);
        Vector2 perpA = axisA.LeftPerp();
        float s1 = Vector2.Cross(d + rA, perpA);
        float s2 = Vector2.Cross(rB, perpA);
        float kp = mA + mB + iA * s1 * s1 + iB * s2 * s2;
        perpMass = kp > 0 ? 1 / kp : 0;
        float a1 = Vector2.Cross(d + rA, axisA);
        float a2 = Vector2.Cross(rB, axisA);
        float ka = mA + mB + iA * a1 * a1 + iB * a2 * a2;
        axialMass = ka > 0 ? 1 / ka : 0;
        springSoftness = new(hertz, dampingRatio, context.h);
        float km = iA + iB;
        motorMass = km > 0 ? 1 / km : 0;
        if (!context.enableWarmStarting)
        {
            perpImpulse = 0;
            springImpulse = 0;
            motorImpulse = 0;
            lowerImpulse = 0;
            upperImpulse = 0;
        }
    }
    public void WarmStart(JointSim joint, StepContext context)
    {
        Debug.Assert(joint.type == JointType.Wheel);
        float mA = joint.invMassA, mB = joint.invMassB;
        float iA = joint.invIA, iB = joint.invIB;
        BodyState dummyState = new();
        BodyState* stateA = &dummyState; if (indexA != -1) stateA = context.states.Data + indexA;
        BodyState* stateB = &dummyState; if (indexB != -1) stateB = context.states.Data + indexB;
        Vector2 rA = stateA->deltaRotation * frameA.p, rB = stateB->deltaRotation * frameB.p;
        Vector2 d = stateB->deltaPosition - stateA->deltaPosition + deltaCenter + (rB - rA);
        Vector2 axisA = frameA.q * new Vector2(1, 0);
        axisA = stateA->deltaRotation * axisA;
        Vector2 perpA = axisA.LeftPerp();
        float a1 = Vector2.Cross(d + rA, axisA);
        float a2 = Vector2.Cross(rB, axisA);
        float s1 = Vector2.Cross(d + rA, perpA);
        float s2 = Vector2.Cross(rB, perpA);
        float axialImpulse = springImpulse + lowerImpulse - upperImpulse;
        Vector2 P = axialImpulse * axisA + perpImpulse * perpA;
        float LA = axialImpulse * a1 + perpImpulse * s1 + motorImpulse;
        float LB = axialImpulse * a2 + perpImpulse * s2 + motorImpulse;
        if (stateA->flags.HasFlag(BodyFlags.Dynamic))
        {
            stateA->linearVelocity = Vector2.MulSub(stateA->linearVelocity, mA, P);
            stateA->angularVelocity -= iA * LA;
        }
        if (stateB->flags.HasFlag(BodyFlags.Dynamic))
        {
            stateB->linearVelocity = Vector2.MulAdd(stateB->linearVelocity, mB, P);
            stateB->angularVelocity += iB * LB;
        }
    }
    public void Solve(JointSim joint, StepContext context, bool useBias)
    {
        Debug.Assert(joint.type == JointType.Wheel);
        float mA = joint.invMassA, mB = joint.invMassB;
        float iA = joint.invIA, iB = joint.invIB;
        BodyState dummyState = new();
        BodyState* stateA = &dummyState; if (indexA != -1) stateA = context.states.Data + indexA;
        BodyState* stateB = &dummyState; if (indexB != -1) stateB = context.states.Data + indexB;
        Vector2 vA = stateA->linearVelocity, vB = stateB->linearVelocity;
        float wA = stateA->angularVelocity, wB = stateB->angularVelocity;
        bool fixedRotation = iA + iB == 0;
        Vector2 rA = stateA->deltaRotation * frameA.p, rB = stateB->deltaRotation * frameB.p;
        Vector2 d = stateB->deltaPosition - stateA->deltaPosition + deltaCenter + (rB - rA);
        Vector2 axisA = frameA.q * new Vector2(1, 0);
        axisA = stateA->deltaRotation * axisA;
        float translation = Vector2.Dot(axisA, d);
        float a1 = Vector2.Cross(d + rA, axisA);
        float a2 = Vector2.Cross(rB, axisA);
        if (enableMotor && !fixedRotation)
        {
            float Cdot = wB - wA - motorSpeed;
            float impulse = -motorMass * Cdot;
            float oldImpulse = motorImpulse;
            float maxImpulse = context.h * maxMotorTorque;
            motorImpulse = Math.Clamp(motorImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = motorImpulse - oldImpulse;
            wA -= iA * impulse;
            wB += iB * impulse;
        }
        if (enableSpring)
        {
            float bias = springSoftness.biasRate * translation;
            float massScale = springSoftness.massScale;
            float impulseScale = springSoftness.impulseScale;
            float Cdot = Vector2.Dot(axisA, vB - vA) + a2 * wB - a1 * wA;
            float impulse = -massScale * axialMass * (Cdot + bias) - impulseScale * springImpulse;
            springImpulse += impulse;
            Vector2 P = impulse * axisA;
            float LA = impulse * a1, LB = impulse * a2;
            vA = Vector2.MulSub(vA, mA, P);
            wA -= iA * LA;
            vB = Vector2.MulAdd(vB, mB, P);
            wB += iB * LB;
        }
        if (enableLimit)
        {
            {
                float C = translation - lowerTranslation;
                float bias = 0, massScale = 1, impulseScale = 0;
                if (C > 0) bias = C * context.inv_h;
                else if (useBias)
                {
                    bias = joint.constraintSoftness.biasRate * C;
                    massScale = joint.constraintSoftness.massScale;
                    impulseScale = joint.constraintSoftness.impulseScale;
                }
                float Cdot = Vector2.Dot(axisA, vB - vA) + a2 * wB - a1 * wA;
                float impulse = -massScale * axialMass * (Cdot + bias) - impulseScale * lowerImpulse;
                float oldImpulse = lowerImpulse;
                lowerImpulse = Math.Max(oldImpulse + impulse, 0);
                impulse = lowerImpulse - oldImpulse;
                Vector2 P = impulse * axisA;
                float LA = impulse * a1, LB = impulse * a2;
                vA = Vector2.MulSub(vA, mA, P); wA -= iA * LA;
                vB = Vector2.MulAdd(vB, mB, P); wB += iB * LB;
            }
            {
                float C = upperTranslation - translation;
                float bias = 0, massScale = 1, impulseScale = 0;
                if (C > 0) bias = C * context.inv_h;
                else if (useBias)
                {
                    bias = joint.constraintSoftness.biasRate * C;
                    massScale = joint.constraintSoftness.massScale;
                    impulseScale = joint.constraintSoftness.impulseScale;
                }
                float Cdot = Vector2.Dot(axisA, vA - vB) + a1 * wA - a2 * wB;
                float impulse = -massScale * axialMass * (Cdot + bias) - impulseScale * upperImpulse;
                float oldImpulse = upperImpulse;
                upperImpulse = Math.Max(oldImpulse + impulse, 0);
                impulse = upperImpulse - oldImpulse;
                Vector2 P = impulse * axisA;
                float LA = impulse * a1, LB = impulse * a2;
                vA = Vector2.MulAdd(vA, mA, P); wA += iA * LA;
                vB = Vector2.MulSub(vB, mB, P); wB -= iB * LB;
            }
        }
        {
            Vector2 perpA = axisA.LeftPerp();
            float bias = 0, massScale = 1, impulseScale = 0;
            if (useBias)
            {
                bias = joint.constraintSoftness.biasRate * Vector2.Dot(perpA, d);
                massScale = joint.constraintSoftness.massScale;
                impulseScale = joint.constraintSoftness.impulseScale;
            }
            float s1 = Vector2.Cross(d + rA, perpA);
            float s2 = Vector2.Cross(rB, perpA);
            float Cdot = Vector2.Dot(axisA, vB - vA) + s2 * wB - s1 * wA;
            float impulse = -massScale * perpMass * (Cdot + bias) - impulseScale * perpImpulse;
            perpImpulse += impulse;
            Vector2 P = impulse * perpA;
            float LA = impulse * s1, LB = impulse * s2;
            vA = Vector2.MulSub(vA, mA, P); wA -= iA * LA;
            vB = Vector2.MulAdd(vB, mB, P); wB += iB * LB;
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
        Vector2 pA, Vector2 pB, float drawSize, HexColor color)
    {
        Debug.Assert(jointSim.type == JointType.Wheel);
        Transform frameA = transformA * jointSim.localFrameA;
        Transform frameB = transformB * jointSim.localFrameB;
        Vector2 axisA = frameA.q * new Vector2(1, 0);
        draw.DrawSegmentFcn(frameA.p, frameB.p, HexColor.Blue, draw.context);
        if (enableLimit)
        {
            Vector2 lower = Vector2.MulAdd(frameA.p, lowerTranslation, axisA);
            Vector2 upper = Vector2.MulAdd(frameA.p, upperTranslation, axisA);
            Vector2 perp = axisA.LeftPerp();
            draw.DrawSegmentFcn(lower, upper, HexColor.Gray, draw.context);
            draw.DrawSegmentFcn(Vector2.MulSub(lower, 0.1f * drawSize, perp), Vector2.MulAdd(lower, 0.1f * drawSize, perp), HexColor.Green, draw.context);
            draw.DrawSegmentFcn(Vector2.MulSub(upper, 0.1f * drawSize, perp), Vector2.MulAdd(upper, 0.1f * drawSize, perp), HexColor.Red, draw.context);
        }
        else draw.DrawSegmentFcn(Vector2.MulSub(frameA.p, 1, axisA), Vector2.MulAdd(frameA.p, 1, axisA), HexColor.Gray, draw.context);
        draw.DrawPointFcn(frameA.p, 5, HexColor.Gray, draw.context);
        draw.DrawPointFcn(frameB.p, 5, HexColor.DimGray, draw.context);
    }
    public IJoint Copy() => new WheelJoint(this);
}
