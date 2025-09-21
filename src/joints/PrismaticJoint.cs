using System;
using System.Diagnostics;

namespace Box2D;

public unsafe record class PrismaticJoint : IJoint
{
    public Vector2 impulse;
    public float springImpulse;
    public float motorImpulse;
    public float lowerImpulse;
    public float upperImpulse;
    public float hertz;
    public float dampingRatio;
    public float targetTranslation;
    public float maxMotorForce;
    public float motorSpeed;
    public float lowerTranslation;
    public float upperTranslation;

    public int indexA;
    public int indexB;
    public Transform frameA;
    public Transform frameB;
    public Vector2 deltaCenter;
    public Softness springSoftness;

    public bool enableSpring;
    public bool enableLimit;
    public bool enableMotor;
    public static JointID Create(WorldID worldId, ref PrismaticJointDef def)
    {
        Debug.Assert(def.internalValue == Box2D.SECRET_COOKIE);
        Debug.Assert(def.lowerTranslation <= def.upperTranslation);
        World world = worldId.index1;
        Debug.Assert(!world.locked);
        if (world.locked) return new();
        JointPair pair = world.CreateJoint(ref def.base_, JointType.Prismatic);
        pair.jointSim.joint = new PrismaticJoint
        {
            hertz = def.hertz,
            dampingRatio = def.dampingRatio,
            targetTranslation = def.targetTranslation,
            lowerTranslation = def.lowerTranslation,
            upperTranslation = def.upperTranslation,
            maxMotorForce = def.maxMotorForce,
            motorSpeed = def.motorSpeed,
            enableSpring = def.enableSpring,
            enableLimit = def.enableLimit,
            enableMotor = def.enableMotor,
        };
        return new() { index1 = pair.jointSim.jointId + 1, world0 = world, generation = pair.joint.generation };
    }
    public void GetReaction(out float linearImpulse, out float angularImpulse)
    {
        float perpImpulse = impulse.x;
        float axialImpulse = motorImpulse + lowerImpulse - upperImpulse;
        linearImpulse = MathF.Sqrt(perpImpulse * perpImpulse + axialImpulse * axialImpulse);
        angularImpulse = Math.Abs(impulse.y);
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
    public float GetAngularSeparation(float relativeAngle) => relativeAngle;
    public Vector2 GetForce(World world, JointSim base_)
    {
        int idA = base_.bodyIdA;
        Transform transformA = world.GetBodyTransform(idA);
        Vector2 localAxisA = base_.localFrameA.q * new Vector2(1, 0);
        Vector2 axisA = transformA.q * localAxisA;
        Vector2 perpA = axisA.LeftPerp();
        float inv_h = world.inv_h;
        float perpForce = inv_h * impulse.x;
        float axialForce = inv_h * (motorImpulse + lowerImpulse - upperImpulse);
        return perpForce * perpA + axialForce * axisA;
    }
    public float GetTorque(World world, JointSim base_) => world.inv_h * impulse.y;
    public void Prepare(JointSim joint, StepContext context)
    {
        Debug.Assert(joint.type == JointType.Prismatic);
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
        springSoftness = new(hertz, dampingRatio, context.h);
        if (!context.enableWarmStarting)
        {
            impulse = Vector2.Zero; springImpulse = 0;
            motorImpulse = 0; lowerImpulse = 0; upperImpulse = 0;
        }
    }
    public void WarmStart(JointSim joint, StepContext context)
    {
        Debug.Assert(joint.type == JointType.Prismatic);
        float mA = joint.invMassA, mB = joint.invMassB;
        float iA = joint.invIA, iB = joint.invIB;
        BodyState dummyState = new();
        BodyState* stateA = &dummyState; if (indexA != -1) stateA = context.states.Data + indexA;
        BodyState* stateB = &dummyState; if (indexB != -1) stateB = context.states.Data + indexB;
        Vector2 rA = stateA->deltaRotation * frameA.p, rB = stateB->deltaRotation * frameB.p;
        Vector2 d = stateB->deltaPosition - stateA->deltaPosition + deltaCenter + (rB - rA);
        Vector2 axisA = frameA.q * new Vector2(1, 0);
        axisA = stateA->deltaRotation * axisA;
        float a1 = Vector2.Cross(rA + d, axisA);
        float a2 = Vector2.Cross(rB, axisA);
        float axialImpulse = springImpulse + motorImpulse + lowerImpulse - upperImpulse;
        Vector2 perpA = axisA.LeftPerp();
        float s1 = Vector2.Cross(rA + d, perpA);
        float s2 = Vector2.Cross(rB, perpA);
        float perpImpulse = impulse.x, angleImpulse = impulse.y;
        Vector2 P = axialImpulse * axisA + perpImpulse * perpA;
        float LA = axialImpulse * a1 + perpImpulse * s1 + angleImpulse;
        float LB = axialImpulse * a2 + perpImpulse * s2 + angleImpulse;
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
        Debug.Assert(joint.type == JointType.Motor);
        float mA = joint.invMassA, mB = joint.invMassB;
        float iA = joint.invIA, iB = joint.invIB;
        BodyState dummyState = new();
        BodyState* stateA = &dummyState; if (indexA != -1) stateA = context.states.Data + indexA;
        BodyState* stateB = &dummyState; if (indexB != -1) stateB = context.states.Data + indexB;
        Vector2 vA = stateA->linearVelocity, vB = stateB->linearVelocity;
        float wA = stateA->angularVelocity, wB = stateB->angularVelocity;
        Rotation qA = stateA->deltaRotation * frameA.q, qB = stateB->deltaRotation * frameB.q;
        Rotation relQ = Rotation.InvMulRot(qA, qB);
        Vector2 rA = stateA->deltaRotation * frameA.p, rB = stateB->deltaRotation * frameB.p;
        Vector2 d = stateB->deltaPosition - stateA->deltaPosition + deltaCenter + (rB - rA);
        Vector2 axisA = frameA.q * new Vector2(1, 0);
        axisA = stateA->deltaRotation * axisA;
        float translation = Vector2.Dot(axisA, d);
        float a1 = Vector2.Cross(rA + d, axisA);
        float a2 = Vector2.Cross(rB, axisA);
        float k = mA + mB + iA * a1 * a1 + iB * a2 * a2;
        float axialMass = k > 0 ? 1 / k : 0;
        if (enableSpring)
        {
            float C = translation - targetTranslation;
            float bias = springSoftness.biasRate * C;
            float massScale = springSoftness.massScale;
            float impulseScale = springSoftness.impulseScale;
            float Cdot = Vector2.Dot(axisA, vB - vA) + a2 * wB - a1 * wA;
            float deltaImpulse = -massScale * axialMass * (Cdot + bias) - impulseScale * springImpulse;
            springImpulse += deltaImpulse;
            Vector2 P = deltaImpulse * axisA;
            float LA = deltaImpulse * a1, LB = deltaImpulse * a2;
            vA = Vector2.MulSub(vA, mA, P); wA -= iA * LA;
            vB = Vector2.MulAdd(vB, mB, P); wB += iB * LB;
        }
        if (enableMotor)
        {
            float Cdot = Vector2.Dot(axisA, vB - vA) + a2 * wB - a1 * wA;
            float impulse = axialMass * (motorSpeed - Cdot);
            float oldImpulse = motorImpulse;
            float maxImpulse = context.h * maxMotorForce;
            motorImpulse = Math.Clamp(motorImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = motorImpulse - oldImpulse;
            Vector2 P = impulse * axisA;
            float LA = impulse * a1, LB = impulse * a2;
            vA = Vector2.MulSub(vA, mA, P); wA -= iA * LA;
            vB = Vector2.MulAdd(vB, mB, P); wB += iB * LB;
        }
        if (enableLimit)
        {
            float speculativeDistance = 0.25f * (upperTranslation - lowerTranslation);
            {
                float C = translation - lowerTranslation;
                if (C > speculativeDistance)
                {
                    float bias = 0, massScale = 1, impulseScale = 0;
                    if (C > 0) bias = Math.Min(C, Box2D.LengthUnitsPerMeter) * context.inv_h;
                    else if (useBias)
                    {
                        bias = C * context.inv_h;
                        massScale = joint.constraintSoftness.massScale;
                        impulseScale = joint.constraintSoftness.impulseScale;
                    }
                    float oldImpulse = lowerImpulse;
                    float Cdot = Vector2.Dot(axisA, vB - vA) + a2 * wB - a1 * wA;
                    float deltaImpulse = -axialMass * massScale * (Cdot + bias) - impulseScale * oldImpulse;
                    lowerImpulse = Math.Max(oldImpulse + deltaImpulse, 0);
                    deltaImpulse = lowerImpulse - oldImpulse;
                    Vector2 P = impulse * axisA;
                    float LA = deltaImpulse * a1, LB = deltaImpulse * a2;
                    vA = Vector2.MulSub(vA, mA, P); wA -= iA * LA;
                    vB = Vector2.MulAdd(vB, mB, P); wB += iB * LB;
                }
                else lowerImpulse = 0;
            }
            {
                float C = upperTranslation - translation;
                if (C > speculativeDistance)
                {
                    float bias = 0, massScale = 1, impulseScale = 0;
                    if (C > 0) bias = Math.Min(C, Box2D.LengthUnitsPerMeter) * context.inv_h;
                    else if (useBias)
                    {
                        bias = C * context.inv_h;
                        massScale = joint.constraintSoftness.massScale;
                        impulseScale = joint.constraintSoftness.impulseScale;
                    }
                    float oldImpulse = upperImpulse;
                    float Cdot = Vector2.Dot(axisA, vA - vB) + a1 * wA - a2 * wB;
                    float deltaImpulse = -axialMass * massScale * (Cdot + bias) - impulseScale * oldImpulse;
                    upperImpulse = Math.Max(oldImpulse + deltaImpulse, 0);
                    deltaImpulse = upperImpulse - oldImpulse;
                    Vector2 P = impulse * axisA;
                    float LA = deltaImpulse * a1, LB = deltaImpulse * a2;
                    vA = Vector2.MulAdd(vA, mA, P); wA += iA * LA;
                    vB = Vector2.MulSub(vB, mB, P); wB -= iB * LB;
                }
                else upperImpulse = 0;
            }
        }
        {
            Vector2 perpA = axisA.LeftPerp();
            float s1 = Vector2.Cross(d + rA, perpA);
            float s2 = Vector2.Cross(rB, perpA);
            Vector2 Cdot = new(Vector2.Dot(perpA, vB - vA) + s2 * wB - s1 * wA, wB - wA);
            Vector2 bias = Vector2.Zero;
            float massScale = 1, impulseScale = 0;
            if (useBias)
            {
                Vector2 C = new(Vector2.Dot(perpA, d), relQ.GetAngle());
                bias = joint.constraintSoftness.biasRate * C;
                massScale = joint.constraintSoftness.massScale;
                impulseScale = joint.constraintSoftness.impulseScale;
            }
            float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            float k12 = iA * s1 + iB * s2;
            float k22 = iA + iB;
            if (k22 == 0) k22 = 1;
            Mat22 K = new(new(k11, k12), new(k12, k22));
            Vector2 b = K.Solve(Cdot + bias);
            Vector2 deltaImpulse = -massScale * b - impulseScale * this.impulse;
            this.impulse += deltaImpulse;
            Vector2 P = deltaImpulse.x * perpA;
            float LA = deltaImpulse.x * s1 + deltaImpulse.y, LB = deltaImpulse.x * s2 + deltaImpulse.y;
            vA = Vector2.MulSub(vA, mA, P); wA -= iA * LA;
            vB = Vector2.MulAdd(vB, mB, P); wB += iB * LB;
        }
        Debug.Assert(vA.IsValid());
        Debug.Assert(float.IsFinite(wA));
        Debug.Assert(vB.IsValid());
        Debug.Assert(float.IsFinite(wB));
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
        Debug.Assert(jointSim.type == JointType.Prismatic);
        Transform frameA = transformA * jointSim.localFrameA;
        Transform frameB = transformB * jointSim.localFrameB;
        Vector2 axisA = frameA.q * new Vector2(1, 0);
        draw.DrawSegmentFcn(frameA.p, frameB.p, HexColor.DimGray, draw.context);
        if (enableLimit)
        {
            float b = 0.25f * drawScale;
            Vector2 lower = Vector2.MulAdd(frameA.p, lowerTranslation, axisA);
            Vector2 upper = Vector2.MulAdd(frameA.p, upperTranslation, axisA);
            Vector2 perp = axisA.LeftPerp();
            draw.DrawSegmentFcn(lower, upper, HexColor.Gray, draw.context);
            draw.DrawSegmentFcn(Vector2.MulSub(lower, b, perp), Vector2.MulAdd(lower, b, perp), HexColor.Green, draw.context);
            draw.DrawSegmentFcn(Vector2.MulSub(upper, b, perp), Vector2.MulAdd(upper, b, perp), HexColor.Red, draw.context);
        }
        else draw.DrawSegmentFcn(frameA.p - axisA, frameA.p + axisA, HexColor.Gray, draw.context);
        if (enableSpring) draw.DrawPointFcn(Vector2.MulAdd(frameA.p, targetTranslation, axisA), 8, HexColor.Violet, draw.context);
        draw.DrawPointFcn(frameA.p, 5, HexColor.Gray, draw.context);
        draw.DrawPointFcn(frameB.p, 5, HexColor.Blue, draw.context);
    }
    public IJoint Copy() => new PrismaticJoint(this);
}
