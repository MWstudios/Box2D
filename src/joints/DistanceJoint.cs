using System;
using System.Diagnostics;

namespace Box2D;

public unsafe record class DistanceJoint : IJoint
{
    public float length;
    public float hertz;
    public float dampingRatio;
    public float lowerSpringForce;
    public float upperSpringForce;
    public float minLength;
    public float maxLength;

    public float maxMotorForce;
    public float motorSpeed;

    public float impulse;
    public float lowerImpulse;
    public float upperImpulse;
    public float motorImpulse;

    public int indexA;
    public int indexB;
    public Vector2 anchorA;
    public Vector2 anchorB;
    public Vector2 deltaCenter;
    public Softness distanceSoftness;
    public float axialMass;

    public bool enableSpring;
    public bool enableLimit;
    public bool enableMotor;
    public static JointID Create(WorldID worldId, ref DistanceJointDef def)
    {
        Debug.Assert(def.internalValue == Box2D.SECRET_COOKIE);
        World world = worldId.index1;
        Debug.Assert(!world.locked);
        if (world.locked) return new();
        Debug.Assert(float.IsFinite(def.length) && def.length > 0);
        Debug.Assert(def.lowerSpringForce <= def.upperSpringForce);
        JointPair pair = world.CreateJoint(ref def.base_, JointType.Distance);
        pair.jointSim.joint = new DistanceJoint
        {
            length = Math.Max(def.length, Box2D.LinearSlop),
            hertz = def.hertz,
            dampingRatio = def.dampingRatio,
            minLength = Math.Max(def.minLength, Box2D.LinearSlop),
            maxLength = Math.Max(def.minLength, def.maxLength),
            maxMotorForce = def.maxMotorForce,
            motorSpeed = def.motorSpeed,
            enableSpring = def.enableSpring,
            lowerSpringForce = def.lowerSpringForce,
            upperSpringForce = def.upperSpringForce,
            enableLimit = def.enableLimit,
            enableMotor = def.enableMotor,
            impulse = 0,
            lowerImpulse = 0,
            upperImpulse = 0,
            motorImpulse = 0,
        };
        return new() { index1 = pair.jointSim.jointId + 1, world0 = world, generation = pair.joint.generation };
    }
    public void GetReaction(out float linearImpulse, out float angularImpulse)
    {
        linearImpulse = Math.Abs(impulse + lowerImpulse - upperImpulse + motorImpulse);
        angularImpulse = 0;
    }
    public float GetLinearSeparation(Transform xfA, Transform xfB, Vector2 dp)
    {
        float length = dp.Length();
        return enableSpring ? enableLimit ?
            length < minLength ? minLength - length
            : length > maxLength ? length - maxLength : 0
            : 0 : Math.Abs(length - this.length);
    }
    public Vector2 GetForce(World world, JointSim base_)
    {
        Transform transformA = world.GetBodyTransform(base_.bodyIdA);
        Transform transformB = world.GetBodyTransform(base_.bodyIdB);
        Vector2 pA = transformA.TransformPoint(base_.localFrameA.p);
        Vector2 pB = transformB.TransformPoint(base_.localFrameB.p);
        Vector2 axis = (pB - pA).Normalize();
        float force = (impulse + lowerImpulse - upperImpulse + motorImpulse) * world.inv_h;
        return force * axis;
    }
    public void Prepare(JointSim joint, StepContext context)
    {
        Debug.Assert(joint.type == JointType.Distance);
        int idA = joint.bodyIdA, idB = joint.bodyIdB;
        World world = context.world;
        Body bodyA = world.bodies[idA], bodyB = world.bodies[idB];
        Debug.Assert(bodyA.setIndex == (int)SetType.Awake || bodyB.setIndex == (int)SetType.Awake);
        SolverSet setA = world.solverSets[bodyA.setIndex];
        SolverSet setB = world.solverSets[bodyB.setIndex];
        int localIndexA = bodyA.localIndex, localIndexB = bodyB.localIndex;
        BodySim bodySimA = setA.bodySims[localIndexA], bodySimB = setB.bodySims[localIndexB];
        float mA = bodySimA.invMass, iA = bodySimA.invMass;
        float mB = bodySimB.invMass, iB = bodySimB.invMass;
        joint.invMassA = mA; joint.invMassB = mB;
        joint.invIA = iA; joint.invIB = iB;
        indexA = bodyA.setIndex == (int)SetType.Awake ? localIndexA : -1;
        indexB = bodyB.setIndex == (int)SetType.Awake ? localIndexB : -1;
        anchorA = bodySimA.transform.q * (joint.localFrameA.p - bodySimA.localCenter);
        anchorB = bodySimB.transform.q * (joint.localFrameB.p - bodySimB.localCenter);
        deltaCenter = bodySimB.center - bodySimA.center;
        Vector2 rA = anchorA, rB = anchorB;
        Vector2 separation = rB - rA + deltaCenter;
        Vector2 axis = separation.Normalize();
        float crA = Vector2.Cross(rA, axis), crB = Vector2.Cross(rB, axis);
        float k = mA + mB + iA * crA * crA + iB * crB * crB;
        axialMass = k > 0 ? 1 / k : 0;
        distanceSoftness = new(hertz, dampingRatio, context.h);
        if (!context.enableWarmStarting)
        {
            impulse = 0;
            lowerImpulse = 0;
            upperImpulse = 0;
            motorImpulse = 0;
        }
    }
    public void WarmStart(JointSim joint, StepContext context)
    {
        Debug.Assert(joint.type == JointType.Distance);
        float mA = joint.invMassA, mB = joint.invMassB;
        float iA = joint.invIA, iB = joint.invIB;
        BodyState dummyState = new();
        BodyState* stateA = &dummyState; if (indexA != -1) stateA = context.states.Data + indexA;
        BodyState* stateB = &dummyState; if (indexB != -1) stateB = context.states.Data + indexB;
        Vector2 rA = stateA->deltaRotation * anchorA;
        Vector2 rB = stateB->deltaRotation * anchorB;
        Vector2 ds = stateB->deltaPosition - stateA->deltaPosition + (rB - rA);
        Vector2 axis = (deltaCenter + ds).Normalize();
        float axialImpulse = impulse + lowerImpulse - upperImpulse + motorImpulse;
        Vector2 P = axialImpulse * axis;
        if (stateA->flags.HasFlag(BodyFlags.Dynamic))
        {
            stateA->linearVelocity = Vector2.MulSub(stateA->linearVelocity, mA, P);
            stateA->angularVelocity -= iA * Vector2.Cross(rA, P);
        }
        if (stateB->flags.HasFlag(BodyFlags.Dynamic))
        {
            stateB->linearVelocity = Vector2.MulAdd(stateB->linearVelocity, mB, P);
            stateB->angularVelocity += iB * Vector2.Cross(rB, P);
        }
    }
    public void Solve(JointSim joint, StepContext context, bool useBias)
    {
        Debug.Assert(joint.type == JointType.Distance);
        float mA = joint.invMassA, mB = joint.invMassB;
        float iA = joint.invIA, iB = joint.invIB;
        BodyState dummyState = new();
        BodyState* stateA = &dummyState; if (indexA != -1) stateA = context.states.Data + indexA;
        BodyState* stateB = &dummyState; if (indexB != -1) stateB = context.states.Data + indexB;
        Vector2 vA = stateA->linearVelocity; float wA = stateA->angularVelocity;
        Vector2 vB = stateB->linearVelocity; float wB = stateB->angularVelocity;
        Vector2 rA = stateA->deltaRotation * anchorA;
        Vector2 rB = stateB->deltaRotation * anchorB;
        Vector2 ds = stateB->deltaPosition - stateA->deltaPosition + (rB - rA);
        Vector2 separation = deltaCenter + ds;
        Vector2 axis = separation.GetLengthAndNormalize(out float length);
        if (enableSpring && (minLength < maxLength || !enableLimit))
        {
            if (hertz > 0)
            {
                Vector2 vr = vB - vA + (Vector2.CrossSV(wB, rB) - Vector2.CrossSV(wA, rA));
                float Cdot = Vector2.Dot(axis, vr);
                float C = length - this.length;
                float bias = distanceSoftness.biasRate * C;
                float m = distanceSoftness.massScale * axialMass;
                float oldImpulse = this.impulse;
                float impulse = -m * (Cdot + bias) - distanceSoftness.impulseScale * oldImpulse;
                float h = context.h;
                this.impulse = Math.Clamp(this.impulse + impulse, lowerSpringForce * h, upperSpringForce * h);
                impulse = this.impulse - oldImpulse;
                Vector2 P = impulse * axis;
                vA = Vector2.MulSub(vA, mA, P); wA -= iA * Vector2.Cross(rA, P);
                vB = Vector2.MulAdd(vB, mB, P); wB += iB * Vector2.Cross(rB, P);
            }
            if (enableLimit)
            {
                {
                    Vector2 vr = vB - vA + (Vector2.CrossSV(wB, rB) - Vector2.CrossSV(wA, rA));
                    float Cdot = Vector2.Dot(axis, vr);
                    float C = length - minLength;
                    float bias = 0, massCoeff = 1, impulseCoeff = 0;
                    if (C > 0) bias = C * context.inv_h;
                    else if (useBias)
                    {
                        bias = joint.constraintSoftness.biasRate * C;
                        massCoeff = joint.constraintSoftness.massScale;
                        impulseCoeff = joint.constraintSoftness.impulseScale;
                    }
                    float impulse = -massCoeff * axialMass * (Cdot + bias) - impulseCoeff * lowerImpulse;
                    float newImpulse = Math.Max(0, lowerImpulse + impulse);
                    impulse = newImpulse - lowerImpulse;
                    lowerImpulse = newImpulse;
                    Vector2 P = impulse * axis;
                    vA = Vector2.MulSub(vA, mA, P); wA -= iA * Vector2.Cross(rA, P);
                    vB = Vector2.MulAdd(vB, mB, P); wB += iB * Vector2.Cross(rB, P);
                }
                {
                    Vector2 vr = vA - vB + (Vector2.CrossSV(wA, rA) - Vector2.CrossSV(wB, rB));
                    float Cdot = Vector2.Dot(axis, vr);
                    float C = maxLength - length;
                    float bias = 0, massScale = 1, impulseScale = 0;
                    if (C > 0) bias = C * context.inv_h;
                    else if (useBias)
                    {
                        bias = joint.constraintSoftness.biasRate * C;
                        massScale = joint.constraintSoftness.massScale;
                        impulseScale = joint.constraintSoftness.impulseScale;
                    }
                    float impulse = -massScale * axialMass * (Cdot + bias) - impulseScale * upperImpulse;
                    float newImpulse = Math.Max(0, upperImpulse + impulse);
                    impulse = newImpulse - upperImpulse;
                    upperImpulse = newImpulse;
                    Vector2 P = -impulse * axis;
                    vA = Vector2.MulSub(vA, mA, P); wA -= iA * Vector2.Cross(rA, P);
                    vB = Vector2.MulAdd(vB, mB, P); wB += iB * Vector2.Cross(rB, P);
                }
            }
            if (enableMotor)
            {
                Vector2 vr = vB - vA + (Vector2.CrossSV(wB, rB) - Vector2.CrossSV(wA, rA));
                float Cdot = Vector2.Dot(axis, vr);
                float impulse = axialMass * (motorSpeed - Cdot);
                float oldImpulse = motorImpulse;
                float maxImpulse = context.h * maxMotorForce;
                motorImpulse = Math.Clamp(motorImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = motorImpulse - oldImpulse;
                Vector2 P = impulse * axis;
                vA = Vector2.MulSub(vA, mA, P); wA -= iA * Vector2.Cross(rA, P);
                vB = Vector2.MulAdd(vB, mB, P); wB += iB * Vector2.Cross(rB, P);
            }
        }
        else
        {
            Vector2 vr = vB - vA + (Vector2.CrossSV(wB, rB) - Vector2.CrossSV(wA, rA));
            float Cdot = Vector2.Dot(axis, vr);
            float C = length - this.length;
            float bias = 0, massScale = 1, impulseScale = 0;
            if (useBias)
            {
                bias = joint.constraintSoftness.biasRate * C;
                massScale = joint.constraintSoftness.massScale;
                impulseScale = joint.constraintSoftness.impulseScale;
            }
            float impulse = -massScale * axialMass * (Cdot + bias) - impulseScale * this.impulse;
            this.impulse += impulse;
            Vector2 P = impulse * axis;
            vA = Vector2.MulSub(vA, mA, P); wA -= iA * Vector2.Cross(rA, P);
            vB = Vector2.MulAdd(vB, mB, P); wB += iB * Vector2.Cross(rB, P);
        }
        if (stateA->flags.HasFlag(BodyFlags.Dynamic))
        { stateA->linearVelocity = vA; stateA->angularVelocity = wA; }
        if (stateB->flags.HasFlag(BodyFlags.Dynamic))
        { stateB->linearVelocity = vB; stateB->angularVelocity = wB; }
    }
    public void Draw(DebugDraw draw, JointSim jointSim, Transform transformA, Transform transformB,
        Vector2 pA, Vector2 pB, float drawSize, HexColor color)
    {
        Debug.Assert(jointSim.type == JointType.Distance);
        Vector2 axis = (pB - pA).Normalize();
        if (minLength < maxLength && enableLimit)
        {
            Vector2 pMin = Vector2.MulAdd(pA, minLength, axis);
            Vector2 pMax = Vector2.MulAdd(pA, maxLength, axis);
            Vector2 offset = 0.05f * Box2D.LengthUnitsPerMeter * axis.RightPerp();
            if (minLength > Box2D.LinearSlop) draw.DrawSegmentFcn(pMin - offset, pMin + offset, HexColor.LightGreen, draw.context);
            if (maxLength < Box2D.Huge) draw.DrawSegmentFcn(pMax - offset, pMax + offset, HexColor.Red, draw.context);
            if (minLength > Box2D.LinearSlop && maxLength < Box2D.Huge) draw.DrawSegmentFcn(pMin, pMax, HexColor.Gray, draw.context);
        }
        draw.DrawSegmentFcn(pA, pB, HexColor.White, draw.context);
        draw.DrawPointFcn(pA, 4, HexColor.White, draw.context);
        draw.DrawPointFcn(pB, 4, HexColor.White, draw.context);
        if (hertz > 0 && enableSpring)
        {
            Vector2 pRest = Vector2.MulAdd(pA, length, axis);
            draw.DrawPointFcn(pRest, 4, HexColor.Blue, draw.context);
        }
    }
    public IJoint Copy() => new DistanceJoint(this);
}
