using System;
using System.Diagnostics;

namespace Box2D;

public unsafe record class RevoluteJoint : IJoint
{
    public Vector2 linearImpulse;
    public float springImpulse;
    public float motorImpulse;
    public float lowerImpulse;
    public float upperImpulse;
    public float hertz;
    public float dampingRatio;
    public float targetAngle;
    public float maxMotorTorque;
    public float motorSpeed;
    public float lowerAngle;
    public float upperAngle;

    public int indexA;
    public int indexB;
    public Transform frameA;
    public Transform frameB;
    public Vector2 deltaCenter;
    public float axialMass;
    public Softness springSoftness;

    public bool enableSpring;
    public bool enableMotor;
    public bool enableLimit;
    public static JointID Create(WorldID worldId, ref RevoluteJointDef def)
    {
        Debug.Assert(def.internalValue == Box2D.SECRET_COOKIE);
        Debug.Assert(def.lowerAngle <= def.upperAngle);
        Debug.Assert(def.lowerAngle >= -0.99f * MathF.PI);
        Debug.Assert(def.upperAngle <= 0.99f * MathF.PI);
        World world = worldId.index1;
        Debug.Assert(!world.locked);
        if (world.locked) return new();
        JointPair pair = world.CreateJoint(ref def.base_, JointType.Revolute);
        pair.jointSim.joint = new RevoluteJoint
        {
            targetAngle = Math.Clamp(def.targetAngle, -MathF.PI, MathF.PI),
            hertz = def.hertz,
            dampingRatio = def.dampingRatio,
            lowerAngle = def.lowerAngle,
            upperAngle = def.upperAngle,
            maxMotorTorque = def.maxMotorTorque,
            motorSpeed = def.motorSpeed,
            enableSpring = def.enableSpring,
            enableLimit = def.enableLimit,
            enableMotor = def.enableMotor,
        };
        return new() { index1 = pair.jointSim.jointId + 1, world0 = world, generation = pair.joint.generation };
    }
    public void GetReaction(out float linearImpulse, out float angularImpulse)
    {
        linearImpulse = this.linearImpulse.Length();
        angularImpulse = Math.Abs(motorImpulse + lowerImpulse - upperImpulse);
    }
    public float GetLinearSeparation(Transform xfA, Transform xfB, Vector2 dp) => dp.Length();
    public float GetAngularSeparation(float relativeAngle)
    {
        return enableLimit ?
            relativeAngle < lowerAngle ? lowerAngle - relativeAngle
            : upperAngle < relativeAngle ? relativeAngle - upperAngle : 0
            : 0;
    }
    public Vector2 GetForce(World world, JointSim base_) => world.inv_h * linearImpulse;
    public float GetTorque(World world, JointSim base_) => world.inv_h * (motorImpulse + lowerImpulse - upperImpulse);
    public void Prepare(JointSim joint, StepContext context)
    {
        Debug.Assert(joint.type == JointType.Revolute);
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
        float k = iA + iB;
        axialMass = k > 0 ? 1 / k : 0;
        springSoftness = new(hertz, dampingRatio, context.h);
        if (!context.enableWarmStarting)
        {
            linearImpulse = Vector2.Zero; springImpulse = 0;
            motorImpulse = 0; lowerImpulse = 0; upperImpulse = 0;
        }
    }
    public void WarmStart(JointSim joint, StepContext context)
    {
        Debug.Assert(joint.type == JointType.Revolute);
        float mA = joint.invMassA, mB = joint.invMassB;
        float iA = joint.invIA, iB = joint.invIB;
        BodyState dummyState = new();
        BodyState* stateA = &dummyState; if (indexA != -1) stateA = context.states.Data + indexA;
        BodyState* stateB = &dummyState; if (indexB != -1) stateB = context.states.Data + indexB;
        Vector2 rA = stateA->deltaRotation * frameA.p, rB = stateB->deltaRotation * frameB.p;
        float axialImpulse = springImpulse + motorImpulse + lowerImpulse - upperImpulse;
        if (stateA->flags.HasFlag(BodyFlags.Dynamic))
        {
            stateA->linearVelocity = Vector2.MulSub(stateA->linearVelocity, mA, linearImpulse);
            stateA->angularVelocity -= iA * (Vector2.Cross(rA, linearImpulse) + axialImpulse);
        }
        if (stateB->flags.HasFlag(BodyFlags.Dynamic))
        {
            stateB->linearVelocity = Vector2.MulAdd(stateB->linearVelocity, mB, linearImpulse);
            stateB->angularVelocity += iB * (Vector2.Cross(rB, linearImpulse) + axialImpulse);
        }
    }
    public void Solve(JointSim joint, StepContext context, bool useBias)
    {
        Debug.Assert(joint.type == JointType.Revolute);
        float mA = joint.invMassA, mB = joint.invMassB;
        float iA = joint.invIA, iB = joint.invIB;
        BodyState dummyState = new();
        BodyState* stateA = &dummyState; if (indexA != -1) stateA = context.states.Data + indexA;
        BodyState* stateB = &dummyState; if (indexB != -1) stateB = context.states.Data + indexB;
        Vector2 vA = stateA->linearVelocity, vB = stateB->linearVelocity;
        float wA = stateA->angularVelocity, wB = stateB->angularVelocity;
        Rotation qA = stateA->deltaRotation * frameA.q, qB = stateB->deltaRotation * frameB.q;
        Rotation relQ = Rotation.InvMulRot(qA, qB);
        bool fixedRotation = iA + iB == 0;
        if (enableSpring && !fixedRotation)
        {
            float jointAngle = relQ.GetAngle();
            float jointAngleDelta = Rotation.UnwindAngle(jointAngle - targetAngle);
            float bias = springSoftness.biasRate * jointAngleDelta;
            float massScale = springSoftness.massScale;
            float impulseScale = springSoftness.impulseScale;
            float Cdot = wB - wA;
            float impulse = -massScale * axialMass * (Cdot + bias) - impulseScale * springImpulse;
            springImpulse += impulse;
            wA -= iA * impulse;
            wB += iB * impulse;
        }
        if (enableMotor && !fixedRotation)
        {
            float Cdot = wB - wA - motorSpeed;
            float impulse = -axialMass * Cdot;
            float oldImpulse = motorImpulse;
            float maxImpulse = context.h * maxMotorTorque;
            motorImpulse = Math.Clamp(motorImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = motorImpulse - oldImpulse;
            wA -= iA * impulse;
            wB += iB * impulse;
        }
        if (enableLimit && !fixedRotation)
        {
            float jointAngle = relQ.GetAngle();
            {
                float C = jointAngle - lowerAngle;
                float bias = 0, massScale = 1, impulseScale = 0;
                if (C > 0) bias = C * context.inv_h;
                else if (useBias)
                {
                    bias = joint.constraintSoftness.biasRate * C;
                    massScale = joint.constraintSoftness.massScale;
                    impulseScale = joint.constraintSoftness.impulseScale;
                }
                float Cdot = wB - wA;
                float oldImpulse = lowerImpulse;
                float impulse = -massScale * axialMass * (Cdot + bias) - impulseScale * oldImpulse;
                lowerImpulse = Math.Max(oldImpulse + impulse, 0);
                impulse = lowerImpulse - oldImpulse;
                wA -= iA * impulse;
                wB += iB * impulse;
            }
            {
                float C = upperAngle - jointAngle;
                float bias = 0, massScale = 1, impulseScale = 0;
                if (C > 0) bias = C * context.inv_h;
                else if (useBias)
                {
                    bias = joint.constraintSoftness.biasRate * C;
                    massScale = joint.constraintSoftness.massScale;
                    impulseScale = joint.constraintSoftness.impulseScale;
                }
                float Cdot = wA - wB;
                float oldImpulse = upperImpulse;
                float impulse = -massScale * axialMass * (Cdot + bias) - impulseScale * oldImpulse;
                upperImpulse = Math.Max(oldImpulse + impulse, 0);
                impulse = upperImpulse - oldImpulse;
                wA += iA * impulse;
                wB -= iB * impulse;
            }
        }
        {
            Vector2 rA = stateA->deltaRotation * frameA.p, rB = stateB->deltaRotation * frameB.p;
            Vector2 Cdot = vB + Vector2.CrossSV(wB, rB) - (vA + Vector2.CrossSV(wA, rA));
            Vector2 bias = Vector2.Zero;
            float massScale = 1, impulseScale = 0;
            if (useBias)
            {
                Vector2 dcA = stateA->deltaPosition, dcB = stateB->deltaPosition;
                Vector2 separation = dcB - dcA + (rB - rA) + deltaCenter;
                bias = joint.constraintSoftness.biasRate * separation;
                massScale = joint.constraintSoftness.massScale;
                impulseScale = joint.constraintSoftness.impulseScale;
            }
            Mat22 K = new(new(mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB, -rA.y * rA.x * iA - rB.y * rB.x * iB),
                new(0, mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB));
            K.cy.x = K.cx.y;
            K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
            Vector2 b = K.Solve(Cdot + bias);
            Vector2 impulse = -massScale * b - impulseScale * linearImpulse;
            linearImpulse += impulse;
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
        Vector2 pA, Vector2 pB, float drawSize, HexColor color)
    {
        Debug.Assert(jointSim.type == JointType.Revolute);
        Transform frameA = transformA * jointSim.localFrameA, frameB = transformB * jointSim.localFrameB;
        float radius = 0.25f * drawSize;
        draw.DrawCircleFcn(frameB.p, radius, HexColor.Gray, draw.context);
        Vector2 rx = new(radius, 0), r = frameA.q * rx;
        draw.DrawSegmentFcn(frameA.p, frameA.p + r, HexColor.Gray, draw.context);
        r = frameB.q * rx;
        draw.DrawSegmentFcn(frameB.p, frameB.p + r, HexColor.Gray, draw.context);
        if (draw.drawJointExtras)
        {
            float jointAngle = Rotation.RelativeAngle(frameA.q, frameB.q);
            draw.DrawStringFcn(frameA.p + r, $"{180 * jointAngle / MathF.PI:F1} deg", HexColor.White, draw.context);
        }
        if (enableLimit)
        {
            Rotation rotLo = frameA.q * new Rotation(lowerAngle);
            Vector2 rlo = rotLo * rx;
            Rotation rotHi = frameA.q * new Rotation(upperAngle);
            Vector2 rhi = rotHi * rx;
            draw.DrawSegmentFcn(frameB.p, frameB.p + rlo, HexColor.Green, draw.context);
            draw.DrawSegmentFcn(frameB.p, frameB.p + rhi, HexColor.Red, draw.context);
        }
        if (enableSpring)
        {
            Rotation q = frameA.q * new Rotation(targetAngle);
            draw.DrawSegmentFcn(frameB.p, frameB.p + q * rx, HexColor.Violet, draw.context);
        }
        draw.DrawSegmentFcn(transformA.p, frameA.p, HexColor.Gold, draw.context);
        draw.DrawSegmentFcn(frameA.p, frameB.p, HexColor.Gold, draw.context);
        draw.DrawSegmentFcn(transformB.p, frameB.p, HexColor.Gold, draw.context);
    }
    public IJoint Copy() => new RevoluteJoint(this);
}
