using System;
using System.Diagnostics;

namespace Box2D;

public unsafe record class WeldJoint : IJoint
{
    public float linearHertz;
    public float linearDampingRatio;
    public float angularHertz;
    public float angularDampingRatio;

    public Softness linearSpring;
    public Softness angularSpring;
    public Vector2 linearImpulse;
    public float angularImpulse;

    public int indexA;
    public int indexB;
    public Transform frameA;
    public Transform frameB;
    public Vector2 deltaCenter;
    public float axialMass;
    public static JointID Create(WorldID worldId, ref WeldJointDef def)
    {
        Debug.Assert(def.internalValue == Box2D.SECRET_COOKIE);
        World world = worldId.index1;
        Debug.Assert(!world.locked);
        if (world.locked) return new();
        JointPair pair = world.CreateJoint(ref def.base_, JointType.Weld);
        pair.jointSim.joint = new WeldJoint
        {
            linearHertz = def.linearHertz,
            linearDampingRatio = def.linearDampingRatio,
            angularHertz = def.angularHertz,
            angularDampingRatio = def.angularDampingRatio,
            linearImpulse = Vector2.Zero,
            angularImpulse = 0
        };
        return new() { index1 = pair.jointSim.jointId + 1, world0 = world, generation = pair.joint.generation };
    }
    public void GetReaction(out float linearImpulse, out float angularImpulse)
    { linearImpulse = this.linearImpulse.Length(); angularImpulse = Math.Abs(this.angularImpulse); }
    public float GetLinearSeparation(Transform xfA, Transform xfB, Vector2 dp) => linearHertz == 0 ? dp.Length() : 0;
    public float GetAngularSeparation(float relativeAngle) => linearHertz == 0 ? relativeAngle : 0;
    public Vector2 GetForce(World world, JointSim base_) => world.inv_h * linearImpulse;
    public float GetTorque(World world, JointSim base_) => world.inv_h * angularImpulse;
    public void Prepare(JointSim joint, StepContext context)
    {
        Debug.Assert(joint.type == JointType.Weld);
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
        float ka = iA + iB;
        axialMass = ka > 0 ? 1 / ka : 0;
        if (linearHertz == 0) linearSpring = joint.constraintSoftness;
        else linearSpring = new(linearHertz, linearDampingRatio, context.h);
        if (angularHertz == 0) angularSpring = joint.constraintSoftness;
        else angularSpring = new(angularHertz, angularDampingRatio, context.h);
        if (!context.enableWarmStarting) { linearImpulse = Vector2.Zero; angularImpulse = 0; }
    }
    public void WarmStart(JointSim joint, StepContext context)
    {
        //Debug.Assert(joint.type == JointType.Weld); // why not?
        float mA = joint.invMassA, mB = joint.invMassB;
        float iA = joint.invIA, iB = joint.invIB;
        BodyState dummyState = new();
        BodyState* stateA = &dummyState; if (indexA != -1) stateA = context.states.Data + indexA;
        BodyState* stateB = &dummyState; if (indexB != -1) stateB = context.states.Data + indexB;
        Vector2 rA = stateA->deltaRotation * frameA.p, rB = stateB->deltaRotation * frameB.p;
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
        {
            Rotation qA = stateA->deltaRotation * frameA.q, qB = stateB->deltaRotation * frameB.q;
            Rotation relQ = Rotation.InvMulRot(qA, qB);
            float jointAngle = relQ.GetAngle();
            float bias = 0, massScale = 1, impulseScale = 0;
            if (useBias || angularHertz > 0)
            {
                bias = angularSpring.biasRate * jointAngle;
                massScale = angularSpring.massScale;
                impulseScale = angularSpring.impulseScale;
            }
            float Cdot = wB - wA;
            float impulse = -massScale * axialMass * (Cdot + bias) - impulseScale * angularImpulse;
            angularImpulse += impulse;
            wA -= iA * impulse;
            wB += iB * impulse;
        }
        {
            Vector2 rA = stateA->deltaRotation * frameA.p, rB = stateB->deltaRotation * frameB.p;
            Vector2 bias = Vector2.Zero;
            float massScale = 1, impulseScale = 0;
            if (useBias || linearHertz > 0)
            {
                Vector2 dcA = stateA->deltaPosition, dcB = stateB->deltaPosition;
                bias = linearSpring.biasRate * (dcB - dcA + (rB - rA) + deltaCenter);
                massScale = linearSpring.massScale;
                impulseScale = linearSpring.impulseScale;
            }
            Vector2 Cdot = vB + Vector2.CrossSV(wB, rB) - (vA + Vector2.CrossSV(wA, rA));
            Mat22 K = new(new(mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB, -rA.y * rA.x * iA - rB.y * rB.x * iB),
                new(0, mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB));
            Vector2 b = K.Solve(Cdot + bias);
            Vector2 impulse = new(-massScale * b.x - impulseScale * linearImpulse.x,
                -massScale * b.y - impulseScale * linearImpulse.y);
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
        Debug.Assert(jointSim.type == JointType.Weld);
        Transform frameA = transformA * jointSim.localFrameA;
        Transform frameB = transformB * jointSim.localFrameB;
        Polygon box = Geometry.MakeBox(0.25f * drawSize, 0.25f * drawSize);
        Vector2[] points = new Vector2[4];
        for (int i = 0; i < 4; i++) points[i] = frameA.TransformPoint(box.vertices[i]);
        draw.DrawPolygonFcn(points, HexColor.DarkOrange, draw.context);
        for (int i = 0; i < 4; i++) points[i] = frameB.TransformPoint(box.vertices[i]);
        draw.DrawPolygonFcn(points, HexColor.DarkCyan, draw.context);
    }
    public IJoint Copy() => new WeldJoint(this);
}
