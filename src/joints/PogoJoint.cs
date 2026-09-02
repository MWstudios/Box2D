using System;
using System.Diagnostics;

namespace Box2D;

/// <summary>The pogo joint is a hybrid between a contact and a joint</summary>
public unsafe record class PogoJoint : IJoint
{
    public static Vector2 Axis = new(0, 1);
    public Vector2 normal;
    public float restLength;
    public float hertz;
    public float dampingRatio;
    public float maxTensionForce;
    public float maxCompressionForce;
    public float impulse;
    public int indexA;
    public int indexB;
    public Transform frameA;
    public Transform frameB;
    public Vector2 deltaCenter;
    public float linearMass;
    public float velocity;
    public static JointID Create(WorldID worldId, ref PogoJointDef def)
    {
        Debug.Assert(def.internalValue == Box2D.SECRET_COOKIE);
        Debug.Assert(float.IsFinite(def.restLength) && def.restLength >= 0); 
        Debug.Assert(float.IsFinite(def.hertz) && def.hertz >= 0); 
        Debug.Assert(float.IsFinite(def.dampingRatio) && def.dampingRatio >= 0); 
        Debug.Assert(float.IsFinite(def.maxTensionForce) && def.maxTensionForce >= 0); 
        Debug.Assert(float.IsFinite(def.maxCompressionForce) && def.maxCompressionForce >= 0); 
        World world = worldId.index1;
        Debug.Assert(!world.locked);
        if (world.locked) return new();
        JointPair pair = world.CreateJoint(ref def.base_, JointType.Pogo);
        pair.jointSim.joint = new PogoJoint
        {
            restLength = def.restLength,
            hertz = def.hertz,
            dampingRatio = def.dampingRatio,
            maxTensionForce = def.maxTensionForce,
            maxCompressionForce = def.maxCompressionForce,
            normal = def.normal,
            impulse = def.impulse,
            velocity = def.velocity
        };
        return new() { index1 = pair.jointSim.jointId + 1, world0 = world, generation = pair.joint.generation };
    }
    public void GetReaction(out float linearImpulse, out float angularImpulse)
    { linearImpulse = MathF.Abs(impulse); angularImpulse = 0; }
    public Vector2 GetForce(World world, JointSim base_) => world.inv_h * impulse * normal;
    public void Prepare(JointSim joint, StepContext context)
    {
        Debug.Assert(joint.type == JointType.Pogo);
        int idA = joint.bodyIdA, idB = joint.bodyIdB;
        World world = context.world;
        Body bodyA = world.bodies[idA], bodyB = world.bodies[idB];
        Debug.Assert(bodyA.setIndex == (int)SetType.Awake || bodyB.setIndex == (int)SetType.Awake);
        SolverSet setA = world.solverSets[bodyA.setIndex];
        SolverSet setB = world.solverSets[bodyB.setIndex];
        int localIndexA = bodyA.localIndex, localIndexB = bodyB.localIndex;
        BodySim bodySimA = setA.bodySims[localIndexA], bodySimB = setB.bodySims[localIndexB];
        float mA = bodySimA.invMass, iA = bodySimA.invInertia;
        float mB = bodySimB.invMass, iB = bodySimB.invInertia;
        joint.invMassA = mA; joint.invMassB = mB;
        joint.invIA = iA; joint.invIB = iB;
        indexA = bodyA.setIndex == (int)SetType.Awake ? localIndexA : -1;
        indexB = bodyB.setIndex == (int)SetType.Awake ? localIndexB : -1;
        frameA = new(bodySimA.transform.q * (joint.localFrameA.p - bodySimA.localCenter), bodySimA.transform.q * joint.localFrameA.q);
        frameB = new(bodySimB.transform.q * (joint.localFrameB.p - bodySimB.localCenter), bodySimB.transform.q * joint.localFrameB.q);
        deltaCenter = bodySimB.center - bodySimA.center;
        Vector2 rA = frameA.p, rB = frameB.p;
        float crA = Vector2.Cross(rA, normal), crB = Vector2.Cross(rB, normal);
        float k = mA + mB + iA * crA * crA + iB * crB * crB;
        linearMass = k > 0 ? 1 / k : 0;
        if (!context.enableWarmStarting) impulse = 0;
    }
    public void WarmStart(JointSim joint, StepContext context)
    {
        Debug.Assert(joint.type == JointType.Pogo);
        float mA = joint.invMassA, mB = joint.invMassB;
        float iA = joint.invIA, iB = joint.invIB;
        BodyState* stateA = BodyState.IdentityPtr; if (indexA != -1) stateA = context.states.Data + indexA;
        BodyState* stateB = BodyState.IdentityPtr; if (indexB != -1) stateB = context.states.Data + indexB;
        Vector2 rA = stateA->deltaRotation * frameA.p;
        Vector2 rB = stateB->deltaRotation * frameB.p;
        Vector2 linearImpulse = impulse * normal;
        if (stateA->flags.HasFlag(BodyFlags.Dynamic))
        {
            stateA->linearVelocity = Vector2.MulSub(stateA->linearVelocity, mA, linearImpulse);
            stateA->angularVelocity -= iA * Vector2.Cross(rA, linearImpulse);
        }
        if (stateB->flags.HasFlag(BodyFlags.Dynamic))
        {
            stateB->linearVelocity = Vector2.MulAdd(stateB->linearVelocity, mB, linearImpulse);
            stateB->angularVelocity += iB * Vector2.Cross(rB, linearImpulse);
        }
    }
    public void Solve(JointSim joint, StepContext context, bool useBias)
    {
        if (hertz == 0) { this.impulse = 0; return; }
        Debug.Assert(joint.type == JointType.Pogo);
        float mA = joint.invMassA, mB = joint.invMassB;
        float iA = joint.invIA, iB = joint.invIB;

        BodyState* stateA = BodyState.IdentityPtr; if (indexA != -1) stateA = context.states.Data + indexA;
        BodyState* stateB = BodyState.IdentityPtr; if (indexB != -1) stateB = context.states.Data + indexB;
        Vector2 vA = stateA->linearVelocity; float wA = stateA->angularVelocity;
        Vector2 vB = stateB->linearVelocity; float wB = stateB->angularVelocity;
        Vector2 rA = stateA->deltaRotation * frameA.p;
        Vector2 rB = stateB->deltaRotation * frameB.p;
        float bias = 0;
        if (useBias)
        {
            Vector2 dcA = stateA->deltaPosition, dcB = stateB->deltaPosition;
            Vector2 d = dcB - dcA + (rB - rA) + deltaCenter;
            Vector2 pogoAxis = frameB.q * Axis;
            float c = Vector2.Dot(pogoAxis, d) - restLength;
            velocity = Box2D.SpringDamper(hertz, dampingRatio, c, velocity, context.h);
            bias = -velocity;
        }
        Vector2 vr = vB + Vector2.CrossSV(wB, rB) - (vA + Vector2.CrossSV(wA, rA));
        float cdot = Vector2.Dot(normal, vr);
        float maxTensionImpulse = context.h * maxTensionForce;
        float maxCompressionImpulse = context.h * maxCompressionForce;
        float oldImpulse = this.impulse;
        float impulse = -linearMass * (cdot + bias);
        this.impulse = Math.Clamp(this.impulse + impulse, -maxTensionImpulse, maxCompressionImpulse);
        impulse = this.impulse - oldImpulse;
        Vector2 P = impulse * normal;
        vA = Vector2.MulSub(vA, mA, P); wA -= iA * Vector2.Cross(rA, P);
        vB = Vector2.MulAdd(vB, mB, P); wA += iB * Vector2.Cross(rB, P);
        if (stateA->flags.HasFlag(BodyFlags.Dynamic)) { stateA->linearVelocity = vA; stateA->angularVelocity = wA; }
        if (stateB->flags.HasFlag(BodyFlags.Dynamic)) { stateB->linearVelocity = vB; stateB->angularVelocity = wB; }
    }
    public void Draw(DebugDraw draw, JointSim jointSim, WorldTransform transformA, WorldTransform transformB,
        Position pA, Position pB, float drawSize, HexColor color)
    {
        draw.DrawLineFcn(pA, pB, HexColor.LightGray, draw.context);
    }
    public void HashStateDeep(ref ulong hash)
    {
        hash = Box2D.FnvMixFloat(hash, impulse);
        hash = Box2D.FnvMixFloat(hash, velocity);
    }
    public IJoint Copy() => new PogoJoint(this);
}
