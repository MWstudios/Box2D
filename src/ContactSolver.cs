using System;
using System.Diagnostics;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.Arm;
using System.Runtime.Intrinsics.X86;

namespace Box2D;

public struct ContactConstraintPoint
{
    public Vector2 anchorA, anchorB;
    public float baseSeparation;
    public float restitutionVelocity;
    public float normalImpulse;
    public float tangentImpulse;
    public float totalNormalImpulse;
    public float normalMass;
    public float tangentMass;
}

public struct ContactConstraint
{
    /// <summary>base-1, 0 for null</summary>
    public int indexA, indexB;
    public ContactConstraintPoint point0, point1;
    public Vector2 normal;
    public float invMassA, invMassB;
    public float invIA, invIB;
    public float friction;
    public float restitution;
    public float tangentSpeed;
    public float rollingResistance;
    public float rollingMass;
    public float rollingImpulse;
    public Softness softness;
    public int pointCount;
}
public unsafe partial class StepContext
{
    public void PrepareContacts_Overflow()
    {
        ref GraphColor color = ref graph.colors[Box2D.GraphColorCount - 1];
        ContactConstraint[] constraints = color.overflowConstraints;
        int contactCount = color.contactSims.Count;
        var contacts = color.contactSims;

        float warmStartScale = world.enableWarmStarting ? 1 : 0;
        for (int i = 0; i < contactCount; i++)
        {
            ContactSim contactSim = contacts[i];
            ref Manifold manifold = ref contactSim.manifold;
            int pointCount = manifold.pointCount;
            Debug.Assert(0 < pointCount && pointCount <= 2);
            int indexA = contactSim.bodySimIndexA, indexB = contactSim.bodySimIndexB;

            ref ContactConstraint constraint = ref constraints[i];
            constraint.indexA = indexA + 1;
            constraint.indexB = indexB + 1;
            constraint.normal = manifold.normal;
            constraint.friction = contactSim.friction;
            constraint.rollingResistance = contactSim.rollingResistance;
            constraint.rollingImpulse = warmStartScale * manifold.rollingImpulse;
            constraint.tangentSpeed = contactSim.tangentSpeed;
            constraint.pointCount = pointCount;
            float mA = contactSim.invMassA, iA = contactSim.invIA;
            float mB = contactSim.invMassB, iB = contactSim.invIB;
            if (indexA == -1 || indexB == -1) constraint.softness = staticSoftness;
            else constraint.softness = contactSoftness;
            constraint.invMassA = mA;
            constraint.invIA = iA;
            constraint.invMassB = mB;
            constraint.invIB = iB;
            float k = iA + iB;
            constraint.rollingMass = k > 0 ? 1 / k : 0;
            Vector2 normal = constraint.normal;
            Vector2 tangent = constraint.normal.RightPerp();
            for (int j = 0; j < pointCount; j++)
            {
                ref ManifoldPoint mp = ref manifold.point0;
                if (j == 1) mp = ref manifold.point1;
                ref ContactConstraintPoint cp = ref constraint.point0;
                if (j == 1) cp = ref constraint.point1;
                cp.normalImpulse = warmStartScale * mp.normalImpulse;
                cp.tangentImpulse = warmStartScale * mp.tangentImpulse;
                cp.totalNormalImpulse = 0;
                Vector2 rA = mp.anchorA, rB = mp.anchorB;
                cp.anchorA = rA; cp.anchorB = rB;
                cp.baseSeparation = mp.separation - Vector2.Dot(rB - rA, normal);
                float rnA = Vector2.Cross(rA, normal), rnB = Vector2.Cross(rB, normal);
                float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                cp.normalMass = kNormal > 0 ? 1 / kNormal : 0;
                float rtA = Vector2.Cross(rA, tangent), rtB = Vector2.Cross(rB, tangent);
                float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
                cp.tangentMass = kTangent > 0 ? 1 / kTangent : 0;
                cp.restitutionVelocity = mp.restitutionVelocity;
            }
        }
    }
    public void WarmStartContacts_Overflow()
    {
        ref GraphColor color = ref graph.colors[Box2D.GraphColorCount - 1];
        var constraints = color.overflowConstraints;
        int contactCount = color.contactSims.Count;
        SolverSet awakeSet = world.solverSets[(int)SetType.Awake];
        var states = awakeSet.bodyStates;
        
        for (int i = 0; i < contactCount; i++)
        {
            ref ContactConstraint constraint = ref constraints[i];
            int indexA = constraint.indexA - 1, indexB = constraint.indexB - 1;
            BodyState* stateA = BodyState.IdentityPtr; if (indexA != -1) stateA = states.Data + indexA;
            BodyState* stateB = BodyState.IdentityPtr; if (indexB != -1) stateB = states.Data + indexB;
            Vector2 vA = stateA->linearVelocity;
            float wA = stateA->angularVelocity;
            Vector2 vB = stateB->linearVelocity;
            float wB = stateB->angularVelocity;
            float mA = constraint.invMassA;
            float iA = constraint.invIA;
            float mB = constraint.invMassB;
            float iB = constraint.invIB;
            Vector2 normal = constraint.normal, tangent = normal.RightPerp();
            int pointCount = constraint.pointCount;
            for (int j = 0; j < pointCount; j++)
            {
                ref ContactConstraintPoint cp = ref constraint.point0;
                if (j == 1) cp = ref constraint.point1;
                Vector2 rA = cp.anchorA, rB = cp.anchorB;
                Vector2 P = cp.normalImpulse * normal + cp.tangentImpulse * tangent;
                cp.totalNormalImpulse += cp.normalImpulse;
                wA -= iA * Vector2.Cross(rA, P);
                vA = Vector2.MulAdd(vA, -mA, P);
                wB += iB * Vector2.Cross(rB, P);
                vB = Vector2.MulAdd(vB, mB, P);
            }
            wA -= iA * constraint.rollingImpulse;
            wB += iB * constraint.rollingImpulse;
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
    }
    public void SolveContacts_Overflow(bool useBias)
    {
        GraphColor color = graph.colors[Box2D.GraphColorCount - 1];
        var constraints = color.overflowConstraints;
        int contactCount = color.contactSims.Count;
        SolverSet awakeSet = world.solverSets[(int)SetType.Awake];
        var states = awakeSet.bodyStates;
        float contactSpeed = world.contactSpeed;
        
        for (int i = 0; i < contactCount; i++)
        {
            ref ContactConstraint constraint = ref constraints[i];
            float mA = constraint.invMassA;
            float iA = constraint.invIA;
            float mB = constraint.invMassB;
            float iB = constraint.invIB;
            int indexA = constraint.indexA - 1, indexB = constraint.indexB - 1;
            BodyState* stateA = BodyState.IdentityPtr; if (indexA != -1) stateA = states.Data + indexA;
            Vector2 vA = stateA->linearVelocity;
            float wA = stateA->angularVelocity;
            Rotation dqA = stateA->deltaRotation;
            BodyState* stateB = BodyState.IdentityPtr; if (indexB != -1) stateB = states.Data + indexB;
            Vector2 vB = stateB->linearVelocity;
            float wB = stateB->angularVelocity;
            Rotation dqB = stateB->deltaRotation;
            Vector2 dp = stateB->deltaPosition - stateA->deltaPosition;
            Vector2 normal = constraint.normal, tangent = normal.RightPerp();
            float friction = constraint.friction;
            Softness softness = constraint.softness;
            int pointCount = constraint.pointCount;
            float totalNormalImpulse = 0;
            bool stopRestitution = true;
            for (int j = 0; j < pointCount; j++)
            {
                ref ContactConstraintPoint cp = ref constraint.point0;
                if (j == 1) cp = ref constraint.point1;
                Vector2 rA = cp.anchorA, rB = cp.anchorB;
                Vector2 ds = dp + dqB * rB - dqA * rA;
                float s = cp.baseSeparation + Vector2.Dot(ds, normal);
                float velocityBias = 0, massScale = 1, impulseScale = 0;
                if (s > 0) velocityBias = s * inv_h;
                else if (useBias)
                {
                    velocityBias = Math.Max(softness.massScale * softness.biasRate * s, -contactSpeed);
                    massScale = softness.massScale;
                    impulseScale = softness.impulseScale;
                }
                Vector2 vrA = vA + Vector2.CrossSV(wA, rA);
                Vector2 vrB = vB + Vector2.CrossSV(wB, rB);
                float vn = Vector2.Dot(vrB - vrA, normal);
                if (!useBias && cp.restitutionVelocity > 0)
                {
                    velocityBias = Math.Min(velocityBias, -cp.restitutionVelocity);
                    stopRestitution &= s > 0;
                }
                float impulse = -cp.normalMass * (massScale * vn + velocityBias) - impulseScale * cp.normalImpulse;
                float newImpulse = Math.Max(cp.normalImpulse + impulse, 0);
                impulse = newImpulse - cp.normalImpulse;
                cp.normalImpulse = newImpulse;
                cp.totalNormalImpulse += impulse;
                totalNormalImpulse += newImpulse;
                Vector2 P = impulse * normal;
                vA = Vector2.MulSub(vA, mA, P);
                wA -= iA * Vector2.Cross(rA, P);
                vB = Vector2.MulAdd(vB, mB, P);
                wB += iB * Vector2.Cross(rB, P);
            }
            if (!useBias)
            {
                if (stopRestitution)
                {
                    constraint.point0.restitutionVelocity = 0;
                    constraint.point1.restitutionVelocity = 0;
                }
                {
                    float deltaLambda = -constraint.rollingMass * (wB - wA);
                    float lambda = constraint.rollingImpulse;
                    float maxLambda = constraint.rollingResistance * totalNormalImpulse;
                    constraint.rollingImpulse = Math.Clamp(lambda + deltaLambda, -maxLambda, maxLambda);
                    deltaLambda = constraint.rollingImpulse - lambda;
                    wA -= iA * deltaLambda;
                    wB += iB * deltaLambda;
                }
                for (int j = 0; j < pointCount; j++)
                {
                    ref ContactConstraintPoint cp = ref constraint.point0;
                    if (j == 1) cp = ref constraint.point1;
                    Vector2 rA = cp.anchorA, rB = cp.anchorB;
                    Vector2 vrB = vB + Vector2.CrossSV(wB, rB);
                    Vector2 vrA = vA + Vector2.CrossSV(wA, rA);
                    float vt = Vector2.Dot(vrB - vrA, tangent) - constraint.tangentSpeed;
                    float impulse = cp.tangentMass * -vt;
                    float maxFriction = friction * cp.normalImpulse;
                    float newImpulse = Math.Clamp(cp.tangentImpulse + impulse, -maxFriction, maxFriction);
                    impulse = newImpulse - cp.tangentImpulse;
                    cp.tangentImpulse = newImpulse;
                    Vector2 P = impulse * tangent;
                    vA = Vector2.MulSub(vA, mA, P);
                    wA -= iA * Vector2.Cross(rA, P);
                    vB = Vector2.MulAdd(vB, mB, P);
                    wB += iB * Vector2.Cross(rB, P);
                }
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
    }
    public void StoreImpulses_Overflow()
    {
        GraphColor color = graph.colors[Box2D.GraphColorCount - 1];
        var constraints = color.overflowConstraints;
        int contactCount = color.contactSims.Count;
        BitSet hitEventBitSet = world.taskContexts[0].hitEventBitSet;
        float negHitThreshold = -world.hitEventThreshold;
        bool hasHitEvents = world.taskContexts[0].hasHitEvents;
        for (int i = 0; i < contactCount; i++)
        {
            ref ContactConstraint constraint = ref constraints[i];
            ContactSim contactSim = color.contactSims[i];
            ref Manifold manifold = ref contactSim.manifold;
            int pointCount = manifold.pointCount;
            if (pointCount > 0)
            {
                manifold.point0.normalImpulse = constraint.point0.normalImpulse;
                manifold.point0.tangentImpulse = constraint.point0.tangentImpulse;
                manifold.point0.totalNormalImpulse = constraint.point0.totalNormalImpulse;
            }
            if (pointCount > 1)
            {
                manifold.point1.normalImpulse = constraint.point1.normalImpulse;
                manifold.point1.tangentImpulse = constraint.point1.tangentImpulse;
                manifold.point1.totalNormalImpulse = constraint.point1.totalNormalImpulse;
            }
            if (contactSim.simFlags.HasFlag(ContactFlags.SimEnableHitEvent))
            {
                if (contactSim.manifold.pointCount > 0)
                {
                    if (contactSim.manifold.point0.normalVelocity < negHitThreshold && contactSim.manifold.point0.totalNormalImpulse > 0)
                    {
                        hitEventBitSet.SetBit(contactSim.contactId);
                        hasHitEvents = true;
                    }
                    else if (contactSim.manifold.pointCount > 1)
                    {
                        if (contactSim.manifold.point1.normalVelocity < negHitThreshold && contactSim.manifold.point1.totalNormalImpulse > 0)
                        {
                            hitEventBitSet.SetBit(contactSim.contactId);
                            hasHitEvents = true;
                        }
                    }
                }
            }
            manifold.rollingImpulse = constraint.rollingImpulse;
        }
        world.taskContexts[0].hasHitEvents = hasHitEvents;
    }
}
public interface IContactSolverW
{
    public static IContactSolverW Instance() => Avx.IsSupported ? new ContactSolverAVX() :
        AdvSimd.IsSupported ? new ContactSolverNeon() : Sse.IsSupported ? new ContactSolverSSE() : new ContactSolverFloat();
    public void PrepareContacts_Wide(ref SolverBlock block, StepContext context);
    public void WarmStartContacts_Wide(ref SolverBlock block, StepContext context);
    public void PushContacts_Wide(ref SolverBlock block, StepContext context);
    public void SolveContacts_Wide(ref SolverBlock block, StepContext context);
    public void StoreImpulses_Wide(ref SolverBlock block, StepContext context, int workerIndex);
}
public class ContactSolverAVX : IContactSolverW
{
    public struct Vector2W
    {
        public Vector256<float> X, Y;
        public static Vector2W operator -(Vector2W a, Vector2W b) => new() { X = a.X - b.X, Y = a.Y - b.Y };
    }
    struct RotationW
    {
        public Vector256<float> C, S;
    }
    static Vector256<float> NegW(Vector256<float> a) => Avx.Xor(a, Vector256.Create(-0f));
    static Vector256<float> SymClampW(Vector256<float> a, Vector256<float> b) => Avx.Max(Avx.Subtract(Vector256<float>.Zero, b), Avx.Min(a, b));
    static bool AllZeroW(Vector256<float> a) => Avx.MoveMask(Avx.CompareEqual(a, Vector256<float>.Zero)) == 0xFF;
    static Vector2W RightPerpW(Vector2W a) => new() { X = a.Y, Y = -a.X };
    static Vector256<float> DotW(Vector2W a, Vector2W b) => Avx.Add(Avx.Multiply(a.X, b.X), Avx.Multiply(a.Y, b.Y));
    static Vector256<float> CrossW(Vector2W a, Vector2W b) => Avx.Subtract(Avx.Multiply(a.X, b.Y), Avx.Multiply(a.Y, b.X));
    static Vector2W RotateVectorW(RotationW q, Vector2W v) =>
        new() { X = Avx.Subtract(Avx.Multiply(q.C, v.X), Avx.Multiply(q.S, v.Y)), Y = Avx.Add(Avx.Multiply(q.S, v.X), Avx.Multiply(q.C, v.Y)) };
    static Vector256<float> SoftMaskW(Vector256<int> a, Vector256<int> b) => Avx2.Or(Avx2.CompareEqual(a, Vector256<int>.Zero), Avx2.CompareEqual(b, Vector256<int>.Zero)).AsSingle();
    public struct ContactConstraintWide
    {
        public Vector256<int> indexA, indexB;
        public Vector256<float> invMassA, invMassB;
        public Vector256<float> invIA, invIB;
        public Vector2W normal;
        public Vector2W anchorA1, anchorB1;
        public Vector2W anchorA2, anchorB2;
        public Vector256<float> normalMass1, normalMass2;
        public Vector256<float> baseSeparation1, baseSeparation2;
        public Vector256<float> normalImpulse1, normalImpulse2;
        public Vector256<float> totalNormalImpulse1, totalNormalImpulse2;
        public Vector256<float> tangentImpulse1, tangentImpulse2;
        public Vector256<float> rollingImpulse;
        public Vector256<float> friction;
        public Vector256<float> tangentSpeed;
        public Vector256<float> rollingResistance;
        public Vector256<float> tangentMass1, tangentMass2;
        public Vector256<float> negRestitutionVelocity1, negRestitutionVelocity2;
    }
    struct BodyStateW
    {
         public Vector2W v;
         public Vector256<float> w;
         public Vector256<float> flags;
         public Vector2W dp;
         public RotationW dq;
    }
    unsafe BodyStateW GatherBodies(BodyState* states, int* indices)
    {
        Debug.Assert(((nuint)states & 0x1F) == 0);
        Vector256<float> identity = Vector256.Create(0f, 0, 0, 0, 0, 0, 1, 0);
        int i1 = indices[0] - 1, i2 = indices[1] - 1, i3 = indices[2] - 1, i4 = indices[3] - 1,
            i5 = indices[4] - 1, i6 = indices[5] - 1, i7 = indices[6] - 1, i8 = indices[7] - 1;
        Vector256<float> b0 = i1 == -1 ? identity : Avx.LoadAlignedVector256((float*)(states + i1));
        Vector256<float> b1 = i2 == -1 ? identity : Avx.LoadAlignedVector256((float*)(states + i2));
        Vector256<float> b2 = i3 == -1 ? identity : Avx.LoadAlignedVector256((float*)(states + i3));
        Vector256<float> b3 = i4 == -1 ? identity : Avx.LoadAlignedVector256((float*)(states + i4));
        Vector256<float> b4 = i5 == -1 ? identity : Avx.LoadAlignedVector256((float*)(states + i5));
        Vector256<float> b5 = i6 == -1 ? identity : Avx.LoadAlignedVector256((float*)(states + i6));
        Vector256<float> b6 = i7 == -1 ? identity : Avx.LoadAlignedVector256((float*)(states + i7));
        Vector256<float> b7 = i8 == -1 ? identity : Avx.LoadAlignedVector256((float*)(states + i8));
        Vector256<float> t0 = Avx.UnpackLow(b0, b1);
        Vector256<float> t1 = Avx.UnpackHigh(b0, b1);
        Vector256<float> t2 = Avx.UnpackLow(b2, b3);
        Vector256<float> t3 = Avx.UnpackHigh(b2, b3);
        Vector256<float> t4 = Avx.UnpackLow(b4, b5);
        Vector256<float> t5 = Avx.UnpackHigh(b4, b5);
        Vector256<float> t6 = Avx.UnpackLow(b6, b7);
        Vector256<float> t7 = Avx.UnpackHigh(b6, b7);
        Vector256<float> tt0 = Avx.Shuffle(t0, t2, 0b01000100);
        Vector256<float> tt1 = Avx.Shuffle(t0, t2, 0b11101110);
        Vector256<float> tt2 = Avx.Shuffle(t1, t3, 0b01000100);
        Vector256<float> tt3 = Avx.Shuffle(t1, t3, 0b11101110);
        Vector256<float> tt4 = Avx.Shuffle(t4, t6, 0b01000100);
        Vector256<float> tt5 = Avx.Shuffle(t4, t6, 0b11101110);
        Vector256<float> tt6 = Avx.Shuffle(t5, t7, 0b01000100);
        Vector256<float> tt7 = Avx.Shuffle(t5, t7, 0b11101110);
        return new()
        {
            v = new() { X = Avx.Permute2x128(tt0, tt4, 0x20), Y = Avx.Permute2x128(tt1, tt5, 0x20) },
            w = Avx.Permute2x128(tt2, tt6, 0x20),
            flags = Avx.Permute2x128(tt3, tt7, 0x20),
            dp = new() { X = Avx.Permute2x128(tt0, tt4, 0x31), Y = Avx.Permute2x128(tt1, tt5, 0x31) },
            dq = new() { C = Avx.Permute2x128(tt2, tt6, 0x31), S = Avx.Permute2x128(tt3, tt7, 0x31) },
        };
    }
    unsafe void ScatterBodies(BodyState* states, int* indices, ref BodyStateW simdBody)
    {
        Debug.Assert(((nuint)states & 0x1F) == 0);
        Vector256<float> t0 = Avx.UnpackLow(simdBody.v.X, simdBody.v.Y);
        Vector256<float> t1 = Avx.UnpackHigh(simdBody.v.X, simdBody.v.Y);
        Vector256<float> t2 = Avx.UnpackLow(simdBody.w, simdBody.flags);
        Vector256<float> t3 = Avx.UnpackHigh(simdBody.w, simdBody.flags);
        Vector256<float> t4 = Avx.UnpackLow(simdBody.dp.X, simdBody.dp.Y);
        Vector256<float> t5 = Avx.UnpackHigh(simdBody.dp.X, simdBody.dp.Y);
        Vector256<float> t6 = Avx.UnpackLow(simdBody.dq.C, simdBody.dq.S);
        Vector256<float> t7 = Avx.UnpackHigh(simdBody.dq.C, simdBody.dq.S);
        Vector256<float> tt0 = Avx.Shuffle(t0, t2, 0b01000100);
        Vector256<float> tt1 = Avx.Shuffle(t0, t2, 0b11101110);
        Vector256<float> tt2 = Avx.Shuffle(t1, t3, 0b01000100);
        Vector256<float> tt3 = Avx.Shuffle(t1, t3, 0b11101110);
        Vector256<float> tt4 = Avx.Shuffle(t4, t6, 0b01000100);
        Vector256<float> tt5 = Avx.Shuffle(t4, t6, 0b11101110);
        Vector256<float> tt6 = Avx.Shuffle(t5, t7, 0b01000100);
        Vector256<float> tt7 = Avx.Shuffle(t5, t7, 0b11101110);
        int i1 = indices[0] - 1, i2 = indices[1] - 1, i3 = indices[2] - 1, i4 = indices[3] - 1,
            i5 = indices[4] - 1, i6 = indices[5] - 1, i7 = indices[6] - 1, i8 = indices[7] - 1;
        if (i1 != -1 && states[i1].flags.HasFlag(BodyFlags.Dynamic)) Avx.StoreAligned((float*)(states + i1), Avx.Permute2x128(tt0, tt4, 0x20));
        if (i2 != -1 && states[i2].flags.HasFlag(BodyFlags.Dynamic)) Avx.StoreAligned((float*)(states + i2), Avx.Permute2x128(tt1, tt5, 0x20));
        if (i3 != -1 && states[i3].flags.HasFlag(BodyFlags.Dynamic)) Avx.StoreAligned((float*)(states + i3), Avx.Permute2x128(tt2, tt6, 0x20));
        if (i4 != -1 && states[i4].flags.HasFlag(BodyFlags.Dynamic)) Avx.StoreAligned((float*)(states + i4), Avx.Permute2x128(tt3, tt7, 0x20));
        if (i5 != -1 && states[i5].flags.HasFlag(BodyFlags.Dynamic)) Avx.StoreAligned((float*)(states + i5), Avx.Permute2x128(tt0, tt4, 0x31));
        if (i6 != -1 && states[i6].flags.HasFlag(BodyFlags.Dynamic)) Avx.StoreAligned((float*)(states + i6), Avx.Permute2x128(tt1, tt5, 0x31));
        if (i7 != -1 && states[i7].flags.HasFlag(BodyFlags.Dynamic)) Avx.StoreAligned((float*)(states + i7), Avx.Permute2x128(tt2, tt6, 0x31));
        if (i8 != -1 && states[i8].flags.HasFlag(BodyFlags.Dynamic)) Avx.StoreAligned((float*)(states + i8), Avx.Permute2x128(tt3, tt7, 0x31));
    }
    [System.Runtime.CompilerServices.InlineArray(8)] struct ContactSimLanes { public ContactSim sim; }
    public unsafe void PrepareContacts_Wide(ref SolverBlock block, StepContext context)
    {
        World world = context.world;
        var spans = context.contactPrepareSpans;
        var wideBase = (ContactConstraintsAVX)context.wideContactConstraints;
        Vector256<float> warmStartScale = world.enableWarmStarting ? Vector256<float>.One : Vector256<float>.Zero;
        int wideIndex = block.startIndex, endWideIndex = block.startIndex + block.count;
        int colorIndex = 0;
        while (spans[colorIndex + 1].start <= wideIndex) colorIndex++;
        while (wideIndex < endWideIndex)
        {
            int colorWideStart = spans[colorIndex].start;
            int colorWideEndIndex = Math.Min(spans[colorIndex + 1].start, endWideIndex);
            int colorContactCount = spans[colorIndex].count;
            var contactSims = spans[colorIndex].contacts;
            ContactSimLanes contactLanes = new();
            for (; wideIndex < colorWideEndIndex; wideIndex++)
            {
                var cw = wideBase.wideConstraints + wideIndex;
                int localWideIndex = wideIndex - colorWideStart;
                for (int laneIndex = 0; laneIndex < 8; laneIndex++)
                {
                    int contactIndex = 8 * localWideIndex + laneIndex;
                    if (contactIndex < colorContactCount)
                    {
                        ContactSim c = contactSims[contactIndex];
                        contactLanes[laneIndex] = c;
                        ((int*)&cw->indexA)[laneIndex] = c.bodySimIndexA + 1;
                        ((int*)&cw->indexB)[laneIndex] = c.bodySimIndexB + 1;
#if B2_VALIDATE
                        Body bodyA = world.bodies[c.bodyIdA];
                        int validIndexA = bodyA.setIndex == (int)SetType.Awake ? bodyA.localIndex : -1;
                        Body bodyB = world.bodies[c.bodyIdB];
                        int validIndexB = bodyB.setIndex == (int)SetType.Awake ? bodyB.localIndex : -1;
                        Debug.Assert(c.bodyIdA == validIndexA);
                        Debug.Assert(c.bodyIdB == validIndexB);
#endif
                    }
                    else contactLanes[laneIndex] = ContactSim.Zero;
                }
                cw->invMassA = Vector256.Create(contactLanes[0].invMassA, contactLanes[1].invMassA, contactLanes[2].invMassA, contactLanes[3].invMassA, contactLanes[4].invMassA, contactLanes[5].invMassA, contactLanes[6].invMassA, contactLanes[7].invMassA);
                cw->invMassB = Vector256.Create(contactLanes[0].invMassB, contactLanes[1].invMassB, contactLanes[2].invMassB, contactLanes[3].invMassB, contactLanes[4].invMassB, contactLanes[5].invMassB, contactLanes[6].invMassB, contactLanes[7].invMassB);
                cw->invIA = Vector256.Create(contactLanes[0].invIA, contactLanes[1].invIA, contactLanes[2].invIA, contactLanes[3].invIA, contactLanes[4].invIA, contactLanes[5].invIA, contactLanes[6].invIA, contactLanes[7].invIA);
                cw->invIB = Vector256.Create(contactLanes[0].invIB, contactLanes[1].invIB, contactLanes[2].invIB, contactLanes[3].invIB, contactLanes[4].invIB, contactLanes[5].invIB, contactLanes[6].invIB, contactLanes[7].invIB);
                cw->normal.X = Vector256.Create(contactLanes[0].manifold.normal.x, contactLanes[1].manifold.normal.x, contactLanes[2].manifold.normal.x, contactLanes[3].manifold.normal.x, contactLanes[4].manifold.normal.x, contactLanes[5].manifold.normal.x, contactLanes[6].manifold.normal.x, contactLanes[7].manifold.normal.x);
                cw->normal.Y = Vector256.Create(contactLanes[0].manifold.normal.y, contactLanes[1].manifold.normal.y, contactLanes[2].manifold.normal.y, contactLanes[3].manifold.normal.y, contactLanes[4].manifold.normal.y, contactLanes[5].manifold.normal.y, contactLanes[6].manifold.normal.y, contactLanes[7].manifold.normal.y);
                cw->friction = Vector256.Create(contactLanes[0].friction, contactLanes[1].friction, contactLanes[2].friction, contactLanes[3].friction, contactLanes[4].friction, contactLanes[5].friction, contactLanes[6].friction, contactLanes[7].friction);
                cw->tangentSpeed = Vector256.Create(contactLanes[0].tangentSpeed, contactLanes[1].tangentSpeed, contactLanes[2].tangentSpeed, contactLanes[3].tangentSpeed, contactLanes[4].tangentSpeed, contactLanes[5].tangentSpeed, contactLanes[6].tangentSpeed, contactLanes[7].tangentSpeed);
                cw->rollingResistance = Vector256.Create(contactLanes[0].rollingResistance, contactLanes[1].rollingResistance, contactLanes[2].rollingResistance, contactLanes[3].rollingResistance, contactLanes[4].rollingResistance, contactLanes[5].rollingResistance, contactLanes[6].rollingResistance, contactLanes[7].rollingResistance);
                cw->rollingImpulse = Vector256.Create(contactLanes[0].manifold.rollingImpulse, contactLanes[1].manifold.rollingImpulse, contactLanes[2].manifold.rollingImpulse, contactLanes[3].manifold.rollingImpulse, contactLanes[4].manifold.rollingImpulse, contactLanes[5].manifold.rollingImpulse, contactLanes[6].manifold.rollingImpulse, contactLanes[7].manifold.rollingImpulse);
                cw->rollingImpulse = Avx.Multiply(warmStartScale, cw->rollingImpulse);
                Vector2W tangent = RightPerpW(cw->normal);
                {
                    Vector256<float> m1; fixed (Manifold* m = &contactLanes[0].manifold) m1 = Avx.LoadVector256(&m->point0.anchorA.x);
                    Vector256<float> m2; fixed (Manifold* m = &contactLanes[1].manifold) m2 = Avx.LoadVector256(&m->point0.anchorA.x);
                    Vector256<float> m3; fixed (Manifold* m = &contactLanes[2].manifold) m3 = Avx.LoadVector256(&m->point0.anchorA.x);
                    Vector256<float> m4; fixed (Manifold* m = &contactLanes[3].manifold) m4 = Avx.LoadVector256(&m->point0.anchorA.x);
                    Vector256<float> m5; fixed (Manifold* m = &contactLanes[4].manifold) m5 = Avx.LoadVector256(&m->point0.anchorA.x);
                    Vector256<float> m6; fixed (Manifold* m = &contactLanes[5].manifold) m6 = Avx.LoadVector256(&m->point0.anchorA.x);
                    Vector256<float> m7; fixed (Manifold* m = &contactLanes[6].manifold) m7 = Avx.LoadVector256(&m->point0.anchorA.x);
                    Vector256<float> m8; fixed (Manifold* m = &contactLanes[7].manifold) m8 = Avx.LoadVector256(&m->point0.anchorA.x);
                    Vector256<float> t1 = Avx.UnpackLow(m1, m2), t2 = Avx.UnpackHigh(m1, m2);
                    Vector256<float> t3 = Avx.UnpackLow(m3, m4), t4 = Avx.UnpackHigh(m3, m4);
                    Vector256<float> t5 = Avx.UnpackLow(m5, m6), t6 = Avx.UnpackHigh(m5, m6);
                    Vector256<float> t7 = Avx.UnpackLow(m7, m8), t8 = Avx.UnpackHigh(m7, m8);
                    Vector256<float> tt1 = Avx.Shuffle(t1, t3, 0b01000100), tt2 = Avx.Shuffle(t1, t3, 0b11101110);
                    Vector256<float> tt3 = Avx.Shuffle(t2, t4, 0b01000100), tt4 = Avx.Shuffle(t2, t4, 0b11101110);
                    Vector256<float> tt5 = Avx.Shuffle(t5, t7, 0b01000100), tt6 = Avx.Shuffle(t5, t7, 0b11101110);
                    Vector256<float> tt7 = Avx.Shuffle(t6, t8, 0b01000100), tt8 = Avx.Shuffle(t6, t8, 0b11101110);
                    cw->anchorA1.X = Avx.Permute2x128(tt1, tt5, 0x20);
                    cw->anchorA1.Y = Avx.Permute2x128(tt2, tt6, 0x20);
                    cw->anchorB1.X = Avx.Permute2x128(tt3, tt7, 0x20);
                    cw->anchorB1.Y = Avx.Permute2x128(tt4, tt8, 0x20);
                    cw->baseSeparation1 = Avx.Permute2x128(tt1, tt5, 0x31);
                    cw->normalImpulse1 = Avx.Permute2x128(tt2, tt6, 0x31);
                    cw->tangentImpulse1 = Avx.Permute2x128(tt3, tt7, 0x31);
                    cw->negRestitutionVelocity1 = Avx.Permute2x128(tt4, tt8, 0x31);

                    Vector256<float> offset = DotW(cw->anchorB1 - cw->anchorA1, cw->normal);
                    cw->baseSeparation1 = cw->baseSeparation1 - offset;
                    cw->negRestitutionVelocity1 = -cw->negRestitutionVelocity1;
                    cw->normalImpulse1 = warmStartScale * cw->normalImpulse1;
                    cw->tangentImpulse1 = warmStartScale * cw->tangentImpulse1;
                    cw->totalNormalImpulse1 = Vector256<float>.Zero;
                    {
                        Vector256<float> rnA = CrossW(cw->anchorA1, cw->normal);
                        Vector256<float> rnB = CrossW(cw->anchorB1, cw->normal);
                        Vector256<float> k = cw->invMassA + cw->invMassB + cw->invIA * rnA * rnA + cw->invIB * rnB * rnB;
                        cw->normalMass1 = Avx.BlendVariable(Vector256<float>.Zero, Vector256<float>.One / k, Avx.CompareGreaterThan(k, Vector256<float>.Zero));
                    }
                    {
                        Vector256<float> rnA = CrossW(cw->anchorA1, tangent);
                        Vector256<float> rnB = CrossW(cw->anchorB1, tangent);
                        Vector256<float> k = cw->invMassA + cw->invMassB + cw->invIA * rnA * rnA + cw->invIB * rnB * rnB;
                        cw->tangentMass1 = Avx.BlendVariable(Vector256<float>.Zero, Vector256<float>.One / k, Avx.CompareGreaterThan(k, Vector256<float>.Zero));
                    }
                }
                {
                    Vector256<float> m1; fixed (Manifold* m = &contactLanes[0].manifold) m1 = Avx.LoadVector256(&m->point1.anchorA.x);
                    Vector256<float> m2; fixed (Manifold* m = &contactLanes[1].manifold) m2 = Avx.LoadVector256(&m->point1.anchorA.x);
                    Vector256<float> m3; fixed (Manifold* m = &contactLanes[2].manifold) m3 = Avx.LoadVector256(&m->point1.anchorA.x);
                    Vector256<float> m4; fixed (Manifold* m = &contactLanes[3].manifold) m4 = Avx.LoadVector256(&m->point1.anchorA.x);
                    Vector256<float> m5; fixed (Manifold* m = &contactLanes[4].manifold) m5 = Avx.LoadVector256(&m->point1.anchorA.x);
                    Vector256<float> m6; fixed (Manifold* m = &contactLanes[5].manifold) m6 = Avx.LoadVector256(&m->point1.anchorA.x);
                    Vector256<float> m7; fixed (Manifold* m = &contactLanes[6].manifold) m7 = Avx.LoadVector256(&m->point1.anchorA.x);
                    Vector256<float> m8; fixed (Manifold* m = &contactLanes[7].manifold) m8 = Avx.LoadVector256(&m->point1.anchorA.x);
                    Vector256<float> t1 = Avx.UnpackLow(m1, m2), t2 = Avx.UnpackHigh(m1, m2);
                    Vector256<float> t3 = Avx.UnpackLow(m3, m4), t4 = Avx.UnpackHigh(m3, m4);
                    Vector256<float> t5 = Avx.UnpackLow(m5, m6), t6 = Avx.UnpackHigh(m5, m6);
                    Vector256<float> t7 = Avx.UnpackLow(m7, m8), t8 = Avx.UnpackHigh(m7, m8);
                    Vector256<float> tt1 = Avx.Shuffle(t1, t3, 0b01000100), tt2 = Avx.Shuffle(t1, t3, 0b11101110);
                    Vector256<float> tt3 = Avx.Shuffle(t2, t4, 0b01000100), tt4 = Avx.Shuffle(t2, t4, 0b11101110);
                    Vector256<float> tt5 = Avx.Shuffle(t5, t7, 0b01000100), tt6 = Avx.Shuffle(t5, t7, 0b11101110);
                    Vector256<float> tt7 = Avx.Shuffle(t6, t8, 0b01000100), tt8 = Avx.Shuffle(t6, t8, 0b11101110);
                    cw->anchorA2.X = Avx.Permute2x128(tt1, tt5, 0x20);
                    cw->anchorA2.Y = Avx.Permute2x128(tt2, tt6, 0x20);
                    cw->anchorB2.X = Avx.Permute2x128(tt3, tt7, 0x20);
                    cw->anchorB2.Y = Avx.Permute2x128(tt4, tt8, 0x20);
                    cw->baseSeparation2 = Avx.Permute2x128(tt1, tt5, 0x31);
                    cw->normalImpulse2 = Avx.Permute2x128(tt2, tt6, 0x31);
                    cw->tangentImpulse2 = Avx.Permute2x128(tt3, tt7, 0x31);
                    cw->negRestitutionVelocity2 = Avx.Permute2x128(tt4, tt8, 0x31);

                    Vector256<float> offset = DotW(cw->anchorB2 - cw->anchorA2, cw->normal);
                    cw->baseSeparation2 = cw->baseSeparation2 - offset;
                    cw->negRestitutionVelocity2 = -cw->negRestitutionVelocity2;
                    cw->normalImpulse2 = warmStartScale * cw->normalImpulse2;
                    cw->tangentImpulse2 = warmStartScale * cw->tangentImpulse2;
                    cw->totalNormalImpulse2 = Vector256<float>.Zero;
                    {
                        Vector256<float> rnA = CrossW(cw->anchorA2, cw->normal);
                        Vector256<float> rnB = CrossW(cw->anchorB2, cw->normal);
                        Vector256<float> k = cw->invMassA + cw->invMassB + cw->invIA * rnA * rnA + cw->invIB * rnB * rnB;
                        cw->normalMass2 = Avx.BlendVariable(Vector256<float>.Zero, Vector256<float>.One / k, Avx.CompareGreaterThan(k, Vector256<float>.Zero));
                    }
                    {
                        Vector256<float> rnA = CrossW(cw->anchorA2, tangent);
                        Vector256<float> rnB = CrossW(cw->anchorB2, tangent);
                        Vector256<float> k = cw->invMassA + cw->invMassB + cw->invIA * rnA * rnA + cw->invIB * rnB * rnB;
                        cw->tangentMass2 = Avx.BlendVariable(Vector256<float>.Zero, Vector256<float>.One / k, Avx.CompareGreaterThan(k, Vector256<float>.Zero));
                    }
                }
                Vector256<float> massScale = Vector256.GreaterThan(Vector256.Create(contactLanes[0].manifold.pointCount, contactLanes[1].manifold.pointCount, contactLanes[2].manifold.pointCount, contactLanes[3].manifold.pointCount,
                    contactLanes[4].manifold.pointCount, contactLanes[5].manifold.pointCount, contactLanes[6].manifold.pointCount, contactLanes[7].manifold.pointCount), Vector256<int>.One).AsSingle();
                cw->normalMass2 = Avx.BlendVariable(Vector256<float>.Zero, cw->normalMass2, massScale);
                cw->tangentMass2 = Avx.BlendVariable(Vector256<float>.Zero, cw->tangentMass2, massScale);
            }
            colorIndex++;
        }
    }
    public unsafe void WarmStartContacts_Wide(ref SolverBlock block, StepContext context)
    {
        var states = context.states.Data;
        var constraints = ((ContactConstraintsAVX)context.graph.colors[block.colorIndex].wideConstraints).wideConstraints;
        {
            for (int i = block.startIndex; i < block.startIndex + block.count; i++)
            {
                ContactConstraintWide* c = constraints + i;
                BodyStateW bA = GatherBodies(states, (int*)&c->indexA);
                BodyStateW bB = GatherBodies(states, (int*)&c->indexB);
                Vector256<float> tangentX = c->normal.Y;
                Vector256<float> tangentY = Avx.Subtract(Vector256<float>.Zero, c->normal.X);
                {
                    Vector2W rA = c->anchorA1, rB = c->anchorB1;
                    Vector2W P = new()
                    {
                        X = Avx.Add(Avx.Multiply(c->normalImpulse1, c->normal.X), Avx.Multiply(c->tangentImpulse1, tangentX)),
                        Y = Avx.Add(Avx.Multiply(c->normalImpulse1, c->normal.Y), Avx.Multiply(c->tangentImpulse1, tangentY))
                    };
                    bA.w = Avx.Subtract(bA.w, Avx.Multiply(c->invIA, CrossW(rA, P)));
                    bA.v = new()
                    {
                        X = Avx.Subtract(bA.v.X, Avx.Multiply(c->invMassA, P.X)),
                        Y = Avx.Subtract(bA.v.Y, Avx.Multiply(c->invMassA, P.Y))
                    };
                    bB.w = Avx.Add(bB.w, Avx.Multiply(c->invIB, CrossW(rB, P)));
                    bB.v = new()
                    {
                        X = Avx.Add(bB.v.X, Avx.Multiply(c->invMassB, P.X)),
                        Y = Avx.Add(bB.v.Y, Avx.Multiply(c->invMassB, P.Y))
                    };
                    c->totalNormalImpulse1 = Avx.Add(c->totalNormalImpulse1, c->normalImpulse1);
                }
                {
                    Vector2W rA = c->anchorA2, rB = c->anchorB2;
                    Vector2W P = new()
                    {
                        X = Avx.Add(Avx.Multiply(c->normalImpulse2, c->normal.X), Avx.Multiply(c->tangentImpulse2, tangentX)),
                        Y = Avx.Add(Avx.Multiply(c->normalImpulse2, c->normal.Y), Avx.Multiply(c->tangentImpulse2, tangentY))
                    };
                    bA.w = Avx.Subtract(bA.w, Avx.Multiply(c->invIA, CrossW(rA, P)));
                    bA.v = new()
                    {
                        X = Avx.Subtract(bA.v.X, Avx.Multiply(c->invMassA, P.X)),
                        Y = Avx.Subtract(bA.v.Y, Avx.Multiply(c->invMassA, P.Y))
                    };
                    bB.w = Avx.Add(bB.w, Avx.Multiply(c->invIB, CrossW(rB, P)));
                    bB.v = new()
                    {
                        X = Avx.Add(bB.v.X, Avx.Multiply(c->invMassB, P.X)),
                        Y = Avx.Add(bB.v.Y, Avx.Multiply(c->invMassB, P.Y))
                    };
                    c->totalNormalImpulse2 = Avx.Add(c->totalNormalImpulse2, c->normalImpulse2);
                }
                bA.w = Avx.Subtract(bA.w, Avx.Multiply(c->invIA, c->rollingImpulse));
                bB.w = Avx.Add(bB.w, Avx.Multiply(c->invIB, c->rollingImpulse));
                ScatterBodies(states, (int*)&c->indexA, ref bA);
                ScatterBodies(states, (int*)&c->indexB, ref bB);
            }
        }
    }
    public unsafe void PushContacts_Wide(ref SolverBlock block, StepContext context)
    {
        var states = context.states.Data;
        var constraints = ((ContactConstraintsAVX)context.graph.colors[block.colorIndex].wideConstraints).wideConstraints;
        {
            Vector256<float> inv_h = Vector256.Create(context.inv_h);
            Vector256<float> contactSpeed = Vector256.Create(-context.world.contactSpeed);
            Vector256<float> oneW = Vector256<float>.One;
            Vector256<float> dynamicBiasRate = Vector256.Create(context.contactSoftness.massScale * context.contactSoftness.biasRate);
            Vector256<float> dynamicMassScale = Vector256.Create(context.contactSoftness.massScale);
            Vector256<float> dynamicImpulseScale = Vector256.Create(context.contactSoftness.impulseScale);
            Vector256<float> staticBiasRate = Vector256.Create(context.staticSoftness.massScale * context.staticSoftness.biasRate);
            Vector256<float> staticMassScale = Vector256.Create(context.staticSoftness.massScale);
            Vector256<float> staticImpulseScale = Vector256.Create(context.staticSoftness.impulseScale);
            for (int wideIndex = block.startIndex; wideIndex < block.startIndex + block.count; wideIndex++)
            {
                ContactConstraintWide* c = constraints + wideIndex;
                BodyStateW bA = GatherBodies(states, (int*)&c->indexA);
                BodyStateW bB = GatherBodies(states, (int*)&c->indexB);
                Vector256<float> softMask = SoftMaskW(c->indexA, c->indexB),
                    biasRate = Avx.BlendVariable(dynamicBiasRate, staticBiasRate, softMask),
                    massScale = Avx.BlendVariable(dynamicMassScale, staticMassScale, softMask),
                    impulseScale = Avx.BlendVariable(dynamicImpulseScale, staticImpulseScale, softMask);
                Vector2W dp = new() { X = Avx.Subtract(bB.dp.X, bA.dp.X), Y = Avx.Subtract(bB.dp.Y, bA.dp.Y) };
                {
                    Vector2W rA = c->anchorA1, rB = c->anchorB1;
                    Vector2W rsA = RotateVectorW(bA.dq, rA), rsB = RotateVectorW(bB.dq, rB);
                    Vector2W ds = new() { X = Avx.Add(dp.X, Avx.Subtract(rsB.X, rsA.X)), Y = Avx.Add(dp.Y, Avx.Subtract(rsB.Y, rsA.Y)) };
                    Vector256<float> s = Avx.Add(DotW(c->normal, ds), c->baseSeparation1);
                    Vector256<float> separated = Avx.CompareGreaterThan(s, Vector256<float>.Zero);
                    Vector256<float> specBias = Avx.Multiply(s, inv_h), overlapBias = Avx.Max(Avx.Multiply(biasRate, s), contactSpeed);
                    Vector256<float> velocityBias = Avx.BlendVariable(overlapBias, specBias, separated);
                    Vector256<float> pointMassScale = Avx.BlendVariable(massScale, oneW, separated);
                    Vector256<float> pointImpulseScale = Avx.BlendVariable(impulseScale, Vector256<float>.Zero, separated);
                    Vector256<float> dvx = Avx.Subtract(Avx.Subtract(bB.v.X, Avx.Multiply(bB.w, rB.Y)), Avx.Subtract(bA.v.X, Avx.Multiply(bA.w, rA.Y)));
                    Vector256<float> dvy = Avx.Subtract(Avx.Add(bB.v.Y, Avx.Multiply(bB.w, rB.X)), Avx.Add(bA.v.Y, Avx.Multiply(bA.w, rA.X)));
                    Vector256<float> vn = Avx.Add(Avx.Multiply(dvx, c->normal.X), Avx.Multiply(dvy, c->normal.Y));
                    Vector256<float> negImpulse = Avx.Add(Avx.Multiply(c->normalMass1, Avx.Add(Avx.Multiply(pointMassScale, vn), velocityBias)), Avx.Multiply(pointImpulseScale, c->normalImpulse1));
                    Vector256<float> newImpulse = Avx.Max(Avx.Subtract(c->normalImpulse1, negImpulse), Vector256<float>.Zero);
                    Vector256<float> impulse = Avx.Subtract(newImpulse, c->normalImpulse1);
                    c->normalImpulse1 = newImpulse;
                    c->totalNormalImpulse1 = Avx.Add(c->totalNormalImpulse1, impulse);
                    Vector256<float> Px = Avx.Multiply(impulse, c->normal.X);
                    Vector256<float> Py = Avx.Multiply(impulse, c->normal.Y);
                    bA.v = new()
                    {
                        X = Avx.Subtract(bA.v.X, Avx.Multiply(c->invMassA, Px)),
                        Y = Avx.Subtract(bA.v.Y, Avx.Multiply(c->invMassA, Py))
                    };
                    bA.w = Avx.Subtract(bA.w, Avx.Multiply(c->invIA, Avx.Subtract(Avx.Multiply(rA.X, Py), Avx.Multiply(rA.Y, Px))));
                    bB.v = new()
                    {
                        X = Avx.Add(bB.v.X, Avx.Multiply(c->invMassB, Px)),
                        Y = Avx.Add(bB.v.Y, Avx.Multiply(c->invMassB, Py))
                    };
                    bB.w = Avx.Add(bB.w, Avx.Multiply(c->invIB, Avx.Subtract(Avx.Multiply(rB.X, Py), Avx.Multiply(rB.Y, Px))));
                }
                {
                    Vector2W rA = c->anchorA2, rB = c->anchorB2;
                    Vector2W rsA = RotateVectorW(bA.dq, rA), rsB = RotateVectorW(bB.dq, rB);
                    Vector2W ds = new() { X = Avx.Add(dp.X, Avx.Subtract(rsB.X, rsA.X)), Y = Avx.Add(dp.Y, Avx.Subtract(rsB.Y, rsA.Y)) };
                    Vector256<float> s = Avx.Add(DotW(c->normal, ds), c->baseSeparation2);
                    Vector256<float> separated = Avx.CompareGreaterThan(s, Vector256<float>.Zero);
                    Vector256<float> specBias = Avx.Multiply(s, inv_h), overlapBias = Avx.Max(Avx.Multiply(biasRate, s), contactSpeed);
                    Vector256<float> velocityBias = Avx.BlendVariable(overlapBias, specBias, separated);
                    Vector256<float> pointMassScale = Avx.BlendVariable(massScale, oneW, separated);
                    Vector256<float> pointImpulseScale = Avx.BlendVariable(impulseScale, Vector256<float>.Zero, separated);
                    Vector256<float> dvx = Avx.Subtract(Avx.Subtract(bB.v.X, Avx.Multiply(bB.w, rB.Y)), Avx.Subtract(bA.v.X, Avx.Multiply(bA.w, rA.Y)));
                    Vector256<float> dvy = Avx.Subtract(Avx.Add(bB.v.Y, Avx.Multiply(bB.w, rB.X)), Avx.Add(bA.v.Y, Avx.Multiply(bA.w, rA.X)));
                    Vector256<float> vn = Avx.Add(Avx.Multiply(dvx, c->normal.X), Avx.Multiply(dvy, c->normal.Y));
                    Vector256<float> negImpulse = Avx.Add(Avx.Multiply(c->normalMass2, Avx.Add(Avx.Multiply(pointMassScale, vn), velocityBias)), Avx.Multiply(pointImpulseScale, c->normalImpulse2));
                    Vector256<float> newImpulse = Avx.Max(Avx.Subtract(c->normalImpulse2, negImpulse), Vector256<float>.Zero);
                    Vector256<float> impulse = Avx.Subtract(newImpulse, c->normalImpulse2);
                    c->normalImpulse2 = newImpulse;
                    c->totalNormalImpulse2 = Avx.Add(c->totalNormalImpulse2, impulse);
                    Vector256<float> Px = Avx.Multiply(impulse, c->normal.X);
                    Vector256<float> Py = Avx.Multiply(impulse, c->normal.Y);
                    bA.v = new()
                    {
                        X = Avx.Subtract(bA.v.X, Avx.Multiply(c->invMassA, Px)),
                        Y = Avx.Subtract(bA.v.Y, Avx.Multiply(c->invMassA, Py))
                    };
                    bA.w = Avx.Subtract(bA.w, Avx.Multiply(c->invIA, Avx.Subtract(Avx.Multiply(rA.X, Py), Avx.Multiply(rA.Y, Px))));
                    bB.v = new()
                    {
                        X = Avx.Add(bB.v.X, Avx.Multiply(c->invMassB, Px)),
                        Y = Avx.Add(bB.v.Y, Avx.Multiply(c->invMassB, Py))
                    };
                    bB.w = Avx.Add(bB.w, Avx.Multiply(c->invIB, Avx.Subtract(Avx.Multiply(rB.X, Py), Avx.Multiply(rB.Y, Px))));
                }
                ScatterBodies(states, (int*)&c->indexA, ref bA);
                ScatterBodies(states, (int*)&c->indexB, ref bB);
            }
        }
    }
    public unsafe void SolveContacts_Wide(ref SolverBlock block, StepContext context)
    {
        var states = context.states.Data;
        GraphColor color = context.graph.colors[block.colorIndex];
        var constraints = ((ContactConstraintsAVX)context.graph.colors[block.colorIndex].wideConstraints).wideConstraints;
        Vector256<float> inv_h = Vector256.Create(context.inv_h);
        for (int wideIndex = 0; wideIndex < block.startIndex + block.count; wideIndex++)
        {
            ContactConstraintWide* c = constraints + wideIndex;
            BodyStateW bA = GatherBodies(states, (int*)&c->indexA);
            BodyStateW bB = GatherBodies(states, (int*)&c->indexB);
            Vector256<float> resitutionMask1 = Avx.CompareGreaterThan(Vector256<float>.Zero, c->negRestitutionVelocity1);
            Vector256<float> resitutionMask2 = Avx.CompareGreaterThan(Vector256<float>.Zero, c->negRestitutionVelocity2);
            bool haveResitution = !AllZeroW(Avx.Or(resitutionMask1, resitutionMask2));
            Vector256<float> keepRestitution = Vector256<float>.Zero;
            Vector256<float> totalNormalImpulse = Vector256<float>.Zero;
            Vector2W dp = bB.dp - bA.dp;
            {
                Vector2W rA = c->anchorA1, rB = c->anchorB1;
                Vector2W rsA = RotateVectorW(bA.dq, rA), rsB = RotateVectorW(bB.dq, rB);
                Vector2W ds = new() { X = dp.X + (rsB.X - rsA.X), Y = dp.Y + (rsB.Y - rsA.Y) };
                Vector256<float> s = DotW(c->normal, ds) + c->baseSeparation1;
                Vector256<float> specBias = s * inv_h;
                Vector256<float> velocityBias = Avx.Max(Vector256<float>.Zero, specBias);
                if (haveResitution)
                {
                    Vector256<float> separated = Avx.CompareGreaterThan(s, Vector256<float>.Zero);
                    velocityBias = Avx.BlendVariable(velocityBias, c->negRestitutionVelocity1, resitutionMask1);
                    keepRestitution = Avx.Or(keepRestitution, Avx.AndNot(resitutionMask1, separated));
                }
                Vector256<float> dvx = (bB.v.X - bB.w * rB.Y) - (bA.v.X - bA.w * rA.Y);
                Vector256<float> dvy = (bB.v.Y + bB.w * rB.X) - (bA.v.Y + bA.w * rA.X);
                Vector256<float> vn = dvx * c->normal.X + dvy * c->normal.Y;
                Vector256<float> newImpulse = Avx.Max(c->normalImpulse1 - c->normalMass1 * (vn + velocityBias), Vector256<float>.Zero);
                Vector256<float> impulse = newImpulse - c->normalImpulse1;
                c->normalImpulse1 = newImpulse;
                c->totalNormalImpulse1 = c->totalNormalImpulse1 + impulse;
                totalNormalImpulse = totalNormalImpulse + newImpulse;
                Vector256<float> Px = impulse * c->normal.X, Py = impulse * c->normal.Y;
                bA.v = new()
                {
                    X = Avx.Subtract(bA.v.X, Avx.Multiply(c->invMassA, Px)),
                    Y = Avx.Subtract(bA.v.Y, Avx.Multiply(c->invMassA, Py))
                };
                bA.w = Avx.Subtract(bA.w, Avx.Multiply(c->invIA, Avx.Subtract(Avx.Multiply(rA.X, Py), Avx.Multiply(rA.Y, Px))));
                bB.v = new()
                {
                    X = Avx.Add(bB.v.X, Avx.Multiply(c->invMassB, Px)),
                    Y = Avx.Add(bB.v.Y, Avx.Multiply(c->invMassB, Py))
                };
                bB.w = Avx.Add(bB.w, Avx.Multiply(c->invIB, Avx.Subtract(Avx.Multiply(rB.X, Py), Avx.Multiply(rB.Y, Px))));
            }
            {
                Vector2W rA = c->anchorA2, rB = c->anchorB2;
                Vector2W rsA = RotateVectorW(bA.dq, rA), rsB = RotateVectorW(bB.dq, rB);
                Vector2W ds = new() { X = dp.X + (rsB.X - rsA.X), Y = dp.Y + (rsB.Y - rsA.Y) };
                Vector256<float> s = DotW(c->normal, ds) + c->baseSeparation2;
                Vector256<float> specBias = s * inv_h;
                Vector256<float> velocityBias = Avx.Max(Vector256<float>.Zero, specBias);
                if (haveResitution)
                {
                    Vector256<float> separated = Avx.CompareGreaterThan(s, Vector256<float>.Zero);
                    velocityBias = Avx.BlendVariable(velocityBias, c->negRestitutionVelocity2, resitutionMask2);
                    keepRestitution = Avx.Or(keepRestitution, Avx.AndNot(resitutionMask2, separated));
                }
                Vector256<float> dvx = (bB.v.X - bB.w * rB.Y) - (bA.v.X - bA.w * rA.Y);
                Vector256<float> dvy = (bB.v.Y + bB.w * rB.X) - (bA.v.Y + bA.w * rA.X);
                Vector256<float> vn = dvx * c->normal.X + dvy * c->normal.Y;
                Vector256<float> newImpulse = Avx.Max(c->normalImpulse2 - c->normalMass2 * (vn + velocityBias), Vector256<float>.Zero);
                Vector256<float> impulse = newImpulse - c->normalImpulse2;
                c->normalImpulse2 = newImpulse;
                c->totalNormalImpulse2 = c->totalNormalImpulse2 + impulse;
                totalNormalImpulse = totalNormalImpulse + newImpulse;
                Vector256<float> Px = impulse * c->normal.X, Py = impulse * c->normal.Y;
                bA.v = new()
                {
                    X = Avx.Subtract(bA.v.X, Avx.Multiply(c->invMassA, Px)),
                    Y = Avx.Subtract(bA.v.Y, Avx.Multiply(c->invMassA, Py))
                };
                bA.w = Avx.Subtract(bA.w, Avx.Multiply(c->invIA, Avx.Subtract(Avx.Multiply(rA.X, Py), Avx.Multiply(rA.Y, Px))));
                bB.v = new()
                {
                    X = Avx.Add(bB.v.X, Avx.Multiply(c->invMassB, Px)),
                    Y = Avx.Add(bB.v.Y, Avx.Multiply(c->invMassB, Py))
                };
                bB.w = Avx.Add(bB.w, Avx.Multiply(c->invIB, Avx.Subtract(Avx.Multiply(rB.X, Py), Avx.Multiply(rB.Y, Px))));
            }
            c->negRestitutionVelocity1 = Avx.BlendVariable(Vector256<float>.Zero, c->negRestitutionVelocity1, keepRestitution);
            c->negRestitutionVelocity2 = Avx.BlendVariable(Vector256<float>.Zero, c->negRestitutionVelocity2, keepRestitution);
            if (!AllZeroW(c->rollingResistance))
            {
                Vector256<float> k = c->invIA + c->invIB;
                Vector256<float> deltaLambda = Avx.BlendVariable(Vector256<float>.Zero, Avx.Divide(Avx.Subtract(bA.w, bB.w), k), Avx.CompareGreaterThan(k, Vector256<float>.Zero));
                Vector256<float> lambda = c->rollingImpulse;
                Vector256<float> maxLambda = Avx.Multiply(c->rollingResistance, totalNormalImpulse);
                c->rollingImpulse = SymClampW(Avx.Add(lambda, deltaLambda), maxLambda);
                deltaLambda = Avx.Subtract(c->rollingImpulse, lambda);
                bA.w = Avx.Subtract(bA.w, Avx.Multiply(c->invIA, deltaLambda));
                bB.w = Avx.Add(bB.w, Avx.Multiply(c->invIB, deltaLambda));
            }
            Vector256<float> tangentX = c->normal.Y;
            Vector256<float> tangentY = Avx.Subtract(Vector256<float>.Zero, c->normal.X);
            {
                Vector2W rA = c->anchorA1, rB = c->anchorB1;
                Vector256<float> dvx = Avx.Subtract(Avx.Subtract(bB.v.X, Avx.Multiply(bB.w, rB.Y)), Avx.Subtract(bA.v.X, Avx.Multiply(bA.w, rA.Y)));
                Vector256<float> dvy = Avx.Subtract(Avx.Add(bB.v.Y, Avx.Multiply(bB.w, rB.X)), Avx.Add(bA.v.Y, Avx.Multiply(bA.w, rA.X)));
                Vector256<float> vt = Avx.Add(Avx.Multiply(dvx, tangentX), Avx.Multiply(dvy, tangentY));
                vt = Avx.Subtract(vt, c->tangentSpeed);
                Vector256<float> negImpulse = Avx.Multiply(c->tangentMass1, vt);
                Vector256<float> maxFriction = Avx.Multiply(c->friction, c->normalImpulse1);
                Vector256<float> newImpulse = Avx.Subtract(c->tangentImpulse1, negImpulse);
                //no symclamp?
                newImpulse = Avx.Max(Avx.Subtract(Vector256<float>.Zero, maxFriction), Avx.Min(newImpulse, maxFriction));
                Vector256<float> impulse = Avx.Subtract(newImpulse, c->tangentImpulse1);
                c->tangentImpulse1 = newImpulse;
                Vector256<float> Px = Avx.Multiply(impulse, tangentX);
                Vector256<float> Py = Avx.Multiply(impulse, tangentY);
                bA.v = new()
                {
                    X = Avx.Subtract(bA.v.X, Avx.Multiply(c->invMassA, Px)),
                    Y = Avx.Subtract(bA.v.Y, Avx.Multiply(c->invMassA, Py))
                };
                bA.w = Avx.Subtract(bA.w, Avx.Multiply(c->invIA, Avx.Subtract(Avx.Multiply(rA.X, Py), Avx.Multiply(rA.Y, Px))));
                bB.v = new()
                {
                    X = Avx.Add(bB.v.X, Avx.Multiply(c->invMassB, Px)),
                    Y = Avx.Add(bB.v.Y, Avx.Multiply(c->invMassB, Py))
                };
                bB.w = Avx.Add(bB.w, Avx.Multiply(c->invIB, Avx.Subtract(Avx.Multiply(rB.X, Py), Avx.Multiply(rB.Y, Px))));
            }
            {
                Vector2W rA = c->anchorA2, rB = c->anchorB2;
                Vector256<float> dvx = Avx.Subtract(Avx.Subtract(bB.v.X, Avx.Multiply(bB.w, rB.Y)), Avx.Subtract(bA.v.X, Avx.Multiply(bA.w, rA.Y)));
                Vector256<float> dvy = Avx.Subtract(Avx.Add(bB.v.Y, Avx.Multiply(bB.w, rB.X)), Avx.Add(bA.v.Y, Avx.Multiply(bA.w, rA.X)));
                Vector256<float> vt = Avx.Add(Avx.Multiply(dvx, tangentX), Avx.Multiply(dvy, tangentY));
                vt = Avx.Subtract(vt, c->tangentSpeed);
                Vector256<float> negImpulse = Avx.Multiply(c->tangentMass2, vt);
                Vector256<float> maxFriction = Avx.Multiply(c->friction, c->normalImpulse2);
                Vector256<float> newImpulse = Avx.Subtract(c->tangentImpulse2, negImpulse);
                newImpulse = Avx.Max(Avx.Subtract(Vector256<float>.Zero, maxFriction), Avx.Min(newImpulse, maxFriction));
                Vector256<float> impulse = Avx.Subtract(newImpulse, c->tangentImpulse2);
                c->tangentImpulse2 = newImpulse;
                Vector256<float> Px = Avx.Multiply(impulse, tangentX);
                Vector256<float> Py = Avx.Multiply(impulse, tangentY);
                bA.v = new()
                {
                    X = Avx.Subtract(bA.v.X, Avx.Multiply(c->invMassA, Px)),
                    Y = Avx.Subtract(bA.v.Y, Avx.Multiply(c->invMassA, Py))
                };
                bA.w = Avx.Subtract(bA.w, Avx.Multiply(c->invIA, Avx.Subtract(Avx.Multiply(rA.X, Py), Avx.Multiply(rA.Y, Px))));
                bB.v = new()
                {
                    X = Avx.Add(bB.v.X, Avx.Multiply(c->invMassB, Px)),
                    Y = Avx.Add(bB.v.Y, Avx.Multiply(c->invMassB, Py))
                };
                bB.w = Avx.Add(bB.w, Avx.Multiply(c->invIB, Avx.Subtract(Avx.Multiply(rB.X, Py), Avx.Multiply(rB.Y, Px))));
            }
            ScatterBodies(states, (int*)&c->indexA, ref bA);
            ScatterBodies(states, (int*)&c->indexB, ref bB);
        }
    }
    public unsafe void StoreImpulses_Wide(ref SolverBlock block, StepContext context, int workerIndex)
    {
        var spans = context.contactPrepareSpans;
        var wideBase = ((ContactConstraintsAVX)context.wideContactConstraints).wideConstraints;
        TaskContext taskContext = context.world.taskContexts[workerIndex];
        BitSet hitEventBitSet = taskContext.hitEventBitSet;
        bool hasHitEvents = taskContext.hasHitEvents;
        float negHitThreshold = -context.world.hitEventThreshold;
        int wideIndex = block.startIndex;
        int endWideIndex = block.startIndex + block.count;
        int colorIndex = 0;
        while (spans[colorIndex + 1].start <= wideIndex) colorIndex++;
        while (wideIndex < endWideIndex)
        {
            int colorWideEndIndex = Math.Min(spans[colorIndex + 1].start, endWideIndex);
            int colorWideStart = spans[colorIndex].start;
            int colorContactCount = spans[colorIndex].count;
            var contactSims = spans[colorIndex].contacts;
            for (; wideIndex < colorWideEndIndex; wideIndex++)
            {
                ContactConstraintWide* c = wideBase + wideIndex;
                float* rollingImpulse = (float*)&c->rollingImpulse;
                float* normalImpulse1 = (float*)&c->normalImpulse1;
                float* normalImpulse2 = (float*)&c->normalImpulse2;
                float* tangentImpulse1 = (float*)&c->tangentImpulse1;
                float* tangentImpulse2 = (float*)&c->tangentImpulse2;
                float* totalNormalImpulse1 = (float*)&c->totalNormalImpulse1;
                float* totalNormalImpulse2 = (float*)&c->totalNormalImpulse2;
                int localWideIndex = wideIndex - colorWideStart;
                int baseIndex = 8 * localWideIndex;
                for (int laneIndex = 0; laneIndex < 8; ++laneIndex)
                {
                    int contactIndex = baseIndex + laneIndex;
                    if (contactIndex >= colorContactCount) break;
                    ContactSim contactSim = contactSims[contactIndex];
                    ref Manifold m = ref contactSim.manifold;
                    m.rollingImpulse = rollingImpulse[laneIndex];
                    m.point0.normalImpulse = normalImpulse1[laneIndex];
                    m.point0.tangentImpulse = tangentImpulse1[laneIndex];
                    m.point0.totalNormalImpulse = totalNormalImpulse1[laneIndex];
                    m.point1.normalImpulse = normalImpulse2[laneIndex];
                    m.point1.tangentImpulse = tangentImpulse2[laneIndex];
                    m.point1.totalNormalImpulse = totalNormalImpulse2[laneIndex];
                    if (contactSim.simFlags.HasFlag(ContactFlags.SimEnableHitEvent))
                    {
                        if (contactSim.manifold.pointCount > 0 && m.point0.normalVelocity < negHitThreshold && m.point0.totalNormalImpulse > 0)
                        {
                            hitEventBitSet.SetBit(contactSim.contactId);
                            hasHitEvents = true;
                            break;
                        }
                        if (contactSim.manifold.pointCount > 1 && m.point1.normalVelocity < negHitThreshold && m.point1.totalNormalImpulse > 0)
                        {
                            hitEventBitSet.SetBit(contactSim.contactId);
                            hasHitEvents = true;
                            break;
                        }
                    }
                }
            }
            colorIndex++;
        }
    }
}
public class ContactSolverNeon : IContactSolverW
{
    public struct Vector2W
    {
        public Vector128<float> X, Y;
        public static Vector2W operator -(Vector2W a, Vector2W b) => new() { X = a.X - b.X, Y = a.Y - b.Y };
    }
    struct RotationW
    {
        public Vector128<float> C, S;
    }
    static Vector128<float> NegW(Vector128<float> a) => AdvSimd.Negate(a);
    static Vector128<float> SymClampW(Vector128<float> a, Vector128<float> b) => AdvSimd.Max(AdvSimd.Negate(b), Sse.Min(a, b));
    static bool AllZeroW(Vector128<float> a) => AdvSimd.Arm64.MinAcross(AdvSimd.CompareEqual(a, Vector128<float>.Zero)).GetElement(0) != 0;
    static Vector2W RightPerpW(Vector2W a) => new() { X = a.Y, Y = -a.X };
    static Vector128<float> DotW(Vector2W a, Vector2W b) => AdvSimd.Add(AdvSimd.Multiply(a.X, b.X), AdvSimd.Multiply(a.Y, b.Y));
    static Vector128<float> CrossW(Vector2W a, Vector2W b) => AdvSimd.Subtract(AdvSimd.Multiply(a.X, b.Y), AdvSimd.Multiply(a.Y, b.X));
    static Vector2W RotateVectorW(RotationW q, Vector2W v) =>
        new() { X = AdvSimd.Subtract(AdvSimd.Multiply(q.C, v.X), AdvSimd.Multiply(q.S, v.Y)), Y = AdvSimd.Add(AdvSimd.Multiply(q.S, v.X), AdvSimd.Multiply(q.C, v.Y)) };
    static Vector128<float> BlendW(Vector128<float> a, Vector128<float> b, Vector128<float> mask) => AdvSimd.BitwiseSelect(mask, b, a);
    static Vector128<float> UnpackLoW(Vector128<float> a, Vector128<float> b) => AdvSimd.Arm64.ZipLow(a, b);
    static Vector128<float> UnpackHiW(Vector128<float> a, Vector128<float> b) => AdvSimd.Arm64.ZipHigh(a, b);
    static Vector128<float> SoftMaskW(Vector128<int> a, Vector128<int> b) => AdvSimd.Or(AdvSimd.CompareEqual(a, Vector128<int>.Zero), AdvSimd.CompareEqual(b, Vector128<int>.Zero)).AsSingle();
    public struct ContactConstraintWide
    {
        public Vector128<int> indexA, indexB;
        public Vector128<float> invMassA, invMassB;
        public Vector128<float> invIA, invIB;
        public Vector2W normal;
        public Vector2W anchorA1, anchorB1;
        public Vector2W anchorA2, anchorB2;
        public Vector128<float> normalMass1, normalMass2;
        public Vector128<float> baseSeparation1, baseSeparation2;
        public Vector128<float> normalImpulse1, normalImpulse2;
        public Vector128<float> totalNormalImpulse1, totalNormalImpulse2;
        public Vector128<float> tangentImpulse1, tangentImpulse2;
        public Vector128<float> rollingImpulse;
        public Vector128<float> friction;
        public Vector128<float> tangentSpeed;
        public Vector128<float> rollingResistance;
        public Vector128<float> tangentMass1, tangentMass2;
        public Vector128<float> negRestitutionVelocity1, negRestitutionVelocity2;
    }
    struct BodyStateW
    {
        public Vector2W v;
        public Vector128<float> w;
        public Vector128<float> flags;
        public Vector2W dp;
        public RotationW dq;
    }
    unsafe BodyStateW GatherBodies(BodyState* states, int* indices)
    {
        Debug.Assert(((nuint)states & 0x1F) == 0);
        Vector128<float> identityA = Vector128.Create(0f, 0, 0, 0), identityB = Vector128.Create(0f, 0, 1, 0);
        int i1 = indices[0] - 1, i2 = indices[1] - 1, i3 = indices[2] - 1, i4 = indices[3] - 1;
        Vector128<float> b1a = i1 == -1 ? identityA : AdvSimd.LoadVector128((float*)(states + i1));
        Vector128<float> b1b = i1 == -1 ? identityB : AdvSimd.LoadVector128((float*)(states + i1));
        Vector128<float> b2a = i2 == -1 ? identityA : AdvSimd.LoadVector128((float*)(states + i2));
        Vector128<float> b2b = i2 == -1 ? identityB : AdvSimd.LoadVector128((float*)(states + i2));
        Vector128<float> b3a = i3 == -1 ? identityA : AdvSimd.LoadVector128((float*)(states + i3));
        Vector128<float> b3b = i3 == -1 ? identityB : AdvSimd.LoadVector128((float*)(states + i3));
        Vector128<float> b4a = i4 == -1 ? identityA : AdvSimd.LoadVector128((float*)(states + i4));
        Vector128<float> b4b = i4 == -1 ? identityB : AdvSimd.LoadVector128((float*)(states + i4));
        Vector128<float> t1a = UnpackLoW(b1a, b3a);
        Vector128<float> t2a = UnpackLoW(b2a, b4a);
        Vector128<float> t3a = UnpackHiW(b1a, b3a);
        Vector128<float> t4a = UnpackHiW(b2a, b4a);
        Vector128<float> t1b = UnpackLoW(b1b, b3b);
        Vector128<float> t2b = UnpackLoW(b2b, b4b);
        Vector128<float> t3b = UnpackHiW(b1b, b3b);
        Vector128<float> t4b = UnpackHiW(b2b, b4b);
        return new()
        {
            v = new() { X = UnpackLoW(t1a, t2a), Y = UnpackHiW(t1a, t2a) },
            w = UnpackLoW(t3a, t4a),
            flags = UnpackHiW(t3a, t4a),
            dp = new() { X = UnpackLoW(t1b, t2b), Y = UnpackHiW(t1b, t2b) },
            dq = new() { C = UnpackLoW(t3b, t4b), S = UnpackHiW(t3b, t4b) },
        };

    }
    unsafe void ScatterBodies(BodyState* states, int* indices, ref BodyStateW simdBody)
    {
        Debug.Assert(((nuint)states & 0x1F) == 0);
        int i1 = indices[0] - 1, i2 = indices[1] - 1, i3 = indices[2] - 1, i4 = indices[3] - 1;
        if (AdvSimd.Arm64.IsSupported)
        {
            // Matches original Box2D 3.0 NEON b2ScatterBodies: transposes v.X/v.Y and w/flags
            // via vtrnq_f32 (TransposeEven/TransposeOdd), then writes only the lower 128 bits
            // of each body state (v.x, v.y, w, flags). dp and dq are not modified by the solver.
            // See https://github.com/erincatto/box2d/blob/241aa82e4c76577a4621402b0fb95f2478a0318f/src/contact_solver.c#L1302
            var te_v = AdvSimd.Arm64.TransposeEven(simdBody.v.X, simdBody.v.Y);
            var to_v = AdvSimd.Arm64.TransposeOdd(simdBody.v.X, simdBody.v.Y);
            var te_wf = AdvSimd.Arm64.TransposeEven(simdBody.w, simdBody.flags);
            var to_wf = AdvSimd.Arm64.TransposeOdd(simdBody.w, simdBody.flags);
            if (i1 != -1 && states[i1].flags.HasFlag(BodyFlags.Dynamic))
                AdvSimd.Store((float*)(states + i1), Vector128.Create(te_v.GetLower(), te_wf.GetLower()));
            if (i2 != -1 && states[i2].flags.HasFlag(BodyFlags.Dynamic))
                AdvSimd.Store((float*)(states + i2), Vector128.Create(to_v.GetLower(), to_wf.GetLower()));
            if (i3 != -1 && states[i3].flags.HasFlag(BodyFlags.Dynamic))
                AdvSimd.Store((float*)(states + i3), Vector128.Create(te_v.GetUpper(), te_wf.GetUpper()));
            if (i4 != -1 && states[i4].flags.HasFlag(BodyFlags.Dynamic))
                AdvSimd.Store((float*)(states + i4), Vector128.Create(to_v.GetUpper(), to_wf.GetUpper()));
        }
        else
        {
            // Fallback (non-ARM64) — matches original C scalar b2ScatterBodies:
            // only writes linearVelocity and angularVelocity, no flags/dp/dq.
            for (int i = 0; i < 4; i++)
            {
                int idx = indices[i] - 1;
                if (idx != -1)
                {
                    BodyState* state = states + idx;
                    state->linearVelocity.x = simdBody.v.X.GetElement(i);
                    state->linearVelocity.y = simdBody.v.Y.GetElement(i);
                    state->angularVelocity = simdBody.w.GetElement(i);
                }
            }
        }
    }
    [System.Runtime.CompilerServices.InlineArray(4)] struct ContactSimLanes { public ContactSim sim; }
    public unsafe void PrepareContacts_Wide(ref SolverBlock block, StepContext context)
    {
        World world = context.world;
        var spans = context.contactPrepareSpans;
        var wideBase = (ContactConstraintsNeon)context.wideContactConstraints;
        Vector128<float> warmStartScale = world.enableWarmStarting ? Vector128<float>.One : Vector128<float>.Zero;
        int wideIndex = block.startIndex, endWideIndex = block.startIndex + block.count;
        int colorIndex = 0;
        while (spans[colorIndex + 1].start <= wideIndex) colorIndex++;
        while (wideIndex < endWideIndex)
        {
            int colorWideStart = spans[colorIndex].start;
            int colorWideEndIndex = Math.Min(spans[colorIndex + 1].start, endWideIndex);
            int colorContactCount = spans[colorIndex].count;
            var contactSims = spans[colorIndex].contacts;
            ContactSimLanes contactLanes = new();
            for (; wideIndex < colorWideEndIndex; wideIndex++)
            {
                var cw = wideBase.wideConstraints + wideIndex;
                int localWideIndex = wideIndex - colorWideStart;
                for (int laneIndex = 0; laneIndex < 4; laneIndex++)
                {
                    int contactIndex = 4 * localWideIndex + laneIndex;
                    if (contactIndex < colorContactCount)
                    {
                        ContactSim c = contactSims[contactIndex];
                        contactLanes[laneIndex] = c;
                        ((int*)&cw->indexA)[laneIndex] = c.bodySimIndexA + 1;
                        ((int*)&cw->indexB)[laneIndex] = c.bodySimIndexB + 1;
#if B2_VALIDATE
                        Body bodyA = world.bodies[c.bodyIdA];
                        int validIndexA = bodyA.setIndex == (int)SetType.Awake ? bodyA.localIndex : -1;
                        Body bodyB = world.bodies[c.bodyIdB];
                        int validIndexB = bodyB.setIndex == (int)SetType.Awake ? bodyB.localIndex : -1;
                        Debug.Assert(c.bodyIdA == validIndexA);
                        Debug.Assert(c.bodyIdB == validIndexB);
#endif
                    }
                    else contactLanes[laneIndex] = ContactSim.Zero;
                }
                cw->invMassA = Vector128.Create(contactLanes[0].invMassA, contactLanes[1].invMassA, contactLanes[2].invMassA, contactLanes[3].invMassA);
                cw->invMassB = Vector128.Create(contactLanes[0].invMassB, contactLanes[1].invMassB, contactLanes[2].invMassB, contactLanes[3].invMassB);
                cw->invIA = Vector128.Create(contactLanes[0].invIA, contactLanes[1].invIA, contactLanes[2].invIA, contactLanes[3].invIA);
                cw->invIB = Vector128.Create(contactLanes[0].invIB, contactLanes[1].invIB, contactLanes[2].invIB, contactLanes[3].invIB);
                cw->normal.X = Vector128.Create(contactLanes[0].manifold.normal.x, contactLanes[1].manifold.normal.x, contactLanes[2].manifold.normal.x, contactLanes[3].manifold.normal.x);
                cw->normal.Y = Vector128.Create(contactLanes[0].manifold.normal.y, contactLanes[1].manifold.normal.y, contactLanes[2].manifold.normal.y, contactLanes[3].manifold.normal.y);
                cw->friction = Vector128.Create(contactLanes[0].friction, contactLanes[1].friction, contactLanes[2].friction, contactLanes[3].friction);
                cw->tangentSpeed = Vector128.Create(contactLanes[0].tangentSpeed, contactLanes[1].tangentSpeed, contactLanes[2].tangentSpeed, contactLanes[3].tangentSpeed);
                cw->rollingResistance = Vector128.Create(contactLanes[0].rollingResistance, contactLanes[1].rollingResistance, contactLanes[2].rollingResistance, contactLanes[3].rollingResistance);
                cw->rollingImpulse = Vector128.Create(contactLanes[0].manifold.rollingImpulse, contactLanes[1].manifold.rollingImpulse, contactLanes[2].manifold.rollingImpulse, contactLanes[3].manifold.rollingImpulse);
                cw->rollingImpulse = AdvSimd.Multiply(warmStartScale, cw->rollingImpulse);
                Vector2W tangent = RightPerpW(cw->normal);
                {
                    Vector128<float> m1a, m1b; fixed (Manifold* m = &contactLanes[0].manifold) { m1a = AdvSimd.LoadVector128(&m->point0.anchorA.x); m1b = AdvSimd.LoadVector128(&m->point0.anchorB.x); }
                    Vector128<float> m2a, m2b; fixed (Manifold* m = &contactLanes[1].manifold) { m2a = AdvSimd.LoadVector128(&m->point0.anchorA.x); m2b = AdvSimd.LoadVector128(&m->point0.anchorB.x); }
                    Vector128<float> m3a, m3b; fixed (Manifold* m = &contactLanes[2].manifold) { m3a = AdvSimd.LoadVector128(&m->point0.anchorA.x); m3b = AdvSimd.LoadVector128(&m->point0.anchorB.x); }
                    Vector128<float> m4a, m4b; fixed (Manifold* m = &contactLanes[3].manifold) { m4a = AdvSimd.LoadVector128(&m->point0.anchorA.x); m4b = AdvSimd.LoadVector128(&m->point0.anchorB.x); }
                    Vector128<float> t1a = UnpackLoW(m1a, m3a), t2a = UnpackLoW(m2a, m4a);
                    Vector128<float> t3a = UnpackHiW(m1a, m3a), t4a = UnpackHiW(m2a, m4a);
                    Vector128<float> t1b = UnpackLoW(m1b, m3b), t2b = UnpackLoW(m2b, m4b);
                    Vector128<float> t3b = UnpackHiW(m1b, m3b), t4b = UnpackHiW(m2b, m4b);
                    cw->anchorA2.X = UnpackLoW(t1a, t2a);
                    cw->anchorA2.Y = UnpackHiW(t1a, t2a);
                    cw->anchorB2.X = UnpackLoW(t3a, t4a);
                    cw->anchorB2.Y = UnpackHiW(t3a, t4a);
                    cw->baseSeparation2 = UnpackLoW(t1b, t2b);
                    cw->normalImpulse2 = UnpackHiW(t1b, t2b);
                    cw->tangentImpulse2 = UnpackLoW(t3b, t4b);
                    cw->negRestitutionVelocity2 = UnpackHiW(t3b, t4b);

                    Vector128<float> offset = DotW(cw->anchorB1 - cw->anchorA1, cw->normal);
                    cw->baseSeparation1 = cw->baseSeparation1 - offset;
                    cw->negRestitutionVelocity1 = -cw->negRestitutionVelocity1;
                    cw->normalImpulse1 = warmStartScale * cw->normalImpulse1;
                    cw->tangentImpulse1 = warmStartScale * cw->tangentImpulse1;
                    cw->totalNormalImpulse1 = Vector128<float>.Zero;
                    {
                        Vector128<float> rnA = CrossW(cw->anchorA1, cw->normal);
                        Vector128<float> rnB = CrossW(cw->anchorB1, cw->normal);
                        Vector128<float> k = cw->invMassA + cw->invMassB + cw->invIA * rnA * rnA + cw->invIB * rnB * rnB;
                        cw->normalMass1 = BlendW(Vector128<float>.Zero, Vector128<float>.One / k, AdvSimd.CompareGreaterThan(k, Vector128<float>.Zero));
                    }
                    {
                        Vector128<float> rnA = CrossW(cw->anchorA1, tangent);
                        Vector128<float> rnB = CrossW(cw->anchorB1, tangent);
                        Vector128<float> k = cw->invMassA + cw->invMassB + cw->invIA * rnA * rnA + cw->invIB * rnB * rnB;
                        cw->tangentMass1 = BlendW(Vector128<float>.Zero, Vector128<float>.One / k, AdvSimd.CompareGreaterThan(k, Vector128<float>.Zero));
                    }
                }
                {
                    Vector128<float> m1a, m1b; fixed (Manifold* m = &contactLanes[0].manifold) { m1a = AdvSimd.LoadVector128(&m->point1.anchorA.x); m1b = AdvSimd.LoadVector128(&m->point1.anchorB.x); }
                    Vector128<float> m2a, m2b; fixed (Manifold* m = &contactLanes[1].manifold) { m2a = AdvSimd.LoadVector128(&m->point1.anchorA.x); m2b = AdvSimd.LoadVector128(&m->point1.anchorB.x); }
                    Vector128<float> m3a, m3b; fixed (Manifold* m = &contactLanes[2].manifold) { m3a = AdvSimd.LoadVector128(&m->point1.anchorA.x); m3b = AdvSimd.LoadVector128(&m->point1.anchorB.x); }
                    Vector128<float> m4a, m4b; fixed (Manifold* m = &contactLanes[3].manifold) { m4a = AdvSimd.LoadVector128(&m->point1.anchorA.x); m4b = AdvSimd.LoadVector128(&m->point1.anchorB.x); }
                    Vector128<float> t1a = UnpackLoW(m1a, m3a), t2a = UnpackLoW(m2a, m4a);
                    Vector128<float> t3a = UnpackHiW(m1a, m3a), t4a = UnpackHiW(m2a, m4a);
                    Vector128<float> t1b = UnpackLoW(m1b, m3b), t2b = UnpackLoW(m2b, m4b);
                    Vector128<float> t3b = UnpackHiW(m1b, m3b), t4b = UnpackHiW(m2b, m4b);
                    cw->anchorA2.X = UnpackLoW(t1a, t2a);
                    cw->anchorA2.Y = UnpackHiW(t1a, t2a);
                    cw->anchorB2.X = UnpackLoW(t3a, t4a);
                    cw->anchorB2.Y = UnpackHiW(t3a, t4a);
                    cw->baseSeparation2 = UnpackLoW(t1b, t2b);
                    cw->normalImpulse2 = UnpackHiW(t1b, t2b);
                    cw->tangentImpulse2 = UnpackLoW(t3b, t4b);
                    cw->negRestitutionVelocity2 = UnpackHiW(t3b, t4b);

                    Vector128<float> offset = DotW(cw->anchorB2 - cw->anchorA2, cw->normal);
                    cw->baseSeparation2 = cw->baseSeparation2 - offset;
                    cw->negRestitutionVelocity2 = -cw->negRestitutionVelocity2;
                    cw->normalImpulse2 = warmStartScale * cw->normalImpulse2;
                    cw->tangentImpulse2 = warmStartScale * cw->tangentImpulse2;
                    cw->totalNormalImpulse2 = Vector128<float>.Zero;
                    {
                        Vector128<float> rnA = CrossW(cw->anchorA2, cw->normal);
                        Vector128<float> rnB = CrossW(cw->anchorB2, cw->normal);
                        Vector128<float> k = cw->invMassA + cw->invMassB + cw->invIA * rnA * rnA + cw->invIB * rnB * rnB;
                        cw->normalMass2 = BlendW(Vector128<float>.Zero, Vector128<float>.One / k, AdvSimd.CompareGreaterThan(k, Vector128<float>.Zero));
                    }
                    {
                        Vector128<float> rnA = CrossW(cw->anchorA2, tangent);
                        Vector128<float> rnB = CrossW(cw->anchorB2, tangent);
                        Vector128<float> k = cw->invMassA + cw->invMassB + cw->invIA * rnA * rnA + cw->invIB * rnB * rnB;
                        cw->tangentMass2 = BlendW(Vector128<float>.Zero, Vector128<float>.One / k, AdvSimd.CompareGreaterThan(k, Vector128<float>.Zero));
                    }
                }
                Vector128<float> massScale = Vector128.GreaterThan(Vector128.Create(contactLanes[0].manifold.pointCount, contactLanes[1].manifold.pointCount, contactLanes[2].manifold.pointCount, contactLanes[3].manifold.pointCount), Vector128<int>.One).AsSingle();
                cw->normalMass2 = BlendW(Vector128<float>.Zero, cw->normalMass2, massScale);
                cw->tangentMass2 = BlendW(Vector128<float>.Zero, cw->tangentMass2, massScale);
            }
            colorIndex++;
        }
    }
    public unsafe void WarmStartContacts_Wide(ref SolverBlock block, StepContext context)
    {
        var states = context.states.Data;
        var constraints = ((ContactConstraintsNeon)context.graph.colors[block.colorIndex].wideConstraints).wideConstraints;
        {
            for (int wideIndex = block.startIndex; wideIndex < block.startIndex + block.count; wideIndex++)
            {
                ContactConstraintWide* c = constraints + wideIndex;
                BodyStateW bA = GatherBodies(states, (int*)&c->indexA);
                BodyStateW bB = GatherBodies(states, (int*)&c->indexB);
                Vector128<float> tangentX = c->normal.Y;
                Vector128<float> tangentY = AdvSimd.Subtract(Vector128<float>.Zero, c->normal.X);
                {
                    Vector2W rA = c->anchorA1, rB = c->anchorB1;
                    Vector2W P = new()
                    {
                        X = AdvSimd.Add(AdvSimd.Multiply(c->normalImpulse1, c->normal.X), AdvSimd.Multiply(c->tangentImpulse1, tangentX)),
                        Y = AdvSimd.Add(AdvSimd.Multiply(c->normalImpulse1, c->normal.Y), AdvSimd.Multiply(c->tangentImpulse1, tangentY))
                    };
                    bA.w = AdvSimd.Subtract(bA.w, AdvSimd.Multiply(c->invIA, CrossW(rA, P)));
                    bA.v = new()
                    {
                        X = AdvSimd.Subtract(bA.v.X, AdvSimd.Multiply(c->invMassA, P.X)),
                        Y = AdvSimd.Subtract(bA.v.Y, AdvSimd.Multiply(c->invMassA, P.Y))
                    };
                    bB.w = AdvSimd.Add(bB.w, AdvSimd.Multiply(c->invIB, CrossW(rB, P)));
                    bB.v = new()
                    {
                        X = AdvSimd.Add(bB.v.X, AdvSimd.Multiply(c->invMassB, P.X)),
                        Y = AdvSimd.Add(bB.v.Y, AdvSimd.Multiply(c->invMassB, P.Y))
                    };
                    c->totalNormalImpulse1 = AdvSimd.Add(c->totalNormalImpulse1, c->normalImpulse1);
                }
                {
                    Vector2W rA = c->anchorA2, rB = c->anchorB2;
                    Vector2W P = new()
                    {
                        X = AdvSimd.Add(AdvSimd.Multiply(c->normalImpulse2, c->normal.X), AdvSimd.Multiply(c->tangentImpulse2, tangentX)),
                        Y = AdvSimd.Add(AdvSimd.Multiply(c->normalImpulse2, c->normal.Y), AdvSimd.Multiply(c->tangentImpulse2, tangentY))
                    };
                    bA.w = AdvSimd.Subtract(bA.w, AdvSimd.Multiply(c->invIA, CrossW(rA, P)));
                    bA.v = new()
                    {
                        X = AdvSimd.Subtract(bA.v.X, AdvSimd.Multiply(c->invMassA, P.X)),
                        Y = AdvSimd.Subtract(bA.v.Y, AdvSimd.Multiply(c->invMassA, P.Y))
                    };
                    bB.w = AdvSimd.Add(bB.w, AdvSimd.Multiply(c->invIB, CrossW(rB, P)));
                    bB.v = new()
                    {
                        X = AdvSimd.Add(bB.v.X, AdvSimd.Multiply(c->invMassB, P.X)),
                        Y = AdvSimd.Add(bB.v.Y, AdvSimd.Multiply(c->invMassB, P.Y))
                    };
                    c->totalNormalImpulse2 = AdvSimd.Add(c->totalNormalImpulse2, c->normalImpulse2);
                }
                bA.w = AdvSimd.Subtract(bA.w, AdvSimd.Multiply(c->invIA, c->rollingImpulse));
                bB.w = AdvSimd.Add(bB.w, AdvSimd.Multiply(c->invIB, c->rollingImpulse));
                ScatterBodies(states, (int*)&c->indexA, ref bA);
                ScatterBodies(states, (int*)&c->indexB, ref bB);
            }
        }
    }
    public unsafe void PushContacts_Wide(ref SolverBlock block, StepContext context)
    {
        var states = context.states.Data;
        var constraints = ((ContactConstraintsNeon)context.graph.colors[block.colorIndex].wideConstraints).wideConstraints;
        {
            Vector128<float> inv_h = Vector128.Create(context.inv_h);
            Vector128<float> contactSpeed = Vector128.Create(-context.world.contactSpeed);
            Vector128<float> oneW = Vector128<float>.One;
            Vector128<float> dynamicBiasRate = Vector128.Create(context.contactSoftness.massScale * context.contactSoftness.biasRate);
            Vector128<float> dynamicMassScale = Vector128.Create(context.contactSoftness.massScale);
            Vector128<float> dynamicImpulseScale = Vector128.Create(context.contactSoftness.impulseScale);
            Vector128<float> staticBiasRate = Vector128.Create(context.staticSoftness.massScale * context.staticSoftness.biasRate);
            Vector128<float> staticMassScale = Vector128.Create(context.staticSoftness.massScale);
            Vector128<float> staticImpulseScale = Vector128.Create(context.staticSoftness.impulseScale);
            for (int wideIndex = block.startIndex; wideIndex < block.startIndex + block.count; wideIndex++)
            {
                ContactConstraintWide* c = constraints + wideIndex;
                BodyStateW bA = GatherBodies(states, (int*)&c->indexA);
                BodyStateW bB = GatherBodies(states, (int*)&c->indexB);
                Vector128<float> softMask = SoftMaskW(c->indexA, c->indexB),
                    biasRate = BlendW(dynamicBiasRate, staticBiasRate, softMask),
                    massScale = BlendW(dynamicMassScale, staticMassScale, softMask),
                    impulseScale = BlendW(dynamicImpulseScale, staticImpulseScale, softMask);
                Vector2W dp = new() { X = AdvSimd.Subtract(bB.dp.X, bA.dp.X), Y = AdvSimd.Subtract(bB.dp.Y, bA.dp.Y) };
                {
                    Vector2W rA = c->anchorA1, rB = c->anchorB1;
                    Vector2W rsA = RotateVectorW(bA.dq, rA), rsB = RotateVectorW(bB.dq, rB);
                    Vector2W ds = new() { X = AdvSimd.Add(dp.X, AdvSimd.Subtract(rsB.X, rsA.X)), Y = AdvSimd.Add(dp.Y, AdvSimd.Subtract(rsB.Y, rsA.Y)) };
                    Vector128<float> s = AdvSimd.Add(DotW(c->normal, ds), c->baseSeparation1);
                    Vector128<float> separated = AdvSimd.CompareGreaterThan(s, Vector128<float>.Zero);
                    Vector128<float> velocityBias = AdvSimd.Multiply(s, inv_h), overlapBias = AdvSimd.Max(AdvSimd.Multiply(biasRate, s), contactSpeed);
                    Vector128<float> bias = BlendW(overlapBias, velocityBias, separated);
                    Vector128<float> pointMassScale = BlendW(massScale, oneW, separated);
                    Vector128<float> pointImpulseScale = BlendW(impulseScale, Vector128<float>.Zero, separated);
                    Vector128<float> dvx = AdvSimd.Subtract(AdvSimd.Subtract(bB.v.X, AdvSimd.Multiply(bB.w, rB.Y)), AdvSimd.Subtract(bA.v.X, AdvSimd.Multiply(bA.w, rA.Y)));
                    Vector128<float> dvy = AdvSimd.Subtract(AdvSimd.Add(bB.v.Y, AdvSimd.Multiply(bB.w, rB.X)), AdvSimd.Add(bA.v.Y, AdvSimd.Multiply(bA.w, rA.X)));
                    Vector128<float> vn = AdvSimd.Add(AdvSimd.Multiply(dvx, c->normal.X), AdvSimd.Multiply(dvy, c->normal.Y));
                    Vector128<float> negImpulse = AdvSimd.Add(AdvSimd.Multiply(c->normalMass1, AdvSimd.Add(AdvSimd.Multiply(pointMassScale, vn), bias)), AdvSimd.Multiply(pointImpulseScale, c->normalImpulse1));
                    Vector128<float> newImpulse = AdvSimd.Max(AdvSimd.Subtract(c->normalImpulse1, negImpulse), Vector128<float>.Zero);
                    Vector128<float> impulse = AdvSimd.Subtract(newImpulse, c->normalImpulse1);
                    c->normalImpulse1 = newImpulse;
                    c->totalNormalImpulse1 = AdvSimd.Add(c->totalNormalImpulse1, impulse);
                    Vector128<float> Px = AdvSimd.Multiply(impulse, c->normal.X);
                    Vector128<float> Py = AdvSimd.Multiply(impulse, c->normal.Y);
                    bA.v = new()
                    {
                        X = AdvSimd.Subtract(bA.v.X, AdvSimd.Multiply(c->invMassA, Px)),
                        Y = AdvSimd.Subtract(bA.v.Y, AdvSimd.Multiply(c->invMassA, Py))
                    };
                    bA.w = AdvSimd.Subtract(bA.w, AdvSimd.Multiply(c->invIA, AdvSimd.Subtract(AdvSimd.Multiply(rA.X, Py), AdvSimd.Multiply(rA.Y, Px))));
                    bB.v = new()
                    {
                        X = AdvSimd.Add(bB.v.X, AdvSimd.Multiply(c->invMassB, Px)),
                        Y = AdvSimd.Add(bB.v.Y, AdvSimd.Multiply(c->invMassB, Py))
                    };
                    bB.w = AdvSimd.Add(bB.w, AdvSimd.Multiply(c->invIB, AdvSimd.Subtract(AdvSimd.Multiply(rB.X, Py), AdvSimd.Multiply(rB.Y, Px))));
                }
                {
                    Vector2W rA = c->anchorA2, rB = c->anchorB2;
                    Vector2W rsA = RotateVectorW(bA.dq, rA), rsB = RotateVectorW(bB.dq, rB);
                    Vector2W ds = new() { X = AdvSimd.Add(dp.X, AdvSimd.Subtract(rsB.X, rsA.X)), Y = AdvSimd.Add(dp.Y, AdvSimd.Subtract(rsB.Y, rsA.Y)) };
                    Vector128<float> s = AdvSimd.Add(DotW(c->normal, ds), c->baseSeparation2);
                    Vector128<float> separated = AdvSimd.CompareGreaterThan(s, Vector128<float>.Zero);
                    Vector128<float> specBias = AdvSimd.Multiply(s, inv_h), overlapBias = AdvSimd.Max(AdvSimd.Multiply(biasRate, s), contactSpeed);
                    Vector128<float> velocityBias = BlendW(overlapBias, specBias, separated);
                    Vector128<float> pointMassScale = BlendW(massScale, oneW, separated);
                    Vector128<float> pointImpulseScale = BlendW(impulseScale, Vector128<float>.Zero, separated);
                    Vector128<float> dvx = AdvSimd.Subtract(AdvSimd.Subtract(bB.v.X, AdvSimd.Multiply(bB.w, rB.Y)), AdvSimd.Subtract(bA.v.X, AdvSimd.Multiply(bA.w, rA.Y)));
                    Vector128<float> dvy = AdvSimd.Subtract(AdvSimd.Add(bB.v.Y, AdvSimd.Multiply(bB.w, rB.X)), AdvSimd.Add(bA.v.Y, AdvSimd.Multiply(bA.w, rA.X)));
                    Vector128<float> vn = AdvSimd.Add(AdvSimd.Multiply(dvx, c->normal.X), AdvSimd.Multiply(dvy, c->normal.Y));
                    Vector128<float> negImpulse = AdvSimd.Add(AdvSimd.Multiply(c->normalMass2, AdvSimd.Add(AdvSimd.Multiply(pointMassScale, vn), velocityBias)), AdvSimd.Multiply(pointImpulseScale, c->normalImpulse2));
                    Vector128<float> newImpulse = AdvSimd.Max(AdvSimd.Subtract(c->normalImpulse2, negImpulse), Vector128<float>.Zero);
                    Vector128<float> impulse = AdvSimd.Subtract(newImpulse, c->normalImpulse2);
                    c->normalImpulse2 = newImpulse;
                    c->totalNormalImpulse2 = AdvSimd.Add(c->totalNormalImpulse2, impulse);
                    Vector128<float> Px = AdvSimd.Multiply(impulse, c->normal.X);
                    Vector128<float> Py = AdvSimd.Multiply(impulse, c->normal.Y);
                    bA.v = new()
                    {
                        X = AdvSimd.Subtract(bA.v.X, AdvSimd.Multiply(c->invMassA, Px)),
                        Y = AdvSimd.Subtract(bA.v.Y, AdvSimd.Multiply(c->invMassA, Py))
                    };
                    bA.w = AdvSimd.Subtract(bA.w, AdvSimd.Multiply(c->invIA, AdvSimd.Subtract(AdvSimd.Multiply(rA.X, Py), AdvSimd.Multiply(rA.Y, Px))));
                    bB.v = new()
                    {
                        X = AdvSimd.Add(bB.v.X, AdvSimd.Multiply(c->invMassB, Px)),
                        Y = AdvSimd.Add(bB.v.Y, AdvSimd.Multiply(c->invMassB, Py))
                    };
                    bB.w = AdvSimd.Add(bB.w, AdvSimd.Multiply(c->invIB, AdvSimd.Subtract(AdvSimd.Multiply(rB.X, Py), AdvSimd.Multiply(rB.Y, Px))));
                }
                ScatterBodies(states, (int*)&c->indexA, ref bA);
                ScatterBodies(states, (int*)&c->indexB, ref bB);
            }
        }
    }
    public unsafe void SolveContacts_Wide(ref SolverBlock block, StepContext context)
    {
        var states = context.states.Data;
        GraphColor color = context.graph.colors[block.colorIndex];
        var constraints = ((ContactConstraintsNeon)context.graph.colors[block.colorIndex].wideConstraints).wideConstraints;
        Vector128<float> inv_h = Vector128.Create(context.inv_h);
        for (int wideIndex = 0; wideIndex < block.startIndex + block.count; wideIndex++)
        {
            ContactConstraintWide* c = constraints + wideIndex;
            BodyStateW bA = GatherBodies(states, (int*)&c->indexA);
            BodyStateW bB = GatherBodies(states, (int*)&c->indexB);
            Vector128<float> resitutionMask1 = AdvSimd.CompareGreaterThan(Vector128<float>.Zero, c->negRestitutionVelocity1);
            Vector128<float> resitutionMask2 = AdvSimd.CompareGreaterThan(Vector128<float>.Zero, c->negRestitutionVelocity2);
            bool haveResitution = !AllZeroW(AdvSimd.Or(resitutionMask1, resitutionMask2));
            Vector128<float> keepRestitution = Vector128<float>.Zero;
            Vector128<float> totalNormalImpulse = Vector128<float>.Zero;
            Vector2W dp = bB.dp - bA.dp;
            {
                Vector2W rA = c->anchorA1, rB = c->anchorB1;
                Vector2W rsA = RotateVectorW(bA.dq, rA), rsB = RotateVectorW(bB.dq, rB);
                Vector2W ds = new() { X = dp.X + (rsB.X - rsA.X), Y = dp.Y + (rsB.Y - rsA.Y) };
                Vector128<float> s = DotW(c->normal, ds) + c->baseSeparation1;
                Vector128<float> specBias = s * inv_h;
                Vector128<float> velocityBias = AdvSimd.Max(Vector128<float>.Zero, specBias);
                if (haveResitution)
                {
                    Vector128<float> separated = AdvSimd.CompareGreaterThan(s, Vector128<float>.Zero);
                    velocityBias = BlendW(velocityBias, c->negRestitutionVelocity1, resitutionMask1);
                    keepRestitution = AdvSimd.Or(keepRestitution, AdvSimd.BitwiseClear(resitutionMask1, separated));
                }
                Vector128<float> dvx = (bB.v.X - bB.w * rB.Y) - (bA.v.X - bA.w * rA.Y);
                Vector128<float> dvy = (bB.v.Y + bB.w * rB.X) - (bA.v.Y + bA.w * rA.X);
                Vector128<float> vn = dvx * c->normal.X + dvy * c->normal.Y;
                Vector128<float> newImpulse = AdvSimd.Max(c->normalImpulse1 - c->normalMass1 * (vn + velocityBias), Vector128<float>.Zero);
                Vector128<float> impulse = newImpulse - c->normalImpulse1;
                c->normalImpulse1 = newImpulse;
                c->totalNormalImpulse1 = c->totalNormalImpulse1 + impulse;
                totalNormalImpulse = totalNormalImpulse + newImpulse;
                Vector128<float> Px = impulse * c->normal.X, Py = impulse * c->normal.Y;
                bA.v = new()
                {
                    X = AdvSimd.Subtract(bA.v.X, AdvSimd.Multiply(c->invMassA, Px)),
                    Y = AdvSimd.Subtract(bA.v.Y, AdvSimd.Multiply(c->invMassA, Py))
                };
                bA.w = AdvSimd.Subtract(bA.w, AdvSimd.Multiply(c->invIA, AdvSimd.Subtract(AdvSimd.Multiply(rA.X, Py), AdvSimd.Multiply(rA.Y, Px))));
                bB.v = new()
                {
                    X = AdvSimd.Add(bB.v.X, AdvSimd.Multiply(c->invMassB, Px)),
                    Y = AdvSimd.Add(bB.v.Y, AdvSimd.Multiply(c->invMassB, Py))
                };
                bB.w = AdvSimd.Add(bB.w, AdvSimd.Multiply(c->invIB, AdvSimd.Subtract(AdvSimd.Multiply(rB.X, Py), AdvSimd.Multiply(rB.Y, Px))));
            }
            {
                Vector2W rA = c->anchorA2, rB = c->anchorB2;
                Vector2W rsA = RotateVectorW(bA.dq, rA), rsB = RotateVectorW(bB.dq, rB);
                Vector2W ds = new() { X = dp.X + (rsB.X - rsA.X), Y = dp.Y + (rsB.Y - rsA.Y) };
                Vector128<float> s = DotW(c->normal, ds) + c->baseSeparation2;
                Vector128<float> specBias = s * inv_h;
                Vector128<float> velocityBias = AdvSimd.Max(Vector128<float>.Zero, specBias);
                if (haveResitution)
                {
                    Vector128<float> separated = AdvSimd.CompareGreaterThan(s, Vector128<float>.Zero);
                    velocityBias = BlendW(velocityBias, c->negRestitutionVelocity2, resitutionMask2);
                    keepRestitution = AdvSimd.Or(keepRestitution, AdvSimd.BitwiseClear(resitutionMask2, separated));
                }
                Vector128<float> dvx = (bB.v.X - bB.w * rB.Y) - (bA.v.X - bA.w * rA.Y);
                Vector128<float> dvy = (bB.v.Y + bB.w * rB.X) - (bA.v.Y + bA.w * rA.X);
                Vector128<float> vn = dvx * c->normal.X + dvy * c->normal.Y;
                Vector128<float> newImpulse = AdvSimd.Max(c->normalImpulse2 - c->normalMass2 * (vn + velocityBias), Vector128<float>.Zero);
                Vector128<float> impulse = newImpulse - c->normalImpulse2;
                c->normalImpulse2 = newImpulse;
                c->totalNormalImpulse2 = c->totalNormalImpulse2 + impulse;
                totalNormalImpulse = totalNormalImpulse + newImpulse;
                Vector128<float> Px = impulse * c->normal.X, Py = impulse * c->normal.Y;
                bA.v = new()
                {
                    X = AdvSimd.Subtract(bA.v.X, AdvSimd.Multiply(c->invMassA, Px)),
                    Y = AdvSimd.Subtract(bA.v.Y, AdvSimd.Multiply(c->invMassA, Py))
                };
                bA.w = AdvSimd.Subtract(bA.w, AdvSimd.Multiply(c->invIA, AdvSimd.Subtract(AdvSimd.Multiply(rA.X, Py), AdvSimd.Multiply(rA.Y, Px))));
                bB.v = new()
                {
                    X = AdvSimd.Add(bB.v.X, AdvSimd.Multiply(c->invMassB, Px)),
                    Y = AdvSimd.Add(bB.v.Y, AdvSimd.Multiply(c->invMassB, Py))
                };
                bB.w = AdvSimd.Add(bB.w, AdvSimd.Multiply(c->invIB, AdvSimd.Subtract(AdvSimd.Multiply(rB.X, Py), AdvSimd.Multiply(rB.Y, Px))));
            }
            c->negRestitutionVelocity1 = BlendW(Vector128<float>.Zero, c->negRestitutionVelocity1, keepRestitution);
            c->negRestitutionVelocity2 = BlendW(Vector128<float>.Zero, c->negRestitutionVelocity2, keepRestitution);
            if (!AllZeroW(c->rollingResistance))
            {
                Vector128<float> k = c->invIA + c->invIB;
                Vector128<float> deltaLambda = BlendW(Vector128<float>.Zero, AdvSimd.Arm64.Divide(AdvSimd.Subtract(bA.w, bB.w), k), AdvSimd.CompareGreaterThan(k, Vector128<float>.Zero));
                Vector128<float> lambda = c->rollingImpulse;
                Vector128<float> maxLambda = AdvSimd.Multiply(c->rollingResistance, totalNormalImpulse);
                c->rollingImpulse = SymClampW(AdvSimd.Add(lambda, deltaLambda), maxLambda);
                deltaLambda = AdvSimd.Subtract(c->rollingImpulse, lambda);
                bA.w = AdvSimd.Subtract(bA.w, AdvSimd.Multiply(c->invIA, deltaLambda));
                bB.w = AdvSimd.Add(bB.w, AdvSimd.Multiply(c->invIB, deltaLambda));
            }
            Vector128<float> tangentX = c->normal.Y;
            Vector128<float> tangentY = AdvSimd.Subtract(Vector128<float>.Zero, c->normal.X);
            {
                Vector2W rA = c->anchorA1, rB = c->anchorB1;
                Vector128<float> dvx = AdvSimd.Subtract(AdvSimd.Subtract(bB.v.X, AdvSimd.Multiply(bB.w, rB.Y)), AdvSimd.Subtract(bA.v.X, AdvSimd.Multiply(bA.w, rA.Y)));
                Vector128<float> dvy = AdvSimd.Subtract(AdvSimd.Add(bB.v.Y, AdvSimd.Multiply(bB.w, rB.X)), AdvSimd.Add(bA.v.Y, AdvSimd.Multiply(bA.w, rA.X)));
                Vector128<float> vt = AdvSimd.Add(AdvSimd.Multiply(dvx, tangentX), AdvSimd.Multiply(dvy, tangentY));
                vt = AdvSimd.Subtract(vt, c->tangentSpeed);
                Vector128<float> negImpulse = AdvSimd.Multiply(c->tangentMass1, vt);
                Vector128<float> maxFriction = AdvSimd.Multiply(c->friction, c->normalImpulse1);
                Vector128<float> newImpulse = AdvSimd.Subtract(c->tangentImpulse1, negImpulse);
                //no symclamp?
                newImpulse = AdvSimd.Max(AdvSimd.Subtract(Vector128<float>.Zero, maxFriction), AdvSimd.Min(newImpulse, maxFriction));
                Vector128<float> impulse = AdvSimd.Subtract(newImpulse, c->tangentImpulse1);
                c->tangentImpulse1 = newImpulse;
                Vector128<float> Px = AdvSimd.Multiply(impulse, tangentX);
                Vector128<float> Py = AdvSimd.Multiply(impulse, tangentY);
                bA.v = new()
                {
                    X = AdvSimd.Subtract(bA.v.X, AdvSimd.Multiply(c->invMassA, Px)),
                    Y = AdvSimd.Subtract(bA.v.Y, AdvSimd.Multiply(c->invMassA, Py))
                };
                bA.w = AdvSimd.Subtract(bA.w, AdvSimd.Multiply(c->invIA, AdvSimd.Subtract(AdvSimd.Multiply(rA.X, Py), AdvSimd.Multiply(rA.Y, Px))));
                bB.v = new()
                {
                    X = AdvSimd.Add(bB.v.X, AdvSimd.Multiply(c->invMassB, Px)),
                    Y = AdvSimd.Add(bB.v.Y, AdvSimd.Multiply(c->invMassB, Py))
                };
                bB.w = AdvSimd.Add(bB.w, AdvSimd.Multiply(c->invIB, AdvSimd.Subtract(AdvSimd.Multiply(rB.X, Py), AdvSimd.Multiply(rB.Y, Px))));
            }
            {
                Vector2W rA = c->anchorA2, rB = c->anchorB2;
                Vector128<float> dvx = AdvSimd.Subtract(AdvSimd.Subtract(bB.v.X, AdvSimd.Multiply(bB.w, rB.Y)), AdvSimd.Subtract(bA.v.X, AdvSimd.Multiply(bA.w, rA.Y)));
                Vector128<float> dvy = AdvSimd.Subtract(AdvSimd.Add(bB.v.Y, AdvSimd.Multiply(bB.w, rB.X)), AdvSimd.Add(bA.v.Y, AdvSimd.Multiply(bA.w, rA.X)));
                Vector128<float> vt = AdvSimd.Add(AdvSimd.Multiply(dvx, tangentX), AdvSimd.Multiply(dvy, tangentY));
                vt = AdvSimd.Subtract(vt, c->tangentSpeed);
                Vector128<float> negImpulse = AdvSimd.Multiply(c->tangentMass2, vt);
                Vector128<float> maxFriction = AdvSimd.Multiply(c->friction, c->normalImpulse2);
                Vector128<float> newImpulse = AdvSimd.Subtract(c->tangentImpulse2, negImpulse);
                newImpulse = AdvSimd.Max(AdvSimd.Subtract(Vector128<float>.Zero, maxFriction), AdvSimd.Min(newImpulse, maxFriction));
                Vector128<float> impulse = AdvSimd.Subtract(newImpulse, c->tangentImpulse2);
                c->tangentImpulse2 = newImpulse;
                Vector128<float> Px = AdvSimd.Multiply(impulse, tangentX);
                Vector128<float> Py = AdvSimd.Multiply(impulse, tangentY);
                bA.v = new()
                {
                    X = AdvSimd.Subtract(bA.v.X, AdvSimd.Multiply(c->invMassA, Px)),
                    Y = AdvSimd.Subtract(bA.v.Y, AdvSimd.Multiply(c->invMassA, Py))
                };
                bA.w = AdvSimd.Subtract(bA.w, AdvSimd.Multiply(c->invIA, AdvSimd.Subtract(AdvSimd.Multiply(rA.X, Py), AdvSimd.Multiply(rA.Y, Px))));
                bB.v = new()
                {
                    X = AdvSimd.Add(bB.v.X, AdvSimd.Multiply(c->invMassB, Px)),
                    Y = AdvSimd.Add(bB.v.Y, AdvSimd.Multiply(c->invMassB, Py))
                };
                bB.w = AdvSimd.Add(bB.w, AdvSimd.Multiply(c->invIB, AdvSimd.Subtract(AdvSimd.Multiply(rB.X, Py), AdvSimd.Multiply(rB.Y, Px))));
            }
            ScatterBodies(states, (int*)&c->indexA, ref bA);
            ScatterBodies(states, (int*)&c->indexB, ref bB);
        }
    }
    public unsafe void StoreImpulses_Wide(ref SolverBlock block, StepContext context, int workerIndex)
    {
        var spans = context.contactPrepareSpans;
        var wideBase = ((ContactConstraintsNeon)context.wideContactConstraints).wideConstraints;
        TaskContext taskContext = context.world.taskContexts[workerIndex];
        BitSet hitEventBitSet = taskContext.hitEventBitSet;
        bool hasHitEvents = taskContext.hasHitEvents;
        float negHitThreshold = -context.world.hitEventThreshold;
        int wideIndex = block.startIndex;
        int endWideIndex = block.startIndex + block.count;
        int colorIndex = 0;
        while (spans[colorIndex + 1].start <= wideIndex) colorIndex++;
        while (wideIndex < endWideIndex)
        {
            int colorWideEndIndex = Math.Min(spans[colorIndex + 1].start, endWideIndex);
            int colorWideStart = spans[colorIndex].start;
            int colorContactCount = spans[colorIndex].count;
            var contactSims = spans[colorIndex].contacts;
            for (; wideIndex < colorWideEndIndex; wideIndex++)
            {
                ContactConstraintWide* c = wideBase + wideIndex;
                float* rollingImpulse = (float*)&c->rollingImpulse;
                float* normalImpulse1 = (float*)&c->normalImpulse1;
                float* normalImpulse2 = (float*)&c->normalImpulse2;
                float* tangentImpulse1 = (float*)&c->tangentImpulse1;
                float* tangentImpulse2 = (float*)&c->tangentImpulse2;
                float* totalNormalImpulse1 = (float*)&c->totalNormalImpulse1;
                float* totalNormalImpulse2 = (float*)&c->totalNormalImpulse2;
                int localWideIndex = wideIndex - colorWideStart;
                int baseIndex = 8 * localWideIndex;
                for (int laneIndex = 0; laneIndex < 4; ++laneIndex)
                {
                    int contactIndex = baseIndex + laneIndex;
                    if (contactIndex >= colorContactCount) break;
                    ContactSim contactSim = contactSims[contactIndex];
                    ref Manifold m = ref contactSim.manifold;
                    m.rollingImpulse = rollingImpulse[laneIndex];
                    m.point0.normalImpulse = normalImpulse1[laneIndex];
                    m.point0.tangentImpulse = tangentImpulse1[laneIndex];
                    m.point0.totalNormalImpulse = totalNormalImpulse1[laneIndex];
                    m.point1.normalImpulse = normalImpulse2[laneIndex];
                    m.point1.tangentImpulse = tangentImpulse2[laneIndex];
                    m.point1.totalNormalImpulse = totalNormalImpulse2[laneIndex];
                    if (contactSim.simFlags.HasFlag(ContactFlags.SimEnableHitEvent))
                    {
                        if (contactSim.manifold.pointCount > 0 && m.point0.normalVelocity < negHitThreshold && m.point0.totalNormalImpulse > 0)
                        {
                            hitEventBitSet.SetBit(contactSim.contactId);
                            hasHitEvents = true;
                            break;
                        }
                        if (contactSim.manifold.pointCount > 1 && m.point1.normalVelocity < negHitThreshold && m.point1.totalNormalImpulse > 0)
                        {
                            hitEventBitSet.SetBit(contactSim.contactId);
                            hasHitEvents = true;
                            break;
                        }
                    }
                }
            }
            colorIndex++;
        }
    }
}
public class ContactSolverSSE : IContactSolverW
{
    public struct Vector2W
    {
        public Vector128<float> X, Y;
        public static Vector2W operator -(Vector2W a, Vector2W b) => new() { X = a.X - b.X, Y = a.Y - b.Y };
    }
    struct RotationW
    {
        public Vector128<float> C, S;
    }
    static Vector128<float> NegW(Vector128<float> a) => Sse.Xor(a, Vector128.Create(-0f));
    static Vector128<float> SymClampW(Vector128<float> a, Vector128<float> b) => Sse.Max(Sse.Subtract(Vector128<float>.Zero, b), Sse.Min(a, b));
    static bool AllZeroW(Vector128<float> a) => Sse.MoveMask(Sse.CompareEqual(a, Vector128<float>.Zero)) == 0xFF;
    static Vector128<float> BlendW(Vector128<float> a, Vector128<float> b, Vector128<float> mask) => Sse.Or(Sse.And(mask, b), Sse.AndNot(mask, a));
    static Vector2W RightPerpW(Vector2W a) => new() { X = a.Y, Y = -a.X };
    static Vector128<float> DotW(Vector2W a, Vector2W b) => Sse.Add(Sse.Multiply(a.X, b.X), Sse.Multiply(a.Y, b.Y));
    static Vector128<float> CrossW(Vector2W a, Vector2W b) => Sse.Subtract(Sse.Multiply(a.X, b.Y), Sse.Multiply(a.Y, b.X));
    static Vector2W RotateVectorW(RotationW q, Vector2W v) =>
        new() { X = Sse.Subtract(Sse.Multiply(q.C, v.X), Sse.Multiply(q.S, v.Y)), Y = Sse.Add(Sse.Multiply(q.S, v.X), Sse.Multiply(q.C, v.Y)) };
    static Vector128<float> SoftMaskW(Vector128<int> a, Vector128<int> b) => Sse2.Or(Sse2.CompareEqual(a, Vector128<int>.Zero), Sse2.CompareEqual(b, Vector128<int>.Zero)).AsSingle();
    public struct ContactConstraintWide
    {
        public Vector128<int> indexA, indexB;
        public Vector128<float> invMassA, invMassB;
        public Vector128<float> invIA, invIB;
        public Vector2W normal;
        public Vector2W anchorA1, anchorB1;
        public Vector2W anchorA2, anchorB2;
        public Vector128<float> normalMass1, normalMass2;
        public Vector128<float> baseSeparation1, baseSeparation2;
        public Vector128<float> normalImpulse1, normalImpulse2;
        public Vector128<float> totalNormalImpulse1, totalNormalImpulse2;
        public Vector128<float> tangentImpulse1, tangentImpulse2;
        public Vector128<float> rollingImpulse;
        public Vector128<float> friction;
        public Vector128<float> tangentSpeed;
        public Vector128<float> rollingResistance;
        public Vector128<float> tangentMass1, tangentMass2;
        public Vector128<float> negRestitutionVelocity1, negRestitutionVelocity2;
    }
    struct BodyStateW
    {
        public Vector2W v;
        public Vector128<float> w;
        public Vector128<float> flags;
        public Vector2W dp;
        public RotationW dq;
    }
    unsafe BodyStateW GatherBodies(BodyState* states, int* indices)
    {
        Debug.Assert(((nuint)states & 0x1F) == 0);
        Vector128<float> identityA = Vector128.Create(0f, 0, 0, 0), identityB = Vector128.Create(0f, 0, 1, 0);
        int i1 = indices[0] - 1, i2 = indices[1] - 1, i3 = indices[2] - 1, i4 = indices[3] - 1;
        Vector128<float> b1a = i1 == -1 ? identityA : Sse.LoadAlignedVector128((float*)(states + i1));
        Vector128<float> b1b = i1 == -1 ? identityB : Sse.LoadAlignedVector128((float*)(states + i1));
        Vector128<float> b2a = i2 == -1 ? identityA : Sse.LoadAlignedVector128((float*)(states + i2));
        Vector128<float> b2b = i2 == -1 ? identityB : Sse.LoadAlignedVector128((float*)(states + i2));
        Vector128<float> b3a = i3 == -1 ? identityA : Sse.LoadAlignedVector128((float*)(states + i3));
        Vector128<float> b3b = i3 == -1 ? identityB : Sse.LoadAlignedVector128((float*)(states + i3));
        Vector128<float> b4a = i4 == -1 ? identityA : Sse.LoadAlignedVector128((float*)(states + i4));
        Vector128<float> b4b = i4 == -1 ? identityB : Sse.LoadAlignedVector128((float*)(states + i4));
        Vector128<float> t1a = Sse.UnpackLow(b1a, b3a);
        Vector128<float> t2a = Sse.UnpackLow(b2a, b4a);
        Vector128<float> t3a = Sse.UnpackHigh(b1a, b3a);
        Vector128<float> t4a = Sse.UnpackHigh(b2a, b4a);
        Vector128<float> t1b = Sse.UnpackLow(b1b, b3b);
        Vector128<float> t2b = Sse.UnpackLow(b2b, b4b);
        Vector128<float> t3b = Sse.UnpackHigh(b1b, b3b);
        Vector128<float> t4b = Sse.UnpackHigh(b2b, b4b);
        return new()
        {
            v = new() { X = Sse.UnpackLow(t1a, t2a), Y = Sse.UnpackHigh(t1a, t2a) },
            w = Sse.UnpackLow(t3a, t4a), flags = Sse.UnpackHigh(t3a, t4a),
            dp = new() { X = Sse.UnpackLow(t1b, t2b), Y = Sse.UnpackHigh(t1b, t2b) },
            dq = new() { C = Sse.UnpackLow(t3b, t4b), S = Sse.UnpackHigh(t3b, t4b) },
        };
    }
    unsafe void ScatterBodies(BodyState* states, int* indices, ref BodyStateW simdBody)
    {
        Debug.Assert(((nuint)states & 0x1F) == 0);
        Vector128<float> t1 = Sse.UnpackLow(simdBody.v.X, simdBody.v.Y);
        Vector128<float> t2 = Sse.UnpackHigh(simdBody.v.X, simdBody.v.Y);
        Vector128<float> t3 = Sse.UnpackLow(simdBody.w, simdBody.flags);
        Vector128<float> t4 = Sse.UnpackHigh(simdBody.w, simdBody.flags);
        int i1 = indices[0] - 1, i2 = indices[1] - 1, i3 = indices[2] - 1, i4 = indices[3] - 1;
        if (i1 != -1 && states[i1].flags.HasFlag(BodyFlags.Dynamic)) Sse.StoreAligned((float*)(states + i1), Sse.Shuffle(t1, t3, 0b01000100));
        if (i2 != -1 && states[i2].flags.HasFlag(BodyFlags.Dynamic)) Sse.StoreAligned((float*)(states + i2), Sse.Shuffle(t1, t3, 0b11101110));
        if (i3 != -1 && states[i3].flags.HasFlag(BodyFlags.Dynamic)) Sse.StoreAligned((float*)(states + i3), Sse.Shuffle(t2, t4, 0b01000100));
        if (i4 != -1 && states[i4].flags.HasFlag(BodyFlags.Dynamic)) Sse.StoreAligned((float*)(states + i4), Sse.Shuffle(t2, t4, 0b11101110));
    }
    [System.Runtime.CompilerServices.InlineArray(4)] struct ContactSimLanes { public ContactSim sim; }
    public unsafe void PrepareContacts_Wide(ref SolverBlock block, StepContext context)
    {
        World world = context.world;
        var spans = context.contactPrepareSpans;
        var wideBase = (ContactConstraintsSSE)context.wideContactConstraints;
        Vector128<float> warmStartScale = world.enableWarmStarting ? Vector128<float>.One : Vector128<float>.Zero;
        int wideIndex = block.startIndex, endWideIndex = block.startIndex + block.count;
        int colorIndex = 0;
        while (spans[colorIndex + 1].start <= wideIndex) colorIndex++;
        while (wideIndex < endWideIndex)
        {
            int colorWideStart = spans[colorIndex].start;
            int colorWideEndIndex = Math.Min(spans[colorIndex + 1].start, endWideIndex);
            int colorContactCount = spans[colorIndex].count;
            var contactSims = spans[colorIndex].contacts;
            ContactSimLanes contactLanes = new();
            for (; wideIndex < colorWideEndIndex; wideIndex++)
            {
                var cw = wideBase.wideConstraints + wideIndex;
                int localWideIndex = wideIndex - colorWideStart;
                for (int laneIndex = 0; laneIndex < 4; laneIndex++)
                {
                    int contactIndex = 4 * localWideIndex + laneIndex;
                    if (contactIndex < colorContactCount)
                    {
                        ContactSim c = contactSims[contactIndex];
                        contactLanes[laneIndex] = c;
                        ((int*)&cw->indexA)[laneIndex] = c.bodySimIndexA + 1;
                        ((int*)&cw->indexB)[laneIndex] = c.bodySimIndexB + 1;
#if B2_VALIDATE
                        Body bodyA = world.bodies[c.bodyIdA];
                        int validIndexA = bodyA.setIndex == (int)SetType.Awake ? bodyA.localIndex : -1;
                        Body bodyB = world.bodies[c.bodyIdB];
                        int validIndexB = bodyB.setIndex == (int)SetType.Awake ? bodyB.localIndex : -1;
                        Debug.Assert(c.bodyIdA == validIndexA);
                        Debug.Assert(c.bodyIdB == validIndexB);
#endif
                    }
                    else contactLanes[laneIndex] = ContactSim.Zero;
                }
                cw->invMassA = Vector128.Create(contactLanes[0].invMassA, contactLanes[1].invMassA, contactLanes[2].invMassA, contactLanes[3].invMassA);
                cw->invMassB = Vector128.Create(contactLanes[0].invMassB, contactLanes[1].invMassB, contactLanes[2].invMassB, contactLanes[3].invMassB);
                cw->invIA = Vector128.Create(contactLanes[0].invIA, contactLanes[1].invIA, contactLanes[2].invIA, contactLanes[3].invIA);
                cw->invIB = Vector128.Create(contactLanes[0].invIB, contactLanes[1].invIB, contactLanes[2].invIB, contactLanes[3].invIB);
                cw->normal.X = Vector128.Create(contactLanes[0].manifold.normal.x, contactLanes[1].manifold.normal.x, contactLanes[2].manifold.normal.x, contactLanes[3].manifold.normal.x);
                cw->normal.Y = Vector128.Create(contactLanes[0].manifold.normal.y, contactLanes[1].manifold.normal.y, contactLanes[2].manifold.normal.y, contactLanes[3].manifold.normal.y);
                cw->friction = Vector128.Create(contactLanes[0].friction, contactLanes[1].friction, contactLanes[2].friction, contactLanes[3].friction);
                cw->tangentSpeed = Vector128.Create(contactLanes[0].tangentSpeed, contactLanes[1].tangentSpeed, contactLanes[2].tangentSpeed, contactLanes[3].tangentSpeed);
                cw->rollingResistance = Vector128.Create(contactLanes[0].rollingResistance, contactLanes[1].rollingResistance, contactLanes[2].rollingResistance, contactLanes[3].rollingResistance);
                cw->rollingImpulse = Vector128.Create(contactLanes[0].manifold.rollingImpulse, contactLanes[1].manifold.rollingImpulse, contactLanes[2].manifold.rollingImpulse, contactLanes[3].manifold.rollingImpulse);
                cw->rollingImpulse = Sse.Multiply(warmStartScale, cw->rollingImpulse);
                Vector2W tangent = RightPerpW(cw->normal);
                {
                    Vector128<float> m1a, m1b; fixed (Manifold* m = &contactLanes[0].manifold) { m1a = Sse.LoadVector128(&m->point0.anchorA.x); m1b = Sse.LoadVector128(&m->point0.anchorB.x); }
                    Vector128<float> m2a, m2b; fixed (Manifold* m = &contactLanes[1].manifold) { m2a = Sse.LoadVector128(&m->point0.anchorA.x); m2b = Sse.LoadVector128(&m->point0.anchorB.x); }
                    Vector128<float> m3a, m3b; fixed (Manifold* m = &contactLanes[2].manifold) { m3a = Sse.LoadVector128(&m->point0.anchorA.x); m3b = Sse.LoadVector128(&m->point0.anchorB.x); }
                    Vector128<float> m4a, m4b; fixed (Manifold* m = &contactLanes[3].manifold) { m4a = Sse.LoadVector128(&m->point0.anchorA.x); m4b = Sse.LoadVector128(&m->point0.anchorB.x); }
                    Vector128<float> t1a = Sse.UnpackLow(m1a, m3a), t2a = Sse.UnpackLow(m2a, m4a);
                    Vector128<float> t3a = Sse.UnpackHigh(m1a, m3a), t4a = Sse.UnpackHigh(m2a, m4a);
                    Vector128<float> t1b = Sse.UnpackLow(m1b, m3b), t2b = Sse.UnpackLow(m2b, m4b);
                    Vector128<float> t3b = Sse.UnpackHigh(m1b, m3b), t4b = Sse.UnpackHigh(m2b, m4b);
                    cw->anchorA2.X = Sse.UnpackLow(t1a, t2a);
                    cw->anchorA2.Y = Sse.UnpackHigh(t1a, t2a);
                    cw->anchorB2.X = Sse.UnpackLow(t3a, t4a);
                    cw->anchorB2.Y = Sse.UnpackHigh(t3a, t4a);
                    cw->baseSeparation2 = Sse.UnpackLow(t1b, t2b);
                    cw->normalImpulse2 = Sse.UnpackHigh(t1b, t2b);
                    cw->tangentImpulse2 = Sse.UnpackLow(t3b, t4b);
                    cw->negRestitutionVelocity2 = Sse.UnpackHigh(t3b, t4b);

                    Vector128<float> offset = DotW(cw->anchorB1 - cw->anchorA1, cw->normal);
                    cw->baseSeparation1 = cw->baseSeparation1 - offset;
                    cw->negRestitutionVelocity1 = -cw->negRestitutionVelocity1;
                    cw->normalImpulse1 = warmStartScale * cw->normalImpulse1;
                    cw->tangentImpulse1 = warmStartScale * cw->tangentImpulse1;
                    cw->totalNormalImpulse1 = Vector128<float>.Zero;
                    {
                        Vector128<float> rnA = CrossW(cw->anchorA1, cw->normal);
                        Vector128<float> rnB = CrossW(cw->anchorB1, cw->normal);
                        Vector128<float> k = cw->invMassA + cw->invMassB + cw->invIA * rnA * rnA + cw->invIB * rnB * rnB;
                        cw->normalMass1 = BlendW(Vector128<float>.Zero, Vector128<float>.One / k, Sse.CompareGreaterThan(k, Vector128<float>.Zero));
                    }
                    {
                        Vector128<float> rnA = CrossW(cw->anchorA1, tangent);
                        Vector128<float> rnB = CrossW(cw->anchorB1, tangent);
                        Vector128<float> k = cw->invMassA + cw->invMassB + cw->invIA * rnA * rnA + cw->invIB * rnB * rnB;
                        cw->tangentMass1 = BlendW(Vector128<float>.Zero, Vector128<float>.One / k, Sse.CompareGreaterThan(k, Vector128<float>.Zero));
                    }
                }
                {
                    Vector128<float> m1a, m1b; fixed (Manifold* m = &contactLanes[0].manifold) { m1a = Sse.LoadVector128(&m->point1.anchorA.x); m1b = Sse.LoadVector128(&m->point1.anchorB.x); }
                    Vector128<float> m2a, m2b; fixed (Manifold* m = &contactLanes[1].manifold) { m2a = Sse.LoadVector128(&m->point1.anchorA.x); m2b = Sse.LoadVector128(&m->point1.anchorB.x); }
                    Vector128<float> m3a, m3b; fixed (Manifold* m = &contactLanes[2].manifold) { m3a = Sse.LoadVector128(&m->point1.anchorA.x); m3b = Sse.LoadVector128(&m->point1.anchorB.x); }
                    Vector128<float> m4a, m4b; fixed (Manifold* m = &contactLanes[3].manifold) { m4a = Sse.LoadVector128(&m->point1.anchorA.x); m4b = Sse.LoadVector128(&m->point1.anchorB.x); }
                    Vector128<float> t1a = Sse.UnpackLow(m1a, m3a), t2a = Sse.UnpackLow(m2a, m4a);
                    Vector128<float> t3a = Sse.UnpackHigh(m1a, m3a), t4a = Sse.UnpackHigh(m2a, m4a);
                    Vector128<float> t1b = Sse.UnpackLow(m1b, m3b), t2b = Sse.UnpackLow(m2b, m4b);
                    Vector128<float> t3b = Sse.UnpackHigh(m1b, m3b), t4b = Sse.UnpackHigh(m2b, m4b);
                    cw->anchorA2.X = Sse.UnpackLow(t1a, t2a);
                    cw->anchorA2.Y = Sse.UnpackHigh(t1a, t2a);
                    cw->anchorB2.X = Sse.UnpackLow(t3a, t4a);
                    cw->anchorB2.Y = Sse.UnpackHigh(t3a, t4a);
                    cw->baseSeparation2 = Sse.UnpackLow(t1b, t2b);
                    cw->normalImpulse2 = Sse.UnpackHigh(t1b, t2b);
                    cw->tangentImpulse2 = Sse.UnpackLow(t3b, t4b);
                    cw->negRestitutionVelocity2 = Sse.UnpackHigh(t3b, t4b);

                    Vector128<float> offset = DotW(cw->anchorB2 - cw->anchorA2, cw->normal);
                    cw->baseSeparation2 = cw->baseSeparation2 - offset;
                    cw->negRestitutionVelocity2 = -cw->negRestitutionVelocity2;
                    cw->normalImpulse2 = warmStartScale * cw->normalImpulse2;
                    cw->tangentImpulse2 = warmStartScale * cw->tangentImpulse2;
                    cw->totalNormalImpulse2 = Vector128<float>.Zero;
                    {
                        Vector128<float> rnA = CrossW(cw->anchorA2, cw->normal);
                        Vector128<float> rnB = CrossW(cw->anchorB2, cw->normal);
                        Vector128<float> k = cw->invMassA + cw->invMassB + cw->invIA * rnA * rnA + cw->invIB * rnB * rnB;
                        cw->normalMass2 = BlendW(Vector128<float>.Zero, Vector128<float>.One / k, Sse.CompareGreaterThan(k, Vector128<float>.Zero));
                    }
                    {
                        Vector128<float> rnA = CrossW(cw->anchorA2, tangent);
                        Vector128<float> rnB = CrossW(cw->anchorB2, tangent);
                        Vector128<float> k = cw->invMassA + cw->invMassB + cw->invIA * rnA * rnA + cw->invIB * rnB * rnB;
                        cw->tangentMass2 = BlendW(Vector128<float>.Zero, Vector128<float>.One / k, Sse.CompareGreaterThan(k, Vector128<float>.Zero));
                    }
                }
                Vector128<float> massScale = Vector128.GreaterThan(Vector128.Create(contactLanes[0].manifold.pointCount, contactLanes[1].manifold.pointCount, contactLanes[2].manifold.pointCount, contactLanes[3].manifold.pointCount), Vector128<int>.One).AsSingle();
                cw->normalMass2 = BlendW(Vector128<float>.Zero, cw->normalMass2, massScale);
                cw->tangentMass2 = BlendW(Vector128<float>.Zero, cw->tangentMass2, massScale);
            }
            colorIndex++;
        }
    }
    public unsafe void WarmStartContacts_Wide(ref SolverBlock block, StepContext context)
    {
        var states = context.states.Data;
        var constraints = ((ContactConstraintsSSE)context.graph.colors[block.colorIndex].wideConstraints).wideConstraints;
        {
            for (int wideIndex = block.startIndex; wideIndex < block.startIndex + block.count; wideIndex++)
            {
                ContactConstraintWide* c = constraints + wideIndex;
                BodyStateW bA = GatherBodies(states, (int*)&c->indexA);
                BodyStateW bB = GatherBodies(states, (int*)&c->indexB);
                Vector128<float> tangentX = c->normal.Y;
                Vector128<float> tangentY = Sse.Subtract(Vector128<float>.Zero, c->normal.X);
                {
                    Vector2W rA = c->anchorA1, rB = c->anchorB1;
                    Vector2W P = new()
                    {
                        X = Sse.Add(Sse.Multiply(c->normalImpulse1, c->normal.X), Sse.Multiply(c->tangentImpulse1, tangentX)),
                        Y = Sse.Add(Sse.Multiply(c->normalImpulse1, c->normal.Y), Sse.Multiply(c->tangentImpulse1, tangentY))
                    };
                    bA.w = Sse.Subtract(bA.w, Sse.Multiply(c->invIA, CrossW(rA, P)));
                    bA.v = new()
                    {
                        X = Sse.Subtract(bA.v.X, Sse.Multiply(c->invMassA, P.X)),
                        Y = Sse.Subtract(bA.v.Y, Sse.Multiply(c->invMassA, P.Y))
                    };
                    bB.w = Sse.Add(bB.w, Sse.Multiply(c->invIB, CrossW(rB, P)));
                    bB.v = new()
                    {
                        X = Sse.Add(bB.v.X, Sse.Multiply(c->invMassB, P.X)),
                        Y = Sse.Add(bB.v.Y, Sse.Multiply(c->invMassB, P.Y))
                    };
                    c->totalNormalImpulse1 = Sse.Add(c->totalNormalImpulse1, c->normalImpulse1);
                }
                {
                    Vector2W rA = c->anchorA2, rB = c->anchorB2;
                    Vector2W P = new()
                    {
                        X = Sse.Add(Sse.Multiply(c->normalImpulse2, c->normal.X), Sse.Multiply(c->tangentImpulse2, tangentX)),
                        Y = Sse.Add(Sse.Multiply(c->normalImpulse2, c->normal.Y), Sse.Multiply(c->tangentImpulse2, tangentY))
                    };
                    bA.w = Sse.Subtract(bA.w, Sse.Multiply(c->invIA, CrossW(rA, P)));
                    bA.v = new()
                    {
                        X = Sse.Subtract(bA.v.X, Sse.Multiply(c->invMassA, P.X)),
                        Y = Sse.Subtract(bA.v.Y, Sse.Multiply(c->invMassA, P.Y))
                    };
                    bB.w = Sse.Add(bB.w, Sse.Multiply(c->invIB, CrossW(rB, P)));
                    bB.v = new()
                    {
                        X = Sse.Add(bB.v.X, Sse.Multiply(c->invMassB, P.X)),
                        Y = Sse.Add(bB.v.Y, Sse.Multiply(c->invMassB, P.Y))
                    };
                    c->totalNormalImpulse2 = Sse.Add(c->totalNormalImpulse2, c->normalImpulse2);
                }
                bA.w = Sse.Subtract(bA.w, Sse.Multiply(c->invIA, c->rollingImpulse));
                bB.w = Sse.Add(bB.w, Sse.Multiply(c->invIB, c->rollingImpulse));
                ScatterBodies(states, (int*)&c->indexA, ref bA);
                ScatterBodies(states, (int*)&c->indexB, ref bB);
            }
        }
    }
    public unsafe void PushContacts_Wide(ref SolverBlock block, StepContext context)
    {
        var states = context.states.Data;
        var constraints = ((ContactConstraintsSSE)context.graph.colors[block.colorIndex].wideConstraints).wideConstraints;
        {
            Vector128<float> inv_h = Vector128.Create(context.inv_h);
            Vector128<float> contactSpeed = Vector128.Create(-context.world.contactSpeed);
            Vector128<float> oneW = Vector128<float>.One;
            Vector128<float> dynamicBiasRate = Vector128.Create(context.contactSoftness.massScale * context.contactSoftness.biasRate);
            Vector128<float> dynamicMassScale = Vector128.Create(context.contactSoftness.massScale);
            Vector128<float> dynamicImpulseScale = Vector128.Create(context.contactSoftness.impulseScale);
            Vector128<float> staticBiasRate = Vector128.Create(context.staticSoftness.massScale * context.staticSoftness.biasRate);
            Vector128<float> staticMassScale = Vector128.Create(context.staticSoftness.massScale);
            Vector128<float> staticImpulseScale = Vector128.Create(context.staticSoftness.impulseScale);
            for (int wideIndex = block.startIndex; wideIndex < block.startIndex + block.count; wideIndex++)
            {
                ContactConstraintWide* c = constraints + wideIndex;
                BodyStateW bA = GatherBodies(states, (int*)&c->indexA);
                BodyStateW bB = GatherBodies(states, (int*)&c->indexB);
                Vector128<float> softMask = SoftMaskW(c->indexA, c->indexB),
                    biasRate = BlendW(dynamicBiasRate, staticBiasRate, softMask),
                    massScale = BlendW(dynamicMassScale, staticMassScale, softMask),
                    impulseScale = BlendW(dynamicImpulseScale, staticImpulseScale, softMask);
                Vector2W dp = new() { X = Sse.Subtract(bB.dp.X, bA.dp.X), Y = Sse.Subtract(bB.dp.Y, bA.dp.Y) };
                {
                    Vector2W rA = c->anchorA1, rB = c->anchorB1;
                    Vector2W rsA = RotateVectorW(bA.dq, rA), rsB = RotateVectorW(bB.dq, rB);
                    Vector2W ds = new() { X = Sse.Add(dp.X, Sse.Subtract(rsB.X, rsA.X)), Y = Sse.Add(dp.Y, Sse.Subtract(rsB.Y, rsA.Y)) };
                    Vector128<float> s = Sse.Add(DotW(c->normal, ds), c->baseSeparation1);
                    Vector128<float> separated = Sse.CompareGreaterThan(s, Vector128<float>.Zero);
                    Vector128<float> specBias = Sse.Multiply(s, inv_h), overlapBias = Sse.Max(Sse.Multiply(biasRate, s), contactSpeed);
                    Vector128<float> velocityBias = BlendW(overlapBias, specBias, separated);
                    Vector128<float> pointMassScale = BlendW(massScale, oneW, separated);
                    Vector128<float> pointImpulseScale = BlendW(impulseScale, Vector128<float>.Zero, separated);
                    Vector128<float> dvx = Sse.Subtract(Sse.Subtract(bB.v.X, Sse.Multiply(bB.w, rB.Y)), Sse.Subtract(bA.v.X, Sse.Multiply(bA.w, rA.Y)));
                    Vector128<float> dvy = Sse.Subtract(Sse.Add(bB.v.Y, Sse.Multiply(bB.w, rB.X)), Sse.Add(bA.v.Y, Sse.Multiply(bA.w, rA.X)));
                    Vector128<float> vn = Sse.Add(Sse.Multiply(dvx, c->normal.X), Sse.Multiply(dvy, c->normal.Y));
                    Vector128<float> negImpulse = Sse.Add(Sse.Multiply(c->normalMass1, Sse.Add(Sse.Multiply(pointMassScale, vn), velocityBias)), Sse.Multiply(pointImpulseScale, c->normalImpulse1));
                    Vector128<float> newImpulse = Sse.Max(Sse.Subtract(c->normalImpulse1, negImpulse), Vector128<float>.Zero);
                    Vector128<float> impulse = Sse.Subtract(newImpulse, c->normalImpulse1);
                    c->normalImpulse1 = newImpulse;
                    c->totalNormalImpulse1 = Sse.Add(c->totalNormalImpulse1, impulse);
                    Vector128<float> Px = Sse.Multiply(impulse, c->normal.X);
                    Vector128<float> Py = Sse.Multiply(impulse, c->normal.Y);
                    bA.v = new()
                    {
                        X = Sse.Subtract(bA.v.X, Sse.Multiply(c->invMassA, Px)),
                        Y = Sse.Subtract(bA.v.Y, Sse.Multiply(c->invMassA, Py))
                    };
                    bA.w = Sse.Subtract(bA.w, Sse.Multiply(c->invIA, Sse.Subtract(Sse.Multiply(rA.X, Py), Sse.Multiply(rA.Y, Px))));
                    bB.v = new()
                    {
                        X = Sse.Add(bB.v.X, Sse.Multiply(c->invMassB, Px)),
                        Y = Sse.Add(bB.v.Y, Sse.Multiply(c->invMassB, Py))
                    };
                    bB.w = Sse.Add(bB.w, Sse.Multiply(c->invIB, Sse.Subtract(Sse.Multiply(rB.X, Py), Sse.Multiply(rB.Y, Px))));
                }
                {
                    Vector2W rA = c->anchorA2, rB = c->anchorB2;
                    Vector2W rsA = RotateVectorW(bA.dq, rA), rsB = RotateVectorW(bB.dq, rB);
                    Vector2W ds = new() { X = Sse.Add(dp.X, Sse.Subtract(rsB.X, rsA.X)), Y = Sse.Add(dp.Y, Sse.Subtract(rsB.Y, rsA.Y)) };
                    Vector128<float> s = Sse.Add(DotW(c->normal, ds), c->baseSeparation2);
                    Vector128<float> separated = Sse.CompareGreaterThan(s, Vector128<float>.Zero);
                    Vector128<float> specBias = Sse.Multiply(s, inv_h), overlapBias = Sse.Max(Sse.Multiply(biasRate, s), contactSpeed);
                    Vector128<float> velocityBias = BlendW(overlapBias, specBias, separated);
                    Vector128<float> pointMassScale = BlendW(massScale, oneW, separated);
                    Vector128<float> pointImpulseScale = BlendW(impulseScale, Vector128<float>.Zero, separated);
                    Vector128<float> dvx = Sse.Subtract(Sse.Subtract(bB.v.X, Sse.Multiply(bB.w, rB.Y)), Sse.Subtract(bA.v.X, Sse.Multiply(bA.w, rA.Y)));
                    Vector128<float> dvy = Sse.Subtract(Sse.Add(bB.v.Y, Sse.Multiply(bB.w, rB.X)), Sse.Add(bA.v.Y, Sse.Multiply(bA.w, rA.X)));
                    Vector128<float> vn = Sse.Add(Sse.Multiply(dvx, c->normal.X), Sse.Multiply(dvy, c->normal.Y));
                    Vector128<float> negImpulse = Sse.Add(Sse.Multiply(c->normalMass2, Sse.Add(Sse.Multiply(pointMassScale, vn), velocityBias)), Sse.Multiply(pointImpulseScale, c->normalImpulse2));
                    Vector128<float> newImpulse = Sse.Max(Sse.Subtract(c->normalImpulse2, negImpulse), Vector128<float>.Zero);
                    Vector128<float> impulse = Sse.Subtract(newImpulse, c->normalImpulse2);
                    c->normalImpulse2 = newImpulse;
                    c->totalNormalImpulse2 = Sse.Add(c->totalNormalImpulse2, impulse);
                    Vector128<float> Px = Sse.Multiply(impulse, c->normal.X);
                    Vector128<float> Py = Sse.Multiply(impulse, c->normal.Y);
                    bA.v = new()
                    {
                        X = Sse.Subtract(bA.v.X, Sse.Multiply(c->invMassA, Px)),
                        Y = Sse.Subtract(bA.v.Y, Sse.Multiply(c->invMassA, Py))
                    };
                    bA.w = Sse.Subtract(bA.w, Sse.Multiply(c->invIA, Sse.Subtract(Sse.Multiply(rA.X, Py), Sse.Multiply(rA.Y, Px))));
                    bB.v = new()
                    {
                        X = Sse.Add(bB.v.X, Sse.Multiply(c->invMassB, Px)),
                        Y = Sse.Add(bB.v.Y, Sse.Multiply(c->invMassB, Py))
                    };
                    bB.w = Sse.Add(bB.w, Sse.Multiply(c->invIB, Sse.Subtract(Sse.Multiply(rB.X, Py), Sse.Multiply(rB.Y, Px))));
                }
                ScatterBodies(states, (int*)&c->indexA, ref bA);
                ScatterBodies(states, (int*)&c->indexB, ref bB);
            }
        }
    }
    public unsafe void SolveContacts_Wide(ref SolverBlock block, StepContext context)
    {
        var states = context.states.Data;
        GraphColor color = context.graph.colors[block.colorIndex];
        var constraints = ((ContactConstraintsSSE)context.graph.colors[block.colorIndex].wideConstraints).wideConstraints;
        Vector128<float> inv_h = Vector128.Create(context.inv_h);
        for (int wideIndex = 0; wideIndex < block.startIndex + block.count; wideIndex++)
        {
            ContactConstraintWide* c = constraints + wideIndex;
            BodyStateW bA = GatherBodies(states, (int*)&c->indexA);
            BodyStateW bB = GatherBodies(states, (int*)&c->indexB);
            Vector128<float> resitutionMask1 = Sse.CompareGreaterThan(Vector128<float>.Zero, c->negRestitutionVelocity1);
            Vector128<float> resitutionMask2 = Sse.CompareGreaterThan(Vector128<float>.Zero, c->negRestitutionVelocity2);
            bool haveResitution = !AllZeroW(Sse.Or(resitutionMask1, resitutionMask2));
            Vector128<float> keepRestitution = Vector128<float>.Zero;
            Vector128<float> totalNormalImpulse = Vector128<float>.Zero;
            Vector2W dp = bB.dp - bA.dp;
            {
                Vector2W rA = c->anchorA1, rB = c->anchorB1;
                Vector2W rsA = RotateVectorW(bA.dq, rA), rsB = RotateVectorW(bB.dq, rB);
                Vector2W ds = new() { X = dp.X + (rsB.X - rsA.X), Y = dp.Y + (rsB.Y - rsA.Y) };
                Vector128<float> s = DotW(c->normal, ds) + c->baseSeparation1;
                Vector128<float> specBias = s * inv_h;
                Vector128<float> velocityBias = Sse.Max(Vector128<float>.Zero, specBias);
                if (haveResitution)
                {
                    Vector128<float> separated = Sse.CompareGreaterThan(s, Vector128<float>.Zero);
                    velocityBias = BlendW(velocityBias, c->negRestitutionVelocity1, resitutionMask1);
                    keepRestitution = Sse.Or(keepRestitution, Sse.AndNot(resitutionMask1, separated));
                }
                Vector128<float> dvx = (bB.v.X - bB.w * rB.Y) - (bA.v.X - bA.w * rA.Y);
                Vector128<float> dvy = (bB.v.Y + bB.w * rB.X) - (bA.v.Y + bA.w * rA.X);
                Vector128<float> vn = dvx * c->normal.X + dvy * c->normal.Y;
                Vector128<float> newImpulse = Sse.Max(c->normalImpulse1 - c->normalMass1 * (vn + velocityBias), Vector128<float>.Zero);
                Vector128<float> impulse = newImpulse - c->normalImpulse1;
                c->normalImpulse1 = newImpulse;
                c->totalNormalImpulse1 = c->totalNormalImpulse1 + impulse;
                totalNormalImpulse = totalNormalImpulse + newImpulse;
                Vector128<float> Px = impulse * c->normal.X, Py = impulse * c->normal.Y;
                bA.v = new()
                {
                    X = Sse.Subtract(bA.v.X, Sse.Multiply(c->invMassA, Px)),
                    Y = Sse.Subtract(bA.v.Y, Sse.Multiply(c->invMassA, Py))
                };
                bA.w = Sse.Subtract(bA.w, Sse.Multiply(c->invIA, Sse.Subtract(Sse.Multiply(rA.X, Py), Sse.Multiply(rA.Y, Px))));
                bB.v = new()
                {
                    X = Sse.Add(bB.v.X, Sse.Multiply(c->invMassB, Px)),
                    Y = Sse.Add(bB.v.Y, Sse.Multiply(c->invMassB, Py))
                };
                bB.w = Sse.Add(bB.w, Sse.Multiply(c->invIB, Sse.Subtract(Sse.Multiply(rB.X, Py), Sse.Multiply(rB.Y, Px))));
            }
            {
                Vector2W rA = c->anchorA2, rB = c->anchorB2;
                Vector2W rsA = RotateVectorW(bA.dq, rA), rsB = RotateVectorW(bB.dq, rB);
                Vector2W ds = new() { X = dp.X + (rsB.X - rsA.X), Y = dp.Y + (rsB.Y - rsA.Y) };
                Vector128<float> s = DotW(c->normal, ds) + c->baseSeparation2;
                Vector128<float> specBias = s * inv_h;
                Vector128<float> velocityBias = Sse.Max(Vector128<float>.Zero, specBias);
                if (haveResitution)
                {
                    Vector128<float> separated = Sse.CompareGreaterThan(s, Vector128<float>.Zero);
                    velocityBias = BlendW(velocityBias, c->negRestitutionVelocity2, resitutionMask2);
                    keepRestitution = Sse.Or(keepRestitution, Sse.AndNot(resitutionMask2, separated));
                }
                Vector128<float> dvx = (bB.v.X - bB.w * rB.Y) - (bA.v.X - bA.w * rA.Y);
                Vector128<float> dvy = (bB.v.Y + bB.w * rB.X) - (bA.v.Y + bA.w * rA.X);
                Vector128<float> vn = dvx * c->normal.X + dvy * c->normal.Y;
                Vector128<float> newImpulse = Sse.Max(c->normalImpulse2 - c->normalMass2 * (vn + velocityBias), Vector128<float>.Zero);
                Vector128<float> impulse = newImpulse - c->normalImpulse2;
                c->normalImpulse2 = newImpulse;
                c->totalNormalImpulse2 = c->totalNormalImpulse2 + impulse;
                totalNormalImpulse = totalNormalImpulse + newImpulse;
                Vector128<float> Px = impulse * c->normal.X, Py = impulse * c->normal.Y;
                bA.v = new()
                {
                    X = Sse.Subtract(bA.v.X, Sse.Multiply(c->invMassA, Px)),
                    Y = Sse.Subtract(bA.v.Y, Sse.Multiply(c->invMassA, Py))
                };
                bA.w = Sse.Subtract(bA.w, Sse.Multiply(c->invIA, Sse.Subtract(Sse.Multiply(rA.X, Py), Sse.Multiply(rA.Y, Px))));
                bB.v = new()
                {
                    X = Sse.Add(bB.v.X, Sse.Multiply(c->invMassB, Px)),
                    Y = Sse.Add(bB.v.Y, Sse.Multiply(c->invMassB, Py))
                };
                bB.w = Sse.Add(bB.w, Sse.Multiply(c->invIB, Sse.Subtract(Sse.Multiply(rB.X, Py), Sse.Multiply(rB.Y, Px))));
            }
            c->negRestitutionVelocity1 = BlendW(Vector128<float>.Zero, c->negRestitutionVelocity1, keepRestitution);
            c->negRestitutionVelocity2 = BlendW(Vector128<float>.Zero, c->negRestitutionVelocity2, keepRestitution);
            if (!AllZeroW(c->rollingResistance))
            {
                Vector128<float> k = c->invIA + c->invIB;
                Vector128<float> deltaLambda = BlendW(Vector128<float>.Zero, Sse.Divide(Sse.Subtract(bA.w, bB.w), k), Sse.CompareGreaterThan(k, Vector128<float>.Zero));
                Vector128<float> lambda = c->rollingImpulse;
                Vector128<float> maxLambda = Sse.Multiply(c->rollingResistance, totalNormalImpulse);
                c->rollingImpulse = SymClampW(Sse.Add(lambda, deltaLambda), maxLambda);
                deltaLambda = Sse.Subtract(c->rollingImpulse, lambda);
                bA.w = Sse.Subtract(bA.w, Sse.Multiply(c->invIA, deltaLambda));
                bB.w = Sse.Add(bB.w, Sse.Multiply(c->invIB, deltaLambda));
            }
            Vector128<float> tangentX = c->normal.Y;
            Vector128<float> tangentY = Sse.Subtract(Vector128<float>.Zero, c->normal.X);
            {
                Vector2W rA = c->anchorA1, rB = c->anchorB1;
                Vector128<float> dvx = Sse.Subtract(Sse.Subtract(bB.v.X, Sse.Multiply(bB.w, rB.Y)), Sse.Subtract(bA.v.X, Sse.Multiply(bA.w, rA.Y)));
                Vector128<float> dvy = Sse.Subtract(Sse.Add(bB.v.Y, Sse.Multiply(bB.w, rB.X)), Sse.Add(bA.v.Y, Sse.Multiply(bA.w, rA.X)));
                Vector128<float> vt = Sse.Add(Sse.Multiply(dvx, tangentX), Sse.Multiply(dvy, tangentY));
                vt = Sse.Subtract(vt, c->tangentSpeed);
                Vector128<float> negImpulse = Sse.Multiply(c->tangentMass1, vt);
                Vector128<float> maxFriction = Sse.Multiply(c->friction, c->normalImpulse1);
                Vector128<float> newImpulse = Sse.Subtract(c->tangentImpulse1, negImpulse);
                //no symclamp?
                newImpulse = Sse.Max(Sse.Subtract(Vector128<float>.Zero, maxFriction), Sse.Min(newImpulse, maxFriction));
                Vector128<float> impulse = Sse.Subtract(newImpulse, c->tangentImpulse1);
                c->tangentImpulse1 = newImpulse;
                Vector128<float> Px = Sse.Multiply(impulse, tangentX);
                Vector128<float> Py = Sse.Multiply(impulse, tangentY);
                bA.v = new()
                {
                    X = Sse.Subtract(bA.v.X, Sse.Multiply(c->invMassA, Px)),
                    Y = Sse.Subtract(bA.v.Y, Sse.Multiply(c->invMassA, Py))
                };
                bA.w = Sse.Subtract(bA.w, Sse.Multiply(c->invIA, Sse.Subtract(Sse.Multiply(rA.X, Py), Sse.Multiply(rA.Y, Px))));
                bB.v = new()
                {
                    X = Sse.Add(bB.v.X, Sse.Multiply(c->invMassB, Px)),
                    Y = Sse.Add(bB.v.Y, Sse.Multiply(c->invMassB, Py))
                };
                bB.w = Sse.Add(bB.w, Sse.Multiply(c->invIB, Sse.Subtract(Sse.Multiply(rB.X, Py), Sse.Multiply(rB.Y, Px))));
            }
            {
                Vector2W rA = c->anchorA2, rB = c->anchorB2;
                Vector128<float> dvx = Sse.Subtract(Sse.Subtract(bB.v.X, Sse.Multiply(bB.w, rB.Y)), Sse.Subtract(bA.v.X, Sse.Multiply(bA.w, rA.Y)));
                Vector128<float> dvy = Sse.Subtract(Sse.Add(bB.v.Y, Sse.Multiply(bB.w, rB.X)), Sse.Add(bA.v.Y, Sse.Multiply(bA.w, rA.X)));
                Vector128<float> vt = Sse.Add(Sse.Multiply(dvx, tangentX), Sse.Multiply(dvy, tangentY));
                vt = Sse.Subtract(vt, c->tangentSpeed);
                Vector128<float> negImpulse = Sse.Multiply(c->tangentMass2, vt);
                Vector128<float> maxFriction = Sse.Multiply(c->friction, c->normalImpulse2);
                Vector128<float> newImpulse = Sse.Subtract(c->tangentImpulse2, negImpulse);
                newImpulse = Sse.Max(Sse.Subtract(Vector128<float>.Zero, maxFriction), Sse.Min(newImpulse, maxFriction));
                Vector128<float> impulse = Sse.Subtract(newImpulse, c->tangentImpulse2);
                c->tangentImpulse2 = newImpulse;
                Vector128<float> Px = Sse.Multiply(impulse, tangentX);
                Vector128<float> Py = Sse.Multiply(impulse, tangentY);
                bA.v = new()
                {
                    X = Sse.Subtract(bA.v.X, Sse.Multiply(c->invMassA, Px)),
                    Y = Sse.Subtract(bA.v.Y, Sse.Multiply(c->invMassA, Py))
                };
                bA.w = Sse.Subtract(bA.w, Sse.Multiply(c->invIA, Sse.Subtract(Sse.Multiply(rA.X, Py), Sse.Multiply(rA.Y, Px))));
                bB.v = new()
                {
                    X = Sse.Add(bB.v.X, Sse.Multiply(c->invMassB, Px)),
                    Y = Sse.Add(bB.v.Y, Sse.Multiply(c->invMassB, Py))
                };
                bB.w = Sse.Add(bB.w, Sse.Multiply(c->invIB, Sse.Subtract(Sse.Multiply(rB.X, Py), Sse.Multiply(rB.Y, Px))));
            }
            ScatterBodies(states, (int*)&c->indexA, ref bA);
            ScatterBodies(states, (int*)&c->indexB, ref bB);
        }
    }
    public unsafe void StoreImpulses_Wide(ref SolverBlock block, StepContext context, int workerIndex)
    {
        var spans = context.contactPrepareSpans;
        var wideBase = ((ContactConstraintsSSE)context.wideContactConstraints).wideConstraints;
        TaskContext taskContext = context.world.taskContexts[workerIndex];
        BitSet hitEventBitSet = taskContext.hitEventBitSet;
        bool hasHitEvents = taskContext.hasHitEvents;
        float negHitThreshold = -context.world.hitEventThreshold;
        int wideIndex = block.startIndex;
        int endWideIndex = block.startIndex + block.count;
        int colorIndex = 0;
        while (spans[colorIndex + 1].start <= wideIndex) colorIndex++;
        while (wideIndex < endWideIndex)
        {
            int colorWideEndIndex = Math.Min(spans[colorIndex + 1].start, endWideIndex);
            int colorWideStart = spans[colorIndex].start;
            int colorContactCount = spans[colorIndex].count;
            var contactSims = spans[colorIndex].contacts;
            for (; wideIndex < colorWideEndIndex; wideIndex++)
            {
                ContactConstraintWide* c = wideBase + wideIndex;
                float* rollingImpulse = (float*)&c->rollingImpulse;
                float* normalImpulse1 = (float*)&c->normalImpulse1;
                float* normalImpulse2 = (float*)&c->normalImpulse2;
                float* tangentImpulse1 = (float*)&c->tangentImpulse1;
                float* tangentImpulse2 = (float*)&c->tangentImpulse2;
                float* totalNormalImpulse1 = (float*)&c->totalNormalImpulse1;
                float* totalNormalImpulse2 = (float*)&c->totalNormalImpulse2;
                int localWideIndex = wideIndex - colorWideStart;
                int baseIndex = 8 * localWideIndex;
                for (int laneIndex = 0; laneIndex < 4; ++laneIndex)
                {
                    int contactIndex = baseIndex + laneIndex;
                    if (contactIndex >= colorContactCount) break;
                    ContactSim contactSim = contactSims[contactIndex];
                    ref Manifold m = ref contactSim.manifold;
                    m.rollingImpulse = rollingImpulse[laneIndex];
                    m.point0.normalImpulse = normalImpulse1[laneIndex];
                    m.point0.tangentImpulse = tangentImpulse1[laneIndex];
                    m.point0.totalNormalImpulse = totalNormalImpulse1[laneIndex];
                    m.point1.normalImpulse = normalImpulse2[laneIndex];
                    m.point1.tangentImpulse = tangentImpulse2[laneIndex];
                    m.point1.totalNormalImpulse = totalNormalImpulse2[laneIndex];
                    if (contactSim.simFlags.HasFlag(ContactFlags.SimEnableHitEvent))
                    {
                        if (contactSim.manifold.pointCount > 0 && m.point0.normalVelocity < negHitThreshold && m.point0.totalNormalImpulse > 0)
                        {
                            hitEventBitSet.SetBit(contactSim.contactId);
                            hasHitEvents = true;
                            break;
                        }
                        if (contactSim.manifold.pointCount > 1 && m.point1.normalVelocity < negHitThreshold && m.point1.totalNormalImpulse > 0)
                        {
                            hitEventBitSet.SetBit(contactSim.contactId);
                            hasHitEvents = true;
                            break;
                        }
                    }
                }
            }
            colorIndex++;
        }
    }
}
public class ContactSolverFloat : IContactSolverW
{
    public struct FloatW
    {
        public float x, y, z, w;
        public static readonly FloatW Zero = new();
        public static readonly FloatW One = new(1, 1, 1, 1);
        public FloatW(float scalar) { x = y = z = w = scalar; }
        public FloatW(float x, float y, float z, float w) { this.x = x; this.y = y; this.z = z; this.w = w; }
        public static FloatW operator -(FloatW a) => new(-a.x, -a.y, -a.z, -a.w);
        public static FloatW operator +(FloatW a, FloatW b) => new(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
        public static FloatW operator -(FloatW a, FloatW b) => new(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
        public static FloatW operator *(FloatW a, FloatW b) => new(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w);
        public static FloatW operator /(FloatW a, FloatW b) => new(a.x / b.x, a.y / b.y, a.z / b.z, a.w / b.w);
    }
    public struct Vector2W
    {
        public FloatW X, Y;
        public static Vector2W operator -(Vector2W a, Vector2W b) => new() { X = a.X - b.X, Y = a.Y - b.Y };
    }
    struct RotationW
    {
        public FloatW C, S;
    }
    static FloatW MulAddW(FloatW a, FloatW b, FloatW c) =>
        new(MathF.FusedMultiplyAdd(b.x, c.x, a.x), MathF.FusedMultiplyAdd(b.y, c.y, a.y), MathF.FusedMultiplyAdd(b.z, c.z, a.z), MathF.FusedMultiplyAdd(b.w, c.w, a.w));
    static FloatW MulSubW(FloatW a, FloatW b, FloatW c) =>
        new(MathF.FusedMultiplyAdd(-b.x, c.x, a.x), MathF.FusedMultiplyAdd(-b.y, c.y, a.y), MathF.FusedMultiplyAdd(-b.z, c.z, a.z), MathF.FusedMultiplyAdd(-b.w, c.w, a.w));
    static FloatW MinW(FloatW a, FloatW b) => new(Math.Min(a.x, b.x), Math.Min(a.y, b.y), Math.Min(a.z, b.z), Math.Min(a.w, b.w));
    static FloatW MaxW(FloatW a, FloatW b) => new(Math.Max(a.x, b.x), Math.Max(a.y, b.y), Math.Max(a.z, b.z), Math.Max(a.w, b.w));
    static FloatW SymClampW(FloatW a, FloatW b) =>
        new(Math.Clamp(a.x, -b.x, b.x), Math.Clamp(a.y, -b.y, b.y), Math.Clamp(a.z, -b.z, b.z), Math.Clamp(a.w, -b.w, b.w));
    static FloatW OrW(FloatW a, FloatW b) =>
        new(a.x != 0 || b.x != 0 ? 1 : 0, a.y != 0 || b.y != 0 ? 1 : 0, a.z != 0 || b.z != 0 ? 1 : 0, a.w != 0 || b.w != 0 ? 1 : 0);
    static FloatW AndNotW(FloatW a, FloatW b) => new(a.x != 0 && b.x == 0 ? 1 : 0, a.y != 0 && b.y == 0 ? 1 : 0, a.z != 0 && b.z == 0 ? 1 : 0, a.w != 0 && b.w == 0 ? 1 : 0);
    static FloatW GreaterThanW(FloatW a, FloatW b) => new(a.x > b.x ? 1 : 0, a.y > b.y ? 1 : 0, a.z > b.z ? 1 : 0, a.w > b.w ? 1 : 0);
    static bool AllZeroW(FloatW a) => a.x == 0 && a.y == 0 && a.z == 0 && a.w == 0;
    static unsafe FloatW LoadW(float* data) => new(data[0], data[1], data[2], data[3]);
    static FloatW BlendW(FloatW a, FloatW b, FloatW mask) =>
        new(mask.x != 0 ? b.x : a.x, mask.y != 0 ? b.y : a.y, mask.z != 0 ? b.z : a.z, mask.w != 0 ? b.w : a.w);
    static FloatW UnpackLoW(FloatW a, FloatW b) => new(a.x, b.x, a.y, b.y);
    static FloatW UnpackHiW(FloatW a, FloatW b) => new(a.z, b.z, a.w, b.w);
    static unsafe FloatW SoftMaskW(int* indexA, int* indexB) => new(indexA[0] == 0 || indexB[0] == 0 ? 1 : 0, indexA[1] == 0 || indexB[1] == 0 ? 1 : 0, indexA[2] == 0 || indexB[2] == 0 ? 1 : 0, indexA[3] == 0 || indexB[3] == 0 ? 1 : 0);
    static Vector2W RightPerpW(Vector2W a) => new() { X = a.Y, Y = -a.X };
    static FloatW DotW(Vector2W a, Vector2W b) => a.X * b.X + a.Y * b.Y;
    static FloatW CrossW(Vector2W a, Vector2W b) => a.X * b.Y - a.Y * b.X;
    static Vector2W RotateVectorW(RotationW q, Vector2W v) => new() { X = q.C * v.X - q.S * v.Y, Y = q.S * v.X + q.C * v.Y };
    public struct ContactConstraintWide
    {
        public Vector128<int> indexA, indexB;
        public FloatW invMassA, invMassB;
        public FloatW invIA, invIB;
        public Vector2W normal;
        public Vector2W anchorA1, anchorB1;
        public Vector2W anchorA2, anchorB2;
        public FloatW normalMass1, normalMass2;
        public FloatW baseSeparation1, baseSeparation2;
        public FloatW normalImpulse1, normalImpulse2;
        public FloatW totalNormalImpulse1, totalNormalImpulse2;
        public FloatW tangentImpulse1, tangentImpulse2;
        public FloatW rollingImpulse;
        public FloatW friction;
        public FloatW tangentSpeed;
        public FloatW rollingResistance;
        public FloatW tangentMass1, tangentMass2;
        public FloatW negRestitutionVelocity1, negRestitutionVelocity2;
    }
    struct BodyStateW
    {
        public Vector2W v;
        public FloatW w;
        public FloatW flags;
        public Vector2W dp;
        public RotationW dq;
    }
    unsafe BodyStateW GatherBodies(BodyState* states, int* indices)
    {
        int i1 = indices[0] - 1, i2 = indices[1] - 1, i3 = indices[2] - 1, i4 = indices[3] - 1;
        BodyState s1 = i1 == -1 ? BodyState.Identity : states[i1];
        BodyState s2 = i2 == -1 ? BodyState.Identity : states[i2];
        BodyState s3 = i3 == -1 ? BodyState.Identity : states[i3];
        BodyState s4 = i4 == -1 ? BodyState.Identity : states[i4];

        return new()
        {
            v = new()
            {
                X = new(s1.linearVelocity.x, s2.linearVelocity.x, s3.linearVelocity.x, s4.linearVelocity.x),
                Y = new(s1.linearVelocity.y, s2.linearVelocity.y, s3.linearVelocity.y, s4.linearVelocity.y)
            },
            w = new(s1.angularVelocity, s2.angularVelocity, s3.angularVelocity, s4.angularVelocity),
            flags = new((float)s1.flags, (float)s2.flags, (float)s3.flags, (float)s4.flags),
            dp = new()
            {
                X = new(s1.deltaPosition.x, s2.deltaPosition.x, s3.deltaPosition.x, s4.deltaPosition.x),
                Y = new(s1.deltaPosition.y, s2.deltaPosition.y, s3.deltaPosition.y, s4.deltaPosition.y)
            },
            dq = new()
            {
                C = new(s1.deltaRotation.c, s2.deltaRotation.c, s3.deltaRotation.c, s4.deltaRotation.c),
                S = new(s1.deltaRotation.s, s2.deltaRotation.s, s3.deltaRotation.s, s4.deltaRotation.s)
            }
        };
    }
    unsafe void ScatterBodies(BodyState* states, int* indices, ref BodyStateW simdBody)
    {
        int i1 = indices[0] - 1, i2 = indices[1] - 1, i3 = indices[2] - 1, i4 = indices[3] - 1;
        if (i1 != -1 && states[i1].flags.HasFlag(BodyFlags.Dynamic))
        {
            BodyState* state = states + i1;
            state->linearVelocity.x = simdBody.v.X.x;
            state->linearVelocity.y = simdBody.v.Y.x;
            state->angularVelocity = simdBody.w.x;
        }
        if (i2 != -1 && states[i2].flags.HasFlag(BodyFlags.Dynamic))
        {
            BodyState* state = states + i2;
            state->linearVelocity.x = simdBody.v.X.y;
            state->linearVelocity.y = simdBody.v.Y.y;
            state->angularVelocity = simdBody.w.y;
        }
        if (i3 != -1 && states[i3].flags.HasFlag(BodyFlags.Dynamic))
        {
            BodyState* state = states + i3;
            state->linearVelocity.x = simdBody.v.X.z;
            state->linearVelocity.y = simdBody.v.Y.z;
            state->angularVelocity = simdBody.w.z;
        }
        if (i4 != -1 && states[i4].flags.HasFlag(BodyFlags.Dynamic))
        {
            BodyState* state = states + i4;
            state->linearVelocity.x = simdBody.v.X.w;
            state->linearVelocity.y = simdBody.v.Y.w;
            state->angularVelocity = simdBody.w.w;
        }
    }
    [System.Runtime.CompilerServices.InlineArray(4)] struct ContactSimLanes { public ContactSim sim; }
    public unsafe void PrepareContacts_Wide(ref SolverBlock block, StepContext context)
    {
        World world = context.world;
        var spans = context.contactPrepareSpans;
        var wideBase = (ContactConstraintsFloat)context.wideContactConstraints;
        FloatW warmStartScale = world.enableWarmStarting ? FloatW.One : FloatW.Zero;
        int wideIndex = block.startIndex, endWideIndex = block.startIndex + block.count;
        int colorIndex = 0;
        while (spans[colorIndex + 1].start <= wideIndex) colorIndex++;
        while (wideIndex < endWideIndex)
        {
            int colorWideStart = spans[colorIndex].start;
            int colorWideEndIndex = Math.Min(spans[colorIndex + 1].start, endWideIndex);
            int colorContactCount = spans[colorIndex].count;
            var contactSims = spans[colorIndex].contacts;
            ContactSimLanes contactLanes = new();
            for (; wideIndex < colorWideEndIndex; wideIndex++)
            {
                var cw = wideBase.wideConstraints + wideIndex;
                int localWideIndex = wideIndex - colorWideStart;
                for (int laneIndex = 0; laneIndex < 4; laneIndex++)
                {
                    int contactIndex = 4 * localWideIndex + laneIndex;
                    if (contactIndex < colorContactCount)
                    {
                        ContactSim c = contactSims[contactIndex];
                        contactLanes[laneIndex] = c;
                        ((int*)&cw->indexA)[laneIndex] = c.bodySimIndexA + 1;
                        ((int*)&cw->indexB)[laneIndex] = c.bodySimIndexB + 1;
#if B2_VALIDATE
                        Body bodyA = world.bodies[c.bodyIdA];
                        int validIndexA = bodyA.setIndex == (int)SetType.Awake ? bodyA.localIndex : -1;
                        Body bodyB = world.bodies[c.bodyIdB];
                        int validIndexB = bodyB.setIndex == (int)SetType.Awake ? bodyB.localIndex : -1;
                        Debug.Assert(c.bodyIdA == validIndexA);
                        Debug.Assert(c.bodyIdB == validIndexB);
#endif
                    }
                    else contactLanes[laneIndex] = ContactSim.Zero;
                }
                cw->invMassA = new(contactLanes[0].invMassA, contactLanes[1].invMassA, contactLanes[2].invMassA, contactLanes[3].invMassA);
                cw->invMassB = new(contactLanes[0].invMassB, contactLanes[1].invMassB, contactLanes[2].invMassB, contactLanes[3].invMassB);
                cw->invIA = new(contactLanes[0].invIA, contactLanes[1].invIA, contactLanes[2].invIA, contactLanes[3].invIA);
                cw->invIB = new(contactLanes[0].invIB, contactLanes[1].invIB, contactLanes[2].invIB, contactLanes[3].invIB);
                cw->normal.X = new(contactLanes[0].manifold.normal.x, contactLanes[1].manifold.normal.x, contactLanes[2].manifold.normal.x, contactLanes[3].manifold.normal.x);
                cw->normal.Y = new(contactLanes[0].manifold.normal.y, contactLanes[1].manifold.normal.y, contactLanes[2].manifold.normal.y, contactLanes[3].manifold.normal.y);
                cw->friction = new(contactLanes[0].friction, contactLanes[1].friction, contactLanes[2].friction, contactLanes[3].friction);
                cw->tangentSpeed = new(contactLanes[0].tangentSpeed, contactLanes[1].tangentSpeed, contactLanes[2].tangentSpeed, contactLanes[3].tangentSpeed);
                cw->rollingResistance = new(contactLanes[0].rollingResistance, contactLanes[1].rollingResistance, contactLanes[2].rollingResistance, contactLanes[3].rollingResistance);
                cw->rollingImpulse = new(contactLanes[0].manifold.rollingImpulse, contactLanes[1].manifold.rollingImpulse, contactLanes[2].manifold.rollingImpulse, contactLanes[3].manifold.rollingImpulse);
                cw->rollingImpulse = warmStartScale * cw->rollingImpulse;
                Vector2W tangent = RightPerpW(cw->normal);
                {
                    FloatW m1a, m1b; fixed (Manifold* m = &contactLanes[0].manifold) { m1a = LoadW(&m->point0.anchorA.x); m1b = LoadW(&m->point0.anchorB.x); }
                    FloatW m2a, m2b; fixed (Manifold* m = &contactLanes[1].manifold) { m2a = LoadW(&m->point0.anchorA.x); m2b = LoadW(&m->point0.anchorB.x); }
                    FloatW m3a, m3b; fixed (Manifold* m = &contactLanes[2].manifold) { m3a = LoadW(&m->point0.anchorA.x); m3b = LoadW(&m->point0.anchorB.x); }
                    FloatW m4a, m4b; fixed (Manifold* m = &contactLanes[3].manifold) { m4a = LoadW(&m->point0.anchorA.x); m4b = LoadW(&m->point0.anchorB.x); }
                    FloatW t1a = UnpackLoW(m1a, m3a), t2a = UnpackLoW(m2a, m4a);
                    FloatW t3a = UnpackHiW(m1a, m3a), t4a = UnpackHiW(m2a, m4a);
                    FloatW t1b = UnpackLoW(m1b, m3b), t2b = UnpackLoW(m2b, m4b);
                    FloatW t3b = UnpackHiW(m1b, m3b), t4b = UnpackHiW(m2b, m4b);
                    cw->anchorA2.X = UnpackLoW(t1a, t2a);
                    cw->anchorA2.Y = UnpackHiW(t1a, t2a);
                    cw->anchorB2.X = UnpackLoW(t3a, t4a);
                    cw->anchorB2.Y = UnpackHiW(t3a, t4a);
                    cw->baseSeparation2 = UnpackLoW(t1b, t2b);
                    cw->normalImpulse2 = UnpackHiW(t1b, t2b);
                    cw->tangentImpulse2 = UnpackLoW(t3b, t4b);
                    cw->negRestitutionVelocity2 = UnpackHiW(t3b, t4b);

                    FloatW offset = DotW(cw->anchorB1 - cw->anchorA1, cw->normal);
                    cw->baseSeparation1 = cw->baseSeparation1 - offset;
                    cw->negRestitutionVelocity1 = -cw->negRestitutionVelocity1;
                    cw->normalImpulse1 = warmStartScale * cw->normalImpulse1;
                    cw->tangentImpulse1 = warmStartScale * cw->tangentImpulse1;
                    cw->totalNormalImpulse1 = FloatW.Zero;
                    {
                        FloatW rnA = CrossW(cw->anchorA1, cw->normal);
                        FloatW rnB = CrossW(cw->anchorB1, cw->normal);
                        FloatW k = cw->invMassA + cw->invMassB + cw->invIA * rnA * rnA + cw->invIB * rnB * rnB;
                        cw->normalMass1 = BlendW(FloatW.Zero, FloatW.One / k, GreaterThanW(k, FloatW.Zero));
                    }
                    {
                        FloatW rnA = CrossW(cw->anchorA1, tangent);
                        FloatW rnB = CrossW(cw->anchorB1, tangent);
                        FloatW k = cw->invMassA + cw->invMassB + cw->invIA * rnA * rnA + cw->invIB * rnB * rnB;
                        cw->tangentMass1 = BlendW(FloatW.Zero, FloatW.One / k, GreaterThanW(k, FloatW.Zero));
                    }
                }
                {
                    FloatW m1a, m1b; fixed (Manifold* m = &contactLanes[0].manifold) { m1a = LoadW(&m->point1.anchorA.x); m1b = LoadW(&m->point1.anchorB.x); }
                    FloatW m2a, m2b; fixed (Manifold* m = &contactLanes[1].manifold) { m2a = LoadW(&m->point1.anchorA.x); m2b = LoadW(&m->point1.anchorB.x); }
                    FloatW m3a, m3b; fixed (Manifold* m = &contactLanes[2].manifold) { m3a = LoadW(&m->point1.anchorA.x); m3b = LoadW(&m->point1.anchorB.x); }
                    FloatW m4a, m4b; fixed (Manifold* m = &contactLanes[3].manifold) { m4a = LoadW(&m->point1.anchorA.x); m4b = LoadW(&m->point1.anchorB.x); }
                    FloatW t1a = UnpackLoW(m1a, m3a), t2a = UnpackLoW(m2a, m4a);
                    FloatW t3a = UnpackHiW(m1a, m3a), t4a = UnpackHiW(m2a, m4a);
                    FloatW t1b = UnpackLoW(m1b, m3b), t2b = UnpackLoW(m2b, m4b);
                    FloatW t3b = UnpackHiW(m1b, m3b), t4b = UnpackHiW(m2b, m4b);
                    cw->anchorA2.X = UnpackLoW(t1a, t2a);
                    cw->anchorA2.Y = UnpackHiW(t1a, t2a);
                    cw->anchorB2.X = UnpackLoW(t3a, t4a);
                    cw->anchorB2.Y = UnpackHiW(t3a, t4a);
                    cw->baseSeparation2 = UnpackLoW(t1b, t2b);
                    cw->normalImpulse2 = UnpackHiW(t1b, t2b);
                    cw->tangentImpulse2 = UnpackLoW(t3b, t4b);
                    cw->negRestitutionVelocity2 = UnpackHiW(t3b, t4b);

                    FloatW offset = DotW(cw->anchorB2 - cw->anchorA2, cw->normal);
                    cw->baseSeparation2 = cw->baseSeparation2 - offset;
                    cw->negRestitutionVelocity2 = -cw->negRestitutionVelocity2;
                    cw->normalImpulse2 = warmStartScale * cw->normalImpulse2;
                    cw->tangentImpulse2 = warmStartScale * cw->tangentImpulse2;
                    cw->totalNormalImpulse2 = FloatW.Zero;
                    {
                        FloatW rnA = CrossW(cw->anchorA2, cw->normal);
                        FloatW rnB = CrossW(cw->anchorB2, cw->normal);
                        FloatW k = cw->invMassA + cw->invMassB + cw->invIA * rnA * rnA + cw->invIB * rnB * rnB;
                        cw->normalMass2 = BlendW(FloatW.Zero, FloatW.One / k, GreaterThanW(k, FloatW.Zero));
                    }
                    {
                        FloatW rnA = CrossW(cw->anchorA2, tangent);
                        FloatW rnB = CrossW(cw->anchorB2, tangent);
                        FloatW k = cw->invMassA + cw->invMassB + cw->invIA * rnA * rnA + cw->invIB * rnB * rnB;
                        cw->tangentMass2 = BlendW(FloatW.Zero, FloatW.One / k, GreaterThanW(k, FloatW.Zero));
                    }
                }
                FloatW massScale = GreaterThanW(new(contactLanes[0].manifold.pointCount, contactLanes[1].manifold.pointCount, contactLanes[2].manifold.pointCount, contactLanes[3].manifold.pointCount), FloatW.One);
                cw->normalMass2 = BlendW(FloatW.Zero, cw->normalMass2, massScale);
                cw->tangentMass2 = BlendW(FloatW.Zero, cw->tangentMass2, massScale);
            }
            colorIndex++;
        }
    }
    public unsafe void WarmStartContacts_Wide(ref SolverBlock block, StepContext context)
    {
        var states = context.states.Data;
        var constraints = ((ContactConstraintsFloat)context.graph.colors[block.colorIndex].wideConstraints).wideConstraints;
        {
            for (int wideIndex = block.startIndex; wideIndex < block.startIndex + block.count; wideIndex++)
            {
                ContactConstraintWide* c = constraints + wideIndex;
                BodyStateW bA = GatherBodies(states, (int*)&c->indexA);
                BodyStateW bB = GatherBodies(states, (int*)&c->indexB);
                FloatW tangentX = c->normal.Y;
                FloatW tangentY = FloatW.Zero - c->normal.X;
                {
                    Vector2W rA = c->anchorA1, rB = c->anchorB1;
                    Vector2W P = new()
                    {
                        X = c->normalImpulse1 * c->normal.X + c->tangentImpulse1 * tangentX,
                        Y = c->normalImpulse1 * c->normal.Y + c->tangentImpulse1 * tangentY
                    };
                    bA.w = MulSubW(bA.w, c->invIA, CrossW(rA, P));
                    bA.v = new()
                    {
                        X = MulSubW(bA.v.X, c->invMassA, P.X),
                        Y = MulSubW(bA.v.Y, c->invMassA, P.Y)
                    };
                    bB.w = MulAddW(bB.w, c->invIB, CrossW(rB, P));
                    bB.v = new()
                    {
                        X = MulAddW(bB.v.X, c->invMassB, P.X),
                        Y = MulAddW(bB.v.Y, c->invMassB, P.Y)
                    };
                    c->totalNormalImpulse1 = c->totalNormalImpulse1 + c->normalImpulse1;
                }
                {
                    Vector2W rA = c->anchorA2, rB = c->anchorB2;
                    Vector2W P = new()
                    {
                        X = c->normalImpulse2 * c->normal.X + c->tangentImpulse2 * tangentX,
                        Y = c->normalImpulse2 * c->normal.Y + c->tangentImpulse2 * tangentY
                    };
                    bA.w = MulSubW(bA.w, c->invIA, CrossW(rA, P));
                    bA.v = new()
                    {
                        X = MulSubW(bA.v.X, c->invMassA, P.X),
                        Y = MulSubW(bA.v.Y, c->invMassA, P.Y)
                    };
                    bB.w = MulAddW(bB.w, c->invIB, CrossW(rB, P));
                    bB.v = new()
                    {
                        X = MulAddW(bB.v.X, c->invMassB, P.X),
                        Y = MulAddW(bB.v.Y, c->invMassB, P.Y)
                    };
                    c->totalNormalImpulse2 = c->totalNormalImpulse2 + c->normalImpulse2;
                }
                bA.w = MulSubW(bA.w, c->invIA, c->rollingImpulse);
                bB.w = MulAddW(bB.w, c->invIB, c->rollingImpulse);
                ScatterBodies(states, (int*)&c->indexA, ref bA);
                ScatterBodies(states, (int*)&c->indexB, ref bB);
            }
        }
    }
    public unsafe void PushContacts_Wide(ref SolverBlock block, StepContext context)
    {
        var states = context.states.Data;
        var constraints = ((ContactConstraintsFloat)context.graph.colors[block.colorIndex].wideConstraints).wideConstraints;
        {
            FloatW inv_h = new(context.inv_h);
            FloatW contactSpeed = new(-context.world.contactSpeed);
            FloatW oneW = new(1);
            FloatW dynamicBiasRate = new(context.contactSoftness.massScale * context.contactSoftness.biasRate);
            FloatW dynamicMassScale = new(context.contactSoftness.massScale);
            FloatW dynamicImpulseScale = new(context.contactSoftness.impulseScale);
            FloatW staticBiasRate = new(context.staticSoftness.massScale * context.staticSoftness.biasRate);
            FloatW staticMassScale = new(context.staticSoftness.massScale);
            FloatW staticImpulseScale = new(context.staticSoftness.impulseScale);
            for (int wideIndex = block.startIndex; wideIndex < block.startIndex + block.count; wideIndex++)
            {
                ContactConstraintWide* c = constraints + wideIndex;
                BodyStateW bA = GatherBodies(states, (int*)&c->indexA);
                BodyStateW bB = GatherBodies(states, (int*)&c->indexB);
                FloatW softMask = SoftMaskW((int*)&c->indexA, (int*)&c->indexB),
                    biasRate = BlendW(dynamicBiasRate, staticBiasRate, softMask),
                    massScale = BlendW(dynamicMassScale, staticMassScale, softMask),
                    impulseScale = BlendW(dynamicImpulseScale, staticImpulseScale, softMask);
                Vector2W dp = new() { X = bB.dp.X - bA.dp.X, Y = bB.dp.Y - bA.dp.Y };
                {
                    Vector2W rA = c->anchorA1, rB = c->anchorB1;
                    Vector2W rsA = RotateVectorW(bA.dq, rA), rsB = RotateVectorW(bB.dq, rB);
                    Vector2W ds = new() { X = dp.X + (rsB.X - rsA.X), Y = dp.Y + (rsB.Y - rsA.Y) };
                    FloatW s = DotW(c->normal, ds) + c->baseSeparation1;
                    FloatW separated = GreaterThanW(s, FloatW.Zero);
                    FloatW specBias = s * inv_h, overlapBias = MaxW(biasRate * s, contactSpeed);
                    FloatW velocityBias = BlendW(overlapBias, specBias, separated);
                    FloatW pointMassScale = BlendW(massScale, oneW, separated);
                    FloatW pointImpulseScale = BlendW(impulseScale, FloatW.Zero, separated);
                    FloatW dvx = (bB.v.X - bB.w * rB.Y) - (bA.v.X - bA.w * rA.Y);
                    FloatW dvy = (bB.v.Y + bB.w * rB.X) - (bA.v.Y + bA.w * rA.X);
                    FloatW vn = dvx * c->normal.X + dvy * c->normal.Y;
                    FloatW negImpulse = c->normalMass1 * (pointMassScale * vn + velocityBias) + pointImpulseScale * c->normalImpulse1;
                    FloatW newImpulse = MaxW(c->normalImpulse1 - negImpulse, FloatW.Zero);
                    FloatW impulse = newImpulse - c->normalImpulse1;
                    c->normalImpulse1 = newImpulse;
                    c->totalNormalImpulse1 = c->totalNormalImpulse1 + impulse;
                    FloatW Px = impulse * c->normal.X;
                    FloatW Py = impulse * c->normal.Y;
                    bA.v = new()
                    {
                        X = MulSubW(bA.v.X, c->invMassA, Px),
                        Y = MulSubW(bA.v.Y, c->invMassA, Py)
                    };
                    bA.w = MulSubW(bA.w, c->invIA, rA.X * Py - rA.Y * Px);
                    bB.v = new()
                    {
                        X = MulAddW(bB.v.X, c->invMassB, Px),
                        Y = MulAddW(bB.v.Y, c->invMassB, Py)
                    };
                    bB.w = MulAddW(bB.w, c->invIB, rB.X * Py - rB.Y * Px);
                }
                {
                    Vector2W rA = c->anchorA2, rB = c->anchorB2;
                    Vector2W rsA = RotateVectorW(bA.dq, rA), rsB = RotateVectorW(bB.dq, rB);
                    Vector2W ds = new() { X = dp.X + (rsB.X - rsA.X), Y = dp.Y + (rsB.Y - rsA.Y) };
                    FloatW s = DotW(c->normal, ds) + c->baseSeparation2;
                    FloatW separated = GreaterThanW(s, FloatW.Zero);
                    FloatW specBias = s * inv_h, overlapBias = MaxW(biasRate * s, contactSpeed);
                    FloatW velocityBias = BlendW(overlapBias, specBias, separated);
                    FloatW pointMassScale = BlendW(massScale, oneW, separated);
                    FloatW pointImpulseScale = BlendW(impulseScale, FloatW.Zero, separated);
                    FloatW dvx = (bB.v.X - bB.w * rB.Y) - (bA.v.X - bA.w * rA.Y);
                    FloatW dvy = (bB.v.Y + bB.w * rB.X) - (bA.v.Y + bA.w * rA.X);
                    FloatW vn = dvx * c->normal.X + dvy * c->normal.Y;
                    FloatW negImpulse = c->normalMass2 * (pointMassScale * vn + velocityBias) + pointImpulseScale * c->normalImpulse2;
                    FloatW newImpulse = MaxW(c->normalImpulse2 - negImpulse, FloatW.Zero);
                    FloatW impulse = newImpulse - c->normalImpulse2;
                    c->normalImpulse2 = newImpulse;
                    c->totalNormalImpulse2 = c->totalNormalImpulse2 + impulse;
                    FloatW Px = impulse * c->normal.X;
                    FloatW Py = impulse * c->normal.Y;
                    bA.v = new()
                    {
                        X = MulSubW(bA.v.X, c->invMassA, Px),
                        Y = MulSubW(bA.v.Y, c->invMassA, Py)
                    };
                    bA.w = MulSubW(bA.w, c->invIA, rA.X * Py - rA.Y * Px);
                    bB.v = new()
                    {
                        X = MulAddW(bB.v.X, c->invMassB, Px),
                        Y = MulAddW(bB.v.Y, c->invMassB, Py)
                    };
                    bB.w = MulAddW(bB.w, c->invIB, rB.X * Py - rB.Y * Px);
                }
                ScatterBodies(states, (int*)&c->indexA, ref bA);
                ScatterBodies(states, (int*)&c->indexB, ref bB);
            }
        }
    }
    public unsafe void SolveContacts_Wide(ref SolverBlock block, StepContext context)
    {
        var states = context.states.Data;
        GraphColor color = context.graph.colors[block.colorIndex];
        var constraints = ((ContactConstraintsFloat)context.graph.colors[block.colorIndex].wideConstraints).wideConstraints;
        FloatW inv_h = new(context.inv_h);
        for (int wideIndex = 0; wideIndex < block.startIndex + block.count; wideIndex++)
        {
            ContactConstraintWide* c = constraints + wideIndex;
            BodyStateW bA = GatherBodies(states, (int*)&c->indexA);
            BodyStateW bB = GatherBodies(states, (int*)&c->indexB);
            FloatW resitutionMask1 = GreaterThanW(FloatW.Zero, c->negRestitutionVelocity1);
            FloatW resitutionMask2 = GreaterThanW(FloatW.Zero, c->negRestitutionVelocity2);
            bool haveResitution = !AllZeroW(OrW(resitutionMask1, resitutionMask2));
            FloatW keepRestitution = FloatW.Zero;
            FloatW totalNormalImpulse = FloatW.Zero;
            Vector2W dp = bB.dp - bA.dp;
            {
                Vector2W rA = c->anchorA1, rB = c->anchorB1;
                Vector2W rsA = RotateVectorW(bA.dq, rA), rsB = RotateVectorW(bB.dq, rB);
                Vector2W ds = new() { X = dp.X + (rsB.X - rsA.X), Y = dp.Y + (rsB.Y - rsA.Y) };
                FloatW s = DotW(c->normal, ds) + c->baseSeparation1;
                FloatW specBias = s * inv_h;
                FloatW velocityBias = MaxW(FloatW.Zero, specBias);
                if (haveResitution)
                {
                    FloatW separated = GreaterThanW(s, FloatW.Zero);
                    velocityBias = BlendW(velocityBias, c->negRestitutionVelocity1, resitutionMask1);
                    keepRestitution = OrW(keepRestitution, AndNotW(resitutionMask1, separated));
                }
                FloatW dvx = (bB.v.X - bB.w * rB.Y) - (bA.v.X - bA.w * rA.Y);
                FloatW dvy = (bB.v.Y + bB.w * rB.X) - (bA.v.Y + bA.w * rA.X);
                FloatW vn = dvx * c->normal.X + dvy * c->normal.Y;
                FloatW newImpulse = MaxW(c->normalImpulse1 - c->normalMass1 * (vn + velocityBias), FloatW.Zero);
                FloatW impulse = newImpulse - c->normalImpulse1;
                c->normalImpulse1 = newImpulse;
                c->totalNormalImpulse1 = c->totalNormalImpulse1 + impulse;
                totalNormalImpulse = totalNormalImpulse + newImpulse;
                FloatW Px = impulse * c->normal.X, Py = impulse * c->normal.Y;
                bA.v = new()
                {
                    X = bA.v.X - c->invMassA * Px,
                    Y = bA.v.Y - c->invMassA * Py
                };
                bA.w = bA.w - c->invIA * (rA.X * Py - rA.Y * Px);
                bB.v = new()
                {
                    X = bB.v.X + c->invMassB * Px,
                    Y = bB.v.Y + c->invMassB * Py
                };
                bB.w = bB.w + c->invIB * (rB.X * Py - rB.Y * Px);
            }
            {
                Vector2W rA = c->anchorA2, rB = c->anchorB2;
                Vector2W rsA = RotateVectorW(bA.dq, rA), rsB = RotateVectorW(bB.dq, rB);
                Vector2W ds = new() { X = dp.X + (rsB.X - rsA.X), Y = dp.Y + (rsB.Y - rsA.Y) };
                FloatW s = DotW(c->normal, ds) + c->baseSeparation2;
                FloatW specBias = s * inv_h;
                FloatW velocityBias = MaxW(FloatW.Zero, specBias);
                if (haveResitution)
                {
                    FloatW separated = GreaterThanW(s, FloatW.Zero);
                    velocityBias = BlendW(velocityBias, c->negRestitutionVelocity2, resitutionMask2);
                    keepRestitution = OrW(keepRestitution, AndNotW(resitutionMask2, separated));
                }
                FloatW dvx = (bB.v.X - bB.w * rB.Y) - (bA.v.X - bA.w * rA.Y);
                FloatW dvy = (bB.v.Y + bB.w * rB.X) - (bA.v.Y + bA.w * rA.X);
                FloatW vn = dvx * c->normal.X + dvy * c->normal.Y;
                FloatW newImpulse = MaxW(c->normalImpulse2 - c->normalMass2 * (vn + velocityBias), FloatW.Zero);
                FloatW impulse = newImpulse - c->normalImpulse2;
                c->normalImpulse2 = newImpulse;
                c->totalNormalImpulse2 = c->totalNormalImpulse2 + impulse;
                totalNormalImpulse = totalNormalImpulse + newImpulse;
                FloatW Px = impulse * c->normal.X, Py = impulse * c->normal.Y;
                bA.v = new()
                {
                    X = bA.v.X - c->invMassA * Px,
                    Y = bA.v.Y - c->invMassA * Py
                };
                bA.w = bA.w - c->invIA * (rA.X * Py - rA.Y * Px);
                bB.v = new()
                {
                    X = bB.v.X + c->invMassB * Px,
                    Y = bB.v.Y + c->invMassB * Py
                };
                bB.w = bB.w + c->invIB * (rB.X * Py - rB.Y * Px);
            }
            c->negRestitutionVelocity1 = BlendW(FloatW.Zero, c->negRestitutionVelocity1, keepRestitution);
            c->negRestitutionVelocity2 = BlendW(FloatW.Zero, c->negRestitutionVelocity2, keepRestitution);
            if (!AllZeroW(c->rollingResistance))
            {
                FloatW k = c->invIA + c->invIB;
                FloatW deltaLambda = BlendW(FloatW.Zero, (bA.w - bB.w) / k, GreaterThanW(k, FloatW.Zero));
                FloatW lambda = c->rollingImpulse;
                FloatW maxLambda = c->rollingResistance * totalNormalImpulse;
                c->rollingImpulse = SymClampW(lambda + deltaLambda, maxLambda);
                deltaLambda = c->rollingImpulse - lambda;
                bA.w = bA.w - c->invIA * deltaLambda;
                bB.w = bB.w + c->invIB * deltaLambda;
            }
            FloatW tangentX = c->normal.Y;
            FloatW tangentY = -c->normal.X;
            {
                Vector2W rA = c->anchorA1, rB = c->anchorB1;
                FloatW dvx = (bB.v.X - bB.w * rB.Y) - (bA.v.X - bA.w * rA.Y);
                FloatW dvy = (bB.v.Y + bB.w * rB.X) - (bA.v.Y + bA.w * rA.X);
                FloatW vt = dvx * tangentX + dvy * tangentY;
                vt = vt - c->tangentSpeed;
                FloatW negImpulse = c->tangentMass1 * vt;
                FloatW maxFriction = c->friction * c->normalImpulse1;
                FloatW newImpulse = c->tangentImpulse1 - negImpulse;
                //no symclamp?
                newImpulse = MaxW(-maxFriction, MinW(newImpulse, maxFriction));
                FloatW impulse = newImpulse - c->tangentImpulse1;
                c->tangentImpulse1 = newImpulse;
                FloatW Px = impulse * tangentX, Py = impulse * tangentY;
                bA.v = new()
                {
                    X = bA.v.X - c->invMassA * Px,
                    Y = bA.v.Y - c->invMassA * Py
                };
                bA.w = bA.w - c->invIA * (rA.X * Py - rA.Y * Px);
                bB.v = new()
                {
                    X = bB.v.X + c->invMassB * Px,
                    Y = bB.v.Y + c->invMassB * Py
                };
                bB.w = bB.w + c->invIB * (rB.X * Py - rB.Y * Px);
            }
            {
                Vector2W rA = c->anchorA2, rB = c->anchorB2;
                FloatW dvx = (bB.v.X - bB.w * rB.Y) - (bA.v.X - bA.w * rA.Y);
                FloatW dvy = (bB.v.Y + bB.w * rB.X) - (bA.v.Y + bA.w * rA.X);
                FloatW vt = dvx * tangentX + dvy * tangentY;
                vt = vt - c->tangentSpeed;
                FloatW negImpulse = c->tangentMass2 * vt;
                FloatW maxFriction = c->friction * c->normalImpulse2;
                FloatW newImpulse = c->tangentImpulse2 - negImpulse;
                //no symclamp?
                newImpulse = MaxW(-maxFriction, MinW(newImpulse, maxFriction));
                FloatW impulse = newImpulse - c->tangentImpulse2;
                c->tangentImpulse2 = newImpulse;
                FloatW Px = impulse * tangentX, Py = impulse * tangentY;
                bA.v = new()
                {
                    X = bA.v.X - c->invMassA * Px,
                    Y = bA.v.Y - c->invMassA * Py
                };
                bA.w = bA.w - c->invIA * (rA.X * Py - rA.Y * Px);
                bB.v = new()
                {
                    X = bB.v.X + c->invMassB * Px,
                    Y = bB.v.Y + c->invMassB * Py
                };
                bB.w = bB.w + c->invIB * (rB.X * Py - rB.Y * Px);
            }
            ScatterBodies(states, (int*)&c->indexA, ref bA);
            ScatterBodies(states, (int*)&c->indexB, ref bB);
        }
    }
    public unsafe void StoreImpulses_Wide(ref SolverBlock block, StepContext context, int workerIndex)
    {
        var spans = context.contactPrepareSpans;
        var wideBase = ((ContactConstraintsFloat)context.wideContactConstraints).wideConstraints;
        TaskContext taskContext = context.world.taskContexts[workerIndex];
        BitSet hitEventBitSet = taskContext.hitEventBitSet;
        bool hasHitEvents = taskContext.hasHitEvents;
        float negHitThreshold = -context.world.hitEventThreshold;
        int wideIndex = block.startIndex;
        int endWideIndex = block.startIndex + block.count;
        int colorIndex = 0;
        while (spans[colorIndex + 1].start <= wideIndex) colorIndex++;
        while (wideIndex < endWideIndex)
        {
            int colorWideEndIndex = Math.Min(spans[colorIndex + 1].start, endWideIndex);
            int colorWideStart = spans[colorIndex].start;
            int colorContactCount = spans[colorIndex].count;
            var contactSims = spans[colorIndex].contacts;
            for (; wideIndex < colorWideEndIndex; wideIndex++)
            {
                ContactConstraintWide* c = wideBase + wideIndex;
                float* rollingImpulse = (float*)&c->rollingImpulse;
                float* normalImpulse1 = (float*)&c->normalImpulse1;
                float* normalImpulse2 = (float*)&c->normalImpulse2;
                float* tangentImpulse1 = (float*)&c->tangentImpulse1;
                float* tangentImpulse2 = (float*)&c->tangentImpulse2;
                float* totalNormalImpulse1 = (float*)&c->totalNormalImpulse1;
                float* totalNormalImpulse2 = (float*)&c->totalNormalImpulse2;
                int localWideIndex = wideIndex - colorWideStart;
                int baseIndex = 8 * localWideIndex;
                for (int laneIndex = 0; laneIndex < 4; ++laneIndex)
                {
                    int contactIndex = baseIndex + laneIndex;
                    if (contactIndex >= colorContactCount) break;
                    ContactSim contactSim = contactSims[contactIndex];
                    ref Manifold m = ref contactSim.manifold;
                    m.rollingImpulse = rollingImpulse[laneIndex];
                    m.point0.normalImpulse = normalImpulse1[laneIndex];
                    m.point0.tangentImpulse = tangentImpulse1[laneIndex];
                    m.point0.totalNormalImpulse = totalNormalImpulse1[laneIndex];
                    m.point1.normalImpulse = normalImpulse2[laneIndex];
                    m.point1.tangentImpulse = tangentImpulse2[laneIndex];
                    m.point1.totalNormalImpulse = totalNormalImpulse2[laneIndex];
                    if (contactSim.simFlags.HasFlag(ContactFlags.SimEnableHitEvent))
                    {
                        if (contactSim.manifold.pointCount > 0 && m.point0.normalVelocity < negHitThreshold && m.point0.totalNormalImpulse > 0)
                        {
                            hitEventBitSet.SetBit(contactSim.contactId);
                            hasHitEvents = true;
                            break;
                        }
                        if (contactSim.manifold.pointCount > 1 && m.point1.normalVelocity < negHitThreshold && m.point1.totalNormalImpulse > 0)
                        {
                            hitEventBitSet.SetBit(contactSim.contactId);
                            hasHitEvents = true;
                            break;
                        }
                    }
                }
            }
            colorIndex++;
        }
    }
}