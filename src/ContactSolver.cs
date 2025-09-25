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
    public float relativeVelocity;
    public float normalImpulse;
    public float tangentImpulse;
    public float totalNormalImpulse;
    public float normalMass;
    public float tangentMass;
}

public struct ContactConstraint
{
    public int indexA;
    public int indexB;
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
    public void PrepareOverflowContacts()
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
            constraint.indexA = indexA;
            constraint.indexB = indexB;
            constraint.normal = manifold.normal;
            constraint.friction = contactSim.friction;
            constraint.restitution = contactSim.restitution;
            constraint.rollingResistance = contactSim.rollingResistance;
            constraint.rollingImpulse = warmStartScale * manifold.rollingImpulse;
            constraint.tangentSpeed = contactSim.tangentSpeed;
            constraint.pointCount = pointCount;
            Vector2 vA = Vector2.Zero;
            float wA = 0, mA = contactSim.invMassA, iA = contactSim.invIA;
            if (indexA != -1)
            {
                BodyState* stateA = states.Data + indexA;
                vA = stateA->linearVelocity;
                wA = stateA->angularVelocity;
            }
            Vector2 vB = Vector2.Zero;
            float wB = 0, mB = contactSim.invMassB, iB = contactSim.invIB;
            if (indexB != -1)
            {
                BodyState* stateB = states.Data + indexB;
                vB = stateB->linearVelocity;
                wB = stateB->angularVelocity;
            }
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
                Vector2 vrA = vA + Vector2.CrossSV(wA, rA);
                Vector2 vrB = vB + Vector2.CrossSV(wB, rB);
                cp.relativeVelocity = Vector2.Dot(normal, vrB - vrA);
            }
        }
    }
    public void WarmStartOverflowContacts()
    {
        ref GraphColor color = ref graph.colors[Box2D.GraphColorCount - 1];
        var constraints = color.overflowConstraints;
        int contactCount = color.contactSims.Count;
        SolverSet awakeSet = world.solverSets[(int)SetType.Awake];
        var states = awakeSet.bodyStates;
        BodyState dummyState = new();
        for (int i = 0; i < contactCount; i++)
        {
            ref ContactConstraint constraint = ref constraints[i];
            int indexA = constraint.indexA, indexB = constraint.indexB;
            BodyState* stateA = &dummyState; if (indexA != -1) stateA = states.Data + indexA;
            BodyState* stateB = &dummyState; if (indexB != -1) stateB = states.Data + indexB;
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
    public void SolveOverflowContacts(bool useBias)
    {
        GraphColor color = graph.colors[Box2D.GraphColorCount - 1];
        var constraints = color.overflowConstraints;
        int contactCount = color.contactSims.Count;
        SolverSet awakeSet = world.solverSets[(int)SetType.Awake];
        var states = awakeSet.bodyStates;
        float contactSpeed = world.contactSpeed;
        BodyState dummyState = new();
        for (int i = 0; i < contactCount; i++)
        {
            ref ContactConstraint constraint = ref constraints[i];
            float mA = constraint.invMassA;
            float iA = constraint.invIA;
            float mB = constraint.invMassB;
            float iB = constraint.invIB;
            BodyState* stateA = &dummyState; if (constraint.indexA != -1) stateA = states.Data + constraint.indexA;
            Vector2 vA = stateA->linearVelocity;
            float wA = stateA->angularVelocity;
            Rotation dqA = stateA->deltaRotation;
            BodyState* stateB = &dummyState; if (constraint.indexB != -1) stateB = states.Data + constraint.indexA;
            Vector2 vB = stateB->linearVelocity;
            float wB = stateB->angularVelocity;
            Rotation dqB = stateB->deltaRotation;
            Vector2 dp = stateB->deltaPosition - stateA->deltaPosition;
            Vector2 normal = constraint.normal, tangent = normal.RightPerp();
            float friction = constraint.friction;
            Softness softness = constraint.softness;
            int pointCount = constraint.pointCount;
            float totalNormalImpulse = 0;
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
                float impulse = -cp.normalMass * (massScale * vn + velocityBias) - impulseScale * cp.normalImpulse;
                float newImpulse = Math.Max(cp.normalImpulse + impulse, 0);
                impulse = newImpulse - cp.normalImpulse;
                cp.normalImpulse = newImpulse;
                cp.totalNormalImpulse += newImpulse;
                totalNormalImpulse += newImpulse;
                Vector2 P = impulse * normal;
                vA = Vector2.MulSub(vA, mA, P);
                wA -= iA * Vector2.Cross(rA, P);
                vB = Vector2.MulAdd(vB, mB, P);
                wB += iB * Vector2.Cross(rB, P);
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
            {
                float deltaLambda = -constraint.rollingMass * (wB - wA);
                float lambda = constraint.rollingImpulse;
                float maxLambda = constraint.rollingResistance * totalNormalImpulse;
                constraint.rollingImpulse = Math.Clamp(lambda + deltaLambda, -maxLambda, maxLambda);
                deltaLambda = constraint.rollingImpulse - lambda;
                wA -= iA * deltaLambda;
                wB += iB * deltaLambda;
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
    public void ApplyOverflowRestitution()
    {
        GraphColor color = graph.colors[Box2D.GraphColorCount - 1];
        var constraints = color.overflowConstraints;
        int contactCount = color.contactSims.Count;
        SolverSet awakeSet = world.solverSets[(int)SetType.Awake];
        var states = awakeSet.bodyStates;
        float threshold = world.restitutionThreshold;
        BodyState dummyState = new();
        for (int i = 0; i < contactCount; i++)
        {
            ref ContactConstraint constraint = ref constraints[i];
            float restitution = constraint.restitution;
            if (restitution == 0) continue;
            float mA = constraint.invMassA;
            float iA = constraint.invIA;
            float mB = constraint.invMassB;
            float iB = constraint.invIB;
            BodyState* stateA = &dummyState; if (constraint.indexA != -1) stateA = states.Data + constraint.indexA;
            Vector2 vA = stateA->linearVelocity;
            float wA = stateA->angularVelocity;
            BodyState* stateB = &dummyState; if (constraint.indexB != -1) stateB = states.Data + constraint.indexA;
            Vector2 vB = stateB->linearVelocity;
            float wB = stateB->angularVelocity;
            Vector2 normal = constraint.normal;
            int pointCount = constraint.pointCount;
            for (int j = 0; j < pointCount; j++)
            {
                ref ContactConstraintPoint cp = ref constraint.point0;
                if (j == 1) cp = ref constraint.point1;
                if (cp.relativeVelocity > -threshold || cp.totalNormalImpulse == 0) continue;
                Vector2 rA = cp.anchorA, rB = cp.anchorB;
                Vector2 vrB = vB + Vector2.CrossSV(wB, rB);
                Vector2 vrA = vA + Vector2.CrossSV(wA, rA);
                float vn = Vector2.Dot(vrB - vrA, normal);
                float impulse = -cp.normalMass * (vn + restitution * cp.relativeVelocity);
                float newImpulse = Math.Max(-cp.normalImpulse + impulse, 0);
                impulse = newImpulse - cp.normalImpulse;
                cp.normalImpulse = newImpulse;
                cp.totalNormalImpulse += impulse;
                Vector2 P = impulse * normal;
                vA = Vector2.MulSub(vA, mA, P);
                wA -= iA * Vector2.Cross(rA, P);
                vB = Vector2.MulAdd(vB, mB, P);
                wB += iB * Vector2.Cross(rB, P);
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
    public void StoreOverflowImpulses()
    {
        GraphColor color = graph.colors[Box2D.GraphColorCount - 1];
        var constraints = color.overflowConstraints;
        int contactCount = color.contactSims.Count;
        for (int i = 0; i < contactCount; i++)
        {
            ref ContactConstraint constraint = ref constraints[i];
            ContactSim contact = color.contactSims[i];
            ref Manifold manifold = ref contact.manifold;
            int pointCount = manifold.pointCount;
            if (pointCount > 0)
            {
                manifold.point0.normalImpulse = constraint.point0.normalImpulse;
                manifold.point0.tangentImpulse = constraint.point0.tangentImpulse;
                manifold.point0.totalNormalImpulse = constraint.point0.totalNormalImpulse;
                manifold.point0.normalVelocity = constraint.point0.relativeVelocity;
            }
            if (pointCount > 1)
            {
                manifold.point1.normalImpulse = constraint.point1.normalImpulse;
                manifold.point1.tangentImpulse = constraint.point1.tangentImpulse;
                manifold.point1.totalNormalImpulse = constraint.point1.totalNormalImpulse;
                manifold.point1.normalVelocity = constraint.point1.relativeVelocity;
            }
            manifold.rollingImpulse = constraint.rollingImpulse;
        }
    }
}
public interface IContactSolverW
{
    public static IContactSolverW Instance() => Avx.IsSupported ? new ContactSolverAVX() :
        AdvSimd.IsSupported ? new ContactSolverNeon() : Sse.IsSupported ? new ContactSolverSSE() : new ContactSolverFloat();
    public void PrepareContactsTask(int startIndex, int endIndex, StepContext context);
    public void WarmStartContactsTask(int startIndex, int endIndex, StepContext context, int colorIndex);
    public void SolveContactsTask(int startIndex, int endIndex, StepContext context, int colorIndex, bool useBias);
    public void ApplyRestitutionTask(int startIndex, int endIndex, StepContext context, int colorIndex);
    public void StoreImpulsesTask(int startIndex, int endIndex, StepContext context);
}
public class ContactSolverAVX : IContactSolverW
{
    public struct Vector2W
    {
        public Vector256<float> X, Y;
    }
    struct RotationW
    {
        public Vector256<float> C, S;
    }
    static Vector256<float> SymClampW(Vector256<float> a, Vector256<float> b) => Avx.Max(Avx.Subtract(Vector256<float>.Zero, b), Avx.Min(a, b));
    static bool AllZeroW(Vector256<float> a) => Avx.MoveMask(Avx.CompareEqual(a, Vector256<float>.Zero)) == 0xFF;
    static Vector256<float> DotW(Vector2W a, Vector2W b) => Avx.Add(Avx.Multiply(a.X, b.X), Avx.Multiply(a.Y, b.Y));
    static Vector256<float> CrossW(Vector2W a, Vector2W b) => Avx.Subtract(Avx.Multiply(a.X, b.Y), Avx.Multiply(a.Y, b.X));
    static Vector2W RotateVectorW(RotationW q, Vector2W v) =>
        new() { X = Avx.Subtract(Avx.Multiply(q.C, v.X), Avx.Multiply(q.S, v.Y)), Y = Avx.Add(Avx.Multiply(q.S, v.X), Avx.Multiply(q.C, v.Y)) };
    public struct ContactConstraintSIMD
    {
        public Vector256<int> indexA, indexB;
        public Vector256<float> invMassA, invMassB;
        public Vector256<float> invIA, invIB;
        public Vector2W normal;
        public Vector256<float> friction;
        public Vector256<float> tangentSpeed;
        public Vector256<float> rollingResistance;
        public Vector256<float> rollingMass;
        public Vector256<float> rollingImpulse;
        public Vector256<float> biasRate;
        public Vector256<float> massScale;
        public Vector256<float> impulseScale;
        public Vector2W anchorA1, anchorB1;
        public Vector256<float> normalMass1, tangentMass1;
        public Vector256<float> baseSeparation1;
        public Vector256<float> normalImpulse1;
        public Vector256<float> totalNormalImpulse1;
        public Vector256<float> tangentImpulse1;
        public Vector2W anchorA2, anchorB2;
        public Vector256<float> baseSeparation2;
        public Vector256<float> normalImpulse2;
        public Vector256<float> totalNormalImpulse2;
        public Vector256<float> tangentImpulse2;
        public Vector256<float> normalMass2, tangentMass2;
        public Vector256<float> restitution;
        public Vector256<float> relativeVelocity1, relativeVelocity2;
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
        Vector256<float> b0 = indices[0] == -1 ? identity : Avx.LoadAlignedVector256((float*)(states + indices[0]));
        Vector256<float> b1 = indices[1] == -1 ? identity : Avx.LoadAlignedVector256((float*)(states + indices[1]));
        Vector256<float> b2 = indices[2] == -1 ? identity : Avx.LoadAlignedVector256((float*)(states + indices[2]));
        Vector256<float> b3 = indices[3] == -1 ? identity : Avx.LoadAlignedVector256((float*)(states + indices[3]));
        Vector256<float> b4 = indices[4] == -1 ? identity : Avx.LoadAlignedVector256((float*)(states + indices[4]));
        Vector256<float> b5 = indices[5] == -1 ? identity : Avx.LoadAlignedVector256((float*)(states + indices[5]));
        Vector256<float> b6 = indices[6] == -1 ? identity : Avx.LoadAlignedVector256((float*)(states + indices[6]));
        Vector256<float> b7 = indices[7] == -1 ? identity : Avx.LoadAlignedVector256((float*)(states + indices[7]));
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
        if (indices[0] != -1 && states[indices[0]].flags.HasFlag(BodyFlags.Dynamic)) Avx.StoreAligned((float*)(states + indices[0]), Avx.Permute2x128(tt0, tt4, 0x20));
        if (indices[1] != -1 && states[indices[1]].flags.HasFlag(BodyFlags.Dynamic)) Avx.StoreAligned((float*)(states + indices[1]), Avx.Permute2x128(tt1, tt5, 0x20));
        if (indices[2] != -1 && states[indices[2]].flags.HasFlag(BodyFlags.Dynamic)) Avx.StoreAligned((float*)(states + indices[2]), Avx.Permute2x128(tt2, tt6, 0x20));
        if (indices[3] != -1 && states[indices[3]].flags.HasFlag(BodyFlags.Dynamic)) Avx.StoreAligned((float*)(states + indices[3]), Avx.Permute2x128(tt3, tt7, 0x20));
        if (indices[4] != -1 && states[indices[4]].flags.HasFlag(BodyFlags.Dynamic)) Avx.StoreAligned((float*)(states + indices[4]), Avx.Permute2x128(tt0, tt4, 0x31));
        if (indices[5] != -1 && states[indices[5]].flags.HasFlag(BodyFlags.Dynamic)) Avx.StoreAligned((float*)(states + indices[5]), Avx.Permute2x128(tt1, tt5, 0x31));
        if (indices[6] != -1 && states[indices[6]].flags.HasFlag(BodyFlags.Dynamic)) Avx.StoreAligned((float*)(states + indices[6]), Avx.Permute2x128(tt2, tt6, 0x31));
        if (indices[7] != -1 && states[indices[7]].flags.HasFlag(BodyFlags.Dynamic)) Avx.StoreAligned((float*)(states + indices[7]), Avx.Permute2x128(tt3, tt7, 0x31));
    }
    public unsafe void PrepareContactsTask(int startIndex, int endIndex, StepContext context)
    {
        World world = context.world;
        ContactSim[] contacts = context.contacts;
        var awakeStates = context.states;
        Softness contactSoftness = context.contactSoftness;
        Softness staticSoftness = context.staticSoftness;
        bool enableSoftening = world.enableContactSoftening;
        float warmStartScale = world.enableWarmStarting ? 1 : 0;
        for (int i = startIndex; i < endIndex; i++)
        {
            var constraint = ((ContactConstraintsAVX)context.simdContactConstraints).simdConstraints + i;
            for (int j = 0; j < 8; j++)
            {
                ref ContactSim contactSim = ref contacts[8 * i + j];
                if (contactSim != null)
                {
                    Manifold manifold = contactSim.manifold;
                    int indexA = contactSim.bodySimIndexA, indexB = contactSim.bodySimIndexB;

                    ((int*)&constraint->indexA)[j] = indexA;
                    ((int*)&constraint->indexB)[j] = indexB;
                    Vector2 vA = Vector2.Zero;
                    float wA = 0, mA = contactSim.invMassA, iA = contactSim.invIA;
                    if (indexA != -1)
                    {
                        vA = awakeStates[indexA].linearVelocity;
                        wA = awakeStates[indexA].angularVelocity;
                    }
                    Vector2 vB = Vector2.Zero;
                    float wB = 0, mB = contactSim.invMassB, iB = contactSim.invIB;
                    if (indexB != -1)
                    {
                        vB = awakeStates[indexB].linearVelocity;
                        wB = awakeStates[indexB].angularVelocity;
                    }
                    ((float*)&constraint->invMassA)[j] = mA;
                    ((float*)&constraint->invMassB)[j] = mB;
                    ((float*)&constraint->invIA)[j] = iA;
                    ((float*)&constraint->invIB)[j] = iB;
                    {
                        float k = iA + iB;
                        ((float*)&constraint->rollingMass)[j] = k > 0.0f ? 1.0f / k : 0.0f;
                    }
                    Softness soft = contactSoftness;
                    if (indexA == -1 || indexB == -1) soft = staticSoftness;
                    else if (enableSoftening)
                    {
                        float contactHertz = Math.Min(world.contactHertz, 0.125f * context.inv_h);
                        float ratio = 1;
                        if (mA < mB) ratio = Math.Max(0.5f, mA / mB);
                        else if (mB < mA) ratio = Math.Max(0.5f, mB / mA);
                        soft = new(ratio * contactHertz, ratio * world.contactDampingRatio, context.h);
                    }

                    Vector2 normal = manifold.normal;
                    ((float*)&constraint->normal.X)[j] = normal.x;
                    ((float*)&constraint->normal.Y)[j] = normal.y;

                    ((float*)&constraint->friction)[j] = contactSim.friction;
                    ((float*)&constraint->tangentSpeed)[j] = contactSim.tangentSpeed;
                    ((float*)&constraint->restitution)[j] = contactSim.restitution;
                    ((float*)&constraint->rollingResistance)[j] = contactSim.rollingResistance;
                    ((float*)&constraint->rollingImpulse)[j] = warmStartScale * manifold.rollingImpulse;

                    ((float*)&constraint->biasRate)[j] = soft.biasRate;
                    ((float*)&constraint->massScale)[j] = soft.massScale;
                    ((float*)&constraint->impulseScale)[j] = soft.impulseScale;

                    Vector2 tangent = normal.RightPerp();

                    {
                        ref ManifoldPoint mp = ref manifold.point0;

                        Vector2 rA = mp.anchorA;
                        Vector2 rB = mp.anchorB;

                        ((float*)&constraint->anchorA1.X)[j] = rA.x;
                        ((float*)&constraint->anchorA1.Y)[j] = rA.y;
                        ((float*)&constraint->anchorB1.X)[j] = rB.x;
                        ((float*)&constraint->anchorB1.Y)[j] = rB.y;

                        ((float*)&constraint->baseSeparation1)[j] = mp.separation - Vector2.Dot(rB - rA, normal);

                        ((float*)&constraint->normalImpulse1)[j] = warmStartScale * mp.normalImpulse;
                        ((float*)&constraint->tangentImpulse1)[j] = warmStartScale * mp.tangentImpulse;
                        ((float*)&constraint->totalNormalImpulse1)[j] = 0.0f;

                        float rnA = Vector2.Cross(rA, normal);
                        float rnB = Vector2.Cross(rB, normal);
                        float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                        ((float*)&constraint->normalMass1)[j] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

                        float rtA = Vector2.Cross(rA, tangent);
                        float rtB = Vector2.Cross(rB, tangent);
                        float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
                        ((float*)&constraint->tangentMass1)[j] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

                        // relative velocity for restitution
                        Vector2 vrA = vA + Vector2.CrossSV(wA, rA);
                        Vector2 vrB = vB + Vector2.CrossSV(wB, rB);
                        ((float*)&constraint->relativeVelocity1)[j] = Vector2.Dot(normal, vrB - vrA);
                    }

                    int pointCount = manifold.pointCount;
                    Debug.Assert(0 < pointCount && pointCount <= 2);

                    if (pointCount == 2)
                    {
                        ref ManifoldPoint mp = ref manifold.point1;

                        Vector2 rA = mp.anchorA;
                        Vector2 rB = mp.anchorB;

                        ((float*)&constraint->anchorA2.X)[j] = rA.x;
                        ((float*)&constraint->anchorA2.Y)[j] = rA.y;
                        ((float*)&constraint->anchorB2.X)[j] = rB.x;
                        ((float*)&constraint->anchorB2.Y)[j] = rB.y;

                        ((float*)&constraint->baseSeparation2)[j] = mp.separation - Vector2.Dot(rB - rA, normal);

                        ((float*)&constraint->normalImpulse2)[j] = warmStartScale * mp.normalImpulse;
                        ((float*)&constraint->tangentImpulse2)[j] = warmStartScale * mp.tangentImpulse;
                        ((float*)&constraint->totalNormalImpulse2)[j] = 0.0f;

                        float rnA = Vector2.Cross(rA, normal);
                        float rnB = Vector2.Cross(rB, normal);
                        float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                        ((float*)&constraint->normalMass2)[j] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

                        float rtA = Vector2.Cross(rA, tangent);
                        float rtB = Vector2.Cross(rB, tangent);
                        float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
                        ((float*)&constraint->tangentMass2)[j] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

                        // relative velocity for restitution
                        Vector2 vrA = vA + Vector2.CrossSV(wA, rA);
                        Vector2 vrB = vB + Vector2.CrossSV(wB, rB);
                        ((float*)&constraint->relativeVelocity2)[j] = Vector2.Dot(normal, vrB - vrA);
                    }
                    else
                    {
                        // dummy data that has no effect
                        ((float*)&constraint->baseSeparation2)[j] = 0.0f;
                        ((float*)&constraint->normalImpulse2)[j] = 0.0f;
                        ((float*)&constraint->tangentImpulse2)[j] = 0.0f;
                        ((float*)&constraint->totalNormalImpulse2)[j] = 0.0f;
                        ((float*)&constraint->anchorA2.X)[j] = 0.0f;
                        ((float*)&constraint->anchorA2.Y)[j] = 0.0f;
                        ((float*)&constraint->anchorB2.X)[j] = 0.0f;
                        ((float*)&constraint->anchorB2.Y)[j] = 0.0f;
                        ((float*)&constraint->normalMass2)[j] = 0.0f;
                        ((float*)&constraint->tangentMass2)[j] = 0.0f;
                        ((float*)&constraint->relativeVelocity2)[j] = 0.0f;
                    }
                }
                else
                {
                    ((int*)&constraint->indexA)[j] = -1;
                    ((int*)&constraint->indexB)[j] = -1;

                    ((float*)&constraint->invMassA)[j] = 0.0f;
                    ((float*)&constraint->invMassB)[j] = 0.0f;
                    ((float*)&constraint->invIA)[j] = 0.0f;
                    ((float*)&constraint->invIB)[j] = 0.0f;

                    ((float*)&constraint->normal.X)[j] = 0.0f;
                    ((float*)&constraint->normal.Y)[j] = 0.0f;
                    ((float*)&constraint->friction)[j] = 0.0f;
                    ((float*)&constraint->tangentSpeed)[j] = 0.0f;
                    ((float*)&constraint->rollingResistance)[j] = 0.0f;
                    ((float*)&constraint->rollingMass)[j] = 0.0f;
                    ((float*)&constraint->rollingImpulse)[j] = 0.0f;
                    ((float*)&constraint->biasRate)[j] = 0.0f;
                    ((float*)&constraint->massScale)[j] = 0.0f;
                    ((float*)&constraint->impulseScale)[j] = 0.0f;

                    ((float*)&constraint->anchorA1.X)[j] = 0.0f;
                    ((float*)&constraint->anchorA1.Y)[j] = 0.0f;
                    ((float*)&constraint->anchorB1.X)[j] = 0.0f;
                    ((float*)&constraint->anchorB1.Y)[j] = 0.0f;
                    ((float*)&constraint->baseSeparation1)[j] = 0.0f;
                    ((float*)&constraint->normalImpulse1)[j] = 0.0f;
                    ((float*)&constraint->tangentImpulse1)[j] = 0.0f;
                    ((float*)&constraint->totalNormalImpulse1)[j] = 0.0f;
                    ((float*)&constraint->normalMass1)[j] = 0.0f;
                    ((float*)&constraint->tangentMass1)[j] = 0.0f;

                    ((float*)&constraint->anchorA2.X)[j] = 0.0f;
                    ((float*)&constraint->anchorA2.Y)[j] = 0.0f;
                    ((float*)&constraint->anchorB2.X)[j] = 0.0f;
                    ((float*)&constraint->anchorB2.Y)[j] = 0.0f;
                    ((float*)&constraint->baseSeparation2)[j] = 0.0f;
                    ((float*)&constraint->normalImpulse2)[j] = 0.0f;
                    ((float*)&constraint->tangentImpulse2)[j] = 0.0f;
                    ((float*)&constraint->totalNormalImpulse2)[j] = 0.0f;
                    ((float*)&constraint->normalMass2)[j] = 0.0f;
                    ((float*)&constraint->tangentMass2)[j] = 0.0f;

                    ((float*)&constraint->restitution)[j] = 0.0f;
                    ((float*)&constraint->relativeVelocity1)[j] = 0.0f;
                    ((float*)&constraint->relativeVelocity2)[j] = 0.0f;
                }
            }
        }
    }
    public unsafe void WarmStartContactsTask(int startIndex, int endIndex, StepContext context, int colorIndex)
    {
        var states = context.states.Data;
        var constraints = ((ContactConstraintsAVX)context.graph.colors[colorIndex].simdConstraints).simdConstraints;
        {
            for (int i = startIndex; i < endIndex; i++)
            {
                ContactConstraintSIMD* c = constraints + i;
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
                }
                bA.w = Avx.Subtract(bA.w, Avx.Multiply(c->invIA, c->rollingImpulse));
                bB.w = Avx.Add(bB.w, Avx.Multiply(c->invIB, c->rollingImpulse));
                ScatterBodies(states, (int*)&c->indexA, ref bA);
                ScatterBodies(states, (int*)&c->indexB, ref bB);
            }
        }
    }
    public unsafe void SolveContactsTask(int startIndex, int endIndex, StepContext context, int colorIndex, bool useBias)
    {
        var states = context.states.Data;
        var constraints = ((ContactConstraintsAVX)context.graph.colors[colorIndex].simdConstraints).simdConstraints;
        {
            Vector256<float> inv_h = Vector256.Create(context.inv_h);
            Vector256<float> contactSpeed = Vector256.Create(-context.world.contactSpeed);
            Vector256<float> oneW = Vector256<float>.One;
            for (int i = startIndex; i < endIndex; i++)
            {
                ContactConstraintSIMD* c = constraints + i;
                BodyStateW bA = GatherBodies(states, (int*)&c->indexA);
                BodyStateW bB = GatherBodies(states, (int*)&c->indexB);
                Vector256<float> biasRate, massScale, impulseScale;
                if (useBias)
                {
                    biasRate = Avx.Multiply(c->massScale, c->biasRate);
                    massScale = c->massScale;
                    impulseScale = c->impulseScale;
                }
                else
                {
                    biasRate = Vector256<float>.Zero;
                    massScale = oneW;
                    impulseScale = Vector256<float>.Zero;
                }
                Vector256<float> totalNormalImpulse = Vector256<float>.Zero;
                Vector2W dp = new() { X = Avx.Subtract(bB.dp.X, bA.dp.X), Y = Avx.Subtract(bB.dp.Y, bA.dp.Y) };
                {
                    Vector2W rA = c->anchorA1, rB = c->anchorB1;
                    Vector2W rsA = RotateVectorW(bA.dq, rA), rsB = RotateVectorW(bB.dq, rB);
                    Vector2W ds = new() { X = Avx.Add(dp.X, Avx.Subtract(rsB.X, rsA.X)), Y = Avx.Add(dp.Y, Avx.Subtract(rsB.Y, rsA.Y)) };
                    Vector256<float> s = Avx.Add(DotW(c->normal, ds), c->baseSeparation1);
                    Vector256<float> mask = Avx.CompareGreaterThan(s, Vector256<float>.Zero);
                    Vector256<float> specBias = Avx.Multiply(s, inv_h), softBias = Avx.Max(Avx.Multiply(biasRate, s), contactSpeed);
                    Vector256<float> bias = Avx.BlendVariable(softBias, specBias, mask);
                    Vector256<float> pointMassScale = Avx.BlendVariable(massScale, oneW, mask);
                    Vector256<float> pointImpulseScale = Avx.BlendVariable(impulseScale, Vector256<float>.Zero, mask);
                    Vector256<float> dvx = Avx.Subtract(Avx.Subtract(bB.v.X, Avx.Multiply(bB.w, rB.Y)), Avx.Subtract(bA.v.X, Avx.Multiply(bA.w, rA.Y)));
                    Vector256<float> dvy = Avx.Subtract(Avx.Add(bB.v.Y, Avx.Multiply(bB.w, rB.X)), Avx.Add(bA.v.Y, Avx.Multiply(bA.w, rA.X)));
                    Vector256<float> vn = Avx.Add(Avx.Multiply(dvx, c->normal.X), Avx.Multiply(dvy, c->normal.Y));
                    Vector256<float> negImpulse = Avx.Add(Avx.Multiply(c->normalMass1, Avx.Add(Avx.Multiply(pointMassScale, vn), bias)), Avx.Multiply(pointImpulseScale, c->normalImpulse1));
                    Vector256<float> newImpulse = Avx.Max(Avx.Subtract(c->normalImpulse1, negImpulse), Vector256<float>.Zero);
                    Vector256<float> impulse = Avx.Subtract(newImpulse, c->normalImpulse1);
                    c->normalImpulse1 = newImpulse;
                    c->totalNormalImpulse1 = Avx.Add(c->totalNormalImpulse1, newImpulse);
                    totalNormalImpulse = Avx.Add(totalNormalImpulse, newImpulse);
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
                    Vector256<float> mask = Avx.CompareGreaterThan(s, Vector256<float>.Zero);
                    Vector256<float> specBias = Avx.Multiply(s, inv_h), softBias = Avx.Max(Avx.Multiply(biasRate, s), contactSpeed);
                    Vector256<float> bias = Avx.BlendVariable(softBias, specBias, mask);
                    Vector256<float> pointMassScale = Avx.BlendVariable(massScale, oneW, mask);
                    Vector256<float> pointImpulseScale = Avx.BlendVariable(impulseScale, Vector256<float>.Zero, mask);
                    Vector256<float> dvx = Avx.Subtract(Avx.Subtract(bB.v.X, Avx.Multiply(bB.w, rB.Y)), Avx.Subtract(bA.v.X, Avx.Multiply(bA.w, rA.Y)));
                    Vector256<float> dvy = Avx.Subtract(Avx.Add(bB.v.Y, Avx.Multiply(bB.w, rB.X)), Avx.Add(bA.v.Y, Avx.Multiply(bA.w, rA.X)));
                    Vector256<float> vn = Avx.Add(Avx.Multiply(dvx, c->normal.X), Avx.Multiply(dvy, c->normal.Y));
                    //different than 1, is this intended?
                    Vector256<float> negImpulse = Avx.Add(Avx.Multiply(c->normalMass2, Avx.Add(Avx.Multiply(pointMassScale, vn), bias)), Avx.Multiply(pointImpulseScale, c->normalImpulse2));
                    Vector256<float> newImpulse = Avx.Max(Avx.Subtract(c->normalImpulse2, negImpulse), Vector256<float>.Zero);
                    Vector256<float> impulse = Avx.Subtract(newImpulse, c->normalImpulse2);
                    c->normalImpulse2 = newImpulse;
                    c->totalNormalImpulse2 = Avx.Add(c->totalNormalImpulse2, newImpulse);
                    totalNormalImpulse = Avx.Add(totalNormalImpulse, newImpulse);
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
                {
                    Vector256<float> deltaLambda = Avx.Multiply(c->rollingMass, Avx.Subtract(bA.w, bB.w));
                    Vector256<float> lambda = c->rollingImpulse;
                    Vector256<float> maxLambda = Avx.Multiply(c->rollingResistance, totalNormalImpulse);
                    c->rollingImpulse = SymClampW(Avx.Add(lambda, deltaLambda), maxLambda);
                    deltaLambda = Avx.Subtract(c->rollingImpulse, lambda);
                    bA.w = Avx.Subtract(bA.w, Avx.Multiply(c->invIA, deltaLambda));
                    bB.w = Avx.Add(bB.w, Avx.Multiply(c->invIB, deltaLambda));
                }
                ScatterBodies(states, (int*)&c->indexA, ref bA);
                ScatterBodies(states, (int*)&c->indexB, ref bB);
            }
        }
    }
    public unsafe void ApplyRestitutionTask(int startIndex, int endIndex, StepContext context, int colorIndex)
    {
        var states = context.states.Data;
        var constraints = ((ContactConstraintsAVX)context.graph.colors[colorIndex].simdConstraints).simdConstraints;
        {
            Vector256<float> threshold = Vector256.Create(context.world.restitutionThreshold);
            Vector256<float> zero = Vector256<float>.Zero;
            for (int i = startIndex; i < endIndex; i++)
            {
                ContactConstraintSIMD* c = constraints + i;
                if (AllZeroW(c->restitution)) continue;
                Vector256<float> restitutionMask = Avx.CompareEqual(c->restitution, zero);
                BodyStateW bA = GatherBodies(states, (int*)&c->indexA);
                BodyStateW bB = GatherBodies(states, (int*)&c->indexB);
                {
                    Vector256<float> mask1 = Avx.CompareGreaterThan(Avx.Add(c->relativeVelocity1, threshold), zero);
                    Vector256<float> mask2 = Avx.CompareEqual(c->totalNormalImpulse1, zero);
                    Vector256<float> mask = Avx.Or(Avx.Or(mask1, mask2), restitutionMask);
                    Vector256<float> mass = Avx.BlendVariable(c->normalMass1, zero, mask);
                    Vector2W rA = c->anchorA1, rB = c->anchorB1;
                    Vector256<float> dvx = Avx.Subtract(Avx.Subtract(bB.v.X, Avx.Multiply(bB.w, rB.Y)), Avx.Subtract(bA.v.X, Avx.Multiply(bA.w, rA.Y)));
                    Vector256<float> dvy = Avx.Subtract(Avx.Add(bB.v.Y, Avx.Multiply(bB.w, rB.X)), Avx.Add(bA.v.Y, Avx.Multiply(bA.w, rA.X)));
                    Vector256<float> vn = Avx.Add(Avx.Multiply(dvx, c->normal.X), Avx.Multiply(dvy, c->normal.Y));
                    Vector256<float> negImpulse = Avx.Multiply(mask, Avx.Add(vn, Avx.Multiply(c->restitution, c->relativeVelocity1)));
                    Vector256<float> newImpulse = Avx.Max(Avx.Subtract(c->normalImpulse1, negImpulse), Vector256<float>.Zero);
                    Vector256<float> deltaImpulse = Avx.Subtract(newImpulse, c->normalImpulse1);
                    c->normalImpulse1 = newImpulse;
                    c->totalNormalImpulse1 = Avx.Add(c->totalNormalImpulse1, deltaImpulse);
                    Vector256<float> Px = Avx.Multiply(deltaImpulse, c->normal.X);
                    Vector256<float> Py = Avx.Multiply(deltaImpulse, c->normal.Y);
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
                    Vector256<float> mask1 = Avx.CompareGreaterThan(Avx.Add(c->relativeVelocity2, threshold), zero);
                    Vector256<float> mask2 = Avx.CompareEqual(c->totalNormalImpulse2, zero);
                    Vector256<float> mask = Avx.Or(Avx.Or(mask1, mask2), restitutionMask);
                    Vector256<float> mass = Avx.BlendVariable(c->normalMass2, zero, mask);
                    Vector2W rA = c->anchorA2, rB = c->anchorB2;
                    Vector256<float> dvx = Avx.Subtract(Avx.Subtract(bB.v.X, Avx.Multiply(bB.w, rB.Y)), Avx.Subtract(bA.v.X, Avx.Multiply(bA.w, rA.Y)));
                    Vector256<float> dvy = Avx.Subtract(Avx.Add(bB.v.Y, Avx.Multiply(bB.w, rB.X)), Avx.Add(bA.v.Y, Avx.Multiply(bA.w, rA.X)));
                    Vector256<float> vn = Avx.Add(Avx.Multiply(dvx, c->normal.X), Avx.Multiply(dvy, c->normal.Y));
                    Vector256<float> negImpulse = Avx.Multiply(mask, Avx.Add(vn, Avx.Multiply(c->restitution, c->relativeVelocity2)));
                    Vector256<float> newImpulse = Avx.Max(Avx.Subtract(c->normalImpulse2, negImpulse), Vector256<float>.Zero);
                    Vector256<float> deltaImpulse = Avx.Subtract(newImpulse, c->normalImpulse2);
                    c->normalImpulse2 = newImpulse;
                    c->totalNormalImpulse2 = Avx.Add(c->totalNormalImpulse2, deltaImpulse);
                    Vector256<float> Px = Avx.Multiply(deltaImpulse, c->normal.X);
                    Vector256<float> Py = Avx.Multiply(deltaImpulse, c->normal.Y);
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
    public unsafe void StoreImpulsesTask(int startIndex, int endIndex, StepContext context)
    {
        ContactSim[] contacts = context.contacts;
        Manifold dummy = new();
        ContactConstraintSIMD* constraints = ((ContactConstraintsAVX)context.simdContactConstraints).simdConstraints;
        {
            for (int constraintIndex = startIndex; constraintIndex < endIndex; constraintIndex++)
            {
                ContactConstraintSIMD* c = constraints + constraintIndex;
                float* rollingImpulse = (float*)&c->rollingImpulse;
                float* normalImpulse1 = (float*)&c->normalImpulse1;
                float* normalImpulse2 = (float*)&c->normalImpulse2;
                float* tangentImpulse1 = (float*)&c->tangentImpulse1;
                float* tangentImpulse2 = (float*)&c->tangentImpulse2;
                float* totalNormalImpulse1 = (float*)&c->totalNormalImpulse1;
                float* totalNormalImpulse2 = (float*)&c->totalNormalImpulse2;
                float* normalVelocity1 = (float*)&c->relativeVelocity1;
                float* normalVelocity2 = (float*)&c->relativeVelocity2;
                int baseIndex = 8 * constraintIndex;
                for (int laneIndex = 0; laneIndex < 8; ++laneIndex)
                {
                    ref Manifold m = ref dummy;
                    if (contacts[baseIndex + laneIndex] != null) m = ref contacts[baseIndex + laneIndex].manifold;
                    m.rollingImpulse = rollingImpulse[laneIndex];
                    m.point0.normalImpulse = normalImpulse1[laneIndex];
                    m.point0.tangentImpulse = tangentImpulse1[laneIndex];
                    m.point0.totalNormalImpulse = totalNormalImpulse1[laneIndex];
                    m.point0.normalVelocity = normalVelocity1[laneIndex];
                    m.point1.normalImpulse = normalImpulse2[laneIndex];
                    m.point1.tangentImpulse = tangentImpulse2[laneIndex];
                    m.point1.totalNormalImpulse = totalNormalImpulse2[laneIndex];
                    m.point1.normalVelocity = normalVelocity2[laneIndex];
                }
            }
        }
    }
}
public class ContactSolverNeon : IContactSolverW
{
    public struct Vector2W
    {
        public Vector128<float> X, Y;
    }
    struct RotationW
    {
        public Vector128<float> C, S;
    }
    static Vector128<float> SymClampW(Vector128<float> a, Vector128<float> b) => AdvSimd.Max(AdvSimd.Negate(b), Sse.Min(a, b));
    static bool AllZeroW(Vector128<float> a)
    {
        Vector128<float> cmp_result = AdvSimd.CompareEqual(a, Vector128<float>.Zero);
        return AdvSimd.Arm64.IsSupported ? AdvSimd.Arm64.MinAcross(cmp_result).GetElement(0) != 0
            : AdvSimd.Extract(cmp_result, 0) != 0 && AdvSimd.Extract(cmp_result, 1) != 0 && AdvSimd.Extract(cmp_result, 2) != 0 && AdvSimd.Extract(cmp_result, 3) != 0;
    }
    static Vector128<float> DotW(Vector2W a, Vector2W b) => AdvSimd.Add(AdvSimd.Multiply(a.X, b.X), AdvSimd.Multiply(a.Y, b.Y));
    static Vector128<float> CrossW(Vector2W a, Vector2W b) => AdvSimd.Subtract(AdvSimd.Multiply(a.X, b.Y), AdvSimd.Multiply(a.Y, b.X));
    static Vector2W RotateVectorW(RotationW q, Vector2W v) =>
        new() { X = AdvSimd.Subtract(AdvSimd.Multiply(q.C, v.X), AdvSimd.Multiply(q.S, v.Y)), Y = AdvSimd.Add(AdvSimd.Multiply(q.S, v.X), AdvSimd.Multiply(q.C, v.Y)) };
    static Vector128<float> BlendW(Vector128<float> a, Vector128<float> b, Vector128<float> mask) => AdvSimd.BitwiseSelect(mask, b, a);
    static Vector128<float> UnpackLoW(Vector128<float> a, Vector128<float> b)
    {
        if (AdvSimd.Arm64.IsSupported) return AdvSimd.Arm64.ZipLow(a, b);
        Vector64<float> a1 = a.GetLower(), b1 = b.GetLower();
        return Vector128.Create(a1, b1);
    }
    static Vector128<float> UnpackHiW(Vector128<float> a, Vector128<float> b)
    {
        if (AdvSimd.Arm64.IsSupported) return AdvSimd.Arm64.ZipHigh(a, b);
        Vector64<float> a1 = a.GetUpper(), b1 = b.GetUpper();
        return Vector128.Create(a1, b1);
    }
    public struct ContactConstraintSIMD
    {
        public Vector128<int> indexA, indexB;
        public Vector128<float> invMassA, invMassB;
        public Vector128<float> invIA, invIB;
        public Vector2W normal;
        public Vector128<float> friction;
        public Vector128<float> tangentSpeed;
        public Vector128<float> rollingResistance;
        public Vector128<float> rollingMass;
        public Vector128<float> rollingImpulse;
        public Vector128<float> biasRate;
        public Vector128<float> massScale;
        public Vector128<float> impulseScale;
        public Vector2W anchorA1, anchorB1;
        public Vector128<float> normalMass1, tangentMass1;
        public Vector128<float> baseSeparation1;
        public Vector128<float> normalImpulse1;
        public Vector128<float> totalNormalImpulse1;
        public Vector128<float> tangentImpulse1;
        public Vector2W anchorA2, anchorB2;
        public Vector128<float> baseSeparation2;
        public Vector128<float> normalImpulse2;
        public Vector128<float> totalNormalImpulse2;
        public Vector128<float> tangentImpulse2;
        public Vector128<float> normalMass2, tangentMass2;
        public Vector128<float> restitution;
        public Vector128<float> relativeVelocity1, relativeVelocity2;
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
        Vector128<float> b1a = indices[0] == -1 ? identityA : AdvSimd.LoadVector128((float*)(states + indices[0]));
        Vector128<float> b1b = indices[0] == -1 ? identityB : AdvSimd.LoadVector128((float*)(states + indices[0]));
        Vector128<float> a = indices[1] == -1 ? identityA : AdvSimd.LoadVector128((float*)(states + indices[1]));
        Vector128<float> b = indices[1] == -1 ? identityB : AdvSimd.LoadVector128((float*)(states + indices[1]));
        Vector128<float> b3a = indices[2] == -1 ? identityA : AdvSimd.LoadVector128((float*)(states + indices[2]));
        Vector128<float> b3b = indices[2] == -1 ? identityB : AdvSimd.LoadVector128((float*)(states + indices[2]));
        Vector128<float> b4a = indices[3] == -1 ? identityA : AdvSimd.LoadVector128((float*)(states + indices[3]));
        Vector128<float> b4b = indices[3] == -1 ? identityB : AdvSimd.LoadVector128((float*)(states + indices[3]));
        Vector128<float> t1a = UnpackLoW(b1a, b3a);
        Vector128<float> t2a = UnpackLoW(a, b4a);
        Vector128<float> t3a = UnpackHiW(b1a, b3a);
        Vector128<float> t4a = UnpackHiW(a, b4a);
        Vector128<float> t1b = UnpackLoW(b1b, b3b);
        Vector128<float> t2b = UnpackLoW(b, b4b);
        Vector128<float> t3b = UnpackHiW(b1b, b3b);
        Vector128<float> t4b = UnpackHiW(b, b4b);
        return new()
        {
            v = new() { X = UnpackLoW(t1a, t2a), Y = UnpackHiW(t1a, t2a) },
            w = UnpackLoW(t3a, t4a),
            flags = UnpackHiW(t3a, t4a),
            dp = new() { X = UnpackLoW(t1b, t2b), Y = UnpackHiW(t1b, t2b) },
            dq = new() { C = UnpackLoW(t3b, t4b), S = UnpackHiW(t3b, t4b) },
        };

    }
    [Obsolete("vtrnq instruction is not in C#")] unsafe void ScatterBodies(BodyState* states, int* indices, ref BodyStateW simdBody)
    {
        if (indices[0] != -1)
        {
            BodyState* state = states + indices[0];
            state->linearVelocity.x = simdBody.v.X.GetElement(0);
            state->linearVelocity.y = simdBody.v.Y.GetElement(0);
            state->angularVelocity = simdBody.w.GetElement(0);
        }
        if (indices[1] != -1)
        {
            BodyState* state = states + indices[1];
            state->linearVelocity.x = simdBody.v.X.GetElement(1);
            state->linearVelocity.y = simdBody.v.Y.GetElement(1);
            state->angularVelocity = simdBody.w.GetElement(1);
        }
        if (indices[2] != -1)
        {
            BodyState* state = states + indices[2];
            state->linearVelocity.x = simdBody.v.X.GetElement(2);
            state->linearVelocity.y = simdBody.v.Y.GetElement(2);
            state->angularVelocity = simdBody.w.GetElement(2);
        }
        if (indices[3] != -1)
        {
            BodyState* state = states + indices[3];
            state->linearVelocity.x = simdBody.v.X.GetElement(3);
            state->linearVelocity.y = simdBody.v.Y.GetElement(3);
            state->angularVelocity = simdBody.w.GetElement(3);
        }
        Debug.Assert(((nuint)states & 0x1F) == 0);
        throw new NotImplementedException("vtrnq instruction is not in C#");
        /*Vector256<float> r1 = AdvSimd.Arm64.TransposeEven(simdBody.v.X, simdBody.v.Y);
        Vector256<float> r2 = AdvSimd.Arm64.TransposeOdd(simdBody.w, simdBody.flags);
        if (indices[0] != -1 && states[indices[0]].flags.HasFlag(BodyFlags.Dynamic)) AdvSimd.Store((float*)(states + indices[0]), Vector128.Create(r1.GetLower().GetLower(), r2.GetLower().GetLower()));
        if (indices[1] != -1 && states[indices[1]].flags.HasFlag(BodyFlags.Dynamic)) AdvSimd.Store((float*)(states + indices[1]), Vector128.Create(r1.GetUpper().GetLower(), r2.GetUpper().GetLower()));
        if (indices[2] != -1 && states[indices[2]].flags.HasFlag(BodyFlags.Dynamic)) AdvSimd.Store((float*)(states + indices[2]), Vector128.Create(r1.GetLower().GetUpper(), r2.GetLower().GetUpper()));
        if (indices[3] != -1 && states[indices[3]].flags.HasFlag(BodyFlags.Dynamic)) AdvSimd.Store((float*)(states + indices[3]), Vector128.Create(r1.GetUpper().GetUpper(), r2.GetUpper().GetUpper()));*/

    }
    public unsafe void PrepareContactsTask(int startIndex, int endIndex, StepContext context)
    {
        World world = context.world;
        ContactSim[] contacts = context.contacts;
        var awakeStates = context.states;
        Softness contactSoftness = context.contactSoftness;
        Softness staticSoftness = context.staticSoftness;
        bool enableSoftening = world.enableContactSoftening;
        float warmStartScale = world.enableWarmStarting ? 1 : 0;
        for (int i = startIndex; i < endIndex; i++)
        {
            var constraint = ((ContactConstraintsNeon)context.simdContactConstraints).simdConstraints + i;
            for (int j = 0; j < 4; j++)
            {
                ref ContactSim contactSim = ref contacts[4 * i + j];
                if (contactSim != null)
                {
                    Manifold manifold = contactSim.manifold;
                    int indexA = contactSim.bodySimIndexA, indexB = contactSim.bodySimIndexB;

                    ((int*)&constraint->indexA)[j] = indexA;
                    ((int*)&constraint->indexB)[j] = indexB;
                    Vector2 vA = Vector2.Zero;
                    float wA = 0, mA = contactSim.invMassA, iA = contactSim.invIA;
                    if (indexA != -1)
                    {
                        vA = awakeStates[indexA].linearVelocity;
                        wA = awakeStates[indexA].angularVelocity;
                    }
                    Vector2 vB = Vector2.Zero;
                    float wB = 0, mB = contactSim.invMassB, iB = contactSim.invIB;
                    if (indexB != -1)
                    {
                        vB = awakeStates[indexB].linearVelocity;
                        wB = awakeStates[indexB].angularVelocity;
                    }
                    ((float*)&constraint->invMassA)[j] = mA;
                    ((float*)&constraint->invMassB)[j] = mB;
                    ((float*)&constraint->invIA)[j] = iA;
                    ((float*)&constraint->invIB)[j] = iB;
                    {
                        float k = iA + iB;
                        ((float*)&constraint->rollingMass)[j] = k > 0.0f ? 1.0f / k : 0.0f;
                    }
                    Softness soft = contactSoftness;
                    if (indexA == -1 || indexB == -1) soft = staticSoftness;
                    else if (enableSoftening)
                    {
                        float contactHertz = Math.Min(world.contactHertz, 0.125f * context.inv_h);
                        float ratio = 1;
                        if (mA < mB) ratio = Math.Max(0.5f, mA / mB);
                        else if (mB < mA) ratio = Math.Max(0.5f, mB / mA);
                        soft = new(ratio * contactHertz, ratio * world.contactDampingRatio, context.h);
                    }

                    Vector2 normal = manifold.normal;
                    ((float*)&constraint->normal.X)[j] = normal.x;
                    ((float*)&constraint->normal.Y)[j] = normal.y;

                    ((float*)&constraint->friction)[j] = contactSim.friction;
                    ((float*)&constraint->tangentSpeed)[j] = contactSim.tangentSpeed;
                    ((float*)&constraint->restitution)[j] = contactSim.restitution;
                    ((float*)&constraint->rollingResistance)[j] = contactSim.rollingResistance;
                    ((float*)&constraint->rollingImpulse)[j] = warmStartScale * manifold.rollingImpulse;

                    ((float*)&constraint->biasRate)[j] = soft.biasRate;
                    ((float*)&constraint->massScale)[j] = soft.massScale;
                    ((float*)&constraint->impulseScale)[j] = soft.impulseScale;

                    Vector2 tangent = normal.RightPerp();

                    {
                        ref ManifoldPoint mp = ref manifold.point0;

                        Vector2 rA = mp.anchorA;
                        Vector2 rB = mp.anchorB;

                        ((float*)&constraint->anchorA1.X)[j] = rA.x;
                        ((float*)&constraint->anchorA1.Y)[j] = rA.y;
                        ((float*)&constraint->anchorB1.X)[j] = rB.x;
                        ((float*)&constraint->anchorB1.Y)[j] = rB.y;

                        ((float*)&constraint->baseSeparation1)[j] = mp.separation - Vector2.Dot(rB - rA, normal);

                        ((float*)&constraint->normalImpulse1)[j] = warmStartScale * mp.normalImpulse;
                        ((float*)&constraint->tangentImpulse1)[j] = warmStartScale * mp.tangentImpulse;
                        ((float*)&constraint->totalNormalImpulse1)[j] = 0.0f;

                        float rnA = Vector2.Cross(rA, normal);
                        float rnB = Vector2.Cross(rB, normal);
                        float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                        ((float*)&constraint->normalMass1)[j] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

                        float rtA = Vector2.Cross(rA, tangent);
                        float rtB = Vector2.Cross(rB, tangent);
                        float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
                        ((float*)&constraint->tangentMass1)[j] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

                        // relative velocity for restitution
                        Vector2 vrA = vA + Vector2.CrossSV(wA, rA);
                        Vector2 vrB = vB + Vector2.CrossSV(wB, rB);
                        ((float*)&constraint->relativeVelocity1)[j] = Vector2.Dot(normal, vrB - vrA);
                    }

                    int pointCount = manifold.pointCount;
                    Debug.Assert(0 < pointCount && pointCount <= 2);

                    if (pointCount == 2)
                    {
                        ref ManifoldPoint mp = ref manifold.point1;

                        Vector2 rA = mp.anchorA;
                        Vector2 rB = mp.anchorB;

                        ((float*)&constraint->anchorA2.X)[j] = rA.x;
                        ((float*)&constraint->anchorA2.Y)[j] = rA.y;
                        ((float*)&constraint->anchorB2.X)[j] = rB.x;
                        ((float*)&constraint->anchorB2.Y)[j] = rB.y;

                        ((float*)&constraint->baseSeparation2)[j] = mp.separation - Vector2.Dot(rB - rA, normal);

                        ((float*)&constraint->normalImpulse2)[j] = warmStartScale * mp.normalImpulse;
                        ((float*)&constraint->tangentImpulse2)[j] = warmStartScale * mp.tangentImpulse;
                        ((float*)&constraint->totalNormalImpulse2)[j] = 0.0f;

                        float rnA = Vector2.Cross(rA, normal);
                        float rnB = Vector2.Cross(rB, normal);
                        float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                        ((float*)&constraint->normalMass2)[j] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

                        float rtA = Vector2.Cross(rA, tangent);
                        float rtB = Vector2.Cross(rB, tangent);
                        float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
                        ((float*)&constraint->tangentMass2)[j] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

                        // relative velocity for restitution
                        Vector2 vrA = vA + Vector2.CrossSV(wA, rA);
                        Vector2 vrB = vB + Vector2.CrossSV(wB, rB);
                        ((float*)&constraint->relativeVelocity2)[j] = Vector2.Dot(normal, vrB - vrA);
                    }
                    else
                    {
                        // dummy data that has no effect
                        ((float*)&constraint->baseSeparation2)[j] = 0.0f;
                        ((float*)&constraint->normalImpulse2)[j] = 0.0f;
                        ((float*)&constraint->tangentImpulse2)[j] = 0.0f;
                        ((float*)&constraint->totalNormalImpulse2)[j] = 0.0f;
                        ((float*)&constraint->anchorA2.X)[j] = 0.0f;
                        ((float*)&constraint->anchorA2.Y)[j] = 0.0f;
                        ((float*)&constraint->anchorB2.X)[j] = 0.0f;
                        ((float*)&constraint->anchorB2.Y)[j] = 0.0f;
                        ((float*)&constraint->normalMass2)[j] = 0.0f;
                        ((float*)&constraint->tangentMass2)[j] = 0.0f;
                        ((float*)&constraint->relativeVelocity2)[j] = 0.0f;
                    }
                }
                else
                {
                    ((int*)&constraint->indexA)[j] = -1;
                    ((int*)&constraint->indexB)[j] = -1;

                    ((float*)&constraint->invMassA)[j] = 0.0f;
                    ((float*)&constraint->invMassB)[j] = 0.0f;
                    ((float*)&constraint->invIA)[j] = 0.0f;
                    ((float*)&constraint->invIB)[j] = 0.0f;

                    ((float*)&constraint->normal.X)[j] = 0.0f;
                    ((float*)&constraint->normal.Y)[j] = 0.0f;
                    ((float*)&constraint->friction)[j] = 0.0f;
                    ((float*)&constraint->tangentSpeed)[j] = 0.0f;
                    ((float*)&constraint->rollingResistance)[j] = 0.0f;
                    ((float*)&constraint->rollingMass)[j] = 0.0f;
                    ((float*)&constraint->rollingImpulse)[j] = 0.0f;
                    ((float*)&constraint->biasRate)[j] = 0.0f;
                    ((float*)&constraint->massScale)[j] = 0.0f;
                    ((float*)&constraint->impulseScale)[j] = 0.0f;

                    ((float*)&constraint->anchorA1.X)[j] = 0.0f;
                    ((float*)&constraint->anchorA1.Y)[j] = 0.0f;
                    ((float*)&constraint->anchorB1.X)[j] = 0.0f;
                    ((float*)&constraint->anchorB1.Y)[j] = 0.0f;
                    ((float*)&constraint->baseSeparation1)[j] = 0.0f;
                    ((float*)&constraint->normalImpulse1)[j] = 0.0f;
                    ((float*)&constraint->tangentImpulse1)[j] = 0.0f;
                    ((float*)&constraint->totalNormalImpulse1)[j] = 0.0f;
                    ((float*)&constraint->normalMass1)[j] = 0.0f;
                    ((float*)&constraint->tangentMass1)[j] = 0.0f;

                    ((float*)&constraint->anchorA2.X)[j] = 0.0f;
                    ((float*)&constraint->anchorA2.Y)[j] = 0.0f;
                    ((float*)&constraint->anchorB2.X)[j] = 0.0f;
                    ((float*)&constraint->anchorB2.Y)[j] = 0.0f;
                    ((float*)&constraint->baseSeparation2)[j] = 0.0f;
                    ((float*)&constraint->normalImpulse2)[j] = 0.0f;
                    ((float*)&constraint->tangentImpulse2)[j] = 0.0f;
                    ((float*)&constraint->totalNormalImpulse2)[j] = 0.0f;
                    ((float*)&constraint->normalMass2)[j] = 0.0f;
                    ((float*)&constraint->tangentMass2)[j] = 0.0f;

                    ((float*)&constraint->restitution)[j] = 0.0f;
                    ((float*)&constraint->relativeVelocity1)[j] = 0.0f;
                    ((float*)&constraint->relativeVelocity2)[j] = 0.0f;
                }
            }
        }
    }
    public unsafe void WarmStartContactsTask(int startIndex, int endIndex, StepContext context, int colorIndex)
    {
        var states = context.states.Data;
        var constraints = ((ContactConstraintsNeon)context.graph.colors[colorIndex].simdConstraints).simdConstraints;
        {
            for (int i = startIndex; i < endIndex; i++)
            {
                ContactConstraintSIMD* c = constraints + i;
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
                }
                bA.w = AdvSimd.Subtract(bA.w, AdvSimd.Multiply(c->invIA, c->rollingImpulse));
                bB.w = AdvSimd.Add(bB.w, AdvSimd.Multiply(c->invIB, c->rollingImpulse));
                ScatterBodies(states, (int*)&c->indexA, ref bA);
                ScatterBodies(states, (int*)&c->indexB, ref bB);
            }
        }
    }
    public unsafe void SolveContactsTask(int startIndex, int endIndex, StepContext context, int colorIndex, bool useBias)
    {
        var states = context.states.Data;
        var constraints = ((ContactConstraintsNeon)context.graph.colors[colorIndex].simdConstraints).simdConstraints;
        {
            Vector128<float> inv_h = Vector128.Create(context.inv_h);
            Vector128<float> contactSpeed = Vector128.Create(-context.world.contactSpeed);
            Vector128<float> oneW = Vector128<float>.One;
            for (int i = startIndex; i < endIndex; i++)
            {
                ContactConstraintSIMD* c = constraints + i;
                BodyStateW bA = GatherBodies(states, (int*)&c->indexA);
                BodyStateW bB = GatherBodies(states, (int*)&c->indexB);
                Vector128<float> biasRate, massScale, impulseScale;
                if (useBias)
                {
                    biasRate = AdvSimd.Multiply(c->massScale, c->biasRate);
                    massScale = c->massScale;
                    impulseScale = c->impulseScale;
                }
                else
                {
                    biasRate = Vector128<float>.Zero;
                    massScale = oneW;
                    impulseScale = Vector128<float>.Zero;
                }
                Vector128<float> totalNormalImpulse = Vector128<float>.Zero;
                Vector2W dp = new() { X = AdvSimd.Subtract(bB.dp.X, bA.dp.X), Y = AdvSimd.Subtract(bB.dp.Y, bA.dp.Y) };
                {
                    Vector2W rA = c->anchorA1, rB = c->anchorB1;
                    Vector2W rsA = RotateVectorW(bA.dq, rA), rsB = RotateVectorW(bB.dq, rB);
                    Vector2W ds = new() { X = AdvSimd.Add(dp.X, AdvSimd.Subtract(rsB.X, rsA.X)), Y = AdvSimd.Add(dp.Y, AdvSimd.Subtract(rsB.Y, rsA.Y)) };
                    Vector128<float> s = AdvSimd.Add(DotW(c->normal, ds), c->baseSeparation1);
                    Vector128<float> mask = AdvSimd.CompareGreaterThan(s, Vector128<float>.Zero);
                    Vector128<float> specBias = AdvSimd.Multiply(s, inv_h), softBias = AdvSimd.Max(AdvSimd.Multiply(biasRate, s), contactSpeed);
                    Vector128<float> bias = BlendW(softBias, specBias, mask);
                    Vector128<float> pointMassScale = BlendW(massScale, oneW, mask);
                    Vector128<float> pointImpulseScale = BlendW(impulseScale, Vector128<float>.Zero, mask);
                    Vector128<float> dvx = AdvSimd.Subtract(AdvSimd.Subtract(bB.v.X, AdvSimd.Multiply(bB.w, rB.Y)), AdvSimd.Subtract(bA.v.X, AdvSimd.Multiply(bA.w, rA.Y)));
                    Vector128<float> dvy = AdvSimd.Subtract(AdvSimd.Add(bB.v.Y, AdvSimd.Multiply(bB.w, rB.X)), AdvSimd.Add(bA.v.Y, AdvSimd.Multiply(bA.w, rA.X)));
                    Vector128<float> vn = AdvSimd.Add(AdvSimd.Multiply(dvx, c->normal.X), AdvSimd.Multiply(dvy, c->normal.Y));
                    Vector128<float> negImpulse = AdvSimd.Add(AdvSimd.Multiply(c->normalMass1, AdvSimd.Add(AdvSimd.Multiply(pointMassScale, vn), bias)), AdvSimd.Multiply(pointImpulseScale, c->normalImpulse1));
                    Vector128<float> newImpulse = AdvSimd.Max(AdvSimd.Subtract(c->normalImpulse1, negImpulse), Vector128<float>.Zero);
                    Vector128<float> impulse = AdvSimd.Subtract(newImpulse, c->normalImpulse1);
                    c->normalImpulse1 = newImpulse;
                    c->totalNormalImpulse1 = AdvSimd.Add(c->totalNormalImpulse1, newImpulse);
                    totalNormalImpulse = AdvSimd.Add(totalNormalImpulse, newImpulse);
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
                    Vector128<float> mask = AdvSimd.CompareGreaterThan(s, Vector128<float>.Zero);
                    Vector128<float> specBias = AdvSimd.Multiply(s, inv_h), softBias = AdvSimd.Max(AdvSimd.Multiply(biasRate, s), contactSpeed);
                    Vector128<float> bias = BlendW(softBias, specBias, mask);
                    Vector128<float> pointMassScale = BlendW(massScale, oneW, mask);
                    Vector128<float> pointImpulseScale = BlendW(impulseScale, Vector128<float>.Zero, mask);
                    Vector128<float> dvx = AdvSimd.Subtract(AdvSimd.Subtract(bB.v.X, AdvSimd.Multiply(bB.w, rB.Y)), AdvSimd.Subtract(bA.v.X, AdvSimd.Multiply(bA.w, rA.Y)));
                    Vector128<float> dvy = AdvSimd.Subtract(AdvSimd.Add(bB.v.Y, AdvSimd.Multiply(bB.w, rB.X)), AdvSimd.Add(bA.v.Y, AdvSimd.Multiply(bA.w, rA.X)));
                    Vector128<float> vn = AdvSimd.Add(AdvSimd.Multiply(dvx, c->normal.X), AdvSimd.Multiply(dvy, c->normal.Y));
                    //different than 1, is this intended?
                    Vector128<float> negImpulse = AdvSimd.Add(AdvSimd.Multiply(c->normalMass2, AdvSimd.Add(AdvSimd.Multiply(pointMassScale, vn), bias)), AdvSimd.Multiply(pointImpulseScale, c->normalImpulse2));
                    Vector128<float> newImpulse = AdvSimd.Max(AdvSimd.Subtract(c->normalImpulse2, negImpulse), Vector128<float>.Zero);
                    Vector128<float> impulse = AdvSimd.Subtract(newImpulse, c->normalImpulse2);
                    c->normalImpulse2 = newImpulse;
                    c->totalNormalImpulse2 = AdvSimd.Add(c->totalNormalImpulse2, newImpulse);
                    totalNormalImpulse = AdvSimd.Add(totalNormalImpulse, newImpulse);
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
                {
                    Vector128<float> deltaLambda = AdvSimd.Multiply(c->rollingMass, AdvSimd.Subtract(bA.w, bB.w));
                    Vector128<float> lambda = c->rollingImpulse;
                    Vector128<float> maxLambda = AdvSimd.Multiply(c->rollingResistance, totalNormalImpulse);
                    c->rollingImpulse = SymClampW(AdvSimd.Add(lambda, deltaLambda), maxLambda);
                    deltaLambda = AdvSimd.Subtract(c->rollingImpulse, lambda);
                    bA.w = AdvSimd.Subtract(bA.w, AdvSimd.Multiply(c->invIA, deltaLambda));
                    bB.w = AdvSimd.Add(bB.w, AdvSimd.Multiply(c->invIB, deltaLambda));
                }
                ScatterBodies(states, (int*)&c->indexA, ref bA);
                ScatterBodies(states, (int*)&c->indexB, ref bB);
            }
        }
    }
    public unsafe void ApplyRestitutionTask(int startIndex, int endIndex, StepContext context, int colorIndex)
    {
        var states = context.states.Data;
        var constraints = ((ContactConstraintsNeon)context.graph.colors[colorIndex].simdConstraints).simdConstraints;
        {
            Vector128<float> threshold = Vector128.Create(context.world.restitutionThreshold);
            Vector128<float> zero = Vector128<float>.Zero;
            for (int i = startIndex; i < endIndex; i++)
            {
                ContactConstraintSIMD* c = constraints + i;
                if (AllZeroW(c->restitution)) continue;
                Vector128<float> restitutionMask = AdvSimd.CompareEqual(c->restitution, zero);
                BodyStateW bA = GatherBodies(states, (int*)&c->indexA);
                BodyStateW bB = GatherBodies(states, (int*)&c->indexB);
                {
                    Vector128<float> mask1 = AdvSimd.CompareGreaterThan(AdvSimd.Add(c->relativeVelocity1, threshold), zero);
                    Vector128<float> mask2 = AdvSimd.CompareEqual(c->totalNormalImpulse1, zero);
                    Vector128<float> mask = AdvSimd.Or(AdvSimd.Or(mask1, mask2), restitutionMask);
                    Vector128<float> mass = BlendW(c->normalMass1, zero, mask);
                    Vector2W rA = c->anchorA1, rB = c->anchorB1;
                    Vector128<float> dvx = AdvSimd.Subtract(AdvSimd.Subtract(bB.v.X, AdvSimd.Multiply(bB.w, rB.Y)), AdvSimd.Subtract(bA.v.X, AdvSimd.Multiply(bA.w, rA.Y)));
                    Vector128<float> dvy = AdvSimd.Subtract(AdvSimd.Add(bB.v.Y, AdvSimd.Multiply(bB.w, rB.X)), AdvSimd.Add(bA.v.Y, AdvSimd.Multiply(bA.w, rA.X)));
                    Vector128<float> vn = AdvSimd.Add(AdvSimd.Multiply(dvx, c->normal.X), AdvSimd.Multiply(dvy, c->normal.Y));
                    Vector128<float> negImpulse = AdvSimd.Multiply(mask, AdvSimd.Add(vn, AdvSimd.Multiply(c->restitution, c->relativeVelocity1)));
                    Vector128<float> newImpulse = AdvSimd.Max(AdvSimd.Subtract(c->normalImpulse1, negImpulse), Vector128<float>.Zero);
                    Vector128<float> deltaImpulse = AdvSimd.Subtract(newImpulse, c->normalImpulse1);
                    c->normalImpulse1 = newImpulse;
                    c->totalNormalImpulse1 = AdvSimd.Add(c->totalNormalImpulse1, deltaImpulse);
                    Vector128<float> Px = AdvSimd.Multiply(deltaImpulse, c->normal.X);
                    Vector128<float> Py = AdvSimd.Multiply(deltaImpulse, c->normal.Y);
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
                    Vector128<float> mask1 = AdvSimd.CompareGreaterThan(AdvSimd.Add(c->relativeVelocity2, threshold), zero);
                    Vector128<float> mask2 = AdvSimd.CompareEqual(c->totalNormalImpulse2, zero);
                    Vector128<float> mask = AdvSimd.Or(AdvSimd.Or(mask1, mask2), restitutionMask);
                    Vector128<float> mass = BlendW(c->normalMass2, zero, mask);
                    Vector2W rA = c->anchorA2, rB = c->anchorB2;
                    Vector128<float> dvx = AdvSimd.Subtract(AdvSimd.Subtract(bB.v.X, AdvSimd.Multiply(bB.w, rB.Y)), AdvSimd.Subtract(bA.v.X, AdvSimd.Multiply(bA.w, rA.Y)));
                    Vector128<float> dvy = AdvSimd.Subtract(AdvSimd.Add(bB.v.Y, AdvSimd.Multiply(bB.w, rB.X)), AdvSimd.Add(bA.v.Y, AdvSimd.Multiply(bA.w, rA.X)));
                    Vector128<float> vn = AdvSimd.Add(AdvSimd.Multiply(dvx, c->normal.X), AdvSimd.Multiply(dvy, c->normal.Y));
                    Vector128<float> negImpulse = AdvSimd.Multiply(mask, AdvSimd.Add(vn, AdvSimd.Multiply(c->restitution, c->relativeVelocity2)));
                    Vector128<float> newImpulse = AdvSimd.Max(AdvSimd.Subtract(c->normalImpulse2, negImpulse), Vector128<float>.Zero);
                    Vector128<float> deltaImpulse = AdvSimd.Subtract(newImpulse, c->normalImpulse2);
                    c->normalImpulse2 = newImpulse;
                    c->totalNormalImpulse2 = AdvSimd.Add(c->totalNormalImpulse2, deltaImpulse);
                    Vector128<float> Px = AdvSimd.Multiply(deltaImpulse, c->normal.X);
                    Vector128<float> Py = AdvSimd.Multiply(deltaImpulse, c->normal.Y);
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
    public unsafe void StoreImpulsesTask(int startIndex, int endIndex, StepContext context)
    {
        ContactSim[] contacts = context.contacts;
        Manifold dummy = new();
        ContactConstraintSIMD* constraints = ((ContactConstraintsNeon)context.simdContactConstraints).simdConstraints;
        {
            for (int constraintIndex = startIndex; constraintIndex < endIndex; constraintIndex++)
            {
                ContactConstraintSIMD* c = constraints + constraintIndex;
                float* rollingImpulse = (float*)&c->rollingImpulse;
                float* normalImpulse1 = (float*)&c->normalImpulse1;
                float* normalImpulse2 = (float*)&c->normalImpulse2;
                float* tangentImpulse1 = (float*)&c->tangentImpulse1;
                float* tangentImpulse2 = (float*)&c->tangentImpulse2;
                float* totalNormalImpulse1 = (float*)&c->totalNormalImpulse1;
                float* totalNormalImpulse2 = (float*)&c->totalNormalImpulse2;
                float* normalVelocity1 = (float*)&c->relativeVelocity1;
                float* normalVelocity2 = (float*)&c->relativeVelocity2;
                int baseIndex = 4 * constraintIndex;
                for (int laneIndex = 0; laneIndex < 4; ++laneIndex)
                {
                    ref Manifold m = ref dummy;
                    if (contacts[baseIndex + laneIndex] != null) m = ref contacts[baseIndex + laneIndex].manifold;
                    m.rollingImpulse = rollingImpulse[laneIndex];
                    m.point0.normalImpulse = normalImpulse1[laneIndex];
                    m.point0.tangentImpulse = tangentImpulse1[laneIndex];
                    m.point0.totalNormalImpulse = totalNormalImpulse1[laneIndex];
                    m.point0.normalVelocity = normalVelocity1[laneIndex];
                    m.point1.normalImpulse = normalImpulse2[laneIndex];
                    m.point1.tangentImpulse = tangentImpulse2[laneIndex];
                    m.point1.totalNormalImpulse = totalNormalImpulse2[laneIndex];
                    m.point1.normalVelocity = normalVelocity2[laneIndex];
                }
            }
        }
    }
}
public class ContactSolverSSE : IContactSolverW
{
    public struct Vector2W
    {
        public Vector128<float> X, Y;
    }
    struct RotationW
    {
        public Vector128<float> C, S;
    }
    static Vector128<float> SymClampW(Vector128<float> a, Vector128<float> b) => Sse.Max(Sse.Subtract(Vector128<float>.Zero, b), Sse.Min(a, b));
    static bool AllZeroW(Vector128<float> a) => Sse.MoveMask(Sse.CompareEqual(a, Vector128<float>.Zero)) == 0xFF;
    static Vector128<float> BlendW(Vector128<float> a, Vector128<float> b, Vector128<float> mask) => Sse.Or(Sse.And(mask, b), Sse.AndNot(mask, a));
    static Vector128<float> DotW(Vector2W a, Vector2W b) => Sse.Add(Sse.Multiply(a.X, b.X), Sse.Multiply(a.Y, b.Y));
    static Vector128<float> CrossW(Vector2W a, Vector2W b) => Sse.Subtract(Sse.Multiply(a.X, b.Y), Sse.Multiply(a.Y, b.X));
    static Vector2W RotateVectorW(RotationW q, Vector2W v) =>
        new() { X = Sse.Subtract(Sse.Multiply(q.C, v.X), Sse.Multiply(q.S, v.Y)), Y = Sse.Add(Sse.Multiply(q.S, v.X), Sse.Multiply(q.C, v.Y)) };
    public struct ContactConstraintSIMD
    {
        public Vector128<int> indexA, indexB;
        public Vector128<float> invMassA, invMassB;
        public Vector128<float> invIA, invIB;
        public Vector2W normal;
        public Vector128<float> friction;
        public Vector128<float> tangentSpeed;
        public Vector128<float> rollingResistance;
        public Vector128<float> rollingMass;
        public Vector128<float> rollingImpulse;
        public Vector128<float> biasRate;
        public Vector128<float> massScale;
        public Vector128<float> impulseScale;
        public Vector2W anchorA1, anchorB1;
        public Vector128<float> normalMass1, tangentMass1;
        public Vector128<float> baseSeparation1;
        public Vector128<float> normalImpulse1;
        public Vector128<float> totalNormalImpulse1;
        public Vector128<float> tangentImpulse1;
        public Vector2W anchorA2, anchorB2;
        public Vector128<float> baseSeparation2;
        public Vector128<float> normalImpulse2;
        public Vector128<float> totalNormalImpulse2;
        public Vector128<float> tangentImpulse2;
        public Vector128<float> normalMass2, tangentMass2;
        public Vector128<float> restitution;
        public Vector128<float> relativeVelocity1, relativeVelocity2;
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
        Vector128<float> b1a = indices[0] == -1 ? identityA : Sse.LoadAlignedVector128((float*)(states + indices[0]));
        Vector128<float> b1b = indices[0] == -1 ? identityB : Sse.LoadAlignedVector128((float*)(states + indices[0]));
        Vector128<float> a = indices[1] == -1 ? identityA : Sse.LoadAlignedVector128((float*)(states + indices[1]));
        Vector128<float> b = indices[1] == -1 ? identityB : Sse.LoadAlignedVector128((float*)(states + indices[1]));
        Vector128<float> b3a = indices[2] == -1 ? identityA : Sse.LoadAlignedVector128((float*)(states + indices[2]));
        Vector128<float> b3b = indices[2] == -1 ? identityB : Sse.LoadAlignedVector128((float*)(states + indices[2]));
        Vector128<float> b4a = indices[3] == -1 ? identityA : Sse.LoadAlignedVector128((float*)(states + indices[3]));
        Vector128<float> b4b = indices[3] == -1 ? identityB : Sse.LoadAlignedVector128((float*)(states + indices[3]));
        Vector128<float> t1a = Sse.UnpackLow(b1a, b3a);
        Vector128<float> t2a = Sse.UnpackLow(a, b4a);
        Vector128<float> t3a = Sse.UnpackHigh(b1a, b3a);
        Vector128<float> t4a = Sse.UnpackHigh(a, b4a);
        Vector128<float> t1b = Sse.UnpackLow(b1b, b3b);
        Vector128<float> t2b = Sse.UnpackLow(b, b4b);
        Vector128<float> t3b = Sse.UnpackHigh(b1b, b3b);
        Vector128<float> t4b = Sse.UnpackHigh(b, b4b);
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
        if (indices[0] != -1 && states[indices[0]].flags.HasFlag(BodyFlags.Dynamic)) Sse.StoreAligned((float*)(states + indices[0]), Sse.Shuffle(t1, t3, 0b01000100));
        if (indices[1] != -1 && states[indices[1]].flags.HasFlag(BodyFlags.Dynamic)) Sse.StoreAligned((float*)(states + indices[1]), Sse.Shuffle(t1, t3, 0b11101110));
        if (indices[2] != -1 && states[indices[2]].flags.HasFlag(BodyFlags.Dynamic)) Sse.StoreAligned((float*)(states + indices[2]), Sse.Shuffle(t2, t4, 0b01000100));
        if (indices[3] != -1 && states[indices[3]].flags.HasFlag(BodyFlags.Dynamic)) Sse.StoreAligned((float*)(states + indices[3]), Sse.Shuffle(t2, t4, 0b11101110));
    }
    public unsafe void PrepareContactsTask(int startIndex, int endIndex, StepContext context)
    {
        World world = context.world;
        ContactSim[] contacts = context.contacts;
        var awakeStates = context.states;
        Softness contactSoftness = context.contactSoftness;
        Softness staticSoftness = context.staticSoftness;
        bool enableSoftening = world.enableContactSoftening;
        float warmStartScale = world.enableWarmStarting ? 1 : 0;
        for (int i = startIndex; i < endIndex; i++)
        {
            var constraint = ((ContactConstraintsSSE)context.simdContactConstraints).simdConstraints + i;
            for (int j = 0; j < 4; j++)
            {
                ref ContactSim contactSim = ref contacts[4 * i + j];
                if (contactSim != null)
                {
                    Manifold manifold = contactSim.manifold;
                    int indexA = contactSim.bodySimIndexA, indexB = contactSim.bodySimIndexB;

                    ((int*)&constraint->indexA)[j] = indexA;
                    ((int*)&constraint->indexB)[j] = indexB;
                    Vector2 vA = Vector2.Zero;
                    float wA = 0, mA = contactSim.invMassA, iA = contactSim.invIA;
                    if (indexA != -1)
                    {
                        vA = awakeStates[indexA].linearVelocity;
                        wA = awakeStates[indexA].angularVelocity;
                    }
                    Vector2 vB = Vector2.Zero;
                    float wB = 0, mB = contactSim.invMassB, iB = contactSim.invIB;
                    if (indexB != -1)
                    {
                        vB = awakeStates[indexB].linearVelocity;
                        wB = awakeStates[indexB].angularVelocity;
                    }
                    ((float*)&constraint->invMassA)[j] = mA;
                    ((float*)&constraint->invMassB)[j] = mB;
                    ((float*)&constraint->invIA)[j] = iA;
                    ((float*)&constraint->invIB)[j] = iB;
                    {
                        float k = iA + iB;
                        ((float*)&constraint->rollingMass)[j] = k > 0.0f ? 1.0f / k : 0.0f;
                    }
                    Softness soft = contactSoftness;
                    if (indexA == -1 || indexB == -1) soft = staticSoftness;
                    else if (enableSoftening)
                    {
                        float contactHertz = Math.Min(world.contactHertz, 0.125f * context.inv_h);
                        float ratio = 1;
                        if (mA < mB) ratio = Math.Max(0.5f, mA / mB);
                        else if (mB < mA) ratio = Math.Max(0.5f, mB / mA);
                        soft = new(ratio * contactHertz, ratio * world.contactDampingRatio, context.h);
                    }

                    Vector2 normal = manifold.normal;
                    ((float*)&constraint->normal.X)[j] = normal.x;
                    ((float*)&constraint->normal.Y)[j] = normal.y;

                    ((float*)&constraint->friction)[j] = contactSim.friction;
                    ((float*)&constraint->tangentSpeed)[j] = contactSim.tangentSpeed;
                    ((float*)&constraint->restitution)[j] = contactSim.restitution;
                    ((float*)&constraint->rollingResistance)[j] = contactSim.rollingResistance;
                    ((float*)&constraint->rollingImpulse)[j] = warmStartScale * manifold.rollingImpulse;

                    ((float*)&constraint->biasRate)[j] = soft.biasRate;
                    ((float*)&constraint->massScale)[j] = soft.massScale;
                    ((float*)&constraint->impulseScale)[j] = soft.impulseScale;

                    Vector2 tangent = normal.RightPerp();

                    {
                        ref ManifoldPoint mp = ref manifold.point0;

                        Vector2 rA = mp.anchorA;
                        Vector2 rB = mp.anchorB;

                        ((float*)&constraint->anchorA1.X)[j] = rA.x;
                        ((float*)&constraint->anchorA1.Y)[j] = rA.y;
                        ((float*)&constraint->anchorB1.X)[j] = rB.x;
                        ((float*)&constraint->anchorB1.Y)[j] = rB.y;

                        ((float*)&constraint->baseSeparation1)[j] = mp.separation - Vector2.Dot(rB - rA, normal);

                        ((float*)&constraint->normalImpulse1)[j] = warmStartScale * mp.normalImpulse;
                        ((float*)&constraint->tangentImpulse1)[j] = warmStartScale * mp.tangentImpulse;
                        ((float*)&constraint->totalNormalImpulse1)[j] = 0.0f;

                        float rnA = Vector2.Cross(rA, normal);
                        float rnB = Vector2.Cross(rB, normal);
                        float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                        ((float*)&constraint->normalMass1)[j] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

                        float rtA = Vector2.Cross(rA, tangent);
                        float rtB = Vector2.Cross(rB, tangent);
                        float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
                        ((float*)&constraint->tangentMass1)[j] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

                        // relative velocity for restitution
                        Vector2 vrA = vA + Vector2.CrossSV(wA, rA);
                        Vector2 vrB = vB + Vector2.CrossSV(wB, rB);
                        ((float*)&constraint->relativeVelocity1)[j] = Vector2.Dot(normal, vrB - vrA);
                    }

                    int pointCount = manifold.pointCount;
                    Debug.Assert(0 < pointCount && pointCount <= 2);

                    if (pointCount == 2)
                    {
                        ref ManifoldPoint mp = ref manifold.point1;

                        Vector2 rA = mp.anchorA;
                        Vector2 rB = mp.anchorB;

                        ((float*)&constraint->anchorA2.X)[j] = rA.x;
                        ((float*)&constraint->anchorA2.Y)[j] = rA.y;
                        ((float*)&constraint->anchorB2.X)[j] = rB.x;
                        ((float*)&constraint->anchorB2.Y)[j] = rB.y;

                        ((float*)&constraint->baseSeparation2)[j] = mp.separation - Vector2.Dot(rB - rA, normal);

                        ((float*)&constraint->normalImpulse2)[j] = warmStartScale * mp.normalImpulse;
                        ((float*)&constraint->tangentImpulse2)[j] = warmStartScale * mp.tangentImpulse;
                        ((float*)&constraint->totalNormalImpulse2)[j] = 0.0f;

                        float rnA = Vector2.Cross(rA, normal);
                        float rnB = Vector2.Cross(rB, normal);
                        float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                        ((float*)&constraint->normalMass2)[j] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

                        float rtA = Vector2.Cross(rA, tangent);
                        float rtB = Vector2.Cross(rB, tangent);
                        float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
                        ((float*)&constraint->tangentMass2)[j] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

                        // relative velocity for restitution
                        Vector2 vrA = vA + Vector2.CrossSV(wA, rA);
                        Vector2 vrB = vB + Vector2.CrossSV(wB, rB);
                        ((float*)&constraint->relativeVelocity2)[j] = Vector2.Dot(normal, vrB - vrA);
                    }
                    else
                    {
                        // dummy data that has no effect
                        ((float*)&constraint->baseSeparation2)[j] = 0.0f;
                        ((float*)&constraint->normalImpulse2)[j] = 0.0f;
                        ((float*)&constraint->tangentImpulse2)[j] = 0.0f;
                        ((float*)&constraint->totalNormalImpulse2)[j] = 0.0f;
                        ((float*)&constraint->anchorA2.X)[j] = 0.0f;
                        ((float*)&constraint->anchorA2.Y)[j] = 0.0f;
                        ((float*)&constraint->anchorB2.X)[j] = 0.0f;
                        ((float*)&constraint->anchorB2.Y)[j] = 0.0f;
                        ((float*)&constraint->normalMass2)[j] = 0.0f;
                        ((float*)&constraint->tangentMass2)[j] = 0.0f;
                        ((float*)&constraint->relativeVelocity2)[j] = 0.0f;
                    }
                }
                else
                {
                    ((int*)&constraint->indexA)[j] = -1;
                    ((int*)&constraint->indexB)[j] = -1;

                    ((float*)&constraint->invMassA)[j] = 0.0f;
                    ((float*)&constraint->invMassB)[j] = 0.0f;
                    ((float*)&constraint->invIA)[j] = 0.0f;
                    ((float*)&constraint->invIB)[j] = 0.0f;

                    ((float*)&constraint->normal.X)[j] = 0.0f;
                    ((float*)&constraint->normal.Y)[j] = 0.0f;
                    ((float*)&constraint->friction)[j] = 0.0f;
                    ((float*)&constraint->tangentSpeed)[j] = 0.0f;
                    ((float*)&constraint->rollingResistance)[j] = 0.0f;
                    ((float*)&constraint->rollingMass)[j] = 0.0f;
                    ((float*)&constraint->rollingImpulse)[j] = 0.0f;
                    ((float*)&constraint->biasRate)[j] = 0.0f;
                    ((float*)&constraint->massScale)[j] = 0.0f;
                    ((float*)&constraint->impulseScale)[j] = 0.0f;

                    ((float*)&constraint->anchorA1.X)[j] = 0.0f;
                    ((float*)&constraint->anchorA1.Y)[j] = 0.0f;
                    ((float*)&constraint->anchorB1.X)[j] = 0.0f;
                    ((float*)&constraint->anchorB1.Y)[j] = 0.0f;
                    ((float*)&constraint->baseSeparation1)[j] = 0.0f;
                    ((float*)&constraint->normalImpulse1)[j] = 0.0f;
                    ((float*)&constraint->tangentImpulse1)[j] = 0.0f;
                    ((float*)&constraint->totalNormalImpulse1)[j] = 0.0f;
                    ((float*)&constraint->normalMass1)[j] = 0.0f;
                    ((float*)&constraint->tangentMass1)[j] = 0.0f;

                    ((float*)&constraint->anchorA2.X)[j] = 0.0f;
                    ((float*)&constraint->anchorA2.Y)[j] = 0.0f;
                    ((float*)&constraint->anchorB2.X)[j] = 0.0f;
                    ((float*)&constraint->anchorB2.Y)[j] = 0.0f;
                    ((float*)&constraint->baseSeparation2)[j] = 0.0f;
                    ((float*)&constraint->normalImpulse2)[j] = 0.0f;
                    ((float*)&constraint->tangentImpulse2)[j] = 0.0f;
                    ((float*)&constraint->totalNormalImpulse2)[j] = 0.0f;
                    ((float*)&constraint->normalMass2)[j] = 0.0f;
                    ((float*)&constraint->tangentMass2)[j] = 0.0f;

                    ((float*)&constraint->restitution)[j] = 0.0f;
                    ((float*)&constraint->relativeVelocity1)[j] = 0.0f;
                    ((float*)&constraint->relativeVelocity2)[j] = 0.0f;
                }
            }
        }
    }
    public unsafe void WarmStartContactsTask(int startIndex, int endIndex, StepContext context, int colorIndex)
    {
        var states = context.states.Data;
        var constraints = ((ContactConstraintsSSE)context.graph.colors[colorIndex].simdConstraints).simdConstraints;
        {
            for (int i = startIndex; i < endIndex; i++)
            {
                ContactConstraintSIMD* c = constraints + i;
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
                }
                bA.w = Sse.Subtract(bA.w, Sse.Multiply(c->invIA, c->rollingImpulse));
                bB.w = Sse.Add(bB.w, Sse.Multiply(c->invIB, c->rollingImpulse));
                ScatterBodies(states, (int*)&c->indexA, ref bA);
                ScatterBodies(states, (int*)&c->indexB, ref bB);
            }
        }
    }
    public unsafe void SolveContactsTask(int startIndex, int endIndex, StepContext context, int colorIndex, bool useBias)
    {
        var states = context.states.Data;
        var constraints = ((ContactConstraintsSSE)context.graph.colors[colorIndex].simdConstraints).simdConstraints;
        {
            Vector128<float> inv_h = Vector128.Create(context.inv_h);
            Vector128<float> contactSpeed = Vector128.Create(-context.world.contactSpeed);
            Vector128<float> oneW = Vector128<float>.One;
            for (int i = startIndex; i < endIndex; i++)
            {
                ContactConstraintSIMD* c = constraints + i;
                BodyStateW bA = GatherBodies(states, (int*)&c->indexA);
                BodyStateW bB = GatherBodies(states, (int*)&c->indexB);
                Vector128<float> biasRate, massScale, impulseScale;
                if (useBias)
                {
                    biasRate = Sse.Multiply(c->massScale, c->biasRate);
                    massScale = c->massScale;
                    impulseScale = c->impulseScale;
                }
                else
                {
                    biasRate = Vector128<float>.Zero;
                    massScale = oneW;
                    impulseScale = Vector128<float>.Zero;
                }
                Vector128<float> totalNormalImpulse = Vector128<float>.Zero;
                Vector2W dp = new() { X = Sse.Subtract(bB.dp.X, bA.dp.X), Y = Sse.Subtract(bB.dp.Y, bA.dp.Y) };
                {
                    Vector2W rA = c->anchorA1, rB = c->anchorB1;
                    Vector2W rsA = RotateVectorW(bA.dq, rA), rsB = RotateVectorW(bB.dq, rB);
                    Vector2W ds = new() { X = Sse.Add(dp.X, Sse.Subtract(rsB.X, rsA.X)), Y = Sse.Add(dp.Y, Sse.Subtract(rsB.Y, rsA.Y)) };
                    Vector128<float> s = Sse.Add(DotW(c->normal, ds), c->baseSeparation1);
                    Vector128<float> mask = Sse.CompareGreaterThan(s, Vector128<float>.Zero);
                    Vector128<float> specBias = Sse.Multiply(s, inv_h), softBias = Sse.Max(Sse.Multiply(biasRate, s), contactSpeed);
                    Vector128<float> bias = BlendW(softBias, specBias, mask);
                    Vector128<float> pointMassScale = BlendW(massScale, oneW, mask);
                    Vector128<float> pointImpulseScale = BlendW(impulseScale, Vector128<float>.Zero, mask);
                    Vector128<float> dvx = Sse.Subtract(Sse.Subtract(bB.v.X, Sse.Multiply(bB.w, rB.Y)), Sse.Subtract(bA.v.X, Sse.Multiply(bA.w, rA.Y)));
                    Vector128<float> dvy = Sse.Subtract(Sse.Add(bB.v.Y, Sse.Multiply(bB.w, rB.X)), Sse.Add(bA.v.Y, Sse.Multiply(bA.w, rA.X)));
                    Vector128<float> vn = Sse.Add(Sse.Multiply(dvx, c->normal.X), Sse.Multiply(dvy, c->normal.Y));
                    Vector128<float> negImpulse = Sse.Add(Sse.Multiply(c->normalMass1, Sse.Add(Sse.Multiply(pointMassScale, vn), bias)), Sse.Multiply(pointImpulseScale, c->normalImpulse1));
                    Vector128<float> newImpulse = Sse.Max(Sse.Subtract(c->normalImpulse1, negImpulse), Vector128<float>.Zero);
                    Vector128<float> impulse = Sse.Subtract(newImpulse, c->normalImpulse1);
                    c->normalImpulse1 = newImpulse;
                    c->totalNormalImpulse1 = Sse.Add(c->totalNormalImpulse1, newImpulse);
                    totalNormalImpulse = Sse.Add(totalNormalImpulse, newImpulse);
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
                    Vector128<float> mask = Sse.CompareGreaterThan(s, Vector128<float>.Zero);
                    Vector128<float> specBias = Sse.Multiply(s, inv_h), softBias = Sse.Max(Sse.Multiply(biasRate, s), contactSpeed);
                    Vector128<float> bias = BlendW(softBias, specBias, mask);
                    Vector128<float> pointMassScale = BlendW(massScale, oneW, mask);
                    Vector128<float> pointImpulseScale = BlendW(impulseScale, Vector128<float>.Zero, mask);
                    Vector128<float> dvx = Sse.Subtract(Sse.Subtract(bB.v.X, Sse.Multiply(bB.w, rB.Y)), Sse.Subtract(bA.v.X, Sse.Multiply(bA.w, rA.Y)));
                    Vector128<float> dvy = Sse.Subtract(Sse.Add(bB.v.Y, Sse.Multiply(bB.w, rB.X)), Sse.Add(bA.v.Y, Sse.Multiply(bA.w, rA.X)));
                    Vector128<float> vn = Sse.Add(Sse.Multiply(dvx, c->normal.X), Sse.Multiply(dvy, c->normal.Y));
                    //different than 1, is this intended?
                    Vector128<float> negImpulse = Sse.Add(Sse.Multiply(c->normalMass2, Sse.Add(Sse.Multiply(pointMassScale, vn), bias)), Sse.Multiply(pointImpulseScale, c->normalImpulse2));
                    Vector128<float> newImpulse = Sse.Max(Sse.Subtract(c->normalImpulse2, negImpulse), Vector128<float>.Zero);
                    Vector128<float> impulse = Sse.Subtract(newImpulse, c->normalImpulse2);
                    c->normalImpulse2 = newImpulse;
                    c->totalNormalImpulse2 = Sse.Add(c->totalNormalImpulse2, newImpulse);
                    totalNormalImpulse = Sse.Add(totalNormalImpulse, newImpulse);
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
                {
                    Vector128<float> deltaLambda = Sse.Multiply(c->rollingMass, Sse.Subtract(bA.w, bB.w));
                    Vector128<float> lambda = c->rollingImpulse;
                    Vector128<float> maxLambda = Sse.Multiply(c->rollingResistance, totalNormalImpulse);
                    c->rollingImpulse = SymClampW(Sse.Add(lambda, deltaLambda), maxLambda);
                    deltaLambda = Sse.Subtract(c->rollingImpulse, lambda);
                    bA.w = Sse.Subtract(bA.w, Sse.Multiply(c->invIA, deltaLambda));
                    bB.w = Sse.Add(bB.w, Sse.Multiply(c->invIB, deltaLambda));
                }
                ScatterBodies(states, (int*)&c->indexA, ref bA);
                ScatterBodies(states, (int*)&c->indexB, ref bB);
            }
        }
    }
    public unsafe void ApplyRestitutionTask(int startIndex, int endIndex, StepContext context, int colorIndex)
    {
        var states = context.states.Data;
        var constraints = ((ContactConstraintsSSE)context.graph.colors[colorIndex].simdConstraints).simdConstraints;
        {
            Vector128<float> threshold = Vector128.Create(context.world.restitutionThreshold);
            Vector128<float> zero = Vector128<float>.Zero;
            for (int i = startIndex; i < endIndex; i++)
            {
                ContactConstraintSIMD* c = constraints + i;
                if (AllZeroW(c->restitution)) continue;
                Vector128<float> restitutionMask = Sse.CompareEqual(c->restitution, zero);
                BodyStateW bA = GatherBodies(states, (int*)&c->indexA);
                BodyStateW bB = GatherBodies(states, (int*)&c->indexB);
                {
                    Vector128<float> mask1 = Sse.CompareGreaterThan(Sse.Add(c->relativeVelocity1, threshold), zero);
                    Vector128<float> mask2 = Sse.CompareEqual(c->totalNormalImpulse1, zero);
                    Vector128<float> mask = Sse.Or(Sse.Or(mask1, mask2), restitutionMask);
                    Vector128<float> mass = BlendW(c->normalMass1, zero, mask);
                    Vector2W rA = c->anchorA1, rB = c->anchorB1;
                    Vector128<float> dvx = Sse.Subtract(Sse.Subtract(bB.v.X, Sse.Multiply(bB.w, rB.Y)), Sse.Subtract(bA.v.X, Sse.Multiply(bA.w, rA.Y)));
                    Vector128<float> dvy = Sse.Subtract(Sse.Add(bB.v.Y, Sse.Multiply(bB.w, rB.X)), Sse.Add(bA.v.Y, Sse.Multiply(bA.w, rA.X)));
                    Vector128<float> vn = Sse.Add(Sse.Multiply(dvx, c->normal.X), Sse.Multiply(dvy, c->normal.Y));
                    Vector128<float> negImpulse = Sse.Multiply(mask, Sse.Add(vn, Sse.Multiply(c->restitution, c->relativeVelocity1)));
                    Vector128<float> newImpulse = Sse.Max(Sse.Subtract(c->normalImpulse1, negImpulse), Vector128<float>.Zero);
                    Vector128<float> deltaImpulse = Sse.Subtract(newImpulse, c->normalImpulse1);
                    c->normalImpulse1 = newImpulse;
                    c->totalNormalImpulse1 = Sse.Add(c->totalNormalImpulse1, deltaImpulse);
                    Vector128<float> Px = Sse.Multiply(deltaImpulse, c->normal.X);
                    Vector128<float> Py = Sse.Multiply(deltaImpulse, c->normal.Y);
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
                    Vector128<float> mask1 = Sse.CompareGreaterThan(Sse.Add(c->relativeVelocity2, threshold), zero);
                    Vector128<float> mask2 = Sse.CompareEqual(c->totalNormalImpulse2, zero);
                    Vector128<float> mask = Sse.Or(Sse.Or(mask1, mask2), restitutionMask);
                    Vector128<float> mass = BlendW(c->normalMass2, zero, mask);
                    Vector2W rA = c->anchorA2, rB = c->anchorB2;
                    Vector128<float> dvx = Sse.Subtract(Sse.Subtract(bB.v.X, Sse.Multiply(bB.w, rB.Y)), Sse.Subtract(bA.v.X, Sse.Multiply(bA.w, rA.Y)));
                    Vector128<float> dvy = Sse.Subtract(Sse.Add(bB.v.Y, Sse.Multiply(bB.w, rB.X)), Sse.Add(bA.v.Y, Sse.Multiply(bA.w, rA.X)));
                    Vector128<float> vn = Sse.Add(Sse.Multiply(dvx, c->normal.X), Sse.Multiply(dvy, c->normal.Y));
                    Vector128<float> negImpulse = Sse.Multiply(mask, Sse.Add(vn, Sse.Multiply(c->restitution, c->relativeVelocity2)));
                    Vector128<float> newImpulse = Sse.Max(Sse.Subtract(c->normalImpulse2, negImpulse), Vector128<float>.Zero);
                    Vector128<float> deltaImpulse = Sse.Subtract(newImpulse, c->normalImpulse2);
                    c->normalImpulse2 = newImpulse;
                    c->totalNormalImpulse2 = Sse.Add(c->totalNormalImpulse2, deltaImpulse);
                    Vector128<float> Px = Sse.Multiply(deltaImpulse, c->normal.X);
                    Vector128<float> Py = Sse.Multiply(deltaImpulse, c->normal.Y);
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
    public unsafe void StoreImpulsesTask(int startIndex, int endIndex, StepContext context)
    {
        ContactSim[] contacts = context.contacts;
        Manifold dummy = new();
        ContactConstraintSIMD* constraints = ((ContactConstraintsSSE)context.simdContactConstraints).simdConstraints;
        {
            for (int constraintIndex = startIndex; constraintIndex < endIndex; constraintIndex++)
            {
                ContactConstraintSIMD* c = constraints + constraintIndex;
                float* rollingImpulse = (float*)&c->rollingImpulse;
                float* normalImpulse1 = (float*)&c->normalImpulse1;
                float* normalImpulse2 = (float*)&c->normalImpulse2;
                float* tangentImpulse1 = (float*)&c->tangentImpulse1;
                float* tangentImpulse2 = (float*)&c->tangentImpulse2;
                float* totalNormalImpulse1 = (float*)&c->totalNormalImpulse1;
                float* totalNormalImpulse2 = (float*)&c->totalNormalImpulse2;
                float* normalVelocity1 = (float*)&c->relativeVelocity1;
                float* normalVelocity2 = (float*)&c->relativeVelocity2;
                int baseIndex = 4 * constraintIndex;
                for (int laneIndex = 0; laneIndex < 4; ++laneIndex)
                {
                    ref Manifold m = ref dummy;
                    if (contacts[baseIndex + laneIndex] != null) m = ref contacts[baseIndex + laneIndex].manifold;
                    m.rollingImpulse = rollingImpulse[laneIndex];
                    m.point0.normalImpulse = normalImpulse1[laneIndex];
                    m.point0.tangentImpulse = tangentImpulse1[laneIndex];
                    m.point0.totalNormalImpulse = totalNormalImpulse1[laneIndex];
                    m.point0.normalVelocity = normalVelocity1[laneIndex];
                    m.point1.normalImpulse = normalImpulse2[laneIndex];
                    m.point1.tangentImpulse = tangentImpulse2[laneIndex];
                    m.point1.totalNormalImpulse = totalNormalImpulse2[laneIndex];
                    m.point1.normalVelocity = normalVelocity2[laneIndex];
                }
            }
        }
    }
}
public class ContactSolverFloat : IContactSolverW
{
    public struct FloatW
    {
        public float x, y, z, w;
        public static readonly FloatW Zero = new();
        public FloatW(float scalar) { x = y = z = w = scalar; }
        public FloatW(float x, float y, float z, float w) { this.x = x; this.y = y; this.z = z; this.w = w; }
        public static FloatW operator +(FloatW a, FloatW b) => new(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
        public static FloatW operator -(FloatW a, FloatW b) => new(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
        public static FloatW operator *(FloatW a, FloatW b) => new(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w);
    }
    public struct Vector2W
    {
        public FloatW X, Y;
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
    static FloatW GreaterThanW(FloatW a, FloatW b) => new(a.x > b.x ? 1 : 0, a.y > b.y ? 1 : 0, a.z > b.z ? 1 : 0, a.w > b.w ? 1 : 0);
    static FloatW EqualsW(FloatW a, FloatW b) => new(a.x == b.x ? 1 : 0, a.y == b.y ? 1 : 0, a.z == b.z ? 1 : 0, a.w == b.w ? 1 : 0);
    static bool AllZeroW(FloatW a) => a.x == 0 && a.y == 0 && a.z == 0 && a.w == 0;
    static FloatW BlendW(FloatW a, FloatW b, FloatW mask) =>
        new(mask.x != 0 ? b.x : a.x, mask.y != 0 ? b.y : a.y, mask.z != 0 ? b.z : a.z, mask.w != 0 ? b.w : a.w);
    static FloatW DotW(Vector2W a, Vector2W b) => a.X * b.X + a.Y * b.Y;
    static FloatW CrossW(Vector2W a, Vector2W b) => a.X * b.Y - a.Y * b.X;
    static Vector2W RotateVectorW(RotationW q, Vector2W v) => new() { X = q.C * v.X - q.S * v.Y, Y = q.S * v.X + q.C * v.Y };
    public struct ContactConstraintSIMD
    {
        public Vector128<int> indexA, indexB;
        public FloatW invMassA, invMassB;
        public FloatW invIA, invIB;
        public Vector2W normal;
        public FloatW friction;
        public FloatW tangentSpeed;
        public FloatW rollingResistance;
        public FloatW rollingMass;
        public FloatW rollingImpulse;
        public FloatW biasRate;
        public FloatW massScale;
        public FloatW impulseScale;
        public Vector2W anchorA1, anchorB1;
        public FloatW normalMass1, tangentMass1;
        public FloatW baseSeparation1;
        public FloatW normalImpulse1;
        public FloatW totalNormalImpulse1;
        public FloatW tangentImpulse1;
        public Vector2W anchorA2, anchorB2;
        public FloatW baseSeparation2;
        public FloatW normalImpulse2;
        public FloatW totalNormalImpulse2;
        public FloatW tangentImpulse2;
        public FloatW normalMass2, tangentMass2;
        public FloatW restitution;
        public FloatW relativeVelocity1, relativeVelocity2;
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
        BodyState identity = new();
        BodyState s1 = indices[0] == -1 ? identity : states[indices[0]];
        BodyState s2 = indices[1] == -1 ? identity : states[indices[1]];
        BodyState s3 = indices[2] == -1 ? identity : states[indices[2]];
        BodyState s4 = indices[3] == -1 ? identity : states[indices[3]];

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
        if (indices[0] != -1 && states[indices[0]].flags.HasFlag(BodyFlags.Dynamic))
        {
            BodyState* state = states + indices[0];
            state->linearVelocity.x = simdBody.v.X.x;
            state->linearVelocity.y = simdBody.v.Y.x;
            state->angularVelocity = simdBody.w.x;
        }
        if (indices[1] != -1 && states[indices[1]].flags.HasFlag(BodyFlags.Dynamic))
        {
            BodyState* state = states + indices[1];
            state->linearVelocity.x = simdBody.v.X.y;
            state->linearVelocity.y = simdBody.v.Y.y;
            state->angularVelocity = simdBody.w.y;
        }
        if (indices[2] != -1 && states[indices[2]].flags.HasFlag(BodyFlags.Dynamic))
        {
            BodyState* state = states + indices[2];
            state->linearVelocity.x = simdBody.v.X.z;
            state->linearVelocity.y = simdBody.v.Y.z;
            state->angularVelocity = simdBody.w.z;
        }
        if (indices[3] != -1 && states[indices[3]].flags.HasFlag(BodyFlags.Dynamic))
        {
            BodyState* state = states + indices[3];
            state->linearVelocity.x = simdBody.v.X.w;
            state->linearVelocity.y = simdBody.v.Y.w;
            state->angularVelocity = simdBody.w.w;
        }
    }
    public unsafe void PrepareContactsTask(int startIndex, int endIndex, StepContext context)
    {
        World world = context.world;
        ContactSim[] contacts = context.contacts;
        var awakeStates = context.states;
        Softness contactSoftness = context.contactSoftness;
        Softness staticSoftness = context.staticSoftness;
        bool enableSoftening = world.enableContactSoftening;
        float warmStartScale = world.enableWarmStarting ? 1 : 0;
        for (int i = startIndex; i < endIndex; i++)
        {
            var constraint = ((ContactConstraintsFloat)context.simdContactConstraints).simdConstraints + i;
            for (int j = 0; j < 4; j++)
            {
                ref ContactSim contactSim = ref contacts[4 * i + j];
                if (contactSim != null)
                {
                    Manifold manifold = contactSim.manifold;
                    int indexA = contactSim.bodySimIndexA, indexB = contactSim.bodySimIndexB;

                    ((int*)&constraint->indexA)[j] = indexA;
                    ((int*)&constraint->indexB)[j] = indexB;
                    Vector2 vA = Vector2.Zero;
                    float wA = 0, mA = contactSim.invMassA, iA = contactSim.invIA;
                    if (indexA != -1)
                    {
                        vA = awakeStates[indexA].linearVelocity;
                        wA = awakeStates[indexA].angularVelocity;
                    }
                    Vector2 vB = Vector2.Zero;
                    float wB = 0, mB = contactSim.invMassB, iB = contactSim.invIB;
                    if (indexB != -1)
                    {
                        vB = awakeStates[indexB].linearVelocity;
                        wB = awakeStates[indexB].angularVelocity;
                    }
                    ((float*)&constraint->invMassA)[j] = mA;
                    ((float*)&constraint->invMassB)[j] = mB;
                    ((float*)&constraint->invIA)[j] = iA;
                    ((float*)&constraint->invIB)[j] = iB;
                    {
                        float k = iA + iB;
                        ((float*)&constraint->rollingMass)[j] = k > 0.0f ? 1.0f / k : 0.0f;
                    }
                    Softness soft = contactSoftness;
                    if (indexA == -1 || indexB == -1) soft = staticSoftness;
                    else if (enableSoftening)
                    {
                        float contactHertz = Math.Min(world.contactHertz, 0.125f * context.inv_h);
                        float ratio = 1;
                        if (mA < mB) ratio = Math.Max(0.5f, mA / mB);
                        else if (mB < mA) ratio = Math.Max(0.5f, mB / mA);
                        soft = new(ratio * contactHertz, ratio * world.contactDampingRatio, context.h);
                    }

                    Vector2 normal = manifold.normal;
                    ((float*)&constraint->normal.X)[j] = normal.x;
                    ((float*)&constraint->normal.Y)[j] = normal.y;

                    ((float*)&constraint->friction)[j] = contactSim.friction;
                    ((float*)&constraint->tangentSpeed)[j] = contactSim.tangentSpeed;
                    ((float*)&constraint->restitution)[j] = contactSim.restitution;
                    ((float*)&constraint->rollingResistance)[j] = contactSim.rollingResistance;
                    ((float*)&constraint->rollingImpulse)[j] = warmStartScale * manifold.rollingImpulse;

                    ((float*)&constraint->biasRate)[j] = soft.biasRate;
                    ((float*)&constraint->massScale)[j] = soft.massScale;
                    ((float*)&constraint->impulseScale)[j] = soft.impulseScale;

                    Vector2 tangent = normal.RightPerp();

                    {
                        ref ManifoldPoint mp = ref manifold.point0;

                        Vector2 rA = mp.anchorA;
                        Vector2 rB = mp.anchorB;

                        ((float*)&constraint->anchorA1.X)[j] = rA.x;
                        ((float*)&constraint->anchorA1.Y)[j] = rA.y;
                        ((float*)&constraint->anchorB1.X)[j] = rB.x;
                        ((float*)&constraint->anchorB1.Y)[j] = rB.y;

                        ((float*)&constraint->baseSeparation1)[j] = mp.separation - Vector2.Dot(rB - rA, normal);

                        ((float*)&constraint->normalImpulse1)[j] = warmStartScale * mp.normalImpulse;
                        ((float*)&constraint->tangentImpulse1)[j] = warmStartScale * mp.tangentImpulse;
                        ((float*)&constraint->totalNormalImpulse1)[j] = 0.0f;

                        float rnA = Vector2.Cross(rA, normal);
                        float rnB = Vector2.Cross(rB, normal);
                        float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                        ((float*)&constraint->normalMass1)[j] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

                        float rtA = Vector2.Cross(rA, tangent);
                        float rtB = Vector2.Cross(rB, tangent);
                        float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
                        ((float*)&constraint->tangentMass1)[j] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

                        // relative velocity for restitution
                        Vector2 vrA = vA + Vector2.CrossSV(wA, rA);
                        Vector2 vrB = vB + Vector2.CrossSV(wB, rB);
                        ((float*)&constraint->relativeVelocity1)[j] = Vector2.Dot(normal, vrB - vrA);
                    }

                    int pointCount = manifold.pointCount;
                    Debug.Assert(0 < pointCount && pointCount <= 2);

                    if (pointCount == 2)
                    {
                        ref ManifoldPoint mp = ref manifold.point1;

                        Vector2 rA = mp.anchorA;
                        Vector2 rB = mp.anchorB;

                        ((float*)&constraint->anchorA2.X)[j] = rA.x;
                        ((float*)&constraint->anchorA2.Y)[j] = rA.y;
                        ((float*)&constraint->anchorB2.X)[j] = rB.x;
                        ((float*)&constraint->anchorB2.Y)[j] = rB.y;

                        ((float*)&constraint->baseSeparation2)[j] = mp.separation - Vector2.Dot(rB - rA, normal);

                        ((float*)&constraint->normalImpulse2)[j] = warmStartScale * mp.normalImpulse;
                        ((float*)&constraint->tangentImpulse2)[j] = warmStartScale * mp.tangentImpulse;
                        ((float*)&constraint->totalNormalImpulse2)[j] = 0.0f;

                        float rnA = Vector2.Cross(rA, normal);
                        float rnB = Vector2.Cross(rB, normal);
                        float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                        ((float*)&constraint->normalMass2)[j] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

                        float rtA = Vector2.Cross(rA, tangent);
                        float rtB = Vector2.Cross(rB, tangent);
                        float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
                        ((float*)&constraint->tangentMass2)[j] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

                        // relative velocity for restitution
                        Vector2 vrA = vA + Vector2.CrossSV(wA, rA);
                        Vector2 vrB = vB + Vector2.CrossSV(wB, rB);
                        ((float*)&constraint->relativeVelocity2)[j] = Vector2.Dot(normal, vrB - vrA);
                    }
                    else
                    {
                        // dummy data that has no effect
                        ((float*)&constraint->baseSeparation2)[j] = 0.0f;
                        ((float*)&constraint->normalImpulse2)[j] = 0.0f;
                        ((float*)&constraint->tangentImpulse2)[j] = 0.0f;
                        ((float*)&constraint->totalNormalImpulse2)[j] = 0.0f;
                        ((float*)&constraint->anchorA2.X)[j] = 0.0f;
                        ((float*)&constraint->anchorA2.Y)[j] = 0.0f;
                        ((float*)&constraint->anchorB2.X)[j] = 0.0f;
                        ((float*)&constraint->anchorB2.Y)[j] = 0.0f;
                        ((float*)&constraint->normalMass2)[j] = 0.0f;
                        ((float*)&constraint->tangentMass2)[j] = 0.0f;
                        ((float*)&constraint->relativeVelocity2)[j] = 0.0f;
                    }
                }
                else
                {
                    ((int*)&constraint->indexA)[j] = -1;
                    ((int*)&constraint->indexB)[j] = -1;

                    ((float*)&constraint->invMassA)[j] = 0.0f;
                    ((float*)&constraint->invMassB)[j] = 0.0f;
                    ((float*)&constraint->invIA)[j] = 0.0f;
                    ((float*)&constraint->invIB)[j] = 0.0f;

                    ((float*)&constraint->normal.X)[j] = 0.0f;
                    ((float*)&constraint->normal.Y)[j] = 0.0f;
                    ((float*)&constraint->friction)[j] = 0.0f;
                    ((float*)&constraint->tangentSpeed)[j] = 0.0f;
                    ((float*)&constraint->rollingResistance)[j] = 0.0f;
                    ((float*)&constraint->rollingMass)[j] = 0.0f;
                    ((float*)&constraint->rollingImpulse)[j] = 0.0f;
                    ((float*)&constraint->biasRate)[j] = 0.0f;
                    ((float*)&constraint->massScale)[j] = 0.0f;
                    ((float*)&constraint->impulseScale)[j] = 0.0f;

                    ((float*)&constraint->anchorA1.X)[j] = 0.0f;
                    ((float*)&constraint->anchorA1.Y)[j] = 0.0f;
                    ((float*)&constraint->anchorB1.X)[j] = 0.0f;
                    ((float*)&constraint->anchorB1.Y)[j] = 0.0f;
                    ((float*)&constraint->baseSeparation1)[j] = 0.0f;
                    ((float*)&constraint->normalImpulse1)[j] = 0.0f;
                    ((float*)&constraint->tangentImpulse1)[j] = 0.0f;
                    ((float*)&constraint->totalNormalImpulse1)[j] = 0.0f;
                    ((float*)&constraint->normalMass1)[j] = 0.0f;
                    ((float*)&constraint->tangentMass1)[j] = 0.0f;

                    ((float*)&constraint->anchorA2.X)[j] = 0.0f;
                    ((float*)&constraint->anchorA2.Y)[j] = 0.0f;
                    ((float*)&constraint->anchorB2.X)[j] = 0.0f;
                    ((float*)&constraint->anchorB2.Y)[j] = 0.0f;
                    ((float*)&constraint->baseSeparation2)[j] = 0.0f;
                    ((float*)&constraint->normalImpulse2)[j] = 0.0f;
                    ((float*)&constraint->tangentImpulse2)[j] = 0.0f;
                    ((float*)&constraint->totalNormalImpulse2)[j] = 0.0f;
                    ((float*)&constraint->normalMass2)[j] = 0.0f;
                    ((float*)&constraint->tangentMass2)[j] = 0.0f;

                    ((float*)&constraint->restitution)[j] = 0.0f;
                    ((float*)&constraint->relativeVelocity1)[j] = 0.0f;
                    ((float*)&constraint->relativeVelocity2)[j] = 0.0f;
                }
            }
        }
    }
    public unsafe void WarmStartContactsTask(int startIndex, int endIndex, StepContext context, int colorIndex)
    {
        var states = context.states.Data;
        var constraints = ((ContactConstraintsFloat)context.graph.colors[colorIndex].simdConstraints).simdConstraints;
        {
            for (int i = startIndex; i < endIndex; i++)
            {
                ContactConstraintSIMD* c = constraints + i;
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
                }
                bA.w = MulSubW(bA.w, c->invIA, c->rollingImpulse);
                bB.w = MulAddW(bB.w, c->invIB, c->rollingImpulse);
                ScatterBodies(states, (int*)&c->indexA, ref bA);
                ScatterBodies(states, (int*)&c->indexB, ref bB);
            }
        }
    }
    public unsafe void SolveContactsTask(int startIndex, int endIndex, StepContext context, int colorIndex, bool useBias)
    {
        var states = context.states.Data;
        var constraints = ((ContactConstraintsFloat)context.graph.colors[colorIndex].simdConstraints).simdConstraints;
        {
            FloatW inv_h = new(context.inv_h);
            FloatW contactSpeed = new(-context.world.contactSpeed);
            FloatW oneW = new(1);
            for (int i = startIndex; i < endIndex; i++)
            {
                ContactConstraintSIMD* c = constraints + i;
                BodyStateW bA = GatherBodies(states, (int*)&c->indexA);
                BodyStateW bB = GatherBodies(states, (int*)&c->indexB);
                FloatW biasRate, massScale, impulseScale;
                if (useBias)
                {
                    biasRate = c->massScale * c->biasRate;
                    massScale = c->massScale;
                    impulseScale = c->impulseScale;
                }
                else
                {
                    biasRate = FloatW.Zero;
                    massScale = oneW;
                    impulseScale = FloatW.Zero;
                }
                FloatW totalNormalImpulse = FloatW.Zero;
                Vector2W dp = new() { X = bB.dp.X - bA.dp.X, Y = bB.dp.Y - bA.dp.Y };
                {
                    Vector2W rA = c->anchorA1, rB = c->anchorB1;
                    Vector2W rsA = RotateVectorW(bA.dq, rA), rsB = RotateVectorW(bB.dq, rB);
                    Vector2W ds = new() { X = dp.X + (rsB.X - rsA.X), Y = dp.Y + (rsB.Y - rsA.Y) };
                    FloatW s = DotW(c->normal, ds) + c->baseSeparation1;
                    FloatW mask = GreaterThanW(s, FloatW.Zero);
                    FloatW specBias = s * inv_h, softBias = MaxW(biasRate * s, contactSpeed);
                    FloatW bias = BlendW(softBias, specBias, mask);
                    FloatW pointMassScale = BlendW(massScale, oneW, mask);
                    FloatW pointImpulseScale = BlendW(impulseScale, FloatW.Zero, mask);
                    FloatW dvx = (bB.v.X - bB.w * rB.Y) - (bA.v.X - bA.w * rA.Y);
                    FloatW dvy = (bB.v.Y + bB.w * rB.X) - (bA.v.Y + bA.w * rA.X);
                    FloatW vn = dvx * c->normal.X + dvy * c->normal.Y;
                    FloatW negImpulse = c->normalMass1 * (pointMassScale * vn + bias) + pointImpulseScale * c->normalImpulse1;
                    FloatW newImpulse = MaxW(c->normalImpulse1 - negImpulse, FloatW.Zero);
                    FloatW impulse = newImpulse - c->normalImpulse1;
                    c->normalImpulse1 = newImpulse;
                    c->totalNormalImpulse1 = c->totalNormalImpulse1 + newImpulse;
                    totalNormalImpulse += newImpulse;
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
                    FloatW mask = GreaterThanW(s, FloatW.Zero);
                    FloatW specBias = s * inv_h, softBias = MaxW(biasRate * s, contactSpeed);
                    FloatW bias = BlendW(softBias, specBias, mask);
                    FloatW pointMassScale = BlendW(massScale, oneW, mask);
                    FloatW pointImpulseScale = BlendW(impulseScale, FloatW.Zero, mask);
                    FloatW dvx = (bB.v.X - bB.w * rB.Y) - (bA.v.X - bA.w * rA.Y);
                    FloatW dvy = (bB.v.Y + bB.w * rB.X) - (bA.v.Y + bA.w * rA.X);
                    FloatW vn = dvx * c->normal.X + dvy * c->normal.Y;
                    //different than 1, is this intended?
                    FloatW negImpulse = c->normalMass2 * (pointMassScale * vn + bias) + pointImpulseScale * c->normalImpulse2;
                    FloatW newImpulse = MaxW(c->normalImpulse2 - negImpulse, FloatW.Zero);
                    FloatW impulse = newImpulse - c->normalImpulse2;
                    c->normalImpulse2 = newImpulse;
                    c->totalNormalImpulse2 = c->totalNormalImpulse2 + newImpulse;
                    totalNormalImpulse += newImpulse;
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
                FloatW tangentX = c->normal.Y;
                FloatW tangentY = FloatW.Zero - c->normal.X;
                {
                    Vector2W rA = c->anchorA1, rB = c->anchorB1;
                    FloatW dvx = (bB.v.X - bB.w * rB.Y) - (bA.v.X - bA.w * rA.Y);
                    FloatW dvy = (bB.v.Y + bB.w * rB.X) - (bA.v.Y + bA.w * rA.X);
                    FloatW vt = dvx * tangentX + dvy * tangentY;
                    vt -= c->tangentSpeed;
                    FloatW negImpulse = c->tangentMass1 * vt;
                    FloatW maxFriction = c->friction * c->normalImpulse1;
                    FloatW newImpulse = c->tangentImpulse1 - negImpulse;
                    //no symclamp?
                    newImpulse = MaxW(FloatW.Zero - maxFriction, MinW(newImpulse, maxFriction));
                    FloatW impulse = newImpulse - c->tangentImpulse1;
                    c->tangentImpulse1 = newImpulse;
                    FloatW Px = impulse * tangentX;
                    FloatW Py = impulse * tangentY;
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
                    FloatW dvx = (bB.v.X - bB.w * rB.Y) - (bA.v.X - bA.w * rA.Y);
                    FloatW dvy = (bB.v.Y + bB.w * rB.X) - (bA.v.Y + bA.w * rA.X);
                    FloatW vt = dvx * tangentX + dvy * tangentY;
                    vt -= c->tangentSpeed;
                    FloatW negImpulse = c->tangentMass2 * vt;
                    FloatW maxFriction = c->friction * c->normalImpulse2;
                    FloatW newImpulse = c->tangentImpulse2 - negImpulse;
                    newImpulse = MaxW(FloatW.Zero - maxFriction, MinW(newImpulse, maxFriction));
                    FloatW impulse = newImpulse - c->tangentImpulse2;
                    c->tangentImpulse2 = newImpulse;
                    FloatW Px = impulse * tangentX;
                    FloatW Py = impulse * tangentY;
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
                    FloatW deltaLambda = c->rollingMass * (bA.w - bB.w);
                    FloatW lambda = c->rollingImpulse;
                    FloatW maxLambda = c->rollingResistance * totalNormalImpulse;
                    c->rollingImpulse = SymClampW(lambda + deltaLambda, maxLambda);
                    deltaLambda = c->rollingImpulse - lambda;
                    bA.w = MulSubW(bA.w, c->invIA, deltaLambda);
                    bB.w = MulAddW(bB.w, c->invIB, deltaLambda);
                }
                ScatterBodies(states, (int*)&c->indexA, ref bA);
                ScatterBodies(states, (int*)&c->indexB, ref bB);
            }
        }
    }
    public unsafe void ApplyRestitutionTask(int startIndex, int endIndex, StepContext context, int colorIndex)
    {
        var states = context.states.Data;
        var constraints = ((ContactConstraintsFloat)context.graph.colors[colorIndex].simdConstraints).simdConstraints;
        {
            FloatW threshold = new(context.world.restitutionThreshold);
            FloatW zero = FloatW.Zero;
            for (int i = startIndex; i < endIndex; i++)
            {
                ContactConstraintSIMD* c = constraints + i;
                if (AllZeroW(c->restitution)) continue;
                FloatW restitutionMask = EqualsW(c->restitution, zero);
                BodyStateW bA = GatherBodies(states, (int*)&c->indexA);
                BodyStateW bB = GatherBodies(states, (int*)&c->indexB);
                {
                    FloatW mask1 = GreaterThanW(c->relativeVelocity1 + threshold, zero);
                    FloatW mask2 = EqualsW(c->totalNormalImpulse1, zero);
                    FloatW mask = OrW(OrW(mask1, mask2), restitutionMask);
                    FloatW mass = BlendW(c->normalMass1, zero, mask);
                    Vector2W rA = c->anchorA1, rB = c->anchorB1;
                    FloatW dvx = (bB.v.X - bB.w * rB.Y) - (bA.v.X - bA.w * rA.Y);
                    FloatW dvy = (bB.v.Y + bB.w * rB.X) - (bA.v.Y + bA.w * rA.X);
                    FloatW vn = dvx * c->normal.X + dvy * c->normal.Y;
                    FloatW negImpulse = mask * (vn + c->restitution * c->relativeVelocity1);
                    FloatW newImpulse = MaxW(c->normalImpulse1 - negImpulse, FloatW.Zero);
                    FloatW deltaImpulse = newImpulse - c->normalImpulse1;
                    c->normalImpulse1 = newImpulse;
                    c->totalNormalImpulse1 += deltaImpulse;
                    FloatW Px = deltaImpulse * c->normal.X;
                    FloatW Py = deltaImpulse * c->normal.Y;
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
                    FloatW mask1 = GreaterThanW(c->relativeVelocity2 + threshold, zero);
                    FloatW mask2 = EqualsW(c->totalNormalImpulse2, zero);
                    FloatW mask = OrW(OrW(mask1, mask2), restitutionMask);
                    FloatW mass = BlendW(c->normalMass2, zero, mask);
                    Vector2W rA = c->anchorA2, rB = c->anchorB2;
                    FloatW dvx = (bB.v.X - bB.w * rB.Y) - (bA.v.X - bA.w * rA.Y);
                    FloatW dvy = (bB.v.Y + bB.w * rB.X) - (bA.v.Y + bA.w * rA.X);
                    FloatW vn = dvx * c->normal.X + dvy * c->normal.Y;
                    FloatW negImpulse = mask * (vn + c->restitution * c->relativeVelocity2);
                    FloatW newImpulse = MaxW(c->normalImpulse2 - negImpulse, FloatW.Zero);
                    FloatW deltaImpulse = newImpulse - c->normalImpulse2;
                    c->normalImpulse2 = newImpulse;
                    c->totalNormalImpulse2 += deltaImpulse;
                    FloatW Px = deltaImpulse * c->normal.X;
                    FloatW Py = deltaImpulse * c->normal.Y;
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
    public unsafe void StoreImpulsesTask(int startIndex, int endIndex, StepContext context)
    {
        ContactSim[] contacts = context.contacts;
        Manifold dummy = new();
        ContactConstraintSIMD* constraints = ((ContactConstraintsFloat)context.simdContactConstraints).simdConstraints;
        {
            for (int constraintIndex = startIndex; constraintIndex < endIndex; constraintIndex++)
            {
                ContactConstraintSIMD* c = constraints + constraintIndex;
                float* rollingImpulse = (float*)&c->rollingImpulse;
                float* normalImpulse1 = (float*)&c->normalImpulse1;
                float* normalImpulse2 = (float*)&c->normalImpulse2;
                float* tangentImpulse1 = (float*)&c->tangentImpulse1;
                float* tangentImpulse2 = (float*)&c->tangentImpulse2;
                float* totalNormalImpulse1 = (float*)&c->totalNormalImpulse1;
                float* totalNormalImpulse2 = (float*)&c->totalNormalImpulse2;
                float* normalVelocity1 = (float*)&c->relativeVelocity1;
                float* normalVelocity2 = (float*)&c->relativeVelocity2;
                int baseIndex = 4 * constraintIndex;
                for (int laneIndex = 0; laneIndex < 4; ++laneIndex)
                {
                    ref Manifold m = ref dummy;
                    if (contacts[baseIndex + laneIndex] != null) m = ref contacts[baseIndex + laneIndex].manifold;
                    m.rollingImpulse = rollingImpulse[laneIndex];
                    m.point0.normalImpulse = normalImpulse1[laneIndex];
                    m.point0.tangentImpulse = tangentImpulse1[laneIndex];
                    m.point0.totalNormalImpulse = totalNormalImpulse1[laneIndex];
                    m.point0.normalVelocity = normalVelocity1[laneIndex];
                    m.point1.normalImpulse = normalImpulse2[laneIndex];
                    m.point1.tangentImpulse = tangentImpulse2[laneIndex];
                    m.point1.totalNormalImpulse = totalNormalImpulse2[laneIndex];
                    m.point1.normalVelocity = normalVelocity2[laneIndex];
                }
            }
        }
    }
}