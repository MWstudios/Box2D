using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;
using Box2D.API;

namespace Box2D.Particle;

[DebuggerDisplay("{IndexA}-{IndexB} ({Weight}) {Normal} {Flags}")] public struct ParticleContact
{
    public int IndexA { get; set; } public int IndexB { get; set; } public float Weight { get; set; }
    public void SetIndices(int a, int b)
    {
        Debug.Assert(a <= Box2D.MaxParticleIndex && b <= Box2D.MaxParticleIndex);
        IndexA = a; IndexB = b;
    }
    public Vector2 Normal { get; set; } public ParticleFlag Flags { get; set; }
    public static bool operator ==(ParticleContact c1, ParticleContact c2) =>
        c1.IndexA == c2.IndexA && c1.IndexB == c2.IndexB && c1.Weight == c2.Weight && c1.Flags == c2.Flags && c1.Normal == c2.Normal;
    public static bool operator !=(ParticleContact c1, ParticleContact c2) =>
        c1.IndexA != c2.IndexA || c1.IndexB != c2.IndexB || c1.Weight != c2.Weight || c1.Flags != c2.Flags || c1.Normal != c2.Normal;
    public bool ApproximatelyEqual(ParticleContact rhs) => IndexA == rhs.IndexA && IndexB == rhs.IndexB &&
        Flags == rhs.Flags && Math.Abs(Weight - rhs.Weight) < 0.01 && (Normal - rhs.Normal).Length() < 0.01;
    public override bool Equals(object obj) => obj is ParticleContact pc && this == pc;
    public override int GetHashCode() => HashCode.Combine(IndexA, IndexB, Weight, Normal);
}
public struct ParticleBodyContact
{
    public int index; public Body body; public Shape fixture; public float weight; public Vector2 normal; public float mass;
}
[DebuggerDisplay("{indexA}-{indexB} S{strength} D{distance} {flags}")] public struct ParticlePair
{
    public int indexA, indexB; public ParticleFlag flags; public float strength; public float distance;
}
[DebuggerDisplay("({indexA} {pa} {ka})-({indexB} {pb} {kb})-({indexC} {pc} {kc}) s{s} S{strength} {flags}")] public struct ParticleTriad
{
    public int indexA, indexB, indexC; public ParticleFlag flags; public float strength; public Vector2 pa, pb, pc; public float ka, kb, kc, s;
}
unsafe class ParticleBodyContactRemovePredicate
{
    int k_maxContactsPerPoint = 3; ParticleSystem m_system; int m_lastIndex = -1, m_currentContacts = 0; int* m_discarded;
    public ParticleBodyContactRemovePredicate(ParticleSystem system, int* discarded) { m_system = system; m_discarded = discarded; }
    public bool this[ParticleBodyContact contact]
    {
        get
        {
            if (contact.index != m_lastIndex) { m_currentContacts = 0; m_lastIndex = contact.index; }
            if (m_currentContacts++ > k_maxContactsPerPoint) { ++*m_discarded; return true; }
            Vector2 n = contact.normal * m_system.GetParticleDiameter() * (1 - contact.weight);
            Vector2 pos = m_system.PositionBuffer[contact.index] + n;
            Transform t = m_system.World.GetBodyTransform(contact.fixture.bodyId);
            if (!contact.fixture.shape.TestPoint(t.InvTransformPoint(pos)))
            {
                DistanceInput input = new()
                {
                    proxyA = contact.fixture.MakeDistanceProxy(),
                    proxyB = new Circle { center = pos, radius = 0 }.MakeProxy(),
                    transformA = t,
                    transformB = Transform.Identity,
                };
                SimplexCache cache = new();
                var distance = input.ShapeDistance(ref cache, null);
                if (distance.distance < Box2D.ParticleLinearSlop) return false;
                ++*m_discarded;
                return true;
            }
            return false;
        }
    }
}
class ExpirationTimeComparator
{
    int[] m_expirationTimes;
    public ExpirationTimeComparator(int[] expirationTimes) => m_expirationTimes = expirationTimes;
    public int Compare(int particleIndexA, int particleIndexB)
    {
        int expirationTimeA = m_expirationTimes[particleIndexA], expirationTimeB = m_expirationTimes[particleIndexB];
        int infiniteExpirationTimeA = expirationTimeA.CompareTo(0), infiniteExpirationTimeB = expirationTimeB.CompareTo(0);
        return infiniteExpirationTimeA == infiniteExpirationTimeB ? expirationTimeA.CompareTo(expirationTimeB) : infiniteExpirationTimeA;
    }
}
public class ParticleSystem
{
    static int xTruncBits = 12;
    static int yTruncBits = 12;
    static int tagBits = 8 * sizeof(uint);
    static uint yOffset = 1u << (yTruncBits - 1);
    static int yShift = tagBits - yTruncBits;
    static int xShift = tagBits - yTruncBits - xTruncBits;
    static uint xScale = 1u << xShift;
    static uint xOffset = xScale * (1u << (xTruncBits - 1));
    static uint yMask = ((1u << yTruncBits) - 1u) << yShift;
    static uint xMask = ~yMask;
    static uint relativeTagRight = 1u << xShift;
    static uint relativeTagBottomLeft = (uint)((1 << yShift) + (uint.MaxValue << xShift));
    static uint relativeTagBottomRight = (1u << yShift) + (1u << xShift);
    public int CreateParticle(ParticleDef def)
    {
        Debug.Assert(!World.locked);
        if (World.locked) return 0;
        if (Count >= InternalAllocatedCapacity)
        {
            int capacity = Count > 0 ? 2 * Count : Box2D.MinParticleSystemBufferCapacity;
            ReallocateInternalAllocatedBuffers(capacity);
        }
        if (Count >= InternalAllocatedCapacity)
        {
            if (DestroyByAge)
            {
                DestroyOldestParticle(0, false);
                SolveZombie();
            }
            else return Box2D.InvalidParticleIndex;
        }
        int index = Count++;
        PositionBuffer[index] = def.position;
        VelocityBuffer[index] = def.velocity;
        ForceBuffer[index] = Vector2.Zero;
        ImpulseBuffer[index] = Vector2.Zero;
        if (StaticPressureBuffer != null) StaticPressureBuffer[index] = 0;
        if (ColorBuffer != null || !def.color.IsZero()) { RequestBuffer(ref ColorBuffer)[index] = def.color; }
        if (UserDataBuffer != null || def.userData != null) { RequestBuffer(ref UserDataBuffer)[index] = def.userData; }
        Proxy proxy = new();
        bool finiteLifetime = def.lifetime > 0;
        if (ExpirationTimeBuffer != null || finiteLifetime)
        {
            SetParticleLifetime(index, finiteLifetime ? def.lifetime : ExpirationTimeToLifetime(-GetQuantizedtimeElapsed()));
            IndexByExpirationTimeBuffer[index] = index;
        }
        proxy.index = index; ProxyBuffer.Add(proxy);
        ParticleGroup group = def.group;
        GroupBuffer[index] = group;
        if (group != null)
        {
            if (group.BufferIndex < group.LastIndex)
            {
                RotateBuffer(group.BufferIndex, group.LastIndex, index);
                Debug.Assert(group.LastIndex == index);
                group.LastIndex = index + 1;
            }
            else
            {
                group.BufferIndex = index;
                group.LastIndex = index + 1;
            }
        }
        SetParticleFlags(index, def.flags);
        return index;
    }
    public ParticleHandle GetParticleHandleFromIndex(int index)
    {
        Debug.Assert(index >= 0 && index < Count && index != Box2D.InvalidParticleIndex);
        ParticleHandle handle = RequestBuffer(ref HandleIndexBuffer)[index];
        if (handle != null) return handle;
        handle = new() { Index = index }; HandleIndexBuffer[index] = handle;
        return handle;
    }
    public void DestroyParticle(int index) => DestroyParticle(index, false);
    public void DestroyParticle(int index, bool callDestructionListener)
    {
        ParticleFlag flags = ParticleFlag.Zombie;
        if (callDestructionListener) flags |= ParticleFlag.DestructionListener;
        SetParticleFlags(index, FlagsBuffer[index] | flags);
    }
    public void DestroyOldestParticle(int index, bool callDestructionListener)
    {
        int particleCount = Count;
        Debug.Assert(index >= 0 && index < particleCount);
        Debug.Assert(IndexByExpirationTimeBuffer != null);
        int oldestFiniteLifetimeParticle = IndexByExpirationTimeBuffer[particleCount - index - 1], oldestInfiniteLifetimeParticle = IndexByExpirationTimeBuffer[index];
        DestroyParticle(ExpirationTimeBuffer[oldestFiniteLifetimeParticle] > 0 ? oldestFiniteLifetimeParticle : oldestInfiniteLifetimeParticle, callDestructionListener);
    }
    public int DestroyParticlesInShape(Shape shape, Transform xf) => DestroyParticlesInShape(shape, xf, false);
    public int DestroyParticlesInShape(Shape shape, Transform xf, bool callDestructionListener)
    {
        Debug.Assert(!World.locked);
        if (World.locked) return 0;
        int destroyed = 0;
        ParticleQueryCallback callback = new()
        {
            ReportParticle = (particleSystem, index) =>
            {
                if (particleSystem != this) return false;
                Debug.Assert(index >= 0 && index < Count);
                if (shape.shape.TestPoint(xf.InvTransformPoint(PositionBuffer[index])))
                {
                    DestroyParticle(index, callDestructionListener); destroyed++;
                }
                return true;
            }
        };
        AABB aabb = shape.ComputeAABB(xf);
        WorldAPI.World_OverlapAABB(new WorldID { generation = World.generation, index1 = World }, aabb, new QueryFilter(), null, null, callback);
        return destroyed;
    }
    public ParticleGroup CreateParticleGroup(ParticleGroupDef groupDef)
    {
        Debug.Assert(!World.locked); if (World.locked) return null;
        Transform transform = new(groupDef.Position, new(groupDef.Angle));
        int firstIndex = Count;
        if (groupDef.Shape != null) CreateParticlesWithShapeForGroup(groupDef.Shape, groupDef, transform, groupDef.TriangleGrid);
        if (groupDef.Shapes != null) CreateParticlesWithShapesForGroup(groupDef.Shapes, groupDef, transform, groupDef.TriangleGrid);
        if (groupDef.ParticleCount > 0)
        {
            Debug.Assert(groupDef.PositionData != null);
            for (int i = 0; i < groupDef.ParticleCount; i++) CreateParticleForGroup(groupDef, transform, groupDef.PositionData[i]);
        }
        int lastIndex = Count;
        ParticleGroup group = new()
        { System = this, BufferIndex = firstIndex, LastIndex = lastIndex, m_strength = groupDef.Strength, UserData = groupDef.UserData, Transform = transform };
        ParticleGroupList.Add(group);
        for (int i = firstIndex; i < lastIndex; i++) GroupBuffer[i] = group;
        SetGroupFlags(group, groupDef.GroupFlags);
        ConnectionFilter filter = new();
        UpdateContacts(true); UpdatePairsAndTriads(firstIndex, lastIndex, filter);
        if (groupDef.Group != null) { JoinParticleGroups(groupDef.Group, group); group = groupDef.Group; }
        return group;
    }
    class JoinParticleGroupsFilter : ConnectionFilter
    {
        int m_threshold;
        public JoinParticleGroupsFilter(int threshold) { m_threshold = threshold; }
        public override bool ShouldCreatePair(int a, int b) => (a < m_threshold && m_threshold <= b) || (b < m_threshold && m_threshold <= a);
        public override bool ShouldCreateTriad(int a, int b, int c) =>
            (a < m_threshold || b < m_threshold || c < m_threshold) &&
            (m_threshold <= a || m_threshold <= b || m_threshold <= c);
    }
    public void JoinParticleGroups(ParticleGroup groupA, ParticleGroup groupB)
    {
        Debug.Assert(!World.locked); if (World.locked) return;
        Debug.Assert(groupA != groupB); RotateBuffer(groupB.BufferIndex, groupB.LastIndex, Count);
        Debug.Assert(groupB.LastIndex == Count); RotateBuffer(groupA.BufferIndex, groupA.LastIndex, groupB.BufferIndex);
        Debug.Assert(groupA.LastIndex == groupB.BufferIndex);
        JoinParticleGroupsFilter filter = new(groupB.BufferIndex);
        UpdateContacts(true); UpdatePairsAndTriads(groupA.BufferIndex, groupB.LastIndex, filter);
        for (int i = groupB.BufferIndex; i < groupB.LastIndex; i++) GroupBuffer[i] = groupA;
        ParticleGroupFlag groupFlags = groupA.m_groupFlags | groupB.m_groupFlags;
        SetGroupFlags(groupA, groupFlags);
        groupA.LastIndex = groupB.BufferIndex = groupB.LastIndex;
        DestroyParticleGroup(groupB);
    }
    public unsafe void SplitParticleGroup(ParticleGroup group)
    {
        UpdateContacts(true);
        int particleCount = group.ParticleCount;
        ParticleListNode[] nodeBuffer = new ParticleListNode[particleCount];
        InitializeParticleLists(group, nodeBuffer); MergeParticleListsInContact(group, nodeBuffer);
        ParticleListNode* survivingList = FindLongestParticleList(group, nodeBuffer);
        MergeZombieParticleListNodes(group, nodeBuffer, survivingList);
        CreateParticleGroupsFromParticleList(group, nodeBuffer, survivingList);
        UpdatePairsAndTriadsWithParticleList(group, nodeBuffer);
    }
    /// <summary></summary>
    public List<ParticleGroup> ParticleGroupList { get; } = new();
    /// <summary>Pause or unpause the particle system. When paused, <see cref="World.Step(float)"/>
    /// skips over this particle system. All b2ParticleSystem function calls still work.</summary>
    public bool Paused { get; set; }
    /// <summary>Change the particle density. Particle density affects the mass of the particles, which in turn
    /// affects how the particles interact with b2Bodies. Note that the density does not affect how the particles interact with each other. </summary>
    public float GetDensity() => density;
    /// <summary>Change the particle density. Particle density affects the mass of the particles, which in turn
    /// affects how the particles interact with b2Bodies. Note that the density does not affect how the particles interact with each other. </summary>
    public void SetDensity(float value)
    { density = value; SetInverseDensity(1 / value); }
    ///<summary>Change the particle gravity scale. Adjusts the effect of the global gravity vector on particles. Default value is 1.0f.</summary>
    public float GravityScale { get; set; } = 1;
    /// <summary>Particles behave as circles with this radius. In Box2D units.</summary>
    public float GetRadius() => GetParticleDiameter() * 0.5f;
    /// <summary>Particles behave as circles with this radius. In Box2D units.</summary>
    public void SetRadius(float value)
    { SetParticleDiameter(2 * value); SetSquaredDiameter(GetParticleDiameter() * GetParticleDiameter()); SetInverseDiameter(1 / GetParticleDiameter()); }
    public void ReallocateHandleBuffers(int newCapacity)
    {
        Debug.Assert(newCapacity > InternalAllocatedCapacity);
        HandleIndexBuffer = new ParticleHandle[newCapacity];
        ReallocateBuffer(ref HandleIndexBuffer, InternalAllocatedCapacity, newCapacity, true);
    }
    public T[] RequestBuffer<T>(ref T[] buffer)
    {
        if (buffer == null)
        {
            if (InternalAllocatedCapacity == 0) ReallocateInternalAllocatedBuffers(Box2D.MinParticleSystemBufferCapacity);
            buffer = new T[InternalAllocatedCapacity];
        }
        return buffer;
    }
    public ParticleGroup[] GetGroupBuffer() => GroupBuffer;
    public float[] GetWeightBuffer() => WeightBuffer;
    public ParticleFlag[] GetFlagsBuffer() => FlagsBuffer;
    public void SetParticleFlags(int index, ParticleFlag flags)
    {
        ref ParticleFlag oldFlags = ref FlagsBuffer[index];
        if ((oldFlags & ~flags) != 0) NeedsUpdateAllParticleFlags = true;
        if ((~AllParticleFlags & flags) != 0)
        {
            if (flags.HasFlag(ParticleFlag.Tensile)) Accumulation2Buffer = RequestBuffer(ref Accumulation2Buffer);
            if (flags.HasFlag(ParticleFlag.ColorMixing)) ColorBuffer = RequestBuffer(ref ColorBuffer);
            AllParticleFlags |= flags;
        }
        oldFlags = flags;
    }
    public ParticleFlag GetParticleFlags(int index) => FlagsBuffer[index];
    public void SetFlagsBuffer(ParticleFlag[] buffer) => FlagsBuffer = buffer;
    public List<ParticleContact> GetContacts() => ContactBuffer;
    public List<ParticleBodyContact> GetBodyContacts() => BodyContactBuffer;
    public List<ParticlePair> GetPairs() => PairBuffer;
    public List<ParticleTriad> GetTriads() => TriadBuffer;
    public void SetStuckThreshold(int iterations)
    {
        StuckThreshold = iterations;
        if (iterations > 0)
        {
            LastBodyContactStepBuffer = RequestBuffer(ref LastBodyContactStepBuffer);
            BodyContactCountBuffer = RequestBuffer(ref BodyContactCountBuffer);
            ConsecutiveContactStepsBuffer = RequestBuffer(ref ConsecutiveContactStepsBuffer);
        }
    }
    public List<int> GetStuckCandidates() => StuckParticleBuffer;
    void ReallocateBuffer<T>(ref T[] oldBuffer, int oldCapacity, int newCapacity, bool deferred)
    {
        Debug.Assert(newCapacity > oldCapacity);
        //Debug.Assert((oldBuffer?.Length ?? 0) == 0 || newCapacity <= oldBuffer.Length);
        if (!deferred || oldBuffer != null /*&& (oldBuffer?.Length ?? 0) == 0*/)
        {
            T[] newBuffer = new T[newCapacity];
            if (oldBuffer != null) HPCsharp.ParallelAlgorithms.Copy.CopyPar(oldBuffer, 0, newBuffer, 0, oldBuffer.Length);
            oldBuffer = newBuffer;
        }
    }
    static int LimitCapacity(int capacity, int maxCount) => maxCount > 0 && capacity > maxCount ? maxCount : capacity;
    void ReallocateInternalAllocatedBuffers(int capacity)
    {
        capacity = LimitCapacity(capacity, MaxCount);
        /*capacity = LimitCapacity(capacity, FlagsBuffer?.Length ?? 0);
        capacity = LimitCapacity(capacity, PositionBuffer?.Length ?? 0);
        capacity = LimitCapacity(capacity, VelocityBuffer?.Length ?? 0);
        capacity = LimitCapacity(capacity, ColorBuffer?.Length ?? 0);
        capacity = LimitCapacity(capacity, UserDataBuffer?.Length ?? 0);*/
        if (InternalAllocatedCapacity < capacity)
        {
            ReallocateHandleBuffers(capacity);
            ReallocateBuffer(ref FlagsBuffer, InternalAllocatedCapacity, capacity, false);
            bool stuck = StuckThreshold > 0;
            ReallocateBuffer(ref LastBodyContactStepBuffer, InternalAllocatedCapacity, capacity, stuck);
            ReallocateBuffer(ref BodyContactCountBuffer, InternalAllocatedCapacity, capacity, stuck);
            ReallocateBuffer(ref ConsecutiveContactStepsBuffer, InternalAllocatedCapacity, capacity, stuck);
            ReallocateBuffer(ref PositionBuffer, InternalAllocatedCapacity, capacity, false);
            ReallocateBuffer(ref VelocityBuffer, InternalAllocatedCapacity, capacity, false);
            ReallocateBuffer(ref ForceBuffer, InternalAllocatedCapacity, capacity, false);
            ReallocateBuffer(ref ImpulseBuffer, InternalAllocatedCapacity, capacity, false);
            ReallocateBuffer(ref WeightBuffer, InternalAllocatedCapacity, capacity, false);
            ReallocateBuffer(ref StaticPressureBuffer, InternalAllocatedCapacity, capacity, true);
            ReallocateBuffer(ref AccumulationBuffer, InternalAllocatedCapacity, capacity, false);
            ReallocateBuffer(ref Accumulation2Buffer, InternalAllocatedCapacity, capacity, true);
            ReallocateBuffer(ref DepthBuffer, InternalAllocatedCapacity, capacity, true);
            ReallocateBuffer(ref ColorBuffer, InternalAllocatedCapacity, capacity, true);
            ReallocateBuffer(ref GroupBuffer, InternalAllocatedCapacity, capacity, false);
            ReallocateBuffer(ref UserDataBuffer, InternalAllocatedCapacity, capacity, true);
            ReallocateBuffer(ref ExpirationTimeBuffer, InternalAllocatedCapacity, capacity, true);
            ReallocateBuffer(ref IndexByExpirationTimeBuffer, InternalAllocatedCapacity, capacity, true);
            InternalAllocatedCapacity = capacity;
        }
    }
    public float ComputeCollisionEnergy()
    {
        float sum_v2 = 0;
        for (int k = 0; k < ContactBuffer.Count; k++)
        {
            ParticleContact contact = ContactBuffer[k];
            Vector2 v = VelocityBuffer[contact.IndexB] - VelocityBuffer[contact.IndexA];
            float vn = Vector2.Dot(v, contact.Normal); if (vn < 0) sum_v2 += vn * vn;
        }
        return 0.5f * GetParticleMass() * sum_v2;
    }
    public bool StrictContactCheck { get; set; } = false;
    public void SetParticleLifetime(int index, float lifetime)
    {
        Debug.Assert(ValidateParticleIndex(index));
        bool initializeExpirationTimes = IndexByExpirationTimeBuffer == null;
        ExpirationTimeBuffer = RequestBuffer(ref ExpirationTimeBuffer);
        IndexByExpirationTimeBuffer = RequestBuffer(ref IndexByExpirationTimeBuffer);
        if (initializeExpirationTimes)
        {
            int particleCount = Count;
            for (int i = 0; i < particleCount; i++) IndexByExpirationTimeBuffer[i] = i;
        }
        int quantizedLifetime = (int)(lifetime / LifetimeGranularity);
        int newExpirationTime = quantizedLifetime > 0 ? GetQuantizedtimeElapsed() + quantizedLifetime : quantizedLifetime;
        if (newExpirationTime != ExpirationTimeBuffer[index])
        {
            ExpirationTimeBuffer[index] = newExpirationTime;
            ExpirationTimeBufferRequiresSorting = true;
        }
    }
    public float GetParticleLifetime(int index)
    {
        Debug.Assert(ValidateParticleIndex(index));
        return ExpirationTimeToLifetime(GetExpirationTimeBuffer()[index]);
    }
    public bool DestructionByAge { get => DestroyByAge; set { if (value) GetExpirationTimeBuffer(); DestroyByAge = value; } }
    public int[] GetExpirationTimeBuffer() => RequestBuffer(ref ExpirationTimeBuffer);
    float ExpirationTimeToLifetime(int expirationTime) => (expirationTime > 0 ? expirationTime - GetQuantizedtimeElapsed() : expirationTime) * LifetimeGranularity;
    int[] GetIndexByExpirationTimeBuffer()
    {
        if (Count != 0) SetParticleLifetime(0, GetParticleLifetime(0));
        else IndexByExpirationTimeBuffer = RequestBuffer(ref IndexByExpirationTimeBuffer);
        return IndexByExpirationTimeBuffer;
    }
    public void ParticleApplyBufferLinearImpulse(int index, Vector2 impulse)
    { if (ForceCanBeApplied(FlagsBuffer[index])) { PrepareImpulseBuffer(); ImpulseBuffer[index] += impulse; } }
    public void ParticleApplyLinearImpulse(int index, Vector2 impulse) => ApplyLinearImpulse(index, index + 1, impulse);
    public unsafe void ApplyLinearImpulse(int firstIndex, int lastIndex, Vector2 impulse)
    {
        Vector2 velocityDelta = 1 / ((lastIndex - firstIndex) * GetParticleMass()) * impulse;
        int i = firstIndex;
        if (Avx.IsSupported)
        {
            Vector256<float> vD = Vector256.Create(velocityDelta.x, velocityDelta.y, velocityDelta.x, velocityDelta.y, velocityDelta.x, velocityDelta.y, velocityDelta.x, velocityDelta.y);
            fixed (Vector2* v = VelocityBuffer) for (; i + 4 <= lastIndex; i += 4)
                    Avx.Store((float*)v + i, Avx.Add(Avx.LoadVector256((float*)v + i), vD));
        }
        for (; i < lastIndex; i++) VelocityBuffer[i] += velocityDelta;
    }
    public void ParticleApplyForce(int index, Vector2 force)
    { if (force != Vector2.Zero && ForceCanBeApplied(FlagsBuffer[index])) { PrepareForceBuffer(); ForceBuffer[index] += force; } }
    public unsafe void ApplyForce(int firstIndex, int lastIndex, Vector2 force)
    {
        ParticleFlag flags = 0;
        for (int i = firstIndex; i < lastIndex; i++) flags |= FlagsBuffer[i];
        Debug.Assert(ForceCanBeApplied(flags));
        Vector2 distributedForce = force / (lastIndex - firstIndex);
        if (distributedForce != Vector2.Zero)
        {
            PrepareForceBuffer();
            int i = firstIndex;
            if (Avx.IsSupported)
            {
                Vector256<float> vD = Vector256.Create(distributedForce.x, distributedForce.y, distributedForce.x, distributedForce.y, distributedForce.x, distributedForce.y, distributedForce.x, distributedForce.y);
                fixed (Vector2* v = ForceBuffer) for (; i + 4 <= lastIndex; i += 4)
                        Avx.Store((float*)v + i, Avx.Add(Avx.LoadVector256((float*)v + i), vD));
            }
            for (; i < lastIndex; i++) ForceBuffer[i] += distributedForce;
        }
    }
    public void QueryAABB(ParticleQueryCallback callback, AABB aabb)
    {
        if (ProxyBuffer.Count == 0) return;
        uint tag = ComputeTag(aabb.lowerBound * GetInverseDiameter());
        uint tag2 = ComputeTag(aabb.upperBound * GetInverseDiameter());
        int firstProxy = ProxyBuffer.FindIndex(x => x.tag >= tag),
            lastProxy = ProxyBuffer.FindIndex(x => x.tag > tag2);
        if (lastProxy == -1) lastProxy = ProxyBuffer.Count;
        for (int i = firstProxy; i < lastProxy; i++)
        {
            Vector2 p = PositionBuffer[ProxyBuffer[i].index];
            if (aabb.lowerBound.x < p.x && p.x < aabb.upperBound.x && aabb.lowerBound.y < p.y && p.y < aabb.upperBound.y)
                if (!callback.ReportParticle(this, i)) break;
        }
    }
    public void QueryShapeAABB(ParticleQueryCallback callback, Shape shape, Transform transform) { AABB aabb = shape.ComputeAABB(transform); QueryAABB(callback, aabb); }
    public void RayCast(ParticleRayCastCallback callback, Vector2 point1, Vector2 translation)
    {
        if (ProxyBuffer.Count == 0) return;
        AABB aabb = new(Vector2.Min(point1, point1 + translation), Vector2.Max(point1, point1 + translation));
        float fraction = 1;
        float v2 = Vector2.Dot(translation, translation);
        InsideBoundsEnumerator enumerator = GetInsideBoundsEnumenator(aabb); int i;
        while ((i = enumerator.GetNext()) >= 0)
        {
            Vector2 p = point1 - PositionBuffer[i];
            float pv = Vector2.Dot(p, translation), p2 = Vector2.Dot(p, p),
                determinant = pv * pv - v2 * (p2 - GetSquaredDiameter());
            if (determinant >= 0)
            {
                float sqrtDeterminant = MathF.Sqrt(determinant), t = (-pv - sqrtDeterminant) / v2;
                if (t > fraction) continue;
                if (t < 0) { t = (-pv + sqrtDeterminant) / v2; if (t < 0 || t > fraction) continue; }
                Vector2 n = p + t * translation; n.Normalize();
                float f = callback.ReportParticle(this, i, point1 + t * translation, n, t);
                fraction = Math.Min(fraction, f); if (fraction <= 0) break;
            }
        }
    }
    public unsafe AABB ComputeAABB()
    {
        int particleCount = Count;
        AABB aabb = new(new(float.MaxValue, float.MaxValue), new(float.MinValue, float.MinValue));
        int i = 0;
        if (Avx.IsSupported)
        {
            Vector256<float> lowerBound = Vector256.Create(float.MaxValue), upperBound = Vector256.Create(float.MinValue);
            fixed (Vector2* pB = PositionBuffer) for (; i + 4 <= Count; i += 4)
                {
                    Vector256<float> p = Avx.LoadVector256((float*)(pB + i));
                    lowerBound = Avx.Min(lowerBound, p);
                    upperBound = Avx.Max(upperBound, p);
                }
            aabb.lowerBound = Vector2.Min(Vector2.Min(new(lowerBound.GetElement(0), lowerBound.GetElement(1)), new(lowerBound.GetElement(2), lowerBound.GetElement(3))),
                Vector2.Min(new(lowerBound.GetElement(4), lowerBound.GetElement(5)), new(lowerBound.GetElement(6), lowerBound.GetElement(7))));
            aabb.upperBound = Vector2.Max(Vector2.Max(new(upperBound.GetElement(0), upperBound.GetElement(1)), new(upperBound.GetElement(2), upperBound.GetElement(3))),
                Vector2.Max(new(upperBound.GetElement(4), upperBound.GetElement(5)), new(upperBound.GetElement(6), upperBound.GetElement(7))));
        }
        for (; i < particleCount; i++)
        {
            Vector2 p = PositionBuffer[i];
            aabb.lowerBound = Vector2.Min(aabb.lowerBound, p);
            aabb.upperBound = Vector2.Max(aabb.upperBound, p);
        }
        aabb.lowerBound.x -= GetRadius();
        aabb.lowerBound.y -= GetRadius();
        aabb.upperBound.x += GetRadius();
        aabb.upperBound.y += GetRadius();
        return aabb;
    }
    public enum ExceptionType { BufferTooSmall, ParticleIndexOutOfBounds, NumErrors, NoExceptions }
    public void SetParticleVelocity(int index, Vector2 v) => VelocityBuffer[index] = v;
    public Vector2 GetParticlePosition(int index) => VelocityBuffer[index];
    public unsafe ExceptionType CopyPositionBuffer(int startIndex, int numParticles, Vector2[] outBuf)
    {
        ExceptionType exception = IsBufCopyValid(startIndex, numParticles, sizeof(Vector2) * numParticles, outBuf.Length);
        if (exception != ExceptionType.NoExceptions) return exception;
        HPCsharp.ParallelAlgorithms.Copy.CopyPar(PositionBuffer, startIndex, outBuf, 0, numParticles);
        return ExceptionType.NoExceptions;
    }
    public unsafe ExceptionType CopyColorBuffer(int startIndex, int numParticles, ParticleColor[] outBuf)
    {
        ExceptionType exception = IsBufCopyValid(startIndex, numParticles, sizeof(ParticleColor) * numParticles, outBuf.Length);
        if (exception != ExceptionType.NoExceptions) return exception;
        HPCsharp.ParallelAlgorithms.Copy.CopyPar(ColorBuffer, startIndex, outBuf, 0, numParticles);
        return ExceptionType.NoExceptions;
    }
    public ExceptionType CopyWeightBuffer(int startIndex, int numParticles, float[] outBuf)
    {
        ExceptionType exception = IsBufCopyValid(startIndex, numParticles, sizeof(float) * numParticles, outBuf.Length);
        if (exception != ExceptionType.NoExceptions) return exception;
        HPCsharp.ParallelAlgorithms.Copy.CopyPar(WeightBuffer, startIndex, outBuf, 0, numParticles);
        return ExceptionType.NoExceptions;
    }
    public ExceptionType CopyBuffer<T>(int startIndex, int numParticles, T[] inBufWithOffset, T[] outBuf, int copySize)
    {
        ExceptionType exception = IsBufCopyValid(startIndex, numParticles, copySize, outBuf.Length);
        if (exception != ExceptionType.NoExceptions) return exception;
        HPCsharp.ParallelAlgorithms.Copy.CopyPar(inBufWithOffset, 0, outBuf, 0, copySize);
        return ExceptionType.NoExceptions;
    }
    ExceptionType IsBufCopyValid(int startIndex, int numParticles, int copySize, int bufSize)
    {
        int maxNumParticles = Count; if (copySize == 0) return ExceptionType.NoExceptions;
        if (startIndex < 0 || startIndex >= maxNumParticles || numParticles < 0 || numParticles + startIndex > maxNumParticles) return ExceptionType.ParticleIndexOutOfBounds;
        if (copySize > bufSize) return ExceptionType.BufferTooSmall; return ExceptionType.NoExceptions;
    }
    [DebuggerDisplay("{index}, {tag}")] public struct Proxy
    {
        public int index; public uint tag;
        public static bool operator <(Proxy a, Proxy b) => a.tag < b.tag;
        public static bool operator <(uint a, Proxy b) => a < b.tag;
        public static bool operator <(Proxy a, uint b) => a.tag < b;
        public static bool operator >(Proxy a, Proxy b) => a.tag > b.tag;
        public static bool operator >(uint a, Proxy b) => a > b.tag;
        public static bool operator >(Proxy a, uint b) => a.tag > b;
    }
    public class ConnectionFilter
    {
        public virtual bool IsNecessary(int index) => true;
        public virtual bool ShouldCreatePair(int a, int b) => true;
        public virtual bool ShouldCreateTriad(int a, int b, int c) => true;
    }
    public unsafe class InsideBoundsEnumerator
    {
        public InsideBoundsEnumerator(uint lower, uint upper, List<Proxy> proxies, int first, int last)
        {
            m_proxies = proxies;
            m_xLower = lower & xMask; m_xUpper = upper & xMask;
            m_yLower = lower & yMask; m_yUpper = upper & yMask;
            m_first = first; m_last = last; Debug.Assert(m_first <= m_last);
        }
        public int GetNext()
        {
            while (m_first < m_last)
            {
                uint xTag = m_proxies[m_first].tag & xMask, yTag = m_proxies[m_first].tag & yMask;
                Debug.Assert(yTag >= m_yLower); Debug.Assert(yTag <= m_yUpper);
                if (xTag >= m_xLower && xTag <= m_xUpper) return m_proxies[m_first++].index;
                m_first++;
            }
            return Box2D.InvalidParticleIndex;
        }
        uint m_xLower, m_xUpper, m_yLower, m_yUpper;
        int m_first, m_last; List<Proxy> m_proxies;
    }
    unsafe struct ParticleListNode
    {
        public ParticleListNode* list, next; public int count, index;
    }
    static ParticleFlag pairFlags = ParticleFlag.Spring | ParticleFlag.Barrier, extraDampingFlags = ParticleFlag.StaticPressure,
        triadFlags = ParticleFlag.Elastic, noPressureFlags = ParticleFlag.Powder | ParticleFlag.Tensile;
    float density;
    float particleDiameter;
    float inverseDiameter;
    float inverseDensity;
    float squaredDiameter;
    /// <summary>Set the maximum number of particles. A value of 0 means there is no maximum.
    /// Note: If you try to CreateParticle() with more than this count, <see cref="Box2D.InvalidParticleIndex"/> is returned unless
    /// <see cref="DestructionByAge"/> is used to enable the destruction of the oldest particles in the system.</summary>
    public int MaxCount { get; set; } = 0;
    /// <summary>Increases pressure in response to compression. Smaller values allow more compression.</summary>
    public float PressureStrength { get; set; } = 0.05f;
    /// <summary>Reduces velocity along the collision normal. Smaller value reduces less.</summary>
    public float DampingStrength { get; set; } = 1;
    /// <summary>Restores shape of elastic particle groups. Larger values increase elastic particle velocity.</summary>
    public float ElasticStrength { get; set; } = 0.25f;
    /// <summary>Restores length of spring particle groups. Larger values increase spring particle velocity.</summary>
    public float SpringStrength { get; set; } = 0.25f;
    /// <summary>Reduces relative velocity of viscous particles. Larger values slow down viscous particles more.</summary>
    public float ViscousStrength { get; set; } = 0.25f;
    /// <summary>Produces pressure on tensile particles. 0~0.2. Larger values increase the amount of surface tension.</summary>
    public float SurfaceTensionPressureStrength { get; set; } = 0.2f;
    /// <summary>Produces pressure on tensile particles. 0~0.2. Larger values increase the amount of surface tension.</summary>
    public float SurfaceTensionNormalStrength { get; set; } = 0.2f;
    /// <summary>Produces additional pressure on repulsive particles. Larger values repulse more.
    /// Negative values mean attraction. The range where particles behave stably is about -0.2 to 2.0.</summary>
    public float RepulsiveStrength { get; set; } = 1;
    /// <summary>Produces repulsion between powder particles. Larger values repulse more.</summary>
    public float PowderStrength { get; set; } = 0.5f;
    /// <summary>Pushes particles out of solid particle group. Larger values repulse more.</summary>
    public float EjectionStrength { get; set; } = 0.5f;
    /// <summary>Produces static pressure. Larger values increase the pressure on neighboring partilces. By default, 8 iterations.
    /// You can reduce the number of iterations down to 1 in some situations, but this may cause instabilities when many particles come together.
    /// If you see particles popping away from each other like popcorn, you may have to increase the number of iterations.
    /// For a description of static pressure, see http://en.wikipedia.org/wiki/Static_pressure#Static_pressure_in_fluid_dynamics.</summary>
    public float StaticPressureStrength { get; set; } = 0.2f;
    /// <summary>Reduces instability in static pressure calculation. Larger values make stabilize static pressure with fewer iterations.</summary>
    public float StaticPressureRelaxation { get; set; } = 0.2f;
    /// <summary>Computes static pressure more precisely. See SetStaticPressureIterations for details.</summary>
    public int StaticPressureIterations { get; set; } = 8;
    /// <summary>Determines how fast colors are mixed. 1.0f ==> mixed immediately,
    /// 0.5f ==> mixed half way each simulation step (see <see cref="World.Step(float)"/>)</summary>
    public float ColorMixingStrength { get; set; } = 0.5f;
    /// <summary>Whether to destroy particles by age when no more particles can be created.</summary>
    bool DestroyByAge { get; set; } = true;
    /// <summary>Granularity of particle lifetimes in seconds.  By default this is set to (1.0f / 60.0f) seconds. ParticleSystem uses a 32-bit signed
    /// value to track particle lifetimes so the maximum lifetime of a particle is (2^32 - 1) / (1.0f / lifetimeGranularity) seconds.
    /// With the value set to 1/60 the maximum lifetime or age of a particle is 2.27 years.</summary>
    public float LifetimeGranularity { get; set; } = 1f / 60;
    internal int particleId;
    public ParticleSystem() { SetRadius(1); SetDensity(1); }
    ~ParticleSystem() { while (ParticleGroupList.Count > 0) DestroyParticleGroup(ParticleGroupList[0]); }
    public int CreateParticleForGroup(ParticleGroupDef groupDef, Transform transform, Vector2 position)
    {
        ParticleDef particleDef = new()
        { flags = groupDef.Flags, position = transform.TransformPoint(position), color = groupDef.Color, lifetime = groupDef.Lifetime, group = groupDef.Group, userData = groupDef.UserData };
        Vector2 p = particleDef.position - groupDef.Position;
        particleDef.velocity = groupDef.LinearVelocity + Vector2.CrossSV(groupDef.AngularVelocity, p);
        return CreateParticle(particleDef);
    }
    void CreateParticlesStrokeShapeForGroup(ChainSegment shape, ParticleGroupDef groupDef, Transform transform)
    {
        float stride = groupDef.Stride;
        if (stride == 0) stride = GetParticleStride();
        float positionOnEdge = 0;
        Vector2 d = shape.segment.point2 - shape.segment.point1;
        float edgeLength = d.Length();
        while (positionOnEdge < edgeLength)
        {
            Vector2 p = shape.segment.point1 + positionOnEdge / edgeLength * d;
            CreateParticleForGroup(groupDef, transform, p);
            positionOnEdge += stride;
        }
        positionOnEdge -= edgeLength;
    }
    void CreateParticlesChainShapeForGroup(ChainShape shape, ParticleGroupDef groupDef, Transform transform)
    {
        float stride = groupDef.Stride;
        if (stride == 0) stride = GetParticleStride();
        float positionOnEdge = 0;
        for (int childIndex = 0; childIndex < shape.shapeIndices.Length; childIndex++)
        {
            ChainSegment edge = (ChainSegment)World.shapes[shape.shapeIndices[childIndex]].shape;
            Vector2 d = edge.segment.point2 - edge.segment.point1;
            float edgeLength = d.Length();
            while (positionOnEdge < edgeLength)
            {
                Vector2 p = edge.segment.point1 + positionOnEdge / edgeLength * d;
                CreateParticleForGroup(groupDef, transform, p);
                positionOnEdge += stride;
            }
            positionOnEdge -= edgeLength;
        }
    }
    void CreateParticlesFillShapeForGroup(Shape shape, ParticleGroupDef groupDef, Transform transform, bool triangleGrid)
    {
        float stride = groupDef.Stride;
        if (stride == 0) stride = GetParticleStride();
        bool shiftX = false; float strideY = stride * (triangleGrid ? MathF.Sqrt(3) / 2 : 1);
        Transform identity = Transform.Identity;
        AABB aabb = shape.ComputeAABB(identity);
        for (float y = MathF.Floor(aabb.lowerBound.y / strideY) * strideY; y < aabb.upperBound.y; y += strideY)
        {
            for (float x = (MathF.Floor(aabb.lowerBound.x / stride) - (shiftX ? 0.5f : 0)) * stride; x < aabb.upperBound.x; x += stride)
            {
                Vector2 p = new(x, y);
                if (shape.shape.TestPoint(p))
                    CreateParticleForGroup(groupDef, transform, p);
            }
            if (triangleGrid) shiftX = !shiftX;
        }
    }
    public void CreateParticlesWithShapeForGroup(Shape shape, ParticleGroupDef groupDef, Transform transform, bool triangleGrid)
    {
        switch (shape.shape)
        {
            case ChainSegment s: CreateParticlesStrokeShapeForGroup(s, groupDef, transform); break;
            case Polygon or Circle: CreateParticlesFillShapeForGroup(shape, groupDef, transform, triangleGrid); break;
            default: Debug.Assert(false); break;
        }
    }
    /*class CompositeShape : Shape
    {
        Shape[] m_shapes;
        public CompositeShape(Shape[] shapes) { m_shapes = shapes; }
        public override int ChildCount => 1;
        public override Shape Clone() => throw new NotImplementedException();
        public override void ComputeAABB(out AABB aabb, ref Transform transform, int childIndex)
        {
            aabb.lowerBound.x = aabb.lowerBound.y = float.MaxValue;
            aabb.upperBound.x = aabb.upperBound.y = float.MinValue;
            Debug.Assert(childIndex == 0);
            for (int i = 0; i < m_shapes.Length; i++) for (int j = 0; j < m_shapes[j].ChildCount; j++)
                { AABB subaabb = m_shapes[i].ComputeAABB(transform); aabb = AABB.Union(aabb, subaabb); }
        }
        public override void ComputeDistance(Transform transform, Vector2 point, out float distance, out Vector2 normal, int childIndex)
            => throw new NotImplementedException();
        public override float ComputeSubmergedArea(ref Vector2 normal, float offset, ref Transform xf, out Vector2 sc)
        {
            sc = Vector2.Zero;
            float area = 0; for (int i = 0; i < m_shapes.Length; i++) area += m_shapes[i].ComputeSubmergedArea(ref normal, offset, ref xf, out sc); return area;
        }
        public override bool RayCast(out RayCastOutput output, ref RayCastInput input, ref Transform transform, int childIndex,
            bool lowerLimit = true, bool upperLimit = true) => throw new NotImplementedException();
        public override bool TestPoint(ref Transform transform, Vector2 point)
        { for (int i = 0; i < m_shapes.Length; i++) if (m_shapes[i].shape.TestPoint(ref transform, point)) return true; return false; }
        protected override void ComputeProperties() => throw new NotImplementedException();
    }*/
    public void CreateParticlesWithShapesForGroup(Shape[] shapes, ParticleGroupDef groupDef, Transform transform, bool triangleGrid)
    {
        float stride = groupDef.Stride;
        if (stride == 0) stride = GetParticleStride();
        bool shiftX = false; float strideY = stride * (triangleGrid ? MathF.Sqrt(3) / 2 : 1);
        Transform identity = Transform.Identity;
        AABB aabb;
        aabb.lowerBound.x = aabb.lowerBound.y = float.MaxValue;
        aabb.upperBound.x = aabb.upperBound.y = float.MinValue;
        for (int i = 0; i < shapes.Length; i++) { AABB subaabb = shapes[i].ComputeAABB(transform); aabb = AABB.Union(aabb, subaabb); }
        for (float y = MathF.Floor(aabb.lowerBound.y / strideY) * strideY; y < aabb.upperBound.y; y += strideY)
        {
            for (float x = (MathF.Floor(aabb.lowerBound.x / stride) - (shiftX ? 0.5f : 0)) * stride; x < aabb.upperBound.x; x += stride)
            {
                Vector2 p = new(x, y);
                for (int i = 0; i < shapes.Length; i++) if (shapes[i].shape.TestPoint(p))
                        CreateParticleForGroup(groupDef, transform, p);
            }
            if (triangleGrid) shiftX = !shiftX;
        }
    }
    public int CloneParticle(int oldIndex, ParticleGroup group)
    {
        ParticleDef def = new() { flags = FlagsBuffer[oldIndex], position = PositionBuffer[oldIndex], velocity = PositionBuffer[oldIndex] };
        if (ColorBuffer != null) def.color = ColorBuffer[oldIndex];
        if (UserDataBuffer != null) def.userData = UserDataBuffer[oldIndex];
        def.group = group;
        int newIndex = CreateParticle(def);
        if (HandleIndexBuffer != null)
        {
            ParticleHandle handle = HandleIndexBuffer[oldIndex];
            if (handle != null) handle.Index = newIndex;
            HandleIndexBuffer[newIndex] = handle;
            HandleIndexBuffer[oldIndex] = null;
        }
        if (LastBodyContactStepBuffer != null) LastBodyContactStepBuffer[newIndex] = LastBodyContactStepBuffer[oldIndex];
        if (BodyContactCountBuffer != null) BodyContactCountBuffer[newIndex] = BodyContactCountBuffer[oldIndex];
        if (ConsecutiveContactStepsBuffer != null) ConsecutiveContactStepsBuffer[newIndex] = ConsecutiveContactStepsBuffer[oldIndex];
        if (HasForce) ForceBuffer[newIndex] = ForceBuffer[oldIndex];
        if (HasImpulse) ImpulseBuffer[newIndex] = ImpulseBuffer[oldIndex];
        if (StaticPressureBuffer != null) StaticPressureBuffer[newIndex] = StaticPressureBuffer[oldIndex];
        if (ExpirationTimeBuffer != null) ExpirationTimeBuffer[newIndex] = ExpirationTimeBuffer[oldIndex];
        return newIndex;
    }
    public unsafe void DestroyParticleGroup(ParticleGroup group)
    {
        Debug.Assert(ParticleGroupList.Count > 0);
        Debug.Assert(group != null);
        SetGroupFlags(group, 0);
        fixed (ParticleGroup* g = GroupBuffer) NativeMemory.Clear(g + group.BufferIndex, (nuint)((group.LastIndex - group.BufferIndex) * sizeof(nint)));
        ParticleGroupList.Remove(group);
        //how to mass delete particles? find out BlockAllocator.cpp
    }
    public void UpdatePairsAndTriads(int firstIndex, int lastIndex, ConnectionFilter filter)
    {
        Debug.Assert(firstIndex <= lastIndex); ParticleFlag particleFlags = 0;
        for (int i = firstIndex; i < lastIndex; i++) particleFlags |= FlagsBuffer[i];
        if ((particleFlags & pairFlags) != 0)
        {
            for (int k = 0; k < ContactBuffer.Count; k++)
            {
                ParticleContact contact = ContactBuffer[k];
                int a = contact.IndexA, b = contact.IndexB;
                ParticleFlag af = FlagsBuffer[a], bf = FlagsBuffer[b];
                ParticleGroup groupA = GroupBuffer[a], groupB = GroupBuffer[b];
                if (a >= firstIndex && a < lastIndex && b >= firstIndex && b < lastIndex &&
                    !(af | bf).HasFlag(ParticleFlag.Zombie) && ((af | bf) & pairFlags) != 0 && (filter.IsNecessary(a) || filter.IsNecessary(b)) &&
                    ParticleCanBeConnected(af, groupA) && ParticleCanBeConnected(bf, groupB) && filter.ShouldCreatePair(a, b))
                {
                    ParticlePair pair = new() { indexA = a, indexB = b, flags = contact.Flags, distance = Vector2.Distance(PositionBuffer[a], PositionBuffer[b]),
                        strength = Math.Min(groupA?.m_strength ?? 1, groupB?.m_strength ?? 1) };
                    PairBuffer.Add(pair);
                }
            }
            PairBuffer = Unique(HPCsharp.ParallelAlgorithm.SortMergePseudoInPlacePar(PairBuffer,
                Comparer<ParticlePair>.Create(ComparePairIndices)), new ParticlePairComparer()).ToList();
        }
        if ((particleFlags & triadFlags) != 0)
        {
            VoronoiDiagram diagram = new(lastIndex - firstIndex);
            for (int i = firstIndex; i < lastIndex; i++)
            {
                ParticleFlag flags = FlagsBuffer[i];
                ParticleGroup group = GroupBuffer[i];
                if (!flags.HasFlag(ParticleFlag.Zombie) && ParticleCanBeConnected(flags, group))
                    diagram.AddGenerator(PositionBuffer[i], i, filter.IsNecessary(i));
            }
            diagram.Generate(Box2D.ParticleStride / 2, Box2D.ParticleStride * 2);
            diagram.GetNodes((a, b, c) =>
            {
                ParticleFlag af = FlagsBuffer[a], bf = FlagsBuffer[b], cf = FlagsBuffer[c];
                if (((af | bf | cf) & triadFlags) != 0 && filter.ShouldCreateTriad(a, b, c))
                {
                    Vector2 pa = PositionBuffer[a], pb = PositionBuffer[b], pc = PositionBuffer[c];
                    Vector2 dab = pa - pb, dbc = pb - pc, dca = pc - pa;
                    float maxDistanceSquared = Box2D.MaxTriadDistanceSquared * GetSquaredDiameter();
                    if (Vector2.Dot(dab, dab) > maxDistanceSquared || Vector2.Dot(dbc, dbc) > maxDistanceSquared || Vector2.Dot(dca, dca) > maxDistanceSquared) return;
                    ParticleGroup groupA = GroupBuffer[a], groupB = GroupBuffer[b], groupC = GroupBuffer[c];
                    ParticleTriad triad = new()
                    {
                        indexA = a, indexB = b, indexC = c, flags = af | bf | cf,
                        strength = Math.Min(Math.Min(groupA?.m_strength ?? 1, groupB?.m_strength ?? 1), groupC?.m_strength ?? 1),
                        ka = -Vector2.Dot(dca, dab), kb = -Vector2.Dot(dab, dbc), kc = -Vector2.Dot(dbc, dca),
                        s = Vector2.Cross(pa, pb) + Vector2.Cross(pb, pc) + Vector2.Cross(pc, pa)
                    };
                    Vector2 midPoint = (pa + pb + pc) / 3;
                    triad.pa = pa - midPoint; triad.pb = pb - midPoint; triad.pc = pc - midPoint;
                    TriadBuffer.Add(triad);
                }
            });
            TriadBuffer = Unique(HPCsharp.ParallelAlgorithm.SortMergePseudoInPlacePar(TriadBuffer,
                Comparer<ParticleTriad>.Create(CompareTriadIndices)), new ParticleTriadComparer()).ToList();
        }
    }
    static IEnumerable<T> Unique<T>(IEnumerable<T> e, IEqualityComparer<T> comparer) //std::unique
    {
        T firstValue = default; bool first = true;
        foreach (var item in e)
        {
            if (first) yield return item;
            else if (!comparer.Equals(item, firstValue)) yield return item;
            firstValue = item;
        }
    }
    class ReactiveFilter : ConnectionFilter
    {
        ParticleFlag[] m_flagsBuffer; public ReactiveFilter(ParticleFlag[] flagsBuffer) => m_flagsBuffer = flagsBuffer;
        public override bool IsNecessary(int index) => m_flagsBuffer[index].HasFlag(ParticleFlag.Reactive);
    }
    public void UpdatePairsAndTriadsWithReactiveParticles()
    {
        UpdatePairsAndTriads(0, Count, new ReactiveFilter(FlagsBuffer));
        for (int i = 0; i < Count; i++) FlagsBuffer[i] &= ~ParticleFlag.Reactive;
        AllParticleFlags &= ~ParticleFlag.Reactive;
    }
    public static int ComparePairIndices(ParticlePair a, ParticlePair b)
    { int diffA = a.indexA - b.indexA; if (diffA != 0) return diffA.CompareTo(0); return a.indexB.CompareTo(b.indexB); }
    struct ParticlePairComparer : IEqualityComparer<ParticlePair>
    {
        public bool Equals(ParticlePair x, ParticlePair y) => MatchPairIndices(x, y);
        public int GetHashCode([DisallowNull] ParticlePair obj) =>
            HashCode.Combine(obj.indexA, obj.indexB, obj.distance, obj.distance, obj.flags);
    }
    public static bool MatchPairIndices(ParticlePair a, ParticlePair b) => a.indexA == b.indexA && a.indexB == b.indexB;
    public static int CompareTriadIndices(ParticleTriad a, ParticleTriad b)
    {
        int diffA = a.indexA - b.indexA; if (diffA != 0) return diffA.CompareTo(0);
        int diffB = a.indexB - b.indexB; if (diffB != 0) return diffB.CompareTo(0); return a.indexC.CompareTo(b.indexC);
    }
    struct ParticleTriadComparer : IEqualityComparer<ParticleTriad>
    {
        public bool Equals(ParticleTriad x, ParticleTriad y) => MatchTriadIndices(x, y);
        public int GetHashCode([DisallowNull] ParticleTriad obj) =>
             HashCode.Combine(obj.indexA, obj.indexB, obj.indexC, obj.flags);
    }
    public static bool MatchTriadIndices(ParticleTriad a, ParticleTriad b) => a.indexA == b.indexA && a.indexB == b.indexB && a.indexC == b.indexC;
    unsafe static void InitializeParticleLists(ParticleGroup group, ParticleListNode[] nodeBuffer)
    {
        fixed (ParticleListNode* pln = nodeBuffer) for (int i = 0; i < group.ParticleCount; i++)
                nodeBuffer[i] = new() { count = 1, index = i + group.BufferIndex, list = pln + i };
    }
    unsafe void MergeParticleListsInContact(ParticleGroup group, ParticleListNode[] nodeBuffer)
    {
        for (int k = 0; k < ContactBuffer.Count; k++)
        {
            ParticleContact contact = ContactBuffer[k];
            int a = contact.IndexA, b = contact.IndexB;
            if (!group.ContainsParticle(a) || !group.ContainsParticle(b)) continue;
            ParticleListNode* listA = nodeBuffer[a - group.BufferIndex].list,
                listB = nodeBuffer[b - group.BufferIndex].list;
            if (listA == listB) continue;
            if (listA->count < listB->count) { ParticleListNode* l = listA; listA = listB; listB = l; }
            Debug.Assert(listA->count >= listB->count);
            MergeParticleLists(listA, listB);
        }
    }
    unsafe static void MergeParticleLists(ParticleListNode* listA, ParticleListNode* listB)
    {
        Debug.Assert(listA != listB);
        ParticleListNode* b = listB;
        while (true)
        {
            b->list = listA; ParticleListNode* nextB = b->next;
            if (nextB != null) b = nextB; else { b->next = listA->next; break; }
        }
        listA->next = listB; listA->count += listB->count; listB->count = 0;
    }
    unsafe static ParticleListNode* FindLongestParticleList(ParticleGroup group, ParticleListNode[] nodeBuffer)
    {
        fixed (ParticleListNode* nb = nodeBuffer)
        {
            ParticleListNode* result = nb;
            for (int i = 0; i < group.ParticleCount; i++)
            {
                ParticleListNode* node = nb + i;
                if (result->count < node->count) result = node;
            }
            return result;
        }
    }
    unsafe void MergeZombieParticleListNodes(ParticleGroup group, ParticleListNode[] nodeBuffer, ParticleListNode* survivingList)
    {
        fixed (ParticleListNode* pln = nodeBuffer) for (int i = 0; i < group.ParticleCount; i++)
            {
                ParticleListNode* node = pln + i;
                if (node != survivingList && (FlagsBuffer[node->index] & ParticleFlag.Zombie) > 0)
                    MergeParticleListAndNode(survivingList, node);
            }
    }
    unsafe static void MergeParticleListAndNode(ParticleListNode* list, ParticleListNode* node)
    {
        Debug.Assert(node != list); Debug.Assert(node->list == node); Debug.Assert(node->count == 1);
        node->list = list; node->next = list->next; list->next = node; list->count++; node->count = 0;
    }
    unsafe void CreateParticleGroupsFromParticleList(ParticleGroup group, ParticleListNode[] nodeBuffer, ParticleListNode* survivingList)
    {
        ParticleGroupDef def = new() { GroupFlags = group.m_groupFlags, UserData = group.UserData };
        fixed (ParticleListNode* pln = nodeBuffer) for (int i = 0; i < group.ParticleCount; i++)
            {
                ParticleListNode* list = pln + i;
                if (list->count == 0 || list == survivingList) continue;
                Debug.Assert(list->list == list);
                ParticleGroup newGroup = CreateParticleGroup(def);
                for (ParticleListNode* node = list; node != null; node = node->next)
                {
                    int oldIndex = node->index;
                    Debug.Assert((FlagsBuffer[oldIndex] & ParticleFlag.Zombie) == 0);
                    int newIndex = CloneParticle(oldIndex, newGroup);
                    FlagsBuffer[oldIndex] |= ParticleFlag.Zombie;
                    node->index = newIndex;
                }
            }
    }
    void UpdatePairsAndTriadsWithParticleList(ParticleGroup group, ParticleListNode[] nodeBuffer)
    {
        int bufferIndex = group.BufferIndex;
        var p = CollectionsMarshal.AsSpan(PairBuffer);
        for (int k = 0; k < PairBuffer.Count; k++)
        {
            ref ParticlePair pair = ref p[k];
            int a = pair.indexA, b = pair.indexB;
            if (group.ContainsParticle(a)) pair.indexA = nodeBuffer[a - bufferIndex].index;
            if (group.ContainsParticle(b)) pair.indexB = nodeBuffer[b - bufferIndex].index;
        }
        var t = CollectionsMarshal.AsSpan(TriadBuffer);
        for (int k = 0; k < TriadBuffer.Count; k++)
        {
            ref ParticleTriad triad = ref t[k];
            int a = triad.indexA, b = triad.indexB, c = triad.indexC;
            if (group.ContainsParticle(a)) triad.indexA = nodeBuffer[a - bufferIndex].index;
            if (group.ContainsParticle(b)) triad.indexB = nodeBuffer[b - bufferIndex].index;
            if (group.ContainsParticle(c)) triad.indexC = nodeBuffer[c - bufferIndex].index;
        }
    }
    public void ComputeDepth()
    {
        ParticleContact[] contactGroups = new ParticleContact[ContactBuffer.Count];
        int contactGroupsCount = 0;
        for (int k = 0; k < ContactBuffer.Count; k++)
        {
            ParticleContact contact = ContactBuffer[k];
            int a = contact.IndexA, b = contact.IndexB;
            ParticleGroup groupA = GroupBuffer[a], groupB = GroupBuffer[b];
            if (groupA != null && groupA == groupB && (groupA.m_groupFlags & ParticleGroupFlag.NeedsUpdateDepth) != 0) contactGroups[contactGroupsCount++] = contact;
        }
        ParticleGroup[] groupsToUpdate = new ParticleGroup[ParticleGroupList.Count];
        int groupsToUpdateCount = 0;
        for (int k = 0; k < ParticleGroupList.Count; k++)
        {
            ParticleGroup group = ParticleGroupList[k];
            if ((group.m_groupFlags & ParticleGroupFlag.NeedsUpdateDepth) != 0)
            {
                groupsToUpdate[groupsToUpdateCount++] = group;
                SetGroupFlags(group, group.m_groupFlags & ~ParticleGroupFlag.NeedsUpdateDepth);
                for (int i = group.BufferIndex; i < group.LastIndex; i++) AccumulationBuffer[i] = 0;
            }
        }
        for (int k = 0; k < contactGroupsCount; k++)
        {
            ParticleContact contact = contactGroups[k]; float w = contact.Weight;
            AccumulationBuffer[contact.IndexA] += w; AccumulationBuffer[contact.IndexB] += w;
        }
        Debug.Assert(DepthBuffer != null);
        for (int i = 0; i < groupsToUpdateCount; i++)
        {
            ParticleGroup group = groupsToUpdate[i];
            for (int j = group.BufferIndex; j < group.LastIndex; j++)
            {
                float w = AccumulationBuffer[j];
                DepthBuffer[j] = w < 0.8f ? 0 : float.MaxValue;
            }
        }
        int iterationCount = (int)Math.Sqrt(Count);
        for (int t = 0; t < iterationCount; t++)
        {
            bool updated = false;
            for (int k = 0; k < contactGroupsCount; k++)
            {
                ParticleContact contact = contactGroups[k];
                float r = 1 - contact.Weight;
                ref float ap0 = ref DepthBuffer[contact.IndexA], bp0 = ref DepthBuffer[contact.IndexB]; float ap1 = bp0 + r, bp1 = ap0 + r;
                if (ap0 > ap1) { ap0 = ap1; updated = true; }
                if (bp0 > bp1) { bp0 = bp1; updated = true; }
            }
            if (!updated) break;
        }
        for (int i = 0; i < groupsToUpdateCount; i++)
        {
            ParticleGroup group = groupsToUpdate[i];
            for (int j = group.BufferIndex; j < group.LastIndex; j++)
            {
                ref float p = ref DepthBuffer[i]; if (p < float.MaxValue) p *= GetParticleDiameter(); else p = 0;
            }
        }
    }
    public unsafe InsideBoundsEnumerator GetInsideBoundsEnumenator(AABB aabb)
    {
        uint lowerTag = ComputeTag(GetInverseDiameter() * aabb.lowerBound - 1);
        uint upperTag = ComputeTag(GetInverseDiameter() * aabb.upperBound + 1);
        int firstProxy = ProxyBuffer.FindIndex(x => x.tag >= lowerTag), lastProxy = ProxyBuffer.FindIndex(x => x.tag > upperTag);
        return new(lowerTag, upperTag, ProxyBuffer, firstProxy == -1 ? ProxyBuffer.Count : firstProxy, lastProxy == -1 ? ProxyBuffer.Count : lastProxy);
    }
    public void UpdateAllParticleFlags()
    {
        AllParticleFlags = 0;
        for (int i = 0; i < Count; i++) AllParticleFlags |= FlagsBuffer[i];
        NeedsUpdateAllParticleFlags = false;
    }
    public void UpdateAllGroupFlags()
    {
        AllGroupFlags = 0;
        for (int i = 0; i < ParticleGroupList.Count; i++) AllGroupFlags |= ParticleGroupList[i].m_groupFlags;
        NeedsUpdateAllGroupFlags = false;
    }
    public void AddContact(int a, int b, List<ParticleContact> contacts, ref int index)
    {
        Vector2 d = PositionBuffer[b] - PositionBuffer[a]; float distBtParticlesSq = Vector2.Dot(d, d);
        if (distBtParticlesSq < GetSquaredDiameter())
        { //fixed a bug in the Google original (when particles are equal, weight becomes NaN and particle pulls everything far away)
            float invD = distBtParticlesSq == 0 ? 0 : 1 / MathF.Sqrt(distBtParticlesSq);
            ParticleContact contact = new()
            {
                IndexA = a, IndexB = b, Normal = invD * d, Flags = FlagsBuffer[a] | FlagsBuffer[b],
                Weight = 1 - distBtParticlesSq * invD * GetInverseDiameter()
            };
            if (index != -1 && index < contacts.Count) contacts[index++] = contact; else { contacts.Add(contact); index++; }
        }
    }
    public void FindContacts_Reference(List<ParticleContact> contacts)
    {
        int index = 0, c = 0;
        for (int a = 0; a < ProxyBuffer.Count; a++)
        {
            uint rightTag = ComputeRelativeTag(ProxyBuffer[a].tag, 1, 0);
            for (int b = a + 1; b < ProxyBuffer.Count; b++)
            {
                if (rightTag < ProxyBuffer[b].tag) break;
                AddContact(ProxyBuffer[a].index, ProxyBuffer[b].index, contacts, ref index);
            }
            uint bottomLeftTag = ComputeRelativeTag(ProxyBuffer[a].tag, -1, 1);
            for (; c < ProxyBuffer.Count; c++) if (bottomLeftTag <= ProxyBuffer[c].tag) break;
            uint bottomRightTag = ComputeRelativeTag(ProxyBuffer[a].tag, 1, 1);
            for (int b = c; b < ProxyBuffer.Count; b++)
            {
                if (bottomRightTag < ProxyBuffer[b].tag) break;
                AddContact(ProxyBuffer[a].index, ProxyBuffer[b].index, contacts, ref index);
            }
        }
        contacts.RemoveRange(index, contacts.Count - index);
    }
    struct FindContactCheck { public uint particleIndex; public uint comparatorIndex; }
    struct FindContactInput { public int proxyIndex; public Vector2 position; }
    static int NUM_V32_SLOTS = 4;
    void ReorderForFindContact(FindContactInput[] reordered)
    {
        int i = 0;
        for (; i < Count; i++)
        {
            int proxyIndex = ProxyBuffer[i].index; ref FindContactInput r = ref reordered[i];
            r.proxyIndex = proxyIndex; r.position = PositionBuffer[proxyIndex];
        }
        for (; i < reordered.Length; i++) { ref FindContactInput r = ref reordered[i]; r.proxyIndex = 0; r.position = new(float.MaxValue, float.MaxValue); }
    }
    void GatherChecksOneParticle(uint bound, int startIndex, int particleIndex, ref int nextUncheckedIndex, List<FindContactCheck> checks)
    {
        for (int comparatorIndex = startIndex; comparatorIndex < Count; comparatorIndex += NUM_V32_SLOTS)
        {
            if (ProxyBuffer[comparatorIndex].tag > bound) break;
            FindContactCheck Out = new() { particleIndex = (uint)particleIndex, comparatorIndex = (uint)comparatorIndex };
            checks.Add(Out);
            nextUncheckedIndex = comparatorIndex + NUM_V32_SLOTS;
        }
    }
    void GatherChecks(List<FindContactCheck> checks)
    {
        int bottomLeftIndex = 0;
        for (int particleIndex = 0; particleIndex < Count; particleIndex++)
        {
            uint particleTag = ProxyBuffer[particleIndex].tag;
            uint rightBound = particleTag + relativeTagRight;
            int nextUncheckedIndex = particleIndex + 1;
            GatherChecksOneParticle(rightBound, particleIndex + 1, particleIndex, ref nextUncheckedIndex, checks);
            uint bottomLeftTag = particleTag + relativeTagBottomLeft;
            for (; bottomLeftIndex < Count; bottomLeftIndex++) if (bottomLeftTag <= ProxyBuffer[bottomLeftIndex].tag) break;
            uint bottomRightBound = particleTag + relativeTagBottomRight;
            int bottomStartIndex = Math.Max(bottomLeftIndex, nextUncheckedIndex);
            GatherChecksOneParticle(bottomRightBound, bottomStartIndex, particleIndex, ref nextUncheckedIndex, checks);
        }
    }
    void FindContacts_Simd(List<ParticleContact> contacts)
    {
        FindContactInput[] reordered = new FindContactInput[Count + NUM_V32_SLOTS];
        ReorderForFindContact(reordered);
        List<FindContactCheck> checks = new(Count * 3); //MAX_EXPECTED_CHECKS_PER_PARTICLE
        GatherChecks(checks);
        //The rest is assembly
    }
    void FindContacts(List<ParticleContact> contacts)
    {
        FindContacts_Reference(contacts); //or FindContacts_Simd if implemented
    }
    public static void UpdateProxyTags(uint[] tags, List<Proxy> proxies) //never called
    {
        ParticleParallel(proxies.Count, (start, end, p) => { for (int i = start; i < end; i++) p[i] = p[i] with { tag = tags[p[i].index] }; }, proxies);
    }
    public static bool ProxyBufferHasIndex(int index, List<Proxy> a, int start, int count)
    { for (int j = 0; j < count; j++) if (a[j + start].index == index) return true; return false; }
    public static int NumProxiesWithSameTag(List<Proxy> a, int aI, List<Proxy> b, int bI, int count)
    {
        uint tag = a[0].tag;
        for (int num = 0; num < count; num++) if (a[num + aI].tag != tag || b[num + bI].tag != tag) return num; return count;
    }
    public static bool AreProxyBuffersTheSame(List<Proxy> a, List<Proxy> b)
    {
        if (a.Count != b.Count) return false;
        for (int i = 0; i < a.Count;)
        {
            int numWithSameTag = NumProxiesWithSameTag(a, i, b, i, a.Count - i);
            if (numWithSameTag == 0) return false;
            for (int j = 0; j < numWithSameTag; j++) if (!ProxyBufferHasIndex(a[i + j].index, b, i, numWithSameTag)) return false;
            i += numWithSameTag;
        }
        return true;
    }
    internal void UpdateProxies_Reference(List<Proxy> proxies)
    {
        for (int i = 0; i < proxies.Count; i++)
        { int index = proxies[i].index; proxies[i] = new() { tag = ComputeTag(PositionBuffer[index] * GetInverseDiameter()), index = index }; }
    }
    static void ExtractXY(Vector256<float> pos1, Vector256<float> pos2, out Vector256<float> x, out Vector256<float> y)
    {
         x = Avx.Shuffle(pos1, pos2, 0b10_00_10_00); y = Avx.Shuffle(pos1, pos2, 0b11_01_11_01);
    }
    static Vector256<float> SIMDHasFlag(Vector256<int> v, Vector256<int> flags) =>
        Avx.CompareGreaterThan(Avx.And(v.AsSingle(), flags.AsSingle()), Vector256<float>.Zero);
    static void SpreadTwice(Vector256<float> v, out Vector256<float> low, out Vector256<float> high)
    {
        low = Avx2.PermuteVar8x32(v, Vector256.Create(0, 0, 1, 1, 2, 2, 3, 3));
        high = Avx2.PermuteVar8x32(v, Vector256.Create(4, 4, 5, 5, 6, 6, 7, 7));
    }
    internal unsafe void UpdateProxies_Simd(List<Proxy> proxies) // does not work
    {
        int i = 0; uint[] tags = new uint[Count];
        Vector256<uint> yo = Vector256.Create(yOffset), ys = Vector256.Create((uint)yShift);
        Vector256<float> id = Vector256.Create(GetInverseDiameter()), xs = Vector256.Create((float)xScale), xo = Vector256.Create((float)xOffset);
        fixed (uint* t = tags) fixed (Vector2* pos = PositionBuffer) for (; i + 8 <= Count; i += 8)
            {
                Vector256<float> pos1 = Avx.Multiply(Avx.LoadVector256((float*)(pos + i)), id), pos2 = Avx.Multiply(Avx.LoadVector256((float*)(pos + i + 4)), id);
                ExtractXY(pos1, pos2, out var x, out var y);
                Avx.Store(t + i, Avx2.Add(Avx2.ShiftLeftLogicalVariable(Avx2.Add(Avx.ConvertToVector256Int32WithTruncation(y).AsUInt32(), yo), ys),
                    Avx.ConvertToVector256Int32WithTruncation(Fma.MultiplyAdd(x, xs, xo)).AsUInt32()));
            }
        for (; i < Count; i++) tags[i] = ComputeTag(PositionBuffer[i] * GetInverseDiameter());
        UpdateProxyTags(tags, proxies);
    }
    internal void UpdateProxies(List<Proxy> proxies)
    {
        //if (Avx2.IsSupported) UpdateProxies_Simd(proxies);
        //else
            UpdateProxies_Reference(proxies);
    }
    //internal void SortProxies(ref List<Proxy> proxies) => proxies.Sort((x, y) => x.tag.CompareTo(y.tag));
    internal void FilterContacts(List<ParticleContact> contacts)
    {
        ParticleContactFilter contactFilter = GetParticleContactFilter();
        if (contactFilter == null) return;
        contacts.RemoveAll(x => x.Flags.HasFlag(ParticleFlag.ParticleContactFilter) && !contactFilter.ShouldCollidePP(this, x.IndexA, x.IndexB));
    }
    class FixedSet<T> where T : IEquatable<T>, IComparable<T>
    {
        public T[] Set { get; private set; } public bool[] Valid { get; set; } public int Count { get; set; }
        public int Allocate(int numberOfObjects)
        {
            if (numberOfObjects > 0)
            {
                Set = new T[numberOfObjects]; Valid = new bool[numberOfObjects];
                for (int i = 0; i < Valid.Length; i++) Valid[i] = true;
                Count = numberOfObjects;
            }
            return Count;
        }
        public int GetIndex(T item)
        {
            for (int i = 0; i < Count; i++) if (Valid[i] && Set[i].Equals(item)) return i;
            return -1;
        }
        public void Invalidate(int index) => Valid[index] = false;
        public int FindItemIndex(T item, Func<T, T, bool> comparerPredicate)
        {
            if (Count > 0)
                for (int i = 0; i < Count; i++) if (Valid[i] && comparerPredicate(Set[i], item)) return i;
            return -1;
        }
    }
    class FixtureParticleSet : FixedSet<(Shape, int)>
    {
        public unsafe void Initialize(List<ParticleBodyContact> bodyContacts, ParticleFlag[] particleFlagsBuffer)
        {
            if (Allocate(bodyContacts.Count) > 0)
            {
                int insertedContacts = 0;
                for (int i = 0; i < bodyContacts.Count; i++)
                {
                    ref (Shape, int) fixtureParticle = ref Set[insertedContacts];
                    ParticleBodyContact bodyContact = bodyContacts[i];
                    if (bodyContact.index == Box2D.InvalidParticleIndex || (particleFlagsBuffer[bodyContact.index] & ParticleFlag.FixtureContactListener) == 0) continue;
                    fixtureParticle.Item1 = bodyContact.fixture; fixtureParticle.Item2 = bodyContact.index;
                    insertedContacts++;
                }
                Count = insertedContacts;
                HPCsharp.ParallelAlgorithm.SortMergeInPlacePar(Set, 0, insertedContacts, Comparer<(Shape, int)>.Create((a, b) =>
                {
                    int c = ((nint)(&a.Item1)).CompareTo((nint)(&b.Item1));
                    return c == 0 ? a.Item2.CompareTo(b.Item2) : c;
                }));
            }
        }
        public unsafe int Find((Shape, int) pair)
        {
            int index = FindItemIndex(pair, (a, b) => ((nint)(&a.Item1)).CompareTo((nint)(&b.Item1)) >= 0 || a.Item2 >= b.Item2);
            if (index >= 0)
            {
                (Shape, int) found = Set[index];
                if (found.Item2 != pair.Item2 || found.Item1 != pair.Item1) index = -1;
            }
            return index;
        }
    }
    class ParticlePairSet : FixedSet<(int, int)>
    {
        public void Initialize(List<ParticleContact> contacts, ParticleFlag[] particleFlagsBuffer)
        {
            if (Allocate(contacts.Count) > 0)
            {
                int insertedContacts = 0;
                for (int i = 0; i < contacts.Count; i++)
                {
                    ref (int, int) pair = ref Set[i];
                    ParticleContact contact = contacts[i];
                    if (contact.IndexA == Box2D.InvalidParticleIndex || contact.IndexB == Box2D.InvalidParticleIndex ||
                        ((particleFlagsBuffer[contact.IndexA] | particleFlagsBuffer[contact.IndexB]) & ParticleFlag.ParticleContactListener) == 0)
                        continue;
                    pair.Item1 = contact.IndexA; pair.Item2 = contact.IndexB; insertedContacts++;
                }
                Count = insertedContacts;
                HPCsharp.ParallelAlgorithm.SortMergeInPlacePar(Set, 0, insertedContacts, Comparer<(int, int)>.Create((a, b) =>
                {
                    int c = a.Item1.CompareTo(b.Item1);
                    return c == 0 ? a.Item2.CompareTo(b.Item2) : c;
                }));
            }
        }
        public int Find((int, int) pair)
        {
            int index = FindItemIndex(pair, (a, b) => a.Item1 >= b.Item1 || a.Item2 >= b.Item2);
            if (index >= 0)
            {
                (int, int) found = Set[index];
                if (found.Item1 != pair.Item1 || found.Item2 != pair.Item2) index = -1;
            }
            if (index < 0)
            {
                index = FindItemIndex((pair.Item2, pair.Item1), (a, b) => a.Item1 >= b.Item1 || a.Item2 >= b.Item2);
                if (index >= 0)
                {
                    (int, int) found = Set[index];
                    if (found.Item1 != pair.Item2 || found.Item2 != pair.Item1) index = -1;
                }
            }
            return index;
        }
    }
    void NotifyContactListenerPreContact(ParticlePairSet particlePairs)
    {
        ParticleContactListener contactListener = GetParticleContactListener();
        if (contactListener == null) return;
        particlePairs.Initialize(ContactBuffer, GetFlagsBuffer());
    } //unfinished?
    void NotifyContactListenerPostContact(ParticlePairSet particlePairs)
    {
        ParticleContactListener contactListener = GetParticleContactListener();
        if (contactListener == null) return;
        for (int i = 0; i < ContactBuffer.Count; i++)
        {
            if (ContactBuffer[i].Flags.HasFlag(ParticleFlag.ParticleContactListener))
            {
                int itemIndex = particlePairs.Find((ContactBuffer[i].IndexA, ContactBuffer[i].IndexB));
                if (itemIndex >= 0) particlePairs.Invalidate(itemIndex);
                else contactListener.BeginContactPPC?.Invoke(this, ContactBuffer[i]);
            }
        }
        for (int i = 0; i < particlePairs.Count; i++) if (particlePairs.Valid[i])
                contactListener.EndContactP?.Invoke(this, particlePairs.Set[i].Item1, particlePairs.Set[i].Item2);
    }
    void UpdateContacts(bool exceptZombie)
    {
        UpdateProxies(ProxyBuffer); ProxyBuffer = HPCsharp.ParallelAlgorithm.SortMergePseudoInPlacePar(ProxyBuffer, Comparer<Proxy>.Create((x, y) => x.tag.CompareTo(y.tag)));
        ParticlePairSet particlePairs = new();
        NotifyContactListenerPreContact(particlePairs);
        FindContacts(ContactBuffer); FilterContacts(ContactBuffer);
        NotifyContactListenerPostContact(particlePairs);
        if (exceptZombie) ContactBuffer.RemoveAll(x => x.Flags.HasFlag(ParticleFlag.Zombie));
    }
    void NotifyBodyContactListenerPreContact(FixtureParticleSet fixtureSet)
    {
        ParticleContactListener contactListener = GetFixtureContactListener();
        if (contactListener == null) return;
        fixtureSet.Initialize(BodyContactBuffer, GetFlagsBuffer());
    }
    void NotifyBodyContactListenerPostContact(FixtureParticleSet fixtureSet)
    {
        ParticleContactListener contactListener = GetFixtureContactListener();
        if (contactListener == null) return;
        for (int i = 0; i < BodyContactBuffer.Count; i++)
        {
            Debug.Assert(BodyContactBuffer != null);
            if (FlagsBuffer[BodyContactBuffer[i].index].HasFlag(ParticleFlag.FixtureContactListener))
            {
                (Shape, int) fixtureParticleToFind = (BodyContactBuffer[i].fixture, BodyContactBuffer[i].index);
                int index = fixtureSet.Find(fixtureParticleToFind);
                if (index >= 0) fixtureSet.Invalidate(index);
                else contactListener.BeginContactPPBC?.Invoke(this, BodyContactBuffer[i]);
            }
        }
        (Shape, int)[] fixtureParticles = fixtureSet.Set;
        bool[] fixtureParticlesValid = fixtureSet.Valid;
        for (int i = 0; i < fixtureParticles.Length; i++) if (fixtureParticlesValid[i])
                contactListener.EndContactFP?.Invoke(fixtureParticles[i].Item1, this, fixtureParticles[i].Item2);
    }
    class FixtureParticleQueryCallback : ParticleQueryCallback
    {
        public ReportFixtureAndParticle QueryReportFixtureAndParticle;
        ParticleSystem system;
        public bool ReportFixture(World world, Shape f)
        {
            if (f.sensorIndex != -1) return true;
            Transform transform = world.GetBodyTransform(f.bodyId);
            AABB aabb = f.shape.ComputeAABB(transform);
            InsideBoundsEnumerator enumerator = system.GetInsideBoundsEnumenator(aabb);
            int index;
            while ((index = enumerator.GetNext()) >= 0) QueryReportFixtureAndParticle?.Invoke(f, 0, index);
            return true;
        }
        public FixtureParticleQueryCallback(ParticleSystem system)
        {
            ShouldQueryParticleSystem = s => false;
            this.system = system;
        }
        public delegate void ReportFixtureAndParticle(Shape fixture, int childIndex, int index);
    }
    unsafe void UpdateBodyContacts()
    {
        FixtureParticleSet fixtureSet = new();
        NotifyBodyContactListenerPreContact(fixtureSet);
        if (StuckThreshold > 0)
        {
            int i = 0;
            if (Avx2.IsSupported)
            {
                Vector256<int> t = Vector256.Create(Timestamp);
                fixed (int* bccb = BodyContactCountBuffer) fixed (int* ccsb = ConsecutiveContactStepsBuffer)
                fixed (int* lbcsb = LastBodyContactStepBuffer) for (; i + 8 <= Count; i += 8)
                    {
                        Avx.Store(bccb + i, Vector256<int>.Zero);
                        Avx.Store(ccsb + i, Avx2.BlendVariable(Avx.LoadVector256(ccsb + i), Vector256<int>.Zero, Avx2.CompareGreaterThan(t, Avx.LoadVector256(lbcsb + i))));
                    }
            }
            for (; i < Count; i++)
            {
                BodyContactCountBuffer[i] = 0;
                if (Timestamp > LastBodyContactStepBuffer[i] + 1) ConsecutiveContactStepsBuffer[i] = 0;
            }
        }
        int bcb = 0, spb = 0;
        float m_radius = 0.5f * GetParticleDiameter() * Box2D.FixtureParticleCollisionRadiusScaler, m_inverseMass = GetParticleInvMass(),
            m_inverseRadius = GetInverseDiameter() * 2 / Box2D.FixtureParticleCollisionRadiusScaler;
        ParticleContactFilter contactFilter = GetFixtureContactFilter();
        FixtureParticleQueryCallback callback = new(this)
        {
            QueryReportFixtureAndParticle = (f, i, a) =>
            {
                Vector2 ap = PositionBuffer[a];
                Body b = World.bodies[f.bodyId];
                BodySim sim = World.GetBodySim(b);
                DistanceInput input = new()
                {
                    proxyA = f.MakeDistanceProxy(),
                    proxyB = new Circle { center = ap }.MakeProxy(),
                    transformA = sim.transform,
                    transformB = Transform.Identity,
                };
                SimplexCache cache = new();
                DistanceOutput output = input.ShapeDistance(ref cache, null);
                /*if (output.normal.x == 0 && output.normal.y == 0 && output.distance == 0)
                {
                    Manifold m = ContactRegister.ComputeManifold(f, sim.transform,
                        new() { shape = new Circle { center = ap }, type = ShapeType.Circle }, Transform.Identity);
                    output.distance = m.point0.separation;
                    output.normal = m.normal;
                }*/
                if (output.distance < m_radius && (contactFilter == null ||
                    !GetFlagsBuffer()[a].HasFlag(ParticleFlag.FixtureContactFilter) || contactFilter.ShouldCollideFP(f, this, a)))
                {
                    float bI = b.inertia - b.mass * sim.localCenter.LengthSquared();
                    float rpn = Vector2.Cross(ap - sim.center, output.normal),
                        invM = (FlagsBuffer[a].HasFlag(ParticleFlag.Wall) ? 0 : m_inverseMass) + sim.invMass + (bI > 0 ? 1 / bI : 0) * rpn * rpn;
                    ParticleBodyContact pbc = new()
                    {
                        index = a,
                        body = b,
                        fixture = f,
                        normal = -output.normal,
                        mass = invM > 0 ? 1 / invM : 0,
                        weight = 1 - output.distance * m_inverseRadius //* (PointCollision ? 1 : 0.5)
                    };
                    if (bcb < BodyContactBuffer.Count) BodyContactBuffer[bcb++] = pbc; else { BodyContactBuffer.Add(pbc); bcb++; }
                    DetectStuckParticle(a, ref spb);
                }
            }
        };
        WorldAPI.World_OverlapAABB(new() { generation = World.generation, index1 = World }, ComputeAABB(), new(),
            (shape, _) => callback.ReportFixture(shape.world0, shape.world0.GetShape(shape)), null, callback);
        BodyContactBuffer.RemoveRange(bcb, BodyContactBuffer.Count - bcb);
        StuckParticleBuffer.RemoveRange(spb, StuckParticleBuffer.Count - spb);
        if (StrictContactCheck) RemoveSpuriousBodyContacts();
        NotifyBodyContactListenerPostContact(fixtureSet);
    }
    static void ParticleParallel<T>(int length, Action<int, int, T> callback, T local = default)
    {
        if (length == 0) return;
        var batchSize = (int)Math.Ceiling((float)length / Environment.ProcessorCount);
        var batches = (int)Math.Ceiling((float)length / batchSize);
        //batchSize = length; batches = 1;
        //int countdown = batches;
        System.Threading.CountdownEvent countdown = new(batches);
        for (int i = 0; i < batches; i++)
        {
            var start = i * batchSize;
            var end = Math.Min(start + batchSize, length);
            System.Threading.ThreadPool.QueueUserWorkItem(w => { callback(start, end, (T)w); countdown.Signal(); }, local);
        }
        countdown.Wait();
        //while (countdown.CurrentCount > 0) X86Base.Pause();
    }
    unsafe internal void Solve(float dt, float inv_dt, int particleIterations)
    {
        if (Count == 0 || locked) return; locked = true;
        // If particle lifetimes are enabled, destroy particles that are too old.
        if (ExpirationTimeBuffer != null) SolveLifetimes(dt, inv_dt);
        if (AllParticleFlags.HasFlag(ParticleFlag.Zombie)) SolveZombie();
        if (NeedsUpdateAllParticleFlags) UpdateAllParticleFlags();
        if (NeedsUpdateAllGroupFlags) UpdateAllGroupFlags();
        if (Paused) return;
        dt /= particleIterations;
        inv_dt *= particleIterations;
        for (IterationIndex = 0; IterationIndex < particleIterations; IterationIndex++)
        {
            ++Timestamp;
            UpdateContacts(false);
            UpdateBodyContacts();
            ComputeWeight();
            if (AllGroupFlags.HasFlag(ParticleGroupFlag.NeedsUpdateDepth)) ComputeDepth();
            if (AllParticleFlags.HasFlag(ParticleFlag.Reactive)) UpdatePairsAndTriadsWithReactiveParticles();
            if (HasForce) SolveForce(dt, inv_dt);
            if (HasImpulse) SolveImpulse(dt, inv_dt);
            if (AllParticleFlags.HasFlag(ParticleFlag.Viscous)) SolveViscous();
            if (AllParticleFlags.HasFlag(ParticleFlag.Repulsive)) SolveRepulsive(dt, inv_dt);
            if (AllParticleFlags.HasFlag(ParticleFlag.Powder)) SolvePowder(dt, inv_dt);
            if (AllParticleFlags.HasFlag(ParticleFlag.Tensile)) SolveTensile(dt, inv_dt);
            if (AllGroupFlags.HasFlag(ParticleGroupFlag.Solid)) SolveSolid(dt, inv_dt);
            if (AllParticleFlags.HasFlag(ParticleFlag.ColorMixing)) SolveColorMixing();
            SolveGravity(dt, inv_dt);
            if (AllParticleFlags.HasFlag(ParticleFlag.StaticPressure)) SolveStaticPressure(dt, inv_dt);
            SolvePressure(dt, inv_dt); SolveDamping(dt, inv_dt);
            if (AllParticleFlags.HasFlag(extraDampingFlags)) SolveExtraDamping();
            // SolveElastic and SolveSpring refer the current velocities for numerical stability, they should be called as late as possible.
            if (AllParticleFlags.HasFlag(ParticleFlag.Elastic)) SolveElastic(dt, inv_dt);
            if (AllParticleFlags.HasFlag(ParticleFlag.Spring)) SolveSpring(dt, inv_dt);
            LimitVelocity(dt, inv_dt);
            if (AllGroupFlags.HasFlag(ParticleGroupFlag.Rigid)) SolveRigidDamping();
            if (AllParticleFlags.HasFlag(ParticleFlag.Barrier)) SolveBarrier(dt, inv_dt);
            // SolveCollision, SolveRigid and SolveWall should be called after
            // other force functions because they may require particles to have specific velocities.
            SolveCollision(dt, inv_dt);
            if (AllGroupFlags.HasFlag(ParticleGroupFlag.Rigid)) SolveRigid(dt, inv_dt);
            if (AllParticleFlags.HasFlag(ParticleFlag.Wall)) SolveWall();
            int i = 0;
            if (Avx.IsSupported)
            {
                Vector256<float> dtv = Vector256.Create(dt);
                fixed (Vector2* pB = PositionBuffer) fixed (Vector2* vB = VelocityBuffer) for (; i + 4 <= Count; i += 4)
                        Avx.Store((float*)(pB + i), Avx.Add(Avx.LoadVector256((float*)(pB + i)), Avx.Multiply(Avx.LoadVector256((float*)(vB + i)), dtv)));
            }
            for (; i < Count; i++) PositionBuffer[i] += dt * VelocityBuffer[i];
        }
        locked = false;
    }
    unsafe void SolveCollision(float dt, float inv_dt)
    {
        AABB aabb = new(new(float.MaxValue, float.MaxValue), new(float.MinValue, float.MinValue));
        int i = 0;
        if (Avx.IsSupported)
        {
            Vector256<float> lowerBound = Vector256.Create(float.MaxValue), upperBound = Vector256.Create(float.MinValue), stepDt = Vector256.Create(dt);
            fixed (Vector2* pB = PositionBuffer) fixed (Vector2* vB = VelocityBuffer) for (; i + 4 <= Count; i += 4)
                {
                    Vector256<float> v = Avx.LoadVector256((float*)(vB + i)), p1 = Avx.LoadVector256((float*)(pB + i)), p2 = Avx.Add(p1, Avx.Multiply(stepDt, v));
                    lowerBound = Avx.Min(lowerBound, Avx.Min(p1, p2));
                    upperBound = Avx.Max(upperBound, Avx.Max(p1, p2));
                }
            aabb.lowerBound = Vector2.Min(Vector2.Min(new(lowerBound.GetElement(0), lowerBound.GetElement(1)), new(lowerBound.GetElement(2), lowerBound.GetElement(3))),
                Vector2.Min(new(lowerBound.GetElement(4), lowerBound.GetElement(5)), new(lowerBound.GetElement(6), lowerBound.GetElement(7))));
            aabb.upperBound = Vector2.Max(Vector2.Max(new(upperBound.GetElement(0), upperBound.GetElement(1)), new(upperBound.GetElement(2), upperBound.GetElement(3))),
                Vector2.Max(new(upperBound.GetElement(4), upperBound.GetElement(5)), new(upperBound.GetElement(6), upperBound.GetElement(7))));
        }
        for (; i < Count; i++)
        {
            Vector2 v = VelocityBuffer[i], p1 = PositionBuffer[i], p2 = p1 + dt * v;
            aabb.lowerBound = Vector2.Min(aabb.lowerBound, Vector2.Min(p1, p2));
            aabb.upperBound = Vector2.Max(aabb.upperBound, Vector2.Max(p1, p2));
        }
        ParticleContactFilter contactFilter = GetFixtureContactFilter();
        FixtureParticleQueryCallback callback = new(this)
        {
            QueryReportFixtureAndParticle = (f, i, a) =>
            {
                if (contactFilter == null || !GetFlagsBuffer()[a].HasFlag(ParticleFlag.FixtureContactFilter) || contactFilter.ShouldCollideFP(f, this, a))
                {
                    Vector2 av = VelocityBuffer[a]; CastOutput output = new(); RayCastInput input = new();
                    BodySim sim = World.GetBodySim(World.bodies[f.bodyId]);
                    if (IterationIndex == 0)
                    {
                        Vector2 p1 = new Transform(sim.center0, sim.rotation0).InvTransformPoint(PositionBuffer[a]);
                        if (f.shape is Circle)
                        {
                            p1 -= sim.localCenter;
                            p1 = sim.rotation0 * p1;
                            p1 = sim.transform.q.InvRotateVector(p1);
                            p1 += sim.localCenter;
                        }
                        input.origin = sim.transform.TransformPoint(p1);
                    }
                    else input.origin = PositionBuffer[a];
                    input.translation = PositionBuffer[a] + dt * av - input.origin;
                    input.maxFraction = 1;
                    if ((output = f.RayCast(ref input, sim.transform)).hit)
                    {
                        Vector2 v = inv_dt * (input.origin + output.fraction * input.translation + output.normal * Box2D.ParticleLinearSlop - PositionBuffer[a]);
                        VelocityBuffer[a] = v;
                        //ParticleApplyBufferLinearImpulse(a, GetParticleMass() * (av - v));
                        ParticleApplyForce(a, inv_dt * GetParticleMass() * (av - v));
                    }
                }
            }
        };
        WorldAPI.World_OverlapAABB(new() { generation = World.generation, index1 = World }, aabb, new(), (s, _) => callback.ReportFixture(s.world0, s.world0.GetShape(s)), null, callback);
    }
    unsafe void LimitVelocity(float dt, float inv_dt)
    {
        float criticalVelocitySquared = Box2D.MaxParticleVelocity == -1 ? GetCriticalVelocitySquared(dt, inv_dt)
            : Box2D.MaxParticleVelocity * Box2D.MaxParticleVelocity;
        int i = 0;
        if (Avx2.IsSupported)
        {
            Vector256<float> criticalV = Vector256.Create(criticalVelocitySquared);
            fixed (Vector2* vB = VelocityBuffer) for (; i + 8 <= Count; i += 8)
                {
                    Vector256<float> pos1 = Avx.LoadVector256((float*)(vB + i)), pos2 = Avx.LoadVector256((float*)(vB + i + 4));
                    ExtractXY(pos1, pos2, out var x, out var y);
                    SpreadTwice(Avx.Add(Avx.Multiply(x, x), Avx.Multiply(y, y)), out var v2_1, out var v2_2);
                    Avx.Store((float*)(vB + i), Avx.BlendVariable(pos1, Avx.Multiply(pos1, Avx.Sqrt(Avx.Divide(criticalV, v2_1))), Avx.CompareGreaterThan(v2_1, criticalV)));
                    Avx.Store((float*)(vB + i + 4), Avx.BlendVariable(pos2, Avx.Multiply(pos2, Avx.Sqrt(Avx.Divide(criticalV, v2_2))), Avx.CompareGreaterThan(v2_2, criticalV)));
                }
        }
        for (; i < Count; i++)
        {
            ref Vector2 v = ref VelocityBuffer[i]; float v2 = Vector2.Dot(v, v);
            if (v2 > criticalVelocitySquared) v *= MathF.Sqrt(criticalVelocitySquared / v2);
        }
    }
    unsafe void SolveGravity(float dt, float inv_dt)
    {
        Vector2 gravity = dt * GravityScale * World.gravity;
        int i = 0;
        if (Avx.IsSupported)
        {
            Vector256<float> g = Vector256.Create(gravity.x, gravity.y, gravity.x, gravity.y, gravity.x, gravity.y, gravity.x, gravity.y);
            fixed (Vector2* vB = VelocityBuffer) for (; i + 4 <= Count; i += 4)
                    Avx.Store((float*)(vB + i), Avx.Add(Avx.LoadVector256((float*)(vB + i)), g));
        }
        for (; i < Count; i++) VelocityBuffer[i] += gravity;
    }
    unsafe void SolveBarrier(float dt, float inv_dt)
    {
        int i = 0;
        if (Avx2.IsSupported)
        {
            Vector256<int> bw = Vector256.Create((int)(ParticleFlag.Barrier | ParticleFlag.Wall));
            fixed (ParticleFlag* fB = FlagsBuffer) fixed (Vector2* vB = VelocityBuffer) for (; i + 8 <= Count; i += 8)
                {
                    Vector256<float> v1 = Avx.LoadVector256((float*)(vB + i)), v2 = Avx.LoadVector256((float*)(vB + i + 4));
                    SpreadTwice(Avx.LoadVector256((int*)fB + i).AsSingle(), out var flags1, out var flags2);
                    Avx.Store((float*)(vB + i), Avx.BlendVariable(v1, Vector256<float>.Zero, SIMDHasFlag(flags1.AsInt32(), bw)));
                    Avx.Store((float*)(vB + i + 4), Avx.BlendVariable(v2, Vector256<float>.Zero, SIMDHasFlag(flags2.AsInt32(), bw)));
                }
        }
        for (; i < Count; i++)
        {
            if ((FlagsBuffer[i] & (ParticleFlag.Barrier | ParticleFlag.Wall)) == (ParticleFlag.Barrier | ParticleFlag.Wall))
                VelocityBuffer[i] = Vector2.Zero;
        }
        float tmax = Box2D.BarrierCollisionTime * dt;
        ParticleParallel(PairBuffer.Count, (start, end, p) =>
        {
            for (int k = start; k < end; k++)
            {
                if (PairBuffer[k].flags.HasFlag(ParticleFlag.Barrier))
                {
                    int a = PairBuffer[k].indexA, b = PairBuffer[k].indexB;
                    Vector2 pa = PositionBuffer[a], pb = PositionBuffer[b];
                    AABB aabb = new(Vector2.Min(pa, pb), Vector2.Max(pa, pb));
                    ParticleGroup aGroup = GroupBuffer[a], bGroup = GroupBuffer[b];
                    Vector2 va = GetLinearVelocity(aGroup, a, pa), vb = GetLinearVelocity(bGroup, b, pb);
                    Vector2 pba = pb - pa, vba = vb - va;
                    InsideBoundsEnumerator enumerator = GetInsideBoundsEnumenator(aabb);
                    int c;
                    while ((c = enumerator.GetNext()) >= 0)
                    {
                        Vector2 pc = PositionBuffer[c]; ParticleGroup cGroup = GroupBuffer[c];
                        if (aGroup != cGroup && bGroup != cGroup)
                        {
                            Vector2 vc = GetLinearVelocity(cGroup, c, pc);
                            Vector2 pca = pc - pa, vca = vc - va;
                            float e2 = Vector2.Cross(vba, vca), e1 = Vector2.Cross(pba, vca) - Vector2.Cross(pca, vba), e0 = Vector2.Cross(pba, pca);
                            float s, t; Vector2 qba, qca;
                            if (e2 == 0)
                            {
                                if (e1 == 0) continue; t = -e0 / e1;
                                if (!(t >= 0 && t < tmax)) continue;
                                qba = pba + t * vca; qca = pca + t * vca;
                                s = Vector2.Dot(qba, qca) / Vector2.Dot(qba, qba);
                                if (!(s >= 0 && s <= 1)) continue;
                            }
                            else
                            {
                                float det = e1 * e1 - 4 * e0 * e2; if (det < 0) continue;
                                float sqrtDet = MathF.Sqrt(det), t1 = (-e1 - sqrtDet) * (2 * e2), t2 = (-e1 + sqrtDet) / (2 * e2);
                                if (t1 > t2) (t1, t2) = (t2, t1); t = t1;
                                qba = pba + t * vba; qca = pca + t * vca;
                                s = Vector2.Dot(qba, qca) / Vector2.Dot(qba, qba);
                                if (!(t >= 0 && t < tmax && s >= 0 && s <= 1))
                                {
                                    t = t2; if (!(t >= 0 && t < tmax)) continue;
                                    qba = pba + t * vba; qca = pca + t * vca;
                                    s = Vector2.Dot(qba, qca) / Vector2.Dot(qba, qba);
                                    if (!(s >= 0 && s <= 1)) continue;
                                }
                            }
                            Vector2 dv = va + s * vba - vc, f = GetParticleMass() * dv;
                            if (IsRigidGroup(cGroup))
                            {
                                cGroup.UpdateStatistics();
                                cGroup.m_linearVelocity += cGroup.m_invMass * f;
                                cGroup.m_angularVelocity += cGroup.m_invInertia * Vector2.Cross(pc - cGroup.GetCenter, f);
                            }
                            else VelocityBuffer[c] += dv;
                            ParticleApplyForce(c, -inv_dt * f);
                            //ParticleApplyBufferLinearImpulse(c, -f);
                        }
                    }
                }
            }
        }, PairBuffer);
    }
    unsafe void SolveStaticPressure(float dt, float inv_dt)
    {
        StaticPressureBuffer = RequestBuffer(ref StaticPressureBuffer);
        float crticialPressure = GetCriticalPressure(dt, inv_dt),
            pressurePerWeight = StaticPressureStrength * crticialPressure,
            maxPressure = Box2D.MaxParticlePressure * crticialPressure,
            relaxation = StaticPressureRelaxation;
        for (int t = 0; t < StaticPressureIterations; t++)
        {
            fixed (float* a = AccumulationBuffer) NativeMemory.Clear(a, (nuint)(4 * Count));
            ParticleParallel(ContactBuffer.Count, (start, end, a) =>
            {
                for (int k = start; k < end; k++)
                {
                    ParticleContact contact = ContactBuffer[k];
                    if (contact.Flags.HasFlag(ParticleFlag.StaticPressure))
                    {
                        System.Threading.Interlocked.Exchange(ref a[contact.IndexA], a[contact.IndexA] + contact.Weight * StaticPressureBuffer[contact.IndexB]);
                        System.Threading.Interlocked.Exchange(ref a[contact.IndexB], a[contact.IndexB] + contact.Weight * StaticPressureBuffer[contact.IndexA]);
                    }
                }
            }, AccumulationBuffer);
            int i = 0;
            if (Avx2.IsSupported)
            {
                Vector256<float> ppW = Vector256.Create(pressurePerWeight), rel = Vector256.Create(relaxation), maxP = Vector256.Create(maxPressure),
                    minW = Vector256.Create(Box2D.MinParticleWeight);
                Vector256<int> sP = Vector256.Create((int)ParticleFlag.StaticPressure);
                fixed (float* wB = WeightBuffer) fixed (float* spB = StaticPressureBuffer)
                fixed (ParticleFlag* fB = FlagsBuffer) fixed (float* aB = AccumulationBuffer) for (; i + 8 <= Count; i += 8)
                    {
                        Vector256<float> w = Avx.LoadVector256(wB + i);
                        Avx.Store(spB + i, Avx.BlendVariable(Avx.Min(Avx.Max(Avx.Add(Avx.LoadVector256(aB + i),
                            Avx.Divide(Avx.Multiply(ppW, Avx.Subtract(w, minW)), Avx.Add(w, rel))), Vector256<float>.Zero), maxP), Vector256<float>.Zero,
                            SIMDHasFlag(Avx.LoadVector256((int*)fB + i), sP)));
                    }
            }
            for (; i < Count; i++)
            {
                float w = WeightBuffer[i];
                StaticPressureBuffer[i] = FlagsBuffer[i].HasFlag(ParticleFlag.StaticPressure) ?
                    Math.Clamp((AccumulationBuffer[i] + pressurePerWeight * (w - Box2D.MinParticleWeight)) / (w + relaxation), 0, maxPressure) : 0;
            }
        }
    }
    void ComputeWeight()
    {
        WeightBuffer = new float[Count];
        ParticleParallel(BodyContactBuffer.Count, (start, end, wb) =>
        {
            for (int k = start; k < end; k++)
            {
                ParticleBodyContact contact = BodyContactBuffer[k];
                WeightBuffer[contact.index] += contact.weight;
            }
        }, WeightBuffer);
        ParticleParallel(ContactBuffer.Count, (start, end, wb) =>
        {
            for (int k = start; k < end; k++)
            {
                ParticleContact contact = ContactBuffer[k]; float w = contact.Weight;
                WeightBuffer[contact.IndexB] += w; WeightBuffer[contact.IndexA] += w;
            }
        }, WeightBuffer);
    }
    unsafe void SolvePressure(float dt, float inv_dt)
    {
        float criticalPressure = GetCriticalPressure(dt, inv_dt),
            pressurePerWeight = PressureStrength * criticalPressure,
            maxPressure = Box2D.MaxParticlePressure * criticalPressure;
        int i = 0;
        if (Avx.IsSupported)
        {
            Vector256<float> ppW = Vector256.Create(pressurePerWeight), minW = Vector256.Create(Box2D.MinParticleWeight),
                maxP = Vector256.Create(maxPressure), zero = Vector256<float>.Zero;
            fixed (float* wB = WeightBuffer) fixed (float* aB = AccumulationBuffer) for (; i + 8 <= Count; i += 8)
                    Avx.Store(aB + i, Avx.Min(Avx.Multiply(ppW, Avx.Max(zero, Avx.Subtract(Avx.LoadVector256(wB + i), minW))), maxP));
        }
        for (; i < Count; i++)
        {
            float w = WeightBuffer[i], h = pressurePerWeight * Math.Max(0, w - Box2D.MinParticleWeight);
            AccumulationBuffer[i] = Math.Min(h, maxPressure);
        }
        if ((AllParticleFlags & noPressureFlags) != 0)
        {
            i = 0;
            if (Avx2.IsSupported)
            {
                Vector256<int> npf = Vector256.Create((int)noPressureFlags); Vector256<float> zero = Vector256<float>.Zero;
                fixed (ParticleFlag* fB = FlagsBuffer) fixed (float* aB = AccumulationBuffer) for (; i + 8 <= Count; i += 8)
                        Avx.Store(aB + i, Avx.BlendVariable(Avx.LoadVector256(aB + i), zero, SIMDHasFlag(Avx.LoadVector256((int*)fB + i), npf)));
            }
            for (; i < Count; i++)
                if ((FlagsBuffer[i] & noPressureFlags) != 0) AccumulationBuffer[i] = 0;
        }
        if (AllParticleFlags.HasFlag(ParticleFlag.StaticPressure))
        {
            Debug.Assert(StaticPressureBuffer != null);
            i = 0;
            if (Avx2.IsSupported)
            {
                Vector256<int> sp = Vector256.Create((int)ParticleFlag.StaticPressure);
                fixed (ParticleFlag* fB = FlagsBuffer) fixed (float* spB = StaticPressureBuffer) fixed (float* aB = AccumulationBuffer) for (; i + 8 <= Count; i += 8)
                    {
                        Vector256<float> a = Avx.LoadVector256(aB + i);
                        Avx.Store(aB + i, Avx.BlendVariable(a, Avx.Add(a, Avx.LoadVector256(spB + i)), SIMDHasFlag(Avx.LoadVector256((int*)fB + i), sp)));
                    }
            }
            for (; i < Count; i++) if (FlagsBuffer[i].HasFlag(ParticleFlag.StaticPressure))
                    AccumulationBuffer[i] += StaticPressureBuffer[i];
        }
        float invMass = GetParticleInvMass();
        float velocityPerPressure = dt * GetInverseDensity() * GetInverseDiameter();
        int count = BodyContactBuffer.Count;
        //ParticleParallel(count, (start, end, c) =>
        //{
            for (int k = 0; k < count; k++)
            {
                ParticleBodyContact contact = BodyContactBuffer[k];
                Vector2 f = velocityPerPressure * contact.weight * contact.mass * (AccumulationBuffer[contact.index] + pressurePerWeight * contact.weight) * contact.normal;
                VelocityBuffer[contact.index] -= invMass * f;
                contact.body.ApplyLinearImpulse(World, f, PositionBuffer[contact.index]);
            }
        //}, BodyContactBuffer);
        count = ContactBuffer.Count;
        ParticleParallel(count, (start, end, v) =>
        {
            for (int k = start; k < end; k++)
            {
                ParticleContact contact = ContactBuffer[k];
                Vector2 f = velocityPerPressure * contact.Weight * (AccumulationBuffer[contact.IndexA] + AccumulationBuffer[contact.IndexB]) * contact.Normal;
                System.Threading.Interlocked.Exchange(ref VelocityBuffer[contact.IndexA].x, VelocityBuffer[contact.IndexA].x - f.x);
                System.Threading.Interlocked.Exchange(ref VelocityBuffer[contact.IndexA].y, VelocityBuffer[contact.IndexA].y - f.y);
                System.Threading.Interlocked.Exchange(ref VelocityBuffer[contact.IndexB].x, VelocityBuffer[contact.IndexB].x + f.x);
                System.Threading.Interlocked.Exchange(ref VelocityBuffer[contact.IndexB].y, VelocityBuffer[contact.IndexB].y + f.y);
            }
        }, VelocityBuffer);
    }
    void SolveDamping(float dt, float inv_dt)
    {
        float invMass = GetParticleInvMass();
        float linearDamping = DampingStrength, quadraticDamping = 1 / GetCriticalVelocity(dt, inv_dt);
        int count = BodyContactBuffer.Count;
        ParticleParallel(count, (start, end, vb) =>
        {
            for (int k = start; k < end; k++)
            {
                ParticleBodyContact contact = BodyContactBuffer[k];
                Vector2 v = World.GetLinearVelocityFromWorldPoint(contact.body, PositionBuffer[contact.index]) - VelocityBuffer[contact.index];
                float vn = Vector2.Dot(v, contact.normal);
                if (vn < 0)
                {
                    float damping = Math.Max(linearDamping * contact.weight, Math.Min(-quadraticDamping * vn, 0.5f));
                    Vector2 f = damping * contact.mass * vn * contact.normal;
                    VelocityBuffer[contact.index] += invMass * f;
                    contact.body.ApplyLinearImpulse(World, -f, PositionBuffer[contact.index]);
                }
            }
        }, VelocityBuffer);
        count = ContactBuffer.Count;
        ParticleParallel(count, (start, end, v) =>
        {
            for (int k = start; k < end; k++)
            {
                ParticleContact contact = ContactBuffer[k];
                float vn = Vector2.Dot(VelocityBuffer[contact.IndexB] - VelocityBuffer[contact.IndexA], contact.Normal);
                if (vn < 0)
                {
                    Vector2 f = Math.Max(linearDamping * contact.Weight, Math.Min(-quadraticDamping * vn, 0.5f)) * vn * contact.Normal;
                    VelocityBuffer[contact.IndexA] += f; VelocityBuffer[contact.IndexB] -= f;
                }
            }
        }, VelocityBuffer);
    }
    void SolveRigidDamping()
    {
        float particleInvMass = GetParticleInvMass(),
            damping = DampingStrength;
        int count = BodyContactBuffer.Count;
        ParticleParallel(count, (start, end, cb) =>
        {
            for (int k = start; k < end; k++)
            {
                ParticleBodyContact contact = BodyContactBuffer[k];
                ParticleGroup aGroup = GroupBuffer[contact.index];
                if (IsRigidGroup(aGroup))
                {
                    Vector2 p = PositionBuffer[contact.index];
                    float vn = Vector2.Dot(World.GetLinearVelocityFromWorldPoint(contact.body, p)
                        - aGroup.GetLinearVelocityFromWorldPoint(p), contact.normal);
                    if (vn < 0)
                    {
                        aGroup.UpdateStatistics();
                        BodySim sim = World.GetBodySim(contact.body);
                        float invMassA = aGroup.m_invMass,
                            invInertiaA = aGroup.m_invInertia,
                            tangentDistanceA = Vector2.Cross(p - aGroup.GetCenter, contact.normal),
                            invMassB = sim.invMass,
                            invInertiaB = sim.invInertia,
                            tangentDistanceB = Vector2.Cross(p - World.GetBodySim(contact.body).center, contact.normal);
                        float f = damping * Math.Min(contact.weight, 1) * ComputeDampingImpulse(
                            invMassA, invInertiaA, tangentDistanceA, invMassB, invInertiaB, tangentDistanceB, vn);
                        ApplyDamping(invMassA, invInertiaA, tangentDistanceA, true, aGroup, contact.index, f, contact.normal);
                        contact.body.ApplyLinearImpulse(World, -f * contact.normal, p);
                    }
                }
            }
        }, BodyContactBuffer);
        count = ContactBuffer.Count;
        ParticleParallel(count, (start, end, cb) =>
        {
            for (int k = start; k < end; k++)
            {
                ParticleContact contact = ContactBuffer[k];
                int a = contact.IndexA, b = contact.IndexB;
                ParticleGroup aGroup = GroupBuffer[a], bGroup = GroupBuffer[b];
                bool aRigid = IsRigidGroup(aGroup), bRigid = IsRigidGroup(bGroup);
                if (aGroup != bGroup && (aRigid || bRigid))
                {
                    Vector2 p = 0.5f * (PositionBuffer[a] + PositionBuffer[b]),
                        v = GetLinearVelocity(bGroup, b, p) - GetLinearVelocity(aGroup, a, p);
                    float vn = Vector2.Dot(v, contact.Normal);
                    if (vn < 0)
                    {
                        InitDampingParameterWithRigidGroupOrParticle(out float invMassA,
                            out float invInertiaA, out float tangentDistanceA, aRigid, aGroup, a, p, contact.Normal, particleInvMass);
                        InitDampingParameterWithRigidGroupOrParticle(out float invMassB,
                            out float invInertiaB, out float tangentDistanceB, bRigid, bGroup, b, p, contact.Normal, particleInvMass);
                        float f = damping * contact.Weight * ComputeDampingImpulse(invMassA, invInertiaA, tangentDistanceA, invMassB, invInertiaB, tangentDistanceB, vn);
                        ApplyDamping(invMassA, invInertiaA, tangentDistanceA, aRigid, aGroup, a, f, contact.Normal);
                        ApplyDamping(invMassB, invInertiaB, tangentDistanceB, bRigid, bGroup, b, -f, contact.Normal);
                    }
                }
            }
        }, ContactBuffer);
    }
    void SolveExtraDamping()
    {
        float invMass = GetParticleInvMass();
        ParticleParallel(BodyContactBuffer.Count, (start, end, vb) =>
        {
            for (int k = start; k < end; k++)
            {
                ParticleBodyContact contact = BodyContactBuffer[k];
                if ((FlagsBuffer[contact.index] & extraDampingFlags) != 0)
                {
                    Vector2 v = World.GetLinearVelocityFromWorldPoint(contact.body, PositionBuffer[contact.index]) - VelocityBuffer[contact.index];
                    float vn = Vector2.Dot(v, contact.normal);
                    if (vn < 0)
                    {
                        Vector2 f = 0.5f * contact.mass * vn * contact.normal;
                        VelocityBuffer[contact.index] += invMass * f;
                        contact.body.ApplyLinearImpulse(World, -f, PositionBuffer[contact.index]);
                    }
                }
            }
        }, VelocityBuffer);
    }
    unsafe void SolveWall()
    {
        int i = 0;
        if (Avx2.IsSupported)
        {
            Vector256<int> npf = Vector256.Create((int)ParticleFlag.Wall);
            fixed (ParticleFlag* fB = FlagsBuffer) fixed (Vector2* vB = VelocityBuffer) for (; i + 8 <= Count; i += 8)
                {
                    Vector256<float> v1 = Avx.LoadVector256((float*)(vB + i)), v2 = Avx.LoadVector256((float*)(vB + i + 4));
                    SpreadTwice(Avx.LoadVector256((int*)fB + i).AsSingle(), out var flags1, out var flags2);
                    Avx.Store((float*)(vB + i), Avx.BlendVariable(v1, Vector256<float>.Zero, SIMDHasFlag(flags1.AsInt32(), npf)));
                    Avx.Store((float*)(vB + i + 4), Avx.BlendVariable(v2, Vector256<float>.Zero, SIMDHasFlag(flags2.AsInt32(), npf)));
                }
        }
        for (; i < Count; i++) if (FlagsBuffer[i].HasFlag(ParticleFlag.Wall))
                VelocityBuffer[i] = Vector2.Zero;
    }
    void SolveRigid(float dt, float inv_dt)
    {
        ParticleParallel(ParticleGroupList.Count, (start, end, p) =>
        {
            for (int i = start; i < end; i++)
            {
                if (p[i].m_groupFlags.HasFlag(ParticleGroupFlag.Rigid))
                {
                    p[i].UpdateStatistics();
                    Rotation rotation = new(dt * p[i].m_angularVelocity);
                    Transform transform = new(p[i].m_center + dt * p[i].m_linearVelocity - rotation * p[i].m_center, rotation),
                        velocityTransform = new(transform.p * inv_dt, new(inv_dt * (transform.q.c - 1), inv_dt * transform.q.s));
                    p[i].Transform = transform * p[i].Transform;
                    ParticleParallel(p[i].LastIndex - p[i].BufferIndex, (start, end, v) =>
                    {
                        for (int j = p[i].BufferIndex + start; j < p[i].BufferIndex + end; j++)
                            v[j] = velocityTransform.TransformPoint(PositionBuffer[j]);
                    }, VelocityBuffer);
                }
            }
        }, ParticleGroupList);
    }
    void SolveElastic(float dt, float inv_dt)
    {
        float elasticStrength = inv_dt * ElasticStrength;
        ParticleParallel(TriadBuffer.Count, (start, end, b) =>
        {
            for (int k = start; k < end; k++)
            {
                ParticleTriad triad = TriadBuffer[k];
                if (triad.flags.HasFlag(ParticleFlag.Elastic))
                {
                    Vector2 pa = PositionBuffer[triad.indexA], pb = PositionBuffer[triad.indexB], pc = PositionBuffer[triad.indexC];
                    pa += dt * VelocityBuffer[triad.indexA];
                    pb += dt * VelocityBuffer[triad.indexB];
                    pc += dt * VelocityBuffer[triad.indexC];
                    Vector2 midPoint = 1f / 3 * (pa + pb + pc);
                    pa -= midPoint; pb -= midPoint; pc -= midPoint;
                    Rotation r = new(Vector2.Dot(triad.pa, pa) + Vector2.Dot(triad.pb, pb) + Vector2.Dot(triad.pc, pc),
                        Vector2.Cross(triad.pa, pa) + Vector2.Cross(triad.pb, pb) + Vector2.Cross(triad.pc, pc));
                    r = r.Normalize();
                    float strength = elasticStrength * triad.strength;
                    Vector2 fa = strength * (r * triad.pa - pa);
                    Vector2 fb = strength * (r * triad.pb - pb);
                    Vector2 fc = strength * (r * triad.pc - pc);
                    if (Box2D.ElasticPreserveVelocity)
                    {
                        Vector2 mid = 1f / 3 * (fa + fb + fc);
                        fa -= mid; fb -= mid; fc -= mid;
                    }
                    VelocityBuffer[triad.indexA] += fa;
                    VelocityBuffer[triad.indexB] += fb;
                    VelocityBuffer[triad.indexC] += fc;
                }
            }
        }, VelocityBuffer);
    }
    void SolveSpring(float dt, float inv_dt)
    {
        float springStrength = inv_dt * SpringStrength;
        ParticleParallel(PairBuffer.Count, (start, end, p) =>
        {
            for (int k = start; k < end; k++)
            {
                ParticlePair pair = p[k];
                if (pair.flags.HasFlag(ParticleFlag.Spring))
                {
                    ref Vector2 va = ref VelocityBuffer[pair.indexA], vb = ref VelocityBuffer[pair.indexB];
                    Vector2 d = PositionBuffer[pair.indexB] + dt * vb - PositionBuffer[pair.indexA] - dt * va;
                    float r0 = pair.distance, r1 = d.Length(), strength = springStrength * pair.strength;
                    Vector2 f = strength * (r0 - r1) / r1 * d; va -= f; vb += f;
                }
            }
        }, PairBuffer);
    }
    unsafe void SolveTensile(float dt, float inv_dt)
    {
        Debug.Assert(Accumulation2Buffer != null);
        fixed (Vector2* a = Accumulation2Buffer) NativeMemory.Clear(a, (nuint)(Count * 8));
        int count = ContactBuffer.Count;
        ParticleParallel(count, (start, end, a2b) =>
        {
            for (int k = start; k < end; k++)
            {
                ParticleContact contact = ContactBuffer[k];
                if (contact.Flags.HasFlag(ParticleFlag.Tensile))
                {
                    Vector2 weightedNormal = (1 - contact.Weight) * contact.Weight * contact.Normal;
                    a2b[contact.IndexA] -= weightedNormal;
                    a2b[contact.IndexB] += weightedNormal;
                }
            }
        }, Accumulation2Buffer);
        float criticalVelocity = GetCriticalVelocity(dt, inv_dt),
            pressureStrength = SurfaceTensionPressureStrength * criticalVelocity,
            normalStrength = SurfaceTensionNormalStrength * criticalVelocity,
            maxVelocityVariation = Box2D.MaxParticleForce * criticalVelocity;
        ParticleParallel(count, (start, end, vb) =>
        {
            for (int k = start; k < end; k++)
            {
                ParticleContact contact = ContactBuffer[k];
                if (contact.Flags.HasFlag(ParticleFlag.Tensile))
                {
                    Vector2 f = Math.Min(pressureStrength * (WeightBuffer[contact.IndexA] + WeightBuffer[contact.IndexB] - 2) +
                        normalStrength * Vector2.Dot(Accumulation2Buffer[contact.IndexB] - Accumulation2Buffer[contact.IndexA], contact.Normal), maxVelocityVariation)
                        * contact.Weight * contact.Normal;
                    System.Threading.Interlocked.Exchange(ref vb[contact.IndexA].x, vb[contact.IndexA].x - f.x);
                    System.Threading.Interlocked.Exchange(ref vb[contact.IndexA].y, vb[contact.IndexA].y - f.y);
                    System.Threading.Interlocked.Exchange(ref vb[contact.IndexB].x, vb[contact.IndexB].x + f.x);
                    System.Threading.Interlocked.Exchange(ref vb[contact.IndexB].y, vb[contact.IndexB].y + f.y);
                }
            }
        }, VelocityBuffer);
    }
    void SolveViscous()
    {
        float invMass = GetParticleInvMass();
        float viscousStrength = ViscousStrength;
        int count = BodyContactBuffer.Count;
        ParticleParallel(count, (start, end, vb) =>
        {
            for (int k = start; k < end; k++)
            {
                ParticleBodyContact contact = BodyContactBuffer[k];
                if (FlagsBuffer[contact.index].HasFlag(ParticleFlag.Viscous))
                {
                    Vector2 f = viscousStrength * contact.mass * contact.weight *
                        (World.GetLinearVelocityFromWorldPoint(contact.body, PositionBuffer[contact.index]) - vb[contact.index]);
                    vb[contact.index] += invMass * f;
                    contact.body.ApplyLinearImpulse(World, -f, PositionBuffer[contact.index]);
                }
            }
        }, VelocityBuffer);
        ParticleParallel(count, (start, end, vb) =>
        {
            for (int k = start; k < end; k++)
            {
                ParticleContact contact = ContactBuffer[k];
                if (contact.Flags.HasFlag(ParticleFlag.Viscous))
                {
                    Vector2 f = viscousStrength * contact.Weight * (vb[contact.IndexB] - vb[contact.IndexA]);
                    System.Threading.Interlocked.Exchange(ref vb[contact.IndexA].x, vb[contact.IndexA].x + f.x);
                    System.Threading.Interlocked.Exchange(ref vb[contact.IndexA].y, vb[contact.IndexA].y + f.y);
                    System.Threading.Interlocked.Exchange(ref vb[contact.IndexB].x, vb[contact.IndexB].x - f.x);
                    System.Threading.Interlocked.Exchange(ref vb[contact.IndexB].y, vb[contact.IndexB].y - f.y);
                }
            }
        }, VelocityBuffer);
    }
    void SolveRepulsive(float dt, float inv_dt)
    {
        float repulsiveStrength = RepulsiveStrength * GetCriticalVelocity(dt, inv_dt);
        ParticleParallel(ContactBuffer.Count, (start, end, vb) =>
        {
            for (int k = start; k < end; k++)
            {
                ParticleContact contact = ContactBuffer[k];
                if (contact.Flags.HasFlag(ParticleFlag.Repulsive) && GroupBuffer[contact.IndexA] != GroupBuffer[contact.IndexB])
                {
                    Vector2 f = repulsiveStrength * contact.Weight * contact.Normal;
                    vb[contact.IndexA] -= f; vb[contact.IndexB] += f;
                }
            }
        }, VelocityBuffer);
    }
    void SolvePowder(float dt, float inv_dt)
    {
        float powderStrength = PowderStrength * GetCriticalVelocity(dt, inv_dt);
        float minWeight = 1 - Box2D.ParticleStride;
        ParticleParallel(ContactBuffer.Count, (start, end, vb) =>
        {
            for (int k = start; k < end; k++)
            {
                ParticleContact contact = ContactBuffer[k];
                if (contact.Flags.HasFlag(ParticleFlag.Powder) && contact.Weight > minWeight)
                {
                    Vector2 f = powderStrength * (contact.Weight - minWeight) * contact.Normal;
                    vb[contact.IndexA] -= f; vb[contact.IndexB] += f;
                }
            }
        }, VelocityBuffer);
    }
    void SolveSolid(float dt, float inv_dt)
    {
        Debug.Assert(DepthBuffer != null);
        float ejectionStrength = inv_dt * EjectionStrength;
        ParticleParallel(ContactBuffer.Count, (start, end, vb) =>
        {
            for (int k = start; k < end; k++)
            {
                ParticleContact contact = ContactBuffer[k];
                if (GroupBuffer[contact.IndexA] != GroupBuffer[contact.IndexB])
                {
                    Vector2 f = ejectionStrength * (DepthBuffer[contact.IndexA] + DepthBuffer[contact.IndexB]) * contact.Weight * contact.Normal;
                    vb[contact.IndexA] -= f; vb[contact.IndexB] += f;
                }
            }
        }, VelocityBuffer);
    }
    unsafe void SolveForce(float dt, float inv_dt)
    {
        float velocityPerForce = dt * GetParticleInvMass();
        int i = 0;
        if (Avx.IsSupported)
        {
            Vector256<float> vpF = Vector256.Create(velocityPerForce);
            fixed (Vector2* vB = VelocityBuffer) fixed (Vector2* fB = ForceBuffer) for (; i + 4 <= Count; i += 4)
                    Avx.Store((float*)(vB + i), Avx.Add(Avx.LoadVector256((float*)(vB + i)), Avx.Multiply(vpF, Avx.LoadVector256((float*)(fB + i)))));
        }
        for (; i < Count; i++) VelocityBuffer[i] += velocityPerForce * ForceBuffer[i];
        HasForce = false;
    }
    unsafe void SolveImpulse(float dt, float inv_dt)
    {
        float velocityPerImpulse = GetParticleInvMass();
        int i = 0;
        if (Avx.IsSupported)
        {
            Vector256<float> vpI = Vector256.Create(velocityPerImpulse);
            fixed (Vector2* vB = VelocityBuffer) fixed (Vector2* iB = ImpulseBuffer) for (; i + 4 <= Count; i += 4)
                    Avx.Store((float*)(vB + i), Avx.Add(Avx.LoadVector256((float*)(vB + i)), Avx.Multiply(vpI, Avx.LoadVector256((float*)(iB + i)))));
        }
        for (; i < Count; i++) VelocityBuffer[i] += velocityPerImpulse * ImpulseBuffer[i];
        HasImpulse = false;
    }
    void SolveColorMixing()
    {
        Debug.Assert(ColorBuffer != null);
        int colorMixing128 = (int)(128 * ColorMixingStrength);
        if (colorMixing128 > 0)
            ParticleParallel(ContactBuffer.Count, (start, end, cb) =>
            {
                for (int k = start; k < end; k++)
                {
                    ParticleContact contact = cb[k];
                    if ((FlagsBuffer[contact.IndexA] & FlagsBuffer[contact.IndexB]).HasFlag(ParticleFlag.ColorMixing))
                        ParticleColor.MixColors(ref ColorBuffer[contact.IndexA], ref ColorBuffer[contact.IndexB], colorMixing128);
                }
            }, ContactBuffer);
    }
    void SolveZombie()
    {
        int newCount = 0;
        int[] newIndices = new int[Count];
        ParticleFlag allParticleFlags = 0;
        ParticleParallel(Count, (start, end, fb) =>
        {
            for (int i = start; i < end; i++)
            {
                ParticleFlag flags = fb[i];
                if (flags.HasFlag(ParticleFlag.Zombie))
                {
                    if (flags.HasFlag(ParticleFlag.DestructionListener)) World.ParticleRemoved.Invoke(this, i);
                    if (HandleIndexBuffer != null)
                    {
                        ParticleHandle handle = HandleIndexBuffer[i];
                        if (handle != null)
                        {
                            handle.Index = Box2D.InvalidParticleIndex;
                            HandleIndexBuffer[i] = null;
                        }
                    }
                    newIndices[i] = Box2D.InvalidParticleIndex;
                }
                else
                {
                    newIndices[i] = newCount;
                    if (i != newCount)
                    {
                        if (HandleIndexBuffer != null)
                        {
                            ParticleHandle handle = HandleIndexBuffer[i];
                            if (handle != null) handle.Index = newCount;
                            HandleIndexBuffer[newCount] = handle;
                        }
                        FlagsBuffer[newCount] = FlagsBuffer[i];
                        if (LastBodyContactStepBuffer != null) LastBodyContactStepBuffer[newCount] = LastBodyContactStepBuffer[i];
                        if (BodyContactCountBuffer != null) BodyContactCountBuffer[newCount] = BodyContactCountBuffer[i];
                        if (ConsecutiveContactStepsBuffer != null) ConsecutiveContactStepsBuffer[newCount] = ConsecutiveContactStepsBuffer[i];
                        PositionBuffer[newCount] = PositionBuffer[i];
                        VelocityBuffer[newCount] = VelocityBuffer[i];
                        GroupBuffer[newCount] = GroupBuffer[i];
                        if (HasForce) ForceBuffer[newCount] = ForceBuffer[i];
                        if (HasImpulse) ImpulseBuffer[newCount] = ImpulseBuffer[i];
                        if (StaticPressureBuffer != null) StaticPressureBuffer[newCount] = StaticPressureBuffer[i];
                        if (DepthBuffer != null) DepthBuffer[newCount] = DepthBuffer[i];
                        if (ColorBuffer != null) ColorBuffer[newCount] = ColorBuffer[i];
                        if (UserDataBuffer != null) UserDataBuffer[newCount] = UserDataBuffer[i];
                        if (ExpirationTimeBuffer != null) ExpirationTimeBuffer[newCount] = ExpirationTimeBuffer[i];
                    }
                    newCount++;
                    allParticleFlags |= flags;
                }
            }
        }, FlagsBuffer);
        ParticleParallel(ProxyBuffer.Count, (start, end, pb) => { for (int k = start; k < end; k++) pb[k] = pb[k] with { index = newIndices[pb[k].index] }; }, ProxyBuffer);
        ProxyBuffer.RemoveAll(x => x.index < 0);
        //ProxyBuffer = ProxyBuffer.AsParallel().Select(x => x with { index = newIndices[x.index] }).Where(x => x.index >= 0).ToList();
        var cb = CollectionsMarshal.AsSpan(ContactBuffer);
        for (int k = 0; k < ContactBuffer.Count; k++) cb[k].SetIndices(newIndices[cb[k].IndexA], newIndices[cb[k].IndexB]);
        ContactBuffer.RemoveAll(x => x.IndexA < 0 || x.IndexB < 0);
        ParticleParallel(BodyContactBuffer.Count, (start, end, bcb) =>
        { for (int k = start; k < end; k++) bcb[k] = bcb[k] with { index = newIndices[bcb[k].index] }; }, BodyContactBuffer);
        BodyContactBuffer.RemoveAll(x => x.index < 0);
        //BodyContactBuffer = BodyContactBuffer.AsParallel().Select(x => x with { index = newIndices[x.index] }).Where(x => x.index >= 0).ToList();
        ParticleParallel(PairBuffer.Count, (start, end, pb) =>
        { for (int k = start; k < end; k++) { pb[k] = pb[k] with { indexA = newIndices[pb[k].indexA], indexB = newIndices[pb[k].indexB] }; } }, PairBuffer);
        PairBuffer.RemoveAll(x => x.indexA < 0 || x.indexB < 0);
        //PairBuffer = PairBuffer.AsParallel().Select(x => x with { indexA = newIndices[x.indexA], indexB = newIndices[x.indexB] }).Where(x => x.indexA >= 0 && x.indexB >= 0).ToList();
        ParticleParallel(TriadBuffer.Count, (start, end, tb) =>
        {
            for (int k = start; k < end; k++)
            { tb[k] = tb[k] with { indexA = newIndices[tb[k].indexA], indexB = newIndices[tb[k].indexB], indexC = newIndices[tb[k].indexC] }; }
        }, TriadBuffer); 
        TriadBuffer.RemoveAll(x => x.indexA < 0 || x.indexB < 0 || x.indexC < 0);
        if (IndexByExpirationTimeBuffer != null)
        {
            int writeOffset = 0;
            for (int readOffset = 0; readOffset < Count; readOffset++)
            {
                int newIndex = newIndices[IndexByExpirationTimeBuffer[readOffset]];
                if (newIndex != Box2D.InvalidParticleIndex) IndexByExpirationTimeBuffer[writeOffset++] = newIndex;
            }
        }
        ParticleParallel(ParticleGroupList.Count, (start, end, p) =>
        {
            for (int g = start; g < end; g++)
            {
                int firstIndex = newCount, lastIndex = 0; bool modified = false;
                ParticleGroup group = p[g];
                for (int i = group.BufferIndex; i < group.LastIndex; i++)
                {
                    int j = newIndices[i]; if (j >= 0)
                    {
                        firstIndex = Math.Min(firstIndex, j);
                        lastIndex = Math.Max(lastIndex, j + 1);
                    }
                    else modified = true;
                }
                if (firstIndex < lastIndex)
                {
                    group.BufferIndex = firstIndex;
                    group.LastIndex = lastIndex;
                    if (modified)
                    {
                        if (group.m_groupFlags.HasFlag(ParticleGroupFlag.Solid))
                            SetGroupFlags(group, group.m_groupFlags | ParticleGroupFlag.NeedsUpdateDepth);
                    }
                }
                else
                {
                    group.BufferIndex = 0; group.LastIndex = 0;
                    if (!group.m_groupFlags.HasFlag(ParticleGroupFlag.CanBeEmpty))
                        SetGroupFlags(group, group.m_groupFlags | ParticleGroupFlag.WillBeDestroyed);
                }
            }
        }, ParticleGroupList);
        Count = newCount;
        AllParticleFlags = allParticleFlags;
        NeedsUpdateAllParticleFlags = false;
        for (int i = 0; i < ParticleGroupList.Count; i++)
            if (ParticleGroupList[i].m_groupFlags.HasFlag(ParticleGroupFlag.WillBeDestroyed))
                DestroyParticleGroup(ParticleGroupList[i--]);
    }
    void SolveLifetimes(float dt, float inv_dt)
    {
        Debug.Assert(ExpirationTimeBuffer != null);
        Debug.Assert(IndexByExpirationTimeBuffer != null);
        TimeElapsed = LifetimeToExpirationTime(dt);
        int quantizedTimeElpased = GetQuantizedtimeElapsed();
        int particleCount = Count;
        if (ExpirationTimeBufferRequiresSorting)
        {
            ExpirationTimeComparator expirationTimeComparator = new(ExpirationTimeBuffer);
            HPCsharp.ParallelAlgorithm.SortMergeInPlacePar(IndexByExpirationTimeBuffer, Comparer<int>.Create(expirationTimeComparator.Compare));
        }
        for (int i = particleCount; i >= 0; i--)
        {
            int particleIndex = IndexByExpirationTimeBuffer[i];
            if (quantizedTimeElpased < ExpirationTimeBuffer[particleIndex] || ExpirationTimeBuffer[particleIndex] <= 0) break;
            DestroyParticle(particleIndex);
        }
    }
    struct NewIndices
    {
        public int start, mid, end;
        public int this[int i]
        {
            get
            {
                if (i < start) return i;
                else if (i < mid) return i + end - mid;
                else if (i < end) return i + start - mid;
                else return i;
            }
        }
    }
    static void Rotate<T>(T[] array, int first, int middle, int last) //std::rotate
    {
        if (first == middle) return;
        if (middle == last) return;
        int write = first;
        int next_read = first; // read position for when "read" hits "last"
        for (int read = middle; read != last; ++write, ++read)
        {
            if (write == next_read) next_read = read; // track where "first" went
            (array[write], array[read]) = (array[read], array[write]);
        }
        Rotate(array, write, next_read, last); // rotate the remaining sequence into place
    }
    void RotateBuffer(int start, int mid, int end)
    {
        if (start == mid || mid == end) return;
        Debug.Assert(mid >= start && mid <= end);
        NewIndices newIndices = new() { start = start, mid = mid, end = end };
        Rotate(FlagsBuffer, start, mid, end);
        if (LastBodyContactStepBuffer != null) Rotate(LastBodyContactStepBuffer, start, mid, end);
        if (BodyContactCountBuffer != null) Rotate(BodyContactCountBuffer, start, mid, end);
        if (ConsecutiveContactStepsBuffer != null) Rotate(ConsecutiveContactStepsBuffer, start, mid, end);
        Rotate(PositionBuffer, start, mid, end);
        Rotate(VelocityBuffer, start, mid, end);
        Rotate(GroupBuffer, start, mid, end);
        if (HasForce) Rotate(ForceBuffer, start, mid, end);
        if (HasImpulse) Rotate(ImpulseBuffer, start, mid, end);
        if (StaticPressureBuffer != null) Rotate(StaticPressureBuffer, start, mid, end);
        if (DepthBuffer != null) Rotate(DepthBuffer, start, mid, end);
        if (ColorBuffer != null) Rotate(ColorBuffer, start, mid, end);
        if (UserDataBuffer != null) Rotate(UserDataBuffer, start, mid, end);
        if (HandleIndexBuffer != null)
        {
            Rotate(HandleIndexBuffer, start, mid, end);
            for (int i = start; i < end; i++)
                HandleIndexBuffer[i].Index = newIndices[HandleIndexBuffer[i].Index];
        }
        if (ExpirationTimeBuffer != null)
        {
            Rotate(ExpirationTimeBuffer, start, mid, end);
            int particleCount = Count;
            for (int i = 0; i < particleCount; i++)
                IndexByExpirationTimeBuffer[i] = newIndices[IndexByExpirationTimeBuffer[i]];
        }
        var pb = CollectionsMarshal.AsSpan(ProxyBuffer);
        for (int k = 0; k < ProxyBuffer.Count; k++) pb[k].index = newIndices[pb[k].index];
        ProxyBuffer.RemoveAll(x => x.index < 0);
        //ProxyBuffer = ProxyBuffer.AsParallel().Select(x => x with { index = newIndices[x.index] }).Where(x => x.index >= 0).ToList();
        var cb = CollectionsMarshal.AsSpan(ContactBuffer);
        for (int k = 0; k < ContactBuffer.Count; k++) cb[k].SetIndices(newIndices[cb[k].IndexA], newIndices[cb[k].IndexB]);
        ContactBuffer.RemoveAll(x => x.IndexA < 0 || x.IndexB < 0);
        var bcb = CollectionsMarshal.AsSpan(BodyContactBuffer);
        for (int k = 0; k < BodyContactBuffer.Count; k++) bcb[k].index = newIndices[bcb[k].index];
        BodyContactBuffer.RemoveAll(x => x.index < 0);
        //BodyContactBuffer = BodyContactBuffer.AsParallel().Select(x => x with { index = newIndices[x.index] }).Where(x => x.index >= 0).ToList();
        var pab = CollectionsMarshal.AsSpan(PairBuffer);
        for (int k = 0; k < PairBuffer.Count; k++)
        { pab[k].indexA = newIndices[pab[k].indexA]; pab[k].indexB = newIndices[pab[k].indexB]; }
        PairBuffer.RemoveAll(x => x.indexA < 0 || x.indexB < 0);
        //PairBuffer = PairBuffer.AsParallel().Select(x => x with { indexA = newIndices[x.indexA], indexB = newIndices[x.indexB] }).Where(x => x.indexA >= 0 && x.indexB >= 0).ToList();
        var tb = CollectionsMarshal.AsSpan(TriadBuffer);
        for (int k = 0; k < TriadBuffer.Count; k++)
        { tb[k].indexA = newIndices[tb[k].indexA]; tb[k].indexB = newIndices[tb[k].indexB]; tb[k].indexC = newIndices[tb[k].indexC]; }
        TriadBuffer.RemoveAll(x => x.indexA < 0 || x.indexB < 0 || x.indexC < 0);
    }
    float GetCriticalVelocity(float dt, float inv_dt) => GetParticleDiameter() * inv_dt;
    float GetCriticalVelocitySquared(float dt, float inv_dt) => GetCriticalVelocity(dt, inv_dt) * GetCriticalVelocity(dt, inv_dt);
    float GetCriticalPressure(float dt, float inv_dt) => GetDensity() * GetCriticalVelocitySquared(dt, inv_dt);
    public float GetParticleStride() => Box2D.ParticleStride * GetParticleDiameter();
    public float GetParticleMass() => GetDensity() * /*GetParticleStride() * GetParticleStride();**/ GetRadius() * GetRadius() * MathF.PI;
    public float GetParticleInvMass()
    {
        //float inverseStride = InverseDiameter / Box2D.ParticleStride; return InverseDensity * inverseStride * inverseStride;
        float inverseStride = GetInverseDiameter() * 2; return GetInverseDensity() * inverseStride * inverseStride / MathF.PI;
    }
    public ParticleContactFilter GetFixtureContactFilter() => AllParticleFlags.HasFlag(ParticleFlag.FixtureContactFilter) ? World.ParticleContactFilter : null;
    public ParticleContactFilter GetParticleContactFilter() => AllParticleFlags.HasFlag(ParticleFlag.ParticleContactFilter) ? World.ParticleContactFilter : null;
    public ParticleContactListener GetFixtureContactListener() => AllParticleFlags.HasFlag(ParticleFlag.FixtureContactListener) ? World.ParticleContactListener : null;
    public ParticleContactListener GetParticleContactListener() => AllParticleFlags.HasFlag(ParticleFlag.ParticleContactListener) ? World.ParticleContactListener : null;
    public void SetGroupFlags(ParticleGroup group, ParticleGroupFlag flags)
    {
        if ((group.m_groupFlags ^ flags).HasFlag(ParticleGroupFlag.Solid)) flags |= ParticleGroupFlag.NeedsUpdateDepth;
        if ((group.m_groupFlags & ~flags) != 0) NeedsUpdateAllGroupFlags = true;
        if ((~AllGroupFlags & flags) != 0)
        {
            if (flags.HasFlag(ParticleGroupFlag.Solid)) DepthBuffer = RequestBuffer(ref DepthBuffer);
            AllGroupFlags |= flags;
        }
        group.m_groupFlags = flags;
    }
    public unsafe void RemoveSpuriousBodyContacts()
    {
        BodyContactBuffer = HPCsharp.ParallelAlgorithm.SortMergePseudoInPlacePar(BodyContactBuffer, Comparer<ParticleBodyContact>.Create(BodyContactCompare));
        int discarded = 0; ParticleBodyContactRemovePredicate predicate = new(this, &discarded);
        BodyContactBuffer.RemoveAll(x => predicate[x]);
    }
    public static int BodyContactCompare(ParticleBodyContact lhs, ParticleBodyContact rhs) =>
        lhs.index == rhs.index ? -lhs.mass.CompareTo(rhs.mass) : lhs.index.CompareTo(rhs.index);
    public unsafe void DetectStuckParticle(int particle, ref int index)
    {
        if (StuckThreshold <= 0) return;
        BodyContactCountBuffer[particle]++;
        if (BodyContactCountBuffer[particle] == 2)
        {
            ConsecutiveContactStepsBuffer[particle]++;
            if (ConsecutiveContactStepsBuffer[particle] > StuckThreshold)
                if (index < StuckParticleBuffer.Count) StuckParticleBuffer[index++] = particle; else { StuckParticleBuffer.Add(particle); index++; }
        }
        LastBodyContactStepBuffer[particle] = Timestamp;
    }
    bool ValidateParticleIndex(int index) => index >= 0 && index <= Count && index != Box2D.InvalidParticleIndex;
    int GetQuantizedtimeElapsed() => (int)(TimeElapsed >> 32);
    long LifetimeToExpirationTime(float lifetime) => TimeElapsed + (long)(lifetime / LifetimeGranularity * (1L << 32));
    static bool ForceCanBeApplied(ParticleFlag flags) => !flags.HasFlag(ParticleFlag.Wall);
    void PrepareForceBuffer() { if (!HasForce) { HPCsharp.ParallelAlgorithm.FillPar(ForceBuffer, Vector2.Zero, 0, Count); HasForce = true; } }
    void PrepareImpulseBuffer() { if (!HasImpulse) { HPCsharp.ParallelAlgorithm.FillPar(ImpulseBuffer, Vector2.Zero, 0, Count); HasImpulse = true; } }
    static bool IsRigidGroup(ParticleGroup group) => group != null && group.m_groupFlags.HasFlag(ParticleGroupFlag.Rigid);
    public Vector2 GetLinearVelocity(ParticleGroup group, int particleIndex, Vector2 point) =>
        IsRigidGroup(group) ? group.GetLinearVelocityFromWorldPoint(point) : VelocityBuffer[particleIndex];
    void InitDampingParameterWithRigidGroupOrParticle(out float invMass, out float invInertia,
        out float tangentDistance, bool isRigidGroup, ParticleGroup group, int particleIndex, Vector2 point, Vector2 normal, float particleInvMass)
    {
        if (isRigidGroup)
        {
            group.UpdateStatistics();
            invMass = group.m_invMass;
            invInertia = group.m_invInertia;
            tangentDistance = Vector2.Cross(point - group.GetCenter, normal);
        }
        else
        {
            invMass = FlagsBuffer[particleIndex].HasFlag(ParticleFlag.Wall) ? 0 : particleInvMass;
            invInertia = 0; tangentDistance = 0;
        }
    }
    public float ComputeDampingImpulse(float invMassA, float invInertiaA, float tangentDistanceA,
        float invMassB, float invInertiaB, float tangentDistanceB, float normalVelocity)
    {
        float invMass = invMassA + invInertiaA * tangentDistanceA * tangentDistanceA + invMassB + invInertiaB * tangentDistanceB * tangentDistanceB;
        return invMass > 0 ? normalVelocity / invMass : 0;
    }
    public void ApplyDamping(float invMass, float invInertia, float tangentDistance,
        bool isRigidGroup, ParticleGroup group, int particleIndex, float impulse, Vector2 normal)
    {
        if (isRigidGroup)
        {
            group.m_linearVelocity += impulse * invMass * normal;
            group.m_angularVelocity += impulse * tangentDistance * invInertia;
        }
        else VelocityBuffer[particleIndex] += impulse * invMass * normal;
    }
    public int Timestamp { get; private set; }
    public ParticleFlag AllParticleFlags { get; private set; }
    public ParticleGroupFlag AllGroupFlags { get; private set; }
    public bool NeedsUpdateAllParticleFlags { get; private set; }
    public bool NeedsUpdateAllGroupFlags { get; private set; }
    public bool HasForce { get; internal set; }
    public bool HasImpulse { get; private set; }
    public virtual float GetInverseDensity() => inverseDensity;
    protected virtual void SetInverseDensity(float value) => inverseDensity = value;
    public virtual float GetParticleDiameter() => particleDiameter;
    protected virtual void SetParticleDiameter(float value) => particleDiameter = value;
    public virtual float GetInverseDiameter() => inverseDiameter;
    protected virtual void SetInverseDiameter(float value) => inverseDiameter = value;
    public virtual float GetSquaredDiameter() => squaredDiameter;
    protected virtual void SetSquaredDiameter(float value) => squaredDiameter = value;
    public int Count { get; private set; }
    public int IterationIndex { get; private set; }
    public int InternalAllocatedCapacity { get; private set; }
    //public b2SlabAllocator<ParticleHandle> m_handleAllocator;
    public ParticleHandle[] HandleIndexBuffer;
    public ParticleFlag[] FlagsBuffer;
    public Vector2[] PositionBuffer, VelocityBuffer, ForceBuffer;
    public Vector2[] ImpulseBuffer;
    public float[] WeightBuffer; // ComputeDepth(), SolveStaticPressure() and SolvePressure()
    public float[] StaticPressureBuffer; //SolveStaticPressure(), SolvePressure() and CreateParticle()
    public float[] AccumulationBuffer;
    public Vector2[] Accumulation2Buffer; //SolveTensile() and CreateParticle()
    public float[] DepthBuffer; //SolveSolid() and CreateParticle()
    public ParticleColor[] ColorBuffer;
    public ParticleGroup[] GroupBuffer; //which particle belongs to which group
    public object[] UserDataBuffer;
    public int StuckThreshold;
    public int[] LastBodyContactStepBuffer, BodyContactCountBuffer, ConsecutiveContactStepsBuffer;
    public List<int> StuckParticleBuffer { get; } = new();
    public List<Proxy> ProxyBuffer { get; private set; } = new();
    public List<ParticleContact> ContactBuffer { get; } = new();
    public List<ParticleBodyContact> BodyContactBuffer { get; private set; } = new();
    public List<ParticlePair> PairBuffer { get; private set; } = new();
    public List<ParticleTriad> TriadBuffer { get; private set; } = new();
    public int[] ExpirationTimeBuffer, IndexByExpirationTimeBuffer;
    public long TimeElapsed { get; private set; }
    public bool ExpirationTimeBufferRequiresSorting;
    public World World { get; internal set; }
    public bool locked { get; private set; }
    public static uint ComputeTag(Vector2 v) => ((uint)(v.y + yOffset) << yShift) + (uint)(xScale * v.x + xOffset);
    public static uint ComputeRelativeTag(uint tag, int x, int y) => tag + (uint)(y << yShift) + (uint)(x << xShift);
    public static bool ParticleCanBeConnected(ParticleFlag flags, ParticleGroup group) =>
        (flags & (ParticleFlag.Wall | ParticleFlag.Spring | ParticleFlag.Elastic)) != 0 || (group != null && (group.m_groupFlags & ParticleGroupFlag.Rigid) != 0);
}
