using System;
using System.Diagnostics;

namespace Box2D.Particle;

[Flags] public enum ParticleGroupFlag
{
    /// <summary>Prevents overlapping or leaking.</summary>
    Solid = 1,
    /// <summary>Keeps its shape.</summary>
    Rigid = 2,
    /// <summary>Won't be destroyed if it gets empty.</summary>
    CanBeEmpty = 4,
    /// <summary>Will be destroyed on next simulation step.</summary>
    WillBeDestroyed = 8,
    /// <summary>Updates depth data on next simulation step.</summary>
    NeedsUpdateDepth = 16,
    /// <summary></summary>
    InternalMask = WillBeDestroyed | NeedsUpdateDepth
}
public struct ParticleGroupDef
{
    public ParticleFlag Flags; public ParticleGroupFlag GroupFlags; public Vector2 Position, LinearVelocity; public float Angle, AngularVelocity;
    public ParticleColor Color; public float Strength; public Shape Shape; public Shape[] Shapes; public float Stride; public int ParticleCount; public Vector2[] PositionData;
    public float Lifetime; public object UserData; public ParticleGroup Group; public bool OwnShapesArray, TriangleGrid;
    public ParticleGroupDef()
    {
        Flags = 0; GroupFlags = 0; Position = Vector2.Zero; Angle = 0; LinearVelocity = Vector2.Zero; AngularVelocity = 0; Strength = 1;
        Color = ParticleColor.Zero; Shape = null; Shapes = null; Stride = 0; ParticleCount = 0; PositionData = null; Lifetime = 0;
        UserData = null; Group = null; OwnShapesArray = false; TriangleGrid = false;
    }
    public void SetCircleShapesFromVertexList(Vector2[] inBuf, int numShapes, float radius)
    {
        Shape[] pShapes = new Shape[numShapes];
        for (int i = 0; i < numShapes; i++)
            pShapes[i] = new() { shape = new Circle { radius = radius, center = inBuf[i] } };
        OwnShapesArray = true; Shapes = pShapes;
    }
    public void SetPosition(float x, float y) => Position = new(x, y);
    public void SetColor(int r, int g, int b, int a) => Color = new((byte)r, (byte)g, (byte)b, (byte)a);
}
public class ParticleGroup
{
    public bool ContainsParticle(int index) => BufferIndex <= index && index < LastIndex;
    public ParticleFlag GetAllParticleFlags()
    {
        ParticleFlag flags = 0;
        for (int i = BufferIndex; i < LastIndex; i++)
        {
            flags |= System.FlagsBuffer[i];
        }
        return flags;
    }
    ParticleGroupFlag GetGroupFlags() => m_groupFlags & ~ParticleGroupFlag.InternalMask;
    public void SetGroupFlags(ParticleGroupFlag flags)
    {
        Debug.Assert((flags & ParticleGroupFlag.InternalMask) == 0);
        flags |= m_groupFlags & ParticleGroupFlag.InternalMask;
        System.SetGroupFlags(this, flags);
    }
    public float GetMass { get { UpdateStatistics(); return m_mass; } }
    public float GetInertia { get { UpdateStatistics(); return m_inertia; } }
    public Vector2 GetCenter { get { UpdateStatistics(); return m_center; } }
    public Vector2 GetLinearVelocity { get { UpdateStatistics(); return m_linearVelocity; } }
    public float GetAngularVelocity { get { UpdateStatistics(); return m_angularVelocity; } }
    public Vector2 Position => Transform.p;
    public float Angle => Transform.q.GetAngle();
    public Vector2 GetLinearVelocityFromWorldPoint(Vector2 worldPoint)
    {
        UpdateStatistics();
        Vector2 a = worldPoint - m_center;
        return m_linearVelocity + Vector2.CrossSV(m_angularVelocity, a);
    }
    public void ApplyForce(Vector2 force) => System.ApplyForce(BufferIndex, LastIndex, force);
    public void ApplyLinearImpulse(Vector2 impulse) => System.ApplyLinearImpulse(BufferIndex, LastIndex, impulse);
    public void DestroyParticles(bool callDestructionListener)
    {
        Debug.Assert(!System.World.locked);
        if (System.World.locked) return;
        for (int i = BufferIndex; i < LastIndex; i++) System.DestroyParticle(i, callDestructionListener);
    }
    public void DestroyParticles() => DestroyParticles(false);
    public ParticleSystem System { get; internal set; }
    public int BufferIndex { get; set; }
    public int LastIndex;
    public int ParticleCount => LastIndex - BufferIndex;
    public ParticleGroupFlag m_groupFlags;
    public float m_strength = 1;
    public int m_timestamp = -1;
    public float m_mass, m_inertia, m_invMass, m_invInertia, m_angularVelocity;
    public Vector2 m_center, m_linearVelocity;
    public Transform Transform { get; set; } = Transform.Identity;
    public object UserData { get; set; }
    public void UpdateStatistics()
    {
        if (m_timestamp != System.Timestamp)
        {
            float m = System.GetParticleMass();
            m_mass = 0; m_center = Vector2.Zero; m_linearVelocity = Vector2.Zero;
            for (int i = BufferIndex; i < LastIndex; i++)
            {
                m_mass += m; m_center += m * System.PositionBuffer[i];
                m_linearVelocity += m * System.VelocityBuffer[i];
            }
            if (m_mass > 0) { m_invMass = 1 / m_mass; m_center *= m_invMass; m_linearVelocity *= m_invMass; }
            else m_invMass = 0;
            m_inertia = 0; m_angularVelocity = 0;
            for (int i = BufferIndex; i < LastIndex; i++)
            {
                Vector2 p = System.PositionBuffer[i] - m_center;
                Vector2 v = System.VelocityBuffer[i] - m_linearVelocity;
                m_inertia += m * Vector2.Dot(p, p);
                m_angularVelocity += m * Vector2.Cross(p, v);
            }
            if (m_inertia > 0) { m_invInertia = 1 / m_inertia; m_angularVelocity *= m_invInertia; }
            else m_invInertia = 0;
            m_timestamp = System.Timestamp;
        }
    }
}
