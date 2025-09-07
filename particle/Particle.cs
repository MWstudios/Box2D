using System;

namespace Box2D.Particle
{
    [Flags]
    public enum ParticleFlag
    {
        /// <summary>Water particle.</summary>
        Water = 0,
        /// <summary>Removed after next simulation step.</summary>
        Zombie = 0x2,
        /// <summary>Zero velocity.</summary>
        Wall = 0x4,
        /// <summary>With restitution from stretching.</summary>
        Spring = 0x8,
        /// <summary>With restitution from deformation.</summary>
        Elastic = 0x10,
        /// <summary>With viscosity.</summary>
        Viscous = 0x20,
        /// <summary>Without isotropic pressure.</summary>
        Powder = 0x40,
        /// <summary>With surface tension.</summary>
        Tensile = 0x80,
        /// <summary>Mix color between contacting particles.</summary>
        ColorMixing = 0x100,
        /// <summary>Call b2DestructionListener on destruction.</summary>
        DestructionListener = 0x200,
        /// <summary>Prevents other particles from leaking.</summary>
        Barrier = 0x400,
        /// <summary>Less compressibility.</summary>
        StaticPressure = 0x800,
        /// <summary>Makes pairs or triads with other particles.</summary>
        Reactive = 0x1000,
        /// <summary>With high repulsive force.</summary>
        Repulsive = 0x2000,
        /// <summary>Call b2ContactListener when this particle is about to interact with a rigid body or stops interacting with a rigid body.
        /// This results in an expensive operation compared to using b2_fixtureContactFilterParticle to detect collisions between particles.</summary>
        FixtureContactListener = 0x4000,
        /// <summary>Call b2ContactListener when this particle is about to interact with another particle or stops interacting with another particle.
        /// This results in an expensive operation compared to using b2_particleContactFilterParticle to detect collisions between particles.</summary>
        ParticleContactListener = 0x8000,
        /// <summary>Call b2ContactFilter when this particle interacts with rigid bodies.</summary>
        FixtureContactFilter = 0x10000,
        /// <summary>Call b2ContactFilter when this particle interacts with other
        /// particles.</summary>
        ParticleContactFilter = 0x20000
    }
    public struct ParticleColor
    {
        public byte r, g, b, a; static byte k_bitsPerComponent = sizeof(byte) << 3;
        static float k_maxValue = (1u << k_bitsPerComponent) - 1; static float k_inverseMaxValue = 1f / k_maxValue;
        public static ParticleColor Zero => new(0, 0, 0, 0);
        public ParticleColor(byte _r, byte _g, byte _b, byte _a) { r = _r; g = _g; b = _b; a = _a; }
        public bool IsZero() => r == 0 && g == 0 && b == 0 && a == 0;
        public static ParticleColor operator *(ParticleColor c, float s)
        { c.r = (byte)(c.r * s); c.g = (byte)(c.g * s); c.b = (byte)(c.b * s); c.a = (byte)(c.a * s); return c; }
        public static ParticleColor operator *(ParticleColor c, byte s)
        { int scale = s + 1; c.r = (byte)((c.r * scale) >> k_bitsPerComponent); c.g = (byte)((c.g * scale) >> k_bitsPerComponent); c.b = (byte)((c.b * scale) >> k_bitsPerComponent); c.a = (byte)((c.a * scale) >> k_bitsPerComponent); return c; }
        public static ParticleColor operator +(ParticleColor c1, ParticleColor c2)
        { c1.r += c2.r; c1.g += c2.g; c1.b += c2.b; c1.a += c2.a; return c1; }
        public static ParticleColor operator -(ParticleColor c1, ParticleColor c2)
        { c1.r -= c2.r; c1.g -= c2.g; c1.b -= c2.b; c1.a -= c2.a; return c1; }
        public static bool operator ==(ParticleColor c1, ParticleColor c2) => c1.r == c2.r && c1.g == c2.g && c1.b == c2.b && c1.a == c2.a;
        public static bool operator !=(ParticleColor c1, ParticleColor c2) => c1.r != c2.r || c1.g != c2.g || c1.b != c2.b || c1.a != c2.a;
        public void Mix(ParticleColor mixColor, int strength) => MixColors(ref this, ref mixColor, strength);
        public static void MixColors(ref ParticleColor colorA, ref ParticleColor colorB, int strength)
        {
            byte dr = (byte)((strength * (colorB.r - colorA.r)) >> k_bitsPerComponent),
                dg = (byte)((strength * (colorB.g - colorA.g)) >> k_bitsPerComponent),
                db = (byte)((strength * (colorB.b - colorA.b)) >> k_bitsPerComponent),
                da = (byte)((strength * (colorB.a - colorA.a)) >> k_bitsPerComponent);
            colorA.r += dr; colorA.g += dg; colorA.b += db; colorA.a += da;
            colorB.r -= dr; colorB.g -= dg; colorB.b -= db; colorB.a -= da;
        }
        public override bool Equals(object obj) => obj is ParticleColor color && this == color;
        public override int GetHashCode() => HashCode.Combine(r, g, b, a);
    }
    public struct ParticleDef
    {
        public ParticleFlag flags; public Vector2 position, velocity; public ParticleColor color; public float lifetime; public object userData; public ParticleGroup group;
        public void SetPosition(float x, float y) => position = new(x, y);
        public void SetColor(int r, int g, int b, int a) => color = new((byte)r, (byte)g, (byte)b, (byte)a);
    }
    public class ParticleHandle { public int Index { get; set; } }
    class Particle
    {
        public static int CalculateParticleIterations(float gravity, float radius, float timeStep)
        {
            int maxRecommendedParticleIterations = 8;
            float radiusThreshold = 0.01f;
            int iterations = (int)Math.Ceiling(Math.Sqrt(gravity / (radiusThreshold * radius)) * timeStep);
            return Math.Clamp(iterations, 1, maxRecommendedParticleIterations);
        }
    }
}
