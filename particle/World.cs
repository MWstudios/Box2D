using Box2D.Particle;
using System;

namespace Box2D;

public static partial class Box2D
{
    /// <summary>A symbolic constant that stands for particle allocation error.</summary>
    public static int InvalidParticleIndex = -1;

    public static uint MaxParticleIndex = 0x7FFFFFFF;

    /// <summary>The default distance between particles, multiplied by the particle diameter.</summary>
    public static float ParticleStride = 0.75f;

    /// <summary>The minimum particle weight that produces pressure.</summary>
    public static float MinParticleWeight = 1.0f;

    /// <summary>The upper limit for particle pressure.</summary>
    public static float MaxParticlePressure = 0.25f;

    /// <summary>The upper limit for force between particles.</summary>
    public static float MaxParticleForce = 0.5f;

    /// <summary>The upper limit for particle velocity. Set to -1 to use the particle diameter (to prevent clipping through and occasional pops within the liquid).</summary>
    public static float MaxParticleVelocity = -1;

    /// <summary>The maximum distance between particles in a triad, multiplied by the particle diameter.</summary>
    public static float MaxTriadDistance = 2;
    public static float MaxTriadDistanceSquared => MaxTriadDistance * MaxTriadDistance;

    /// <summary>The initial size of particle data buffers.</summary>
    public static int MinParticleSystemBufferCapacity = 256;

    /// <summary>The time into the future that collisions against barrier particles will be detected.</summary>
    public static float BarrierCollisionTime = 2.5f;

    public static float ParticleLinearSlop = 0.005f;

    /// <summary>Multiplier for the radius of a particle when colliding with a fixture.
    /// Originally the diameter was used, so a multiplier of 2 would restore that behaviour.</summary>
    public static float FixtureParticleCollisionRadiusScaler = 1;

    /// <summary>Prevents directional bias when solving elastic triads</summary>
    public static bool ElasticPreserveVelocity = false;

    public static unsafe void ApplyLinearImpulse(this Body body, World world, Vector2 impulse, Vector2 point)
    {
        if (body.type != BodyType.Dynamic || body.setIndex == (int)SetType.Disabled) return;
        if (body.setIndex >= (int)SetType.FirstSleeping) world.WakeBody(body);
        if (body.setIndex == (int)SetType.Awake)
        {
            int localIndex = body.localIndex;
            SolverSet set = world.solverSets[(int)SetType.Awake];
            BodyState* state = set.bodyStates.Data + localIndex;
            BodySim bodySim = set.bodySims[localIndex];
            state->linearVelocity = Vector2.MulAdd(state->linearVelocity, bodySim.invMass, impulse);
            state->angularVelocity += bodySim.invInertia * Vector2.Cross(point - bodySim.center, impulse);
            state->LimitVelocity(world.maxLinearSpeed);
        }
    }
}

public class ParticleQueryCallback
{
    public Func<ParticleSystem, bool> ShouldQueryParticleSystem = _ => true;
    public Func<ParticleSystem, int, bool> ReportParticle = (_, _) => false;
}

public class ParticleRayCastCallback
{
    public Func<ParticleSystem, bool> ShouldQueryParticleSystem = _ => true;
    public Func<ParticleSystem, int, Vector2, Vector2, float, float> ReportParticle = (_, _, _, _, _) => 0;
}

public class ParticleContactFilter
{
    public Func<Shape, ParticleSystem, int, bool> ShouldCollideFP = (_, _, _) => true;
    public Func<ParticleSystem, int, int, bool> ShouldCollidePP = (_, _, _) => true;
}

public class ParticleContactListener
{
    public Action<ParticleSystem, ParticleBodyContact> BeginContactPPBC = null;
    public Action<Shape, ParticleSystem, int> EndContactFP = null;
    public Action<ParticleSystem, ParticleContact> BeginContactPPC = null;
    public Action<ParticleSystem, int, int> EndContactP = null;
}

public partial class World
{
    public Action<ParticleSystem, int> ParticleRemoved = (_, _) => { };
    public ParticleContactFilter ParticleContactFilter = new();
    public ParticleContactListener ParticleContactListener = new();
    public unsafe Vector2 GetLinearVelocityFromWorldPoint(Body body, Vector2 worldPoint) =>
        body.type == BodyType.Static ? Vector2.Zero :
        GetBodyState(body)->linearVelocity + Vector2.CrossSV(GetBodyState(body)->angularVelocity, worldPoint - GetBodySim(body).center);
}
