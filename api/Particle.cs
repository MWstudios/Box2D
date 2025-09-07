using System;
using System.Diagnostics;
using Box2D.Particle;

namespace Box2D.API;

public static class ParticleAPI
{
    public static ParticleSystem Particle_CreateParticleSystem(WorldID world)
    {
        Debug.Assert(world.index1 != null);
        Debug.Assert(!world.index1.locked);
        if (world.index1.locked) return null;
        ParticleSystem p = new() { World = world.index1, particleId = world.index1.particleSystemList.Count };
        world.index1.particleSystemList.Add(p);
        return p;
    }
    public static void Particle_DestroyParticleSystem(ParticleSystem p)
    {
        Debug.Assert(p.World != null);
        Debug.Assert(!p.World.locked);
        if (p.World.locked) return;
        int movedIndex = p.World.particleSystemList.RemoveSwap(p.particleId);
        if (movedIndex != -1) p.World.particleSystemList[p.particleId].particleId = p.particleId;
    }
    public static float Particle_GetSmallestRadius(WorldID world)
    {
        if (world.index1 == null) return float.NaN;
        float smallestRadius = float.MaxValue;
        for (int i = 0; i < world.index1.particleSystemList.Count; i++)
            smallestRadius = Math.Min(smallestRadius, world.index1.particleSystemList[i].GetRadius());
        return smallestRadius;
    }
    public static int Particle_CalculateReasonableParticleIterations(WorldID world, float timeStep)
    {
        if (world.index1 == null || world.index1.particleSystemList.Count == 0) return 1;
        return Particle.Particle.CalculateParticleIterations(world.index1.gravity.Length(), Particle_GetSmallestRadius(world), timeStep);
    }
}
