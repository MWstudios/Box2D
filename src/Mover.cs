using System;

namespace Box2D;

public static class Mover
{
    /// <summary>Solves the position of a mover that satisfies the given collision planes.</summary>
    /// <param name="targetDelta">the desired movement from the position used to generate the collision planes</param>
    /// <param name="planes">the collision planes</param>
    /// <param name="count">the number of collision planes</param>
    public static PlaneSolverResult SolvePlanes(Vector2 targetDelta, CollisionPlane[] planes)
    {
        for (int i = 0; i < planes.Length; i++) planes[i].push = 0;
        Vector2 delta = targetDelta;
        float tolerance = Box2D.LinearSlop;
        int iteration;
        for (iteration = 0; iteration < 20; iteration++)
        {
            float totalPush = 0;
            for (int planeIndex = 0; planeIndex < planes.Length; planeIndex++)
            {
                ref CollisionPlane plane = ref planes[planeIndex];
                float separation = plane.plane.PlaneSeparation(delta) + Box2D.LinearSlop;
                float push = -separation;
                float accumulatedPush = plane.push;
                plane.push = Math.Clamp(plane.push + push, 0, plane.pushLimit);
                push = plane.push - accumulatedPush;
                delta = Vector2.MulAdd(delta, push, plane.plane.normal);
                totalPush += Math.Abs(push);
            }
            if (totalPush < tolerance) break;
        }
        return new() { translation = delta, iterationCount = iteration };
    }
    /// <summary>Clips the velocity against the given collision planes. Planes with zero push or clipVelocity
    /// set to false are skipped.</summary>
    public static Vector2 ClipVector(Vector2 vector, CollisionPlane[] planes)
    {
        Vector2 v = vector;
        for (int planeIndex = 0; planeIndex < planes.Length; planeIndex++)
        {
            ref CollisionPlane plane = ref planes[planeIndex];
            if (plane.push == 0 || !plane.clipVelocity) continue;
            v = Vector2.MulSub(v, Math.Min(0, Vector2.Dot(v, plane.plane.normal)), plane.plane.normal);
        }
        return v;
    }
}
