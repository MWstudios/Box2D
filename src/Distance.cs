using System;
using System.Diagnostics;

namespace Box2D;

public static class Distance
{
    /// <summary>Evaluate the transform sweep at a specific time.</summary>
    public static Transform GetSweepTransform(this Sweep sweep, float time)
    {
        Transform xf = new()
        {
            p = (1 - time) * sweep.c1 + time * sweep.c2,
            q = new Rotation((1 - time) * sweep.q1.c + time * sweep.q2.c, (1 - time) * sweep.q1.s + time * sweep.q2.s).Normalize()
        };
        xf.p -= xf.q * sweep.localCenter;
        return xf;
    }
    /// <summary>Compute the distance between two line segments, clamping at the end points if needed.</summary>
    public static SegmentDistanceResult SegmentDistance(Vector2 p1, Vector2 q1, Vector2 p2, Vector2 q2)
    {
        SegmentDistanceResult result = new();
        Vector2 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
        float dd1 = Vector2.Dot(d1, d1), dd2 = Vector2.Dot(d2, d2);
        float rd1 = Vector2.Dot(r, d1), rd2 = Vector2.Dot(r, d2);
        float epsSqr = Box2D.FLT_EPSILON * Box2D.FLT_EPSILON;
        if (dd1 < epsSqr || dd2 < epsSqr)
        {
            if (dd1 >= epsSqr)
            {
                result.fraction1 = Math.Clamp(-rd1 / dd1, 0, 1);
                result.fraction2 = 0;
            }
            else if (dd2 >= epsSqr)
            {
                result.fraction1 = 0;
                result.fraction2 = Math.Clamp(rd2 / dd2, 0, 1);
            }
            else
            {
                result.fraction1 = 0;
                result.fraction2 = 0;
            }
        }
        else
        {
            float d12 = Vector2.Dot(d1, d2);
            float denominator = dd1 * dd2 - d12 * d12;
            float f1 = 0;
            if (denominator != 0)
            {
                f1 = Math.Clamp((d12 * rd2 - rd1 * dd2) / denominator, 0, 1);
            }
            float f2 = (d12 * f1 + rd2) / dd2;
            if (f2 < 0)
            {
                f2 = 0;
                f1 = Math.Clamp(-rd1 / dd1, 0, 1);
            }
            else if (f2 > 1)
            {
                f2 = 1;
                f1 = Math.Clamp((d12 - rd1) / dd1, 0, 1);
            }
            result.fraction1 = f1;
            result.fraction2 = f2;
        }
        result.closest1 = Vector2.MulAdd(p1, result.fraction1, d1);
        result.closest2 = Vector2.MulAdd(p2, result.fraction2, d2);
        result.distanceSquared = Vector2.DistanceSquared(result.closest1, result.closest2);
        return result;
    }
    /// <summary>Make a proxy for use in overlap, shape cast, and related functions. This is a deep copy of the points.</summary>
    public static ShapeProxy MakeProxy(Vector2[] points, float radius) => new() { points = points, radius = radius };
    public static ShapeProxy MakeOffsetProxy(Vector2[] points, float radius, Vector2 position, Rotation rotation)
    {
        Transform transform = new() { p = position, q = rotation };
        ShapeProxy proxy = new() { points = new Vector2[points.Length], radius = radius };
        for (int i = 0; i < points.Length; i++) proxy.points[i] = transform.TransformPoint(points[i]);
        return proxy;
    }
    public static Vector2 Weight2(float a1, Vector2 w1, float a2, Vector2 w2)
        => new(a1 * w1.x + a2 * w2.x, a1 * w1.y + a2 * w2.y);
    public static Vector2 Weight3(float a1, Vector2 w1, float a2, Vector2 w2, float a3, Vector2 w3)
        => new(a1 * w1.x + a2 * w2.x + a3 * w3.x, a1 * w1.y + a2 * w2.y + a3 * w3.y);
    public static int FindSupport(this ShapeProxy proxy, Vector2 direction)
    {
        int bestIndex = 0;
        float bestValue = Vector2.Dot(proxy.points[0], direction);
        for (int i = 1; i < proxy.points.Length; i++)
        {
            float value = Vector2.Dot(proxy.points[i], direction);
            if (value > bestValue)
            {
                bestIndex = i;
                bestValue = value;
            }
        }
        return bestIndex;
    }
    public static Simplex MakeSimplexFromCache(this ref SimplexCache cache, ShapeProxy proxyA, ShapeProxy proxyB)
    {
        Debug.Assert(cache.count <= 3);
        Simplex s = new() { count = cache.count };
        if (s.count == 0)
        {
            s.v1 = new() { indexA = 0, indexB = 0, wA = proxyA.points[0], wB = proxyB.points[0], a = 1 };
            s.v1.w = s.v1.wA - s.v1.wB;
            s.count = 1;
        }
        else
        {
            if (s.count > 0)
            {
                s.v1 = new() { indexA = cache.indexA[0], indexB = cache.indexB[0], a = -1 };
                s.v1.wA = proxyA.points[s.v1.indexA];
                s.v1.wB = proxyA.points[s.v1.indexB];
                s.v1.w = s.v1.wA - s.v1.wB;
            }
            if (s.count > 1)
            {
                s.v2 = new() { indexA = cache.indexA[1], indexB = cache.indexB[1], a = -1 };
                s.v2.wA = proxyA.points[s.v2.indexA];
                s.v2.wB = proxyA.points[s.v2.indexB];
                s.v2.w = s.v2.wA - s.v2.wB;
            }
            if (s.count > 2)
            {
                s.v3 = new() { indexA = cache.indexA[2], indexB = cache.indexB[2], a = -1 };
                s.v3.wA = proxyA.points[s.v3.indexA];
                s.v3.wB = proxyA.points[s.v3.indexB];
                s.v3.w = s.v3.wA - s.v3.wB;
            }
        }
        return s;
    }
    public static void MakeSimplexCache(this ref SimplexCache cache, ref Simplex simplex)
    {
        cache.count = (ushort)simplex.count;
        if (simplex.count > 0) { cache.indexA[0] = (byte)simplex.v1.indexA; cache.indexB[0] = (byte)simplex.v1.indexB; }
        if (simplex.count > 1) { cache.indexA[1] = (byte)simplex.v2.indexA; cache.indexB[1] = (byte)simplex.v2.indexB; }
        if (simplex.count > 2) { cache.indexA[2] = (byte)simplex.v3.indexA; cache.indexB[2] = (byte)simplex.v3.indexB; }
    }
    public static void ComputeSimplexWitnessPoints(out Vector2 a, out Vector2 b, ref Simplex s)
    {
        switch (s.count)
        {
            case 1: a = s.v1.wA; b = s.v1.wB; break;
            case 2: a = Weight2(s.v1.a, s.v1.wA, s.v2.a, s.v2.wA); b = Weight2(s.v1.a, s.v1.wB, s.v2.a, s.v2.wB); break;
            case 3: a = Weight3(s.v1.a, s.v1.wA, s.v2.a, s.v2.wA, s.v3.a, s.v3.wA); b = a; break;
            default: a = Vector2.Zero; b = Vector2.Zero; throw new ArgumentOutOfRangeException();
        }
    }
    public static Vector2 SolveSimplex2(this ref Simplex s)
    {
        Vector2 w1 = s.v1.w, w2 = s.v2.w, e12 = w2 - w1;
        float d12_2 = -Vector2.Dot(w1, e12);
        if (d12_2 <= 0) { s.v1.a = 1; s.count = 1; return -w1; }
        float d12_1 = Vector2.Dot(w2, e12);
        if (d12_1 <= 0) { s.v2.a = 1; s.count = 1; s.v1 = s.v2; return -w2; }
        float inv_d12 = 1 / (d12_1 + d12_2);
        s.v1.a = d12_1 * inv_d12;
        s.v2.a = d12_2 * inv_d12;
        s.count = 2;
        return Vector2.CrossSV(Vector2.Cross(w1 + w2, e12), e12);
    }
    public static Vector2 SolveSimplex3(this ref Simplex s)
    {
        Vector2 w1 = s.v1.w, w2 = s.v2.w, w3 = s.v3.w, e12 = w2 - w1;
        float w1e12 = Vector2.Dot(w1, e12), w2e12 = Vector2.Dot(w2, e12);
        float d12_1 = w2e12, d12_2 = -w1e12;
        Vector2 e13 = w3 - w1;
        float w1e13 = Vector2.Dot(w1, e13), w3e13 = Vector2.Dot(w3, e13);
        float d13_1 = w3e13, d13_2 = -w1e13;
        Vector2 e23 = w3 - w2;
        float w2e23 = Vector2.Dot(w2, e23), w3e23 = Vector2.Dot(w3, e23);
        float d23_1 = w3e23, d23_2 = -w2e23;
        float n123 = Vector2.Cross(e12, e13);
        float d123_1 = n123 * Vector2.Cross(w2, w3), d123_2 = n123 * Vector2.Cross(w3, w1), d123_3 = n123 * Vector2.Cross(w1, w2);
        if (d12_2 <= 0 && d13_2 <= 0) { s.v1.a = 1; s.count = 1; return -w1; }
        if (d12_1 > 0 && d12_2 > 0 && d123_3 <= 0)
        {
            float inv_d12 = 1 / (d12_1 + d12_2);
            s.v1.a = d12_1 * inv_d12;
            s.v2.a = d12_2 * inv_d12;
            s.count = 2;
            return Vector2.CrossSV(Vector2.Cross(w1 + w2, e12), e12);
        }
        if (d13_1 > 0 && d13_2 > 0 && d123_2 <= 0)
        {
            float inv_d13 = 1 / (d13_1 + d13_2);
            s.v1.a = d13_1 * inv_d13;
            s.v3.a = d13_2 * inv_d13;
            s.count = 2;
            s.v2 = s.v3;
            return Vector2.CrossSV(Vector2.Cross(w1 + w3, e13), e13);
        }
        if (d12_1 <= 0 && d23_2 <= 0) { s.v1.a = 1; s.count = 1; s.v1 = s.v2; return -w2; }
        if (d13_1 <= 0 && d23_1 <= 0) { s.v3.a = 1; s.count = 1; s.v1 = s.v3; return -w3; }
        if (d23_1 > 0 && d23_2 > 0 && d123_1 <= 0)
        {
            float inv_d23 = 1 / (d23_1 + d23_2);
            s.v2.a = d23_1 * inv_d23;
            s.v3.a = d23_2 * inv_d23;
            s.count = 2;
            s.v1 = s.v3;
            return Vector2.CrossSV(Vector2.Cross(w2 + w3, e23), e23);
        }
        float inv_d123 = 1 / (d123_1 + d123_2 + d123_3);
        s.v1.a = d123_1 * inv_d123;
        s.v2.a = d123_2 * inv_d123;
        s.v3.a = d123_3 * inv_d123;
        s.count = 3;
        return Vector2.Zero;
    }
    /// <summary>Compute the closest points between two shapes represented as point clouds.
    /// b2SimplexCache cache is input/output. On the first call set b2SimplexCache.count to zero.
    /// The underlying GJK algorithm may be debugged by passing in debug simplexes and capacity. You may pass in NULL and 0 for these.</summary>
    public static DistanceOutput ShapeDistance(this ref DistanceInput input, ref SimplexCache cache, Simplex[] simplexes)
    {
        Debug.Assert(input.proxyA.points.Length > 0 && input.proxyB.points.Length > 0);
        Debug.Assert(input.proxyA.radius >= 0);
        Debug.Assert(input.proxyB.radius >= 0);
        DistanceOutput output = new();
        ShapeProxy proxyA = input.proxyA;
        ShapeProxy localProxyB = new();
        {
            Transform transform = Transform.InvMulTransforms(input.transformA, input.transformB);
            localProxyB.points = new Vector2[input.proxyB.points.Length];
            for (int i = 0; i < localProxyB.points.Length; i++)
                localProxyB.points[i] = transform.TransformPoint(input.proxyB.points[i]);
        }
        Simplex simplex = cache.MakeSimplexFromCache(proxyA, localProxyB);
        int simplexIndex = 0;
        if (simplexes != null && simplexIndex < simplexes.Length)
        {
            simplexes[simplexIndex] = simplex;
            simplexIndex++;
        }
        Vector2 nonUnitNormal = Vector2.Zero;
        int saveA1 = 0, saveA2 = 0, saveA3 = 0, saveB1 = 0, saveB2 = 0, saveB3 = 0;
        const int maxIterations = 20;
        int iteration = 0;
        while (iteration < maxIterations)
        {
            int saveCount = simplex.count;
            if (saveCount > 0) { saveA1 = simplex.v1.indexA; saveB1 = simplex.v1.indexB; }
            if (saveCount > 1) { saveA2 = simplex.v2.indexA; saveB2 = simplex.v2.indexB; }
            if (saveCount > 2) { saveA3 = simplex.v3.indexA; saveB3 = simplex.v3.indexB; }
            Vector2 d = simplex.count switch
            {
                1 => -simplex.v1.w,
                2 => SolveSimplex2(ref simplex),
                3 => SolveSimplex3(ref simplex),
                _ => throw new ArgumentOutOfRangeException("Invalid simplex point count"),
            };
            if (simplex.count == 3)
            {
                ComputeSimplexWitnessPoints(out Vector2 localPointA, out Vector2 localPointB, ref simplex);
                output.pointA = input.transformA.TransformPoint(localPointA);
                output.pointB = input.transformB.TransformPoint(localPointB);
                return output;
            }

            if (Vector2.Dot(d, d) < Box2D.FLT_EPSILON * Box2D.FLT_EPSILON)
            {
                ComputeSimplexWitnessPoints(out Vector2 localPointA, out Vector2 localPointB, ref simplex);
                output.pointA = input.transformA.TransformPoint(localPointA);
                output.pointB = input.transformB.TransformPoint(localPointB);
                return output;
            }
            nonUnitNormal = d;
            ref SimplexVertex vertex = ref simplex.v1;
            if (simplex.count == 1) vertex = ref simplex.v2;
            if (simplex.count == 2) vertex = ref simplex.v3;
            vertex.indexA = proxyA.FindSupport(d);
            vertex.wA = proxyA.points[vertex.indexA];
            vertex.indexB = localProxyB.FindSupport(-d);
            vertex.wB = localProxyB.points[vertex.indexB];
            vertex.w = vertex.wA - vertex.wB;
            ++iteration;
            bool duplicate = false;
            if (saveCount > 0)
            {
                if (vertex.indexA == saveA1 && vertex.indexB == saveB1) duplicate = true;
                else if (saveCount > 1)
                {
                    if (vertex.indexA == saveA2 && vertex.indexB == saveB2) duplicate = true;
                    else if (saveCount > 2)
                    {
                        if (vertex.indexA == saveA3 && vertex.indexB == saveB3) duplicate = true;
                    }
                }
            }
            if (duplicate) break;
            simplex.count++;
        }
        Vector2 normal = nonUnitNormal.Normalize();
        Debug.Assert(normal.IsNormalized());
        normal = input.transformA.q * normal;
        {
            ComputeSimplexWitnessPoints(out Vector2 localPointA, out Vector2 localPointB, ref simplex);
            output.normal = normal;
            output.distance = Vector2.Distance(localPointA, localPointB);
            output.pointA = input.transformA.TransformPoint(localPointA);
            output.pointB = input.transformB.TransformPoint(localPointB);
        }
        output.iterations = iteration;
        output.simplexCount = simplexIndex;
        cache.MakeSimplexCache(ref simplex);
        if (input.useRadii && output.distance > 0.1f * Box2D.LinearSlop)
        {
            float radiusA = input.proxyA.radius, radiusB = input.proxyB.radius;
            output.distance = Math.Max(0, output.distance - radiusA - radiusB);
            output.pointA = Vector2.MulAdd(output.pointA, radiusA, normal);
            output.pointB = Vector2.MulSub(output.pointB, radiusB, normal);
        }
        return output;
    }
    /// <summary>Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and translation fraction.
    /// Initially touching shapes are treated as a miss.</summary>
    public static CastOutput ShapeCast(this ref ShapeCastPairInput input)
    {
        float linearSlop = Box2D.LinearSlop;
        float totalRadius = input.proxyA.radius + input.proxyB.radius;
        float target = Math.Max(linearSlop, totalRadius - linearSlop);
        float tolerance = 0.25f * linearSlop;
        Debug.Assert(target > tolerance);
        SimplexCache cache = new();
        float fraction = 0;
        DistanceInput distanceInput = new()
        {
            proxyA = input.proxyA,
            proxyB = input.proxyB,
            transformA = input.transformA,
            transformB = input.transformB,
            useRadii = false
        };
        Vector2 delta2 = input.translationB;
        CastOutput output = new();
        int iteration = 0;
        const int maxIterations = 20;
        for (; iteration < maxIterations; iteration++)
        {
            output.iterations++;
            DistanceOutput distanceOutput = distanceInput.ShapeDistance(ref cache, null);
            if (distanceOutput.distance < target + tolerance)
            {
                if (iteration == 0)
                {
                    if (input.canEncroach && distanceOutput.distance > 2 * linearSlop)
                        target = distanceOutput.distance - linearSlop;
                    else
                    {
                        output.hit = true;
                        Vector2 c1 = Vector2.MulAdd(distanceOutput.pointA, input.proxyA.radius, distanceOutput.normal);
                        Vector2 c2 = Vector2.MulAdd(distanceOutput.pointB, -input.proxyB.radius, distanceOutput.normal);
                        output.point = Vector2.Lerp(c1, c2, 0.5f);
                        return output;
                    }
                }
                else
                {
                    Debug.Assert(distanceOutput.distance > 0 && distanceOutput.normal.IsNormalized());
                    output.fraction = fraction;
                    output.point = Vector2.MulAdd(distanceOutput.pointA, input.proxyA.radius, distanceOutput.normal);
                    output.normal = distanceOutput.normal;
                    output.hit = true;
                    return output;
                }
            }
            Debug.Assert(distanceOutput.distance > 0);
            Debug.Assert(distanceOutput.normal.IsNormalized());
            float denominator = Vector2.Dot(delta2, distanceOutput.normal);
            if (denominator >= 0) return output;
            fraction += (target - distanceOutput.distance) / denominator;
            if (fraction >= input.maxFraction) return output;
            distanceInput.transformB.p = Vector2.MulAdd(input.transformB.p, fraction, delta2);
        }
        return output;
    }
    public enum SeparationType { Points, FaceA, FaceB }
    public struct SeparationFunction
    {
        public ShapeProxy proxyA, proxyB;
        public Sweep sweepA, sweepB;
        public Vector2 localPoint, axis;
        public SeparationType type;
    }
    public static SeparationFunction MakeSeparationFunction(this ref SimplexCache cache, ShapeProxy proxyA, ref Sweep sweepA,
        ShapeProxy proxyB, ref Sweep sweepB, float t1)
    {
        SeparationFunction f = new()
        {
            proxyA = proxyA,
            proxyB = proxyB,
            sweepA = sweepA,
            sweepB = sweepB,
        };
        int count = cache.count;
        Debug.Assert(0 < count && count < 3);
        Transform xfA = sweepA.GetSweepTransform(t1), xfB = sweepB.GetSweepTransform(t1);
        if (count == 1)
        {
            f.type = SeparationType.Points;
            Vector2 localPointA = proxyA.points[cache.indexA[0]];
            Vector2 localPointB = proxyB.points[cache.indexB[0]];
            Vector2 pointA = xfA.TransformPoint(localPointA);
            Vector2 pointB = xfB.TransformPoint(localPointB);
            f.axis = (pointB - pointA).Normalize();
            f.localPoint = Vector2.Zero;
            return f;
        }
        if (cache.indexA[0] == cache.indexA[1])
        {
            f.type = SeparationType.FaceB;
            Vector2 localPointB1 = proxyB.points[cache.indexB[0]];
            Vector2 localPointB2 = proxyB.points[cache.indexB[1]];
            f.axis = Vector2.CrossVS(localPointB2 - localPointB1, 1).Normalize();
            Vector2 normal = xfB.q * f.axis;
            f.localPoint = new(0.5f * (localPointB1.x + localPointB2.x), 0.5f * (localPointB1.y + localPointB2.y));
            Vector2 pointB = xfB.TransformPoint(f.localPoint);
            Vector2 localPointA = proxyA.points[cache.indexA[0]];
            Vector2 pointA = xfA.TransformPoint(localPointA);
            float s = Vector2.Dot(pointA - pointB, normal);
            if (s < 0) f.axis = -f.axis;
            return f;
        }
        else
        {
            f.type = SeparationType.FaceA;
            Vector2 localPointA1 = proxyA.points[cache.indexA[0]];
            Vector2 localPointA2 = proxyA.points[cache.indexA[1]];
            f.axis = Vector2.CrossVS(localPointA2 - localPointA1, 1).Normalize();
            Vector2 normal = xfA.q * f.localPoint;
            f.localPoint = new(0.5f * (localPointA1.x + localPointA2.x), 0.5f * (localPointA1.y + localPointA2.y));
            Vector2 pointA = xfA.TransformPoint(f.localPoint);
            Vector2 localPointB = proxyB.points[cache.indexB[0]];
            Vector2 pointB = xfB.TransformPoint(localPointB);
            float s = Vector2.Dot(pointB - pointA, normal);
            if (s < 0) f.axis = -f.axis;
            return f;
        }
    }
    public static float FindMinSeparation(this ref SeparationFunction f, out int indexA, out int indexB, float t)
    {
        Transform xfA = f.sweepA.GetSweepTransform(t), xfB = f.sweepB.GetSweepTransform(t);
        switch (f.type)
        {
            case SeparationType.Points:
                {
                    Vector2 axisA = xfA.q.InvRotateVector(f.axis);
                    Vector2 axisB = xfB.q.InvRotateVector(-f.axis);
                    indexA = f.proxyA.FindSupport(axisA);
                    indexB = f.proxyB.FindSupport(axisB);
                    Vector2 localPointA = f.proxyA.points[indexA];
                    Vector2 localPointB = f.proxyB.points[indexB];
                    Vector2 pointA = xfA.TransformPoint(localPointA);
                    Vector2 pointB = xfB.TransformPoint(localPointB);
                    return Vector2.Dot(pointB - pointA, f.axis);
                }
            case SeparationType.FaceA:
                {
                    Vector2 normal = xfA.q * f.axis;
                    Vector2 pointA = xfA.TransformPoint(f.localPoint);
                    Vector2 axisB = xfB.q.InvRotateVector(-normal);
                    indexA = -1;
                    indexB = f.proxyB.FindSupport(axisB);
                    Vector2 localPointB = f.proxyB.points[indexB];
                    Vector2 pointB = xfB.TransformPoint(localPointB);
                    return Vector2.Dot(pointB - pointA, normal);
                }
            case SeparationType.FaceB:
                {
                    Vector2 normal = xfB.q * f.axis;
                    Vector2 pointB = xfB.TransformPoint(f.localPoint);
                    Vector2 axisA = xfA.q.InvRotateVector(-normal);
                    indexB = -1;
                    indexA = f.proxyA.FindSupport(axisA);
                    Vector2 localPointA = f.proxyA.points[indexA];
                    Vector2 pointA = xfA.TransformPoint(localPointA);
                    return Vector2.Dot(pointA - pointB, normal);
                }
            default:
                {
                    indexA = -1;
                    indexB = -1;
                    throw new ArgumentOutOfRangeException("Invalid separation type");
                }
        }
    }
    public static float EvaluateSeparation(this ref SeparationFunction f, int indexA, int indexB, float t)
    {
        Transform xfA = f.sweepA.GetSweepTransform(t), xfB = f.sweepB.GetSweepTransform(t);
        switch (f.type)
        {
            case SeparationType.Points:
                {
                    Vector2 localPointA = f.proxyA.points[indexA];
                    Vector2 localPointB = f.proxyB.points[indexB];
                    Vector2 pointA = xfA.TransformPoint(localPointA);
                    Vector2 pointB = xfB.TransformPoint(localPointB);
                    return Vector2.Dot(pointB - pointA, f.axis);
                }
            case SeparationType.FaceA:
                {
                    Vector2 normal = xfA.q * f.axis;
                    Vector2 pointA = xfA.TransformPoint(f.localPoint);
                    Vector2 localPointB = f.proxyB.points[indexB];
                    Vector2 pointB = xfB.TransformPoint(localPointB);
                    return Vector2.Dot(pointB - pointA, normal);
                }
            case SeparationType.FaceB:
                {
                    Vector2 normal = xfB.q * f.axis;
                    Vector2 pointB = xfB.TransformPoint(f.localPoint);
                    Vector2 localPointA = f.proxyA.points[indexA];
                    Vector2 pointA = xfA.TransformPoint(localPointA);
                    return Vector2.Dot(pointA - pointB, normal);
                }
            default:
                throw new ArgumentOutOfRangeException("Invalid separation type");
        }
    }
    /// <summary>Compute the upper bound on time before two shapes penetrate. Time is represented as
    /// a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
    /// non-tunneling collisions. If you change the time interval, you should call this function
    /// again.</summary>
    public static TOIOutput TimeOfImpact(this ref TOIInput input)
    {
        TOIOutput output = new()
        {
            state = TOIState.Unknown,
            fraction = input.maxFraction
        };
        Sweep sweepA = input.sweepA;
        Sweep sweepB = input.sweepB;
        Debug.Assert(sweepA.q1.IsNormalized() && sweepA.q2.IsNormalized());
        Debug.Assert(sweepB.q1.IsNormalized() && sweepB.q2.IsNormalized());
        ShapeProxy proxyA = input.proxyA, proxyB = input.proxyB;
        float tMax = input.maxFraction;
        float totalRadius = proxyA.radius + proxyB.radius;
        float target = Math.Max(Box2D.LinearSlop, totalRadius - Box2D.LinearSlop);
        float tolerance = 0.25f * Box2D.LinearSlop;
        Debug.Assert(target > tolerance);
        float t1 = 0;
        const int k_maxIterations = 20;
        int MAX_POLYGON_VERTICES = Math.Max(proxyA.points.Length, proxyB.points.Length);
        int distanceIterations = 0;
        SimplexCache cache = new();
        DistanceInput distanceInput = new()
        {
            proxyA = input.proxyA,
            proxyB = input.proxyB,
            useRadii = false
        };
        while (true)
        {
            Transform xfA = sweepA.GetSweepTransform(t1);
            Transform xfB = sweepB.GetSweepTransform(t1);
            distanceInput.transformA = xfA;
            distanceInput.transformB = xfB;
            DistanceOutput distanceOutput = distanceInput.ShapeDistance(ref cache, null);
            distanceIterations++;
            if (distanceOutput.distance <= 0)
            {
                output.state = TOIState.Overlapped;
                output.fraction = 0;
                break;
            }
            if (distanceOutput.distance <= target + tolerance)
            {
                output.state = TOIState.Hit;
                Vector2 pA = Vector2.MulAdd(distanceOutput.pointA, proxyA.radius, distanceOutput.normal);
                Vector2 pB = Vector2.MulAdd(distanceOutput.pointB, -proxyB.radius, distanceOutput.normal);
                output.point = Vector2.Lerp(pA, pB, 0.5f);
                output.normal = distanceOutput.normal;
                output.fraction = t1;
                break;
            }
            SeparationFunction fcn = cache.MakeSeparationFunction(proxyA, ref sweepA, proxyB, ref sweepB, t1);

            bool done = false;
            float t2 = tMax;
            int pushBackIterations = 0;
            while (true)
            {
                float s2 = fcn.FindMinSeparation(out int indexA, out int indexB, t2);
                if (s2 > target + tolerance)
                {
                    output.state = TOIState.Separated;
                    output.fraction = tMax;
                    done = true;
                    break;
                }
                if (s2 > target - tolerance) { t1 = t2; break; }
                float s1 = fcn.EvaluateSeparation(indexA, indexB, t1);
                if (s1 < target - tolerance)
                {
                    output.state = TOIState.Failed;
                    output.fraction = t1;
                    done = true;
                    break;
                }
                if (s1 <= target + tolerance)
                {
                    output.state = TOIState.Hit;
                    Vector2 pA = Vector2.MulAdd(distanceOutput.pointA, proxyA.radius, distanceOutput.normal);
                    Vector2 pB = Vector2.MulAdd(distanceOutput.pointB, -proxyB.radius, distanceOutput.normal);
                    output.point = Vector2.Lerp(pA, pB, 0.5f);
                    output.fraction = t1;
                    done = true;
                    break;
                }
                int rootIterationCount = 0;
                float a1 = t1, a2 = t2;
                while (rootIterationCount < 50)
                {
                    float t;
                    if ((rootIterationCount & 1) == 1)
                        t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
                    else t = 0.5f * (a1 + a2);
                    rootIterationCount++;
                    float s = fcn.EvaluateSeparation(indexA, indexB, t);
                    if (Math.Abs(s - target) < tolerance) { t2 = t; break; }
                    if (s > target) { a1 = t; s1 = s; }
                    else { a2 = t; s2 = s; }
                }
                pushBackIterations++;
                if (pushBackIterations == MAX_POLYGON_VERTICES) break;
            }
            if (done) break;
            if (distanceIterations == k_maxIterations)
            {
                output.state = TOIState.Failed;
                Vector2 pA = Vector2.MulAdd(distanceOutput.pointA, proxyA.radius, distanceOutput.normal);
                Vector2 pB = Vector2.MulAdd(distanceOutput.pointB, -proxyB.radius, distanceOutput.normal);
                output.point = Vector2.Lerp(pA, pB, 0.5f);
                output.normal = distanceOutput.normal;
                output.fraction = t1;
                break;
            }
        }
        return output;
    }
}
