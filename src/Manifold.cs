using System;
using System.Diagnostics;

namespace Box2D;

public static partial class Geometry
{
    public static Polygon MakeCapsule(Vector2 p1, Vector2 p2, float radius)
    {
        Polygon shape = new() { vertices = [p1, p2], centroid = Vector2.Lerp(p1, p2, 0.5f), radius = radius };
        Vector2 d = p2 - p1;
        Debug.Assert(d.LengthSquared() > Box2D.FLT_EPSILON);
        Vector2 axis = d.Normalize();
        Vector2 normal = axis.RightPerp();
        shape.normals = [normal, -normal];
        return shape;
    }
}

public static class Collision
{
    static ushort B2_MAKE_ID(int A, int B) => (ushort)((A << 8) | (byte)B);
    ///<summary>Compute the contact manifold between two circles</summary>
    public static Manifold CollideCircles(Circle circleA, Transform xfA, Circle circleB, Transform xfB)
    {
        Manifold manifold = new();
        Transform xf = Transform.InvMulTransforms(xfA, xfB);
        Vector2 pointA = circleA.center;
        Vector2 pointB = xf.TransformPoint(circleB.center);
        Vector2 normal = (pointB - pointA).GetLengthAndNormalize(out float distance);
        float radiusA = circleA.radius, radiusB = circleB.radius;
        float separation = distance - radiusA - radiusB;
        if (separation > Box2D.SpeculativeDistance) return manifold;
        Vector2 cA = Vector2.MulAdd(pointA, radiusA, normal);
        Vector2 cB = Vector2.MulAdd(pointB, -radiusB, normal);
        Vector2 contactPointA = Vector2.Lerp(cA, cB, 0.5f);
        manifold.normal = xfA.q * normal;
        ref ManifoldPoint mp = ref manifold.point0;
        mp.anchorA = xfA.q * contactPointA;
        mp.anchorB = mp.anchorA + (xfA.p - xfB.p);
        mp.point = mp.anchorA + xfA.p;
        mp.separation = separation;
        mp.id = 0;
        manifold.pointCount = 1;
        return manifold;
    }

    ///<summary>Compute the contact manifold between a capsule and circle</summary>
    public static Manifold CollideCapsuleAndCircle(Capsule capsuleA, Transform xfA, Circle circleB, Transform xfB)
    {
        Manifold manifold = new();
        Transform xf = Transform.InvMulTransforms(xfA, xfB);
        Vector2 pB = xf.TransformPoint(circleB.center);
        Vector2 p1 = capsuleA.center1, p2 =capsuleA.center2;
        Vector2 e = p2 - p1;
        Vector2 pA;
        float s1 = Vector2.Dot(pB - p1, e);
        float s2 = Vector2.Dot(p2 - pB, e);
        if (s1 < 0) pA = p1;
        else if (s2 < 0) pA = p2;
        else
        {
            float s = s1 / Vector2.Dot(e, e);
            pA = Vector2.MulAdd(p1, s, e);
        }
        Vector2 normal = (pB - pA).GetLengthAndNormalize(out float distance);
        float radiusA = capsuleA.radius;
        float radiusB = circleB.radius;
        float separation = distance - radiusA - radiusB;
        if (separation > Box2D.SpeculativeDistance) return manifold;
        Vector2 cA = Vector2.MulAdd(pA, radiusA, normal);
        Vector2 cB = Vector2.MulAdd(pB, -radiusB, normal);
        Vector2 contactPointA = Vector2.Lerp(cA, cB, 0.5f);
        manifold.normal = xfA.q * normal;
        ref ManifoldPoint mp = ref manifold.point0;
        mp.anchorA = xfA.q * contactPointA;
        mp.anchorB = mp.anchorA + (xfA.p - xfB.p);
        mp.point = xfA.p - mp.anchorA;
        mp.separation = separation;
        mp.id = 0;
        manifold.pointCount = 1;
        return manifold;
    }
    ///<summary>Compute the contact manifold between an segment and a circle</summary>
    public static Manifold CollideSegmentAndCircle(Segment segmentA, Transform xfA, Circle circleB, Transform xfB)
    {
        Capsule capsuleA = new() { center1 = segmentA.point1, center2 = segmentA.point2, radius = 0 };
        return CollideCapsuleAndCircle(capsuleA, xfA, circleB, xfB);
    }
    ///<summary>Compute the contact manifold between a polygon and a circle</summary>
    public static Manifold CollidePolygonAndCircle(Polygon polygonA, Transform xfA, Circle circleB, Transform xfB)
    {
        Manifold manifold = new();
        Transform xf = Transform.InvMulTransforms(xfA, xfB);
        Vector2 center = xf.TransformPoint(circleB.center);
        float radiusA = polygonA.radius;
        float radiusB = circleB.radius;
        float radius = radiusA + radiusB;
        int normalIndex = 0;
        float separation = -float.MaxValue;
        int vertexCount = polygonA.vertices.Length;
        Vector2[] vertices = polygonA.vertices;
        Vector2[] normals = polygonA.normals;
        for (int i = 0; i < vertexCount; i++)
        {
            float s = Vector2.Dot(normals[i], center - vertices[i]);
            if (s > separation) { separation = s; normalIndex = i; }
        }
        if (separation > radius + Box2D.SpeculativeDistance) return manifold;
        int vertIndex1 = normalIndex;
        int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
        Vector2 v1 = vertices[vertIndex1], v2 = vertices[vertIndex2];
        float u1 = Vector2.Dot(center - v1, v2 - v1);
        float u2 = Vector2.Dot(center - v2, v1 - v2);
        if (u1 < 0 && separation > Box2D.FLT_EPSILON)
        {
            Vector2 normal = (center - v1).Normalize();
            separation = Vector2.Dot(center - v1, normal);
            if (separation > radius + Box2D.SpeculativeDistance) return manifold;
            Vector2 cA = Vector2.MulAdd(v1, radiusA, normal);
            Vector2 cB = Vector2.MulSub(center, radiusB, normal);
            Vector2 contactPointA = Vector2.Lerp(cA, cB, 0.5f);
            manifold.normal = xfA.q * normal;
            ref ManifoldPoint mp = ref manifold.point0;
            mp.anchorA = xfA.q * contactPointA;
            mp.anchorB = mp.anchorA + (xfA.p - xfB.p);
            mp.point = xfA.p = mp.anchorA;
            mp.separation = Vector2.Dot(cB - cA, normal);
            mp.id = 0;
            manifold.pointCount = 1;
        }
        else if (u2 < 0 && separation > Box2D.FLT_EPSILON)
        {
            Vector2 normal = (center - v2).Normalize();
            separation = Vector2.Dot(center - v2, normal);
            if (separation > radius + Box2D.SpeculativeDistance) return manifold;
            Vector2 cA = Vector2.MulAdd(v2, radiusA, normal);
            Vector2 cB = Vector2.MulSub(center, radiusB, normal);
            Vector2 contactPointA = Vector2.Lerp(cA, cB, 0.5f);
            manifold.normal = xfA.q * normal;
            ref ManifoldPoint mp = ref manifold.point0;
            mp.anchorA = xfA.q * contactPointA;
            mp.anchorB = mp.anchorA + (xfA.p - xfB.p);
            mp.point = xfA.p + mp.anchorA;
            mp.separation = Vector2.Dot(cB - cA, normal);
            mp.id = 0;
            manifold.pointCount = 1;
        }
        else
        {
            Vector2 normal = normals[normalIndex];
            manifold.normal = xfA.q * normal;
            Vector2 cA = Vector2.MulAdd(center, radiusA - Vector2.Dot(center - v1, normal), normal);
            Vector2 cB = Vector2.MulSub(center, radiusB, normal);
            Vector2 contactPointA = Vector2.Lerp(cA, cB, 0.5f);
            ref ManifoldPoint mp = ref manifold.point0;
            mp.anchorA = xfA.q * contactPointA;
            mp.anchorB = mp.anchorA + (xfA.p - xfB.p);
            mp.point = xfA.p + mp.anchorA;
            mp.separation = separation - radius;
            mp.id = 0;
            manifold.pointCount = 1;
        }
        return manifold;
    }
    ///<summary>Compute the contact manifold between a capsule and circle</summary>
    public static Manifold CollideCapsules(Capsule capsuleA, Transform xfA, Capsule capsuleB, Transform xfB)
    {
        Vector2 origin = capsuleA.center1;
        Transform sfA = new(xfA.p + xfA.q * origin, xfA.q);
        Transform xf = Transform.InvMulTransforms(sfA, xfB);
        Vector2 p1 = Vector2.Zero;
        Vector2 q1 = capsuleA.center2 - origin;
        Vector2 p2 = xf.TransformPoint(capsuleB.center1);
        Vector2 q2 = xf.TransformPoint(capsuleB.center2);
        Vector2 d1 = q1 - p1;
        Vector2 d2 = q2 - p2;
        float dd1 = Vector2.Dot(d1, d1);
        float dd2 = Vector2.Dot(d2, d2);
        const float epsSqr = Box2D.FLT_EPSILON * Box2D.FLT_EPSILON;
        Debug.Assert(dd1 > epsSqr && dd2 > epsSqr);
        Vector2 r = p1 - p2;
        float rd1 = Vector2.Dot(r, d1);
        float rd2 = Vector2.Dot(r, d2);
        float d12 = Vector2.Dot(d1, d2);
        float denom = dd1 * dd2 - d12 * d12;
        float f1 = 0;
        if (denom != 0) f1 = Math.Clamp((d12 * rd2 - rd1 * dd2) / denom, 0, 1);
        float f2 = (d12 * f1 + rd2) / dd2;
        if (f2 < 0) { f2 = 0; f1 = Math.Clamp(-rd1 / dd1, 0, 1); }
        else if (f2 > 1) { f2 = 1; f1 = Math.Clamp((d12 - rd1) / dd1, 0, 1); }
        Vector2 closest1 = Vector2.MulAdd(p1, f1, d1);
        Vector2 closest2 = Vector2.MulAdd(p2, f2, d2);
        float distanceSquared = Vector2.DistanceSquared(closest1, closest2);
        Manifold manifold = new();
        float radiusA = capsuleA.radius, radiusB = capsuleB.radius;
        float radius = radiusA + radiusB;
        float maxDistance = radius + Box2D.SpeculativeDistance;
        if (distanceSquared > maxDistance * maxDistance) return manifold;
        float distance = MathF.Sqrt(distanceSquared);
        Vector2 u1 = d1.GetLengthAndNormalize(out float length1), u2 = d2.GetLengthAndNormalize(out float length2);
        float fp2 = Vector2.Dot(p2 - p1, u1);
        float fq2 = Vector2.Dot(q2 - p1, u1);
        bool outsideA = (fp2 < 0 && fq2 < 0) || (fp2 >= length1 && fq2 >= length1);
        float fp1 = Vector2.Dot(p1 - p2, u2);
        float fq1 = Vector2.Dot(q1 - p2, u2);
        bool outsideB = (fp1 < 0 && fq1 < 0) || (fp1 >= length2 && fq1 >= length2);
        if (!outsideA && !outsideB)
        {
            Vector2 normalA = new();
            float separationA = 0;
            {
                normalA = u1.LeftPerp();
                float ss1 = Vector2.Dot(p2 - p1, normalA);
                float ss2 = Vector2.Dot(q2 - p1, normalA);
                float s1p = ss1 < ss2 ? ss1 : ss2;
                float s1n = -ss1 < -ss2 ? -ss1 : -ss2;
                if (s1p > s1n) separationA = s1p;
                else
                {
                    separationA = s1n;
                    normalA = -normalA;
                }
            }
            Vector2 normalB = new();
            float separationB = 0;
            {
                normalB = u2.LeftPerp();
                float ss1 = Vector2.Dot(p1 - p2, normalB);
                float ss2 = Vector2.Dot(q1 - p2, normalB);
                float s1p = ss1 < ss2 ? ss1 : ss2;
                float s1n = -ss1 < -ss2 ? -ss1 : -ss2;
                if (s1p > s1n) separationB = s1p;
                else
                {
                    separationB = s1n;
                    normalB = -normalB;
                }
            }
            if (separationA + 0.1f * Box2D.LinearSlop >= separationB)
            {
                manifold.normal = normalA;
                Vector2 cp = p2, cq = q2;
                if (fp2 < 0 && fq2 > 0) cp = Vector2.Lerp(p2, q2, -fp2 / (fq2 - fp2));
                else if (fq2 < 0 && fp2 > 0) cq = Vector2.Lerp(q2, p2, -fq2 / (fq2 - fp2));
                if (fp2 > length1 && fq2 < length1) cp = Vector2.Lerp(q2, p2, (fp2 - length1) / (fp2 - fq2));
                else if (fq2 > length1 && fp2 < length1) cq = Vector2.Lerp(q2, p2, (fq2 - length1) / (fq2 - fp2));
                float sp = Vector2.Dot(cp - p1, normalA);
                float sq = Vector2.Dot(cq - p1, normalA);
                if (sp <= distance + Box2D.LinearSlop || sq <= distance + Box2D.LinearSlop)
                {
                    manifold.point0.anchorA = Vector2.MulAdd(cp, 0.5f * (radiusA - radiusB - sp), normalA);
                    manifold.point0.separation = sp - radius;
                    manifold.point0.id = B2_MAKE_ID(0, 0);
                    manifold.point1.anchorA = Vector2.MulAdd(cq, 0.5f * (radiusA - radiusB - sq), normalA);
                    manifold.point1.separation = sq - radius;
                    manifold.point1.id = B2_MAKE_ID(0, 1);
                    manifold.pointCount = 2;
                }
            }
            else
            {
                manifold.normal = -normalB;
                Vector2 cp = p1;
                Vector2 cq = q1;
                if (fp1 < 0 && fq1 > 0) cp = Vector2.Lerp(p1, q1, -fp1 / (fq1 - fp1));
                else if (fq1 < 0 && fp1 > 0) cq = Vector2.Lerp(q1, p1, -fq1 / (fp1 - fq1));
                if (fp1 > length2 && fq1 > length2) cp = Vector2.Lerp(p1, q1, (fp1 - length2) / (fp1 - fq1));
                else if (fq1 > length2 && fp1 > length2) cq = Vector2.Lerp(q1, p1, (fq1 - length2) / (fq1 - fp1));
                float sp = Vector2.Dot(cp - p2, normalB);
                float sq = Vector2.Dot(cq - p2, normalB);
                if (sp <= distance + Box2D.LinearSlop || sq <= distance + Box2D.LinearSlop)
                {
                    manifold.point0.anchorA = Vector2.MulAdd(cp, 0.5f * (radiusB - radiusA - sp), normalB);
                    manifold.point0.separation = sp - radius;
                    manifold.point0.id = B2_MAKE_ID(0, 0);
                    manifold.point1.anchorA = Vector2.MulAdd(cq, 0.5f * (radiusB - radiusA - sp), normalB);
                    manifold.point1.separation = sp - radius;
                    manifold.point1.id = B2_MAKE_ID(1, 0);
                    manifold.pointCount = 2;
                }
            }
        }
        if (manifold.pointCount == 0)
        {
            Vector2 normal = closest2 - closest1;
            if (Vector2.Dot(normal, normal) > epsSqr) normal = normal.Normalize();
            else normal = u1.LeftPerp();
            Vector2 c1 = Vector2.MulAdd(closest1, radiusA, normal);
            Vector2 c2 = Vector2.MulAdd(closest2, -radiusB, normal);
            int i1 = f1 == 0 ? 0 : 1;
            int i2 = f2 == 0 ? 0 : 1;
            manifold.normal = normal;
            manifold.point0.anchorA = Vector2.Lerp(c1, c2, 0.5f);
            manifold.point0.separation = MathF.Sqrt(distanceSquared) - radius;
            manifold.point0.id = B2_MAKE_ID(i1, i2);
            manifold.pointCount = 1;
        }
        manifold.normal = xfA.q * manifold.normal;
        if (manifold.pointCount > 0)
        {
            ref ManifoldPoint mp = ref manifold.point0;
            mp.anchorA = xfA.q * (mp.anchorA + origin);
            mp.anchorB = mp.anchorA + (xfA.p - xfB.p);
            mp.point = xfA.p + mp.anchorA;
        }
        if (manifold.pointCount > 1)
        {
            ref ManifoldPoint mp = ref manifold.point1;
            mp.anchorA = xfA.q * (mp.anchorA + origin);
            mp.anchorB = mp.anchorA + (xfA.p - xfB.p);
            mp.point = xfA.p + mp.anchorA;
        }
        return manifold;
    }
    ///<summary>Compute the contact manifold between an segment and a capsule</summary>
    public static Manifold CollideSegmentAndCapsule(Segment segmentA, Transform xfA, Capsule capsuleB, Transform xfB)
    {
        Capsule capsuleA = new() { center1 = segmentA.point1, center2 = segmentA.point2, radius = 0 };
        return CollideCapsules(capsuleA, xfA, capsuleB, xfB);
    }
    ///<summary>Compute the contact manifold between a polygon and capsule</summary>
    public static Manifold CollidePolygonAndCapsule(Polygon polygonA, Transform xfA, Capsule capsuleB, Transform xfB)
    {
        Polygon polyB = Geometry.MakeCapsule(capsuleB.center1, capsuleB.center2, capsuleB.radius);
        return CollidePolygons(polygonA, xfA, polyB, xfB);
    }
    /// <summary>Polygon clipper used to compute contact points when there are potentially two contact points. </summary>
    static Manifold ClipPolygons(Polygon polyA, Polygon polyB, int edgeA, int edgeB, bool flip)
    {
        Manifold manifold = new();
        Polygon poly1 = polyA, poly2 = polyB;
        int i11 = edgeA, i12 = edgeA + 1 < polyA.vertices.Length ? edgeA + 1 : 0;
        int i21 = edgeB, i22 = edgeB + 1 < polyB.vertices.Length ? edgeB + 1 : 0;
        if (flip) (poly1, poly2, i11, i12, i21, i22) = (poly2, poly1, i21, i22, i11, i12);
        Vector2 normal = poly1.normals[i11];
        Vector2 v11 = poly1.vertices[i11], v12 = poly1.vertices[i12];
        Vector2 v21 = poly2.vertices[i21], v22 = poly2.vertices[i22];
        Vector2 tangent = Vector2.CrossSV(1, normal);
        float lower1 = 0;
        float upper1 = Vector2.Dot(v12 - v11, tangent);
        float upper2 = Vector2.Dot(v21 - v11, tangent);
        float lower2 = Vector2.Dot(v22 - v11, tangent);
        if (upper2 < lower1 || upper1 < lower2) return manifold;
        Vector2 vLower = lower2 < lower1 && upper2 - lower2 > Box2D.FLT_EPSILON ?
            Vector2.Lerp(v22, v21, (lower1 - lower2) / (upper2 - lower2)) : v22;
        Vector2 vUpper = upper2 > upper1 && upper2 - lower2 > Box2D.FLT_EPSILON ?
            Vector2.Lerp(v22, v21, (upper1 - lower2) / (upper2 - lower2)) : v21;
        float separationLower = Vector2.Dot(vLower - v11, normal);
        float separationUpper = Vector2.Dot(vUpper - v11, normal);
        float r1 = poly1.radius, r2 = poly2.radius;
        vLower = Vector2.MulAdd(vLower, 0.5f * (r1 - r2 - separationLower), normal);
        vUpper = Vector2.MulAdd(vUpper, 0.5f * (r1 - r2 - separationUpper), normal);
        float radius = r1 + r2;
        if (!flip)
        {
            manifold.normal = normal;
            {
                manifold.point0.anchorA = vLower;
                manifold.point0.separation = separationLower - radius;
                manifold.point0.id = B2_MAKE_ID(i11, i22);
                manifold.pointCount++;
            }
            {
                manifold.point1.anchorA = vUpper;
                manifold.point1.separation = separationUpper - radius;
                manifold.point1.id = B2_MAKE_ID(i12, i21);
                manifold.pointCount++;
            }
        }
        else
        {
            manifold.normal = -normal;
            {
                manifold.point0.anchorA = vUpper;
                manifold.point0.separation = separationUpper - radius;
                manifold.point0.id = B2_MAKE_ID(i21, i12);
                manifold.pointCount++;
            }
            {
                manifold.point1.anchorA = vLower;
                manifold.point1.separation = separationLower - radius;
                manifold.point1.id = B2_MAKE_ID(i22, i11);
                manifold.pointCount++;
            }
        }
        return manifold;
    }
    static float FindMaxSeparation(out int edgeIndex, Polygon poly1, Polygon poly2)
    {
        int count1 = poly1.vertices.Length, count2 = poly2.vertices.Length;
        Vector2[] n1s = poly1.normals, v1s = poly1.vertices, v2s = poly2.vertices;
        int bestIndex = 0;
        float maxSeparation = -float.MaxValue;
        for (int i = 0; i < count1; i++)
        {
            Vector2 n = n1s[i], v1 = v1s[i];
            float si = float.MaxValue;
            for (int j = 0; j < count2; j++)
            {
                float sij = Vector2.Dot(n, v2s[j] - v1);
                if (sij < si) si = sij;
            }
            if (si > maxSeparation) { maxSeparation = si; bestIndex = i; }
        }
        edgeIndex = bestIndex;
        return maxSeparation;
    }
    ///<summary>Compute the contact manifold between two polygons</summary>
    public static Manifold CollidePolygons(Polygon polygonA, Transform xfA, Polygon polygonB, Transform xfB)
    {
        Vector2 origin = polygonA.vertices[0];
        Transform sfA = new(xfA.p + xfA.q * origin, xfA.q);
        Transform xf = Transform.InvMulTransforms(sfA, xfB);
        Polygon localPolyA = new()
        {
            radius = polygonA.radius,
            vertices = new Vector2[polygonA.vertices.Length],
            normals = new Vector2[polygonA.normals.Length],
        };
        localPolyA.vertices[0] = Vector2.Zero;
        localPolyA.normals[0] = polygonA.normals[0];
        for (int i = 1; i < localPolyA.vertices.Length; i++)
        {
            localPolyA.vertices[i] = polygonA.vertices[i] - origin;
            localPolyA.normals[i] = polygonA.normals[i];
        }
        Polygon localPolyB = new()
        {
            radius = polygonB.radius,
            vertices = new Vector2[polygonB.vertices.Length],
            normals = new Vector2[polygonB.normals.Length],
        };
        for (int i = 0; i < localPolyB.vertices.Length; i++)
        {
            localPolyB.vertices[i] = xf.TransformPoint(polygonB.vertices[i]);
            localPolyB.normals[i] = xf.q * polygonB.normals[i];
        }
        float separationA = FindMaxSeparation(out int edgeA, localPolyA, localPolyB);
        float separationB = FindMaxSeparation(out int edgeB, localPolyB, localPolyA);
        float radius = localPolyA.radius + localPolyB.radius;
        if (separationA > Box2D.SpeculativeDistance + radius || separationB > Box2D.SpeculativeDistance + radius)
            return new();
        bool flip;
        if (separationA >= separationB)
        {
            flip = false;
            Vector2 searchDirection = localPolyA.normals[edgeA];
            int count = localPolyB.vertices.Length;
            Vector2[] normals = localPolyB.normals;
            edgeB = 0;
            float minDot = float.MaxValue;
            for (int i = 0; i < count; i++)
            {
                float dot = Vector2.Dot(searchDirection, normals[i]);
                if (dot < minDot) { minDot = dot; edgeB = i; }
            }
        }
        else
        {
            flip = true;
            Vector2 searchDirection = localPolyB.normals[edgeB];
            int count = localPolyA.vertices.Length;
            Vector2[] normals = localPolyA.normals;
            edgeA = 0;
            float minDot = float.MaxValue;
            for (int i = 0; i < count; i++)
            {
                float dot = Vector2.Dot(searchDirection, normals[i]);
                if (dot < minDot) { minDot = dot; edgeA = i; }
            }
        }
        Manifold manifold = new();
        if (separationA > 0.1f * Box2D.LinearSlop || separationB > 0.1f * Box2D.LinearSlop)
        {
            int i11 = edgeA, i12 = edgeA + 1 < localPolyA.vertices.Length ? edgeA + 1 : 0;
            int i21 = edgeB, i22 = edgeB + 1 < localPolyB.vertices.Length ? edgeB + 1 : 0;
            Vector2 v11 = localPolyA.vertices[i11], v12 = localPolyA.vertices[i12];
            Vector2 v21 = localPolyB.vertices[i21], v22 = localPolyB.vertices[i22];
            SegmentDistanceResult result = Distance.SegmentDistance(v11, v12, v21, v22);
            Debug.Assert(result.distanceSquared > 0);
            float distance = MathF.Sqrt(result.distanceSquared);
            float separation = distance - radius;
            if (distance - radius > Box2D.SpeculativeDistance) return manifold;
            manifold = ClipPolygons(localPolyA, localPolyB, edgeA, edgeB, flip);
            float minSeparation = float.MaxValue;
            if (manifold.pointCount > 0) minSeparation = Math.Min(minSeparation, manifold.point0.separation);
            if (manifold.pointCount > 1) minSeparation = Math.Min(minSeparation, manifold.point1.separation);
            if (separation + 0.1f * Box2D.LinearSlop < minSeparation)
            {
                if (result.fraction1 == 0 && result.fraction2 == 0)
                {
                    Vector2 normal = v21 - v11;
                    float invDistance = 1 / distance;
                    normal *= invDistance;
                    Vector2 c1 = Vector2.MulAdd(v11, localPolyA.radius, normal);
                    Vector2 c2 = Vector2.MulAdd(v21, -localPolyB.radius, normal);
                    manifold.normal = normal;
                    manifold.point0.anchorA = Vector2.Lerp(c1, c2, 0.5f);
                    manifold.point0.separation = distance - radius;
                    manifold.point0.id = B2_MAKE_ID(i11, i21);
                    manifold.pointCount = 1;
                }
                else if (result.fraction1 == 0 && result.fraction2 == 1)
                {
                    Vector2 normal = v22 - v11;
                    float invDistance = 1 / distance;
                    normal *= invDistance;
                    Vector2 c1 = Vector2.MulAdd(v11, localPolyA.radius, normal);
                    Vector2 c2 = Vector2.MulAdd(v22, -localPolyB.radius, normal);
                    manifold.normal = normal;
                    manifold.point0.anchorA = Vector2.Lerp(c1, c2, 0.5f);
                    manifold.point0.separation = distance - radius;
                    manifold.point0.id = B2_MAKE_ID(i11, i22);
                    manifold.pointCount = 1;
                }
                else if (result.fraction1 == 1 && result.fraction2 == 0)
                {
                    Vector2 normal = v21 - v12;
                    float invDistance = 1 / distance;
                    normal *= invDistance;
                    Vector2 c1 = Vector2.MulAdd(v12, localPolyA.radius, normal);
                    Vector2 c2 = Vector2.MulAdd(v21, -localPolyB.radius, normal);
                    manifold.normal = normal;
                    manifold.point0.anchorA = Vector2.Lerp(c1, c2, 0.5f);
                    manifold.point0.separation = distance - radius;
                    manifold.point0.id = B2_MAKE_ID(i12, i21);
                    manifold.pointCount = 1;
                }
                else if (result.fraction1 == 1 && result.fraction2 == 1)
                {
                    Vector2 normal = v22 - v12;
                    float invDistance = 1 / distance;
                    normal *= invDistance;
                    Vector2 c1 = Vector2.MulAdd(v12, localPolyA.radius, normal);
                    Vector2 c2 = Vector2.MulAdd(v22, -localPolyB.radius, normal);
                    manifold.normal = normal;
                    manifold.point0.anchorA = Vector2.Lerp(c1, c2, 0.5f);
                    manifold.point0.separation = distance - radius;
                    manifold.point0.id = B2_MAKE_ID(i12, i22);
                    manifold.pointCount = 1;
                }
            }
        }
        else manifold = ClipPolygons(localPolyA, localPolyB, edgeA, edgeB, flip);
        if (manifold.pointCount > 0)
        {
            manifold.normal = xfA.q * manifold.normal;
            ref ManifoldPoint mp = ref manifold.point0;
            mp.anchorA = xfA.q * (mp.anchorA + origin);
            mp.anchorB = mp.anchorA + (xfA.p - xfB.p);
            mp.point = xfA.p + mp.anchorA;
            if (manifold.pointCount > 1)
            {
                mp = ref manifold.point1;
                mp.anchorA = xfA.q * (mp.anchorA + origin);
                mp.anchorB = mp.anchorA + (xfA.p - xfB.p);
                mp.point = xfA.p + mp.anchorA;
            }
        }
        return manifold;
    }
    ///<summary>Compute the contact manifold between an segment and a polygon</summary>
    public static Manifold CollideSegmentAndPolygon(Segment segmentA, Transform xfA, Polygon polygonB, Transform xfB)
    {
        Polygon polygonA = Geometry.MakeCapsule(segmentA.point1, segmentA.point2, 0);
        return CollidePolygons(polygonA, xfA, polygonB, xfB);
    }
    ///<summary>Compute the contact manifold between a chain segment and a circle</summary>
    public static Manifold CollideChainSegmentAndCircle(ChainSegment segmentA, Transform xfA, Circle circleB, Transform xfB)
    {
        Manifold manifold = new();
        Transform xf = Transform.InvMulTransforms(xfA, xfB);
        Vector2 pB = xf.TransformPoint(circleB.center);
        Vector2 p1 = segmentA.segment.point1, p2 = segmentA.segment.point2;
        Vector2 e = p2 - p1;
        float offset = Vector2.Dot(e.RightPerp(), pB - p1);
        if (offset < 0) return manifold;
        float u = Vector2.Dot(e, p2 - pB);
        float v = Vector2.Dot(e, pB - p1);
        Vector2 pA;
        if (v <= 0)
        {
            Vector2 prevEdge = p1 - segmentA.ghost1;
            float uPrev = Vector2.Dot(prevEdge, pB - p1);
            if (uPrev <= 0) return manifold;
            pA = p1;
        }
        else if (u <= 0)
        {
            Vector2 nextEdge = segmentA.ghost2 - p2;
            float vNext = Vector2.Dot(nextEdge, pB - p2);
            if (vNext > 0) return manifold;
            pA = p2;
        }
        else
        {
            float ee = Vector2.Dot(e, e);
            pA = new(u * p1.x + v * p2.x, u * p1.y + v * p2.y);
            pA = ee > 0 ? 1 / ee * pA : p1;
        }
        Vector2 normal = (pB - pA).GetLengthAndNormalize(out float distance);
        float radius = circleB.radius;
        float separation = distance - radius;
        if (separation > Box2D.SpeculativeDistance) return manifold;
        Vector2 cA = pA;
        Vector2 cB = Vector2.MulAdd(pB, -radius, normal);
        Vector2 contactPointA = Vector2.Lerp(cA, cB, 0.5f);
        manifold.normal = xfA.q * normal;
        ref ManifoldPoint mp = ref manifold.point0;
        mp.anchorA = xfA.q * contactPointA;
        mp.anchorB = mp.anchorA + (xfA.p - xfB.p);
        mp.point = xfA.p + mp.anchorA;
        mp.separation = separation;
        mp.id = 0;
        manifold.pointCount = 1;
        return manifold;
    }
    ///<summary>Compute the contact manifold between a chain segment and a capsule</summary>
    public static Manifold CollideChainSegmentAndCapsule(ChainSegment segmentA, Transform xfA, Capsule capsuleB, Transform xfB, ref SimplexCache cache)
    {
        Polygon polyB = Geometry.MakeCapsule(capsuleB.center1, capsuleB.center2, capsuleB.radius);
        return CollideChainSegmentAndPolygon(segmentA, xfA, polyB, xfB, ref cache);
    }
    static Manifold ClipSegments(Vector2 a1, Vector2 a2, Vector2 b1, Vector2 b2, Vector2 normal, float ra, float rb, ushort id1, ushort id2)
    {
        Manifold manifold = new();
        Vector2 tangent = normal.LeftPerp();
        float lower1 = 0;
        float upper1 = Vector2.Dot(a2 - a1, tangent);
        float upper2 = Vector2.Dot(b1 - a1, tangent);
        float lower2 = Vector2.Dot(b2 - a1, tangent);
        if (upper2 < lower1 || upper1 < lower2) return manifold;
        Vector2 vLower = lower2 < lower1 && upper2 - lower2 > Box2D.FLT_EPSILON ?
            Vector2.Lerp(b2, b1, (lower1 - lower2) / (upper2 - lower2)) : b2;
        Vector2 vUpper = upper2 > upper1 && upper2 - lower2 > Box2D.FLT_EPSILON ?
            Vector2.Lerp(b2, b1, (upper1 - lower2) / (upper2 - lower2)) : b1;
        float separationLower = Vector2.Dot(vLower - a1, normal);
        float separationUpper = Vector2.Dot(vUpper - a1, normal);
        vLower = Vector2.MulAdd(vLower, 0.5f * (ra - rb - separationLower), normal);
        vUpper = Vector2.MulAdd(vUpper, 0.5f * (ra - rb - separationUpper), normal);
        float radius = ra + rb;
        manifold.normal = normal;
        {
            ref ManifoldPoint cp = ref manifold.point0;
            cp.anchorA = vLower;
            cp.separation = separationLower - radius;
            cp.id = id1;
        }
        {
            ref ManifoldPoint cp = ref manifold.point1;
            cp.anchorA = vUpper;
            cp.separation = separationUpper - radius;
            cp.id = id2;
        }
        manifold.pointCount = 2;
        return manifold;
    }
    enum NormalType
    {
        /// <summary>This means the normal points in a direction that is non-smooth relative to a convex vertex and should be skipped</summary>
        Skip,
        /// <summary>This means the normal points in a direction that is smooth relative to a convex vertex and should be used for collision</summary>
        Admit,
        /// <summary>This means the normal is in a region of a concave vertex and should be snapped to the segment normal</summary>
        Snap
    }
    struct ChainSegmentParams
    {
        public Vector2 edge1, normal0, normal2;
        public bool convex1, convex2;
    }
    static NormalType ClassifyNormal(ChainSegmentParams params_, Vector2 normal)
    {
        const float sinTol = 0.01f;
        return Vector2.Dot(normal, params_.edge1) <= 0
            ? params_.convex1 ? Vector2.Cross(normal, params_.normal0) > sinTol ? NormalType.Skip : NormalType.Admit : NormalType.Snap
            : params_.convex2 ? Vector2.Cross(params_.normal2, normal) > sinTol ? NormalType.Skip : NormalType.Admit : NormalType.Snap;
    }
    ///<summary>Compute the contact manifold between a chain segment and a rounded polygon</summary>
    public static Manifold CollideChainSegmentAndPolygon(ChainSegment segmentA, Transform xfA, Polygon polygonB,
                                                       Transform xfB, ref SimplexCache cache)
    {
        Manifold manifold = new();
        Transform xf = Transform.InvMulTransforms(xfA, xfB);
        Vector2 centroidB = xf.TransformPoint(polygonB.centroid);
        float radiusB = polygonB.radius;
        Vector2 p1 = segmentA.segment.point1, p2 = segmentA.segment.point2;
        Vector2 edge1 = (p2 - p1).Normalize();
        const float convexTol = 0.01f;
        Vector2 edge0 = (p1 - segmentA.ghost1).Normalize();
        Vector2 edge2 = (segmentA.ghost2 - p2).Normalize();
        ChainSegmentParams smoothParams = new()
        {
            edge1 = edge1,
            normal0 = edge0.RightPerp(),
            convex1 = Vector2.Cross(edge0, edge1) >= convexTol,
            normal2 = edge2.RightPerp(),
            convex2 = Vector2.Cross(edge1, edge2) >= convexTol,
        };
        Vector2 normal1 = edge1.RightPerp();
        bool behind1 = Vector2.Dot(normal1, centroidB - p1) < 0, behind0 = true, behind2 = true;
        if (smoothParams.convex1) behind0 = Vector2.Dot(smoothParams.normal0, centroidB - p1) < 0;
        if (smoothParams.convex2) behind2 = Vector2.Dot(smoothParams.normal2, centroidB - p2) < 0;
        if (behind1 && behind0 && behind2) return manifold;
        int count = polygonB.vertices.Length;
        Vector2[] vertices = new Vector2[count], normals = new Vector2[count];
        for (int i = 0; i < count; i++)
        {
            vertices[i] = xf.TransformPoint(polygonB.vertices[i]);
            normals[i] = xf.q * polygonB.normals[i];
        }
        DistanceInput input = new()
        {
            proxyA = Distance.MakeProxy([segmentA.segment.point1], 0),
            proxyB = Distance.MakeProxy(vertices, 0),
            transformA = Transform.Identity,
            transformB = Transform.Identity,
            useRadii = false
        };
        DistanceOutput output = input.ShapeDistance(ref cache, null);
        if (output.distance > radiusB + Box2D.SpeculativeDistance) return manifold;
        Vector2 n0 = smoothParams.convex1 ? smoothParams.normal0 : normal1;
        Vector2 n2 = smoothParams.convex2 ? smoothParams.normal2 : normal1;
        int incidentIndex = -1, incidentNormal = -1;
        if (!behind1 && output.distance > 0.1f * Box2D.LinearSlop)
        {
            if (cache.count == 1)
            {
                Vector2 pA = output.pointA, pB = output.pointB;
                Vector2 normal = (pB - pA).Normalize();
                NormalType type = ClassifyNormal(smoothParams, normal);
                if (type == NormalType.Skip) return manifold;
                if (type == NormalType.Admit)
                {
                    manifold.normal = xfA.q * normal;
                    ref ManifoldPoint cp = ref manifold.point0;
                    cp.anchorA = xfA.q * pA;
                    cp.anchorB = cp.anchorA + (xfA.p - xfB.p);
                    cp.point = xfA.p + cp.anchorA;
                    cp.separation = output.distance - radiusB;
                    cp.id = B2_MAKE_ID(cache.indexA[0], cache.indexB[0]);
                    manifold.pointCount = 1;
                    return manifold;
                }
                incidentIndex = cache.indexB[0];
            }
            else
            {
                Debug.Assert(cache.count == 2);
                int ia1 = cache.indexA[0], ia2 = cache.indexA[1],
                    ib1 = cache.indexB[0], ib2 = cache.indexB[1];
                if (ia1 == ia2)
                {
                    Debug.Assert(ib1 != ib2);
                    Vector2 normalB = output.pointA - output.pointB;
                    float dot1 = Vector2.Dot(normalB, normals[ib1]);
                    float dot2 = Vector2.Dot(normalB, normals[ib2]);
                    int ib = dot1 > dot2 ? ib1 : ib2;
                    normalB = normals[ib];
                    NormalType type = ClassifyNormal(smoothParams, -normalB);
                    if (type == NormalType.Skip) return manifold;
                    if (type == NormalType.Admit)
                    {
                        ib1 = ib;
                        ib2 = ib < count - 1 ? ib + 1 : 0;
                        Vector2 b1 = vertices[ib1], b2 = vertices[ib2];
                        dot1 = Vector2.Dot(normalB, p1 - b1);
                        dot2 = Vector2.Dot(normalB, p2 - b1);
                        if (dot1 < dot2)
                        {
                            if (Vector2.Dot(n0, normalB) < Vector2.Dot(normal1, normalB)) return manifold;
                        }
                        else if (Vector2.Dot(n2, normalB) < Vector2.Dot(normal1, normalB)) return manifold;
                        manifold = ClipSegments(b1, b2, p1, p2, normalB, radiusB, 0, B2_MAKE_ID(ib1, 1), B2_MAKE_ID(ib2, 0));
                        Debug.Assert(manifold.pointCount == 0 || manifold.pointCount == 2);
                        if (manifold.pointCount == 2)
                        {
                            manifold.normal = xfA.q * -normalB;
                            manifold.point0.anchorA = xfA.q * manifold.point0.anchorA;
                            manifold.point1.anchorA = xfA.q * manifold.point1.anchorA;
                            Vector2 pAB = xfA.p - xfB.p;
                            manifold.point0.anchorB = manifold.point0.anchorA + pAB;
                            manifold.point1.anchorB = manifold.point1.anchorA + pAB;
                            manifold.point0.point = xfA.p + manifold.point0.anchorA;
                            manifold.point1.point = xfA.p + manifold.point1.anchorA;
                        }
                        return manifold;
                    }
                    incidentNormal = ib;
                }
                else
                {
                    float dot1 = Vector2.Dot(normal1, vertices[ib1] - p1);
                    float dot2 = Vector2.Dot(normal1, vertices[ib2] - p2);
                    incidentIndex = dot1 < dot2 ? ib1 : ib2;
                }
            }
        }
        else
        {
            float edgeSeparation = float.MaxValue;
            for (int i = 0; i < count; i++)
            {
                float s = Vector2.Dot(normal1, vertices[i] - p1);
                if (s < edgeSeparation) { edgeSeparation = s; incidentIndex = i; }
            }
            if (smoothParams.convex1)
            {
                float s0 = float.MaxValue;
                for (int i = 0; i < count; i++)
                {
                    float s = Vector2.Dot(smoothParams.normal0, vertices[i] - p1);
                    if (s < s0) s0 = s;
                }
                if (s0 > edgeSeparation) { edgeSeparation = s0; incidentIndex = -1; }
            }
            if (smoothParams.convex2)
            {
                float s2 = float.MaxValue;
                for (int i = 0; i < count; i++)
                {
                    float s = Vector2.Dot(smoothParams.normal2, vertices[i] - p2);
                    if (s < s2) s2 = s;
                }
                if (s2 > edgeSeparation) { edgeSeparation = s2; incidentIndex = -1; }
            }
            float polygonSeparation = -float.MaxValue;
            int referenceIndex = -1;
            for (int i = 0; i < count; i++)
            {
                Vector2 n = normals[i];
                NormalType type = ClassifyNormal(smoothParams, -n);
                if (type != NormalType.Admit) continue;
                Vector2 p = vertices[i];
                float s = Math.Min(Vector2.Dot(n, p2 - p), Vector2.Dot(n, p1 - p));
                if (s > polygonSeparation) { polygonSeparation = s; incidentIndex = i; }
            }
            if (polygonSeparation > edgeSeparation)
            {
                int ia1 = referenceIndex;
                int ia2 = ia1 < count - 1 ? ia1 + 1 : 0;
                Vector2 a1 = vertices[ia1];
                Vector2 a2 = vertices[ia2];
                Vector2 n = normals[ia1];
                float dot1 = Vector2.Dot(n, p1 - a1);
                float dot2 = Vector2.Dot(n, p2 - a1);
                if (dot1 < dot2)
                {
                    if (Vector2.Dot(n0, n) < Vector2.Dot(normal1, n)) return manifold;
                }
                else if (Vector2.Dot(n2, n) < Vector2.Dot(normal1, n)) return manifold;
                manifold = ClipSegments(a1, a2, p1, p2, normals[ia1], radiusB, 0, B2_MAKE_ID(ia1, 1), B2_MAKE_ID(ia2, 0));
                Debug.Assert(manifold.pointCount == 0 || manifold.pointCount == 2);
                if (manifold.pointCount == 2)
                {
                    manifold.normal = xfA.q * -normals[ia1];
                    manifold.point0.anchorA = xfA.q * manifold.point0.anchorA;
                    manifold.point1.anchorA = xfA.q * manifold.point1.anchorA;
                    Vector2 pAB = xfA.p - xfB.p;
                    manifold.point0.anchorB = manifold.point0.anchorA + pAB;
                    manifold.point1.anchorB = manifold.point1.anchorA + pAB;
                    manifold.point0.point = xfA.p + manifold.point0.anchorA;
                    manifold.point1.point = xfA.p + manifold.point1.anchorA;
                }
                return manifold;
            }
            if (incidentIndex == -1) return manifold;
        }
        Debug.Assert(incidentNormal != -1 || incidentIndex != -1);
        {
            Vector2 b1, b2;
            int ib1, ib2;
            if (incidentNormal != -1)
            {
                ib1 = incidentNormal;
                ib2 = ib1 < count - 1 ? ib1 + 1 : 0;
                b1 = vertices[ib1];
                b2 = vertices[ib2];
            }
            else
            {
                int i2 = incidentIndex;
                int i1 = i2 > 0 ? i2 - 1 : count - 1;
                float d1 = Vector2.Dot(normal1, normals[i1]);
                float d2 = Vector2.Dot(normal1, normals[i2]);
                if (d1 < d2)
                {
                    ib1 = i1; ib2 = i2;
                    b1 = vertices[ib1];
                    b2 = vertices[ib2];
                }
                else
                {
                    ib1 = i2; ib2 = i2 < count - 1 ? i2 + 1 : 0;
                    b1 = vertices[ib1];
                    b2 = vertices[ib2];
                }
            }
            manifold = ClipSegments(p1, p2, b1, b2, normal1, 0, radiusB, B2_MAKE_ID(0, ib2), B2_MAKE_ID(1, ib1));
        }
        Debug.Assert(manifold.pointCount == 0 || manifold.pointCount == 2);
        if (manifold.pointCount == 2)
        {
            manifold.normal = xfA.q * -manifold.normal;
            manifold.point0.anchorA = xfA.q * manifold.point0.anchorA;
            manifold.point1.anchorA = xfA.q * manifold.point1.anchorA;
            Vector2 pAB = xfA.p - xfB.p;
            manifold.point0.anchorB = manifold.point0.anchorA + pAB;
            manifold.point1.anchorB = manifold.point1.anchorA + pAB;
            manifold.point0.point = xfA.p + manifold.point0.anchorA;
            manifold.point1.point = xfA.p + manifold.point1.anchorA;
        }
        return manifold;
    }
}