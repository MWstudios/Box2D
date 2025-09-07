using System;
using System.Diagnostics;
using System.Collections.Generic;

namespace Box2D;

/// <summary>Low level ray cast input data</summary>
public struct RayCastInput
{
    /// <summary>Start point of the ray cast</summary>
    public Vector2 origin;
    /// <summary>Translation of the ray cast</summary>
    public Vector2 translation;
    /// <summary>The maximum fraction of the translation to consider, typically 1</summary>
    public float maxFraction;
}
/// <summary>A distance proxy is used by the GJK algorithm. It encapsulates any shape.</summary>
public class ShapeProxy
{
    /// <summary>The point cloud</summary>
    public Vector2[] points = null;
    /// <summary>The external radius of the point cloud. May be zero.</summary>
    public float radius;
    //public ShapeProxy(ShapeProxy other) { points = other.points?.ToArray(); radius = other.radius;¨}
}
/// <summary>Low level shape cast input in generic form. This allows casting an arbitrary point
/// cloud wrap with a radius. For example, a circle is a single point with a non-zero radius.
/// A capsule is two points with a non-zero radius. A box is four points with a zero radius.</summary>
public struct ShapeCastInput
{
    /// <summary>A generic shape</summary>
    public ShapeProxy proxy;
    /// <summary>The translation of the shape cast</summary>
    public Vector2 translation;
    /// <summary>The maximum fraction of the translation to consider, typically 1</summary>
    public float maxFraction;
    /// <summary>Allow shape cast to encroach when initially touching. This only works if the radius is greater than zero.</summary>
    public bool canEncroach;
}
/// <summary>Low level ray cast or shape-cast output data. Returns a zero fraction and normal in the case of initial overlap.</summary>
public struct CastOutput
{
    /// <summary>The surface normal at the hit point</summary>
    public Vector2 normal;
    /// <summary>The surface hit point</summary>
    public Vector2 point;
    /// <summary>The fraction of the input translation at collision</summary>
    public float fraction;
    /// <summary>The number of iterations used</summary>
    public int iterations;
    /// <summary>Did the cast hit?</summary>
    public bool hit;
}
/// <summary>This holds the mass data computed for a shape.</summary>
public struct MassData
{
    /// <summary>The mass of the shape, usually in kilograms.</summary>
    public float mass;
    /// <summary>The position of the shape's centroid relative to the shape's origin.</summary>
    public Vector2 center;
    /// <summary>The rotational inertia of the shape about the shape center.</summary>
    public float rotationalInertia;
}
/// <summary>A solid circle</summary>
public record Circle : IShape
{
    /// <summary>The local center</summary>
    public Vector2 center;
    /// <summary>The radius</summary>
    public float radius;
    public float GetRadius() => radius;
    ///<summary>Compute the bounding box of a transformed circle</summary>
    public AABB ComputeAABB(Transform xf)
    {
        Vector2 p = xf.TransformPoint(center);
        return new(new(p.x - radius, p.y - radius), new(p.x + radius, p.y + radius));
    }
    public Vector2 GetCentroid() => center;
    public float GetPerimeter() => 2 * MathF.PI * radius;
    public float GetProjectedPerimeter(Vector2 line) => 2 * radius;
    ///<summary>Compute mass properties of a circle</summary>
    public MassData ComputeMass(float density)
    {
        float rr = radius * radius, mass = density * MathF.PI * rr;
        return new()
        {
            mass = mass,
            center = center,
            rotationalInertia = mass * 0.5f * rr
        };
    }
    public ShapeExtent ComputeExtent(Vector2 localCenter) => new() { minExtent = radius, maxExtent = Vector2.Distance(center, localCenter) + radius };
    ///<summary>Ray cast versus circle shape in local space.</summary>
    public CastOutput RayCast(ref RayCastInput input)
    {
        Debug.Assert(Geometry.IsValidRay(ref input));
        Vector2 p = center;
        CastOutput output = new();
        Vector2 s = input.origin - p;
        float r = radius;
        float rr = r * r;
        Vector2 d = input.translation.GetLengthAndNormalize(out float length);
        if (length == 0)
        {
            if (s.LengthSquared() < r)
            {
                output.point = input.origin;
                output.hit = true;
            }
            return output;
        }
        float t = -Vector2.Dot(s, d);
        Vector2 c = Vector2.MulAdd(s, t, d);
        float cc = Vector2.Dot(c, c);
        if (cc > rr) return output;
        float h = MathF.Sqrt(rr - cc);
        float fraction = t - h;
        if (fraction < 0 || input.maxFraction * length < fraction)
        {
            if (s.LengthSquared() < rr)
            {
                output.point = input.origin;
                output.hit = true;
            }
            return output;
        }
        Vector2 hitPoint = Vector2.MulAdd(s, fraction, d);
        output.fraction = fraction / length;
        output.normal = hitPoint.Normalize();
        output.point = Vector2.MulAdd(p, radius, output.normal);
        output.hit = true;
        return output;
    }
    ///<summary>Shape cast versus a circle.</summary>
    public CastOutput ShapeCast(ref ShapeCastInput input)
    {
        ShapeCastPairInput pairInput = new()
        {
            proxyA = Distance.MakeProxy([center], radius),
            proxyB = input.proxy,
            transformA = Transform.Identity,
            transformB = Transform.Identity,
            translationB = input.translation,
            maxFraction = input.maxFraction,
            canEncroach = input.canEncroach
        };
        return pairInput.ShapeCast();
    }
    public PlaneResult CollideMover(ref Capsule mover)
    {
        DistanceInput distanceInput = new()
        {
            proxyA = Distance.MakeProxy([center], 0),
            proxyB = Distance.MakeProxy([mover.center1, mover.center2], 2),
            transformA = Transform.Identity,
            transformB = Transform.Identity,
            useRadii = false,
        };
        float totalRadius = mover.radius + radius;
        SimplexCache cache = new();
        DistanceOutput distanceOutput = distanceInput.ShapeDistance(ref cache, null);
        if (distanceOutput.distance <= totalRadius)
        {
            Plane plane = new(distanceOutput.normal, totalRadius - distanceOutput.distance);
            return new() { plane = plane, point = distanceOutput.pointA, hit = true };
        }
        return new();
    }
    public ShapeProxy MakeProxy() => Distance.MakeProxy([center], radius);
    ///<summary>Test a point for overlap with a circle in local space</summary>
    public bool TestPoint(Vector2 point) => Vector2.DistanceSquared(point, center) <= radius * radius;
}
/// <summary>A solid capsule can be viewed as two semicircles connected
/// by a rectangle.</summary>
public record Capsule : IShape
{
    /// <summary>Local center of the first semicircle</summary>
    public Vector2 center1;
    /// <summary>Local center of the second semicircle</summary>
    public Vector2 center2;
    /// <summary>The radius of the semicircles</summary>
    public float radius;
    public float GetRadius() => radius;
    ///<summary>Compute the bounding box of a transformed capsule</summary>
    public AABB ComputeAABB(Transform xf)
    {
        Vector2 v1 = xf.TransformPoint(center1), v2 = xf.TransformPoint(center2);
        Vector2 r = new(radius, radius);
        Vector2 lower = Vector2.Min(v1, v2) - r, upper = Vector2.Max(v1, v2) + r;
        return new(lower, upper);
    }
    public Vector2 GetCentroid() => Vector2.Lerp(center1, center2, 0.5f);
    public float GetPerimeter() => 2 * Vector2.Distance(center1, center2) + 2 * MathF.PI * radius;
    public float GetProjectedPerimeter(Vector2 line) => Math.Abs(Vector2.Dot(center2 - center1, line)) + 2 * radius;
    ///<summary>Compute mass properties of a capsule</summary>
    public MassData ComputeMass(float density)
    {
        float rr = radius * radius;
        Vector2 p1 = center1, p2 = center2;
        float ll = Vector2.DistanceSquared(p1, p2), length = MathF.Sqrt(ll);
        float circleMass = density * (MathF.PI * radius * radius);
        float boxMass = density * (2 * radius * radius);
        MassData massData = new()
        {
            mass = circleMass + boxMass,
            center = new(0.5f * (p1.x + p2.x), 0.5f * (p1.y + p2.y))
        };
        float lc = 4 * radius / (3 * MathF.PI);
        float h = 0.5f * length;
        float circleInertia = circleMass * (0.5f * rr + h * h + 2 * h * lc);
        float boxInertia = boxMass * (4 * rr + ll) / 12;
        massData.rotationalInertia = circleInertia + boxInertia;
        return massData;
    }
    public ShapeExtent ComputeExtent(Vector2 localCenter) => new()
    {
        minExtent = radius,
        maxExtent = MathF.Sqrt(Math.Max(Vector2.DistanceSquared(center1, localCenter), Vector2.DistanceSquared(center2, localCenter))) + radius
    };
    ///<summary>Ray cast versus capsule shape in local space.</summary>
    public CastOutput RayCast(ref RayCastInput input)
    {
        Debug.Assert(Geometry.IsValidRay(ref input));
        CastOutput output = new();
        Vector2 v1 = center1, v2 = center2;
        Vector2 e = v2 - v1;
        Vector2 a = e.GetLengthAndNormalize(out float capsuleLength);
        if (capsuleLength < Box2D.FLT_EPSILON)
        {
            Circle circle = new() { center = v1, radius = radius };
            return circle.RayCast(ref input);
        }
        Vector2 p1 = input.origin;
        Vector2 d = input.translation;
        Vector2 q = p1 - v1;
        float qa = Vector2.Dot(q, a);
        Vector2 qp = Vector2.MulAdd(q, -qa, a);
        if (Vector2.Dot(qp, qp) < radius * radius)
        {
            if (qa < 0)
            {
                Circle circle = new() { center = v1, radius = radius };
                return circle.RayCast(ref input);
            }
            if (qa > capsuleLength)
            {
                Circle circle = new() { center = v2, radius = radius };
                return circle.RayCast(ref input);
            }
            output.point = input.origin;
            output.hit = true;
            return output;
        }
        Vector2 n = new(a.y, -a.x);
        Vector2 u = d.GetLengthAndNormalize(out float rayLength);
        float den = -a.x * u.y + u.x * a.y;
        if (-Box2D.FLT_EPSILON < den && den < Box2D.FLT_EPSILON) return output;
        Vector2 b1 = Vector2.MulSub(q, radius, n), b2 = Vector2.MulAdd(q, radius, n);
        float invDen = 1 / den;
        float s21 = (a.x * b1.y - b1.x * a.y) * invDen;
        float s22 = (a.x * b2.y - b2.x * a.y) * invDen;
        float s2;
        Vector2 b;
        if (s21 < s22)
        {
            s2 = s21;
            b = b1;
        }
        else
        {
            s2 = s22;
            b = b2;
            n = -n;
        }
        if (s2 < 0 || input.maxFraction * rayLength < s2) return output;
        float s1 = (-b.x * u.y + u.x * b.y) * invDen;
        if (s1 < 0)
        {
            Circle circle = new() { center = v1, radius = radius };
            return circle.RayCast(ref input);
        }
        else if (capsuleLength < s1)
        {
            Circle circle = new() { center = v2, radius = radius };
            return circle.RayCast(ref input);
        }
        else
        {
            output.fraction = s2 / rayLength;
            output.point = Vector2.Lerp(v1, v2, s1 / capsuleLength) + radius * n;
            output.normal = n;
            output.hit = true;
            return output;
        }
    }
    ///<summary>Shape cast versus a capsule.</summary>
    public CastOutput ShapeCast(ref ShapeCastInput input)
    {
        ShapeCastPairInput pairInput = new()
        {
            proxyA = Distance.MakeProxy([center1, center2], radius),
            proxyB = input.proxy,
            transformA = Transform.Identity,
            transformB = Transform.Identity,
            translationB = input.translation,
            maxFraction = input.maxFraction,
            canEncroach = input.canEncroach
        };
        return pairInput.ShapeCast();
    }
    public PlaneResult CollideMover(ref Capsule mover)
    {
        DistanceInput distanceInput = new()
        {
            proxyA = Distance.MakeProxy([center1, center2], 0),
            proxyB = Distance.MakeProxy([center1, center2], mover.radius),
            transformA = Transform.Identity,
            transformB = Transform.Identity,
            useRadii = false,
        };
        float totalRadius = mover.radius + radius;
        SimplexCache cache = new();
        DistanceOutput distanceOutput = distanceInput.ShapeDistance(ref cache, null);
        if (distanceOutput.distance <= totalRadius)
        {
            Plane plane = new(distanceOutput.normal, totalRadius - distanceOutput.distance);
            return new() { plane = plane, point = distanceOutput.pointA, hit = true };
        }
        return new();
    }
    public ShapeProxy MakeProxy() => Distance.MakeProxy([center1, center2], radius);
    ///<summary>Test a point for overlap with a capsule in local space</summary>
    public bool TestPoint(Vector2 point)
    {
        float rr = radius * radius;
        Vector2 p1 = center1, p2 = center2;
        Vector2 d = p2 - p1;
        float dd = Vector2.Dot(d, d);
        if (dd == 0) return Vector2.DistanceSquared(point, p1) <= rr;
        float t = Math.Clamp(Vector2.Dot(point - p1, d) / dd, 0, 1);
        Vector2 c = Vector2.MulAdd(p1, t, d);
        return Vector2.DistanceSquared(point, c) <= rr;
    }
}
/// <summary>A solid convex polygon. It is assumed that the interior of the polygon is to
/// the left of each edge.
/// Polygons have a maximum number of vertices equal to B2_MAX_POLYGON_VERTICES.
/// In most cases you should not need many vertices for a convex polygon.
/// @warning DO NOT fill this out manually, instead use a helper function like
/// MakePolygon or MakeBox.</summary>
public record Polygon : IShape
{
    /// <summary>The polygon vertices</summary>
    public Vector2[] vertices;
    /// <summary>The outward normal vectors of the polygon sides</summary>
    public Vector2[] normals;
    /// <summary>The centroid of the polygon</summary>
    public Vector2 centroid;
    /// <summary>The external radius for rounded polygons</summary>
    public float radius;
    public float GetRadius() => radius;
    ///<summary>Compute the bounding box of a transformed polygon</summary>
    public AABB ComputeAABB(Transform xf)
    {
        Debug.Assert(vertices.Length > 0);
        Vector2 lower = xf.TransformPoint(vertices[0]), upper = lower;
        for (int i = 1; i < vertices.Length; i++)
        {
            Vector2 v = xf.TransformPoint(vertices[i]);
            lower = Vector2.Min(lower, v);
            upper = Vector2.Max(upper, v);
        }
        Vector2 r = new(radius, radius);
        lower -= r;
        upper += r;
        return new(lower, upper);
    }
    public Vector2 GetCentroid() => centroid;
    public float GetPerimeter()
    {
        float perimeter = 2 * MathF.PI * radius;
        Debug.Assert(vertices.Length > 0);
        Vector2 prev = vertices[^1];
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector2 next = vertices[i];
            perimeter += Vector2.Distance(next, prev);
            prev = next;
        }
        return perimeter;
    }
    public float GetProjectedPerimeter(Vector2 line)
    {
        Debug.Assert(vertices.Length > 0);
        float value = Vector2.Dot(vertices[0], line);
        float lower = value, upper = value;
        for (int i = 1; i < vertices.Length; i++)
        {
            value = Vector2.Dot(vertices[i], line);
            lower = Math.Min(lower, value);
            upper = Math.Max(upper, value);
        }
        return upper - lower + 2 * radius;
    }
    ///<summary>Compute mass properties of a polygon</summary>
    public unsafe MassData ComputeMass(float density)
    {
        Debug.Assert(this.vertices.Length > 0);
        if (this.vertices.Length == 1)
        {
            Circle circle = new() { center = this.vertices[0], radius = radius };
            return circle.ComputeMass(density);
        }
        if (this.vertices.Length == 2)
        {
            Capsule capsule = new() { center1 = this.vertices[0], center2 = this.vertices[1], radius = radius };
            return capsule.ComputeMass(density);
        }
        Vector2[] vertices = new Vector2[this.vertices.Length];
        if (radius > 0)
        {
            float sqrt2 = 1.4142135623730950488016887242097f;
            for (int i = 0, j = vertices.Length - 1; i < vertices.Length; j = i++)
            {
                Vector2 n1 = normals[j], n2 = normals[i];
                Vector2 mid = (n1 + n2).Normalize();
                vertices[i] = Vector2.MulAdd(this.vertices[i], sqrt2 * radius, mid);
            }
        }
        else Array.Copy(this.vertices, 0, vertices, 0, vertices.Length);
        Vector2 center = new();
        float area = 0, rotationalInertia = 0;
        Vector2 r = vertices[0];
        const float inv3 = 1 / 3f;
        for (int i = 1; i < vertices.Length - 1; i++)
        {
            Vector2 e1 = vertices[i] - r;
            Vector2 e2 = vertices[i + 1] - r;
            float D = Vector2.Cross(e1, e2);
            float triangleArea = 0.5f * D;
            area += triangleArea;
            center = Vector2.MulAdd(center, triangleArea * inv3, e1 + e2);
            float intx2 = e1.x * e1.x + e2.x * e1.x + e2.x * e2.x;
            float inty2 = e1.y * e1.y + e2.y * e1.y + e2.y * e2.y;
            rotationalInertia += 0.25f * inv3 * D * (intx2 + inty2);
        }
        Debug.Assert(area > Box2D.FLT_EPSILON);
        float invArea = 1 / area;
        center.x *= invArea;
        center.y *= invArea;
        MassData massData = new()
        {
            mass = density * area,
            center = r + center,
            rotationalInertia = density * rotationalInertia - density * area * Vector2.Dot(center, center)
        };
        Debug.Assert(massData.rotationalInertia >= 0);
        return massData;
    }
    public ShapeExtent ComputeExtent(Vector2 localCenter)
    {
        float minExtent = Box2D.Huge, maxExtentSqr = 0;
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector2 v = vertices[i];
            float planeOffset = Vector2.Dot(normals[i], v - centroid);
            minExtent = Math.Min(minExtent, planeOffset);
            maxExtentSqr = Math.Max(maxExtentSqr, Vector2.DistanceSquared(v, localCenter));
        }
        return new() { minExtent = minExtent + radius, maxExtent = MathF.Sqrt(maxExtentSqr) + radius };
    }
    ///<summary>Ray cast versus polygon shape in local space.</summary>
    public CastOutput RayCast(ref RayCastInput input)
    {
        Debug.Assert(Geometry.IsValidRay(ref input));
        if (radius == 0)
        {
            Vector2 base_ = vertices[0];
            Vector2 p1 = input.origin - base_;
            Vector2 d = input.translation;
            float lower = 0, upper = input.maxFraction;
            int index = -1;
            CastOutput output = new();
            for (int edgeIndex = 0; edgeIndex < vertices.Length; edgeIndex++)
            {
                Vector2 vertex = vertices[edgeIndex] - base_;
                float numerator = Vector2.Dot(normals[edgeIndex], vertex - p1);
                float denominator = Vector2.Dot(normals[edgeIndex], d);
                if (denominator == 0)
                {
                    if (numerator < 0) return output;
                }
                else
                {
                    if (denominator < 0 && numerator < lower * denominator)
                    {
                        lower = numerator / denominator;
                        index = edgeIndex;
                    }
                    else if (denominator > 0 && numerator < upper * denominator)
                        upper = numerator / denominator;
                }
                if (upper < lower) return output;
            }
            Debug.Assert(0 <= lower && lower <= input.maxFraction);
            if (index >= 0)
            {
                output.fraction = lower;
                output.normal = normals[index];
                output.point = Vector2.MulAdd(input.origin, lower, d);
                output.hit = true;
            }
            else
            {
                output.point = input.origin;
                output.hit = true;
            }
            return output;
        }
        ShapeCastPairInput castInput = new()
        {
            proxyA = Distance.MakeProxy(vertices, radius),
            proxyB = Distance.MakeProxy([input.origin], 0),
            transformA = Transform.Identity,
            transformB = Transform.Identity,
            translationB = input.translation,
            maxFraction = input.maxFraction,
            canEncroach = false
        };
        return castInput.ShapeCast();
    }
    ///<summary>Shape cast versus a convex polygon.</summary>
    public CastOutput ShapeCast(ref ShapeCastInput input)
    {
        ShapeCastPairInput pairInput = new()
        {
            proxyA = Distance.MakeProxy(vertices, radius),
            proxyB = input.proxy,
            transformA = Transform.Identity,
            transformB = Transform.Identity,
            translationB = input.translation,
            maxFraction = input.maxFraction,
            canEncroach = input.canEncroach
        };
        return pairInput.ShapeCast();
    }
    public PlaneResult CollideMover(ref Capsule mover)
    {
        DistanceInput distanceInput = new()
        {
            proxyA = Distance.MakeProxy(vertices, radius),
            proxyB = Distance.MakeProxy([mover.center1, mover.center2], mover.radius),
            transformA = Transform.Identity,
            transformB = Transform.Identity,
            useRadii = false,
        };
        float totalRadius = mover.radius + radius;
        SimplexCache cache = new();
        DistanceOutput distanceOutput = distanceInput.ShapeDistance(ref cache, null);
        if (distanceOutput.distance <= totalRadius)
        {
            Plane plane = new(distanceOutput.normal, totalRadius - distanceOutput.distance);
            return new() { plane = plane, point = distanceOutput.pointA, hit = true };
        }
        return new();
    }
    public ShapeProxy MakeProxy() => Distance.MakeProxy(vertices, radius);
    ///<summary>Test a point for overlap with a convex polygon in local space</summary>
    public bool TestPoint(Vector2 point)
    {
        DistanceInput input = new()
        {
            proxyA = Distance.MakeProxy(vertices, 0),
            proxyB = Distance.MakeProxy([point], 0),
            transformA = Transform.Identity,
            transformB = Transform.Identity,
            useRadii = false
        };
        SimplexCache cache = new();
        DistanceOutput output = input.ShapeDistance(ref cache, null);
        return output.distance <= radius;
    }
}
/// <summary>A line segment with two-sided collision.</summary>
public record Segment : IShape
{
    /// <summary>The first point</summary>
    public Vector2 point1;
    /// <summary>The second point</summary>
    public Vector2 point2;
    ///<summary>Compute the bounding box of a transformed line segment</summary>
    public AABB ComputeAABB(Transform xf)
    {
        Vector2 v1 = xf.TransformPoint(point1), v2 = xf.TransformPoint(point2);
        Vector2 lower = Vector2.Min(v1, v2);
        Vector2 upper = Vector2.Max(v1, v2);
        return new(lower, upper);
    }
    public Vector2 GetCentroid() => Vector2.Lerp(point1, point2, 0.5f);
    public float GetPerimeter() => 2 * Vector2.Distance(point1, point2);
    public float GetProjectedPerimeter(Vector2 line) => Math.Abs(Vector2.Dot(point2, line) - Vector2.Dot(point1, line));
    public ShapeExtent ComputeExtent(Vector2 localCenter) => new()
    {
        minExtent = 0,
        maxExtent = MathF.Sqrt(Math.Max(Vector2.DistanceSquared(point1, localCenter), Vector2.DistanceSquared(point2, localCenter)))
    };
    ///<summary>Ray cast versus segment shape in local space. Optionally treat the segment as one-sided with hits from
    ///the left side being treated as a miss.</summary>
    public CastOutput RayCast(ref RayCastInput input, bool oneSided)
    {
        if (oneSided)
        {
            float offset = Vector2.Cross(input.origin - point1, point2 - point1);
            if (offset < 0) return new();
        }
        Vector2 p1 = input.origin;
        Vector2 d = input.translation;
        Vector2 v1 = point1, v2 = point2;
        Vector2 e = v2 - v1;
        CastOutput output = new();
        Vector2 eUnit = e.GetLengthAndNormalize(out float length);
        if (length == 0) return output;
        Vector2 normal = eUnit.RightPerp();
        float numerator = Vector2.Dot(normal, v1 - p1);
        float denominator = Vector2.Dot(normal, d);
        if (denominator == 0) return output;
        float t = numerator / denominator;
        if (t < 0 || input.maxFraction < t) return output;
        Vector2 p = Vector2.MulAdd(p1, t, d);
        float s = Vector2.Dot(p - v1, eUnit);
        if (s < 0 || length < s) return output;
        if (numerator > 0) normal = -normal;
        output.fraction = t;
        output.point = p;
        output.normal = normal;
        output.hit = true;
        return output;
    }
    public CastOutput RayCast(ref RayCastInput input) => RayCast(ref input, false);
    ///<summary>Shape cast versus a line segment.</summary>
    public CastOutput ShapeCast(ref ShapeCastInput input)
    {
        ShapeCastPairInput pairInput = new()
        {
            proxyA = Distance.MakeProxy([point1, point2], 0),
            proxyB = input.proxy,
            transformA = Transform.Identity,
            transformB = Transform.Identity,
            translationB = input.translation,
            maxFraction = input.maxFraction,
            canEncroach = input.canEncroach
        };
        return pairInput.ShapeCast();
    }
    public PlaneResult CollideMover(ref Capsule mover)
    {
        DistanceInput distanceInput = new()
        {
            proxyA = Distance.MakeProxy([point1, point2], 0),
            proxyB = Distance.MakeProxy([mover.center1, mover.center2], mover.radius),
            transformA = Transform.Identity,
            transformB = Transform.Identity,
            useRadii = false,
        };
        float totalRadius = mover.radius;
        SimplexCache cache = new();
        DistanceOutput distanceOutput = distanceInput.ShapeDistance(ref cache, null);
        if (distanceOutput.distance <= totalRadius)
        {
            Plane plane = new(distanceOutput.normal, totalRadius - distanceOutput.distance);
            return new() { plane = plane, point = distanceOutput.pointA, hit = true };
        }
        return new();
    }
    public ShapeProxy MakeProxy() => Distance.MakeProxy([point1, point2], 0);
}
/// <summary>A line segment with one-sided collision. Only collides on the right side.
/// Several of these are generated for a chain shape.
/// ghost1 -> point1 -> point2 -> ghost2</summary>
public record ChainSegment : IShape
{
    /// <summary>The tail ghost vertex</summary>
    public Vector2 ghost1;
    /// <summary>The line segment</summary>
    public Segment segment;
    /// <summary>The head ghost vertex</summary>
    public Vector2 ghost2;
    /// <summary>The owning chain shape index (internal usage only)</summary>
    public int chainId;
    public AABB ComputeAABB(Transform xf) => segment.ComputeAABB(xf);
    public Vector2 GetCentroid() => segment.GetCentroid();
    public float GetPerimeter() => segment.GetPerimeter();
    public float GetProjectedPerimeter(Vector2 line) => segment.GetProjectedPerimeter(line);
    public ShapeExtent ComputeExtent(Vector2 localCenter) => segment.ComputeExtent(localCenter);
    public CastOutput RayCast(ref RayCastInput input) => segment.RayCast(ref input, true);
    public CastOutput ShapeCast(ref ShapeCastInput input)
    {
        Vector2 approximateCentroid = input.proxy.points[0];
        for (int i = 1; i < input.proxy.points.Length; i++)
            approximateCentroid += input.proxy.points[i];
        approximateCentroid = 1.0f / input.proxy.points.Length * approximateCentroid;
        Vector2 edge = segment.point2 - segment.point1;
        Vector2 r = approximateCentroid - segment.point1;
        return Vector2.Cross(r, edge) < 0 ? new() : segment.ShapeCast(ref input);
    }
    public PlaneResult CollideMover(ref Capsule mover) => segment.CollideMover(ref mover);
    public ShapeProxy MakeProxy() => segment.MakeProxy();
}
/// <summary>A convex hull. Used to create convex polygons.
/// Do not modify these values directly, instead use b2ComputeHull()</summary>
public partial struct Hull
{
    /// <summary>The final points of the hull</summary>
    public List<Vector2> points = new();
    public Hull() { }
}
/// <summary>Result of computing the distance between two line segments</summary>
public struct SegmentDistanceResult
{
    /// <summary>The closest point on the first segment</summary>
    public Vector2 closest1;
    /// <summary>The closest point on the second segment</summary>
    public Vector2 closest2;
    /// <summary>The barycentric coordinate on the first segment</summary>
    public float fraction1;
    /// <summary>The barycentric coordinate on the second segment</summary>
    public float fraction2;
    /// <summary>The squared distance between the closest points</summary>
    public float distanceSquared;
}
/// <summary>Used to warm start the GJK simplex. If you call this function multiple times with nearby
/// transforms this might improve performance. Otherwise you can zero initialize this.
/// The distance cache must be initialized to zero on the first call.
/// Users should generally just zero initialize this structure for each call.</summary>
public struct SimplexCache
{
    /// <summary>The number of stored simplex points</summary>
    public ushort count = 0;
    /// <summary>The cached simplex indices on shape A</summary>
    public byte[] indexA = new byte[3];
    /// <summary>The cached simplex indices on shape B</summary>
    public byte[] indexB = new byte[3];
    public SimplexCache() { }
}
/// <summary>Input for b2ShapeDistance</summary>
public struct DistanceInput
{
    /// <summary>The proxy for shape A</summary>
    public ShapeProxy proxyA;
    /// <summary>The proxy for shape B</summary>
    public ShapeProxy proxyB;
    /// <summary>The world transform for shape A</summary>
    public Transform transformA;
    /// <summary>The world transform for shape B</summary>
    public Transform transformB;
    /// <summary>Should the proxy radius be considered?</summary>
    public bool useRadii;
}
/// <summary>Output for b2ShapeDistance</summary>
public struct DistanceOutput
{
    /// <summary>Closest point on shapeA</summary>
    public Vector2 pointA;
    /// <summary>Closest point on shapeB</summary>
    public Vector2 pointB;
    /// <summary>Normal vector that points from A to B. Invalid if distance is zero.</summary>
    public Vector2 normal;
    /// <summary>The final distance, zero if overlapped</summary>
    public float distance;
    /// <summary>Number of GJK iterations used</summary>
    public int iterations;
    /// <summary>The number of simplexes stored in the simplex array</summary>
    public int simplexCount;
}
/// <summary>Simplex vertex for debugging the GJK algorithm</summary>
public struct SimplexVertex
{
    /// <summary>support point in proxyA</summary>
    public Vector2 wA;
    /// <summary>support point in proxyB</summary>
    public Vector2 wB;
    /// <summary>wB - wA</summary>
    public Vector2 w;
    /// <summary>barycentric coordinate for closest point</summary>
    public float a;
    /// <summary>wA index</summary>
    public int indexA;
    /// <summary>wB index</summary>
    public int indexB;
}
/// <summary>Simplex from the GJK algorithm</summary>
public struct Simplex
{
    /// <summary>vertices</summary>
    public SimplexVertex v1, v2, v3;
    /// <summary>number of valid vertices</summary>
    public int count;
}
/// <summary>Input parameters for b2ShapeCast</summary>
public struct ShapeCastPairInput
{
    /// <summary>The proxy for shape A</summary>
    public ShapeProxy proxyA;
    /// <summary>The proxy for shape B</summary>
    public ShapeProxy proxyB;
    /// <summary>The world transform for shape A</summary>
    public Transform transformA;
    /// <summary>The world transform for shape B</summary>
    public Transform transformB;
    /// <summary>The translation of shape B</summary>
    public Vector2 translationB;
    /// <summary>The fraction of the translation to consider, typically 1</summary>
    public float maxFraction;
    /// <summary>Allows shapes with a radius to move slightly closer if already touching</summary>
    public bool canEncroach;
}
/// <summary>This describes the motion of a body/shape for TOI computation. Shapes are defined with respect to the body origin,
/// which may not coincide with the center of mass. However, to support dynamics we must interpolate the center of mass
/// position.</summary>
public struct Sweep
{
    /// <summary>Local center of mass position</summary>
    public Vector2 localCenter;
    /// <summary>Starting center of mass world position</summary>
    public Vector2 c1;
    /// <summary>Ending center of mass world position</summary>
    public Vector2 c2;
    /// <summary>Starting world rotation</summary>
    public Rotation q1;
    /// <summary>Ending world rotation</summary>
    public Rotation q2;
}
/// <summary>Time of impact input</summary>
public struct TOIInput
{
    /// <summary>The proxy for shape A</summary>
    public ShapeProxy proxyA;
    /// <summary>The proxy for shape B</summary>
    public ShapeProxy proxyB;
    /// <summary>The movement of shape A</summary>
    public Sweep sweepA;
    /// <summary>The movement of shape B</summary>
    public Sweep sweepB;
    /// <summary>Defines the sweep interval [0, maxFraction]</summary>
    public float maxFraction;
}
/// <summary>Describes the TOI output</summary>
public enum TOIState
{
    Unknown, Failed, Overlapped, Hit, Separated
}
/// <summary>Time of impact output</summary>
public struct TOIOutput
{
    /// <summary>The type of result</summary>
    public TOIState state;
    /// <summary>The hit point</summary>
    public Vector2 point;
    /// <summary>The hit normal</summary>
    public Vector2 normal;
    /// <summary>The sweep time of the collision</summary>
    public float fraction;
}
/// <summary>A manifold point is a contact point belonging to a contact manifold.
/// It holds details related to the geometry and dynamics of the contact points.
/// Box2D uses speculative collision so some contact points may be separated.
/// You may use the totalNormalImpulse to determine if there was an interaction during
/// the time step.</summary>
public struct ManifoldPoint
{
    /// <summary>Location of the contact point in world space. Subject to precision loss at large coordinates.
    /// Should only be used for debugging.</summary>
    public Vector2 point;
    /// <summary>Location of the contact point relative to shapeA's origin in world space
    /// When used internally to the Box2D solver, this is relative to the body center of mass.</summary>
    public Vector2 anchorA;
    /// <summary>Location of the contact point relative to shapeB's origin in world space
    /// When used internally to the Box2D solver, this is relative to the body center of mass.</summary>
    public Vector2 anchorB;
    /// <summary>The separation of the contact point, negative if penetrating</summary>
    public float separation;
    /// <summary>The impulse along the manifold normal vector.</summary>
    public float normalImpulse;
    /// <summary>The friction impulse</summary>
    public float tangentImpulse;
    /// <summary>The total normal impulse applied across sub-stepping and restitution. This is important
    /// to identify speculative contact points that had an interaction in the time step.</summary>
    public float totalNormalImpulse;
    /// <summary>Relative normal velocity pre-solve. Used for hit events. If the normal impulse is
    /// zero then there was no hit. Negative means shapes are approaching.</summary>
    public float normalVelocity;
    /// <summary>Uniquely identifies a contact point between two shapes</summary>
    public ushort id;
    /// <summary>Did this contact point exist the previous step?</summary>
    public bool persisted;
}
/// <summary>A contact manifold describes the contact points between colliding shapes.
/// Box2D uses speculative collision so some contact points may be separated.</summary>
public struct Manifold
{
    /// <summary>The unit normal vector in world space, points from shape A to bodyB</summary>
    public Vector2 normal;
    /// <summary>Angular impulse applied for rolling resistance. N * m * s = kg * m^2 / s</summary>
    public float rollingImpulse;
    /// <summary>The manifold points, up to two are possible in 2D</summary>
    public ManifoldPoint point0, point1;
    /// <summary>The number of contacts points, will be 0, 1, or 2</summary>
    public int pointCount;
}
/// <summary>The dynamic tree structure. This should be considered private data.
/// It is placed here for performance reasons.</summary>
public partial class DynamicTree
{
    /// <summary>The tree nodes</summary>
    public TreeNode[] nodes;
    /// <summary>The root index</summary>
    public int root;
    /// <summary>The number of nodes</summary>
    public int nodeCount;
    /// <summary>Node free list</summary>
    public int freeList;
    /// <summary>Number of proxies created</summary>
    public int proxyCount;
    /// <summary>Leaf indices for rebuild</summary>
    public int[] leafIndices;
    /// <summary>Leaf bounding boxes for rebuild</summary>
    public AABB[] leafBoxes;
    /// <summary>Leaf bounding box centers for rebuild</summary>
    public Vector2[] leafCenters;
    /// <summary>Bins for sorting during rebuild</summary>
    public int[] binIndices;
    /// <summary>Allocated space for rebuilding</summary>
    public int rebuildCapacity;
}
/// <summary>These are performance results returned by dynamic tree queries.</summary>
public struct TreeStats
{
    /// <summary>Number of internal nodes visited during the query</summary>
    public int nodeVisits;
    /// <summary>Number of leaf nodes visited during the query</summary>
    public int leafVisits;
}
/// <summary>This function receives proxies found in the AABB query.</summary>
/// <returns>true if the query should continue</returns>
public delegate bool TreeQueryCallbackFcn(int proxyId, ulong userData, object context);
/// <summary>This function receives clipped ray cast input for a proxy. The function<br/>
/// returns the new ray fraction.<br/>
/// - return a value of 0 to terminate the ray cast<br/>
/// - return a value less than input->maxFraction to clip the ray<br/>
/// - return a value of input->maxFraction to continue the ray cast without clipping</summary>
public delegate float TreeRayCastCallbackFcn(ref RayCastInput input, int proxyId, ulong userData, object context);
/// <summary>This function receives clipped ray cast input for a proxy. The function<br/>
/// returns the new ray fraction.<br/>
/// - return a value of 0 to terminate the ray cast<br/>
/// - return a value less than input->maxFraction to clip the ray<br/>
/// - return a value of input->maxFraction to continue the ray cast without clipping</summary>
public delegate float TreeShapeCastCallbackFcn(ref ShapeCastInput input, int proxyId, ulong userData, object context);
/// <summary>These are the collision planes returned from b2World_CollideMover</summary>
public struct PlaneResult
{
    /// <summary>The collision plane between the mover and a convex shape</summary>
    public Plane plane;
    /// <summary>The collision point on the shape.</summary>
    public Vector2 point;
    /// <summary>Did the collision register a hit? If not this plane should be ignored.</summary>
    public bool hit;
}
/// <summary>These are collision planes that can be fed to b2SolvePlanes. Normally
/// this is assembled by the user from plane results in b2PlaneResult</summary>
public struct CollisionPlane
{
    /// <summary>The collision plane between the mover and some shape</summary>
    public Plane plane;
    /// <summary>Setting this to FLT_MAX makes the plane as rigid as possible. Lower values can
    /// make the plane collision soft. Usually in meters.</summary>
    public float pushLimit;
    /// <summary>The push on the mover determined by b2SolvePlanes. Usually in meters.</summary>
    public float push;
    /// <summary>Indicates if b2ClipVector should clip against this plane. Should be false for soft collision.</summary>
    public bool clipVelocity;
}
/// <summary>Result returned by b2SolvePlanes</summary>
public struct PlaneSolverResult
{
    /// <summary>The translation of the mover</summary>
    public Vector2 translation;
    /// <summary>The number of iterations used by the plane solver. For diagnostics.</summary>
    public int iterationCount;
}