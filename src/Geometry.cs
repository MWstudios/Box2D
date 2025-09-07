using System;
using System.Diagnostics;

namespace Box2D;

public static partial class Geometry
{
    /// <summary>Validate ray cast input data (NaN, etc)</summary>
    public static bool IsValidRay(ref RayCastInput input) => input.origin.IsValid() && input.translation.IsValid() &&
        float.IsFinite(input.maxFraction) && 0 <= input.maxFraction && input.maxFraction < Box2D.Huge;
    public static Vector2 ComputePolygonCentroid(Vector2[] vertices)
    {
        Vector2 center = new();
        float area = 0;
        Vector2 origin = vertices[0];
        const float inv3 = 1 / 3f;
        for (int i = 1; i < vertices.Length - 1; i++)
        {
            Vector2 e1 = vertices[i] - origin;
            Vector2 e2 = vertices[i + 1] - origin;
            float a = 0.5f * Vector2.Cross(e1, e2);
            center = Vector2.MulAdd(center, a * inv3, e1 + e2);
            area += a;
        }
        Debug.Assert(area > Box2D.FLT_EPSILON);
        float invArea = 1 / area;
        center.x *= invArea;
        center.y *= invArea;
        center = origin + center;
        return center;
    }
    /// <summary>Make a convex polygon from a convex hull. This will assert if the hull is not valid.
    /// Do not manually fill in the hull data, it must come directly from ComputeHull</summary>
    public static Polygon MakePolygon(ref Hull hull, float radius)
    {
        Debug.Assert(hull.Validate());
        if (hull.points.Count < 3) return MakeSquare(0.5f);
        Polygon shape = new() { vertices = hull.points.ToArray(), normals = new Vector2[hull.points.Count], radius = radius };
        for (int i = 0; i < shape.vertices.Length; i++)
        {
            int i2 = i + 1 < shape.vertices.Length ? i + 1 : 0;
            Vector2 edge = shape.vertices[i2] - shape.vertices[i];
            Debug.Assert(Vector2.Dot(edge, edge) > Box2D.FLT_EPSILON * Box2D.FLT_EPSILON);
            shape.normals[i] = Vector2.CrossVS(edge, 1).Normalize();
        }
        shape.centroid = ComputePolygonCentroid(shape.vertices);
        return shape;
    }
    /// <summary>Make an offset convex polygon from a convex hull. This will assert if the hull is not valid.
    /// Do not manually fill in the hull data, it must come directly from ComputeHull</summary>
    public static Polygon MakeOffsetPolygon(ref Hull hull, Vector2 position, Rotation rotation)
        => MakeOffsetRoundedPolygon(ref hull, position, rotation, 0);
    /// <summary>Make an offset convex polygon from a convex hull. This will assert if the hull is not valid.
    /// Do not manually fill in the hull data, it must come directly from ComputeHull</summary>
    public static Polygon MakeOffsetRoundedPolygon(ref Hull hull, Vector2 position, Rotation rotation, float radius)
    {
        Debug.Assert(hull.Validate());
        if (hull.points.Count < 3) return MakeSquare(0.5f);
        Transform transform = new(position, rotation);
        Polygon shape = new() { vertices = new Vector2[hull.points.Count], normals = new Vector2[hull.points.Count], radius = radius };
        for (int i = 0; i < shape.vertices.Length; i++)
            shape.vertices[i] = transform.TransformPoint(hull.points[i]);
        for (int i = 0; i < shape.vertices.Length; i++)
        {
            int i2 = i + 1 < shape.vertices.Length ? i + 1 : 0;
            Vector2 edge = shape.vertices[i2] - shape.vertices[i];
            Debug.Assert(Vector2.Dot(edge, edge) > Box2D.FLT_EPSILON * Box2D.FLT_EPSILON);
            shape.normals[i] = Vector2.CrossVS(edge, 1).Normalize();
        }
        shape.centroid = ComputePolygonCentroid(shape.vertices);
        return shape;
    }
    /// <summary>Make a square polygon, bypassing the need for a convex hull.</summary>
    /// <param name="halfWidth">the half-width</param>
    public static Polygon MakeSquare(float halfWidth) => MakeBox(halfWidth, halfWidth);
    /// <summary>Make a box (rectangle) polygon, bypassing the need for a convex hull.</summary>
    /// <param name="halfWidth">the half-width (x-axis)</param>
    /// <param name="halfHeight">the half-height (y-axis)</param>
    public static Polygon MakeBox(float halfWidth, float halfHeight)
    {
        Debug.Assert(float.IsFinite(halfWidth) && halfWidth > 0);
        Debug.Assert(float.IsFinite(halfHeight) && halfHeight > 0);
        return new()
        {
            vertices = [new(-halfWidth, -halfHeight), new(halfWidth, -halfHeight), new(halfWidth, halfHeight), new(-halfWidth, halfHeight)],
            normals = [new(0, -1), new(1, 0), new(0, 1), new(-1, 0)],
            radius = 0,
            centroid = Vector2.Zero
        };
    }
    /// <summary>Make a rounded box, bypassing the need for a convex hull.</summary>
    /// <param name="halfWidth">the half-width (x-axis)</param>
    /// <param name="halfHeight">the half-height (y-axis)</param>
    /// <param name="radius">the radius of the rounded extension</param>
    public static Polygon MakeRoundedBox(float halfWidth, float halfHeight, float radius)
    {
        Debug.Assert(float.IsFinite(radius) && radius >= 0);
        Polygon shape = MakeBox(halfWidth, halfHeight);
        shape.radius = radius;
        return shape;
    }
    /// <summary>Make an offset rounded box, bypassing the need for a convex hull.</summary>
    /// <param name="halfWidth">the half-width (x-axis)</param>
    /// <param name="halfHeight">the half-height (y-axis)</param>
    /// <param name="center">the local center of the box</param>
    /// <param name="rotation">the local rotation of the box</param>
    public static Polygon MakeOffsetBox(float halfWidth, float halfHeight, Vector2 center, Rotation rotation)
    {
        Transform xf = new(center, rotation);
        return new()
        {
            vertices =
            [
                xf.TransformPoint(new(-halfWidth, -halfHeight)),
                xf.TransformPoint(new(halfWidth, -halfHeight)),
                xf.TransformPoint(new(halfWidth, halfHeight)),
                xf.TransformPoint(new(-halfWidth, halfHeight))
            ],
            normals = [xf.q * new Vector2(0, -1), xf.q * new Vector2(1, 0), xf.q * new Vector2(0, 1), xf.q * new Vector2(-1, 0)],
            radius = 0,
            centroid = xf.p
        };
    }
    /// <summary>Make an offset rounded box, bypassing the need for a convex hull.</summary>
    /// <param name="halfWidth">the half-width (x-axis)</param>
    /// <param name="halfHeight">the half-height (y-axis)</param>
    /// <param name="center">the local center of the box</param>
    /// <param name="rotation">the local rotation of the box</param>
    /// <param name="radius">the radius of the rounded extension</param>
    public static Polygon MakeOffsetRoundedBox(float halfWidth, float halfHeight, Vector2 center, Rotation rotation, float radius)
    {
        Debug.Assert(float.IsFinite(radius) && radius >= 0);
        Polygon shape = MakeOffsetBox(halfWidth, halfHeight, center, rotation);
        shape.radius = radius;
        return shape;
    }
    /// <summary>Transform a polygon. This is useful for transferring a shape from one body to another.</summary>
    public static Polygon TransformPolygon(Transform transform, ref Polygon polygon)
    {
        for (int i = 0; i < polygon.vertices.Length; i++)
        {
            polygon.vertices[i] = transform.TransformPoint(polygon.vertices[i]);
            polygon.normals[i] = transform.TransformPoint(polygon.normals[i]);
        }
        polygon.centroid = transform.TransformPoint(polygon.centroid);
        return polygon;
    }
}
