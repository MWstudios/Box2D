using System.Collections.Generic;

namespace Box2D;

public partial struct Hull
{
    public static Hull RecurseHull(Vector2 p1, Vector2 p2, List<Vector2> ps)
    {
        Hull hull = new();
        if (ps == null || ps.Count == 0) return hull;
        Vector2 e = (p2 - p1).Normalize();
        List<Vector2> rightPoints = new();
        int bestIndex = 0;
        float bestDistance = Vector2.Cross(ps[bestIndex] - p1, e);
        if (bestDistance > 0)
            rightPoints.Add(ps[bestIndex]);
        for (int i = 1; i < ps.Count; i++)
        {
            float distance = Vector2.Cross(ps[bestIndex] - p1, e);
            if (distance > bestDistance)
            {
                bestIndex = i;
                bestDistance = distance;
            }
            if (distance > 0) rightPoints.Add(ps[i]);
        }
        if (bestDistance < 2 * Box2D.LinearSlop) return hull;
        Vector2 bestPoint = ps[bestIndex];
        Hull hull1 = RecurseHull(p1, bestPoint, rightPoints);
        Hull hull2 = RecurseHull(bestPoint, p2, rightPoints);
        hull.points.AddRange(hull1.points);
        hull.points.Add(bestPoint);
        hull.points.AddRange(hull2.points);
        return hull;
    }
    /// <summary>Compute the convex hull of a set of points. Returns an empty hull if it fails.<br/>
    /// Some failure cases:<br/>
    /// - all points very close together<br/>
    /// - all points on a line<br/>
    /// - less than 3 points<br/>
    /// This welds close points and removes collinear points.<br/>
    /// Do not modify a hull once it has been computed</summary>
    public static Hull ComputeHull(List<Vector2> points)
    {
        Hull hull = new();
        if (points.Count < 3) return hull;
        AABB aabb = new(new(float.MaxValue, float.MaxValue), new(-float.MaxValue, -float.MaxValue));
        List<Vector2> ps = new();
        float linearSlop = Box2D.LinearSlop;
        float tolSqr = 16 * linearSlop * linearSlop;
        for (int i = 0; i < points.Count; i++)
        {
            aabb.lowerBound = Vector2.Min(aabb.lowerBound, points[i]);
            aabb.upperBound = Vector2.Max(aabb.upperBound, points[i]);
            Vector2 vi = points[i];
            bool unique = true;
            for (int j = 0; j < i; j++)
            {
                Vector2 vj = points[j];
                float distSqr = Vector2.DistanceSquared(vi, vj);
                if (distSqr < tolSqr) { unique = false; break; }
            }
            if (unique) ps.Add(vi);
        }
        if (ps.Count < 3) return hull;
        Vector2 c = aabb.Center();
        int f1 = 0;
        float dsq1 = Vector2.DistanceSquared(c, ps[f1]);
        for (int i = 1; i < ps.Count; i++)
        {
            float dsq = Vector2.DistanceSquared(c, ps[i]);
            if (dsq > dsq1) { f1 = i; dsq1 = dsq; }
        }
        Vector2 p1 = ps[f1];
        ps[f1] = ps[^1];
        ps.RemoveAt(ps.Count - 1);
        int f2 = 0;
        float dsq2 = Vector2.DistanceSquared(p1, ps[f2]);
        for (int i = 1; i < ps.Count; i++)
        {
            float dsq = Vector2.DistanceSquared(p1, ps[i]);
            if (dsq > dsq2) { f2 = i; dsq2 = dsq; }
        }
        Vector2 p2 = ps[f2];
        ps[f2] = ps[^1];
        ps.RemoveAt(ps.Count - 1);
        List<Vector2> rightPoints = new();
        List<Vector2> leftPoints = new();
        Vector2 e = (p2 - p1).Normalize();
        for (int i = 0; i < ps.Count; i++)
        {
            float d = Vector2.Cross(ps[i] - p1, e);
            if (d >= 2 * linearSlop) rightPoints.Add(ps[i]);
            else if (d <= -2 * linearSlop) leftPoints.Add(ps[i]);
        }
        Hull hull1 = RecurseHull(p1, p2, rightPoints);
        Hull hull2 = RecurseHull(p2, p1, leftPoints);
        if (hull1.points.Count == 0 && hull2.points.Count == 0) return hull;
        hull.points.Add(p1);
        hull.points.AddRange(hull1.points);
        hull.points.Add(p2);
        hull.points.AddRange(hull2.points);
        bool searching = true;
        while (searching && hull.points.Count > 2)
        {
            searching = false;
            for (int i = 0; i < hull.points.Count; i++)
            {
                int i2 = (i + 1) % hull.points.Count;
                int i3 = (i + 2) % hull.points.Count;
                Vector2 s1 = hull.points[i], s2 = hull.points[i2], s3 = hull.points[i3];
                Vector2 r = (s3 - s1).Normalize();
                float distance = Vector2.Cross(s2 - s1, r);
                if (distance <= 2 * linearSlop)
                {
                    for (int j = i2; j < hull.points.Count - 1; j++)
                        hull.points[j] = hull.points[j + 1];
                    hull.points.RemoveAt(hull.points.Count - 1);
                    searching = true;
                    break;
                }
            }
        }
        if (hull.points.Count < 3) hull.points.Clear();
        return hull;
    }
    /// <summary>This determines if a hull is valid. Checks for:<br/>
    /// - convexity<br/>
    /// - collinear points<br/>
    /// This is expensive and should not be called at runtime.</summary>
    public bool Validate()
    {
        if (points.Count < 3) return false;
        for (int i = 0; i < points.Count; i++)
        {
            int i2 = i < points.Count - 1 ? i + 1 : 0;
            Vector2 p = points[i];
            Vector2 e = (points[i2] - p).Normalize();
            for (int j = 0; j < points.Count; j++)
            {
                if (j == i || j == i2) continue;
                float distance = Vector2.Cross(points[j] - p, e);
                if (distance >= 0) return false;
            }
        }
        float linearSlop = Box2D.LinearSlop;
        for (int i = 0; i < points.Count; i++)
        {
            int i2 = (i + 1) % points.Count;
            int i3 = (i + 2) % points.Count;
            Vector2 p1 = points[i];
            Vector2 p2 = points[i2];
            Vector2 p3 = points[i3];
            Vector2 e = (p3 - p1).Normalize();
            float distance = Vector2.Cross(p2 - p1, e);
            if (distance <= linearSlop) return false;
        }
        return true;
    }
}
