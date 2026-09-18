using System;
using System.Diagnostics;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.Arm;
using System.Runtime.Intrinsics.X86;

namespace Box2D;
/// <summary>2D vector<br/>
/// This can be used to represent a point or free vector</summary>
public struct Vector2
{
    public float x, y;
    public static readonly Vector2 Zero = new(0, 0);
    public Vector2(float x, float y) { this.x = x; this.y = y; }
    /// <summary>Is this a valid vector? Not NaN or infinity.</summary>
    public bool IsValid() => float.IsFinite(x) && float.IsFinite(y);
    /// <summary>Vector dot product</summary>
    public static float Dot(Vector2 a, Vector2 b) => a.x * b.x + a.y * b.y;
    /// <summary>Vector cross product. In 2D this yields a scalar.</summary>
    public static float Cross(Vector2 a, Vector2 b) => a.x * b.y - a.y * b.x;
    /// <summary>Perform the cross product on a vector and a scalar. In 2D this produces a vector.</summary>
    public static Vector2 CrossVS(Vector2 v, float s) => new(s * v.y, -s * v.x);
    /// <summary>Perform the cross product on a scalar and a vector. In 2D this produces a vector.</summary>
    public static Vector2 CrossSV(float s, Vector2 v) => new(-s * v.y, s * v.x);
    /// <summary>Get a left pointing perpendicular vector. Equivalent to b2CrossSV(1.0f, v)</summary>
    public Vector2 LeftPerp() => new(-y, x);
    /// <summary>Get a right pointing perpendicular vector. Equivalent to b2CrossVS(v, 1.0f)</summary>
    public Vector2 RightPerp() => new(y, -x);
    public static implicit operator Position(Vector2 v) => new(v.x, v.y);
    /// <summary>Vector addition</summary>
    public static Vector2 operator +(Vector2 a, Vector2 b) => new(a.x + b.x, a.y + b.y);
    /// <summary>Vector subtraction</summary>
    public static Vector2 operator -(Vector2 a, Vector2 b) => new(a.x - b.x, a.y - b.y);
    /// <summary>Vector negation</summary>
    public static Vector2 operator -(Vector2 a) => new(-a.x, -a.y);
    public static Vector2 Lerp(Vector2 a, Vector2 b, float t) => new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
    /// <summary>Component-wise multiplication</summary>
    public static Vector2 operator *(Vector2 a, Vector2 b) => new(a.x * b.x, a.y * b.y);
    /// <summary>Multiply a scalar and vector</summary>
    public static Vector2 operator *(float s, Vector2 v) => new(s * v.x, s * v.y);
    /// <summary>Multiply vector and a scalar</summary>
    public static Vector2 operator *(Vector2 v, float s) => new(s * v.x, s * v.y);
    public static Vector2 operator /(Vector2 v, float s) => new(v.x / s, v.y / s);
    public static bool operator ==(Vector2 a, Vector2 b) => a.x == b.x && a.y == b.y;
    public static bool operator !=(Vector2 a, Vector2 b) => a.x != b.x || a.y != b.y;
    public static Vector2 operator +(Vector2 a, float b) => new(a.x + b, a.y + b);
    public static Vector2 operator -(Vector2 a, float b) => new(a.x - b, a.y - b);
    /// <summary>a + s * b</summary>
    public static Vector2 MulAdd(Vector2 a, float s, Vector2 b) => new(a.x + s * b.x, a.y + s * b.y);
    /// <summary>a - s * b</summary>
    public static Vector2 MulSub(Vector2 a, float s, Vector2 b) => new(a.x - s * b.x, a.y - s * b.y);
    /// <summary>Component-wise absolute vector</summary>
    public Vector2 Abs() => new(x < 0 ? -x : x, y < 0 ? -y : y);
    /// <summary>Component-wise minimum vector</summary>
    public static Vector2 Min(Vector2 a, Vector2 b) => new(a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y);
    /// <summary>Component-wise maximum vector</summary>
    public static Vector2 Max(Vector2 a, Vector2 b) => new(a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y);
    /// <summary>Component-wise clamp vector v into the range [a, b]</summary>
    public static Vector2 Clamp(Vector2 v, Vector2 a, Vector2 b) => new(Math.Clamp(v.x, a.x, b.x), Math.Clamp(v.y, a.y, b.y));
    /// <summary>Get the length of this vector (the norm)</summary>
    public float Length() => MathF.Sqrt(x * x + y * y);
    /// <summary>Get the distance between two points</summary>
    public static float Distance(Vector2 a, Vector2 b) { float dx = b.x - a.x, dy = b.y - a.y; return MathF.Sqrt(dx * dx + dy * dy); }
    /// <summary>Convert a vector into a unit vector if possible, otherwise returns the zero vector.</summary>
    public Vector2 Normalize()
    {
        float lengthSquared = x * x + y * y;
        if (lengthSquared > 1000 * Box2D.FLT_EPSILON)
        {
            float s = 1 / MathF.Sqrt(lengthSquared);
            return new(x * s, y * s);
        }
        return new(0, 0);
    }
    /// <summary>Determines if the provided vector is normalized (norm(a) == 1).</summary>
    public bool IsNormalized() => Math.Abs(1 - Dot(this, this)) < 100 * Box2D.FLT_EPSILON;
    /// <summary>Convert a vector into a unit vector if possible, otherwise returns the zero vector. Also
    /// outputs the length.</summary>
    public Vector2 GetLengthAndNormalize(out float length)
    {
        float lengthSquared = x * x + y * y;
        if (lengthSquared > 1000 * Box2D.FLT_EPSILON)
        {
            length = MathF.Sqrt(lengthSquared);
            float s = 1 / length;
            return new(x * s, y * s);
        }
        length = 0;
        return new(0, 0);
    }
    /// <summary>Get the length squared of this vector</summary>
    public float LengthSquared() => x * x + y * y;
    /// <summary>Get the distance squared between points</summary>
    public static float DistanceSquared(Vector2 a, Vector2 b) { Vector2 c = b - a; return c.x * c.x + c.y * c.y; }
    public override string ToString() => $"({x}, {y})";
}
/// <summary>Cosine and sine pair<br/>
/// This uses a custom implementation designed for cross-platform determinism</summary>
public struct CosSin
{
    public float cosine, sine;
    public CosSin(float cosine, float sine) { this.cosine = cosine; this.sine = sine; }
    /// <summary>Compute the cosine and sine of an angle in radians. Implemented
    /// for cross-platform determinism.</summary>
    public CosSin(float radians) => (sine, cosine) = MathF.SinCos(radians);
    public override string ToString() => $"(c={cosine}, s={sine})";
}
/// <summary>2D rotation<br/>
/// This is similar to using a complex number for rotation</summary>
public struct Rotation
{
    public float c = 1, s = 0;
    public static readonly Rotation Identity = new(1, 0);
    public Rotation(float c, float s) { this.c = c; this.s = s; }
    /// <summary>Make a rotation using an angle in radians</summary>
    public Rotation(float radians) { CosSin cs = new(radians); c = cs.cosine; s = cs.sine; }
    /// <summary>Make a rotation using a unit vector</summary>
    public Rotation(Vector2 unitVector) { Debug.Assert(unitVector.IsNormalized()); c = unitVector.x; s = unitVector.y; }
    /// <summary>Compute the rotation between two unit vectors</summary>
    public Rotation(Vector2 v1, Vector2 v2)
    {
        Debug.Assert(Math.Abs(1 - v1.Length()) < 100 * Box2D.FLT_EPSILON);
        Debug.Assert(Math.Abs(1 - v2.Length()) < 100 * Box2D.FLT_EPSILON);
        c = Vector2.Dot(v1, v2); s = Vector2.Cross(v1, v2);
        float mag = MathF.Sqrt(s * s + c * c), invMag = mag > 0 ? 1 / mag : 0;
        c *= invMag; s *= invMag;
    }
    /// <summary>Is this a valid rotation? Not NaN or infinity. Is normalized.</summary>
    public bool IsValid()
    {
        if (!float.IsFinite(c) || !float.IsFinite(s)) return false;
        return IsNormalized();
    }
    /// <summary>Normalize rotation</summary>
    public Rotation Normalize()
    {
        float mag = MathF.Sqrt(s * s + c * c), invMag = mag > 0 ? 1 / mag : 0;
        return new(c * invMag, s * invMag);
    }
    /// <summary>Integrate rotation from angular velocity</summary>
    /// <param name="deltaAngle">the angular displacement in radians</param>
    public Rotation Integrate(float deltaAngle)
    {
        Rotation q2 = new(c - deltaAngle * s, s + deltaAngle * c);
        float mag = MathF.Sqrt(q2.s * q2.s + q2.c * q2.c), invMag = mag > 0 ? 1 / mag : 0;
        return new(q2.c * invMag, q2.s * invMag);
    }
    /// <summary>Is this rotation normalized?</summary>
    public bool IsNormalized() { float qq = s * s + c * c; return 1 - 0.0006f < qq && qq < 1 + 0.0006f; }
    /// <summary>Get the inverse of a rotation</summary>
    public Rotation Invert() => new(c, -s);
    /// <summary>Normalized linear interpolation
    /// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
    ///	https://web.archive.org/web/20170825184056/http://number-none.com/product/Understanding%20Slerp,%20Then%20Not%20Using%20It/</summary>
    public static Rotation NLerp(Rotation q1, Rotation q2, float t)
    {
        Rotation q = new(q1.c + (q2.c - q1.c) * t, q1.s + (q2.s - q1.s) * t);
        float mag = MathF.Sqrt(q.c * q.c + q.s * q.s), invMag = mag > 0 ? 1 / mag : 0;
        return new(q.c * invMag, q.s * invMag);
    }
    /// <summary>Compute the angular velocity necessary to rotate between two rotations over a give time</summary>
    /// <param name="q1">initial rotation</param>
    /// <param name="q2">final rotation</param>
    /// <param name="inv_h">inverse time step</param>
    public float ComputeAngularVelocity(Rotation q1, Rotation q2, float inv_h) => inv_h * (q2.s * q1.c - q2.c * q1.s);
    /// <summary>Get the angle in radians in the range [-pi, pi]</summary>
    public float GetAngle() => MathF.Atan2(s, c);
    /// <summary>Get the x-axis</summary>
    public Vector2 GetXAxis() => new(c, s);
    /// <summary>Get the y-axis</summary>
    public Vector2 GetYAxis() => new(-s, c);
    /// <summary>Multiply two rotations: q * r</summary>
    public static Rotation operator *(Rotation q, Rotation r) => new(q.c * r.c - q.s * r.s, q.s * r.c + q.c * r.s);
    /// <summary>Transpose multiply two rotations: inv(a) * b<br/>
    /// This rotates a vector local in frame b into a vector local in frame a</summary>
    public static Rotation InvMulRot(Rotation a, Rotation b) => new(a.c * b.c + a.s * b.s, a.c * b.s - a.s * b.c);
    /// <summary>Relative angle between a and b.</summary>
    public static float RelativeAngle(Rotation a, Rotation b) => MathF.Atan2(a.c * b.s - a.s * b.c, a.c * b.c + a.s * b.s);
    /// <summary>Convert any angle into the range [-pi, pi]</summary>
    public static float UnwindAngle(float radians)
    {
        Debug.Assert(MathF.Abs(radians) < 1e4f);
        float x = Math.Clamp(radians, -1e6f, 1e6f);
        const double twoPi = 2 * Math.PI, roundToNearest = 6755399441055744;
        return (float)(x - (x / twoPi + roundToNearest - roundToNearest) * twoPi);
    }
    /// <summary>Rotate a vector.</summary>
    public static Vector2 operator *(Rotation q, Vector2 v) => new(q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y);
    /// <summary>Inverse rotate a vector.</summary>
    public Vector2 InvRotateVector(Vector2 v) => new(c * v.x + s * v.y, -s * v.x + c * v.y);
    public static float RelativeCos(Rotation a, Rotation b) => a.c * b.c + a.s * b.s;
    public override string ToString() => $"(c={c}, s={s})";
}
/// <summary>A 2D rigid transform</summary>
public struct Transform
{
    public Vector2 p; public Rotation q;
    public static readonly Transform Identity = new(new(0, 0), new(1, 0));
    public Transform(Vector2 p, Rotation q) { this.p = p; this.q = q; }
    /// <summary>Is this a valid transform? Not NaN or infinity. Rotation is normalized.</summary>
    public bool IsValid() => p.IsValid() && q.IsValid();
    /// <summary>Transform a point (e.g. local space to world space).</summary>
    public Vector2 TransformPoint(Vector2 p) => new(q.c * p.x - q.s * p.y + this.p.x, q.s * p.x + q.c * p.y + this.p.y);
    /// <summary>Inverse transform a point (e.g. world space to local space).</summary>
    public Vector2 InvTransformPoint(Vector2 p) { float vx = p.x - this.p.x, vy = p.y - this.p.y; return new(q.c * vx + q.s * vy, -q.s * vx + q.c * vy); }
    /// <summary>Multiply two transforms. If the result is applied to a point p local to frame B,<br/>
    /// the transform would first convert p to a point local to frame A, then into a point<br/>
    /// in the world frame.<br/>
    /// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p<br/>
    ///    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p</summary>
    public static Transform operator *(Transform a, Transform b) => new(a.q * b.p + a.p, a.q * b.q);
    /// <summary>Creates a transform that converts a local point in frame B to a local point in frame A.<br/>
    /// v2 = A.q' * (B.q * v1 + B.p - A.p)<br/>
    ///    = A.q' * B.q * v1 + A.q' * (B.p - A.p)</summary>
    public static Transform InvMulTransforms(Transform a, Transform b) => new(a.q.InvRotateVector(b.p - a.p), Rotation.InvMulRot(a.q, b.q));
    public override string ToString() => $"{{{q}, {p}}}";
    /// <summary>Promote a float transform to a world transform. Lossless.</summary>
    public static implicit operator WorldTransform(Transform t) => new(t.p, t.q);
}
/// <summary>A world position. Double precision in large world mode so coordinates stay accurate far from the origin.</summary>
public struct Position
{
    public static readonly Position Zero = new(0, 0);
    public Position(float x, float y) { this.x = x; this.y = y; }
    /// <summary>Is this a valid vector? Not NaN or infinity.</summary>
    public bool IsValid() => float.IsFinite(x) && float.IsFinite(y);
#if BOX2D_DOUBLE_PRECISION
    public double x, y;
#else
    public float x, y;
#endif
    public static implicit operator Vector2(Position v) => new(v.x, v.y);
    /// <summary>a - b, demoted to float. The primary precision boundary operation.</summary>
    public static Vector2 operator -(Position a, Position b) => new((float)(a.x - b.x), (float)(a.y - b.y));
    /// <summary>Offset a world position by a vector</summary>
    public static Position operator +(Position p, Vector2 d) => new(p.x + d.x, p.y + d.y);
    /// <summary>Offset a world position by a vector</summary>
    public static Position operator -(Position p, Vector2 d) => new(p.x - d.x, p.y - d.y);
    /// <summary>World position interpolation for sweeps and sampling.</summary>
    public static Position Lerp(Position a, Position b, float t) => new((1 - t) * a.x + t * b.x, (1 - t) * a.y + t * b.y);
}
public struct WorldTransform
{
    public static readonly WorldTransform Identity = new(new(0, 0), new(1, 0));
    public WorldTransform(Position p, Rotation q) { this.p = p; this.q = q; }
    public Position p;
    public Rotation q;
    /// <summary>Is this a valid transform? Not NaN or infinity. Rotation is normalized.</summary>
    public bool IsValid() => p.IsValid() && q.IsValid();
    /// <summary>Transform a local point to a world position. Rotation in float, translation in double.</summary>
    public Position TransformWorldPoint(Vector2 p) => new(this.p.x + q.c * p.x - q.s * p.y, this.p.y + q.s * p.x + q.c * p.y);
    /// <summaryTransform a world position to a local point. One double subtraction, then float.</summary>
    public Vector2 InvTransformWorldPoint(Position p)
    {
        float vx = (float)(p.x - this.p.x), vy = (float)(p.y - this.p.y);
        return new(q.c * vx + q.s + vy, -q.s * vx + q.c * vy);
    }
    /// <summary>Relative transform of frame B in frame A.</summary>
    public static Transform InvMulWorldTransforms(WorldTransform A, WorldTransform B) => new(A.q.InvRotateVector(B.p - A.p), Rotation.InvMulRot(A.q, B.q));
    /// <summary>Shift a world transform into the frame of a base position.</summary>
    public Transform ToRelativeTransform(Position base_) => new(p - base_, q);
    /// <summary>Convert a local transform B into world space using world transform A.</summary>
    public WorldTransform Mul(Transform B) => new(p + q * B.p, q * B.q);
}
/// <summary>A 2-by-2 Matrix</summary>
public struct Mat22
{
    public Vector2 cx, cy;
    public static readonly Mat22 Zero = new(new(0, 0), new(0, 0));
    public Mat22(Vector2 cx, Vector2 cy) { this.cx = cx; this.cy = cy; }
    /// <summary>Multiply a 2-by-2 matrix times a 2D vector</summary>
    public static Vector2 operator *(Mat22 a, Vector2 v) => new(a.cx.x * v.x + a.cy.x * v.y, a.cx.y * v.x + a.cy.y * v.y);
    /// <summary>Get the inverse of a 2-by-2 matrix</summary>
    public Mat22 Inverse()
    {
        float det = cx.x * cy.y - cy.x * cx.y; if (det != 0) det = 1 / det;
        return new(new(det * cy.y, -det * cx.y), new(-det * cy.x, det * cx.x));
    }
    /// <summary>Solve A * x = b, where b is a column vector. This is more efficient
    /// than computing the inverse in one-shot cases.</summary>
    public Vector2 Solve(Vector2 b)
    {
        float det = cx.x * cy.y - cy.x * cx.y; if (det != 0) det = 1 / det;
        return new(det * (cy.y * b.x - cy.x * b.y), det * (cx.x * b.y - cx.y * b.x));
    }
    public override string ToString() => $"{{{cx}, {cy}}}";
}
/// <summary>Axis-aligned bounding box</summary>
public struct AABB
{
    public Vector2 lowerBound, upperBound;
    public AABB(Vector2 lowerBound, Vector2 upperBound) { this.lowerBound = lowerBound; this.upperBound = upperBound; }
    /// <summary>Is this a valid bounding box? Not Nan or infinity. Upper bound greater than or equal to lower bound.</summary>
    public bool IsValid()
    {
        Vector2 d = upperBound - lowerBound;
        return d.x >= 0 && d.y >= 0 && lowerBound.IsValid() && upperBound.IsValid();
    }
    /// <summary>Does a fully contain b</summary>
    public bool Contains(AABB b) => lowerBound.x <= b.lowerBound.x && lowerBound.y <= b.lowerBound.y && b.upperBound.x <= upperBound.x && b.upperBound.y <= upperBound.y;
    /// <summary>Get the center of the AABB.</summary>
    public Vector2 Center() => new(0.5f * (lowerBound.x + upperBound.x), 0.5f * (lowerBound.y + upperBound.y));
    /// <summary>Get the extents of the AABB (half-widths).</summary>
    public Vector2 Extents() => new(0.5f * (upperBound.x - lowerBound.x), 0.5f * (upperBound.y - lowerBound.y));
    /// <summary>Union of two AABBs</summary>
    public static AABB Union(AABB a, AABB b) => new(new(Math.Min(a.lowerBound.x, b.lowerBound.x), Math.Min(a.lowerBound.y, b.lowerBound.y)),
        new(Math.Max(a.upperBound.x, b.upperBound.x), Math.Max(a.upperBound.y, b.upperBound.y)));
    /// <summary>Do a and b overlap</summary>
    public static bool Overlaps(AABB a, AABB b) => !(b.lowerBound.x > a.upperBound.x || b.lowerBound.y > a.upperBound.y || a.lowerBound.x > b.upperBound.x || a.lowerBound.y > b.upperBound.y);
    /// <summary>Compute the bounding box of an array of circles</summary>
    public static AABB MakeAABB(Span<Vector2> points, float radius)
    {
        Debug.Assert(points != Span<Vector2>.Empty && !points.IsEmpty);
        AABB a = new(points[0], points[0]);
        for (int i = 1; i < points.Length; i++)
        {
            a.lowerBound = Vector2.Min(a.lowerBound, points[i]);
            a.upperBound = Vector2.Max(a.upperBound, points[i]);
        }
        Vector2 r = new(radius, radius);
        a.lowerBound -= r; a.upperBound += r; return a;
    }
    public CastOutput RayCast(Vector2 p1, Vector2 p2)
    {
        CastOutput output = new();
        float tMin = -float.MaxValue, tMax = float.MaxValue;
        Vector2 d = p2 - p1, absD = d.Abs(), normal = Vector2.Zero;
        if (absD.x < Box2D.FLT_EPSILON)
        {
            if (p1.x < lowerBound.x || upperBound.x < p1.x) return output;
        }
        else
        {
            float inv_d = 1 / d.x, t1 = (lowerBound.x - p1.x) * inv_d, t2 = (upperBound.x - p1.x) * inv_d, s = -1;
            if (t1 > t2) { (t1, t2) = (t2, t1); s = 1; }
            if (t1 > tMin) { normal = new(s, 0); tMin = t1; }
            tMax = Math.Min(tMax, t2);
            if (tMin > tMax) return output;
        }
        if (absD.y < Box2D.FLT_EPSILON)
        {
            if (p1.y < lowerBound.y || upperBound.y < p1.y) return output;
            float inv_d = 1 / d.y, t1 = (lowerBound.y - p1.y) * inv_d, t2 = (upperBound.y - p1.y) * inv_d, s = -1;
            if (t1 > t2) { (t1, t2) = (t2, t1); s = 1; }
            if (t1 > tMin) { normal = new(0, s); tMin = t1; }
            tMax = Math.Min(tMax, t2);
            if (tMin > tMax) return output;
        }
        if (tMin < 0 || 1 < tMin) return output;
        output.fraction = tMin;
        output.normal = normal;
        output.point = Vector2.Lerp(p1, p2, tMin);
        output.hit = true;
        return output;
    }
    public float Perimeter() => 2 * (upperBound.x - lowerBound.x + upperBound.y - lowerBound.y);
    public bool Enlarge(AABB b)
    {
        bool changed = false;
        if (b.lowerBound.x < lowerBound.x) { lowerBound.x = b.lowerBound.x; changed = true; }
        if (b.lowerBound.y < lowerBound.y) { lowerBound.y = b.lowerBound.y; changed = true; }
        if (upperBound.x < b.upperBound.x) { upperBound.x = b.upperBound.x; changed = true; }
        if (upperBound.y < b.upperBound.y) { upperBound.y = b.upperBound.y; changed = true; }
        return changed;
    }
    /// <summary>Narrow a world coordinate to float, rounding toward negative infinity. Use with
    /// b2RoundUpFloat to build a conservative float box that always contains double bounds,
    /// where plain rounding far from the origin could clip. nextafterf is an exact IEEE
    /// operation, so this is cross-platform deterministic. With large world mode off this is
    /// a plain conversion.</summary>
    public static float RoundDownFloat(double x)
    {
#if BOX2D_DOUBLE_PRECISION
        float f = (float)x;
        return f > x ? MathF.BitIncrement(f) : f;
#else
        return (float)x;
#endif
    }
    /// <summary>Narrow a world coordinate to float, rounding toward positive infinity.</summary>
    public static float RoundUpFloat(double x)
    {
#if BOX2D_DOUBLE_PRECISION
        float f = (float)x;
        return f < x ? MathF.BitDecrement(f) : f;
#else
        return (float)x;
#endif
    }
    /// <summary>Translate a relative AABB into world space, rounding outward so the float box always contains
    /// the true box far from the origin where a float coordinate cannot resolve the shape extent.
    /// Float ULP at 1e8 dwarfs the AABB margin, plain truncation could clip a shape out of its own
    /// box. The broadphase pair order rides on the deterministic directed rounding.</summary>
    public AABB Offset(Position origin) => new(new(RoundDownFloat(origin.x + (double)lowerBound.x), RoundDownFloat(origin.y + (double)lowerBound.y)),
        new(RoundUpFloat(origin.x + (double)upperBound.x), RoundUpFloat(origin.y + (double)upperBound.y)));
    public override string ToString() => $"[{lowerBound}, {upperBound}]";
    public unsafe AABBV LoadAABBV()
    {
        fixed (float* aabb = &lowerBound.x)
            return new() { aabb = AdvSimd.IsSupported ? AdvSimd.LoadVector128(aabb) : Sse.IsSupported ? Sse.LoadVector128(aabb) : Vector128.Load(aabb) };
    }
}
/// <summary>separation = dot(normal, point) - offset</summary>
public struct Plane
{
    public Vector2 normal; public float offset;
    public Plane(Vector2 normal, float offset) { this.normal = normal; this.offset = offset; }
    /// <summary>Is this a valid plane? Normal is a unit vector. Not Nan or infinity.</summary>
    public bool IsValid() => normal.IsValid() && normal.IsNormalized() && float.IsFinite(offset);
    /// <summary>Signed separation of a point from a plane</summary>
    public float PlaneSeparation(Vector2 point) => Vector2.Dot(normal, point) - offset;
    public override string ToString() => $"{{{normal}, {offset}}}";
}
public static partial class Box2D
{
    public const float FLT_EPSILON = 1.192092896e-07F;
    /// <summary>One-dimensional mass-spring-damper simulation. Returns the new velocity given the position and time step.
    /// You can then compute the new position using:
    /// position += timeStep * newVelocity
    /// This drives towards a zero position. By using implicit integration we get a stable solution
    /// that doesn't require transcendental functions.</summary>
    public static float SpringDamper(float hertz, float dampingRatio, float position, float velocity, float timeStep)
    {
        float omega = MathF.Tau * hertz;
        float omegaH = omega * timeStep;
        return (velocity - omega * omegaH * position) / (1 + 2 * dampingRatio * omegaH + omegaH * omegaH);
    }

    //LengthUnitsPerMeter was moved to Constants.cs, so it initializes before Huge

    /// <summary>https://en.wikipedia.org/wiki/Floor_and_ceiling_functions</summary>
    public static int ClampInt(int numerator, int denominator)
    {
        Debug.Assert(denominator > 0 && numerator >= 0);
        return (numerator + denominator - 1) / denominator;
    }
}