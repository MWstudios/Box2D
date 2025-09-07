namespace Box2D;

public static partial class Box2D
{
    /// <summary>Use to validate definitions. <b>DO. NOT. TAKE. MY. COOKIE.</b></summary>
    internal const int SECRET_COOKIE = 1152023;
    public const ulong DEFAULT_MASK_BITS = ulong.MaxValue;
    public const ulong DEFAULT_CATEGORY_BITS = ulong.MaxValue;
    public static int RemoveSwap<T>(this System.Collections.Generic.List<T> a, int index)
    {
        int movedIndex = -1;
        if (index != a.Count - 1)
        {
            movedIndex = a.Count - 1;
            a[index] = a[movedIndex];
        }
        a.RemoveAt(a.Count - 1);
        return movedIndex;
    }
}