using System.Numerics;

namespace Box2D;

public static class CTZ
{
    // https://en.wikipedia.org/wiki/Find_first_set

    public static uint CTZ32(uint block) => (uint)BitOperations.TrailingZeroCount(block);

    public static uint CLZ32(uint value) => (uint)BitOperations.LeadingZeroCount(value);

    public static uint CTZ64(ulong block) => (uint)BitOperations.TrailingZeroCount(block);

    public static bool IsPowerOf2(int x) => (x & (x - 1)) == 0;

    public static int BoundingPowerOf2(int x)
    {
        if (x <= 1)
        {
            return 1;
        }

        return 32 - (int)CLZ32((uint)x - 1);
    }

    public static int RoundUpPowerOf2(int x)
    {
        if (x <= 1)
        {
            return 1;
        }

        return 1 << (32 - (int)CLZ32((uint)x - 1));
    }
    public static int PopCount64(ulong block) => BitOperations.PopCount(block);
}
