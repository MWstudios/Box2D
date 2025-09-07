using System;
using System.Diagnostics;
using System.Runtime.Intrinsics.Arm;
using System.Runtime.Intrinsics.X86;

namespace Box2D;

public class BitSet
{
    public ulong[] bits;
    public uint blockCount;
    public BitSet(uint bitCapacity)
    {
        bits = new ulong[(bitCapacity + 511) >> 9];
        blockCount = 0;
    }
    public void Destroy() { blockCount = 0; }
    public unsafe void SetBitCountAndClear(int bitCount)
    {
        int blockCount = (bitCount + 511) >> 9;
        this.blockCount = (uint)blockCount;
        if (bits.Length < blockCount) bits = new ulong[(bitCount + (bitCount >> 1) + 511) >> 9];
        else fixed (ulong* b = bits) System.Runtime.InteropServices.NativeMemory.Clear(b, (nuint)(blockCount * sizeof(ulong)));
    }
    public void GrowBitSet(int blockCount)
    {
        Debug.Assert(blockCount > this.blockCount);
        if (blockCount > bits.Length)
        {
            ulong[] newBits = new ulong[blockCount + (blockCount >> 1)];
            Buffer.BlockCopy(bits, 0, newBits, 0, bits.Length * sizeof(ulong));
            bits = newBits;
        }
        this.blockCount = (uint)blockCount;
    }
    public int CountSetBits()
    {
        int popCount = 0;
        for (int i = 0; i < blockCount; i++) popCount += CTZ.PopCount64(bits[i]);
        return popCount;
    }
    public unsafe void InPlaceUnion(BitSet setB)
    {
        Debug.Assert(blockCount == setB.blockCount);
        int i = 0;
        fixed (ulong* a = bits) fixed (ulong* b = setB.bits)
        {
            if (Avx.IsSupported)
                for (float* b0 = (float*)b, a0 = (float*)a; i + 8 <= blockCount; i += 8, a0 += 8, b0 += 8)
                    Avx.StoreAligned(a0, Avx.Or(Avx.LoadAlignedVector256(b0), Avx.LoadAlignedVector256(a0)));
            if (Sse.IsSupported)
                for (float* b0 = (float*)b, a0 = (float*)a; i + 4 <= blockCount; i += 4, a0 += 4, b0 += 4)
                    Sse.StoreAligned(a0, Sse.Or(Sse.LoadAlignedVector128(b0), Sse.LoadAlignedVector128(a0)));
            if (AdvSimd.IsSupported)
                for (float* b0 = (float*)b, a0 = (float*)a; i + 4 <= blockCount; i += 4, a0 += 4, b0 += 4)
                    AdvSimd.Store(a0, AdvSimd.Or(AdvSimd.LoadVector128(b0), AdvSimd.LoadVector128(a0)));
        }
        for (; i < blockCount; i++) bits[i] |= setB.bits[i];
    }
    public void SetBit(int bitIndex)
    {
        int blockIndex = bitIndex >> 6;
        Debug.Assert(blockIndex < blockCount);
        bits[blockIndex] |= 1ul << (bitIndex & 63);
    }
    public void SetBitGrow(int bitIndex)
    {
        int blockIndex = bitIndex >> 6;
        if (blockIndex >= blockCount) GrowBitSet(blockIndex + 1);
        bits[blockIndex] |= 1ul << (bitIndex & 63);
    }
    public void ClearBit(int bitIndex)
    {
        int blockIndex = bitIndex >> 6;
        if (blockIndex >= blockCount) return;
        bits[blockIndex] &= ~(1ul << (bitIndex & 63));
    }
    public bool GetBit(int bitIndex)
    {
        int blockIndex = bitIndex >> 6;
        if (blockIndex >= blockCount) return false;
        return (bits[blockIndex] & (1ul << (bitIndex & 63))) != 0;
    }
    public int GetBitSetBytes() => bits.Length * sizeof(ulong);
}
