using System.Runtime.InteropServices;

namespace Box2D;

public unsafe class PtrArray<T>
{
    public T* Data; public int Capacity;
    public int Count;
    public PtrArray(int capacity) => Data = (T*)NativeMemory.AlignedAlloc((nuint)((this.Capacity = capacity) * sizeof(T)), 32);
    public void Add(T value)
    {
        if (++Count == Capacity)
            Data = (T*)NativeMemory.AlignedRealloc(Data, (nuint)((Capacity <<= 1) * sizeof(T)), 32);
        Data[Count - 1] = value;
    }
    public int RemoveSwap(int index)
    {
        int movedIndex = -1;
        if (index != Count - 1)
        {
            movedIndex = Count - 1;
            Data[index] = Data[movedIndex];
        }
        Count--;
        return movedIndex;
    }
    public T this[int index] { get => Data[index]; set => Data[index] = value; }
    ~PtrArray() => NativeMemory.AlignedFree(Data);
}
