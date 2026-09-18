using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;

namespace Box2D;

public unsafe struct StackEntry
{
    public void* data;
    public string name = string.Empty;
    public int size = 0;
    public bool usedMalloc = false;
    public StackEntry() { }
}
public unsafe class B2Stack
{
    public void* data;
    public int capacity;
    public int index = 0;
    public int allocation = 0;
    public int maxAllocation = 0;
    public List<StackEntry> entries = new(32);
    public B2Stack(int capacity)
    {
        data = NativeMemory.AlignedAlloc((nuint)(this.capacity = capacity), 64);
    }
    public void Destroy() { NativeMemory.AlignedFree(data); }
    public void* Alloc(int size, string name)
    {
        int alignedSize = ((size - 1) | 0x3F) + 1;
        StackEntry entry = new() { size = alignedSize, name = name };
        if (index + alignedSize > capacity)
        {
            entry.data = NativeMemory.AlignedAlloc((nuint)alignedSize, 64);
            entry.usedMalloc = true;
            Debug.Assert(((nint)entry.data & 0x3F) == 0);
        }
        else
        {
            entry.data = (void*)((nint)data + index);
            entry.usedMalloc = false;
            index += alignedSize;
            Debug.Assert(((nint)data & 0x3F) == 0);
        }
        allocation += alignedSize;
        if (allocation > maxAllocation) maxAllocation = allocation;
        entries.Add(entry);
        return entry.data;
    }
    public void Free(void* mem)
    {
        int entryCount = entries.Count;
        Debug.Assert(entryCount > 0);
        StackEntry entry = entries[entryCount - 1];
        Debug.Assert(mem == entry.data);
        if (entry.usedMalloc) NativeMemory.AlignedFree(mem);
        else index -= entry.size;
        allocation -= entry.size;
        entries.RemoveAt(entries.Count - 1);
    }
    public void Grow()
    {
        Debug.Assert(allocation == 0);
        if (maxAllocation > capacity)
        {
            NativeMemory.AlignedFree(data);
            capacity = maxAllocation + maxAllocation / 2;
            data = NativeMemory.AlignedAlloc((nuint)capacity, 64);
        }
    }
    public int GetCapacity() => capacity;
    public int GetAllocation() => allocation;
    public int GetMaxAllocation() => maxAllocation;
}