using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;

namespace Box2D;

public unsafe struct ArenaEntry
{
    public void* data;
    public string name = string.Empty;
    public int size = 0;
    public bool usedMalloc = false;
    public ArenaEntry() { }
}
public unsafe class ArenaAllocator
{
    public void* data;
    public int capacity;
    public int index = 0;
    public int allocation = 0;
    public int maxAllocation = 0;
    public List<ArenaEntry> entries = new(32);
    public ArenaAllocator(int capacity)
    {
        data = NativeMemory.AlignedAlloc((nuint)(this.capacity = capacity), 32);
    }
    public void Destroy() { NativeMemory.AlignedFree(data); }
    public void* AllocateArenaItem(int size, string name)
    {
        int size32 = ((size - 1) | 0x1F) + 1;
        ArenaEntry entry = new() { size = size32, name = name };
        if (index + size32 > capacity)
        {
            entry.data = NativeMemory.AlignedAlloc((nuint)size32, 32);
            entry.usedMalloc = true;
            Debug.Assert(((nint)entry.data & 0x1F) == 0);
        }
        else
        {
            entry.data = (void*)((nint)data + index);
            entry.usedMalloc = false;
            index += size32;
            Debug.Assert(((nint)data & 0x1F) == 0);
        }
        allocation += size32;
        if (allocation > maxAllocation) maxAllocation = allocation;
        entries.Add(entry);
        return entry.data;
    }
    public unsafe void FreeArenaItem(void* mem)
    {
        int entryCount = entries.Count;
        Debug.Assert(entryCount > 0);
        ArenaEntry entry = entries[entryCount - 1];
        Debug.Assert(mem == entry.data);
        if (entry.usedMalloc) NativeMemory.AlignedFree(mem);
        else index -= entry.size;
        allocation -= entry.size;
        entries.RemoveAt(entries.Count - 1);
    }
    public void GrowArena()
    {
        Debug.Assert(allocation == 0);
        if (maxAllocation > capacity)
        {
            NativeMemory.AlignedFree(data);
            capacity = maxAllocation + maxAllocation / 2;
            data = NativeMemory.AlignedAlloc((nuint)capacity, 32);
        }
    }
    public int GetCapacity() => capacity;
    public int GetArenaAllocation() => allocation;
    public int GetMaxAllocation() => maxAllocation;
}