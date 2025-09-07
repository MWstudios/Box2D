using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace Box2D;

public class IDPool
{
    List<int> freeArray = new(32);
    int nextIndex = 0;
    public int GetIdCount() => nextIndex - freeArray.Count;
    public int GetIdCapacity() => nextIndex;
    public int GetByteCount() => freeArray.Count * sizeof(int);
    public int AllocId()
    {
        int count = freeArray.Count;
        if (count > 0)
        {
            int id = freeArray[^1];
            freeArray.RemoveAt(freeArray.Count - 1);
            return id;
        }
        {
            int id = nextIndex;
            nextIndex++;
            return id;
        }
    }
    public void FreeId(int id)
    {
        Debug.Assert(nextIndex > 0);
        Debug.Assert(0 <= id && id < nextIndex);
        freeArray.Add(id);
    }
    public void ValidateFreeID(int id)
    {
        int freeCount = freeArray.Count;
        for (int i = 0; i < freeCount; i++)
            if (freeArray[i] == id) return;
        throw new IndexOutOfRangeException("Invalid free pool ID");
    }
    public void ValidateUsedID(int id)
    {
        int freeCount = freeArray.Count;
        for (int i = 0; i < freeCount; i++)
            if (freeArray[i] == id)
                throw new IndexOutOfRangeException("Invalid free pool ID");
    }
}
