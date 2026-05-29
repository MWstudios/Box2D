using System;
using System.Diagnostics;
using System.Threading;

namespace Box2D;

/// <summary>Callback invoked by b2ParallelFor to process a range of items. May be called
/// multiple times per worker: work is divided into blocks that workers claim
/// atomically, so a worker that finishes early picks up the next unclaimed
/// block instead of sitting idle. workerIndex is the worker identity and is
/// stable across all invocations from the same worker, so it is safe to use as
/// an index into per-worker state (e.g. world->taskContexts.data + workerIndex).</summary>
public delegate void ParallelForCallback(int startIndex, int endIndex, int workerIndex, object context);
/// <summary>Shared state for one b2ParallelFor invocation. Workers race on nextBlock to
/// claim work, so a slow chunk can't strand the other threads.</summary>
public class ParallelForShared
{
    public int nextBlock, blockCount, blockSize, itemCount;
    public ParallelForCallback callback;
    public object context;
}
public class ParallelForTask
{
    public ParallelForShared shared;
    public int workerIndex;
}
public partial class World
{
    public static void ParallelForTrampoline(object taskContext)
    {
        ParallelForTask task = (ParallelForTask)taskContext;
        ParallelForShared shared = task.shared;
        while (true)
        {
            int blockIndex = Interlocked.Increment(ref shared.nextBlock) - 1;
            if (blockIndex >= shared.blockCount) break;
            int start = blockIndex * shared.blockSize;
            int end = start + shared.blockSize;
            if (end >  shared.itemCount) end = shared.itemCount;
            shared.callback(start, end, task.workerIndex, shared.context);
        }
    }
    /// <summary>Divide [0, itemCount) into blocks and process them with cooperative claiming:
    /// up to world->workerCount tasks are enqueued, and each task loops, atomically
    /// claiming the next unclaimed block until the range is drained. Blocks the
    /// caller until all work is complete. minRange is the minimum block size; block
    /// size grows once itemCount exceeds 4 * workerCount * minRange so block count
    /// stays bounded.</summary>
    public void ParallelFor(ParallelForCallback callback, int itemCount, int minRange, object context)
    {
        if (itemCount <= 0) return;
        Debug.Assert(minRange > 0);
        Debug.Assert(0 < workerCount && workerCount <= Box2D.MaxWorkers);
        int blocksPerWorker = 4;
        int maxBlockCount = blocksPerWorker * workerCount;
        int blockSize = itemCount <= minRange * maxBlockCount ? minRange : (itemCount + maxBlockCount - 1) / maxBlockCount;
        int blockCount = (itemCount + blockSize - 1) / blockSize;
        Debug.Assert(blockCount >= 1);
        Debug.Assert(blockSize * blockCount >= itemCount);
        int taskCount = workerCount < blockCount ? workerCount : blockCount;
        ParallelForShared shared = new() { blockCount = blockCount, blockSize = blockSize, itemCount = itemCount, callback = callback, context = context };
        ParallelForTask[] tasks = new ParallelForTask[Box2D.MaxWorkers];
        object[] handles = new object[Box2D.MaxWorkers];
        for (int i = 0; i < taskCount; i++)
        {
            tasks[i] = new() { shared = shared, workerIndex = i };
            if (taskCount < Box2D.MaxTasks)
            {
                handles[i] = enqueueTaskFcn(ParallelForTrampoline, tasks[i], userTaskContext);
                this.taskCount++;
                activeTaskCount += handles[i] == null ? 0 : 1;
            }
            else
            {
                handles[i] = null;
                ParallelForTrampoline(tasks[i]);
            }
        }
        for (int i = 0; i < taskCount; i++)
        {
            if (handles[i] != null)
            {
                finishTaskFcn(handles[i], userTaskContext);
                activeTaskCount--;
            }
        }
    }
}
