using System.Diagnostics;
using System.Runtime.Intrinsics.X86;
using System.Threading;

namespace Box2D;

public enum SchedulerTaskStatus { Free, Pending, Claimed, Complete }
public class SchedulerTask
{
    public TaskCallback callback;
    public object taskContext;
    public uint status;
}
public struct SchedulerWorkerContext
{
    public Scheduler scheduler;
    public int threadIndex;
}
public class Scheduler
{
    public Thread[] threads = new Thread[Box2D.MaxWorkers];
    public SchedulerWorkerContext[] workerContexts = new SchedulerWorkerContext[Box2D.MaxWorkers];
    /// <summary>total workers including main thread</summary>
    public int workerCount;
    /// <summary>threads created = workerCount - 1</summary>
    public int threadCount;
    public SchedulerTask[] tasks = new SchedulerTask[Box2D.MaxTasks];
    public int nextSlot;
    public Semaphore taskSemaphore;
    public int shutdown;
    /// <summary>Try to claim and execute one pending task.</summary>
    /// <returns>Returns true if work was performed, false otherwise.</returns>
    public bool ExecuteOne()
    {
        int taskCount = Interlocked.Add(ref nextSlot, 0);
        for (int t = 0; t < taskCount; t++)
        {
            SchedulerTask task = tasks[t];
            if (Interlocked.Add(ref task.status, 0) != (int)SchedulerTaskStatus.Pending) continue;
            if (Interlocked.CompareExchange(ref task.status, (int)SchedulerTaskStatus.Claimed, (int)SchedulerTaskStatus.Pending) != (int)SchedulerTaskStatus.Pending) continue;
            task.callback(task.taskContext);
            Interlocked.Exchange(ref task.status, (int)SchedulerTaskStatus.Complete);
            return true;
        }
        return false;
    }
    /// <summary>Background worker thread entry point.</summary>
    public static void WorkerMain(ref SchedulerWorkerContext context)
    {
        Scheduler scheduler = context.scheduler;
        while (true)
        {
            scheduler.taskSemaphore.WaitOne();
            if (Interlocked.Add(ref scheduler.shutdown, 0) != 0) break;
            while (scheduler.ExecuteOne()) ;
        }
    }
    public Scheduler(int workerCount)
    {
        Debug.Assert(0 < workerCount && workerCount <= Box2D.MaxWorkers);
        this.workerCount = workerCount;
        threadCount = workerCount - 1;
        taskSemaphore = new(0, int.MaxValue);
        Interlocked.Exchange(ref shutdown, 0);
        Interlocked.Exchange(ref nextSlot, 0);
        for (int i = 0; i < tasks.Length; i++) tasks[i] = new();
        for (int i = 0; i < threadCount; i++)
        {
            workerContexts[i] = new() { scheduler = this, threadIndex = i + 1 };
            threads[i] = new(() => WorkerMain(ref workerContexts[i])) { Name = $"box2d_worker_{i + 1}" };
        }
    }
    public void Destroy()
    {
        Interlocked.Exchange(ref shutdown, 1);
        taskSemaphore.Release(threadCount);
        for (int i = 0; i < threadCount; i++) threads[i].Join();
        taskSemaphore.Dispose();
    }
    public void Reset() => Interlocked.Exchange(ref nextSlot, 0);
}
public partial class World
{
    public static object SchedulerEnqueueTask(TaskCallback task, object taskContext, object userContext)
    {
        Scheduler scheduler = (Scheduler)userContext;
        int slot = Interlocked.Increment(ref scheduler.nextSlot) - 1;
        Debug.Assert(slot < Box2D.MaxTasks);
        SchedulerTask schedulerTask = scheduler.tasks[slot];
        schedulerTask.callback = task;
        schedulerTask.taskContext = taskContext;
        Interlocked.Exchange(ref schedulerTask.status, (int)SchedulerTaskStatus.Pending);
        scheduler.taskSemaphore.Release();
        return schedulerTask;
    }
    public static void SchedulerFinishTask(object userTask, object userContext)
    {
        if (userTask == null) return;
        Scheduler scheduler = (Scheduler)userContext;
        SchedulerTask waitTask = (SchedulerTask)userTask;
        while (Interlocked.Add(ref waitTask.status, 0) != (int)SchedulerTaskStatus.Complete)
            if (!scheduler.ExecuteOne()) X86Base.Pause();
    }
}
