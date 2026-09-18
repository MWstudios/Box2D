using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace Box2D;

public struct CandidatePair
{
    public const int Batch = 32;
    public int shapeIdA, shapeIdB;
}
public class PairContext
{
    public World world;
    public List<ulong> pairKeys;
    [System.Runtime.CompilerServices.InlineArray(CandidatePair.Batch)] public struct CandidateArray { public CandidatePair candidate; }
    [System.Runtime.CompilerServices.InlineArray(CandidatePair.Batch)] public struct ULongArray { public ulong v; }
    public CandidateArray batch;
    int batchCount;
    public void FlushCandidatePairs()
    {
        int count1 = batchCount;
        batchCount = 0;
        //ULongArray keys = new(), hashes = new();
        BroadPhase bp = world.broadPhase;
        CandidateArray candidates = new();
        int count2 = 0;
        for (int i = 0; i < count1; i++)
        {
            ref CandidatePair candidate = ref batch[i];
            if (!bp.pairSet.Contains(BroadPhase.B2_SHAPE_PAIR_KEY((ulong)candidate.shapeIdA, (ulong)candidate.shapeIdB)))
                candidates[count2++] = batch[i];
        }
        /*for (int i = 0; i < count2; i++)
        {
            Prefetch(world.shapes[candidates[i].shapeIdA]);
            Prefetch(world.shapes[candidates[i].shapeIdB]);
        }*/
        for (int i = 0; i < count2; i++)
        {
            int shapeIdA = candidates[i].shapeIdA, shapeIdB = candidates[i].shapeIdB;
            Shape shapeA = world.shapes[shapeIdA], shapeB = world.shapes[shapeIdB];
            int bodyIdA = shapeA.bodyId, bodyIdB = shapeB.bodyId;
            if (bodyIdA == bodyIdB) continue;
            if (shapeA.sensorIndex != -1 || shapeB.sensorIndex != -1) continue;
            if (!Shape.ShouldShapesCollide(shapeA.filter, shapeB.filter)) continue;
            if (!ContactRegister.CanCollide(shapeA.type, shapeB.type)) continue;
            Body bodyA = world.bodies[bodyIdA], bodyB = world.bodies[bodyIdB];
            if (!world.ShouldBodiesCollide(bodyA, bodyB)) continue;
            if (shapeA.enableCustomFiltering || shapeB.enableCustomFiltering)
            {
                CustomFilterFcn customFilterFcn = world.customFilterFcn;
                if (customFilterFcn != null && !customFilterFcn(new() { index1 = shapeIdA + 1, world0 = world, generation = shapeA.generation },
                            new() { index1 = shapeIdB + 1, world0 = world, generation = shapeB.generation }, world.customFilterContext))
                        continue;
            }
            pairKeys.Add(BroadPhase.B2_SHAPE_PAIR_KEY((ulong)shapeIdA, (ulong)shapeIdB));
        }
    }
    public void AddCandidatePair(int shapeIdA, int shapeIdB)
    {
        ref CandidatePair candidate = ref batch[batchCount];
        candidate.shapeIdA = Math.Min(shapeIdA, shapeIdB);
        candidate.shapeIdB = Math.Max(shapeIdA, shapeIdB);
        batchCount++;
        if (batchCount == 32) FlushCandidatePairs();
    }
    public static bool TestPair(ref TreeNode a, ref TreeNode b) => ((a.flagIndex | b.flagIndex) & DynamicTree.MovedNode) != 0 && AABB.Overlaps(a.aabb, b.aabb);
    public void CollideProxyAndSubtree(ref TreeNode proxy, Span<TreeNode> nodes, int pair)
    {
        uint proxyMark = proxy.flagIndex & DynamicTree.MovedNode;
        AABBV boxv = proxy.aabb.LoadAABBV();
        int shapeId = proxy.shapeIndex;
        Stack<int> stack = new(1024); stack.Push(pair);
        while (stack.Count > 0)
        {
            pair = stack.Pop();
            for (int i = 0; i < 2; i++)
            {
                ref TreeNode node = ref nodes[pair + i];
                if (((node.flagIndex | proxyMark) & DynamicTree.MovedNode) == 0) continue;
                if (!boxv.OverlapNode(ref node)) continue;
                if (node.IsLeaf()) AddCandidatePair(shapeId, node.shapeIndex);
                else stack.Push(node.GetLeftChild());
            }
        }
    }
    /// <summary>Helper for b2CollideCrossPairs to avoid code duplication.</summary>
    /// <param name="stack"></param>
    public void VisitPair(Span<TreeNode> arrayA, Span<TreeNode> arrayB, ref TreeNode nodeA, ref TreeNode nodeB, Stack<IndexPair> stack)
    {
        if (!TestPair(ref nodeA, ref nodeB)) return;
        bool leafA = nodeA.IsLeaf(), leafB = nodeB.IsLeaf();
        if (leafA && leafB) AddCandidatePair(nodeA.shapeIndex, nodeB.shapeIndex);
        else if (leafA) CollideProxyAndSubtree(ref nodeA, arrayB, nodeB.GetLeftChild());
        else if (leafB) CollideProxyAndSubtree(ref nodeB, arrayA, nodeA.GetLeftChild());
        else stack.Push(new() { a = nodeA.GetLeftChild(), b = nodeB.GetLeftChild() });
    }
    /// <summary>This collides two sub-trees against each other. They can live in the same dynamic tree.
    /// This can only generate pairs cross sub-tree, but not within a sub-tree. This fact means
    /// this does not generate duplicate pairs.
    /// For example consider the full binary tree A (B (D  E) C (F G))
    /// Colliding children of A (B and C) can give pairs (D,F) (D,G) (E,F) and (E,G).
    /// Then colliding children of B can give the pair (D,E) and for C (F,G).
    /// So no duplicates even when used for self-collision.
    /// Whenever a proxy is moved, the flag is propagated up the hierarchy to the root. So
    /// self collision gathers all those moved internal nodes and collides their subtrees together.
    /// See Real-time collision detection section 6.3.2. This is faster than querying every moved
    /// proxy against the whole tree. Scaling is linear instead of linear * log.
    /// Many other physics engines do this (Bepu, Rapier, etc). So nothing new here.</summary>
    public void CollideCrossPairs(Span<TreeNode> arrayA, Span<TreeNode> arrayB, ref TreeNode subtreeA, ref TreeNode subTreeB)
    {
        Stack<IndexPair> stack = new(1024);
        VisitPair(arrayA, arrayB, ref subtreeA, ref subTreeB, stack);
        while (stack.Count > 0)
        {
            IndexPair pair = stack.Pop();
            for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++)
                VisitPair(arrayA, arrayB, ref arrayA[pair.a + i], ref arrayB[pair.b + j], stack);
        }
    }
}
public struct NodePair { public TreeNode a, b; }
public struct IndexPair { public int a, b; }
public partial class DynamicTree
{
    public const int CrossSeedCount = 64;
    [System.Runtime.CompilerServices.InlineArray(CrossSeedCount)] struct NodePairQueue { NodePair pair; }
    public static unsafe int GatherCrossSeeds(DynamicTree treeA, DynamicTree treeB, Span<NodePair> seeds)
    {
        NodePairQueue queue = new();
        int mask = 2 * CrossSeedCount - 1, head = 0, tail = 0;
        ref TreeNode rootA = ref treeA.nodes[RootNode], rootB = ref treeB.nodes[RootNode];
        if (PairContext.TestPair(ref rootA, ref rootB))
            queue[tail++ & mask] = new() { a = rootA, b = rootB };
        int seedCount = 0;
        fixed (TreeNode* nodesA = treeA.nodes, nodesB = treeB.nodes)
            while (head < tail && seedCount + (tail - head) + 3 < CrossSeedCount)
            {
                NodePair pair = queue[head++ & mask];
                if (pair.a.IsLeaf() || pair.b.IsLeaf()) { seeds[seedCount++] = pair; continue; }
                TreeNode* a = nodesA + pair.a.GetLeftChild();
                TreeNode* b = nodesB + pair.b.GetLeftChild();
                for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++) if (PairContext.TestPair(ref a[i], ref b[j]))
                    queue[tail++ & mask] = new() { a = a[i], b = b[j] };
            }
        while (head < tail) seeds[seedCount++] = queue[head++ & mask];
        return seedCount;
    }
}
public class CrossContext
{
    [System.Runtime.CompilerServices.InlineArray(2 * DynamicTree.CrossSeedCount)] public struct CrossSeeds { NodePair pair; }
    public World world;
    public CrossSeeds seeds;
    public int staticSeedCount;
}
/// <summary>The broad-phase is used for computing pairs and performing volume queries and ray casts.
/// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
/// It is up to the client to consume the new pairs and to track subsequent overlap.</summary>
public class BroadPhase
{
    static BodyType B2_PROXY_TYPE(int KEY) => (BodyType)((KEY) & 3);
    static int B2_PROXY_ID(int KEY) => KEY >> 2;
    static int B2_PROXY_KEY(int ID, int TYPE) => (ID << 2) | TYPE;
    public static ulong B2_SHAPE_PAIR_KEY(ulong K1, ulong K2) => K1 < K2 ? (K1 << 32) | K2 : (K2 << 32) | K1;
    public DynamicTree[] trees = new DynamicTree[3];
    /// <summary>The moved siblings gathered from the dynamic body tree.</summary>
    public int[] movedSiblings = null;
    /// <summary>Tracks shape pairs that have a b2Contact
    /// todo pairSet can grow quite large on the first time step and remain large</summary>
    public HashSet<ulong> pairSet;
    public BroadPhase(ref Capacity capacity)
    {
        pairSet = new(Math.Max(32, 2 * capacity.contactCount));
        trees[(int)BodyType.Static] = new(Math.Max(16, capacity.staticShapeCount));
        trees[(int)BodyType.Kinematic] = new(16);
        trees[(int)BodyType.Dynamic] = new(Math.Max(16, capacity.dynamicShapeCount));
    }
    public void Destroy()
    {
        for (int i = 0; i < trees.Length; i++) trees[i].Destroy();
    }
    public int CreateProxy(BodyType proxyType, AABB aabb, ulong categoryBits, int shapeIndex, bool forcePairCreation)
    {
        Debug.Assert(0 <= proxyType && (int)proxyType < 3);
        int proxyId = trees[(int)proxyType].CreateProxyInternal(aabb, categoryBits, (ulong)shapeIndex, proxyType != BodyType.Static || forcePairCreation);
        int proxyKey = B2_PROXY_KEY(proxyId, (int)proxyType);
        return proxyKey;
    }
    public void DestroyProxy(int proxyKey)
    {
        BodyType proxyType = B2_PROXY_TYPE(proxyKey);
        int proxyId = B2_PROXY_ID(proxyKey);
        Debug.Assert(0 <= proxyType && (int)proxyType < 3);
        trees[(int)proxyType].DestroyProxy(proxyId);
    }
    public void MoveProxy(int proxyKey, AABB aabb)
    {
        BodyType proxyType = B2_PROXY_TYPE(proxyKey);
        int proxyId = B2_PROXY_ID(proxyKey);
        trees[(int)proxyType].MoveProxyInternal(proxyId, aabb, true);
    }
    public void ValidateBroadphase()
    {
        trees[(int)BodyType.Dynamic].Validate();
        trees[(int)BodyType.Kinematic].Validate();
    }
    public void ValidateNoEnlarged()
    {
        for (int j = 0; j < 3; j++) trees[j].ValidateNoMoved();
    }
    public void MarkProxyMovedSerial(int proxyKey) => trees[(int)B2_PROXY_TYPE(proxyKey)].MarkProxyMovedSerial(B2_PROXY_ID(proxyKey));
    public void MarkProxyMoved(int proxyKey, AABB aabb) => trees[(int)B2_PROXY_TYPE(proxyKey)].MarkProxyMoved(B2_PROXY_ID(proxyKey), aabb);
}
public partial class World
{
    static BodyType B2_PROXY_TYPE(int KEY) => (BodyType)((KEY) & 3);
    static int B2_PROXY_ID(int KEY) => KEY >> 2;
    static int B2_PROXY_KEY(int ID, int TYPE) => (ID << 2) | TYPE;
    static void SelfPairsTask(int startIndex, int endIndex, int workerIndex, object context)
    {
        World world = (World)context;
        DynamicTree tree = world.broadPhase.trees[(int)BodyType.Dynamic];
        PairContext pairContext = new() { world = world, pairKeys = world.taskContexts[workerIndex].pairKeys };
        for (int i = startIndex; i < endIndex; i++)
        {
            int nodeIndex = world.broadPhase.movedSiblings[i];
            pairContext.CollideCrossPairs(tree.nodes, tree.nodes, ref tree.nodes[nodeIndex], ref tree.nodes[nodeIndex + 1]);
        }
        pairContext.FlushCandidatePairs();
    }
    static void CrossPairsTask(int startIndex, int endIndex, int workerIndex, object context)
    {
        CrossContext crossContext = (CrossContext)context;
        World world = crossContext.world;
        TreeNode[] staticNodes = world.broadPhase.trees[(int)BodyType.Static].nodes;
        TreeNode[] kinematicNodes = world.broadPhase.trees[(int)BodyType.Kinematic].nodes;
        TreeNode[] dynamicNodes = world.broadPhase.trees[(int)BodyType.Dynamic].nodes;
        PairContext pairContext = new() { world = world, pairKeys = world.taskContexts[workerIndex].pairKeys };
        for (int i = startIndex; i < endIndex; ++i)
        {
            TreeNode[] nodesB = i < crossContext.staticSeedCount ? staticNodes : kinematicNodes;
            ref NodePair seed = ref crossContext.seeds[i];
            pairContext.CollideCrossPairs(dynamicNodes, nodesB, ref seed.a, ref seed.b);
        }
        pairContext.FlushCandidatePairs();
    }
    static void UpdateTreesTask(object context)
    {
        var trees = ((World)context).broadPhase.trees;
        trees[(int)BodyType.Dynamic].Rebuild(false);
        trees[(int)BodyType.Kinematic].Rebuild(false);
    }
    public void EnqueueTreeUpdate()
    {
        if (taskCount < Box2D.MaxTasks)
        {
            userTreeTask = enqueueTaskFcn(UpdateTreesTask, this, userTaskContext);
            taskCount++;
            activeTaskCount += userTreeTask == null ? 0 : 1;
        }
        else
        {
            userTreeTask = null;
            UpdateTreesTask(this);
        }
    }
    public unsafe void UpdateBroadPhasePairs()
    {
        BroadPhase bp = broadPhase;
        bool needUpdate = bp.trees[(int)BodyType.Static].HasTreeMoved()
            || bp.trees[(int)BodyType.Kinematic].NeedsRebuild()
            || bp.trees[(int)BodyType.Dynamic].NeedsRebuild();
        if (!needUpdate) return;
        for (int i = 0; i < workerCount; i++) taskContexts[i].pairKeys.Clear();
        {
            DynamicTree dynamicTree = bp.trees[(int)BodyType.Dynamic];
            int pairCapacity = dynamicTree.nodeEnd / 2;
            bp.movedSiblings = new int[pairCapacity];
            int dynamicMoveCount = dynamicTree.GatherMovedSiblings(bp.movedSiblings);
            CrossContext.CrossSeeds crossSeeds = new();
            int staticSeedCount = DynamicTree.GatherCrossSeeds(dynamicTree, bp.trees[(int)BodyType.Static], crossSeeds);
            Debug.Assert(staticSeedCount <= DynamicTree.CrossSeedCount);
            int kinematicSeedCount = DynamicTree.GatherCrossSeeds(dynamicTree, bp.trees[(int)BodyType.Static], ((Span<NodePair>)crossSeeds).Slice(staticSeedCount));
            Debug.Assert(kinematicSeedCount <= DynamicTree.CrossSeedCount);
            int crossMoveCount = staticSeedCount + kinematicSeedCount;
            CrossContext crossContext = new() { world = this, seeds = crossSeeds, staticSeedCount = staticSeedCount };
            ParallelFor(CrossPairsTask, crossMoveCount, 1, crossContext);
            ParallelFor(SelfPairsTask, dynamicMoveCount, 64, this);
        }
        bp.trees[(int)BodyType.Static].ClearMoved();
        EnqueueTreeUpdate();
        int pairCount = 0;
        for (int i = 0; i < workerCount; i++) pairCount += taskContexts[i].pairKeys.Count;
        ulong[] pairKeys = new ulong[Math.Max(pairCount, 1)];
        int keyCount = 0;
        fixed (ulong* p = pairKeys) for (int i = 0; i < workerCount; i++)
        {
            List<ulong> workerKeys = taskContexts[i].pairKeys;
            fixed (ulong* w = System.Runtime.InteropServices.CollectionsMarshal.AsSpan(workerKeys))
                if (workerKeys.Count > 0)
                {
                    Buffer.MemoryCopy(w, p + keyCount, workerKeys.Count * sizeof(ulong), workerKeys.Count * sizeof(ulong));
                    keyCount += workerKeys.Count;
                }
        }
        Debug.Assert(keyCount == pairCount);
        for (int i = 0; i < keyCount; i++)
        {
            int shapeIdA = (int)(pairKeys[i] >> 32), shapeIdB = (int)(pairKeys[i] & 0xFFFFFFFF);
            CreateContact(shapes[shapeIdA], shapes[shapeIdB]);
        }
        bp.movedSiblings = null;
        ValidateSolverSets();
    }
}