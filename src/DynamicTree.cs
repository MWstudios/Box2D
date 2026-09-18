using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.Arm;
using System.Runtime.Intrinsics.X86;

namespace Box2D;

/// <summary>A node in the dynamic tree. Siblings sit together at an even index so two nodes fit in
/// a 64 byte cache line. The root is at index zero and index one always empty.</summary>
[StructLayout(LayoutKind.Explicit), DebuggerDisplay("{(IsLeaf() ? $\"Leaf\" : $\"Children {GetLeftChild()}, {GetLeftChild() + 1}\")}")] public struct TreeNode
{
    /// <summary> The node bounding box</summary>
    [FieldOffset(0)] public AABB aabb = new() { lowerBound = new(float.PositiveInfinity, float.PositiveInfinity), upperBound = new(float.NegativeInfinity, float.NegativeInfinity) };
    /// <summary>In 3D this space is used by the AABB z components.</summary>
    [FieldOffset(16)] public ulong padding;
    /// <summary>bit 31 : 1 for leaf node<br/>bit 30 : 1 for moved flag<br/>bits 0-29 : index of the sibling pair node or the proxy id for a leaf</summary>
    [FieldOffset(24)] public uint flagIndex = DynamicTree.EmptyNode;
    /// <summary>The height of an internal node. A leaf has zero height.</summary>
    [FieldOffset(28)] public int height = 0;
    /// <summary>The shape index for a leaf. Truncated from proxy user data.</summary>
    [FieldOffset(28)] public int shapeIndex;
    public TreeNode() { }
    public int GetNodeHeight() => IsLeaf() ? 0 : height;
    public bool IsLeaf() => (flagIndex & DynamicTree.LeafNode) == DynamicTree.LeafNode;
    public bool IsMoved() => (flagIndex & DynamicTree.MovedNode) == DynamicTree.MovedNode;
    public bool IsEmpty() => flagIndex == DynamicTree.EmptyNode;
    public int GetLeftChild() => (int)(flagIndex & DynamicTree.NodeIndexMask);
    public int GetProxyId() => (int)(flagIndex & DynamicTree.NodeIndexMask);
    public void SetLeftChild(int pair) => flagIndex = (flagIndex & ~DynamicTree.NodeIndexMask) | (uint)pair;
}
/// <summary>Separate storage for tree leaves.</summary>
public struct TreeProxy
{
    /// <summary>User data is an index instead of void* because it is used internally as a shape index.</summary>
    public ulong userData;
    /// <summary>Category bits for collision filtering.</summary>
    public ulong categoryBits;
    /// <summary>The leaf node. B2_NULL_INDEX for a free proxy.</summary>
    public int node;
    /// <summary>Next free proxy.</summary>
    public int next;
}
public struct AABBV
{
    public Vector128<float> aabb;
    public unsafe bool OverlapNode(ref TreeNode node)
    {
        if (AdvSimd.IsSupported)
        {
            fixed (AABB* v = &node.aabb)
            {
                Vector128<float> bv = AdvSimd.LoadVector128((float*)v);
                Vector128<float> t1 = Vector128.Create(aabb.GetLower(), bv.GetLower()), t2 = Vector128.Create(aabb.GetUpper(), bv.GetUpper());
                return AdvSimd.Arm64.MinAcross(AdvSimd.CompareLessThanOrEqual(t1, t2).AsUInt32())[0] != 0;
            }
        }
        else if (Sse.IsSupported)
        {
            fixed (AABB* v = &node.aabb)
            {
                Vector128<float> bv = Sse.LoadVector128((float*)v);
                return Sse.MoveMask(Sse.CompareLessThanOrEqual(Sse.MoveLowToHigh(aabb, bv), Sse.MoveHighToLow(aabb, bv))) == 0xF;
            }
        }
        else
        {
            fixed (Vector128<float>* v = &aabb) fixed (AABB* bv = &node.aabb)
            {
                AABB* av = (AABB*)v;
                return av->lowerBound.x <= bv->upperBound.x && av->lowerBound.y <= bv->upperBound.y && bv->lowerBound.x <= av->upperBound.x && bv->lowerBound.y <= av->upperBound.y;
            }
        }
    }
    public static unsafe AABB Union(AABB a, AABB b)
    {
        if (AdvSimd.IsSupported)
        {
            Vector128<float> b1 = AdvSimd.LoadVector128((float*)&a), b2 = AdvSimd.LoadVector128((float*)&b);
            Vector128<float> lower = AdvSimd.Min(b1, b2), upper = AdvSimd.Max(b1, b2);
            AABB result;
            AdvSimd.Store((float*)&result, Vector128.Create(lower.GetLower(), upper.GetUpper()));
            return result;
        }
        if (Sse.IsSupported)
        {
            Vector128<float> b1 = Sse.LoadVector128((float*)&a), b2 = Sse.LoadVector128((float*)&b);
            Vector128<float> lower = Sse.Min(b1, b2), upper = Sse.Max(b1, b2);
            AABB result;
            Sse.Store((float*)&result, Sse.Shuffle(lower, upper, 0b11100100));
            return result;
        }
        return AABB.Union(a, b);
    }
    public static unsafe AABBV UnionPair(TreeNode* pair)
    {
        if (AdvSimd.IsSupported)
        {
            Vector128<float> b1 = AdvSimd.LoadVector128((float*)&pair[0].aabb), b2 = AdvSimd.LoadVector128((float*)&pair[1].aabb);
            return new() { aabb = Vector128.Create(AdvSimd.Min(b1, b2).GetLower(), AdvSimd.Max(b1, b2).GetUpper()) };
        }
        if (Sse.IsSupported)
        {
            Vector128<float> b1 = Sse.LoadVector128((float*)&pair[0].aabb), b2 = Sse.LoadVector128((float*)&pair[1].aabb);
            return new() { aabb = Sse.Shuffle(Sse.Min(b1, b2), Sse.Max(b1, b2), 0b11100100) };
        }
        AABB result = AABB.Union(pair[0].aabb, pair[1].aabb);
        return new() { aabb = Vector128.Load((float*)&result) };
    }
    public static unsafe void Store(ref AABB aabb, AABBV value, bool condition)
    {
        if (AdvSimd.IsSupported)
        {
            fixed (AABB* a = &aabb)
                AdvSimd.Store((float*)a, AdvSimd.BitwiseSelect(AdvSimd.DuplicateToVector128(condition ? 0xFFFFFFFF : 0).AsSingle(),
                    value.aabb, AdvSimd.LoadVector128((float*)a)));
        }
        else if (Sse.IsSupported)
        {
            Vector128<float> mask = Vector128.Create(condition ? 0xFFFFFFFF : 0).AsSingle();
            fixed (AABB* a = &aabb)
                Sse.Store((float*)a, Sse.Or(Sse.And(mask, value.aabb), Sse.AndNot(mask, Sse.LoadVector128((float*)a))));
        }
        else if (condition) aabb = *(AABB*)&value.aabb;
    }
    public static unsafe bool OverlapV(ref AABB a, ref AABB b)
    {
        if (AdvSimd.IsSupported) fixed (AABB* _a = &a, _b = &b)
        {
            Vector128<float> av = AdvSimd.LoadVector128((float*)_a), bv = AdvSimd.LoadVector128((float*)_b);
            Vector128<float> t1 = Vector128.Create(av.GetLower(), bv.GetLower()), t2 = Vector128.Create(bv.GetUpper(), av.GetUpper()); //???
            return AdvSimd.Arm64.MinAcross(AdvSimd.CompareLessThanOrEqual(t1, t2).AsUInt32())[0] != 0;
        }
        if (Sse.IsSupported) fixed (AABB* _a = &a, _b = &b)
        {
            Vector128<float> av = Sse.LoadVector128((float*)_a), bv = Sse.LoadVector128((float*)_b);
            Vector128<float> t1 = Sse.MoveLowToHigh(av, bv), t2 = Sse.MoveHighToLow(av, bv);
            return Sse.MoveMask(Sse.CompareLessThanOrEqual(t1, t2)) == 0xF;
        }
        return a.lowerBound.x <= b.upperBound.x && a.lowerBound.y <= b.upperBound.y && b.lowerBound.x <= a.upperBound.x && b.lowerBound.y <= a.upperBound.y;
    }
}
public partial class DynamicTree
{
    public const int RootNode = 0;
    public const uint MovedNode = 1 << 30;
    public const uint LeafNode = 1u << 31;
    public const uint NodeIndexMask = 0xFFFFFFFF & ~(MovedNode | LeafNode);
    public const uint EmptyNode = NodeIndexMask | LeafNode;
    public int GetRootPair(TreeNode[] nodes)
    {
        ref TreeNode root = ref nodes[RootNode];
        return root.IsLeaf() ? RootNode : root.GetLeftChild();
    }
    public bool HasTreeMoved() => nodes[RootNode].IsMoved();
    public bool NeedsRebuild() => nodes[RootNode].IsMoved() || !dfsOrdered;
    public int GatherMovedSiblings(int[] pairIndices)
    {
        int count = 0;
        for (int pair = 2; pair < nodeEnd; pair += 2)
            if (((nodes[pair].flagIndex | nodes[pair + 1].flagIndex) & MovedNode) != 0)
                pairIndices[count++] = pair;
        return count;
    }
    public int AllocateSiblingPair()
    {
        if (pairFreeList != -1)
        {
            int pair = pairFreeList;
            pairFreeList = parents[pair];
            return pair;
        }
        if (nodeEnd + 2 > nodes.Length)
        {
            int oldCapacity = nodes.Length;
            int newCapacity = oldCapacity + (oldCapacity >> 1);
            newCapacity += newCapacity & 1;
            TreeNode[] oldNodes = nodes; nodes = new TreeNode[newCapacity]; Array.Copy(oldNodes, nodes, oldCapacity);
            int[] oldParents = parents; parents = new int[newCapacity]; Array.Copy(oldParents, parents, oldCapacity);
            swapNodes = null;
        }
        {
            int pair = nodeEnd;
            nodeEnd += 2;
            return pair;
        }
    }
    public void FreePair(int pair)
    {
        Debug.Assert((pair & 1) == 0 && 2 <= pair && pair < nodeEnd);
        nodes[pair] = new();
        nodes[pair + 1] = new();
        parents[pair] = pairFreeList;
        parents[pair + 1] = -1;
        pairFreeList = pair;
    }
    /// <summary>Constructing the tree initializes the node pool.</summary>
    public DynamicTree(int proxyCapacity)
    {
        nodes = new TreeNode[2 * Math.Max(proxyCapacity, 16)];
        parents = new int[2 * Math.Max(proxyCapacity, 16)];
        pairFreeList = -1;
        nodes[RootNode] = new();
        nodes[RootNode + 1] = new();
        parents[RootNode] = -1;
        parents[RootNode + 1] = -1;
        nodeEnd = 2;
        dfsOrdered = true;
        proxies = new TreeProxy[2 * Math.Max(proxyCapacity, 16)];
        for (int i = 0; i < proxies.Length; i++)
        {
            proxies[i].node = -1;
            proxies[i].next = i + 1;
        }
        proxies[^1].node = -1;
        proxies[^1].next = -1;
        proxyFreeList = 0;
        leafIndices = null;
        leafNodes = null;
        leafBoxes = null;
        leafCenters = null;
        binIndices = null;
        rebuildCapacity = 0;
    }
    /// <summary>Destroy the tree, freeing the node pool.</summary>
    public void Destroy() { }
    /// <summary>Allocate a proxy from the pool. Grow the pool if necessary.</summary>
    int AllocateProxy()
    {
        if (proxyFreeList == -1)
        {
            Debug.Assert(proxyCount == nodes.Length);
            TreeProxy[] oldProxies = proxies; proxies = new TreeProxy[proxies.Length + Math.Max(proxies.Length >> 1, 1)];
            Array.Copy(oldProxies, 0, proxies, 0, oldProxies.Length);
            for (int i = oldProxies.Length; i < proxies.Length; i++)
            {
                proxies[i].node = -1;
                proxies[i].next = i + 1;
            }
            proxies[^1].node = -1;
            proxies[^1].next = -1;
            proxyFreeList = oldProxies.Length;
        }
        int proxyIndex = proxyFreeList;
        proxyFreeList = proxies[proxyIndex].next;
        proxies[proxyIndex] = new()
        {
            node = -1,
            next = -1
        };
        proxyCount++;
        return proxyIndex;
    }
    /// <summary>Return a proxy to the pool.</summary>
    void FreeProxy(int proxyId)
    {
        Debug.Assert(0 <= proxyId && proxyId < proxies.Length);
        Debug.Assert(0 < proxyCount);
        proxies[proxyId].node = -1;
        proxies[proxyId].next = proxyFreeList;
        proxyFreeList = proxyId;
        proxyCount--;
    }
    /// <summary>The internal node above a children pair</summary>
    TreeNode MakeInternalNode(Span<TreeNode> nodes, int pair)
    {
        ref TreeNode c1 = ref nodes[pair], c2 = ref nodes[pair + 1];
        return new()
        {
            aabb = AABB.Union(c1.aabb, c2.aabb),
            flagIndex = (uint)pair | ((c1.flagIndex | c1.flagIndex) & MovedNode),
            height = 1 + c1.GetNodeHeight() + c2.GetNodeHeight()
        };
    }
    TreeNode MakeLeafNode(AABB aabb, int proxyId, ulong userData, bool moved) => new()
    {
        aabb = aabb, flagIndex = (uint)proxyId | LeafNode | (moved ? MovedNode : 0), shapeIndex = (int)userData
    };
    /// <summary>A node landed at a new index, so tell what hangs below it</summary>
    void LinkChildren(int nodeIndex)
    {
        ref TreeNode node = ref nodes[nodeIndex];
        if (node.IsLeaf()) proxies[node.GetProxyId()].node = nodeIndex;
        else
        {
            int pair = node.GetLeftChild();
            parents[pair] = nodeIndex;
            parents[pair + 1] = nodeIndex;
        }
    }
    /// <summary>The sweep refit visits indices from high to low, so it needs every child above its parent</summary>
    public bool IsNodeOrdered(Span<TreeNode> nodes, int nodeIndex)
    {
        ref TreeNode node = ref nodes[nodeIndex];
        return node.IsLeaf() || nodeIndex < node.GetLeftChild();
    }
    /// <summary>Greedy algorithm for sibling selection using the SAH<br/>
    /// We have three nodes A-(B,C) and want to add a leaf D, there are three choices.<br/>
    /// 1: make a new parent for A and D : E-(A-(B,C), D)<br/>
    /// 2: associate D with B<br/>
    ///   a: B is a leaf : A-(E-(B,D), C)<br/>
    ///   b: B is an internal node: A-(B{D},C)<br/>
    /// 3: associate D with C<br/>
    ///   a: C is a leaf : A-(B, E-(C,D))<br/>
    ///   b: C is an internal node: A-(B, C{D})<br/>
    /// All of these have a clear cost except when B or C is an internal node. Hence we need to be greedy.<br/>
    /// The cost for cases 1, 2a, and 3a can be computed using the sibling cost formula.<br/>
    /// cost of sibling H = area(union(H, D)) + increased area of ancestors<br/>
    /// Suppose B (or C) is an internal node, then the lowest cost would be one of two cases:<br/>
    /// case1: D becomes a sibling of B<br/>
    /// case2: D becomes a descendant of B along with a new internal node of area(D).</summary>
    int FindBestSibling(AABB boxD)
    {
        Vector2 centerD = boxD.Center();
        float areaD = boxD.Perimeter();
        int nodeIndex = RootNode;
        if (nodes[nodeIndex].IsLeaf()) return nodeIndex;
        AABB rootBox = nodes[nodeIndex].aabb;
        float areaBase = rootBox.Perimeter();
        float directCost = AABB.Union(rootBox, boxD).Perimeter();
        float inheritedCost = 0;
        int bestSibling = nodeIndex;
        float bestCost = directCost;
        while (true)
        {
            int child1 = nodes[nodeIndex].GetLeftChild(), child2 = child1 + 1;
            float cost = directCost + inheritedCost;
            if (cost < bestCost)
            {
                bestSibling = nodeIndex;
                bestCost = cost;
            }
            inheritedCost += directCost - areaBase;
            bool leaf1 = nodes[child1].IsLeaf();
            bool leaf2 = nodes[child2].IsLeaf();
            float lowerCost1 = float.MaxValue;
            AABB box1 = nodes[child1].aabb;
            float directCost1 = AABBV.Union(box1, boxD).Perimeter();
            float area1 = 0;
            if (leaf1)
            {
                float cost1 = directCost1 + inheritedCost;
                if (cost1 < bestCost)
                {
                    bestSibling = child1;
                    bestCost = cost1;
                }
            }
            else
            {
                area1 = box1.Perimeter();
                lowerCost1 = inheritedCost + directCost1 + Math.Min(areaD - area1, 0);
            }
            float lowerCost2 = float.MaxValue;
            AABB box2 = nodes[child2].aabb;
            float directCost2 = AABBV.Union(box2, boxD).Perimeter();
            float area2 = 0;
            if (leaf2)
            {
                float cost2 = directCost2 + inheritedCost;
                if (cost2 < bestCost)
                {
                    bestSibling = child2;
                    bestCost = cost2;
                }
            }
            else
            {
                area2 = box2.Perimeter();
                lowerCost2 = inheritedCost + directCost2 + Math.Min(areaD - area2, 0);
            }
            if (leaf1 && leaf2) break;
            if (bestCost <= lowerCost1 && bestCost <= lowerCost2) break;
            if (lowerCost1 == lowerCost2 && !leaf1)
            {
                Debug.Assert(lowerCost1 < float.MaxValue);
                Debug.Assert(lowerCost2 < float.MaxValue);
                Vector2 d1 = box1.Center() - centerD;
                Vector2 d2 = box2.Center() - centerD;
                lowerCost1 = d1.LengthSquared();
                lowerCost2 = d2.LengthSquared();
            }
            if (lowerCost1 < lowerCost2 && !leaf1)
            {
                nodeIndex = child1;
                areaBase = area1;
                directCost = directCost1;
            }
            else
            {
                nodeIndex = child2;
                areaBase = area2;
                directCost = directCost2;
            }
        }
        return bestSibling;
    }
    enum RotateType { None, BF, BG, CD, CE }
    void SwapNodes(int iDown, int iUp)
    {
        (nodes[iDown], nodes[iUp]) = (nodes[iUp], nodes[iDown]);
        LinkChildren(iDown); LinkChildren(iUp);
        if (!IsNodeOrdered(nodes, iDown) || !IsNodeOrdered(nodes, iUp)) dfsOrdered = false;
        int iC = iDown ^ 1;
        nodes[iC] = MakeInternalNode(nodes, nodes[iC].GetLeftChild());
    }
    /// <summary>Perform a left or right rotation if node A is imbalanced. Returns the new root index.</summary>
    void RotateNodes(int iA)
    {
        ref TreeNode A = ref nodes[iA];
        Debug.Assert(!A.IsLeaf());
        int iB = A.GetLeftChild(), iC = iB + 1;
        ref TreeNode B = ref nodes[iB], C = ref nodes[iC];
        bool leafB = B.IsLeaf(), leafC = C.IsLeaf();
        if (leafB && leafC) return;
        int bestDown = -1, bestUp = -1;
        float bestDelta = 0;
        if (!leafC)
        {
            int iF = C.GetLeftChild(), iG = iF + 1;
            float areaC = C.aabb.Perimeter();
            float deltaBF = AABBV.Union(B.aabb, nodes[iG].aabb).Perimeter() - areaC;
            if (deltaBF > bestDelta) { bestDown = iB; bestUp = iF; bestDelta = deltaBF; }
            float deltaBG = AABBV.Union(B.aabb, nodes[iF].aabb).Perimeter() - areaC;
            if (deltaBG < bestDelta) { bestDown = iB; bestUp = iG; bestDelta = deltaBG; }
        }
        if (!leafB)
        {
            int iD = B.GetLeftChild(), iE = iD + 1;
            float areaB = B.aabb.Perimeter();
            float deltaCD = AABBV.Union(C.aabb, nodes[iE].aabb).Perimeter() - areaB;
            if (deltaCD < bestDelta) { bestDown = iC; bestUp = iD; bestDelta = deltaCD; }
            float deltaCE = AABBV.Union(C.aabb, nodes[iD].aabb).Perimeter() - areaB;
            if (deltaCE < bestDelta) { bestDown = iC; bestUp = iE; bestDelta = deltaCE; }
        }
        if (bestDown != -1) SwapNodes(bestDown, bestUp);
    }
    void InsertLeaf(AABB aabb, int proxyId, bool moved, bool shouldRotate)
    {
        ref TreeProxy proxy = ref proxies[proxyId];
        TreeNode leaf = MakeLeafNode(aabb, proxyId, proxy.userData, moved);
        if (nodes[RootNode].IsEmpty())
        {
            nodes[RootNode] = leaf;
            proxy.node = RootNode;
            return;
        }
        int sibling = FindBestSibling(aabb);
        int pair = AllocateSiblingPair();
        nodes[pair] = nodes[sibling];
        nodes[pair + 1] = leaf;
        parents[pair] = sibling;
        parents[pair + 1] = sibling;
        LinkChildren(pair);
        proxy.node = pair + 1;
        nodes[sibling] = MakeInternalNode(nodes, pair);
        if (!IsNodeOrdered(nodes, sibling) || !IsNodeOrdered(nodes, pair)) dfsOrdered = false;
        int index = sibling;
        while (index != -1)
        {
            if (shouldRotate) RotateNodes(index);
            nodes[index] = MakeInternalNode(nodes, nodes[index].GetLeftChild());
            index = parents[index];
        }
    }
    void RemoveLeaf(int proxyId)
    {
        int leaf = proxies[proxyId].node;
        Debug.Assert(0 <= leaf && leaf < nodeEnd);
        Debug.Assert(nodes[leaf].IsLeaf() && nodes[leaf].GetProxyId() == proxyId);
        if (leaf == RootNode)
        {
            nodes[RootNode] = new();
            return;
        }
        int parent = parents[leaf];
        nodes[parent] = nodes[leaf ^ 1];
        LinkChildren(parent);
        FreePair(leaf & ~1);
        int index = parents[parent];
        while (index != -1)
        {
            nodes[index] = MakeInternalNode(nodes, nodes[index].GetLeftChild());
            index = parents[index];
        }
    }
    public int CreateProxyInternal(AABB aabb, ulong categoryBits, ulong userData, bool markMoved)
    {
        Debug.Assert(aabb.IsValid());
        int proxyId = AllocateProxy();
        ref TreeProxy proxy = ref proxies[proxyId];
        proxy.categoryBits = categoryBits;
        proxy.userData = userData;
        InsertLeaf(aabb, proxyId, markMoved, true);
        return proxyId;
    }
    /// <summary>Create a proxy in the tree as a leaf node. We return the index of the node instead of a pointer so that we can grow the node pool.</summary>
    public int CreateProxy(AABB aabb, ulong categoryBits, ulong userData) => CreateProxyInternal(aabb, categoryBits, userData, false);
    /// <summary>Destroy a proxy. This asserts if the id is invalid.</summary>
    public void DestroyProxy(int proxyId)
    {
        Debug.Assert(0 <= proxyId && proxyId < proxies.Length);
        RemoveLeaf(proxyId);
        FreeProxy(proxyId);
    }
    public void MoveProxyInternal(int proxyId, AABB aabb, bool markMoved)
    {
        Debug.Assert(aabb.IsValid());
        Debug.Assert(aabb.upperBound.x - aabb.lowerBound.x < Box2D.Huge);
        Debug.Assert(aabb.upperBound.y - aabb.lowerBound.y < Box2D.Huge);
        Debug.Assert(0 <= proxyId && proxyId < proxies.Length);
        RemoveLeaf(proxyId);
        InsertLeaf(aabb, proxyId, markMoved, false);
    }
    /// <summary>Move a proxy to a new AABB by removing and reinserting into the tree.</summary>
    public void MoveProxy(int proxyId, AABB aabb) => MoveProxyInternal(proxyId, aabb, false);
    public void EnlargeProxy(int proxyId, AABB aabb)
    {
        Debug.Assert(aabb.IsValid());
        Debug.Assert(aabb.upperBound.x - aabb.lowerBound.x < Box2D.Huge);
        Debug.Assert(aabb.upperBound.y - aabb.lowerBound.y < Box2D.Huge);
        Debug.Assert(0 <= proxyId && proxyId < proxies.Length);
        int index = proxies[proxyId].node;
        ref TreeNode node = ref nodes[index];
        Debug.Assert(node.IsLeaf());
        Debug.Assert(!node.aabb.Contains(aabb));
        node.aabb = aabb;
        node.flagIndex |= MovedNode;
        index = parents[index];
        while (index != -1)
        {
            node = ref nodes[index];
            bool changed = node.aabb.Enlarge(aabb);
            node.flagIndex |= MovedNode;
            index = parents[index];
            if (!changed) break;
        }
        while (index != -1)
        {
            node = ref nodes[index];
            if ((node.flagIndex & MovedNode) != 0) break;
            node.flagIndex |= MovedNode;
            index = parents[index];
        }
    }
    /// <summary>Modify the category bits on a proxy.</summary>
    public void SetCategoryBits(int proxyId, ulong categoryBits)
    {
        Debug.Assert(0 <= proxyId && proxyId < proxies.Length);
        proxies[proxyId].categoryBits = categoryBits;
    }
    /// <summary>Get the category bits on a proxy.</summary>
    public ulong GetCategoryBits(int proxyId)
    {
        Debug.Assert(0 <= proxyId && proxyId < proxies.Length);
        return proxies[proxyId].categoryBits;
    }
    /// <summary>Query an AABB for overlapping proxies. The callback class is called for each proxy that overlaps the supplied AABB.</summary>
    /// <returns>performance data</returns>
    public TreeStats Query(AABB aabb, ulong maskBits, TreeQueryCallbackFcn callback, object context)
    {
        TreeStats result = new();
        if (proxyCount == 0) return result;
        Stack<int> stack = new(1024);
        stack.Push(GetRootPair(nodes));
        AABBV boxv = aabb.LoadAABBV();
        while (stack.Count > 0)
        {
            int pair = stack.Pop();
            result.nodeVisits++;
            for (int i = 0; i < 2; i++)
            {
                ref TreeNode node = ref nodes[pair + i];
                if (boxv.OverlapNode(ref node))
                {
                    if (node.IsLeaf())
                    {
                        int proxyId = node.GetProxyId();
                        ref TreeProxy proxy = ref proxies[proxyId];
                        if ((proxy.categoryBits & maskBits) != 0)
                        {
                            bool proceed = callback(proxyId, proxy.userData, context);
                            result.leafVisits++;
                            if (!proceed) return result;
                        }
                    }
                    else
                    {
                        stack.Push(node.GetLeftChild());
                    }
                }
            }
        }
        return result;
    }
    /// <summary>Query an AABB for overlapping proxies. The callback class is called for each proxy that overlaps the supplied AABB.
    /// No filtering is performed.</summary>
    /// <returns>performance data</returns>
    public TreeStats QueryAll(AABB aabb, TreeQueryCallbackFcn callback, object context)
    {
        TreeStats result = new();
        if (proxyCount == 0) return result;
        Stack<int> stack = new(1024);
        stack.Push(GetRootPair(nodes));
        AABBV boxv = aabb.LoadAABBV();
        while (stack.Count > 0)
        {
            int pair = stack.Pop();
            result.nodeVisits++;
            for (int i = 0; i < 2; i++)
            {
                ref TreeNode node = ref nodes[pair + i];
                if (boxv.OverlapNode(ref node))
                {
                    if (node.IsLeaf())
                    {
                        int proxyId = node.GetProxyId();
                        ref TreeProxy proxy = ref proxies[proxyId];
                        bool proceed = callback(proxyId, proxy.userData, context);
                        result.leafVisits++;
                        if (!proceed) return result;
                    }
                    else
                    {
                        stack.Push(node.GetLeftChild());
                    }
                }
            }
        }
        return result;
    }
    /// <summary>Ray cast against the proxies in the tree. This relies on the callback
    /// to perform a exact ray cast in the case were the proxy contains a shape.
    /// The callback also performs the any collision filtering. This has performance
    /// roughly equal to k * log(n), where k is the number of collisions and n is the
    /// number of proxies in the tree.
    /// Bit-wise filtering using mask bits can greatly improve performance in some scenarios.
    ///	However, this filtering may be approximate, so the user should still apply filtering to results.</summary>
    /// <param name="input">the ray cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1)</param>
    /// <param name="maskBits">mask bit hint: `bool accept = (maskBits &amp; node->categoryBits) != 0;</param>
    /// <param name="callback">a callback class that is called for each proxy that is hit by the ray</param>
    /// <param name="context">user context that is passed to the callback</param>
    /// <returns>performance data</returns>
    public TreeStats CastRay(ref RayCastInput input, ulong maskBits, TreeRayCastCallbackFcn callback, object context)
    {
        TreeStats result = new();
        if (proxyCount == 0) return result;
        Vector2 p1 = input.origin, d = input.translation, r = d.Normalize();
        Vector2 v = Vector2.CrossSV(1, r), abs_v = v.Abs();
        float maxFraction = input.maxFraction;
        Vector2 p2 = Vector2.MulAdd(p1, maxFraction, d);
        AABB segmentAABB = new(Vector2.Min(p1, p2), Vector2.Max(p1, p2));
        AABBV boxv = segmentAABB.LoadAABBV();
        Stack<int> stack = new(1024);
        stack.Push(GetRootPair(nodes));
        RayCastInput subInput = input;
        TreeNode dummy = new();
        while (stack.Count > 0)
        {
            int pair = stack.Pop();
            result.nodeVisits++;
            ref TreeNode hit0 = ref dummy, hit1 = ref dummy;
            bool isLeaf0 = false, isLeaf1 = false;
            int hitCount = 0;
            for (int i = 0; i < 2; i++)
            {
                ref TreeNode node = ref nodes[pair + i];
                if (!boxv.OverlapNode(ref node)) continue;
                AABB nodeAABB = node.aabb;
                Vector2 c = nodeAABB.Center(), h = nodeAABB.Extents();
                float term1 = Math.Abs(Vector2.Dot(v, p1 - c));
                float term2 = Vector2.Dot(abs_v, h);
                if (term2 < term1) continue;
                if (hitCount > 0) { isLeaf1 = node.IsLeaf(); hit1 = node; }
                else { isLeaf0 = node.IsLeaf(); hit0 = node; }
                hitCount++;
            }
            if (hitCount == 2 && !isLeaf0 && !isLeaf1)
            {
                Vector2 center1 = hit0.aabb.Center(), center2 = hit1.aabb.Center();
                if (Vector2.DistanceSquared(center1, p1) < Vector2.DistanceSquared(center2, p1))
                    (hit0, hit1) = (hit1, hit0);
            }
            for (int i = 0; i < hitCount; i++)
            {
                if (i > 0 ? isLeaf1 : isLeaf0)
                {
                    int proxyId = (i > 0 ? hit1 : hit0).GetProxyId();
                    ref TreeProxy proxy = ref proxies[proxyId];
                    if ((proxy.categoryBits & maskBits) == 0) continue;
                    subInput.maxFraction = maxFraction;
                    float value = callback(ref subInput, proxyId, proxy.userData, context);
                    result.leafVisits++;
                    if (value == 0) return result;
                    if (0 < value && value <= maxFraction) //???
                    {
                        maxFraction = value;
                        p2 = Vector2.MulAdd(p1, maxFraction, d);
                        segmentAABB.lowerBound = Vector2.Min(p1, p2);
                        segmentAABB.upperBound = Vector2.Max(p1, p2);
                        boxv = segmentAABB.LoadAABBV();
                    }
                }
                else
                {
                    stack.Push((i > 0 ? hit1 : hit0).GetLeftChild());
                }
            }
        }
        return result;
    }
    /// <summary>Cast a swept AABB through the tree. This has performance roughly equal to k * log(n),
    /// where k is the number of collisions and n is the number of proxies in the tree.</summary>
    /// <param name="input">the AABB cast input. The box sweeps from its origin to origin + maxFraction * translation</param>
    /// <param name="maskBits">filter bits: `bool accept = (maskBits &amp; node->categoryBits) != 0;</param>
    /// <param name="callback">a callback that is called for each proxy the swept box may hit</param>
    /// <param name="context">user context that is passed to the callback</param>
    /// <returns>performance data</returns>
    public TreeStats CastBox(ref BoxCastInput input, ulong maskBits, TreeBoxCastCallbackFcn callback, object context)
    {
        TreeStats result = new();
        if (proxyCount == 0) return result;
        AABB originAABB = input.box;
        Vector2 p1 = originAABB.Center(), extension = originAABB.Extents();
        Vector2 r = input.translation;
        Vector2 v = Vector2.CrossSV(1, r);
        Vector2 abs_v = v.Abs();
        float maxFraction = input.maxFraction;
        Vector2 t = maxFraction * input.translation;
        AABB totalAABB = new(Vector2.Min(originAABB.lowerBound, originAABB.lowerBound + t),
            Vector2.Max(originAABB.upperBound, originAABB.upperBound + t));
        AABBV boxv = totalAABB.LoadAABBV();
        Stack<int> stack = new(1024);
        stack.Push(GetRootPair(nodes));
        BoxCastInput subInput = input;
        TreeNode dummy = new();
        while (stack.Count > 0)
        {
            int pair = stack.Pop();
            result.nodeVisits++;
            ref TreeNode hit0 = ref dummy, hit1 = ref dummy;
            bool isLeaf0 = false, isLeaf1 = false;
            int hitCount = 0;
            for (int i = 0; i < 2; i++)
            {
                ref TreeNode node = ref nodes[pair + i];
                if (!boxv.OverlapNode(ref node)) continue;
                AABB nodeAABB = node.aabb;
                Vector2 c = nodeAABB.Center(), h = nodeAABB.Extents() + extension;
                float term1 = Math.Abs(Vector2.Dot(v, p1 - c));
                float term2 = Vector2.Dot(abs_v, h);
                if (term2 < term1) continue;
                if (hitCount > 0) { isLeaf1 = node.IsLeaf(); hit1 = node; }
                else { isLeaf0 = node.IsLeaf(); hit0 = node; }
                hitCount++;
            }
            if (hitCount == 2 && !isLeaf0 && !isLeaf1)
            {
                Vector2 center1 = hit0.aabb.Center(), center2 = hit1.aabb.Center();
                if (Vector2.DistanceSquared(center1, p1) < Vector2.DistanceSquared(center2, p1))
                {
                    ref TreeNode tmp = ref hit1;
                    hit1 = ref hit0;
                    hit0 = ref tmp;
                }
            }
            for (int i = 0; i < hitCount; i++)
            {
                if (i > 0 ? isLeaf1 : isLeaf0)
                {
                    int proxyId = (i > 0 ? hit1 : hit0).GetProxyId();
                    ref TreeProxy proxy = ref proxies[proxyId];
                    if ((proxy.categoryBits & maskBits) == 0) continue;
                    subInput.maxFraction = maxFraction;
                    float value = callback(ref subInput, proxyId, proxy.userData, context);
                    result.leafVisits++;
                    if (value == 0) return result;
                    if (0 < value && value < maxFraction)
                    {
                        maxFraction = value;
                        t = maxFraction * input.translation;
                        totalAABB.lowerBound = Vector2.Min(originAABB.lowerBound, originAABB.lowerBound + t);
                        totalAABB.upperBound = Vector2.Max(originAABB.upperBound, originAABB.upperBound + t);
                        boxv = totalAABB.LoadAABBV();
                    }
                }
                else
                {
                    stack.Push((i > 0 ? hit1 : hit0).GetLeftChild());
                }
            }
        }
        return result;
    }
    public int GetHeight() => nodes[RootNode].GetNodeHeight();
    ///<summary>The area ratio is the thing that SAH seeks to minimize. SAH
    ///cannot do anything about leaf boxes or the root box. It seeks
    ///to minimize the area of all non-root internal nodes. Divide this
    ///by the root area to make the metric non-dimensional.
    ///So this becomes a meaningful measure of tree quality.</summary>
    public float GetAreaRatio()
    {
        if (proxyCount == 0) return 0;
        float rootArea = nodes[RootNode].aabb.Perimeter();
        if (rootArea <= 0) return 0;
        float internalArea = 0;
        for (int i = 2; i < nodeEnd; i++)
            if (!nodes[i].IsLeaf())
                internalArea += nodes[i].aabb.Perimeter();
        return internalArea / rootArea;
    }
    ///<summary> Get the bounding box that contains the entire tree</summary>
    public AABB GetRootBounds() => proxyCount == 0 ? new() : nodes[RootNode].aabb;
    int ValidateSubtree(int nodeIndex, ref int leafCount)
    {
        Debug.Assert(0 <= nodeIndex && nodeIndex < nodes.Length);
        ref TreeNode node = ref nodes[nodeIndex];
        Debug.Assert(!node.IsEmpty());
        if (node.IsLeaf())
        {
            int proxyId = node.GetProxyId();
            Debug.Assert(0 <= proxyId && proxyId < proxies.Length);
            Debug.Assert(proxies[proxyId].node == nodeIndex);
            Debug.Assert((int)proxies[proxyId].userData == node.shapeIndex);
            leafCount++;
            return 0;
        }
        int pair = node.GetLeftChild();
        Debug.Assert((pair & 1) == 0 && 2 <= pair && pair < nodeEnd);
        Debug.Assert(parents[pair] == nodeIndex);
        Debug.Assert(parents[pair + 1] == nodeIndex);
        Debug.Assert(!dfsOrdered || nodeIndex < pair);
        ref TreeNode c1 = ref nodes[pair], c2 = ref nodes[pair + 1];
        Debug.Assert(node.aabb.Contains(c1.aabb));
        Debug.Assert(node.aabb.Contains(c2.aabb));
        Debug.Assert(node.IsMoved() == (c1.IsMoved() || c2.IsMoved()));
        int height = 1 + ValidateSubtree(pair, ref leafCount) + ValidateSubtree(pair + 1, ref leafCount);
        Debug.Assert(node.height == height);
        return height;
    }
    ///<summary> Get the number of proxies created</summary>
    public int GetProxyCount() => proxyCount;
    /// <summary>Median split heuristic</summary>
    int PartitionMid(Span<int> indices, Span<Vector2> centers, int count)
    {
        if (count <= 2) return count / 2;
        Vector2 lowerBound = centers[0], upperBound = centers[0];
        for (int i = 1; i < count; i++)
        {
            lowerBound = Vector2.Min(lowerBound, centers[i]);
            upperBound = Vector2.Max(upperBound, centers[i]);
        }
        Vector2 d = upperBound - lowerBound;
        Vector2 c = new(0.5f * (lowerBound.x + upperBound.x), 0.5f * (lowerBound.y + upperBound.y));
        int i1 = 0, i2 = count;
        if (d.x > d.y)
        {
            float pivot = c.x;
            while (i1 < i2)
            {
                while (i1 < i2 && centers[i1].x < pivot) i1++;
                while (i1 < i2 && centers[i2 - 1].x >= pivot) i2--;
                if (i1 < i2)
                {
                    (indices[i1], indices[i2 - 1]) = (indices[i2 - 1], indices[i1]);
                    (centers[i1], centers[i2 - 1]) = (centers[i2 - 1], centers[i1]);
                    i1++; i2--;
                }
            }
        }
        else
        {
            float pivot = c.y;
            while (i1 < i2)
            {
                while (i1 < i2 && centers[i1].y < pivot) i1++;
                while (i1 < i2 && centers[i2 - 1].y >= pivot) i2--;
                if (i1 < i2)
                {
                    (indices[i1], indices[i2 - 1]) = (indices[i2 - 1], indices[i1]);
                    (centers[i1], centers[i2 - 1]) = (centers[i2 - 1], centers[i1]);
                    i1++; i2--;
                }
            }
        }
        Debug.Assert(i1 == i2);
        if (i1 > 0 && i1 < count) return i1;
        return count / 2;
    }
    /// <summary>Temporary data used to track the rebuild of a tree node.</summary>
    struct RebuildItem
    {
        /// <summary>Where this node is written and the pair its children go in.</summary>
        public int nodeIndex;
        public int pair, childCount, startIndex, splitIndex, endIndex;
    }
    struct CopyItem
    {
        public int oldPair, newIndex;
    }
    /// <summary>Bump allocate a pair of sibling nodes. Returns index to the first one.</summary>
    int BumpPair(int parent)
    {
        int pair = nodeEnd;
        Debug.Assert(pair + 2 <= nodes.Length);
        nodeEnd += 2;
        parents[pair] = parent;
        parents[pair + 1] = parent;
        return pair;
    }
    /// <summary>Copy a retained subtree from the old array into the rebuilt DFS array using a bump allocator.</summary>
    void CopySubtree(TreeNode node, int newIndex)
    {
        var oldNodes = nodes; var newNodes = swapNodes;
        if (node.IsLeaf())
        {
            newNodes[newIndex] = node;
            proxies[node.GetProxyId()].node = newIndex;
            return;
        }
        Stack<CopyItem> stack = new(1024);
        int oldPair = node.GetLeftChild(), newPair = BumpPair(newIndex);
        node.SetLeftChild(newPair);
        newNodes[newIndex] = node;
        while (true)
        {
            ref TreeNode pair0 = ref newNodes[newPair];
            ref TreeNode pair1 = ref newNodes[newPair + 1];
            pair0 = oldNodes[oldPair];
            pair1 = oldNodes[oldPair + 1];
            if (pair1.IsLeaf())
                proxies[pair1.GetProxyId()].node = newPair + 1;
            else
                stack.Push(new() { oldPair = pair1.GetLeftChild(), newIndex = newPair + 1 });
            if (!pair0.IsLeaf())
            {
                int leftIndex = newPair;
                oldPair = pair0.GetLeftChild();
                newPair = BumpPair(leftIndex);
                newNodes[leftIndex].SetLeftChild(newPair);
                continue;
            }
            proxies[pair0.GetProxyId()].node = newPair;
            if (stack.Count == 0) break;
            CopyItem item = stack.Pop();
            oldPair = item.oldPair;
            newPair = BumpPair(item.newIndex);
            newNodes[item.newIndex].SetLeftChild(newPair);
        }
    }
    void PlaceLeaf(TreeNode leaf, int newIndex)
    {
        if (leaf.IsLeaf())
        {
            swapNodes[newIndex] = leaf;
            proxies[leaf.GetProxyId()].node = newIndex;
        }
        else CopySubtree(leaf, newIndex);
    }
    void BuildTree(int leafCount)
    {
        nodeEnd = 2;
        swapNodes[RootNode + 1] = new();
        parents[RootNode] = -1;
        parents[RootNode + 1] = -1;
        if (leafCount == 1)
        {
            PlaceLeaf(leafNodes[leafIndices[0]], RootNode);
            return;
        }
        List<RebuildItem> stack = new(1024)
        {
            new()
            {
                nodeIndex = RootNode,
                pair = BumpPair(RootNode),
                childCount = -1,
                startIndex = 0,
                endIndex = leafCount,
                splitIndex = PartitionMid(leafIndices, leafCenters, leafCount)
            }
        };
        while (true)
        {
            ref RebuildItem item = ref CollectionsMarshal.AsSpan(stack)[^1];
            item.childCount++;
            if (item.childCount == 2)
            {
                swapNodes[item.nodeIndex] = MakeInternalNode(swapNodes, item.pair);
                if (stack.Count == 1) break;
                stack.RemoveAt(stack.Count - 1);
                continue;
            }
            int slot = item.childCount;
            int startIndex = slot == 0 ? item.startIndex : item.splitIndex;
            int endIndex = slot == 0 ? item.splitIndex : item.endIndex;
            int count = endIndex - startIndex;
            Debug.Assert(count > 0);
            int nodeIndex = item.pair + slot;
            if (count == 1)
            {
                PlaceLeaf(leafNodes[leafIndices[startIndex]], nodeIndex);
                continue;
            }
            stack.Add(new()
            {
                nodeIndex = nodeIndex,
                pair = BumpPair(nodeIndex),
                childCount = -1,
                startIndex = startIndex,
                endIndex = endIndex,
                splitIndex = startIndex + PartitionMid(leafIndices.AsSpan(startIndex, count), leafCenters.AsSpan(startIndex, count), count)
            });
        }
    }
    ///<summary> Rebuild the tree while retaining subtrees that haven't changed. Returns the number of boxes sorted.</summary>
    public unsafe int Rebuild(bool fullBuild)
    {
        if (proxyCount == 0) return 0;
        ref TreeNode root = ref nodes[RootNode];
        if (!fullBuild && !root.IsMoved() && dfsOrdered) return 0;
        if (swapNodes == null) swapNodes = new TreeNode[nodes.Length];
        if (proxyCount > rebuildCapacity)
        {
            int oldCapacity = rebuildCapacity;
            int newCapacity = proxyCount + proxyCount / 2;
            var oldIndices = leafIndices; leafIndices = new int[newCapacity]; if (oldIndices != null) Buffer.BlockCopy(oldIndices, 0, leafIndices, 0, oldIndices.Length * sizeof(int));
            var oldNodes = leafNodes; leafNodes = new TreeNode[newCapacity]; if (oldNodes != null) Buffer.BlockCopy(oldNodes, 0, leafNodes, 0, oldNodes.Length * sizeof(TreeNode));
            var oldCenters = leafCenters; leafCenters = new Vector2[newCapacity]; if (oldCenters != null) Buffer.BlockCopy(oldCenters, 0, leafCenters, 0, oldCenters.Length * sizeof(Vector2));
            rebuildCapacity = newCapacity;
        }
        int leafCount = 0;
        Stack<int> stack = new(1024);
        if (!root.IsLeaf() && (fullBuild || root.IsMoved())) stack.Push(root.GetLeftChild());
        else
        {
            TreeNode node = root;
            node.flagIndex &= ~MovedNode;
            leafIndices[0] = 0;
            leafNodes[0] = node;
            leafCenters[0] = node.aabb.Center();
            leafCount = 1;
        }
        while (stack.Count > 0)
        {
            int pair = stack.Pop();
            for (int i = 0; i < 2; i++)
            {
                ref TreeNode node = ref nodes[pair + i];
                if (!node.IsLeaf() && (fullBuild || node.IsMoved()))
                {
                    stack.Push(node.GetLeftChild());
                    continue;
                }
                node.flagIndex &= ~MovedNode;
                leafIndices[leafCount] = leafCount;
                leafNodes[leafCount] = node;
                leafCenters[leafCount] = node.aabb.Center();
                leafCount++;
            }
        }
        Debug.Assert(0 < leafCount && leafCount <= proxyCount);
        BuildTree(leafCount);
        (nodes, swapNodes) = (swapNodes, nodes);
        pairFreeList = -1;
        dfsOrdered = true;
        Validate();
        ValidateNoMoved();
        return leafCount;
    }
    /// <summary>Set the moved flag on the ancestors of the proxy. Serial use case.</summary>
    public void MarkProxyMovedSerial(int proxyId)
    {
        Debug.Assert(0 <= proxyId && proxyId < proxies.Length);
        int index = proxies[proxyId].node;
        Debug.Assert(0 <= index && index < nodeEnd);
        Debug.Assert(nodes[index].IsLeaf());
        while (index != -1)
        {
            nodes[index].flagIndex |= MovedNode;
            index = parents[index];
        }
    }
    /// <summary>Update a proxy AABB and flag the ancestors as moved. Thread-safe using atomics.</summary>
    public void MarkProxyMoved(int proxyId, AABB aabb)
    {
        Debug.Assert(0 <= proxyId && proxyId < proxies.Length);
        int index = proxies[proxyId].node;
        Debug.Assert(0 <= index && index < nodeEnd);
        ref TreeNode node = ref nodes[index];
        Debug.Assert(node.IsLeaf());
        Debug.Assert(!node.aabb.Contains(aabb));
        node.aabb = aabb;
        node.flagIndex |= MovedNode;
        index = parents[index];
        while (index != -1)
        {
            node = ref nodes[index];
            if ((node.flagIndex & MovedNode) != 0) break;
            uint previousFlags = System.Threading.Interlocked.Or(ref node.flagIndex, MovedNode);
            if ((previousFlags & MovedNode) != 0) break;
            index = parents[index];
        }
    }
    /// <summary>Clear the moved flags from the entire tree.</summary>
    public void ClearMoved()
    {
        ref TreeNode root = ref nodes[RootNode];
        if (!root.IsMoved()) return;
        root.flagIndex &= ~MovedNode;
        if (root.IsLeaf()) return;
        Stack<int> stack = new(1024);
        stack.Push(root.GetLeftChild());
        while (stack.Count > 0)
        {
            int pair = stack.Pop();
            for (int i = 0; i < 2; i++)
            {
                ref TreeNode node = ref nodes[pair + i];
                if ((node.flagIndex & MovedNode) != 0)
                {
                    node.flagIndex &= ~MovedNode;
                    if (!node.IsLeaf()) stack.Push(node.GetLeftChild());
                }
            }
        }
    }
    public int GatherMovedProxies(Span<int> proxyIds)
    {
        ref TreeNode root = ref nodes[RootNode];
        if (!root.IsMoved()) return 0;
        if (root.IsLeaf()) { proxyIds[0] = root.GetProxyId(); return 1; }
        int count = 0;
        Stack<int> stack = new(1024);
        stack.Push(root.GetLeftChild());
        while (stack.Count > 0)
        {
            int pair = stack.Pop();
            for (int i = 0; i < 2; i++)
            {
                ref TreeNode node = ref nodes[pair + i];
                if (!node.IsMoved()) continue;
                if (node.IsLeaf()) proxyIds[count++] = node.GetProxyId();
                else stack.Push(node.GetLeftChild());
            }
        }
        return count;
    }
    /// <summary>Slow refit for unit tests.</summary>
    void RefitSubtree(int rootIndex)
    {
        ref TreeNode root = ref nodes[rootIndex];
        if (root.IsLeaf() || !root.IsMoved()) return;
        Stack<int> stack = new();
        stack.Push(~rootIndex);
        stack.Push(root.GetLeftChild());
        stack.Push(root.GetLeftChild() + 1);
        while (stack.Count > 0)
        {
            int item = stack.Pop();
            if (item < 0)
            {
                ref TreeNode node = ref nodes[~item];
                int pair = node.GetLeftChild();
                node.aabb = AABBV.Union(nodes[pair].aabb, nodes[pair + 1].aabb);
                continue;
            }
            {
                ref TreeNode node = ref nodes[item];
                if (node.IsLeaf() || !node.IsMoved()) continue;
                int pair = node.GetLeftChild();
                stack.Push(~item);
                stack.Push(pair);
                stack.Push(pair + 1);
            }
        }
    }
    public unsafe void Refit()
    {
        if (!HasTreeMoved()) return;
        if (!dfsOrdered) { RefitSubtree(RootNode); return; }
        fixed (TreeNode* n = nodes) for (int pair = nodeEnd - 2; pair >= 0; pair -= 2)
        {
            ref TreeNode node0 = ref nodes[pair];
            ref TreeNode node1 = ref nodes[pair + 1];
            uint flags1 = node0.flagIndex, flags2 = node1.flagIndex;
            if (((flags1 | flags2) & MovedNode) == 0) continue;
            bool refit1 = (flags1 & (LeafNode | MovedNode)) == MovedNode;
            bool refit2 = (flags2 & (LeafNode | MovedNode)) == MovedNode;
            int children1 = refit1 ? (int)(flags1 & NodeIndexMask) : pair;
            int children2 = refit2 ? (int)(flags2 & NodeIndexMask) : pair;
            AABBV.Store(ref node0.aabb, AABBV.UnionPair(n + children1), refit1);
            AABBV.Store(ref node1.aabb, AABBV.UnionPair(n + children2), refit2);
        }
    }
    ///<summary> Get the number of bytes used by this tree</summary>
    public unsafe int GetByteCount() => 20 + nodes.Length * sizeof(TreeNode)
        + rebuildCapacity * (sizeof(int) + sizeof(AABB) + sizeof(Vector2) + sizeof(int));
    ///<summary> Get proxy user data</summary>
    public ulong GetUserData(int proxyId)
    {
        Debug.Assert(0 <= proxyId && proxyId < proxies.Length);
        return proxies[proxyId].userData;
    }
    ///<summary> Get the AABB of a proxy</summary>
    public AABB GetAABB(int proxyId)
    {
        Debug.Assert(0 <= proxyId && proxyId < proxies.Length);
        int nodeIndex = proxies[proxyId].node;
        Debug.Assert(0 <= nodeIndex && nodeIndex < nodeEnd);
        return nodes[nodeIndex].aabb;
    }
    ///<summary> Validate this tree.</summary>
    public void Validate()
    {
#if B2_VALIDATE
        Debug.Assert(2 <= nodeEnd && nodeEnd <= nodes.Length);
        Debug.Assert((nodeEnd & 1) == 0);
        Debug.Assert(parents[RootNode] == -1);
        Debug.Assert(nodes[RootNode + 1].IsEmpty());
        int freePairCount = 0;
        int pair = pairFreeList;
        while (pair != -1)
        {
            Debug.Assert((pair & 1) == 0 && 2 <= pair && pair < nodeEnd);
            Debug.Assert(nodes[pair].IsEmpty());
            Debug.Assert(nodes[pair + 1].IsEmpty());
            pair = parents[pair];
            freePairCount++;
            Debug.Assert(2 * freePairCount < nodeEnd);
        }
        int freeProxyCount = 0;
        int freeIndex = proxyFreeList;
        while (freeIndex != -1)
        {
            Debug.Assert(0 <= freeIndex && freeIndex < proxies.Length);
            Debug.Assert(proxies[freeIndex].node == -1);
            freeIndex = proxies[freeIndex].next;
            freeProxyCount++;
        }
        Debug.Assert(proxyCount + freeProxyCount == proxies.Length);
        Debug.Assert(nodeEnd == 2 * Math.Max(proxyCount, 1) + 2 * freePairCount);
        if (proxyCount == 0)
        {
            Debug.Assert(nodes[RootNode].IsEmpty());
            return;
        }
        int leafCount = 0;
        ValidateSubtree(RootNode, ref leafCount);
        Debug.Assert(leafCount == proxyCount);
#endif
    }
    ///<summary> Validate this tree has no enlarged AABBs. For testing.</summary>
    public void ValidateNoMoved()
    {
#if B2_VALIDATE
        for (int i = 0; i < nodeEnd; i++) Debug.Assert(!nodes[i].IsMoved());
#endif
    }
}