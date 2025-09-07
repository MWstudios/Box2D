using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.InteropServices;

namespace Box2D;

[Flags] public enum TreeNodeFlags : ushort { Allocated = 1, Enlarged = 2, Leaf = 4 }
/// <summary>A node in the dynamic tree.</summary>
[StructLayout(LayoutKind.Explicit)] public struct TreeNode
{
    /// <summary> The node bounding box</summary>
    [FieldOffset(0)] public AABB aabb = new();
    /// <summary>Category bits for collision filtering</summary>
    [FieldOffset(16)] public ulong categoryBits = Box2D.DEFAULT_CATEGORY_BITS;
    /// <summary>Children (internal node)</summary>
    [FieldOffset(24)] public int child1 = -1;
    /// <summary>Children (internal node)</summary>
    [FieldOffset(28)] public int child2 = -1;
    /// <summary>User data (leaf node)</summary>
    [FieldOffset(24)] public ulong userData = ulong.MaxValue;
    /// <summary>The node parent index (allocated node)</summary>
    [FieldOffset(32)] public int parent = -1;
    /// <summary>The node freelist next index (free node)</summary>
    [FieldOffset(32)] public int next = -1;
    [FieldOffset(36)] public ushort height = 0;
    [FieldOffset(38)] public ushort flags = (ushort)TreeNodeFlags.Allocated;
    public TreeNode() { }
    public bool IsLeaf() => ((TreeNodeFlags)flags).HasFlag(TreeNodeFlags.Leaf);
    public bool IsAllocated() => ((TreeNodeFlags)flags).HasFlag(TreeNodeFlags.Allocated);
}
public partial class DynamicTree
{
    /// <summary>Constructing the tree initializes the node pool.</summary>
    public DynamicTree()
    {
        root = -1;
        nodeCount = 0;
        nodes = new TreeNode[16];
        for (int i = 0; i < nodes.Length - 1; i++) nodes[i].next = i + 1;
        nodes[^1].next = -1;
        freeList = 0;
        proxyCount = 0;
        leafIndices = null;
        leafBoxes = null;
        leafCenters = null;
        binIndices = null;
        rebuildCapacity = 0;
    }
    /// <summary>Destroy the tree, freeing the node pool.</summary>
    public void Destroy() { }
    /// <summary>Allocate a node from the pool. Grow the pool if necessary.</summary>
    unsafe int AllocateNode()
    {
        if (freeList == -1)
        {
            Debug.Assert(nodeCount == nodes.Length);
            TreeNode[] oldNodes = nodes; nodes = new TreeNode[nodes.Length + (nodes.Length >> 1)];
            Debug.Assert(oldNodes != null);
            Array.Copy(oldNodes, 0, nodes, 0, oldNodes.Length);
            for (int i = nodeCount; i < nodes.Length; i++) nodes[i].next = i + 1;
            nodes[^1].next = -1;
            freeList = nodeCount;
        }
        int nodeIndex = freeList;
        ref TreeNode node = ref nodes[nodeIndex];
        freeList = node.next;
        node = new();
        nodeCount++;
        return nodeIndex;
    }
    /// <summary>Return a node to the pool.</summary>
    void FreeNode(int nodeId)
    {
        Debug.Assert(0 <= nodeId && nodeId < nodes.Length);
        Debug.Assert(0 < nodeCount);
        nodes[nodeId].next = freeList;
        nodes[nodeId].flags = 0;
        freeList = nodeId;
        nodeCount--;
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
        int rootIndex = root;
        AABB rootBox = nodes[rootIndex].aabb;
        float areaBase = rootBox.Perimeter();
        float directCost = AABB.Union(rootBox, boxD).Perimeter();
        float inheritedCost = 0;
        int bestSibling = rootIndex;
        float bestCost = directCost;
        int index = rootIndex;
        while (nodes[index].height > 0)
        {
            int child1 = nodes[index].child1, child2 = nodes[index].child2;
            float cost = directCost + inheritedCost;
            if (cost < bestCost)
            {
                bestSibling = index;
                bestCost = cost;
            }
            inheritedCost += directCost - areaBase;
            bool leaf1 = nodes[child1].height == 0;
            bool leaf2 = nodes[child2].height == 0;
            float lowerCost1 = float.MaxValue;
            AABB box1 = nodes[child1].aabb;
            float directCost1 = AABB.Union(box1, boxD).Perimeter();
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
            float directCost2 = AABB.Union(box2, boxD).Perimeter();
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
                index = child1;
                areaBase = area1;
                directCost = directCost1;
            }
            else
            {
                index = child2;
                areaBase = area2;
                directCost = directCost2;
            }
            Debug.Assert(nodes[index].height > 0);
        }
        return bestSibling;
    }
    enum RotateType { None, BF, BG, CD, CE }
    /// <summary>Perform a left or right rotation if node A is imbalanced. Returns the new root index.</summary>
    void RotateNodes(int iA)
    {
        Debug.Assert(iA != -1);
        ref TreeNode A = ref nodes[iA];
        if (A.height < 2) return;
        int iB = A.child1, iC = A.child2;
        Debug.Assert(0 <= iB && iB < nodes.Length);
        Debug.Assert(0 <= iC && iC < nodes.Length);
        ref TreeNode B = ref nodes[iB], C = ref nodes[iC];
        if (B.height == 0)
        {
            Debug.Assert(C.height > 0);
            int iF = C.child1, iG = C.child2;
            ref TreeNode F = ref nodes[iF], G = ref nodes[iG];
            Debug.Assert(0 <= iF && iF < nodes.Length);
            Debug.Assert(0 <= iG && iG < nodes.Length);
            float costBase = C.aabb.Perimeter();
            AABB aabbBG = AABB.Union(B.aabb, G.aabb);
            float costBG = aabbBG.Perimeter();
            AABB aabbBF = AABB.Union(B.aabb, F.aabb);
            float costBF = aabbBF.Perimeter();
            if (costBase < costBF && costBase < costBG) return;
            if (costBF < costBG)
            {
                A.child1 = iF; C.child1 = iB;
                B.parent = iC; F.parent = iA;
                C.aabb = aabbBG;
                C.height = (ushort)(1 + Math.Max(B.height, G.height));
                A.height = (ushort)(1 + Math.Max(C.height, F.height));
                C.categoryBits = B.categoryBits | G.categoryBits;
                A.categoryBits = C.categoryBits | F.categoryBits;
                C.flags |= (ushort)((B.flags | G.flags) & (ushort)TreeNodeFlags.Enlarged);
                A.flags |= (ushort)((C.flags | F.flags) & (ushort)TreeNodeFlags.Enlarged);
            }
            else
            {
                A.child1 = iG; C.child2 = iB;
                B.parent = iC; G.parent = iA;
                C.aabb = aabbBF;
                C.height = (ushort)(1 + Math.Max(B.height, F.height));
                A.height = (ushort)(1 + Math.Max(C.height, G.height));
                C.categoryBits = B.categoryBits | F.categoryBits;
                A.categoryBits = C.categoryBits | G.categoryBits;
                C.flags |= (ushort)((B.flags | F.flags) & (ushort)TreeNodeFlags.Enlarged);
                A.flags |= (ushort)((C.flags | G.flags) & (ushort)TreeNodeFlags.Enlarged);
            }
        }
        else if (C.height == 0)
        {
            Debug.Assert(B.height > 0);
            int iD = B.child1, iE = B.child2;
            ref TreeNode D = ref nodes[iD], E = ref nodes[iE];
            Debug.Assert(0 <= iD && iD < nodes.Length);
            Debug.Assert(0 <= iE && iE < nodes.Length);
            float costBase = B.aabb.Perimeter();
            AABB aabbCE = AABB.Union(C.aabb, E.aabb);
            float costCE = aabbCE.Perimeter();
            AABB aabbCD = AABB.Union(C.aabb, D.aabb);
            float costCD = aabbCD.Perimeter();
            if (costBase < costCD && costBase < costCE) return;
            if (costCD < costCE)
            {
                A.child2 = iD; B.child1 = iC;
                C.parent = iB; D.parent = iA;
                B.aabb = aabbCE;
                B.height = (ushort)(1 + Math.Max(C.height, E.height));
                A.height = (ushort)(1 + Math.Max(B.height, D.height));
                B.categoryBits = C.categoryBits | E.categoryBits;
                A.categoryBits = B.categoryBits | D.categoryBits;
                B.flags |= (ushort)((C.flags | E.flags) & (ushort)TreeNodeFlags.Enlarged);
                A.flags |= (ushort)((B.flags | D.flags) & (ushort)TreeNodeFlags.Enlarged);
            }
            else
            {
                A.child2 = iE; B.child2 = iC;
                C.parent = iB; E.parent = iA;
                B.aabb = aabbCD;
                B.height = (ushort)(1 + Math.Max(C.height, D.height));
                A.height = (ushort)(1 + Math.Max(B.height, E.height));
                B.categoryBits = C.categoryBits | D.categoryBits;
                A.categoryBits = B.categoryBits | E.categoryBits;
                B.flags |= (ushort)((C.flags | D.flags) & (ushort)TreeNodeFlags.Enlarged);
                A.flags |= (ushort)((B.flags | E.flags) & (ushort)TreeNodeFlags.Enlarged);
            }
        }
        else
        {
            int iD = B.child1, iE = B.child2, iF = C.child1, iG = C.child2;
            Debug.Assert(0 <= iD && iD < nodes.Length);
            Debug.Assert(0 <= iE && iE < nodes.Length);
            Debug.Assert(0 <= iF && iF < nodes.Length);
            Debug.Assert(0 <= iG && iG < nodes.Length);
            ref TreeNode D = ref nodes[iD], E = ref nodes[iE], F = ref nodes[iF], G = ref nodes[iG];
            float areaB = B.aabb.Perimeter();
            float areaC = C.aabb.Perimeter();
            float costBase = areaB + areaC;
            RotateType bestRotation = RotateType.None;
            float bestCost = costBase;
            AABB aabbBG = AABB.Union(B.aabb, G.aabb);
            float costBF = areaB + aabbBG.Perimeter();
            if (costBF < bestCost)
            {
                bestRotation = RotateType.BF;
                bestCost = costBF;
            }
            AABB aabbBF = AABB.Union(B.aabb, F.aabb);
            float costBG = areaB + aabbBF.Perimeter();
            if (costBG < bestCost)
            {
                bestRotation = RotateType.BG;
                bestCost = costBG;
            }
            AABB aabbCE = AABB.Union(C.aabb, E.aabb);
            float costCD = areaC + aabbCE.Perimeter();
            if (costCD < bestCost)
            {
                bestRotation = RotateType.CD;
                bestCost = costCD;
            }
            AABB aabbCD = AABB.Union(C.aabb, D.aabb);
            float costCE = areaC + aabbCD.Perimeter();
            if (costCE < bestCost)
            {
                bestRotation = RotateType.CE;
            }
            switch (bestRotation)
            {
                case RotateType.None:
                    break;
                case RotateType.BF:
                    A.child1 = iF; C.child1 = iB;
                    B.parent = iC; F.parent = iA;
                    C.aabb = aabbBG;
                    C.height = (ushort)(1 + Math.Max(B.height, G.height));
                    A.height = (ushort)(1 + Math.Max(C.height, F.height));
                    C.categoryBits = B.categoryBits | G.categoryBits;
                    A.categoryBits = C.categoryBits | F.categoryBits;
                    C.flags |= (ushort)((B.flags | G.flags) & (ushort)TreeNodeFlags.Enlarged);
                    A.flags |= (ushort)((C.flags | F.flags) & (ushort)TreeNodeFlags.Enlarged);
                    break;
                case RotateType.BG:
                    A.child1 = iG; C.child2 = iB;
                    B.parent = iC; G.parent = iA;
                    C.aabb = aabbBF;
                    C.height = (ushort)(1 + Math.Max(B.height, F.height));
                    A.height = (ushort)(1 + Math.Max(C.height, G.height));
                    C.categoryBits = B.categoryBits | F.categoryBits;
                    A.categoryBits = C.categoryBits | G.categoryBits;
                    C.flags |= (ushort)((B.flags | F.flags) & (ushort)TreeNodeFlags.Enlarged);
                    A.flags |= (ushort)((C.flags | G.flags) & (ushort)TreeNodeFlags.Enlarged);
                    break;
                case RotateType.CD:
                    A.child2 = iD; B.child1 = iC;
                    C.parent = iB; D.parent = iA;
                    B.aabb = aabbCE;
                    B.height = (ushort)(1 + Math.Max(C.height, E.height));
                    A.height = (ushort)(1 + Math.Max(B.height, D.height));
                    B.categoryBits = C.categoryBits | E.categoryBits;
                    A.categoryBits = B.categoryBits | D.categoryBits;
                    B.flags |= (ushort)((C.flags | E.flags) & (ushort)TreeNodeFlags.Enlarged);
                    A.flags |= (ushort)((B.flags | D.flags) & (ushort)TreeNodeFlags.Enlarged);
                    break;
                case RotateType.CE:
                    A.child2 = iE; B.child2 = iC;
                    C.parent = iB; E.parent = iA;
                    B.aabb = aabbCD;
                    B.height = (ushort)(1 + Math.Max(C.height, D.height));
                    A.height = (ushort)(1 + Math.Max(B.height, E.height));
                    B.categoryBits = C.categoryBits | D.categoryBits;
                    A.categoryBits = B.categoryBits | E.categoryBits;
                    B.flags |= (ushort)((C.flags | D.flags) & (ushort)TreeNodeFlags.Enlarged);
                    A.flags |= (ushort)((B.flags | E.flags) & (ushort)TreeNodeFlags.Enlarged);
                    break;
                default:
                    throw new ArgumentOutOfRangeException("Invalid rotation");
            }
        }
    }
    void InsertLeaf(int leaf, bool shouldRotate)
    {
        if (root == -1)
        {
            root = leaf;
            nodes[root].parent = -1;
            return;
        }
        AABB leafAABB = nodes[leaf].aabb;
        int sibling = FindBestSibling(leafAABB);
        int oldParent = nodes[sibling].parent;
        int newParent = AllocateNode();
        nodes[newParent].parent = oldParent;
        nodes[newParent].userData = ulong.MaxValue;
        nodes[newParent].aabb = AABB.Union(leafAABB, nodes[sibling].aabb);
        nodes[newParent].categoryBits = nodes[leaf].categoryBits | nodes[sibling].categoryBits;
        nodes[newParent].height = (ushort)(nodes[sibling].height + 1);
        if (oldParent != -1)
        {
            if (nodes[oldParent].child1 == sibling) nodes[oldParent].child1 = newParent;
            else nodes[oldParent].child2 = newParent;
            nodes[newParent].child1 = sibling;
            nodes[newParent].child2 = leaf;
            nodes[sibling].parent = newParent;
            nodes[leaf].parent = newParent;
        }
        else
        {
            nodes[newParent].child1 = sibling;
            nodes[newParent].child2 = leaf;
            nodes[sibling].parent = newParent;
            nodes[leaf].parent = newParent;
            root = newParent;
        }
        int index = nodes[leaf].parent;
        while (index != -1)
        {
            int child1 = nodes[index].child1, child2 = nodes[index].child2;
            Debug.Assert(child1 != -1);
            Debug.Assert(child2 != -1);
            nodes[index].aabb = AABB.Union(nodes[child1].aabb, nodes[child2].aabb);
            nodes[index].categoryBits = nodes[child1].categoryBits | nodes[child2].categoryBits;
            nodes[index].height = (ushort)(1 + Math.Max(nodes[child1].height, nodes[child2].height));
            nodes[index].flags |= (ushort)((nodes[child1].flags | nodes[child2].flags) & (ushort)TreeNodeFlags.Enlarged);
            if (shouldRotate) RotateNodes(index);
            index = nodes[index].parent;
        }
    }
    void RemoveLeaf(int leaf)
    {
        if (leaf == root) { root = -1; return; }
        int parent = nodes[leaf].parent;
        int grandParent = nodes[parent].parent;
        int sibling = nodes[parent].child1 == leaf ? nodes[parent].child2 : nodes[parent].child1;
        if (grandParent != -1)
        {
            if (nodes[grandParent].child1 == parent) nodes[grandParent].child1 = sibling;
            else nodes[grandParent].child2 = sibling;
            nodes[sibling].parent = grandParent;
            FreeNode(parent);
            int index = grandParent;
            while (index != -1)
            {
                ref TreeNode node = ref nodes[index], child1 = ref nodes[node.child1], child2 = ref nodes[node.child2];
                node.aabb = AABB.Union(child1.aabb, child2.aabb);
                node.categoryBits = child1.categoryBits | child2.categoryBits;
                node.height = (ushort)(1 + Math.Max(child1.height, child2.height));
                index = node.parent;
            }
        }
        else
        {
            root = sibling;
            nodes[sibling].parent = -1;
            FreeNode(parent);
        }
    }
    /// <summary>Create a proxy. Provide an AABB and a userData value.</summary>
    public int CreateProxy(AABB aabb, ulong categoryBits, ulong userData)
    {
        Debug.Assert(-Box2D.Huge < aabb.lowerBound.x && aabb.lowerBound.x < Box2D.Huge);
        Debug.Assert(-Box2D.Huge < aabb.lowerBound.y && aabb.lowerBound.y < Box2D.Huge);
        Debug.Assert(-Box2D.Huge < aabb.upperBound.x && aabb.upperBound.x < Box2D.Huge);
        Debug.Assert(-Box2D.Huge < aabb.upperBound.y && aabb.upperBound.y < Box2D.Huge);
        int proxyId = AllocateNode();
        nodes[proxyId].aabb = aabb;
        nodes[proxyId].userData = userData;
        nodes[proxyId].categoryBits = categoryBits;
        nodes[proxyId].height = 0;
        nodes[proxyId].flags = (ushort)(TreeNodeFlags.Allocated | TreeNodeFlags.Leaf);
        InsertLeaf(proxyId, true);
        proxyCount++;
        return proxyId;
    }
    /// <summary>Destroy a proxy. This asserts if the id is invalid.</summary>
    public void DestroyProxy(int proxyId)
    {
        Debug.Assert(0 <= proxyId && proxyId < nodes.Length);
        Debug.Assert(nodes[proxyId].IsLeaf());
        RemoveLeaf(proxyId);
        FreeNode(proxyId);
        Debug.Assert(proxyCount > 0);
        proxyCount--;
    }
    /// <summary>Move a proxy to a new AABB by removing and reinserting into the tree.</summary>
    public void MoveProxy(int proxyId, AABB aabb)
    {
        Debug.Assert(aabb.IsValid());
        Debug.Assert(aabb.upperBound.x - aabb.lowerBound.x < Box2D.Huge);
        Debug.Assert(aabb.upperBound.y - aabb.lowerBound.y < Box2D.Huge);
        Debug.Assert(0 <= proxyId && proxyId < nodes.Length);
        Debug.Assert(nodes[proxyId].IsLeaf());
        RemoveLeaf(proxyId);
        nodes[proxyId].aabb = aabb;
        InsertLeaf(proxyId, false);
    }
    /// <summary>Enlarge a proxy and enlarge ancestors as necessary.</summary>
    public void EnlargeProxy(int proxyId, AABB aabb)
    {
        Debug.Assert(aabb.IsValid());
        Debug.Assert(aabb.upperBound.x - aabb.lowerBound.x < Box2D.Huge);
        Debug.Assert(aabb.upperBound.y - aabb.lowerBound.y < Box2D.Huge);
        Debug.Assert(0 <= proxyId && proxyId < nodes.Length);
        Debug.Assert(nodes[proxyId].IsLeaf());
        Debug.Assert(!nodes[proxyId].aabb.Contains(aabb));
        nodes[proxyId].aabb = aabb;
        int parentIndex = nodes[proxyId].parent;
        while (parentIndex != -1)
        {
            bool changed = nodes[parentIndex].aabb.Enlarge(aabb);
            nodes[parentIndex].flags |= (ushort)TreeNodeFlags.Enlarged;
            parentIndex = nodes[parentIndex].parent;
            if (!changed) break;
        }
        while (parentIndex != -1)
        {
            if (((TreeNodeFlags)nodes[parentIndex].flags).HasFlag(TreeNodeFlags.Enlarged)) break;
            nodes[parentIndex].flags |= (ushort)TreeNodeFlags.Enlarged;
            parentIndex = nodes[parentIndex].parent;
        }
    }
    /// <summary>Modify the category bits on a proxy. This is an expensive operation.</summary>
    public void SetCategoryBits(int proxyId, ulong categoryBits)
    {
        Debug.Assert(nodes[proxyId].child1 == -1);
        Debug.Assert(nodes[proxyId].child2 == -1);
        Debug.Assert(((TreeNodeFlags)nodes[proxyId].flags).HasFlag(TreeNodeFlags.Leaf));
        nodes[proxyId].categoryBits = categoryBits;
        int nodeIndex = nodes[proxyId].parent;
        while (nodeIndex != -1)
        {
            ref TreeNode node = ref nodes[nodeIndex];
            int child1 = node.child1; Debug.Assert(child1 != -1);
            int child2 = node.child2; Debug.Assert(child2 != -1);
            node.categoryBits = nodes[child1].categoryBits | nodes[child2].categoryBits;
            nodeIndex = node.parent;
        }
    }
    /// <summary>Get the category bits on a proxy.</summary>
    public ulong GetCategoryBits(int proxyId)
    {
        Debug.Assert(0 <= proxyId && proxyId < nodes.Length);
        return nodes[proxyId].categoryBits;
    }
    /// <summary>Query an AABB for overlapping proxies. The callback class is called for each proxy that overlaps the supplied AABB.</summary>
    /// <returns>performance data</returns>
    public TreeStats Query(AABB aabb, ulong maskBits, TreeQueryCallbackFcn callback, object context)
    {
        TreeStats result = new();
        if (nodeCount == 0) return result;
        Stack<int> stack = new(1024);
        stack.Push(root);
        while (stack.Count > 0)
        {
            int nodeId = stack.Pop();
            ref TreeNode node = ref nodes[nodeId];
            result.nodeVisits++;
            if (AABB.Overlaps(node.aabb, aabb) && (node.categoryBits & maskBits) != 0)
            {
                if (node.IsLeaf())
                {
                    bool proceed = callback(nodeId, node.userData, context);
                    result.leafVisits++;
                    if (!proceed) return result;
                }
                else
                {
                    stack.Push(node.child1); stack.Push(node.child2);
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
        if (nodeCount == 0) return result;
        Stack<int> stack = new(1024);
        stack.Push(root);
        while (stack.Count > 0)
        {
            int nodeId = stack.Pop();
            ref TreeNode node = ref nodes[nodeId];
            result.nodeVisits++;
            if (AABB.Overlaps(node.aabb, aabb))
            {
                if (node.IsLeaf())
                {
                    bool proceed = callback(nodeId, node.userData, context);
                    result.leafVisits++;
                    if (!proceed) return result;
                }
                else
                {
                    stack.Push(node.child1); stack.Push(node.child2);
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
    public TreeStats RayCast(ref RayCastInput input, ulong maskBits, TreeRayCastCallbackFcn callback, object context)
    {
        TreeStats result = new();
        if (nodeCount == 0) return result;
        Vector2 p1 = input.origin, d = input.translation, r = d.Normalize();
        Vector2 v = Vector2.CrossSV(1, r), abs_v = v.Abs();
        float maxFraction = input.maxFraction;
        Vector2 p2 = Vector2.MulAdd(p1, maxFraction, d);
        AABB segmentAABB = new(Vector2.Min(p1, p2), Vector2.Max(p1, p2));
        Stack<int> stack = new(1024);
        stack.Push(root);
        RayCastInput subInput = input;
        while (stack.Count > 0)
        {
            int nodeId = stack.Pop();
            if (nodeId == -1) throw new IndexOutOfRangeException("Invalid dynamic tree shape cast index");
            ref TreeNode node = ref nodes[nodeId];
            result.nodeVisits++;
            AABB nodeAABB = node.aabb;
            if ((node.categoryBits & maskBits) == 0 || !AABB.Overlaps(nodeAABB, segmentAABB)) continue;
            Vector2 c = nodeAABB.Center(), h = nodeAABB.Extents();
            float term1 = Math.Abs(Vector2.Dot(v, p1 - c));
            float term2 = Vector2.Dot(abs_v, h);
            if (term2 < term1) continue;
            if (node.IsLeaf())
            {
                subInput.maxFraction = maxFraction;
                float value = callback(ref subInput, nodeId, node.userData, context);
                result.leafVisits++;
                if (value == 0) return result;
                if (0 < value && value <= maxFraction)
                {
                    maxFraction = value;
                    p2 = Vector2.MulAdd(p1, maxFraction, d);
                    segmentAABB.lowerBound = Vector2.Min(p1, p2);
                    segmentAABB.upperBound = Vector2.Max(p1, p2);
                }
            }
            else
            {
                Vector2 c1 = nodes[node.child1].aabb.Center();
                Vector2 c2 = nodes[node.child2].aabb.Center();
                if (Vector2.DistanceSquared(c1, p1) < Vector2.DistanceSquared(c2, p1))
                {
                    stack.Push(node.child2); stack.Push(node.child1);
                }
                else
                {
                    stack.Push(node.child1); stack.Push(node.child2);
                }
            }
        }
        return result;
    }
    /// <summary>Ray cast against the proxies in the tree. This relies on the callback
    /// to perform a exact ray cast in the case were the proxy contains a shape.
    /// The callback also performs the any collision filtering. This has performance
    /// roughly equal to k * log(n), where k is the number of collisions and n is the
    /// number of proxies in the tree.</summary>
    /// <param name="input">the ray cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).</param>
    /// <param name="maskBits">filter bits: `bool accept = (maskBits &amp; node->categoryBits) != 0;</param>
    /// <param name="callback">a callback class that is called for each proxy that is hit by the shape</param>
    /// <param name="context">user context that is passed to the callback</param>
    /// <returns>performance data</returns>
    public TreeStats ShapeCast(ref ShapeCastInput input, ulong maskBits, TreeShapeCastCallbackFcn callback, object context)
    {
        TreeStats stats = new();
        if (nodeCount == 0 || input.proxy.points.Length == 0) return stats;
        AABB originAABB = new(input.proxy.points[0], input.proxy.points[0]);
        for (int i = 1; i < input.proxy.points.Length; i++)
        {
            originAABB.lowerBound = Vector2.Min(originAABB.lowerBound, input.proxy.points[1]);
            originAABB.upperBound = Vector2.Max(originAABB.upperBound, input.proxy.points[1]);
        }
        Vector2 radius = new(input.proxy.radius, input.proxy.radius);
        originAABB.lowerBound -= radius;
        originAABB.upperBound += radius;
        Vector2 p1 = originAABB.Center(), extension = originAABB.Extents();
        Vector2 r = input.translation;
        Vector2 v = Vector2.CrossSV(1, r);
        Vector2 abs_v = v.Abs();
        float maxFraction = input.maxFraction;
        Vector2 t = maxFraction * input.translation;
        AABB totalAABB = new(Vector2.Min(originAABB.lowerBound, originAABB.lowerBound + t),
            Vector2.Max(originAABB.upperBound, originAABB.upperBound + t));
        ShapeCastInput subInput = input;
        Stack<int> stack = new(1024);
        stack.Push(root);
        while (stack.Count > 0)
        {
            int nodeId = stack.Pop();
            if (nodeId == -1) throw new IndexOutOfRangeException("Invalid dynamic tree shape cast index");
            ref TreeNode node = ref nodes[nodeId];
            stats.nodeVisits++;
            if ((node.categoryBits & maskBits) == 0 || !AABB.Overlaps(node.aabb, totalAABB)) continue;
            Vector2 c = node.aabb.Center(), h = node.aabb.Extents() + extension;
            float term1 = Math.Abs(Vector2.Dot(v, p1 - c));
            float term2 = Vector2.Dot(abs_v, h);
            if (term2 < term1) continue;
            if (node.IsLeaf())
            {
                subInput.maxFraction = maxFraction;
                float value = callback(ref subInput, nodeId, node.userData, context);
                stats.leafVisits++;
                if (value == 0) return stats;
                if (0 < value && value < maxFraction)
                {
                    maxFraction = value;
                    t = maxFraction * input.translation;
                    totalAABB.lowerBound = Vector2.Min(originAABB.lowerBound, originAABB.lowerBound + t);
                    totalAABB.upperBound = Vector2.Max(originAABB.upperBound, originAABB.upperBound + t);
                }
            }
            else
            {
                Vector2 c1 = nodes[node.child1].aabb.Center();
                Vector2 c2 = nodes[node.child2].aabb.Center();
                if (Vector2.DistanceSquared(c1, p1) < Vector2.DistanceSquared(c2, p1))
                {
                    stack.Push(node.child2); stack.Push(node.child1);
                }
                else
                {
                    stack.Push(node.child1); stack.Push(node.child2);
                }
            }
        }
        return stats;
    }
    ///<summary> Get the height of the binary tree.</summary>
    public int GetHeight() => root == -1 ? 0 : nodes[root].height;
    ///<summary> Get the ratio of the sum of the node areas to the root area.</summary>
    public float GetAreaRatio()
    {
        if (root == -1) return 0;
        ref TreeNode root_ = ref nodes[root];
        float rootArea = root_.aabb.Perimeter();
        float totalArea = 0;
        for (int i = 0; i < nodes.Length; i++)
        {
            ref TreeNode node = ref nodes[i];
            if (i == root || !node.IsAllocated() || node.IsLeaf()) continue;
            totalArea += node.aabb.Perimeter();
        }
        return totalArea / rootArea;
    }
    ///<summary> Get the bounding box that contains the entire tree</summary>
    public AABB GetRootBounds() => root != -1 ? nodes[root].aabb : new();
    int ComputeHeight(int nodeId)
    {
        Debug.Assert(0 <= nodeId && nodeId < nodes.Length);
        ref TreeNode node = ref nodes[nodeId];
        if (node.IsLeaf()) return 0;
        int height1 = ComputeHeight(node.child1), height2 = ComputeHeight(node.child2);
        return 1 + Math.Max(height1, height2);
    }
    void ValidateStructure(int index)
    {
        if (index == -1) return;
        if (index == root) Debug.Assert(nodes[index].parent == -1);
        ref TreeNode node = ref nodes[index];
        Debug.Assert(node.flags == 0 || ((TreeNodeFlags)node.flags).HasFlag(TreeNodeFlags.Allocated));
        if (node.IsLeaf())
        {
            Debug.Assert(node.height == 0);
            return;
        }
        int child1 = node.child1, child2 = node.child2;
        Debug.Assert(0 <= child1 && child1 < nodes.Length);
        Debug.Assert(0 <= child2 && child2 < nodes.Length);
        Debug.Assert(nodes[child1].parent == index);
        Debug.Assert(nodes[child2].parent == index);
        if (((TreeNodeFlags)(nodes[child1].flags | nodes[child2].flags)).HasFlag(TreeNodeFlags.Enlarged))
            Debug.Assert(((TreeNodeFlags)node.flags).HasFlag(TreeNodeFlags.Enlarged));
        ValidateStructure(child1);
        ValidateStructure(child2);
    }
    void ValidateMetrics(int index)
    {
        if (index == -1) return;
        ref TreeNode node = ref nodes[index];
        if (node.IsLeaf())
        {
            Debug.Assert(node.height == 0);
            return;
        }
        int child1 = node.child1, child2 = node.child2;
        Debug.Assert(0 <= child1 && child1 < nodes.Length);
        Debug.Assert(0 <= child2 && child2 < nodes.Length);
        int height1 = nodes[child1].height, height2 = nodes[child2].height;
        int height = 1 + Math.Max(height1, height2);
        Debug.Assert(node.height == height);
        Debug.Assert(node.aabb.Contains(nodes[child1].aabb));
        Debug.Assert(node.aabb.Contains(nodes[child2].aabb));
        ulong categoryBits = nodes[child1].categoryBits | nodes[child2].categoryBits;
        Debug.Assert(node.categoryBits == categoryBits);
        ValidateMetrics(child1);
        ValidateMetrics(child2);
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
    /// <summary>Temporary data used to track the rebuild of a tree node</summary>
    class RebuildItem
    {
        public int nodeIndex, childCount, startIndex, splitIndex, endIndex;
    }
    int BuildTree(int leafCount)
    {
        if (leafCount == 1)
        {
            nodes[leafIndices[0]].parent = -1;
            return leafIndices[0];
        }
        Stack<RebuildItem> stack = new(1024);
        stack.Push(new()
        {
            nodeIndex = AllocateNode(),
            childCount = -1,
            startIndex = 0,
            endIndex = leafCount,
            splitIndex = PartitionMid(leafIndices, leafCenters, leafCount)
        });
        while (true)
        {
            RebuildItem item = stack.Peek();
            item.childCount++;
            if (item.childCount == 2)
            {
                if (stack.Count == 1) break;
                stack.Pop();
                RebuildItem parentItem = stack.Peek();
                ref TreeNode parentNode = ref nodes[parentItem.nodeIndex];
                if (parentItem.childCount == 0)
                {
                    Debug.Assert(parentNode.child1 == -1);
                    parentNode.child1 = item.nodeIndex;
                }
                else
                {
                    Debug.Assert(parentItem.childCount == 1);
                    Debug.Assert(parentNode.child2 == -1);
                    parentNode.child2 = item.nodeIndex;
                }
                ref TreeNode node = ref nodes[item.nodeIndex];
                Debug.Assert(node.parent == -1);
                node.parent = parentItem.nodeIndex;
                Debug.Assert(node.child1 != -1);
                Debug.Assert(node.child2 != -1);
                ref TreeNode child1 = ref nodes[node.child1], child2 = ref nodes[node.child2];
                node.aabb = AABB.Union(child1.aabb, child2.aabb);
                node.height = (ushort)(1 + Math.Max(child1.height, child2.height));
                node.categoryBits = child1.categoryBits | child2.categoryBits;
            }
            else
            {
                int startIndex, endIndex;
                if (item.childCount == 0)
                {
                    startIndex = item.startIndex;
                    endIndex = item.splitIndex;
                }
                else
                {
                    Debug.Assert(item.childCount == 1);
                    startIndex = item.splitIndex;
                    endIndex = item.endIndex;
                }
                int count = endIndex - startIndex;
                if (count == 1)
                {
                    int childIndex = leafIndices[startIndex];
                    ref TreeNode node = ref nodes[item.nodeIndex];
                    if (item.childCount == 0)
                    {
                        Debug.Assert(node.child1 == -1);
                        node.child1 = childIndex;
                    }
                    else
                    {
                        Debug.Assert(item.childCount == 1);
                        Debug.Assert(node.child2 == -1);
                        node.child2 = childIndex;
                    }
                    ref TreeNode childNode = ref nodes[childIndex];
                    Debug.Assert(childNode.parent == -1);
                    childNode.parent = item.nodeIndex;
                }
                else
                {
                    Debug.Assert(count > 0);
                    stack.Push(new()
                    {
                        nodeIndex = AllocateNode(),
                        childCount = -1,
                        startIndex = startIndex,
                        endIndex = endIndex,
                        splitIndex = startIndex + PartitionMid(leafIndices.AsSpan(startIndex), leafCenters.AsSpan(startIndex), count)
                    });
                }
            }
        }
        ref TreeNode rootNode = ref nodes[stack.First().nodeIndex];
        Debug.Assert(rootNode.parent == -1);
        Debug.Assert(rootNode.child1 != -1);
        Debug.Assert(rootNode.child2 != -1);
        {
            ref TreeNode child1 = ref nodes[rootNode.child1], child2 = ref nodes[rootNode.child2];
            rootNode.aabb = AABB.Union(child1.aabb, child2.aabb);
            rootNode.height = (ushort)(1 + Math.Max(child1.height, child2.height));
            rootNode.categoryBits = child1.categoryBits | child2.categoryBits;
        }
        return stack.First().nodeIndex;
    }
    ///<summary> Rebuild the tree while retaining subtrees that haven't changed. Returns the number of boxes sorted.</summary>
    public int Rebuild(bool fullBuild)
    {
        if (proxyCount == 0) return 0;
        if (proxyCount > rebuildCapacity)
        {
            int newCapacity = proxyCount + proxyCount / 2;
            leafIndices = new int[newCapacity];
            leafCenters = new Vector2[newCapacity];
        }
        int leafCount = 0;
        Stack<int> stack = new(1024);
        int nodeIndex = root;
        ref TreeNode node = ref nodes[nodeIndex];
        while (true)
        {
            if (node.height == 0 || (!fullBuild && !((TreeNodeFlags)node.flags).HasFlag(TreeNodeFlags.Enlarged)))
            {
                leafIndices[leafCount] = nodeIndex;
                leafCenters[leafCount] = node.aabb.Center();
                leafCount++;
                node.parent = -1;
            }
            else
            {
                int doomedNodeIndex = nodeIndex;
                nodeIndex = node.child1;
                stack.Push(node.child2);
                node = ref nodes[nodeIndex];
                FreeNode(doomedNodeIndex);
                continue;
            }
            if (stack.Count == 0) break;
            nodeIndex = stack.Pop();
            node = ref nodes[nodeIndex];
        }
        Debug.Assert(leafCount <= proxyCount);
        root = BuildTree(leafCount);
        Validate();
        return leafCount;
    }
    ///<summary> Get the number of bytes used by this tree</summary>
    public unsafe int GetByteCount() => 20 + nodes.Length * sizeof(TreeNode)
        + rebuildCapacity * (sizeof(int) + sizeof(AABB) + sizeof(Vector2) + sizeof(int));
    ///<summary> Get proxy user data</summary>
    public ulong GetUserData(int proxyId)
    {
        Debug.Assert(0 <= proxyId && proxyId < nodes.Length);
        return nodes[proxyId].userData;
    }
    ///<summary> Get the AABB of a proxy</summary>
    public AABB GetAABB(int proxyId)
    {
        Debug.Assert(0 <= proxyId && proxyId < nodes.Length);
        return nodes[proxyId].aabb;
    }
    ///<summary> Validate this tree. For testing.</summary>
    public void Validate() { }
    ///<summary> Validate this tree has no enlarged AABBs. For testing.</summary>
    public void ValidateNoEnlarged() { }
}