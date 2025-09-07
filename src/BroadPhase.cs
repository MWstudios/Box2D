using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;

namespace Box2D;

public class MovePair
{
    public int shapeIndexA, shapeIndexB;
    public MovePair next;
    public bool heap;
}
public class MoveResult
{
    public MovePair pairList;
}
public struct QueryPairContext
{
    public World world;
    public MoveResult moveResult;
    public BodyType queryTreeType;
    public int queryProxyKey, queryShapeIndex;
}
/// <summary>The broad-phase is used for computing pairs and performing volume queries and ray casts.
/// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
/// It is up to the client to consume the new pairs and to track subsequent overlap.</summary>
public class BroadPhase
{
    static BodyType B2_PROXY_TYPE(int KEY) => (BodyType)((KEY) & 3);
    static int B2_PROXY_ID(int KEY) => KEY >> 2;
    static int B2_PROXY_KEY(int ID, int TYPE) => (ID << 2) | TYPE;
    static ulong B2_SHAPE_PAIR_KEY(ulong K1, ulong K2) => K1 < K2 ? (K2 << 32) | K2 : (K2 << 32) | K1;
    public DynamicTree[] trees = new DynamicTree[3];
    /// <summary>The move set and array are used to track shapes that have moved significantly
    /// and need a pair query for new contacts. The array has a deterministic order.
    /// todo perhaps just a move set?
    /// todo implement a 32bit hash set for faster lookup
    /// todo moveSet can grow quite large on the first time step and remain large</summary>
    public HashSet<int> moveSet = new(16);
    public List<int> moveArray = new(16);
    /// <summary>These are the results from the pair query and are used to create new contacts
    /// in deterministic order.
    /// todo these could be in the step context</summary>
    public MoveResult[] moveResults = null;
    public MovePair[] movePairs = null;
    public int movePairIndex = 0;
    /// <summary>Tracks shape pairs that have a b2Contact
    /// todo pairSet can grow quite large on the first time step and remain large</summary>
    public HashSet<ulong> pairSet = new(32);
    public BroadPhase()
    {
        for (int i = 0; i < trees.Length; i++) trees[i] = new();
    }
    public void Destroy() { for (int i = 0; i < trees.Length; i++) trees[i].Destroy(); }
    public void BufferMove(int queryProxy)
    {
        if (moveSet.Add(queryProxy)) moveArray.Add(queryProxy);
    }
    public void UnBufferMove(int proxyKey)
    {
        if (moveSet.Remove(proxyKey))
        {
            int count = moveArray.Count;
            for (int i = 0; i < count; i++) if (moveArray[i] == proxyKey)
                {
                    moveArray[i] = moveArray[^1]; moveArray.RemoveAt(moveArray.Count - 1); break;
                }
        }
    }
    public int CreateProxy(BodyType proxyType, AABB aabb, ulong categoryBits, int shapeIndex, bool forcePairCreation)
    {
        Debug.Assert(0 <= proxyType && (int)proxyType < 3);
        int proxyId = trees[(int)proxyType].CreateProxy(aabb, categoryBits, (ulong)shapeIndex);
        int proxyKey = B2_PROXY_KEY(proxyId, (int)proxyType);
        if (proxyType != BodyType.Static || forcePairCreation)
            BufferMove(proxyKey);
        return proxyKey;
    }
    public void DestroyProxy(int proxyKey)
    {
        Debug.Assert(moveArray.Count == moveSet.Count);
        UnBufferMove(proxyKey);
        BodyType proxyType = B2_PROXY_TYPE(proxyKey);
        int proxyId = B2_PROXY_ID(proxyKey);
        Debug.Assert(0 <= proxyType && (int)proxyType < 3);
        trees[(int)proxyType].DestroyProxy(proxyId);
    }
    public void MoveProxy(int proxyKey, AABB aabb)
    {
        BodyType proxyType = B2_PROXY_TYPE(proxyKey);
        int proxyId = B2_PROXY_ID(proxyKey);
        trees[(int)proxyType].MoveProxy(proxyId, aabb);
        BufferMove(proxyKey);
    }
    public void EnlargeProxy(int proxyKey, AABB aabb)
    {
        Debug.Assert(proxyKey != -1);
        BodyType typeIndex = B2_PROXY_TYPE(proxyKey);
        int proxyId = B2_PROXY_ID(proxyKey);
        Debug.Assert(typeIndex != BodyType.Static);
        trees[(int)typeIndex].EnlargeProxy(proxyId, aabb);
        BufferMove(proxyKey);
    }
    public bool TestOverlap(int proxyKeyA, int proxyKeyB)
    {
        int typeIndexA = (int)B2_PROXY_TYPE(proxyKeyA), proxyIdA = B2_PROXY_ID(proxyKeyA),
            typeIndexB = (int)B2_PROXY_TYPE(proxyKeyB), proxyIdB = B2_PROXY_ID(proxyKeyB);
        AABB aabbA = trees[typeIndexA].GetAABB(proxyIdA), aabbB = trees[typeIndexB].GetAABB(proxyIdB);
        return AABB.Overlaps(aabbA, aabbB);
    }
    public void RebuildTrees()
    {
        trees[(int)BodyType.Dynamic].Rebuild(false);
        trees[(int)BodyType.Kinematic].Rebuild(false);
    }
    public int GetShapeIndex(int proxyKey)
    {
        int typeIndex = (int)B2_PROXY_TYPE(proxyKey), proxyId = B2_PROXY_ID(proxyKey);
        return (int)trees[typeIndex].GetUserData(proxyId);
    }
    public void ValidateBroadphase()
    {
        trees[(int)BodyType.Dynamic].Validate();
        trees[(int)BodyType.Kinematic].Validate();
    }
    public void ValidateNoEnlarged()
    {
        //for (int j = 0; j < 3; j++) trees[j].ValidateNoEnlarged(); //???
    }
}
public partial class World
{
    static BodyType B2_PROXY_TYPE(int KEY) => (BodyType)((KEY) & 3);
    static int B2_PROXY_ID(int KEY) => KEY >> 2;
    static int B2_PROXY_KEY(int ID, int TYPE) => (ID << 2) | TYPE;
    static bool PairQueryCallback(int proxyId, ulong userData, object context)
    {
        int shapeId = (int)userData;
        QueryPairContext queryContext = (QueryPairContext)context;
        BroadPhase broadPhase = queryContext.world.broadPhase;
        int proxyKey = B2_PROXY_KEY(proxyId, (int)queryContext.queryTreeType);
        int queryProxyKey = queryContext.queryProxyKey;
        if (proxyKey == queryContext.queryProxyKey) return true;
        BodyType treeType = queryContext.queryTreeType, queryProxyType = B2_PROXY_TYPE(queryProxyKey);

        if (queryProxyType == BodyType.Dynamic)
        {
            if (treeType == BodyType.Dynamic && proxyKey < queryProxyKey)
            {
                if (broadPhase.moveSet.Contains(proxyKey)) return true;
            }
        }
        else
        {
            Debug.Assert(treeType == BodyType.Dynamic);
            if (broadPhase.moveSet.Contains(proxyKey)) return true;
        }
        ulong pairKey = B2_SHAPE_PAIR_KEY((ulong)shapeId, (ulong)queryContext.queryShapeIndex);
        if (broadPhase.pairSet.Contains(pairKey)) return true;
        int shapeIdA, shapeIdB;
        if (proxyKey < queryProxyKey) { shapeIdA = shapeId; shapeIdB = queryContext.queryShapeIndex; }
        else { shapeIdA = queryContext.queryShapeIndex; shapeIdB = shapeId; }
        World world = queryContext.world;
        Shape shapeA = world.shapes[shapeIdA], shapeB = world.shapes[shapeIdB];
        int bodyIdA = shapeA.bodyId, bodyIdB = shapeB.bodyId;
        if (bodyIdA == bodyIdB) return true;
        if (shapeA.sensorIndex != -1 || shapeB.sensorIndex != -1) return true;
        if (!Shape.ShouldShapesCollide(shapeA.filter, shapeB.filter)) return true;
        Body bodyA = world.bodies[bodyIdA], bodyB = world.bodies[bodyIdB];
        if (!world.ShouldBodiesCollide(bodyA, bodyB)) return true;
        if (shapeA.enableCustomFiltering || shapeB.enableCustomFiltering)
        {
            CustomFilterFcn customFilterFcn = queryContext.world.customFilterFcn;
            if (customFilterFcn != null)
            {
                ShapeID idA = new() { index1 = shapeIdA + 1, world0 = world, generation = shapeA.generation };
                ShapeID idB = new() { index1 = shapeIdB + 1, world0 = world, generation = shapeB.generation };
                if (!customFilterFcn(idA, idB, queryContext.world.customFilterContext)) return true;
            }
        }
        int pairIndex = Interlocked.Increment(ref broadPhase.movePairIndex) - 1;
        MovePair pair;
        if (pairIndex < broadPhase.movePairs.Length)
        {
            pair = broadPhase.movePairs[pairIndex]; pair.heap = false;
        }
        else
        {
            pair = new() { heap = true };
        }
        pair.shapeIndexA = shapeIdA; pair.shapeIndexB = shapeIdB;
        pair.next = queryContext.moveResult.pairList;
        queryContext.moveResult.pairList = pair;
        return true;
    }
    static void FindPairsTask(int startIndex, int endIndex, uint threadIndex, object context)
    {
        World world = (World)context;
        BroadPhase bp = world.broadPhase;
        QueryPairContext queryContext;
        queryContext.world = world;
        for (int i = startIndex; i < endIndex; i++)
        {
            queryContext.moveResult = bp.moveResults[i];
            queryContext.moveResult.pairList = null;
            int proxyKey = bp.moveArray[i];
            if (proxyKey == -1) continue;
            BodyType proxyType = B2_PROXY_TYPE(proxyKey);
            int proxyId = B2_PROXY_ID(proxyKey);
            queryContext.queryProxyKey = proxyKey;
            ref DynamicTree baseTree = ref bp.trees[(int)proxyType];
            AABB fatAABB = baseTree.GetAABB(proxyId);
            queryContext.queryShapeIndex = (int)baseTree.GetUserData(proxyId);
            TreeStats stats = new();
            if (proxyType == BodyType.Dynamic)
            {
                queryContext.queryTreeType = BodyType.Kinematic;
                TreeStats statsKinematic = bp.trees[(int)BodyType.Kinematic].Query(fatAABB, Box2D.DEFAULT_MASK_BITS, PairQueryCallback, queryContext);
                stats.nodeVisits += statsKinematic.nodeVisits;
                stats.leafVisits += statsKinematic.leafVisits;
                queryContext.queryTreeType = BodyType.Static;
                TreeStats statsStatic = bp.trees[(int)BodyType.Static].Query(fatAABB, Box2D.DEFAULT_MASK_BITS, PairQueryCallback, queryContext);
                stats.nodeVisits += statsStatic.nodeVisits;
                stats.leafVisits += statsStatic.leafVisits;
            }
            queryContext.queryTreeType = BodyType.Dynamic;
            TreeStats statsDynamic = bp.trees[(int)BodyType.Dynamic].Query(fatAABB, Box2D.DEFAULT_MASK_BITS, PairQueryCallback, queryContext);
            stats.nodeVisits += statsDynamic.nodeVisits;
            stats.leafVisits += statsDynamic.leafVisits;
        }
    }
    public void UpdateBroadPhasePairs()
    {
        BroadPhase bp = broadPhase;
        int moveCount = bp.moveArray.Count;
        Debug.Assert(moveCount == bp.moveSet.Count);
        if (moveCount == 0) return;
        bp.moveResults = new MoveResult[moveCount];
        for (int i = 0; i < bp.moveResults.Length; i++) bp.moveResults[i] = new();
        bp.movePairs = new MovePair[moveCount * 16];
        for (int i = 0; i < bp.movePairs.Length; i++) bp.movePairs[i] = new();
        Interlocked.Exchange(ref bp.movePairIndex, 0);
        int minRange = 64;
        object userPairTask = enqueueTaskFcn(FindPairsTask, moveCount, minRange, this, userTaskContext);
        if (userPairTask != null)
        {
            finishTaskFcn(userPairTask, userTaskContext);
            taskCount++;
        }
        for (int i = 0; i < moveCount; i++)
        {
            MoveResult result = bp.moveResults[i];
            MovePair pair = result.pairList;
            while (pair != null)
            {
                int shapeIdA = pair.shapeIndexA, shapeIdB = pair.shapeIndexB;
                Shape shapeA = shapes[shapeIdA], shapeB = shapes[shapeIdB];
                CreateContact(shapeA, shapeB);
                pair = pair.next;
            }
        }
        bp.moveArray.Clear();
        bp.moveSet.Clear();
        bp.movePairs = null;
        bp.moveResults = null;
        ValidateSolverSets();
    }
}