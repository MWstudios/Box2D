using System;
using System.Diagnostics;

namespace Box2D.API;

public static class ChainAPI
{
    ///<summary>Chain Shape
    /// Create a chain shape
    /// @see ChainDef for details</summary>
    public static ChainID CreateChain(BodyID bodyId, ref ChainDef def)
    {
        Debug.Assert(def.internalValue == Box2D.SECRET_COOKIE);
        Debug.Assert(def.isLoop ? def.points.Length >= 3 : def.points.Length >= 2);
        int segmentCount = def.isLoop ? def.points.Length : def.points.Length - 1;
        Debug.Assert(def.materials.Length == 1 || def.materials.Length == segmentCount);
        World world = World.GetWorldLocked(bodyId.world0);
        if (world == null) return new();
        Body body = world.GetBodyFullID(bodyId);
        WorldTransform transform = world.GetBodyTransformQuick(body);
        int chainId = world.chainIdPool.AllocId();
        if (chainId == world.chainShapes.Count) world.chainShapes.Add(new());
        else Debug.Assert(world.chainShapes[chainId].id == -1);
        ChainShape chainShape = world.chainShapes[chainId];
        chainShape.id = chainId;
        chainShape.bodyId = body.id;
        chainShape.nextChainId = body.headChainId;
        chainShape.segmentCount = segmentCount;
        chainShape.generation++;
        int materialCount = def.materials.Length;
        body.headChainId = chainId;
        ShapeDef shapeDef = new()
        {
            userData = def.userData,
            filter = def.filter,
            enableSensorEvents = def.enableSensorEvents,
            enableContactEvents = false,
            enableHitEvents = false
        };
        Vector2[] points = def.points;
        int n = segmentCount;
        chainShape.shapeIndices = new int[n];
        float tolSqr = Box2D.LinearSlop * Box2D.LinearSlop;
        if (def.isLoop)
        {
            for (int i = 0, prevIndex = n - 1; i < n; prevIndex = i++)
            {
                ChainSegment chainSegment = new();
                chainSegment.ghost1 = points[prevIndex];
                chainSegment.segment.point1 = points[i];
                chainSegment.segment.point2 = points[(i + 1) % n];
                chainSegment.ghost2 = points[(i + 2) % n];
                Debug.Assert(Vector2.DistanceSquared(chainSegment.ghost1, chainSegment.segment.point1) > tolSqr);
                Debug.Assert(Vector2.DistanceSquared(chainSegment.segment.point1, chainSegment.segment.point2) > tolSqr);
                chainSegment.chainId = chainId;
                int materialIndex = materialCount == 1 ? 0 : i;
                shapeDef.material = def.materials[materialIndex];
                Shape shape = world.CreateShapeInternal(body, transform, ref shapeDef, chainSegment, ShapeType.ChainSegment);
                chainShape.shapeIndices[i] = shape.id;
            }
        }
        else
        {
            Debug.Assert(def.ghost1.IsValid());
            Debug.Assert(def.ghost2.IsValid());
            for (int i = 0; i < n; i++)
            {
                ChainSegment chainSegment = new();
                Debug.Assert(i + 1 < def.points.Length);
                chainSegment.ghost1 = i == 0 ? def.ghost1 : points[i - 1];
                chainSegment.segment.point1 = points[i + 0];
                chainSegment.segment.point2 = points[i + 1];
                chainSegment.ghost2 = i == n - 1 ? def.ghost2 : points[i + 2];
                Debug.Assert(Vector2.DistanceSquared(chainSegment.ghost1, chainSegment.segment.point1) > tolSqr);
                Debug.Assert(Vector2.DistanceSquared(chainSegment.segment.point1, chainSegment.segment.point2) > tolSqr);
                Debug.Assert(Vector2.DistanceSquared(chainSegment.segment.point2, chainSegment.ghost2) > tolSqr);
                chainSegment.chainId = chainId;
                int materialIndex = materialCount == 1 ? 0 : i;
                def.materials[materialIndex].Validate();
                shapeDef.material = def.materials[materialIndex];
                Shape shape = world.CreateShapeInternal(body, transform, ref shapeDef, chainSegment, ShapeType.ChainSegment);
                chainShape.shapeIndices[i] = shape.id;
            }
        }
        return new() { index1 = chainId + 1, world0 = world, generation = chainShape.generation };
    }

    ///<summary> Destroy a chain shape</summary>
    public static void DestroyChain(ChainID chainId)
    {
        World world = World.GetWorldLocked(chainId.world0); if (world == null) return;
        ChainShape chain = world.GetChainShape(chainId);
        Body body = world.bodies[chain.bodyId];
        ref int chainIdPtr = ref body.headChainId;
        bool found = false;
        while (chainIdPtr != -1)
        {
            if (chainIdPtr == chain.id) { chainIdPtr = chain.nextChainId; found = true; break; }
            chainIdPtr = ref world.chainShapes[chainIdPtr].nextChainId;
        }
        Debug.Assert(found);
        if (!found) return;
        int count = chain.shapeIndices.Length;
        for (int i = 0; i < count; i++)
        {
            int shapeId = chain.shapeIndices[i];
            Shape shape = world.shapes[shapeId];
            world.DestroyShapeInternal(shape, body);
        }
        world.chainIdPool.FreeId(chain.id);
        chain.id = -1;
        world.ValidateSolverSets();
    }

    ///<summary> Get the world that owns this chain shape</summary>
    public static WorldID GetWorld(ChainID chainId) => new() { index1 = chainId.world0, generation = chainId.world0.generation };

    ///<summary> Get the number of segments on this chain</summary>
    public static int GetSegmentCount(ChainID chainId)
    {
        World world = World.GetWorldLocked(chainId.world0); if (world == null) return 0;
        return world.GetChainShape(chainId).shapeIndices.Length;
    }

    ///<summary>Fill a user array with chain segment shape ids up to the specified capacity. Returns
    /// the actual number of segments returned.</summary>
    public static int GetSegments(ChainID chainId, ShapeID[] segmentArray, int capacity)
    {
        World world = World.GetWorldLocked(chainId.world0); if (world == null) return 0;
        ChainShape chain = world.GetChainShape(chainId);
        int count = Math.Min(chain.shapeIndices.Length, capacity);
        for (int i = 0; i < count; i++)
        {
            int shapeId = chain.shapeIndices[i];
            Shape shape = world.shapes[shapeId];
            segmentArray[i] = new() { index1 = shapeId + 1, world0 = chainId.world0, generation = shape.generation };
        }
        return count;
    }

    /// <summary>Set the chain material on all segments.</summary>
    public static void SetAllSurfaceMaterials(ChainID chainId, ref SurfaceMaterial material)
    {
        material.Validate();
        World world = World.GetWorldLocked(chainId.world0); if (world == null) return;
        ChainShape chainShape = world.GetChainShape(chainId);
        for (int i = 0; i < chainShape.segmentCount; i++)
            world.shapes[chainShape.shapeIndices[i]].material = material;
    }

    /// <summary>Set a chain material by segment index.</summary>
    public static void SetSurfaceMaterial(ChainID chainId, ref SurfaceMaterial material, int segmentIndex)
    {
        material.Validate();
        World world = World.GetWorldLocked(chainId.world0); if (world == null) return;
        ChainShape chainShape = world.GetChainShape(chainId);
        Debug.Assert(0 <= segmentIndex && segmentIndex < chainShape.segmentCount);
        world.shapes[chainShape.shapeIndices[segmentIndex]].material = material;
    }

    /// <summary>Get a chain material by index.</summary>
    public static SurfaceMaterial GetSurfaceMaterial(ChainID chainId, int segmentIndex)
    {
        ChainShape chainShape = chainId.world0.GetChainShape(chainId);
        Debug.Assert(0 <= segmentIndex && segmentIndex < chainShape.shapeIndices.Length);
        return chainId.world0.shapes[chainShape.shapeIndices[segmentIndex]].material;
    }

    ///<summary> Chain identifier validation. Provides validation for up to 64K allocations.</summary>
    public static bool IsValid(ChainID id)
    {
        World world = id.world0;
        if (world == null) return false;
        int chainId = id.index1 - 1;
        if (chainId < 0 || world.chainShapes.Count <= chainId) return false;
        ChainShape chain = world.chainShapes[chainId];
        if (chain.id == -1) return false;
        Debug.Assert(chain.id == chainId);
        return id.generation == chain.generation;
    }
}
