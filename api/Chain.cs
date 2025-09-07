using System;
using System.Diagnostics;
using System.Collections.Generic;

namespace Box2D.API;

public static class ChainAPI
{
    ///<summary>Chain Shape
    /// Create a chain shape
    /// @see ChainDef for details</summary>
    public static ChainID CreateChain(BodyID bodyId, ref ChainDef def)
    {
        Debug.Assert(def.internalValue == Box2D.SECRET_COOKIE);
        Debug.Assert(def.points.Length >= 4);
        Debug.Assert(def.materials.Length == 1 || def.materials.Length == def.points.Length);
        World world = World.GetWorldLocked(bodyId.world0);
        if (world == null) return new();
        Body body = world.GetBodyFullID(bodyId);
        Transform transform = world.GetBodyTransformQuick(body);
        int chainId = world.chainIdPool.AllocId();
        if (chainId == world.chainShapes.Count) world.chainShapes.Add(new());
        else Debug.Assert(world.chainShapes[chainId].id == -1);
        ChainShape chainShape = world.chainShapes[chainId];
        chainShape.id = chainId;
        chainShape.bodyId = body.id;
        chainShape.nextChainId = body.headChainId;
        chainShape.generation++;
        int materialCount = def.materials.Length;
        chainShape.materials = new SurfaceMaterial[materialCount];
        for (int i = 0; i < materialCount; i++)
        {
            ref SurfaceMaterial material = ref def.materials[i];
            Debug.Assert(float.IsFinite(material.friction) && material.friction >= 0);
            Debug.Assert(float.IsFinite(material.restitution) && material.restitution >= 0);
            Debug.Assert(float.IsFinite(material.rollingResistance) && material.rollingResistance >= 0);
            Debug.Assert(float.IsFinite(material.tangentSpeed));
            chainShape.materials[i] = material;
        }
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
        int n = def.points.Length;
        if (def.isLoop)
        {
            chainShape.shapeIndices = new int[n];
            ChainSegment chainSegment = new();
            for (int i = 0, prevIndex = n - 1; i < n - 2; prevIndex = i++)
            {
                chainSegment.ghost1 = points[prevIndex];
                chainSegment.segment.point1 = points[i];
                chainSegment.segment.point2 = points[i + 1];
                chainSegment.ghost1 = points[i + 2];
                chainSegment.chainId = chainId;
                int materialIndex = materialCount == 1 ? 0 : i;
                shapeDef.material = def.materials[materialIndex];
                Shape shape = world.CreateShapeInternal(body, transform, ref shapeDef, chainSegment, ShapeType.ChainSegment);
                chainShape.shapeIndices[i] = shape.id;
            }
            {
                chainSegment.ghost1 = points[n - 3];
                chainSegment.segment.point1 = points[n - 2];
                chainSegment.segment.point2 = points[n - 1];
                chainSegment.ghost2 = points[0];
                chainSegment.chainId = chainId;
                int materialIndex = materialCount == 1 ? 0 : n - 2;
                shapeDef.material = def.materials[materialIndex];
                Shape shape = world.CreateShapeInternal(body, transform, ref shapeDef, chainSegment, ShapeType.ChainSegment);
                chainShape.shapeIndices[n - 2] = shape.id;
            }
            {
                chainSegment.ghost1 = points[n - 2];
                chainSegment.segment.point1 = points[n - 1];
                chainSegment.segment.point2 = points[0];
                chainSegment.ghost2 = points[1];
                chainSegment.chainId = chainId;
                int materialIndex = materialCount == 1 ? 0 : n - 1;
                shapeDef.material = def.materials[materialIndex];
                Shape shape = world.CreateShapeInternal(body, transform, ref shapeDef, chainSegment, ShapeType.ChainSegment);
            }
        }
        else
        {
            chainShape.shapeIndices = new int[n - 3];
            ChainSegment chainSegment = new();
            for (int i = 0; i < n - 3; i++)
            {
                chainSegment.ghost1 = points[i];
                chainSegment.segment.point1 = points[i + 1];
                chainSegment.segment.point2 = points[i + 2];
                chainSegment.ghost2 = points[i + 3];
                chainSegment.chainId = chainId;
                int materialIndex = materialCount == 1 ? 0 : i + 1;
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
            world.DestroyShapeInternal(shape, body, true);
        }
        world.chainIdPool.FreeId(chain.id);
        chain.id = -1;
        world.ValidateSolverSets();
    }

    ///<summary> Get the world that owns this chain shape</summary>
    public static WorldID Chain_GetWorld(ChainID chainId) => new() { index1 = chainId.world0, generation = chainId.world0.generation };

    ///<summary> Get the number of segments on this chain</summary>
    public static int Chain_GetSegmentCount(ChainID chainId)
    {
        World world = World.GetWorldLocked(chainId.world0); if (world == null) return 0;
        return world.GetChainShape(chainId).shapeIndices.Length;
    }

    ///<summary>Fill a user array with chain segment shape ids up to the specified capacity. Returns
    /// the actual number of segments returned.</summary>
    public static int Chain_GetSegments(ChainID chainId, ShapeID[] segmentArray, int capacity)
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

    /// <summary>Get the number of materials used on this chain. Must be 1 or the number of segments.</summary>
    public static int Chain_GetSurfaceMaterialCount(ChainID chainId) => chainId.world0.GetChainShape(chainId).materialCount;

    /// <summary>Set a chain material. If the chain has only one material, this material is applied to all
    /// segments. Otherwise it is applied to a single segment. </summary>
    public static void Chain_SetSurfaceMaterial(ChainID chainId, ref SurfaceMaterial material, int materialIndex)
    {
        World world = World.GetWorldLocked(chainId.world0); if (world == null) return;
        ChainShape chainShape = world.GetChainShape(chainId);
        Debug.Assert(0 <= materialIndex && materialIndex < chainShape.materialCount);
        chainShape.materials[materialIndex] = material;
        Debug.Assert(chainShape.materials.Length == 1 || chainShape.materials.Length == chainShape.shapeIndices.Length);
        int count = chainShape.shapeIndices.Length;
        if (chainShape.materials.Length == 1)
            for (int i = 0; i < count; i++)
                world.shapes[chainShape.shapeIndices[i]].material = material;
        else
        {
            world.shapes[chainShape.shapeIndices[materialIndex]].material = material;
        }
    }

    /// <summary>Get a chain material by index.</summary>
    public static SurfaceMaterial Chain_GetSurfaceMaterial(ChainID chainId, int segmentIndex)
    {
        ChainShape chainShape = chainId.world0.GetChainShape(chainId);
        Debug.Assert(0 <= segmentIndex && segmentIndex < chainShape.shapeIndices.Length);
        return chainShape.materials[segmentIndex];
    }

    ///<summary> Chain identifier validation. Provides validation for up to 64K allocations.</summary>
    public static bool Chain_IsValid(ChainID id)
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
