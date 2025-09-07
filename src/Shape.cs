using System.Collections.Generic;
using System.Diagnostics;

namespace Box2D;

public interface IShape
{
    public float GetRadius() => 0;
    public AABB ComputeAABB(Transform xf) => new(xf.p, xf.p);
    public Vector2 GetCentroid() => Vector2.Zero;
    public float GetPerimeter() => 0;
    public float GetProjectedPerimeter(Vector2 line) => 0;
    public MassData ComputeMass(float density) => new();
    public ShapeExtent ComputeExtent(Vector2 localCenter) => new();
    public CastOutput RayCast(ref RayCastInput input) => new();
    public CastOutput ShapeCast(ref ShapeCastInput input) => new();
    public PlaneResult CollideMover(ref Capsule mover) => new();
    public ShapeProxy MakeProxy() => new();
    public bool TestPoint(Vector2 point) => false;
}
public class Shape
{
    public int id;
    public int bodyId;
    public int prevShapeId;
    public int nextShapeId;
    public int sensorIndex;
    public ShapeType type;
    public SurfaceMaterial material;
    public float density;

    public AABB aabb;
    public AABB fatAABB;
    public Vector2 localCentroid;
    public int proxyKey;

    public Filter filter;
    public object userData;

    public IShape shape;

    public ushort generation;
    public bool enableSensorEvents;
    public bool enableContactEvents;
    public bool enableCustomFiltering;
    public bool enableHitEvents;
    public bool enablePreSolveEvents;
    public bool enlargedAABB;
    static BodyType B2_PROXY_TYPE(int KEY) => (BodyType)((KEY) & 3);
    static int B2_PROXY_ID(int KEY) => KEY >> 2;
    static int B2_PROXY_KEY(int ID, int TYPE) => (ID << 2) | TYPE;
    public float GetRadius() => shape.GetRadius();
    public static bool ShouldShapesCollide(Filter filterA, Filter filterB)
    {
        if (filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0)
            return filterA.groupIndex > 0;
        return (filterA.maskBits & filterB.categoryBits) != 0 && (filterA.categoryBits & filterB.maskBits) != 0;
    }
    public static bool ShouldQueryCollide(Filter shapeFilter, QueryFilter queryFilter) =>
        (shapeFilter.categoryBits & queryFilter.maskBits) != 0 && (shapeFilter.maskBits & queryFilter.categoryBits) != 0;
    public void UpdateAABBs(Transform transform, BodyType proxyType)
    {
        AABB aabb = shape.ComputeAABB(transform);
        aabb.lowerBound.x -= Box2D.SpeculativeDistance;
        aabb.lowerBound.y -= Box2D.SpeculativeDistance;
        aabb.upperBound.x += Box2D.SpeculativeDistance;
        aabb.upperBound.y += Box2D.SpeculativeDistance;
        this.aabb = aabb;
        float margin = proxyType == BodyType.Static ? Box2D.SpeculativeDistance : Box2D.AABBMargin;
        fatAABB = new(new(aabb.lowerBound.x - margin, aabb.lowerBound.y - margin),
            new(aabb.upperBound.x + margin, aabb.upperBound.y + margin));
    }
    public AABB ComputeAABB(Transform xf) => shape.ComputeAABB(xf);
    public Vector2 GetCentroid() => shape.GetCentroid();
    public float GetPerimeter() => shape.GetPerimeter();
    public float GetProjectedPerimeter(Vector2 line) => shape.GetProjectedPerimeter(line);
    public MassData ComputeMass() => shape.ComputeMass(density);
    public ShapeExtent ComputeExtent(Vector2 localCenter) => shape.ComputeExtent(localCenter);
    public CastOutput RayCast(ref RayCastInput input, Transform transform)
    {
        RayCastInput localInput = new()
        {
            origin = transform.InvTransformPoint(input.origin),
            translation = transform.q.InvRotateVector(input.translation),
            maxFraction = input.maxFraction
        };
        CastOutput output = shape.RayCast(ref localInput);
        output.point = transform.TransformPoint(output.point);
        output.normal = transform.q * output.normal;
        return output;
    }
    public CastOutput ShapeCast(ref ShapeCastInput input, Transform transform)
    {
        CastOutput output = new();
        if (input.proxy.points.Length == 0) return output;
        ShapeCastInput localInput = input;
        localInput.proxy = new() { points = new Vector2[input.proxy.points.Length], radius = input.proxy.radius };
        for (int i = 0; i < localInput.proxy.points.Length; i++)
            localInput.proxy.points[i] = transform.TransformPoint(input.proxy.points[i]);
        output = shape.ShapeCast(ref localInput);
        output.point = transform.TransformPoint(output.point);
        output.normal = transform.q * output.normal;
        return output;
    }
    public PlaneResult CollideMover(ref Capsule mover, Transform transform)
    {
        Capsule localMover = new()
        {
            center1 = transform.TransformPoint(mover.center1),
            center2 = transform.TransformPoint(mover.center2),
            radius = mover.radius
        };
        PlaneResult result = shape.CollideMover(ref localMover);
        if (!result.hit) return result;
        result.plane.normal = transform.q * result.plane.normal;
        return result;
    }
    public void CreateProxy(BroadPhase bp, BodyType type, Transform transform, bool forcePairCreation)
    {
        Debug.Assert(proxyKey == -1);
        UpdateAABBs(transform, type);
        proxyKey = bp.CreateProxy(type, fatAABB, filter.categoryBits, id, forcePairCreation);
        Debug.Assert((uint)B2_PROXY_TYPE(proxyKey) < 3);
    }
    public void DestroyProxy(BroadPhase bp)
    {
        if (proxyKey != -1) { bp.DestroyProxy(proxyKey); proxyKey = -1; }
    }
    public ShapeProxy MakeDistanceProxy() => shape.MakeProxy();
}
public class ChainShape
{
    public int id;
    public int bodyId;
    public int nextChainId;
    public int materialCount;
    public int[] shapeIndices;
    public SurfaceMaterial[] materials;
    public ushort generation;

}
public struct ShapeExtent
{
    public float minExtent;
    public float maxExtent;
}
public struct SensorOverlaps
{
    public List<int> overlaps = new();
    public SensorOverlaps() { }
}
public partial class World
{
    public Shape GetShape(ShapeID shapeId)
    {
        int id = shapeId.index1 - 1;
        Shape shape = shapes[id];
        Debug.Assert(shape.id == id /*&& shape.generation == generation*/);
        return shape;
    }
    public ChainShape GetChainShape(ChainID chainId)
    {
        int id = chainId.index1 - 1;
        ChainShape chain = chainShapes[id];
        Debug.Assert(chain.id == id && chain.generation == generation);
        return chain;
    }
    public Shape CreateShapeInternal(Body body, Transform transform, ref ShapeDef def, IShape geometry, ShapeType shapeType)
    {
        int shapeId = shapeIdPool.AllocId();
        if (shapeId == shapes.Count) shapes.Add(new());
        else Debug.Assert(shapes[shapeId].id == -1);
        Shape shape = shapes[shapeId];
        shape.shape = geometry;
        shape.id = shapeId;
        shape.bodyId = body.id;
        shape.type = shapeType;
        shape.density = def.density;
        shape.material = def.material;
        shape.filter = def.filter;
        shape.userData = def.userData;
        shape.enlargedAABB = false;
        shape.enableSensorEvents = def.enableSensorEvents;
        shape.enableContactEvents = def.enableContactEvents;
        shape.enableCustomFiltering = def.enableCustomFiltering;
        shape.enableHitEvents = def.enableHitEvents;
        shape.enablePreSolveEvents = def.enablePreSolveEvents;
        shape.proxyKey = -1;
        shape.localCentroid = shape.GetCentroid();
        shape.aabb = new();
        shape.fatAABB = new();
        shape.generation++;
        if (body.setIndex != (int)SetType.Disabled)
        {
            BodyType proxyType = body.type;
            shape.CreateProxy(broadPhase, proxyType, transform, def.invokeContactCreation || def.isSensor);
        }
        if (body.headShapeId != -1)
        {
            Shape headShape = shapes[body.headShapeId];
            headShape.prevShapeId = shapeId;
        }
        shape.prevShapeId = -1;
        shape.nextShapeId = body.headShapeId;
        body.headShapeId = shapeId;
        body.shapeCount++;
        if (def.isSensor)
        {
            shape.sensorIndex = sensors.Count;
            sensors.Add(new() { hits = new(4), overlaps1 = new(16), overlaps2 = new(16), shapeId = shapeId });
        }
        else shape.sensorIndex = -1;
        ValidateSolverSets();
        return shape;
    }
    public void DestroyShapeInternal(Shape shape, Body body, bool wakeBodies)
    {
        int shapeId = shape.id;
        if (shape.prevShapeId != -1)
        {
            Shape prevShape = shapes[shape.prevShapeId];
            prevShape.nextShapeId = shape.nextShapeId;
        }
        if (shape.nextShapeId != -1)
        {
            Shape nextShape = shapes[shape.nextShapeId];
            nextShape.prevShapeId = shape.prevShapeId;
        }
        if (shapeId == body.headShapeId) body.headShapeId = shape.nextShapeId;
        body.shapeCount--;
        shape.DestroyProxy(broadPhase);
        int contactKey = body.headContactKey;
        while (contactKey != -1)
        {
            int contactId = contactKey >> 1;
            int edgeIndex = contactKey & 1;
            Contact contact = contacts[contactId];
            contactKey = edgeIndex == 1 ? contact.edge1.nextKey : contact.edge0.nextKey;
            if (contact.shapeIdA == shapeId || contact.shapeIdB == shapeId)
                DestroyContact(contact, wakeBodies);
        }
        if (shape.sensorIndex != -1)
        {
            Sensor sensor = sensors[shape.sensorIndex];
            for (int i = 0; i < sensor.overlaps2.Count; i++)
            {
                if (endEventArrayIndex == 1) sensorEndEvents1.Add(new()
                {
                    sensorShapeId = new() { index1 = shapeId + 1, world0 = this, generation = shape.generation },
                    visitorShapeId = new() { index1 = sensor.overlaps2[i].shapeId + 1, world0 = this, generation = sensor.overlaps2[i].generation }
                });
                else sensorEndEvents0.Add(new()
                {
                    sensorShapeId = new() { index1 = shapeId + 1, world0 = this, generation = shape.generation },
                    visitorShapeId = new() { index1 = sensor.overlaps2[i].shapeId + 1, world0 = this, generation = sensor.overlaps2[i].generation }
                });
            }
            int movedIndex = sensors.RemoveSwap(shape.sensorIndex);
            if (movedIndex != -1)
            {
                Sensor movedSensor = sensors[shape.sensorIndex];
                Shape otherSensorShape = shapes[movedSensor.shapeId];
                otherSensorShape.sensorIndex = shape.sensorIndex;
            }
        }
        shapeIdPool.FreeId(shapeId);
        shape.id = -1;
        ValidateSolverSets();
    }
    public void ResetProxy(Shape shape, bool wakeBodies, bool destroyProxy)
    {
        Body body = bodies[shape.bodyId];
        int shapeId = shape.id;
        int contactKey = body.headContactKey;
        while (contactKey != -1)
        {
            int contactId = contactKey >> 1;
            int edgeIndex = contactKey & 1;
            Contact contact = contacts[contactId];
            contactKey = edgeIndex == 1 ? contact.edge1.nextKey : contact.edge0.nextKey;
            if (contact.shapeIdA == shapeId || contact.shapeIdB == shapeId)
                DestroyContact(contact, wakeBodies);
        }
        Transform transform = GetBodyTransformQuick(body);
        if (shape.proxyKey != -1)
        {
            BodyType proxyType = B2_PROXY_TYPE(shape.proxyKey);
            shape.UpdateAABBs(transform, proxyType);
            if (destroyProxy)
            {
                broadPhase.DestroyProxy(shape.proxyKey);
                shape.proxyKey = broadPhase.CreateProxy(proxyType, shape.fatAABB, shape.filter.categoryBits, shapeId, true);
            }
            else broadPhase.MoveProxy(shape.proxyKey, shape.fatAABB);
        }
        else shape.UpdateAABBs(transform, body.type);
        ValidateSolverSets();
    }
}