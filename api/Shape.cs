using System;
using System.Diagnostics;

namespace Box2D.API;

public static class ShapeAPI
{
    public static ShapeID CreateShape(BodyID bodyId, ref ShapeDef def, IShape geometry, ShapeType shapeType)
    {
        Debug.Assert(def.internalValue == Box2D.SECRET_COOKIE);
        Debug.Assert(float.IsFinite(def.density) && def.density >= 0);
        Debug.Assert(float.IsFinite(def.material.friction) && def.material.friction >= 0);
        Debug.Assert(float.IsFinite(def.material.restitution) && def.material.restitution >= 0);
        Debug.Assert(float.IsFinite(def.material.rollingResistance) && def.material.rollingResistance >= 0);
        Debug.Assert(float.IsFinite(def.material.tangentSpeed));
        World world = World.GetWorldLocked(bodyId.world0);
        if (world == null) return new();
        Body body = world.GetBodyFullID(bodyId);
        Transform transform = world.GetBodyTransformQuick(body);
        Shape shape = world.CreateShapeInternal(body, transform, ref def, geometry, shapeType);
        if (def.updateBodyMass) world.UpdateBodyMassData(body);
        world.ValidateSolverSets();
        return new() { index1 = shape.id + 1, world0 = world, generation = shape.generation };
    }
    ///<summary>Create a circle shape and attach it to a body. The shape definition and geometry are fully cloned.
    /// Contacts are not created until the next time step.</summary>
    /// <returns>the shape id for accessing the shape</returns>
    public static ShapeID CreateCircleShape(BodyID bodyId, ref ShapeDef def, Circle circle) => CreateShape(bodyId, ref def, circle, ShapeType.Circle);

    ///<summary>Create a line segment shape and attach it to a body. The shape definition and geometry are fully cloned.
    /// Contacts are not created until the next time step.</summary>
    /// <returns>the shape id for accessing the shape</returns>
    public static ShapeID CreateSegmentShape(BodyID bodyId, ref ShapeDef def, Segment segment) => CreateShape(bodyId, ref def, segment, ShapeType.Segment);

    ///<summary>Create a capsule shape and attach it to a body. The shape definition and geometry are fully cloned.
    /// Contacts are not created until the next time step.</summary>
    /// <returns>the shape id for accessing the shape</returns>
    public static ShapeID CreateCapsuleShape(BodyID bodyId, ref ShapeDef def, Capsule capsule) => CreateShape(bodyId, ref def, capsule, ShapeType.Capsule);

    ///<summary>Create a polygon shape and attach it to a body. The shape definition and geometry are fully cloned.
    /// Contacts are not created until the next time step.</summary>
    /// <returns>the shape id for accessing the shape</returns>
    public static ShapeID CreatePolygonShape(BodyID bodyId, ref ShapeDef def, Polygon polygon) => CreateShape(bodyId, ref def, polygon, ShapeType.Polygon);

    ///<summary>Destroy a shape. You may defer the body mass update which can improve performance if several shapes on a
    ///	body are destroyed at once.
    ///	@see Body_ApplyMassFromShapes</summary>
    public static void DestroyShape(ShapeID shapeId, bool updateBodyMass)
    {
        World world = World.GetWorldLocked(shapeId.world0);
        if (world == null) return;
        Shape shape = world.GetShape(shapeId);
        Body body = world.bodies[shape.bodyId];
        world.DestroyShapeInternal(shape, body, true);
        if (updateBodyMass) world.UpdateBodyMassData(body);
    }

    ///<summary> Shape identifier validation. Provides validation for up to 64K allocations.</summary>
    public static bool IsValid(ShapeID id)
    {
        World world = id.world0;
        if (world == null) return false;
        int shapeId = id.index1 - 1;
        if (shapeId < 0 || world.shapes.Count <= shapeId) return false;
        Shape shape = world.shapes[shapeId];
        if (shape.id == -1) return false;
        Debug.Assert(shape.id == shapeId);
        return id.generation == shape.generation;
    }

    ///<summary> Get the type of a shape</summary>
    public static ShapeType GetType(ShapeID shapeId) => shapeId.world0.GetShape(shapeId).type;

    ///<summary> Get the id of the body that a shape is attached to</summary>
    public static BodyID GetBody(ShapeID shapeId) => shapeId.world0.MakeBodyID(shapeId.world0.GetShape(shapeId).bodyId);

    ///<summary> Get the world that owns this shape</summary>
    public static WorldID GetWorld(ShapeID shapeId) => new() { index1 = shapeId.world0, generation = shapeId.world0.generation };

    ///<summary>Returns true if the shape is a sensor. It is not possible to change a shape
    /// from sensor to solid dynamically because this breaks the contract for
    /// sensor events.</summary>
    public static bool IsSensor(ShapeID shapeId) => shapeId.world0.GetShape(shapeId).sensorIndex != -1;

    ///<summary> Set the user data for a shape</summary>
    public static void SetUserData(ShapeID shapeId, object userData) => shapeId.world0.GetShape(shapeId).userData = userData;

    ///<summary>Get the user data for a shape. This is useful when you get a shape id
    /// from an event or query.</summary>
    public static object GetUserData(ShapeID shapeId) => shapeId.world0.GetShape(shapeId).userData;

    ///<summary>Set the mass density of a shape, usually in kg/m^2.
    /// This will optionally update the mass properties on the parent body.
    /// @see ShapeDef::density, Body_ApplyMassFromShapes</summary>
    public static void SetDensity(ShapeID shapeId, float density, bool updateBodyMass)
    {
        Debug.Assert(float.IsFinite(density) && density >= 0);
        World world = World.GetWorldLocked(shapeId.world0); if (world == null) return;
        Shape shape = world.GetShape(shapeId);
        if (density == shape.density) return;
        shape.density = density;
        if (updateBodyMass) world.UpdateBodyMassData(world.bodies[shape.bodyId]);
    }

    ///<summary> Get the density of a shape, usually in kg/m^2</summary>
    public static float GetDensity(ShapeID shapeId) => shapeId.world0.GetShape(shapeId).density;

    ///<summary>Set the friction on a shape
    /// @see ShapeDef::friction</summary>
    public static void SetFriction(ShapeID shapeId, float friction)
    {
        Debug.Assert(float.IsFinite(friction) &&friction >= 0);
        World world = shapeId.world0; Debug.Assert(!world.locked); if (world.locked) return;
        world.GetShape(shapeId).material.friction = friction;
    }

    ///<summary> Get the friction of a shape</summary>
    public static float GetFriction(ShapeID shapeId) => shapeId.world0.GetShape(shapeId).material.friction;

    ///<summary>Set the shape restitution (bounciness)
    /// @see ShapeDef::restitution</summary>
    public static void SetRestitution(ShapeID shapeId, float restitution)
    {
        Debug.Assert(float.IsFinite(restitution) && restitution >= 0);
        World world = shapeId.world0; Debug.Assert(!world.locked); if (world.locked) return;
        world.GetShape(shapeId).material.restitution = restitution;
    }

    ///<summary> Get the shape restitution</summary>
    public static float GetRestitution(ShapeID shapeId) => shapeId.world0.GetShape(shapeId).material.restitution;

    ///<summary>Set the user material identifier
    /// @see ShapeDef::material</summary>
    public static void SetUserMaterial(ShapeID shapeId, ulong material)
    {
        World world = shapeId.world0; Debug.Assert(!world.locked); if (world.locked) return;
        world.GetShape(shapeId).material.userMaterialId = material;
    }

    ///<summary> Get the user material identifier</summary>
    public static ulong GetMaterial(ShapeID shapeId) => shapeId.world0.GetShape(shapeId).material.userMaterialId;

    ///<summary> Set the shape surface material</summary>
    public static void SetSurfaceMaterial(ShapeID shapeId, SurfaceMaterial surfaceMaterial)
        => shapeId.world0.GetShape(shapeId).material = surfaceMaterial;

    ///<summary> Get the shape surface material</summary>
    public static SurfaceMaterial GetSurfaceMaterial(ShapeID shapeId) => shapeId.world0.GetShape(shapeId).material;

    ///<summary> Get the shape filter</summary>
    public static Filter GetFilter(ShapeID shapeId) => shapeId.world0.GetShape(shapeId).filter;

    ///<summary>Set the current filter. This is almost as expensive as recreating the shape. This may cause
    /// contacts to be immediately destroyed. However contacts are not created until the next world step.
    /// Sensor overlap state is also not updated until the next world step.
    /// @see ShapeDef::filter</summary>
    public static void SetFilter(ShapeID shapeId, Filter filter)
    {
        World world = World.GetWorldLocked(shapeId.world0); if (world == null) return;
        Shape shape = world.GetShape(shapeId);
        if (filter.maskBits == shape.filter.maskBits && filter.categoryBits == shape.filter.categoryBits &&
            filter.groupIndex == shape.filter.groupIndex) return;
        bool destroyProxy = filter.categoryBits != shape.filter.categoryBits;
        shape.filter = filter;
        world.ResetProxy(shape, true, destroyProxy);
    }

    ///<summary>Enable sensor events for this shape.
    /// @see ShapeDef::enableSensorEvents</summary>
    public static void EnableSensorEvents(ShapeID shapeId, bool flag)
    {
        World world = World.GetWorldLocked(shapeId.world0); if (world == null) return;
        world.GetShape(shapeId).enableSensorEvents = flag;
    }

    ///<summary> Returns true if sensor events are enabled.</summary>
    public static bool AreSensorEventsEnabled(ShapeID shapeId) => shapeId.world0.GetShape(shapeId).enableSensorEvents;

    ///<summary>Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
    /// @see ShapeDef::enableContactEvents</summary>
    /// <remarks> changing this at run-time may lead to lost begin/end events</remarks>
    public static void EnableContactEvents(ShapeID shapeId, bool flag)
    {
        World world = World.GetWorldLocked(shapeId.world0); if (world == null) return;
        world.GetShape(shapeId).enableContactEvents = flag;
    }

    ///<summary> Returns true if contact events are enabled</summary>
    public static bool AreContactEventsEnabled(ShapeID shapeId) => shapeId.world0.GetShape(shapeId).enableContactEvents;

    ///<summary>Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
    /// and must be carefully handled due to multithreading. Ignored for sensors.
    /// @see PreSolveFcn</summary>
    public static void EnablePreSolveEvents(ShapeID shapeId, bool flag)
    {
        World world = World.GetWorldLocked(shapeId.world0); if (world == null) return;
        world.GetShape(shapeId).enablePreSolveEvents = flag;
    }

    ///<summary> Returns true if pre-solve events are enabled</summary>
    public static bool ArePreSolveEventsEnabled(ShapeID shapeId) => shapeId.world0.GetShape(shapeId).enablePreSolveEvents;

    ///<summary>Enable contact hit events for this shape. Ignored for sensors.
    /// @see WorldDef.hitEventThreshold</summary>
    public static void EnableHitEvents(ShapeID shapeId, bool flag)
    {
        World world = World.GetWorldLocked(shapeId.world0); if (world == null) return;
        world.GetShape(shapeId).enableHitEvents = flag;
    }

    ///<summary> Returns true if hit events are enabled</summary>
    public static bool AreHitEventsEnabled(ShapeID shapeId) => shapeId.world0.GetShape(shapeId).enableHitEvents;

    ///<summary> Test a point for overlap with a shape</summary>
    public static bool TestPoint(ShapeID shapeId, Vector2 point)
    {
        World world = shapeId.world0;
        Shape shape = world.GetShape(shapeId);
        Transform transform = world.GetBodyTransform(shape.bodyId);
        Vector2 localPoint = transform.InvTransformPoint(point);
        return shape.shape.TestPoint(localPoint);
    }

    ///<summary> Ray cast a shape directly</summary>
    public static CastOutput RayCast(ShapeID shapeId, ref RayCastInput input)
    {
        World world = shapeId.world0;
        Shape shape = world.GetShape(shapeId);
        Transform transform = world.GetBodyTransform(shape.bodyId);
        RayCastInput localInput = new()
        {
            origin = transform.InvTransformPoint(input.origin),
            translation = transform.q.InvRotateVector(input.translation),
            maxFraction = input.maxFraction
        };
        CastOutput output = shape.shape.RayCast(ref localInput);
        if (output.hit)
        {
            output.point = transform.TransformPoint(output.point);
            output.normal = transform.q * output.normal;
        }
        return output;
    }

    ///<summary> Get a copy of the shape's circle. Asserts the type is correct.</summary>
    public static Circle GetCircle(ShapeID shapeId)
    {
        Shape shape = shapeId.world0.GetShape(shapeId);
        Debug.Assert(shape.type == ShapeType.Circle);
        return (Circle)shape.shape;
    }

    ///<summary> Get a copy of the shape's line segment. Asserts the type is correct.</summary>
    public static Segment GetSegment(ShapeID shapeId)
    {
        Shape shape = shapeId.world0.GetShape(shapeId);
        Debug.Assert(shape.type == ShapeType.Segment);
        return (Segment)shape.shape;
    }

    ///<summary>Get a copy of the shape's chain segment. These come from chain shapes.
    /// Asserts the type is correct.</summary>
    public static ChainSegment GetChainSegment(ShapeID shapeId)
    {
        Shape shape = shapeId.world0.GetShape(shapeId);
        Debug.Assert(shape.type == ShapeType.ChainSegment);
        return (ChainSegment)shape.shape;
    }

    ///<summary> Get a copy of the shape's capsule. Asserts the type is correct.</summary>
    public static Capsule GetCapsule(ShapeID shapeId)
    {
        Shape shape = shapeId.world0.GetShape(shapeId);
        Debug.Assert(shape.type == ShapeType.Capsule);
        return (Capsule)shape.shape;
    }

    ///<summary> Get a copy of the shape's convex polygon. Asserts the type is correct.</summary>
    public static Polygon GetPolygon(ShapeID shapeId)
    {
        Shape shape = shapeId.world0.GetShape(shapeId);
        Debug.Assert(shape.type == ShapeType.Polygon);
        return (Polygon)shape.shape;
    }

    ///<summary>Allows you to change a shape to be a circle or update the current circle.
    /// This does not modify the mass properties.
    /// @see Body_ApplyMassFromShapes</summary>
    public static void SetCircle(ShapeID shapeId, ref Circle circle)
    {
        World world = World.GetWorldLocked(shapeId.world0); if (world == null) return;
        Shape shape = world.GetShape(shapeId);
        shape.shape = circle; shape.type = ShapeType.Circle;
        world.ResetProxy(shape, true, true);
    }

    ///<summary>Allows you to change a shape to be a capsule or update the current capsule.
    /// This does not modify the mass properties.
    /// @see Body_ApplyMassFromShapes</summary>
    public static void SetCapsule(ShapeID shapeId, ref Capsule capsule)
    {
        World world = World.GetWorldLocked(shapeId.world0); if (world == null) return;
        Shape shape = world.GetShape(shapeId);
        shape.shape = capsule; shape.type = ShapeType.Capsule;
        world.ResetProxy(shape, true, true);
    }

    ///<summary> Allows you to change a shape to be a segment or update the current segment.</summary>
    public static void SetSegment(ShapeID shapeId, ref Segment segment)
    {
        World world = World.GetWorldLocked(shapeId.world0); if (world == null) return;
        Shape shape = world.GetShape(shapeId);
        shape.shape = segment; shape.type = ShapeType.Segment;
        world.ResetProxy(shape, true, true);
    }

    ///<summary>Allows you to change a shape to be a polygon or update the current polygon.
    /// This does not modify the mass properties.
    /// @see Body_ApplyMassFromShapes</summary>
    public static void SetPolygon(ShapeID shapeId, ref Polygon polygon)
    {
        World world = World.GetWorldLocked(shapeId.world0); if (world == null) return;
        Shape shape = world.GetShape(shapeId);
        shape.shape = polygon; shape.type = ShapeType.Polygon;
        world.ResetProxy(shape, true, true);
    }

    ///<summary>Get the parent chain id if the shape type is a chain segment, otherwise
    /// returns _nullChainId.</summary>
    public static ChainID GetParentChain(ShapeID shapeId)
    {
        World world = shapeId.world0;
        Shape shape = world.GetShape(shapeId);
        if (shape.type == ShapeType.ChainSegment)
        {
            int chainId = ((ChainSegment)shape.shape).chainId;
            if (chainId != -1)
            {
                ChainShape chain = world.chainShapes[chainId];
                return new() { index1 = chainId + 1, world0 = shapeId.world0, generation = chain.generation };
            }
        }
        return new();
    }

    ///<summary> Get the maximum capacity required for retrieving all the touching contacts on a shape</summary>
    public static int GetContactCapacity(ShapeID shapeId)
    {
        World world = World.GetWorldLocked(shapeId.world0); if (world == null) return 0;
        Shape shape = world.GetShape(shapeId);
        if (shape.sensorIndex != -1) return 0;
        return world.bodies[shape.bodyId].contactCount;
    }

    /// <summary>Get the touching contact data for a shape. The provided shapeID will be either shapeIdA or shapeIdB on the contact data.</summary>
    /// <remarks>Box2D uses speculative collision so some contact points may be separated.</remarks>
    /// <returns> the number of elements filled in the provided array<br/>
    /// do not ignore the return value, it specifies the valid number of elements</returns>
    public static int GetContactData(ShapeID shapeId, ContactData[] contactData)
    {
        World world = World.GetWorldLocked(shapeId.world0); if (world == null) return 0;
        Shape shape = world.GetShape(shapeId);
        if (shape.sensorIndex != -1) return 0;
        Body body = world.bodies[shape.bodyId];
        int contactKey = body.headContactKey;
        int index = 0;
        while (contactKey != -1 && index < contactData.Length)
        {
            int contactId = contactKey >> 1;
            int edgeIndex = contactKey & 1;
            Contact contact = world.contacts[contactId];
            if ((contact.shapeIdA == shapeId.index1 - 1 || contact.shapeIdB == shapeId.index1 - 1) &&
                contact.flags.HasFlag(ContactFlags.Touching))
            {
                Shape shapeA = world.shapes[contact.shapeIdA], shapeB = world.shapes[contact.shapeIdB];
                contactData[index] = new()
                {
                    contactId = new() { index1 = contact.contactId + 1, world0 = shapeId.world0, generation = contact.generation },
                    shapeIdA = new() { index1 = shapeA.id + 1, world0 = shapeId.world0, generation = shapeA.generation },
                    shapeIdB = new() { index1 = shapeB.id + 1, world0 = shapeId.world0, generation = shapeB.generation },
                    manifold = world.GetContactSim(contact).manifold
                };
                index++;
            }
            contactKey = edgeIndex == 1 ? contact.edge1.nextKey : contact.edge0.nextKey;
        }
        Debug.Assert(index <= contactData.Length);
        return index;
    }

    /// <summary>Get the maximum capacity required for retrieving all the overlapped shapes on a sensor shape.
    /// This returns 0 if the provided shape is not a sensor.</summary>
    /// <param name="shapeID">the id of a sensor shape</param>
    /// <returns>the required capacity to get all the overlaps in GetSensorOverlaps</returns>
    public static int GetSensorCapacity(ShapeID shapeId)
    {
        World world = World.GetWorldLocked(shapeId.world0); if (world == null) return 0;
        Shape shape = world.GetShape(shapeId);
        if (shape.sensorIndex == -1) return 0;
        return world.sensors[shape.bodyId].overlaps2.Count;
    }

    /// <summary>Get the overlap data for a sensor shape.</summary>
    /// <param name="shapeID">the id of a sensor shape</param>
    /// <param name="visitorIds">a user allocated array that is filled with the overlapping shapes (visitors)</param>
    /// <returns> the number of elements filled in the provided array<br/>
    /// do not ignore the return value, it specifies the valid number of elements<br/>
    /// overlaps may contain destroyed shapes so use IsValid to confirm each overlap</returns>
    public static int GetSensorData(ShapeID shapeId, ShapeID[] visitorIds)
    {
        World world = World.GetWorldLocked(shapeId.world0); if (world == null) return 0;
        Shape shape = world.GetShape(shapeId);
        if (shape.sensorIndex == -1) return 0;
        Sensor sensor = world.sensors[shape.sensorIndex];
        int count = Math.Min(sensor.overlaps2.Count, visitorIds.Length);
        for (int i = 0; i < count; i++)
            visitorIds[i] = new() { index1 = sensor.overlaps2[i].shapeId, world0 = shapeId.world0, generation = sensor.overlaps2[i].generation };
        return count;
    }

    ///<summary> Get the current world AABB</summary>
    public static AABB GetAABB(ShapeID shapeId)
    {
        World world = World.GetWorldLocked(shapeId.world0); if (world == null) return new();
        return world.GetShape(shapeId).aabb;
    }

    ///<summary> Compute the mass data for a shape</summary>
    public static MassData ComputeMassData(ShapeID shapeId)
    {
        World world = World.GetWorldLocked(shapeId.world0); if (world == null) return new();
        return world.GetShape(shapeId).ComputeMass();
    }

    ///<summary>Get the closest point on a shape to a target point. Target and result are in world space.
    /// todo need sample</summary>
    public static Vector2 GetClosestPoint(ShapeID shapeId, Vector2 target)
    {
        World world = World.GetWorldLocked(shapeId.world0); if (world == null) return new();
        Shape shape = world.GetShape(shapeId);
        Body body = world.bodies[shape.bodyId];
        Transform transform = world.GetBodyTransformQuick(body);
        DistanceInput input = new()
        {
            proxyA = shape.MakeDistanceProxy(),
            proxyB = Distance.MakeProxy([target], 0),
            transformA = transform,
            transformB = Transform.Identity,
            useRadii = true
        };
        SimplexCache cache = new();
        return input.ShapeDistance(ref cache, null).pointA;
    }

    public static unsafe void ApplyWindForce(ShapeID shapeId, Vector2 wind, float drag, float lift, float airDensity = 1.225f, bool wake = true)
    {
        World world = World.GetWorldLocked(shapeId.world0); if (world == null) return;
        Shape shape = world.GetShape(shapeId);
        ShapeType shapeType = shape.type;
        if (shapeType != ShapeType.Circle && shapeType != ShapeType.Segment && shapeType != ShapeType.Polygon) return;
        Body body = world.bodies[shape.bodyId];
        if (body.type != BodyType.Dynamic) return;
        if (body.setIndex >= (int)SetType.FirstSleeping && !wake) return;
        BodySim sim = world.GetBodySim(body);
        if (body.setIndex != (int)SetType.Awake) world.WakeBody(body);
        BodyState* state = world.GetBodyState(body);
        Transform transform = sim.transform;
        float volumeUnits = Box2D.LengthUnitsPerMeter * Box2D.LengthUnitsPerMeter * Box2D.LengthUnitsPerMeter;
        airDensity /= volumeUnits;
        Vector2 lever = transform.q * (shape.localCentroid - sim.localCenter);
        Vector2 shapeVelocity = state->linearVelocity + Vector2.CrossSV(state->angularVelocity, lever);
        shape.shape.ApplyWindForce(airDensity, wind, drag, lift, ref transform, sim, lever, shapeVelocity, out Vector2 force, out float torque);
        sim.force += force;
        sim.torque += torque;
    }
}
