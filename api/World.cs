using System;
using System.Diagnostics;
using System.Collections.Generic;

namespace Box2D.API;

public static class WorldAPI
{
    /// <summary>Create a world for rigid body simulation. A world contains bodies, shapes, and constraints.
    /// Each world is completely independent and may be simulated in parallel.</summary>
    /// <returns>the world id.</returns>
    public static WorldID CreateWorld(ref WorldDef def)
    {
        World world = new(ref def); return new() { index1 = world, generation = world.generation };
    }

    ///<summary> Destroy a world</summary>
    public static void DestroyWorld(WorldID worldId) => worldId.index1.Destroy();

    ///<summary> World id validation. Provides validation for up to 64K allocations.</summary>
    public static bool World_IsValid(WorldID id)
    {
        if (id.index1 == null) return false;
        return id.generation == id.index1.generation;
    }

    /// <summary>Simulate a world for one time step. This performs collision detection, integration, and constraint solution.</summary>
    /// <param name="worldID">The world to simulate</param>
    /// <param name="timeStep">The amount of time to simulate, this should be a fixed number. Usually 1/60.</param>
    /// <param name="subStepCount">The number of sub-steps, increasing the sub-step count can increase accuracy. Usually 4.</param>
    public static void World_Step(WorldID worldId, float timeStep, int subStepCount)
    {
        Debug.Assert(float.IsFinite(timeStep));
        Debug.Assert(0 < subStepCount);
        World world = worldId.index1;
        Debug.Assert(!world.locked);
        if (world.locked) return;
        world.bodyMoveEvents.Clear();
        world.sensorBeginEvents.Clear();
        world.contactBeginEvents.Clear();
        world.contactHitEvents.Clear();
        world.jointEvents.Clear();
        world.profile = new();
        if (timeStep == 0)
        {
            world.endEventArrayIndex = 1 - world.endEventArrayIndex;
            if (world.endEventArrayIndex == 1)
            { world.sensorEndEvents1.Clear(); world.contactEndEvents1.Clear(); }
            else { world.sensorEndEvents0.Clear(); world.contactEndEvents0.Clear(); }
            return;
        }
        world.locked = true;
        world.activeTaskCount = 0;
        Stopwatch stepTicks = new(), pairTicks = new();
        stepTicks.Start();
        world.taskCount = 0;
        {
            pairTicks.Start();
            world.UpdateBroadPhasePairs();
            pairTicks.Stop();
            world.profile.pairs = (float)pairTicks.Elapsed.TotalMilliseconds;
        }
        StepContext context = new() { world = world, dt = timeStep, subStepCount = Math.Max(1, subStepCount) };
        if (timeStep > 0)
        {
            context.inv_dt = 1 / timeStep;
            context.h = timeStep / context.subStepCount;
            context.inv_h = context.subStepCount * context.inv_dt;
        }
        else
        {
            context.inv_dt = 0;
            context.h = 0;
            context.inv_h = 0;
        }
        world.inv_h = context.inv_h;
        float contactHertz = Math.Min(world.contactHertz, 0.125f * context.inv_h);
        context.contactSoftness = new(contactHertz, world.contactDampingRatio, context.h);
        context.staticSoftness = new(2 * contactHertz, world.contactDampingRatio, context.h);
        context.restitutionThreshold = world.restitutionThreshold;
        context.maxLinearVelocity = world.maxLinearSpeed;
        context.enableWarmStarting = world.enableWarmStarting;
        {
            pairTicks.Restart();
            World.Collide(context);
            pairTicks.Stop();
        }
        if (context.dt > 0)
        {
            pairTicks.Restart();
            for (int i = 0; i < world.particleSystemList.Count; i++) world.particleSystemList[i].Solve(context.dt, context.inv_dt, 4);
            world.Solve(context);
            pairTicks.Stop();
            world.profile.collide = (float)pairTicks.Elapsed.TotalMilliseconds;
        }
        {
            pairTicks.Restart();
            world.OverlapSensors();
            pairTicks.Stop();
            world.profile.sensors = (float)pairTicks.Elapsed.TotalMilliseconds;
        }
        stepTicks.Stop();
        world.profile.step = (float)stepTicks.Elapsed.TotalMilliseconds;
        Debug.Assert(world.arena.GetArenaAllocation() == 0);
        world.arena.GrowArena();
        Debug.Assert(world.activeTaskCount == 0);
        world.endEventArrayIndex = 1 - world.endEventArrayIndex;
        if (world.endEventArrayIndex == 1)
        { world.sensorEndEvents1.Clear(); world.contactEndEvents1.Clear(); }
        else { world.sensorEndEvents0.Clear(); world.contactEndEvents0.Clear(); }
        world.locked = false;
    }

    ///<summary> Call this to draw shapes and other debug draw data</summary>
    public static void World_Draw(WorldID worldId, DebugDraw draw)
    {
        World world = worldId.index1; //Debug.Assert(!world.locked); if (world.locked) return;
        Debug.Assert(draw.drawingBounds.IsValid());
        const float k_impulseScale = 1, k_axisScale = 0.3f;
        HexColor speculativeColor = HexColor.Gainsboro;
        HexColor addColor = HexColor.Green;
        HexColor persistColor = HexColor.Blue;
        HexColor normalColor = HexColor.DimGray;
        HexColor impulseColor = HexColor.Magenta;
        HexColor frictionColor = HexColor.Yellow;
        HexColor[] graphColors =
        [
            HexColor.Red, HexColor.Orange, HexColor.Yellow, HexColor.Green, HexColor.Cyan, HexColor.Blue,
            HexColor.Violet, HexColor.Pink, HexColor.Chocolate, HexColor.GoldenRod, HexColor.Coral, HexColor.RosyBrown,
            HexColor.Aqua, HexColor.Peru, HexColor.Lime, HexColor.Gold, HexColor.Plum, HexColor.Snow,
            HexColor.Teal, HexColor.Khaki, HexColor.Salmon, HexColor.PeachPuff, HexColor.HoneyDew, HexColor.Black
        ];
        world.debugBodySet.SetBitCountAndClear(world.bodyIdPool.GetIdCapacity());
        world.debugJointSet.SetBitCountAndClear(world.jointIdPool.GetIdCapacity());
        world.debugContactSet.SetBitCountAndClear(world.contactIdPool.GetIdCapacity());
        world.debugIslandSet.SetBitCountAndClear(world.islandIdPool.GetIdCapacity());
        DebugDraw.DrawContext drawContext = new() { world = world, draw = draw };
        for (int i = 0; i < 3; i++)
        {
            world.broadPhase.trees[i].QueryAll(draw.drawingBounds, DebugDraw.DrawQueryCallback, drawContext);
        }
        uint wordCount = world.debugBodySet.blockCount;
        ulong[] bits = world.debugBodySet.bits;
        for (uint k = 0; k < wordCount; k++)
        {
            ulong word = bits[k];
            while (word != 0)
            {
                uint ctz = CTZ.CTZ64(word);
                int bodyId = (int)(64 * k + ctz);
                Body body = world.bodies[bodyId];
                if (draw.drawBodyNames && body.name != null)
                {
                    Vector2 offset = new(0.1f, 0.1f);
                    BodySim bodySim = world.GetBodySim(body);
                    Transform transform = new(bodySim.center, bodySim.transform.q);
                    Vector2 p = transform.TransformPoint(offset);
                    draw.DrawStringFcn(p, body.name, HexColor.BlueViolet, draw.context);
                }
                if (draw.drawMass && body.type == BodyType.Dynamic)
                {
                    Vector2 offset = new(0.1f, 0.1f);
                    BodySim bodySim = world.GetBodySim(body);
                    Transform transform = new(bodySim.center, bodySim.transform.q);
                    draw.DrawSegmentFcn(bodySim.center0, bodySim.center, HexColor.WhiteSmoke, draw.context);
                    draw.DrawTransformFcn(transform, draw.context);
                    Vector2 p = transform.TransformPoint(offset);
                    draw.DrawStringFcn(p, $"  {body.mass:F2}", HexColor.White, draw.context);
                }
                if (draw.drawJoints)
                {
                    int jointKey = body.headJointKey;
                    while (jointKey != -1)
                    {
                        int jointId = jointKey >> 1;
                        int edgeIndex = jointKey & 1;
                        Joint joint = world.joints[jointId];
                        if (!world.debugJointSet.GetBit(jointId))
                        {
                            draw.DrawJoint(world, joint);
                            world.debugJointSet.SetBit(jointId);
                        }
                        jointKey = edgeIndex == 1 ? joint.edge1.nextKey : joint.edge0.nextKey;
                    }
                }
                if (draw.drawContacts && body.type == BodyType.Dynamic)
                {
                    int contactKey = body.headContactKey;
                    while (contactKey != -1)
                    {
                        int contactId = contactKey >> 1;
                        int edgeIndex = contactKey & 1;
                        Contact contact = world.contacts[contactId];
                        contactKey = edgeIndex == 1 ? contact.edge1.nextKey : contact.edge0.nextKey;
                        if (!world.debugContactSet.GetBit(contactId))
                        {
                            ContactSim contactSim = world.GetContactSim(contact);
                            int pointCount = contactSim.manifold.pointCount;
                            Vector2 normal = contactSim.manifold.normal;
                            for (int j = 0; j < pointCount; j++)
                            {
                                ref ManifoldPoint point = ref contactSim.manifold.point0;
                                if (j == 1) point = ref contactSim.manifold.point1;
                                if (draw.drawGraphColors && contact.colorIndex != -1)
                                {
                                    float pointSize = contact.colorIndex == Box2D.GraphColorCount - 1 ? 7.5f : 5;
                                    draw.DrawPointFcn(point.point, pointSize, graphColors[contact.colorIndex], draw.context);
                                }
                                else if (point.separation > Box2D.LinearSlop)
                                    draw.DrawPointFcn(point.point, 5, speculativeColor, draw.context);
                                else if (!point.persisted) draw.DrawPointFcn(point.point, 10, addColor, draw.context);
                                else if (point.persisted) draw.DrawPointFcn(point.point, 5, persistColor, draw.context);
                                if (draw.drawContactNormals)
                                {
                                    Vector2 p1 = point.point;
                                    Vector2 p2 = Vector2.MulAdd(p1, k_axisScale, normal);
                                    draw.DrawSegmentFcn(p1, p2, normalColor, draw.context);
                                }
                                else if (draw.drawContactImpulses)
                                {
                                    Vector2 p1 = point.point;
                                    Vector2 p2 = Vector2.MulAdd(p1, k_impulseScale * point.totalNormalImpulse, normal);
                                    draw.DrawSegmentFcn(p1, p2, impulseColor, draw.context);
                                    draw.DrawStringFcn(p1, $"{1000 * point.totalNormalImpulse:F1}", HexColor.White, draw.context);
                                }
                                if (draw.drawContactFeatures)
                                {
                                    draw.DrawStringFcn(point.point, $"{point.id}", HexColor.Orange, draw.context);
                                }
                                if (draw.drawFrictionImpulses)
                                {
                                    Vector2 tangent = normal.RightPerp();
                                    Vector2 p1 = point.point;
                                    Vector2 p2 = Vector2.MulAdd(p1, k_impulseScale * point.tangentImpulse, tangent);
                                    draw.DrawSegmentFcn(p1, p2, frictionColor, draw.context);
                                    draw.DrawStringFcn(p1, $"{1000 * point.tangentImpulse:F1}", HexColor.White, draw.context);
                                }
                            }
                            world.debugContactSet.SetBit(contactId);
                        }
                        contactKey = edgeIndex == 1 ? contact.edge1.nextKey : contact.edge0.nextKey;
                    }
                }
                if (draw.drawIslands)
                {
                    int islandId = body.islandId;
                    if (islandId != -1 && !world.debugIslandSet.GetBit(islandId))
                    {
                        Island island = world.islands[islandId];
                        if (island.setIndex == -1) continue;
                        int shapeCount = 0;
                        AABB aabb = new(new(float.MaxValue, float.MaxValue), new(-float.MaxValue, -float.MaxValue));
                        int islandBodyId = island.headBody;
                        while (islandBodyId != -1)
                        {
                            Body islandBody = world.bodies[islandBodyId];
                            int shapeId = islandBody.headShapeId;
                            while (shapeId != -1)
                            {
                                Shape shape = world.shapes[shapeId];
                                aabb = AABB.Union(aabb, shape.fatAABB);
                                shapeCount++;
                                shapeId = shape.nextShapeId;
                            }
                            islandBodyId = islandBody.islandNext;
                        }
                        if (shapeCount > 0)
                        {
                            draw.DrawPolygonFcn([aabb.lowerBound, new(aabb.upperBound.x, aabb.lowerBound.y), aabb.upperBound, new(aabb.lowerBound.x, aabb.upperBound.y)], HexColor.OrangeRed, draw.context);
                        }
                        world.debugIslandSet.SetBit(islandId);
                    }
                }
                word &= word - 1;
            }
        }
    }

    ///<summary> Get the body events for the current time step. The event data is transient. Do not store a reference to this data.</summary>
    public static BodyEvents World_GetBodyEvents(WorldID worldId)
    {
        World world = worldId.index1; Debug.Assert(!world.locked); if (world.locked) return new();
        return new() { moveEvents = world.bodyMoveEvents };
    }

    ///<summary> Get sensor events for the current time step. The event data is transient. Do not store a reference to this data.</summary>
    public static SensorEvents World_GetSensorEvents(WorldID worldId)
    {
        World world = worldId.index1; Debug.Assert(!world.locked); if (world.locked) return new();
        int endEventArrayIndex = 1 - world.endEventArrayIndex;
        return new()
        {
            beginEvents = world.sensorBeginEvents,
            endEvents = endEventArrayIndex == 1 ? world.sensorEndEvents1 : world.sensorEndEvents0
        };
    }

    ///<summary> Get contact events for this current time step. The event data is transient. Do not store a reference to this data.</summary>
    public static ContactEvents World_GetContactEvents(WorldID worldId)
    {
        World world = worldId.index1; Debug.Assert(!world.locked); if (world.locked) return new();
        int endEventArrayIndex = 1 - world.endEventArrayIndex;
        return new()
        {
            beginEvents = world.contactBeginEvents,
            endEvents = endEventArrayIndex == 1 ? world.contactEndEvents1 : world.contactEndEvents0
        };
    }

    ///<summary> Get the joint events for the current time step. The event data is transient. Do not store a reference to this data.</summary>
    public static JointEvents World_GetJointEvents(WorldID worldId)
    {
        World world = worldId.index1; Debug.Assert(!world.locked); if (world.locked) return new();
        return new() { jointEvents = world.jointEvents };
    }

    public class WorldQueryContext
    {
        public World world;
        public OverlapResultFcn fcn;
        public QueryFilter filter;
        public object userContext;
    }
    public static bool TreeQueryCallback(int proxyId, ulong userData, object context)
    {
        int shapeId = (int)userData;
        WorldQueryContext worldContext = (WorldQueryContext)context;
        World world = worldContext.world;
        Shape shape = world.shapes[shapeId];
        if (!Shape.ShouldQueryCollide(shape.filter, worldContext.filter)) return true;
        ShapeID id = new() { index1 = shapeId + 1, world0 = world, generation = shape.generation };
        return worldContext.fcn(id, worldContext.userContext);
    }
    ///<summary> Overlap test for all shapes that ref *potentially overlap the provided AABB</summary>
    public static TreeStats World_OverlapAABB(WorldID worldId, AABB aabb, QueryFilter filter, OverlapResultFcn fcn, object context, ParticleQueryCallback callback = null)
    {
        TreeStats treeStats = new();
        World world = worldId.index1; //Debug.Assert(!world.locked); if (world.locked) return treeStats;
        Debug.Assert(aabb.IsValid());
        WorldQueryContext worldContext = new() { world = world, fcn = fcn, filter = filter, userContext = context };
        for (int i = 0; i < 3; i++)
        {
            TreeStats treeResult = world.broadPhase.trees[i].Query(aabb, filter.maskBits, TreeQueryCallback, worldContext);
            treeStats.nodeVisits += treeResult.nodeVisits;
            treeStats.leafVisits += treeResult.leafVisits;
        }
        if (callback != null) for (int i = 0; i < world.particleSystemList.Count; i++)
                if (callback.ShouldQueryParticleSystem(world.particleSystemList[i]))
                    world.particleSystemList[i].QueryAABB(callback, aabb);
        return treeStats;
    }

    public class WorldOverlapContext
    {
        public World world;
        public OverlapResultFcn fcn;
        public QueryFilter filter;
        public ShapeProxy proxy;
        public object userContext;
    }
    public static bool TreeOverlapCallback(int proxyId, ulong userData, object context)
    {
        int shapeId = (int)userData;
        WorldOverlapContext worldContext = (WorldOverlapContext)context;
        World world = worldContext.world;
        Shape shape = world.shapes[shapeId];
        if (!Shape.ShouldQueryCollide(shape.filter, worldContext.filter)) return true;
        Body body = world.bodies[shape.bodyId];
        Transform transform = world.GetBodyTransformQuick(body);
        DistanceInput input = new()
        {
            proxyA = worldContext.proxy,
            proxyB = shape.MakeDistanceProxy(),
            transformA = Transform.Identity,
            transformB = transform,
            useRadii = true
        };
        SimplexCache cache = new();
        DistanceOutput output = Distance.ShapeDistance(ref input, ref cache, null);
        float tolerance = 0.1f * Box2D.LinearSlop;
        if (output.distance > tolerance) return true;
        ShapeID id = new() { index1 = shape.id + 1, world0 = world, generation = shape.generation };
        return worldContext.fcn(id, worldContext.userContext);
    }
    ///<summary> Overlap test for all shapes that overlap the provided shape proxy.</summary>
    public static TreeStats World_OverlapShape(WorldID worldId, ShapeProxy proxy, QueryFilter filter, OverlapResultFcn fcn, object context)
    {
        TreeStats treeStats = new();
        World world = worldId.index1; Debug.Assert(!world.locked); if (world.locked) return treeStats;
        AABB aabb = AABB.MakeAABB(proxy.points, proxy.radius);
        WorldOverlapContext worldContext = new() { world = world, fcn = fcn, filter = filter, proxy = proxy, userContext = context };
        for (int i = 0; i < 3; i++)
        {
            TreeStats treeResult = world.broadPhase.trees[i].Query(aabb, filter.maskBits, TreeOverlapCallback, worldContext);
            treeStats.nodeVisits += treeResult.nodeVisits;
            treeStats.leafVisits += treeResult.leafVisits;
        }
        return treeStats;
    }

    public class WorldRayCastContext
    {
        public World world;
        public CastResultFcn fcn;
        public QueryFilter filter;
        public float fraction;
        public object userContext;
    }
    public static float RayCastCallback(ref RayCastInput input, int proxyId, ulong userData, object context)
    {
        int shapeId = (int)userData;
        WorldRayCastContext worldContext = (WorldRayCastContext)context;
        World world = worldContext.world;
        Shape shape = world.shapes[shapeId];
        if (!Shape.ShouldQueryCollide(shape.filter, worldContext.filter)) return input.maxFraction;
        Body body = world.bodies[shape.bodyId];
        Transform transform = world.GetBodyTransformQuick(body);
        CastOutput output = shape.RayCast(ref input, transform);
        if (output.hit)
        {
            ShapeID id = new() { index1 = shapeId + 1, world0 = world, generation = shape.generation };
            float fraction = worldContext.fcn(id, output.point, output.normal, output.fraction, worldContext.userContext);
            if (0 <= fraction && fraction <= 1) worldContext.fraction = fraction;
            return fraction;
        }
        return input.maxFraction;
    }
    /// <summary>Cast a ray into the world to collect shapes in the path of the ray.
    /// Your callback function controls whether you get the closest point, any point, or n-points.</summary>
    /// <remarks>The callback function may receive shapes in any order</remarks>
    /// <param name="worldID">The world to cast the ray against</param>
    /// <param name="origin">The start point of the ray</param>
    /// <param name="translation">The translation of the ray from the start point to the end point</param>
    /// <param name="filter">Contains bit flags to filter unwanted shapes from the results</param>
    /// <param name="fcn">A user implemented callback function</param>
    /// <param name="context">A user context that is passed along to the callback function</param>
    ///	<returns>traversal performance counters</returns>
    public static TreeStats World_CastRay(WorldID worldId, Vector2 origin, Vector2 translation, QueryFilter filter,
                                        ref CastResultFcn fcn, object context, ParticleRayCastCallback callback = null)
    {
        TreeStats treeStats = new();
        World world = worldId.index1; Debug.Assert(!world.locked); if (world.locked) return treeStats;
        Debug.Assert(origin.IsValid());
        Debug.Assert(translation.IsValid());
        RayCastInput input = new() { origin = origin, translation = translation, maxFraction = 1 };
        WorldRayCastContext worldContext = new() { world = world, fcn = fcn, filter = filter, fraction = 1, userContext = context };
        for (int i = 0; i < 3; i++)
        {
            TreeStats treeResult = world.broadPhase.trees[i].RayCast(ref input, filter.maskBits, RayCastCallback, worldContext);
            treeStats.nodeVisits += treeResult.nodeVisits;
            treeStats.leafVisits += treeResult.leafVisits;
            if (worldContext.fraction == 0) return treeStats;
            input.maxFraction = worldContext.fraction;
        }
        if (callback != null) for (int i = 0; i < world.particleSystemList.Count; i++)
                if (callback.ShouldQueryParticleSystem(world.particleSystemList[i]))
                    world.particleSystemList[i].RayCast(callback, origin, translation);
        return treeStats;
    }

    /// <summary>This callback finds the closest hit. This is the most common callback used in games.</summary>
    public static float RayCastClosestFcn(ShapeID shapeId, Vector2 point, Vector2 normal, float fraction, object context)
    {
        if (fraction == 0) return -1;
        RayResult rayResult = (RayResult)context;
        rayResult.shapeID = shapeId;
        rayResult.point = point;
        rayResult.normal = normal;
        rayResult.fraction = fraction;
        rayResult.hit = true;
        return fraction;
    }
    ///<summary>Cast a ray into the world to collect the closest hit. This is a convenience function. Ignores initial overlap.
    /// This is less general than World_CastRay() and does not allow for custom filtering.</summary>
    public static RayResult World_CastRayClosest(WorldID worldId, Vector2 origin, Vector2 translation, QueryFilter filter)
    {
        RayResult result = new();
        World world = worldId.index1; Debug.Assert(!world.locked); if (world.locked) return result;
        Debug.Assert(origin.IsValid());
        Debug.Assert(translation.IsValid());
        RayCastInput input = new() { origin = origin, translation = translation, maxFraction = 1 };
        WorldRayCastContext worldContext = new() { world = world, fcn = RayCastClosestFcn, filter = filter, fraction = 1, userContext = result };
        for (int i = 0; i < 3; i++)
        {
            TreeStats treeResult = world.broadPhase.trees[i].RayCast(ref input, filter.maskBits, RayCastCallback, worldContext);
            result.nodeVisits += treeResult.nodeVisits;
            result.leafVisits += treeResult.leafVisits;
            if (worldContext.fraction == 0) return result;
            input.maxFraction = worldContext.fraction;
        }
        return result;
    }

    public static float ShapeCastCallback(ref ShapeCastInput input, int proxyId, ulong userData, object context)
    {
        int shapeId = (int)userData;
        WorldRayCastContext worldContext = (WorldRayCastContext)context;
        World world = worldContext.world;
        Shape shape = world.shapes[shapeId];
        if (!Shape.ShouldQueryCollide(shape.filter, worldContext.filter)) return input.maxFraction;
        Body body = world.bodies[shape.bodyId];
        Transform transform = world.GetBodyTransformQuick(body);
        CastOutput output = shape.ShapeCast(ref input, transform);
        if (output.hit)
        {
            ShapeID id = new() { index1 = shapeId + 1, world0 = world, generation = shape.generation };
            float fraction = worldContext.fcn(id, output.point, output.normal, output.fraction, worldContext.userContext);
            if (0 <= fraction && fraction <= 1) worldContext.fraction = fraction;
            return fraction;
        }
        return input.maxFraction;
    }
    ///<summary>Cast a shape through the world. Similar to a cast ray except that a shape is cast instead of a point.
    ///	@see World_CastRay</summary>
    public static TreeStats World_CastShape(WorldID worldId, ref ShapeProxy proxy, Vector2 translation, QueryFilter filter,
                                          ref CastResultFcn fcn, object context)
    {
        TreeStats treeStats = new();
        World world = worldId.index1; Debug.Assert(!world.locked); if (world.locked) return treeStats;
        Debug.Assert(translation.IsValid());
        ShapeCastInput input = new() { proxy = proxy, translation = translation, maxFraction = 1 };
        WorldRayCastContext worldContext = new() { world = world, fcn = fcn, filter = filter, fraction = 1, userContext = context };
        for (int i = 0; i < 3; i++)
        {
            TreeStats treeResult = world.broadPhase.trees[i].ShapeCast(ref input, filter.maskBits, ShapeCastCallback, worldContext);
            treeStats.nodeVisits += treeResult.nodeVisits;
            treeStats.leafVisits += treeResult.leafVisits;
            if (worldContext.fraction == 0) return treeStats;
            input.maxFraction = worldContext.fraction;
        }
        return treeStats;
    }

    public class MoverContext
    {
        public World world;
        public QueryFilter filter;
        public ShapeProxy proxy;
        public Transform transform;
        public object userContext;
    }
    public class WorldMoverCastContext
    {
        public World world;
        public QueryFilter filter;
        public float fraction;
    }
    public static float MoverCastCallback(ref ShapeCastInput input, int proxyId, ulong userData, object context)
    {
        int shapeId = (int)userData;
        WorldMoverCastContext worldContext = (WorldMoverCastContext)context;
        World world = worldContext.world;
        Shape shape = world.shapes[shapeId];
        if (!Shape.ShouldQueryCollide(shape.filter, worldContext.filter)) return worldContext.fraction;
        Body body = world.bodies[shape.bodyId];
        Transform transform = world.GetBodyTransformQuick(body);
        CastOutput output = shape.ShapeCast(ref input, transform);
        if (output.fraction == 0) return worldContext.fraction;
        worldContext.fraction = output.fraction;
        return output.fraction;
    }
    ///<summary>Cast a capsule mover through the world. This is a special shape cast that handles sliding along other shapes while reducing
    /// clipping.</summary>
    public static float World_CastMover(WorldID worldId, ref Capsule mover, Vector2 translation, QueryFilter filter)
    {
        Debug.Assert(translation.IsValid());
        Debug.Assert(mover.radius > 2 * Box2D.LinearSlop);
        World world = worldId.index1; Debug.Assert(!world.locked); if (world.locked) return 1;
        ShapeCastInput input = new()
        {
            proxy = new() { points = [mover.center1, mover.center2], radius = mover.radius },
            translation = translation,
            maxFraction = 1,
            canEncroach = true
        };
        WorldMoverCastContext worldContext = new() { world = world, filter = filter, fraction = 1 };
        for (int i = 0; i < 3; i++)
        {
            world.broadPhase.trees[i].ShapeCast(ref input, filter.maskBits, MoverCastCallback, worldContext);
            if (worldContext.fraction == 0) return 0;
            input.maxFraction = worldContext.fraction;
        }
        return worldContext.fraction;
    }

    public class WorldMoverContext
    {
        public World world;
        public PlaneResultFcn fcn;
        public QueryFilter filter;
        public Capsule mover;
        public object userContext;
    }
    public static bool TreeCollideCallback(int proxyId, ulong userData, object context)
    {
        int shapeId = (int)userData;
        WorldMoverContext worldContext = (WorldMoverContext)context;
        World world = worldContext.world;
        Shape shape = world.shapes[shapeId];
        if (!Shape.ShouldQueryCollide(shape.filter, worldContext.filter)) return true;
        Body body = world.bodies[shape.bodyId];
        Transform transform = world.GetBodyTransformQuick(body);
        PlaneResult result = shape.CollideMover(ref worldContext.mover, transform);
        if (result.hit && result.plane.normal.IsNormalized())
        {
            ShapeID id = new() { index1 = shape.id + 1, world0 = world, generation = shape.generation };
            return worldContext.fcn(id, ref result, worldContext.userContext);
        }
        return true;
    }
    ///<summary>Collide a capsule mover with the world, gathering collision planes that can be fed to SolvePlanes. Useful for
    /// kinematic character movement.</summary>
    public static void World_CollideMover(WorldID worldId, ref Capsule mover, QueryFilter filter, PlaneResultFcn fcn,
                                  object context)
    {
        World world = worldId.index1; Debug.Assert(!world.locked); if (world.locked) return;
        Vector2 r = new(mover.radius, mover.radius);
        AABB aabb = new(Vector2.Min(mover.center1, mover.center2) - r, Vector2.Max(mover.center1, mover.center2) + r);
        WorldMoverContext worldContext = new() { world = world, fcn = fcn, filter = filter, mover = mover, userContext = context };
        for (int i = 0; i < 3; i++)
            world.broadPhase.trees[i].Query(aabb, filter.maskBits, TreeCollideCallback, worldContext);
    }

    ///<summary>Enable/disable sleep. If your application does not need sleeping, you can gain some performance
    /// by disabling sleep completely at the world level.
    /// @see WorldDef</summary>
    public static void World_EnableSleeping(WorldID worldId, bool flag)
    {
        World world = worldId.index1; Debug.Assert(!world.locked); if (world.locked) return;
        if (flag == world.enableSleep) return;
        world.enableSleep = flag;
        if (!flag)
        {
            int setCount = world.solverSets.Count;
            for (int i = (int)SetType.FirstSleeping; i < setCount; i++)
            {
                SolverSet set = world.solverSets[i];
                if (set.bodySims.Count > 0) world.WakeSolverSet(i);
            }
        }
    }

    ///<summary> Is body sleeping enabled?</summary>
    public static bool World_IsSleepingEnabled(WorldID worldId) => worldId.index1.enableSleep;

    ///<summary>Enable/disable continuous collision between dynamic and static bodies. Generally you should keep continuous
    /// collision enabled to prevent fast moving objects from going through static objects. The performance gain from
    /// disabling continuous collision is minor.
    /// @see WorldDef</summary>
    public static void World_EnableContinuous(WorldID worldId, bool flag)
    {
        World world = worldId.index1; Debug.Assert(!world.locked); if (world.locked) return;
        world.enableContinuous = flag;
    }

    ///<summary> Is continuous collision enabled?</summary>
    public static bool World_IsContinuousEnabled(WorldID worldId) => worldId.index1.enableContinuous;

    ///<summary>Adjust the restitution threshold. It is recommended not to make this value very small
    /// because it will prevent bodies from sleeping. Usually in meters per second.
    /// @see WorldDef</summary>
    public static void World_SetRestitutionThreshold(WorldID worldId, float value)
    {
        World world = worldId.index1; Debug.Assert(!world.locked); if (world.locked) return;
        world.restitutionThreshold = Math.Clamp(value, 0, float.MaxValue);
    }

    ///<summary> Get the the restitution speed threshold. Usually in meters per second.</summary>
    public static float World_GetRestitutionThreshold(WorldID worldId) => worldId.index1.restitutionThreshold;

    ///<summary>Adjust the hit event threshold. This controls the collision speed needed to generate a ContactHitEvent.
    /// Usually in meters per second.
    /// @see WorldDef::hitEventThreshold</summary>
    public static void World_SetHitEventThreshold(WorldID worldId, float value)
    {
        World world = worldId.index1; Debug.Assert(!world.locked); if (world.locked) return;
        world.hitEventThreshold = Math.Clamp(value, 0, float.MaxValue);
    }

    ///<summary> Get the the hit event speed threshold. Usually in meters per second.</summary>
    public static float World_GetHitEventThreshold(WorldID worldId) => worldId.index1.hitEventThreshold;

    ///<summary> Register the custom filter callback. This is optional.</summary>
    public static void World_SetCustomFilterCallback(WorldID worldId, ref CustomFilterFcn fcn, object context)
    { worldId.index1.customFilterFcn = fcn; worldId.index1.customFilterContext = context; }

    ///<summary> Register the pre-solve callback. This is optional.</summary>
    public static void World_SetPreSolveCallback(WorldID worldId, ref PreSolveFcn fcn, object context)
    { worldId.index1.preSolveFcn = fcn; worldId.index1.preSolveContext = context; }

    ///<summary>Set the gravity vector for the entire world. Box2D has no concept of an up direction and this
    /// is left as a decision for the application. Usually in m/s^2.
    /// @see WorldDef</summary>
    public static void World_SetGravity(WorldID worldId, Vector2 gravity) => worldId.index1.gravity = gravity;

    ///<summary> Get the gravity vector</summary>
    public static Vector2 World_GetGravity(WorldID worldId) => worldId.index1.gravity;

    public class ExplosionContext
    {
        public World world;
        public Vector2 position;
        public float radius, falloff, impulsePerLength;
    }
    public static unsafe bool ExplosionCallback(int proxyId, ulong userData, object context)
    {
        int shapeId = (int)userData;
        ExplosionContext explosionContext = (ExplosionContext)context;
        World world = explosionContext.world;
        Shape shape = world.shapes[shapeId];
        Body body = world.bodies[shape.bodyId];
        Debug.Assert(body.type == BodyType.Dynamic);
        Transform transform = world.GetBodyTransformQuick(body);
        DistanceInput input = new()
        {
            proxyA = shape.MakeDistanceProxy(),
            proxyB = Distance.MakeProxy([explosionContext.position], 1),
            transformA = transform,
            transformB = Transform.Identity,
            useRadii = true
        };
        SimplexCache cache = new();
        DistanceOutput output = input.ShapeDistance(ref cache, null);
        float radius = explosionContext.radius, falloff = explosionContext.falloff;
        if (output.distance > radius + falloff) return true;
        world.WakeBody(body);
        if (body.setIndex != (int)SetType.Awake) return true;
        Vector2 closestPoint = output.pointA;
        if (output.distance == 0)
        {
            Vector2 localCentroid = shape.GetCentroid();
            closestPoint = transform.TransformPoint(localCentroid);
        }
        Vector2 direction = closestPoint - explosionContext.position;
        direction = direction.LengthSquared() > 100 * Box2D.FLT_EPSILON * Box2D.FLT_EPSILON ? direction.Normalize() : new(1, 0);
        Vector2 localLine = transform.q.InvRotateVector(direction.LeftPerp());
        float perimeter = shape.GetProjectedPerimeter(localLine);
        float scale = 1;
        if (output.distance > radius && falloff > 0)
            scale = Math.Clamp((radius + falloff - output.distance) / falloff, 0, 1);
        float magnitude = explosionContext.impulsePerLength * perimeter * scale;
        Vector2 impulse = magnitude * direction;
        int localIndex = body.localIndex;
        SolverSet set = world.solverSets[(int)SetType.Awake];
        BodyState* state = set.bodyStates.Data + localIndex;
        BodySim bodySim = set.bodySims[localIndex];
        state->linearVelocity = Vector2.MulAdd(state->linearVelocity, bodySim.invMass, impulse);
        state->angularVelocity += bodySim.invInertia * Vector2.Cross(closestPoint - bodySim.center, impulse);
        return true;
    }
    /// <summary>Apply a radial explosion</summary>
    /// <param name="worldID">The world id</param>
    /// <param name="explosionDef">The explosion definition</param>
    public static void World_Explode(WorldID worldId, ref ExplosionDef explosionDef)
    {
        Debug.Assert(explosionDef.position.IsValid());
        Debug.Assert(float.IsFinite(explosionDef.radius) && explosionDef.radius >= 0);
        Debug.Assert(float.IsFinite(explosionDef.falloff) && explosionDef.falloff >= 0);
        Debug.Assert(float.IsFinite(explosionDef.impulsePerLength));
        World world = worldId.index1; Debug.Assert(!world.locked); if (world.locked) return;
        ExplosionContext explosionContext = new() { world = world, position = explosionDef.position,
            radius = explosionDef.radius, falloff = explosionDef.falloff, impulsePerLength = explosionDef.impulsePerLength };
        AABB aabb = new(new(explosionDef.position.x - (explosionDef.radius + explosionDef.falloff),
            explosionDef.position.y - (explosionDef.radius + explosionDef.falloff)),
            new(explosionDef.position.x + (explosionDef.radius + explosionDef.falloff),
            explosionDef.position.y + (explosionDef.radius + explosionDef.falloff)));
        world.broadPhase.trees[(int)BodyType.Dynamic].Query(aabb, explosionDef.maskBits, ExplosionCallback, explosionContext);
    }

    ///<summary> Adjust contact tuning parameters</summary>
    /// <param name="worldID">The world id</param>
    /// <param name="hertz">The contact stiffness (cycles per second)</param>
    /// <param name="dampingRatio">The contact bounciness with 1 being critical damping (non-dimensional)</param>
    /// <param name="pushSpeed">The maximum contact constraint push out speed (meters per second)</param>
    /// <remarks>Advanced feature</remarks>
    public static void World_SetContactTuning(WorldID worldId, float hertz, float dampingRatio, float pushSpeed)
    {
        World world = worldId.index1; Debug.Assert(!world.locked); if (world.locked) return;
        world.contactHertz = Math.Clamp(hertz, 0, float.MaxValue);
        world.contactDampingRatio = Math.Clamp(dampingRatio, 0, float.MaxValue);
        world.contactSpeed = Math.Clamp(pushSpeed, 0, float.MaxValue);
    }

    ///<summary> Set the maximum linear speed. Usually in m/s.</summary>
    public static void World_SetMaximumLinearSpeed(WorldID worldId, float maximumLinearSpeed)
    {
        Debug.Assert(float.IsFinite(maximumLinearSpeed) && maximumLinearSpeed > 0);
        World world = worldId.index1; Debug.Assert(!world.locked); if (world.locked) return;
        world.maxLinearSpeed = maximumLinearSpeed;
    }

    ///<summary> Get the maximum linear speed. Usually in m/s.</summary>
    public static float World_GetMaximumLinearSpeed(WorldID worldId) => worldId.index1.maxLinearSpeed;

    ///<summary>Enable/disable constraint warm starting. Advanced feature for testing. Disabling
    /// warm starting greatly reduces stability and provides no performance gain.</summary>
    public static void World_EnableWarmStarting(WorldID worldId, bool flag)
    {
        World world = worldId.index1; Debug.Assert(!world.locked); if (world.locked) return;
        world.enableWarmStarting = flag;
    }

    ///<summary> Is constraint warm starting enabled?</summary>
    public static bool World_IsWarmStartingEnabled(WorldID worldId) => worldId.index1.enableWarmStarting;

    ///<summary> Get the number of awake bodies.</summary>
    public static int World_GetAwakeBodyCount(WorldID worldId) => worldId.index1.solverSets[(int)SetType.Awake].bodySims.Count;

    ///<summary> Get the current world performance profile</summary>
    public static Profile World_GetProfile(WorldID worldId) => worldId.index1.profile;

    ///<summary> Get world counters and sizes</summary>
    public static Counters World_GetCounters(WorldID worldId)
    {
        World world = worldId.index1;
        Counters s = new()
        {
            bodyCount = world.bodyIdPool.GetIdCount(),
            shapeCount = world.shapeIdPool.GetIdCount(),
            contactCount = world.contactIdPool.GetIdCount(),
            jointCount = world.jointIdPool.GetIdCount(),
            islandCount = world.islandIdPool.GetIdCount(),
            staticTreeHeight = world.broadPhase.trees[(int)BodyType.Static].GetHeight(),
            treeHeight = Math.Max(world.broadPhase.trees[(int)BodyType.Dynamic].GetHeight(), world.broadPhase.trees[(int)BodyType.Kinematic].GetHeight()),
            taskCount = world.taskCount
        };
        for (int i = 0; i < Box2D.GraphColorCount; i++)
            s.colorCounts[i] = world.constraintGraph.colors[i].contactSims.Count + world.constraintGraph.colors[i].jointSims.Count;
        return s;
    }

    ///<summary> Set the user data pointer.</summary>
    public static void World_SetUserData(WorldID worldId, object userData) => worldId.index1.userData = userData;

    ///<summary> Get the user data pointer.</summary>
    public static object World_GetUserData(WorldID worldId) => worldId.index1.userData;

    ///<summary> Set the friction callback. Passing NULL resets to default.</summary>
    public static void World_SetFrictionCallback(WorldID worldId, ref FrictionCallback callback)
    {
        World world = worldId.index1; Debug.Assert(!world.locked); if (world.locked) return;
        world.frictionCallback = callback ?? world.frictionCallback;
    }

    ///<summary> Set the restitution callback. Passing NULL resets to default.</summary>
    public static void World_SetRestitutionCallback(WorldID worldId, ref RestitutionCallback callback)
    {
        World world = worldId.index1; Debug.Assert(!world.locked); if (world.locked) return;
        world.restitutionCallback = callback ?? world.restitutionCallback;
    }

    ///<summary> Dump memory stats to box2d_memory.txt</summary>
    ///<remarks>Probably inaccurate in .NET (many object sizes are estimated)</remarks>
    public static unsafe void World_DumpMemoryStats(WorldID worldId)
    {
        System.IO.FileStream f = System.IO.File.OpenWrite("box2d_memory.txt");
        if (f == null) return;
        World world = worldId.index1;
        System.IO.StreamWriter file = new(f);
        file.WriteLine("id pools");
        file.WriteLine($"body pools: {world.bodyIdPool.GetByteCount()}");
        file.WriteLine($"solver set ids: {world.solverSetIdPool.GetByteCount()}");
        file.WriteLine($"joint ids: {world.jointIdPool.GetByteCount()}");
        file.WriteLine($"contact ids: {world.chainIdPool.GetByteCount()}");
        file.WriteLine($"island ids: {world.islandIdPool.GetByteCount()}");
        file.WriteLine($"shape ids: {world.shapeIdPool.GetByteCount()}");
        file.WriteLine($"chain ids: {world.chainIdPool.GetByteCount()}");
        file.WriteLine();
        file.WriteLine("world arrays");
        file.WriteLine($"bodies: {world.bodies.Count}");
        file.WriteLine($"solver sets: {world.solverSets.Count}");
        file.WriteLine($"joints: {world.joints.Count}");
        file.WriteLine($"contacts: {world.contacts.Count}");
        file.WriteLine($"islands: {world.islands.Count}");
        file.WriteLine($"shapes: {world.shapes.Count}");
        file.WriteLine($"chains: {world.chainShapes.Count}");
        file.WriteLine();
        file.WriteLine("broad-phase");
        file.WriteLine($"static tree: {world.broadPhase.trees[(int)BodyType.Static].GetByteCount()}");
        file.WriteLine($"kinematic tree: {world.broadPhase.trees[(int)BodyType.Kinematic].GetByteCount()}");
        file.WriteLine($"dynamic tree: {world.broadPhase.trees[(int)BodyType.Dynamic].GetByteCount()}");
        HashSet<int> moveSet = world.broadPhase.moveSet;
        file.WriteLine($"moveSet: {moveSet.Count * 8} ({moveSet.Count} {moveSet.Count})");
        file.WriteLine($"moveArray: {world.broadPhase.moveArray.Count * 4}");
        HashSet<ulong> pairSet = world.broadPhase.pairSet;
        file.WriteLine($"pairSet: {pairSet.Count * 28} ({pairSet.Count} {pairSet.Count})");
        file.WriteLine();
        int bodySimCapacity = 0;
        int bodyStateCapacity = 0;
        int jointSimCapacity = 0;
        int contactSimCapacity = 0;
        int islandSimCapacity = 0;
        int solverSetCapacity = world.solverSets.Count;
        for (int i = 0; i < solverSetCapacity; ++i)
        {
            SolverSet set = world.solverSets[i];
            if (set.setIndex == -1) continue;
            bodySimCapacity += set.bodySims.Capacity;
            bodyStateCapacity += set.bodyStates.Capacity;
            jointSimCapacity += set.jointSims.Capacity;
            contactSimCapacity += set.contactSims.Capacity;
            islandSimCapacity += set.islandSims.Capacity;
        }
        file.WriteLine("solver sets");
        file.WriteLine($"body sim: {bodySimCapacity * 104}");
        file.WriteLine($"body state: {bodyStateCapacity * sizeof(BodyState)}");
        file.WriteLine($"joint sim: {jointSimCapacity * 104}");
        file.WriteLine($"contact sim: {contactSimCapacity * (64 + sizeof(Manifold) + sizeof(SimplexCache))}");
        file.WriteLine($"island sim: {islandSimCapacity * 12}");
        file.WriteLine();
        int bodyBitSetBytes = 0;
        contactSimCapacity = 0;
        jointSimCapacity = 0;
        for (int i = 0; i < Box2D.GraphColorCount; ++i)
        {
            ref GraphColor c = ref world.constraintGraph.colors[i];
            bodyBitSetBytes += c.bodySet.GetBitSetBytes();
            contactSimCapacity += c.contactSims.Capacity;
            jointSimCapacity += c.jointSims.Capacity;
        }
        file.WriteLine("constraint graph");
        file.WriteLine($"body bit sets: {bodyBitSetBytes}");
        file.WriteLine($"joint sim: {jointSimCapacity * 104}");
        file.WriteLine($"contact sim: {contactSimCapacity * (64 + sizeof(Manifold) + sizeof(SimplexCache))}");
        file.WriteLine();
        file.Close(); file.Dispose(); f.Dispose();
    }

    ///<summary> This is for internal testing</summary>
    public static void World_RebuildStaticTree(WorldID worldId)
    {
        World world = worldId.index1; Debug.Assert(!world.locked); if (world.locked) return;
        world.broadPhase.trees[(int)SetType.Static].Rebuild(true);
    }

    ///<summary> This is for internal testing</summary>
    public static void World_EnableSpeculative(WorldID worldId, bool flag) => worldId.index1.enableSpeculative = flag;
}
