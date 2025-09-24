using System.Diagnostics;
using System.Collections.Generic;

namespace Box2D;

/// <summary>Used to track shapes that hit sensors using time of impact</summary>
public struct SensorHit
{
    public int sensorId;
    public int visitorId;
}
public class Visitor
{
    public int shapeId;
    public ushort generation;
    public static int Compare(Visitor a, Visitor b) => a.shapeId.CompareTo(b.shapeId);
}
public class SensorTaskContext
{
    public BitSet eventBits;
}
public class Sensor
{
    /// <summary>todo find a way to pool these</summary>
    public List<Visitor> hits = new(), overlaps1 = new(), overlaps2 = new();
    public int shapeId;
}
public partial class World
{
    public class SensorQueryContext
    {
        public World world;
        public SensorTaskContext taskContext;
        public Sensor sensor;
        public Shape sensorShape;
        public Transform transform;
    }
    public static bool SensorQueryCallback(int proxyId, ulong userData, object context)
    {
        int shapeId = (int)userData;
        SensorQueryContext queryContext = (SensorQueryContext)context;
        Shape sensorShape = queryContext.sensorShape;
        int sensorShapeId = sensorShape.id;
        if (shapeId == sensorShapeId) return true;
        World world = queryContext.world;
        Shape otherShape = world.shapes[shapeId];
        if (!otherShape.enableSensorEvents) return true;
        if (otherShape.bodyId == sensorShape.bodyId) return true;
        if (!Shape.ShouldShapesCollide(sensorShape.filter, otherShape.filter)) return true;
        if (sensorShape.enableCustomFiltering || otherShape.enableCustomFiltering)
        {
            CustomFilterFcn customFilterFcn = queryContext.world.customFilterFcn;
            if (customFilterFcn != null)
            {
                ShapeID idA = new() { index1 = sensorShapeId + 1, world0 = world, generation = sensorShape.generation };
                ShapeID idB = new() { index1 = shapeId + 1, world0 = world, generation = otherShape.generation };
                bool shouldCollide = customFilterFcn(idA, idB, queryContext.world.customFilterContext);
                if (!shouldCollide) return true;
            }
        }
        Transform otherTransform = world.GetBodyTransform(otherShape.bodyId);
        DistanceInput input = new()
        {
            proxyA = sensorShape.MakeDistanceProxy(),
            proxyB = otherShape.MakeDistanceProxy(),
            transformA = queryContext.transform,
            transformB = otherTransform,
            useRadii = true
        };
        SimplexCache cache = new();
        DistanceOutput output = input.ShapeDistance(ref cache, null);
        if (output.distance >= 10 * Box2D.FLT_EPSILON) return true;
        Sensor sensor = queryContext.sensor;
        sensor.overlaps2.Add(new() { shapeId = shapeId, generation = otherShape.generation });
        return true;
    }
    public static void SensorTask(int startIndex, int endIndex, uint threadIndex, object context)
    {
        World world = (World)context;
        Debug.Assert(threadIndex < world.workerCount);
        SensorTaskContext taskContext = world.sensorTaskContexts[(int)threadIndex];
        Debug.Assert(startIndex < endIndex);
        DynamicTree[] trees = world.broadPhase.trees;
        for (int sensorIndex = startIndex; sensorIndex < endIndex; sensorIndex++)
        {
            Sensor sensor = world.sensors[sensorIndex];
            Shape sensorShape = world.shapes[sensor.shapeId];
            (sensor.overlaps1, sensor.overlaps2) = (sensor.overlaps2, sensor.overlaps1);
            sensor.overlaps2.Clear();
            int hitCount = sensor.hits.Count;
            for (int i = 0; i < hitCount; i++)
                sensor.overlaps2.Add(sensor.hits[i]);
            sensor.hits.Clear();
            Body body = world.bodies[sensorShape.bodyId];
            if (body.setIndex == (int)SetType.Disabled || !sensorShape.enableSensorEvents)
            {
                if (sensor.overlaps1.Count != 0)
                    taskContext.eventBits.SetBit(sensorIndex);
                continue;
            }
            Transform transform = world.GetBodyTransformQuick(body);
            SensorQueryContext queryContext = new()
            { world = world, taskContext = taskContext, sensor = sensor, sensorShape = sensorShape, transform = transform };
            Debug.Assert(sensorShape.sensorIndex == sensorIndex);
            AABB queryBounds = sensorShape.aabb;
            trees[0].Query(queryBounds, sensorShape.filter.maskBits, SensorQueryCallback, queryContext);
            trees[1].Query(queryBounds, sensorShape.filter.maskBits, SensorQueryCallback, queryContext);
            trees[2].Query(queryBounds, sensorShape.filter.maskBits, SensorQueryCallback, queryContext);
            //sensor.overlaps2.AsParallel().OrderBy(x => x, Comparer<Visitor>.Create(Visitor.Compare))
            //    .Distinct(Comparer<Visitor>.Create(Visitor.Compare));
            sensor.overlaps2.Sort(Visitor.Compare);
            int uniqueCount = 0;
            int overlapCount = sensor.overlaps2.Count;
            for (int i = 0; i < overlapCount; i++)
                if (uniqueCount == 0 || sensor.overlaps2[i].shapeId != sensor.overlaps2[uniqueCount - 1].shapeId)
                    sensor.overlaps2[uniqueCount++] = sensor.overlaps2[i];
            sensor.overlaps2.RemoveRange(uniqueCount, sensor.overlaps2.Count - uniqueCount);
            int count1 = sensor.overlaps1.Count, count2 = sensor.overlaps2.Count;
            if (count1 != count2) taskContext.eventBits.SetBit(sensorIndex);
            else for (int i = 0; i < count1; i++)
                {
                    if (sensor.overlaps1[i].shapeId != sensor.overlaps2[i].shapeId || sensor.overlaps1[i].generation != sensor.overlaps2[i].generation)
                    {
                        taskContext.eventBits.SetBit(sensorIndex);
                        break;
                    }
                }
        }
    }
    public void OverlapSensors()
    {
        int sensorCount = sensors.Count;
        if (sensors.Count == 0) return;
        Debug.Assert(workerCount > 0);
        for (int i = 0; i < workerCount; i++) sensorTaskContexts[i].eventBits.SetBitCountAndClear(sensorCount);
        int minRange = 16;
        object userSensorTask = enqueueTaskFcn(SensorTask, sensorCount, minRange, this, userTaskContext);
        taskCount++;
        if (userSensorTask != null) finishTaskFcn(userSensorTask, userTaskContext);
        ref BitSet bitset = ref sensorTaskContexts[0].eventBits;
        for (int i = 1; i < workerCount; i++) bitset.InPlaceUnion(sensorTaskContexts[i].eventBits);
        ulong[] bits = bitset.bits;
        uint blockCount = bitset.blockCount;
        for (uint k = 0; k < blockCount; k++)
        {
            ulong word = bits[k];
            while (word != 0)
            {
                uint ctz = CTZ.CTZ64(word);
                int sensorIndex = (int)(64 * k + ctz);
                Sensor sensor = sensors[sensorIndex];
                Shape sensorShape = shapes[sensor.shapeId];
                ShapeID sensorId = new() { index1 = sensor.shapeId + 1, world0 = this, generation = sensorShape.generation };
                int count1 = sensor.overlaps1.Count, count2 = sensor.overlaps2.Count;
                int index1 = 0, index2 = 0;
                while (index1 < count1 && index2 < count2)
                {
                    Visitor r1 = sensor.overlaps1[index1], r2 = sensor.overlaps2[index2];
                    if (r1.shapeId == r2.shapeId)
                    {
                        if (r1.generation < r2.generation)
                        {
                            ShapeID visitorId = new() { index1 = r1.shapeId + 1, world0 = this, generation = r1.generation };
                            if (endEventArrayIndex == 1)
                                sensorEndEvents1.Add(new() { sensorShapeId = sensorId, visitorShapeId = visitorId });
                            else sensorEndEvents0.Add(new() { sensorShapeId = sensorId, visitorShapeId = visitorId });
                            index1++;
                        }
                        else if (r1.generation > r2.generation)
                        {
                            ShapeID visitorId = new() { index1 = r2.shapeId + 1, world0 = this, generation = r2.generation };
                            sensorBeginEvents.Add(new() { sensorShapeId = sensorId, visitorShapeId = visitorId });
                            index2++;
                        }
                        else { index1++; index2++; }
                    }
                    else if (r1.shapeId < r2.shapeId)
                    {
                        ShapeID visitorId = new() { index1 = r1.shapeId + 1, world0 = this, generation = r1.generation };
                        if (endEventArrayIndex == 1)
                            sensorEndEvents1.Add(new() { sensorShapeId = sensorId, visitorShapeId = visitorId });
                        else sensorEndEvents0.Add(new() { sensorShapeId = sensorId, visitorShapeId = visitorId });
                        index1++;
                    }
                    else
                    {
                        ShapeID visitorId = new() { index1 = r2.shapeId + 1, world0 = this, generation = r2.generation };
                        sensorBeginEvents.Add(new() { sensorShapeId = sensorId, visitorShapeId = visitorId });
                        index2++;
                    }
                }
                while (index1 < count1)
                {
                    Visitor r1 = sensor.overlaps1[index1];
                    ShapeID visitorId = new() { index1 = r1.shapeId + 1, world0 = this, generation = r1.generation };
                    if (endEventArrayIndex == 1)
                        sensorEndEvents1.Add(new() { sensorShapeId = sensorId, visitorShapeId = visitorId });
                    else sensorEndEvents0.Add(new() { sensorShapeId = sensorId, visitorShapeId = visitorId });
                    index1++;
                }
                while (index2 < count2)
                {
                    Visitor r2 = sensor.overlaps1[index1];
                    ShapeID visitorId = new() { index1 = r2.shapeId + 1, world0 = this, generation = r2.generation };
                    sensorBeginEvents.Add(new() { sensorShapeId = sensorId, visitorShapeId = visitorId });
                    index2++;
                }
                word &= word - 1;
            }
        }
    }
    public void DestroySensor(Shape sensorShape)
    {
        Sensor sensor = sensors[sensorShape.sensorIndex];
        for (int i = 0; i < sensor.overlaps2.Count; i++)
        {
            if (endEventArrayIndex == 1) sensorEndEvents1.Add(new()
            {
                sensorShapeId = new() { index1 = sensorShape.id + 1, world0 = this, generation = sensorShape.generation },
                visitorShapeId = new() { index1 = sensor.overlaps2[i].shapeId + 1, world0 = this, generation = sensor.overlaps2[i].generation }
            });
            else sensorEndEvents0.Add(new()
            {
                sensorShapeId = new() { index1 = sensorShape.id + 1, world0 = this, generation = sensorShape.generation },
                visitorShapeId = new() { index1 = sensor.overlaps2[i].shapeId + 1, world0 = this, generation = sensor.overlaps2[i].generation }
            });
        }
        int movedIndex = sensors.RemoveSwap(sensorShape.sensorIndex);
        if (movedIndex != -1)
        {
            Sensor movedSensor = sensors[sensorShape.sensorIndex];
            Shape otherSensorShape = shapes[movedSensor.shapeId];
            otherSensorShape.sensorIndex = sensorShape.sensorIndex;
        }
    }
}