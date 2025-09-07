using System;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace Box2D;

/// <summary>Task interface<br/>
/// This is prototype for a Box2D task. Your task system is expected to invoke the Box2D task with these arguments.
/// The task spans a range of the parallel-for: [startIndex, endIndex)
/// The worker index must correctly identify each worker in the user thread pool, expected in [0, workerCount).
/// A worker must only exist on only one thread at a time and is analogous to the thread index.
/// The task context is the context pointer sent from Box2D when it is enqueued.
/// The startIndex and endIndex are expected in the range [0, itemCount) where itemCount is the argument to b2EnqueueTaskCallback
/// below. Box2D expects startIndex &lt; endIndex and will execute a loop like this:
/// <code>
/// for (int i = startIndex; i &lt; endIndex; ++i)
/// {
/// 	DoWork();
/// }
/// </code></summary>
public delegate void TaskCallback(int startIndex, int endIndex, uint workerIndex, object taskContext);
/// <summary>These functions can be provided to Box2D to invoke a task system. These are designed to work well with enkiTS.
/// Returns a pointer to the user's task object. May be nullptr. A nullptr indicates to Box2D that the work was executed
/// serially within the callback and there is no need to call b2FinishTaskCallback.
/// The itemCount is the number of Box2D work items that are to be partitioned among workers by the user's task system.
/// This is essentially a parallel-for. The minRange parameter is a suggestion of the minimum number of items to assign
/// per worker to reduce overhead. For example, suppose the task is small and that itemCount is 16. A minRange of 8 suggests
/// that your task system should split the work items among just two workers, even if you have more available.
/// In general the range [startIndex, endIndex) send to b2TaskCallback should obey:<br/>
/// endIndex - startIndex &gt;= minRange<br/>
/// The exception of course is when itemCount &lt; minRange.</summary>
public delegate object EnqueueTaskCallback(TaskCallback task, int itemCount, int minRange, object taskContext, object userContext);
/// <summary>Finishes a user task object that wraps a Box2D task.</summary>
public delegate void FinishTaskCallback(object userTask, object userContext);
/// <summary>Optional friction mixing callback. This intentionally provides no context objects because this is called
/// from a worker thread.
/// This function should not attempt to modify Box2D state or user application state-> </summary>
public delegate float FrictionCallback(float frictionA, ulong userMaterialIdA, float frictionB, ulong userMaterialIdB);
/// <summary>Optional restitution mixing callback. This intentionally provides no context objects because this is called
/// from a worker thread.
/// This function should not attempt to modify Box2D state or user application state-></summary>
public delegate float RestitutionCallback(float restitutionA, ulong userMaterialIdA, float restitutionB, ulong userMaterialIdB);
/// <summary>Result from b2World_RayCastClosest
/// If there is initial overlap the fraction and normal will be zero while the point is an arbitrary point in the overlap region.</summary>
public class RayResult
{
    public ShapeID shapeID;
    public Vector2 point;
    public Vector2 normal;
    public float fraction;
    public int nodeVisits;
    public int leafVisits;
    public bool hit;
}
public class DefaultTaskObject
{
    public List<Task> tasks = new();
    public void Await()
    {
        for (int i = 0; i < tasks.Count; i++) tasks[i].Wait();
    }
}
/// <summary>World definition used to create a simulation world.
/// Must be initialized using b2DefaultWorldDef().</summary>
public struct WorldDef
{
    /// <summary>Gravity vector. Box2D has no up-vector defined.</summary>
    public Vector2 gravity = new(0, -10);
    /// <summary>Restitution speed threshold, usually in m/s. Collisions above this
    /// speed have restitution applied (will bounce).</summary>
    public float restitutionThreshold = Box2D.LengthUnitsPerMeter;
    /// <summary>Threshold speed for hit events. Usually meters per second.</summary>
    public float hitEventThreshold = Box2D.LengthUnitsPerMeter;
    /// <summary>Contact stiffness. Cycles per second. Increasing this increases the speed of overlap recovery, but can introduce jitter.</summary>
    public float contactHertz = 30;
    /// <summary>Contact bounciness. Non-dimensional. You can speed up overlap recovery by decreasing this with
    /// the trade-off that overlap resolution becomes more energetic.</summary>
    public float contactDampingRatio = 10;
    /// <summary>This parameter controls how fast overlap is resolved and usually has units of meters per second. This only
    /// puts a cap on the resolution speed. The resolution speed is increased by increasing the hertz and/or
    /// decreasing the damping ratio.</summary>
    public float contactSpeed = 3 * Box2D.LengthUnitsPerMeter;
    /// <summary>Maximum linear speed. Usually meters per second.</summary>
    public float maximumLinearSpeed = 400 * Box2D.LengthUnitsPerMeter;
    /// <summary>Optional mixing callback for friction. The default uses sqrt(frictionA * frictionB).</summary>
    public FrictionCallback frictionCallback = (a, _, b, _) => MathF.Sqrt(a * b);
    /// <summary>Optional mixing callback for restitution. The default uses max(restitutionA, restitutionB).</summary>
    public RestitutionCallback restitutionCallback = (a, _, b, _) => Math.Max(a, b);
    /// <summary>Can bodies go to sleep to improve performance</summary>
    public bool enableSleep = true;
    /// <summary>Enable continuous collision</summary>
    public bool enableContinuous = true;
    /// <summary>Contact softening when mass ratios are large. Experimental.</summary>
    public bool enableContactSoftening = false;
    /// <summary>Number of workers to use with the provided task system. Box2D performs best when using only
    /// performance cores and accessing a single L2 cache. Efficiency cores and hyper-threading provide
    /// little benefit and may even harm performance.</summary>
    /// <remarks>Box2D does not create threads. This is the number of threads your applications has created
    /// that you are allocating to b2World_Step.
    /// Do not modify the default value unless you are also providing a task system and providing
    /// task callbacks (enqueueTask and finishTask).</remarks>
    private int workerCount = Environment.ProcessorCount;
    /// <summary>Function to spawn tasks</summary>
    public EnqueueTaskCallback enqueueTask = (task, itemCount, minRange, taskContext, userContext) =>
    {
        int threads = Math.Min(itemCount, Environment.ProcessorCount);
        DefaultTaskObject t = new();
        for (int i = 0; i < threads; i++)
        {
            int i2 = i;
            t.tasks.Add(Task.Run(() => task(itemCount * i2 / threads, itemCount * (i2 + 1) / threads, (uint)i2, taskContext)));
        }
        return t;
    };
    /// <summary>Function to finish a task</summary>
    public FinishTaskCallback finishTask = (userTask, userObject) => ((DefaultTaskObject)userTask).Await();
    /// <summary>User context that is provided to enqueueTask and finishTask</summary>
    public object userTaskContext = null;
    /// <summary>User data</summary>
    public object userData = null;
    /// <summary>Used internally to detect a valid definition. DO NOT SET.</summary>
    internal int internalValue = Box2D.SECRET_COOKIE;

    public int WorkerCount { get => workerCount; set => workerCount = value; }

    public WorldDef() { }
}
/// <summary>The body simulation type.
/// Each body is one of these three types. The type determines how the body behaves in the simulation.</summary>
public enum BodyType
{
    /// <summary>zero mass, zero velocity, may be manually moved</summary>
    Static,
    /// <summary>zero mass, velocity set by user, moved by solver</summary>
    Kinematic,
    /// <summary>positive mass, velocity determined by forces, moved by solver</summary>
    Dynamic
}
/// <summary>Motion locks to restrict the body movement</summary>
public struct MotionLocks
{
    /// <summary>Prevent translation along the x-axis</summary>
    public bool linearX;
    /// <summary>Prevent translation along the y-axis</summary>
    public bool linearY;
    /// <summary>Prevent rotation around the z-axis</summary>
    public bool angularZ;
}
/// <summary>A body definition holds all the data needed to construct a rigid body.
/// You can safely re-use body definitions. Shapes are added to a body after construction.
/// Body definitions are temporary objects used to bundle creation parameters.
/// Must be initialized using b2DefaultBodyDef().</summary>
public struct BodyDef
{
    /// <summary>The body type: static, kinematic, or dynamic.</summary>
    public BodyType type = BodyType.Static;
    /// <summary>The initial world position of the body. Bodies should be created with the desired position.</summary>
    /// <remarks>Creating bodies at the origin and then moving them nearly doubles the cost of body creation, especially
    /// if the body is moved after shapes have been added.</remarks>
    public Vector2 position = Vector2.Zero;
    /// <summary>The initial world rotation of the body. Use b2MakeRot() if you have an angle.</summary>
    public Rotation rotation = Rotation.Identity;
    /// <summary>The initial linear velocity of the body's origin. Usually in meters per second.</summary>
    public Vector2 linearVelocity = Vector2.Zero;
    /// <summary>The initial angular velocity of the body. Radians per second.</summary>
    public float angularVelocity = 0;
    /// <summary>Linear damping is used to reduce the linear velocity. The damping parameter
    /// can be larger than 1 but the damping effect becomes sensitive to the
    /// time step when the damping parameter is large.
    /// Generally linear damping is undesirable because it makes objects move slowly
    /// as if they are floating.</summary>
    public float linearDamping = 0;
    /// <summary>Angular damping is used to reduce the angular velocity. The damping parameter
    /// can be larger than 1.0f but the damping effect becomes sensitive to the
    /// time step when the damping parameter is large.
    /// Angular damping can be use slow down rotating bodies.</summary>
    public float angularDamping = 0;
    /// <summary>Scale the gravity applied to this body. Non-dimensional.</summary>
    public float gravityScale = 1;
    /// <summary>Sleep speed threshold, default is 0.05 meters per second</summary>
    public float sleepThreshold = 0.05f * Box2D.LengthUnitsPerMeter;
    /// <summary>Optional body name for debugging. Up to 31 characters (excluding null termination)</summary>
    public string name = null;
    /// <summary>Use this to store application specific body data.</summary>
    public object userData = null;
    /// <summary>Motions locks to restrict linear and angular movement.
    /// Caution: may lead to softer constraints along the locked direction</summary>
    public MotionLocks motionLocks = new();
    /// <summary>Set this flag to false if this body should never fall asleep.</summary>
    public bool enableSleep = true;
    /// <summary>Is this body initially awake or sleeping?</summary>
    public bool isAwake = true;
    /// <summary>Treat this body as high speed object that performs continuous collision detection
    /// against dynamic and kinematic bodies, but not other bullet bodies.
    /// Bullets should be used sparingly. They are not a solution for general dynamic-versus-dynamic
    /// continuous collision.</summary>
    public bool isBullet = false;
    /// <summary>Used to disable a body. A disabled body does not move or collide.</summary>
    public bool isEnabled = true;
    /// <summary>This allows this body to bypass rotational speed limits. Should only be used
    /// for circular objects, like wheels.</summary>
    public bool allowFastRotation = false;
    /// <summary>Used internally to detect a valid definition. DO NOT SET.</summary>
    internal int internalValue = Box2D.SECRET_COOKIE;
    public BodyDef() { }
}
/// <summary>This is used to filter collision on shapes. It affects shape-vs-shape collision
/// and shape-versus-query collision (such as b2World_CastRay).</summary>
public struct Filter
{
    /// <summary>The collision category bits. Normally you would just set one bit. The category bits should
    /// represent your application object types. For example:
    /// <code>
    /// enum MyCategories
    /// {
    ///    Static  = 0x00000001,
    ///    Dynamic = 0x00000002,
    ///    Debris  = 0x00000004,
    ///    Player  = 0x00000008,
    ///    // etc
    /// };
    /// </code></summary>
    public ulong categoryBits = Box2D.DEFAULT_CATEGORY_BITS;
    /// <summary>The collision mask bits. This states the categories that this
    /// shape would accept for collision.
    /// For example, you may want your player to only collide with static objects
    /// and other players.
    /// <code>
    /// maskBits = Static | Player;
    /// </code></summary>
    public ulong maskBits = Box2D.DEFAULT_MASK_BITS;
    /// <summary>Collision groups allow a certain group of objects to never collide (negative)
    /// or always collide (positive). A group index of zero has no effect. Non-zero group filtering
    /// always wins against the mask bits.
    /// For example, you may want ragdolls to collide with other ragdolls but you don't want
    /// ragdoll self-collision. In this case you would give each ragdoll a unique negative group index
    /// and apply that group index to all shapes on the ragdoll.</summary>
    public int groupIndex = 0;
    public Filter() { }
}
/// <summary>The query filter is used to filter collisions between queries and shapes. For example,
/// you may want a ray-cast representing a projectile to hit players and the static environment
/// but not debris.</summary>
public struct QueryFilter
{
    /// <summary>The collision category bits of this query. Normally you would just set one bit.</summary>
    public ulong categoryBits = Box2D.DEFAULT_CATEGORY_BITS;
    /// <summary>The collision mask bits. This states the shape categories that this
    /// query would accept for collision.</summary>
    public ulong maskBits = Box2D.DEFAULT_MASK_BITS;
    public QueryFilter() { }
}
/// <summary>Shape type</summary>
public enum ShapeType
{
    /// <summary>A circle with an offset</summary>
    Circle,
    /// <summary>A capsule is an extruded circle</summary>
    Capsule,
    /// <summary>A line segment</summary>
    Segment,
    /// <summary>A convex polygon</summary>
    Polygon,
    /// <summary>A line segment owned by a chain shape</summary>
    ChainSegment
}
/// <summary>Surface materials allow chain shapes to have per segment surface properties.</summary>
public struct SurfaceMaterial
{
    /// <summary>The Coulomb (dry) friction coefficient, usually in the range [0,1].</summary>
    public float friction = 0.6f;
    /// <summary>The coefficient of restitution (bounce) usually in the range [0,1].
    /// https://en.wikipedia.org/wiki/Coefficient_of_restitution</summary>
    public float restitution = 0;
    /// <summary>The rolling resistance usually in the range [0,1].</summary>
    public float rollingResistance = 0;
    /// <summary>The tangent speed for conveyor belts</summary>
    public float tangentSpeed = 0;
    /// <summary>User material identifier. This is passed with query results and to friction and restitution
    /// combining functions. It is not used internally.</summary>
    public ulong userMaterialId = 0;
    /// <summary>Custom debug draw color.</summary>
    public uint customColor = 0;
    public SurfaceMaterial() { }
}
/// <summary>Used to create a shape.
/// This is a temporary object used to bundle shape creation parameters. You may use
/// the same shape definition to create multiple shapes.
/// Must be initialized using b2DefaultShapeDef().</summary>
public struct ShapeDef
{
    /// <summary>Use this to store application specific shape data.</summary>
    public object userData = null;
    /// <summary>The surface material for this shape.</summary>
    public SurfaceMaterial material = new();
    /// <summary>The density, usually in kg/m^2.
    /// This is not part of the surface material because this is for the interior, which may have
    /// other considerations, such as being hollow. For example a wood barrel may be hollow or full of water.</summary>
    public float density = 1;
    /// <summary>Collision filtering data.</summary>
    public Filter filter = new();
    /// <summary>Enable custom filtering. Only one of the two shapes needs to enable custom filtering. See b2WorldDef.</summary>
    public bool enableCustomFiltering;
    /// <summary>A sensor shape generates overlap events but never generates a collision response.
    /// Sensors do not have continuous collision. Instead, use a ray or shape cast for those scenarios.
    /// Sensors still contribute to the body mass if they have non-zero density.</summary>
    /// <remarks>Sensor events are disabled by default.</remarks>
    public bool isSensor = false;
    /// <summary>Enable sensor events for this shape. This applies to sensors and non-sensors. False by default, even for sensors.</summary>
    public bool enableSensorEvents = false;
    /// <summary>Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors. False by default.</summary>
    public bool enableContactEvents = false;
    /// <summary>Enable hit events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors. False by default.</summary>
    public bool enableHitEvents = false;
    /// <summary>Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
    /// and must be carefully handled due to multithreading. Ignored for sensors.</summary>
    public bool enablePreSolveEvents = false;
    /// <summary>When shapes are created they will scan the environment for collision the next time step. This can significantly slow down
    /// static body creation when there are many static shapes.
    /// This is flag is ignored for dynamic and kinematic shapes which always invoke contact creation.</summary>
    public bool invokeContactCreation = true;
    /// <summary>Should the body update the mass properties when this shape is created. Default is true.</summary>
    public bool updateBodyMass = true;
    /// <summary>Used internally to detect a valid definition. DO NOT SET.</summary>
    internal int internalValue = Box2D.SECRET_COOKIE;
    public ShapeDef() { }
}
/// <summary>Used to create a chain of line segments. This is designed to eliminate ghost collisions with some limitations.<br/>
/// - chains are one-sided<br/>
/// - chains have no mass and should be used on static bodies<br/>
/// - chains have a counter-clockwise winding order (normal points right of segment direction)<br/>
/// - chains are either a loop or open<br/>
/// - a chain must have at least 4 points<br/>
/// - the distance between any two points must be greater than B2_LINEAR_SLOP<br/>
/// - a chain shape should not self intersect (this is not validated)<br/>
/// - an open chain shape has NO COLLISION on the first and final edge<br/>
/// - you may overlap two open chains on their first three and/or last three points to get smooth collision<br/>
/// - a chain shape creates multiple line segment shapes on the body<br/>
/// https://en.wikipedia.org/wiki/Polygonal_chain<br/>
/// Must be initialized using b2DefaultChainDef().<br/>
/// Do not use chain shapes unless you understand the limitations. This is an advanced feature.</summary>
public struct ChainDef
{
    /// <summary>Use this to store application specific shape data.</summary>
    public object userData = null;
    /// <summary>An array of at least 4 points. These are cloned and may be temporary.</summary>
    public Vector2[] points = null;
    /// <summary>Surface materials for each segment. These are cloned.</summary>
    public SurfaceMaterial[] materials = [new()];
    /// <summary>Contact filtering data.</summary>
    public Filter filter = new();
    /// <summary>Indicates a closed chain formed by connecting the first and last points</summary>
    public bool isLoop = false;
    /// <summary>Enable sensors to detect this chain. False by default.</summary>
    public bool enableSensorEvents = false;
    /// <summary>Used internally to detect a valid definition. DO NOT SET.</summary>
    internal int internalValue = Box2D.SECRET_COOKIE;
    public ChainDef() { }
}
/// <summary>Profiling data. Times are in milliseconds.</summary>
public class Profile
{
    public float step;
    public float pairs;
    public float collide;
    public float solve;
    public float prepareStages;
    public float solveConstraints;
    public float prepareConstraints;
    public float integrateVelocities;
    public float warmStart;
    public float solveImpulses;
    public float integratePositions;
    public float relaxImpulses;
    public float applyRestitution;
    public float storeImpulses;
    public float splitIslands;
    public float transforms;
    public float sensorHits;
    public float jointEvents;
    public float hitEvents;
    public float refit;
    public float bullets;
    public float sleepIslands;
    public float sensors;
}
/// <summary>Counters that give details of the simulation size.</summary>
public struct Counters
{
    public int bodyCount = 0;
    public int shapeCount = 0;
    public int contactCount = 0;
    public int jointCount = 0;
    public int islandCount = 0;
    public int stackUsed = 0;
    public int staticTreeHeight = 0;
    public int treeHeight = 0;
    public int byteCount = 0;
    public int taskCount = 0;
    public int[] colorCounts = new int[24];
    public Counters() { }
}
/// <summary>Joint type enumeration<br/>
/// This is useful because all joint types use b2JointId and sometimes you
/// want to get the type of a joint.</summary>
public enum JointType
{
    Distance, Filter, Motor, Prismatic, Revolute, Weld, Wheel
}
/// <summary>Base joint definition used by all joint types.
/// The local frames are measured from the body's origin rather than the center of mass because:
/// 1. you might not know where the center of mass will be
/// 2. if you add/remove shapes from a body and recompute the mass, the joints will be broken</summary>
public struct JointDef
{
    /// <summary>User data pointer</summary>
    public object userData = null;
    /// <summary>The first attached body</summary>
    public BodyID bodyIdA = new();
    /// <summary>The second attached body</summary>
    public BodyID bodyIdB = new();
    /// <summary>The first local joint frame</summary>
    public Transform localFrameA = Transform.Identity;
    /// <summary>The second local joint frame</summary>
    public Transform localFrameB = Transform.Identity;
    /// <summary>Force threshold for joint events</summary>
    public float forceThreshold = float.MaxValue;
    /// <summary>Torque threshold for joint events</summary>
    public float torqueThreshold = float.MaxValue;
    /// <summary>Constraint hertz (advanced feature)</summary>
    public float constraintHertz = 60;
    /// <summary>Constraint damping ratio (advanced feature)</summary>
    public float constraintDampingRatio = 2;
    /// <summary>Debug draw scale</summary>
    public float drawScale = 1;
    /// <summary>Set this flag to true if the attached bodies should collide</summary>
    public bool collideConnected = false;
    public JointDef() { }
}
/// <summary>Distance joint definition
/// Connects a point on body A with a point on body B by a segment.
/// Useful for ropes and springs.</summary>
public struct DistanceJointDef
{
    /// <summary>Base joint definition</summary>
    public JointDef base_ = new();
    /// <summary>The rest length of this joint. Clamped to a stable minimum value.</summary>
    public float length = 1;
    /// <summary>Enable the distance constraint to behave like a spring. If false
    /// then the distance joint will be rigid, overriding the limit and motor.</summary>
    public bool enableSpring = false;
    /// <summary>The lower spring force controls how much tension it can sustain</summary>
    public float lowerSpringForce = -float.MaxValue;
    /// <summary>The upper spring force controls how much compression it an sustain</summary>
    public float upperSpringForce = float.MaxValue;
    /// <summary>The spring linear stiffness Hertz, cycles per second</summary>
    public float hertz = 0;
    /// <summary>The spring linear damping ratio, non-dimensional</summary>
    public float dampingRatio = 0;
    /// <summary>Enable/disable the joint limit</summary>
    public bool enableLimit = false;
    /// <summary>Minimum length. Clamped to a stable minimum value.</summary>
    public float minLength = 0;
    /// <summary>Maximum length. Must be greater than or equal to the minimum length.</summary>
    public float maxLength = Box2D.Huge;
    /// <summary>Enable/disable the joint motor</summary>
    public bool enableMotor = false;
    /// <summary>The maximum motor force, usually in newtons</summary>
    public float maxMotorForce = 0;
    /// <summary>The desired motor speed, usually in meters per second</summary>
    public float motorSpeed = 0;
    /// <summary>Used internally to detect a valid definition. DO NOT SET.</summary>
    internal int internalValue = Box2D.SECRET_COOKIE;
    public DistanceJointDef() { }
}
/// <summary>A motor joint is used to control the relative motion between two bodies
/// You may move local frame A to change the target transform.
/// A typical usage is to control the movement of a dynamic body with respect to the ground.</summary>
public struct MotorJointDef
{
    /// <summary>Base joint definition</summary>
    public JointDef base_ = new();
    /// <summary>The desired linear velocity</summary>
    public Vector2 linearVelocity;
    /// <summary>The maximum motor force in newtons</summary>
    public float maxVelocityForce;
    /// <summary>The desired angular velocity</summary>
    public float angularVelocity;
    /// <summary>The maximum motor torque in newton-meters</summary>
    public float maxVelocityTorque;
    /// <summary>Position correction factor in the range [0,1]</summary>
    public float correctionFactor;
    /// <summary>Linear spring hertz for position control</summary>
    public float linearHertz;
    /// <summary>Linear spring damping ratio</summary>
    public float linearDampingRatio;
    /// <summary>Maximum spring force in newtons</summary>
    public float maxSpringForce;
    /// <summary>Angular spring hertz for position control</summary>
    public float angularHertz;
    /// <summary>Angular spring damping ratio</summary>
    public float angularDampingRatio;
    /// <summary>Maximum spring torque in newton-meters</summary>
    public float maxSpringTorque;
    /// <summary>Used internally to detect a valid definition. DO NOT SET.</summary>
    internal int internalValue = Box2D.SECRET_COOKIE;
    public MotorJointDef() { }
}
/// <summary>A filter joint is used to disable collision between two specific bodies.</summary>
public struct FilterJointDef
{
    /// <summary>Base joint definition</summary>
    public JointDef base_ = new();
    /// <summary>Used internally to detect a valid definition. DO NOT SET.</summary>
    internal int internalValue = Box2D.SECRET_COOKIE;
    public FilterJointDef() { }
}
/// <summary>Prismatic joint definition
/// Body B may slide along the x-axis in local frame A. Body B cannot rotate relative to body A.
/// The joint translation is zero when the local frame origins coincide in world space.</summary>
public struct PrismaticJointDef
{
    /// <summary>Base joint definition</summary>
    public JointDef base_ = new();
    /// <summary>Enable a linear spring along the prismatic joint axis</summary>
    public bool enableSpring = false;
    /// <summary>The spring stiffness Hertz, cycles per second</summary>
    public float hertz = 0;
    /// <summary>The spring damping ratio, non-dimensional</summary>
    public float dampingRatio = 0;
    /// <summary>The target translation for the joint in meters. The spring-damper will drive
    /// to this translation.</summary>
    public float targetTranslation = 0;
    /// <summary>Enable/disable the joint limit</summary>
    public bool enableLimit = false;
    /// <summary>The lower translation limit</summary>
    public float lowerTranslation = 0;
    /// <summary>The upper translation limit</summary>
    public float upperTranslation = 0;
    /// <summary>Enable/disable the joint motor</summary>
    public bool enableMotor = false;
    /// <summary>The maximum motor force, typically in newtons</summary>
    public float maxMotorForce = 0;
    /// <summary>The desired motor speed, typically in meters per second</summary>
    public float motorSpeed = 0;
    /// <summary>Used internally to detect a valid definition. DO NOT SET.</summary>
    internal int internalValue = Box2D.SECRET_COOKIE;
    public PrismaticJointDef() { }
}
/// <summary>Revolute joint definition
/// A point on body B is fixed to a point on body A. Allows relative rotation.</summary>
public struct RevoluteJointDef
{
    /// <summary>Base joint definition</summary>
    public JointDef base_ = new();
    /// <summary>The target angle for the joint in radians. The spring-damper will drive
    /// to this angle.</summary>
    public float targetAngle = 0;
    /// <summary>Enable a rotational spring on the revolute hinge axis</summary>
    public bool enableSpring = false;
    /// <summary>The spring stiffness Hertz, cycles per second</summary>
    public float hertz = 0;
    /// <summary>The spring damping ratio, non-dimensional</summary>
    public float dampingRatio = 0;
    /// <summary>A flag to enable joint limits</summary>
    public bool enableLimit = false;
    /// <summary>The lower angle for the joint limit in radians. Minimum of -0.99*pi radians.</summary>
    public float lowerAngle = 0;
    /// <summary>The upper angle for the joint limit in radians. Maximum of 0.99*pi radians.</summary>
    public float upperAngle = 0;
    /// <summary>A flag to enable the joint motor</summary>
    public bool enableMotor = false;
    /// <summary>The maximum motor torque, typically in newton-meters</summary>
    public float maxMotorTorque = 0;
    /// <summary>The desired motor speed in radians per second</summary>
    public float motorSpeed = 0;
    /// <summary>Used internally to detect a valid definition. DO NOT SET.</summary>
    internal int internalValue = Box2D.SECRET_COOKIE;
    public RevoluteJointDef() { }
}
/// <summary>Weld joint definition
/// Connects two bodies together rigidly. This constraint provides springs to mimic
/// soft-body simulation.</summary>
/// <remarks>The approximate solver in Box2D cannot hold many bodies together rigidly</remarks>
public struct WeldJointDef
{
    /// <summary>Base joint definition</summary>
    public JointDef base_ = new();
    /// <summary>Linear stiffness expressed as Hertz (cycles per second). Use zero for maximum stiffness.</summary>
    public float linearHertz = 0;
    /// <summary>Angular stiffness as Hertz (cycles per second). Use zero for maximum stiffness.</summary>
    public float angularHertz = 0;
    /// <summary>Linear damping ratio, non-dimensional. Use 1 for critical damping.</summary>
    public float linearDampingRatio = 0;
    /// <summary>Angular damping ratio, non-dimensional. Use 1 for critical damping.</summary>
    public float angularDampingRatio = 0;
    /// <summary>Used internally to detect a valid definition. DO NOT SET.</summary>
    internal int internalValue = Box2D.SECRET_COOKIE;
    public WeldJointDef() { }
}
/// <summary>Wheel joint definition
/// Body B is a wheel that may rotate freely and slide along the local x-axis in frame A.
/// The joint translation is zero when the local frame origins coincide in world space.</summary>
public struct WheelJointDef
{
    /// <summary>Base joint definition</summary>
    public JointDef base_ = new();
    /// <summary>Enable a linear spring along the local axis</summary>
    public bool enableSpring = false;
    /// <summary>Spring stiffness in Hertz</summary>
    public float hertz = 0;
    /// <summary>Spring damping ratio, non-dimensional</summary>
    public float dampingRatio = 0;
    /// <summary>Enable/disable the joint linear limit</summary>
    public bool enableLimit = false;
    /// <summary>The lower translation limit</summary>
    public float lowerTranslation = 0;
    /// <summary>The upper translation limit</summary>
    public float upperTranslation = 0;
    /// <summary>Enable/disable the joint rotational motor</summary>
    public bool enableMotor = false;
    /// <summary>The maximum motor torque, typically in newton-meters</summary>
    public float maxMotorTorque = 0;
    /// <summary>The desired motor speed in radians per second</summary>
    public float motorSpeed = 0;
    /// <summary>Used internally to detect a valid definition. DO NOT SET.</summary>
    internal int internalValue = Box2D.SECRET_COOKIE;
    public WheelJointDef() { }
}
/// <summary>The explosion definition is used to configure options for explosions. Explosions
/// consider shape geometry when computing the impulse.</summary>
public struct ExplosionDef
{
    /// <summary>Mask bits to filter shapes</summary>
    public ulong maskBits = Box2D.DEFAULT_MASK_BITS;
    /// <summary>The center of the explosion in world space</summary>
    public Vector2 position = new();
    /// <summary>The radius of the explosion</summary>
    public float radius = 0;
    /// <summary>The falloff distance beyond the radius. Impulse is reduced to zero at this distance.</summary>
    public float falloff = 0;
    /// <summary>Impulse per unit length. This applies an impulse according to the shape perimeter that
    /// is facing the explosion. Explosions only apply to circles, capsules, and polygons. This
    /// may be negative for implosions.</summary>
    public float impulsePerLength = 0;
    public ExplosionDef() { }
}
/// <summary>A begin touch event is generated when a shape starts to overlap a sensor shape.</summary>
public struct SensorBeginTouchEvent
{
    /// <summary>The id of the sensor shape</summary>
    public ShapeID sensorShapeId;
    /// <summary>The id of the shape that began touching the sensor shape</summary>
    public ShapeID visitorShapeId;
}
/// <summary>An end touch event is generated when a shape stops overlapping a sensor shape.
///	These include things like setting the transform, destroying a body or shape, or changing
///	a filter. You will also get an end event if the sensor or visitor are destroyed.
///	Therefore you should always confirm the shape id is valid using b2Shape_IsValid.</summary>
public struct SensorEndTouchEvent
{
    /// <summary>The id of the sensor shape</summary>
    /// <remarks>this shape may have been destroyed</remarks>
    public ShapeID sensorShapeId;
    /// <summary>The id of the shape that began touching the sensor shape</summary>
    /// <remarks>this shape may have been destroyed</remarks>
    public ShapeID visitorShapeId;
}
/// <summary>Sensor events are buffered in the world and are available
/// as begin/end overlap event arrays after the time step is complete.
/// Note: these may become invalid if bodies and/or shapes are destroyed</summary>
public struct SensorEvents
{
    /// <summary>Array of sensor begin touch events</summary>
    public List<SensorBeginTouchEvent> beginEvents;
    /// <summary>Array of sensor end touch events</summary>
    public List<SensorEndTouchEvent> endEvents;
}
/// <summary>A begin touch event is generated when two shapes begin touching.</summary>
public struct ContactBeginTouchEvent
{
    /// <summary>Id of the first shape</summary>
    public ShapeID shapeIdA;
    /// <summary>Id of the second shape</summary>
    public ShapeID shapeIdB;
    /// <summary>The transient contact id. This contact maybe destroyed automatically when the world is modified or simulated.</summary>
    public ContactID contactId;
}
/// <summary>An end touch event is generated when two shapes stop touching.
///	You will get an end event if you do anything that destroys contacts previous to the last
///	world step. These include things like setting the transform, destroying a body
///	or shape, or changing a filter or body type.</summary>
public struct ContactEndTouchEvent
{
    /// <summary>Id of the first shape</summary>
    /// <remarks>this shape may have been destroyed</remarks>
    public ShapeID shapeIdA;
    /// <summary>Id of the second shape</summary>
    /// <remarks>this shape may have been destroyed</remarks>
    public ShapeID shapeIdB;
    /// <summary>Id of the contact.</summary>
    /// <remarks>this shape may have been destroyed</remarks>
    public ContactID contactId;
}
/// <summary>A hit touch event is generated when two shapes collide with a speed faster than the hit speed threshold.
/// This may be reported for speculative contacts that have a confirmed impulse.</summary>
public struct ContactHitEvent
{
    /// <summary>Id of the first shape</summary>
    public ShapeID shapeIdA;
    /// <summary>Id of the second shape</summary>
    public ShapeID shapeIdB;
    /// <summary>Point where the shapes hit at the beginning of the time step.
    /// This is a mid-point between the two surfaces. It could be at speculative
    /// point where the two shapes were not touching at the beginning of the time step.</summary>
    public Vector2 point;
    /// <summary>Normal vector pointing from shape A to shape B</summary>
    public Vector2 normal;
    /// <summary>The speed the shapes are approaching. Always positive. Typically in meters per second.</summary>
    public float approachSpeed;
}
/// <summary>Contact events are buffered in the Box2D world and are available
/// as event arrays after the time step is complete.</summary>
/// <remarks>Note: these may become invalid if bodies and/or shapes are destroyed</remarks>
public struct ContactEvents
{
    /// <summary>Array of begin touch events</summary>
    public List<ContactBeginTouchEvent> beginEvents;
    /// <summary>Array of end touch events</summary>
    public List<ContactEndTouchEvent> endEvents;
    /// <summary>Array of hit events</summary>
    public List<ContactHitEvent> hitEvents;
}
/// <summary>Body move events triggered when a body moves.
/// Triggered when a body moves due to simulation. Not reported for bodies moved by the user.
/// This also has a flag to indicate that the body went to sleep so the application can also
/// sleep that actor/entity/object associated with the body.
/// On the other hand if the flag does not indicate the body went to sleep then the application
/// can treat the actor/entity/object associated with the body as awake.
/// This is an efficient way for an application to update game object transforms rather than
/// calling functions such as b2Body_GetTransform() because this data is delivered as a contiguous array
/// and it is only populated with bodies that have moved.</summary>
/// <remarks>If sleeping is disabled all dynamic and kinematic bodies will trigger move events.</remarks>
public struct BodyMoveEvent
{
    public object userData;
    public Transform transform;
    public BodyID bodyId;
    public bool fellAsleep;
}
/// <summary>Body events are buffered in the Box2D world and are available
/// as event arrays after the time step is complete.</summary>
/// <remarks>Note: this data becomes invalid if bodies are destroyed</remarks>
public struct BodyEvents
{
    /// <summary>Array of move events</summary>
    public List<BodyMoveEvent> moveEvents;
}
/// <summary>Joint events report joints that are awake and have a force and/or torque exceeding the threshold
/// The observed forces and torques are not returned for efficiency reasons.</summary>
public struct JointEvent
{
    /// <summary>The joint id</summary>
    public JointID jointId;
    /// <summary>The user data from the joint for convenience</summary>
    public object userData;
}
/// <summary>Joint events are buffered in the world and are available
/// as event arrays after the time step is complete.</summary>
/// <remarks>Note: this data becomes invalid if joints are destroyed</remarks>
public struct JointEvents
{
    /// <summary>Array of events</summary>
    public List<JointEvent> jointEvents;
}
/// <summary>The contact data for two shapes. By convention the manifold normal points
/// from shape A to shape B.
/// @see b2Shape_GetContactData() and b2Body_GetContactData()</summary>
public struct ContactData
{
    public ContactID contactId;
    public ShapeID shapeIdA;
    public ShapeID shapeIdB;
    public Manifold manifold;
}
/// <summary>Prototype for a contact filter callback.
/// This is called when a contact pair is considered for collision. This allows you to
/// perform custom logic to prevent collision between shapes. This is only called if
/// one of the two shapes has custom filtering enabled.<br/>
/// Notes:<br/>
/// - this function must be thread-safe<br/>
/// - this is only called if one of the two shapes has enabled custom filtering<br/>
/// - this may be called for awake dynamic bodies and sensors</summary>
/// <returns>false if you want to disable the collision</returns>
/// <remarks>Do not attempt to modify the world inside this callback</remarks>
public delegate bool CustomFilterFcn(ShapeID shapeIdA, ShapeID shapeIdB, object context);
/// <summary>
/// Prototype for a pre-solve callback.
/// This is called after a contact is updated. This allows you to inspect a
/// contact before it goes to the solver. If you are careful, you can modify the
/// contact manifold (e.g. modify the normal).<br/>
/// Notes:<br/>
/// - this function must be thread-safe<br/>
/// - this is only called if the shape has enabled pre-solve events<br/>
/// - this is called only for awake dynamic bodies<br/>
/// - this is not called for sensors<br/>
/// - the supplied manifold has impulse values from the previous step</summary>
/// <returns>false if you want to disable the contact this step</returns>
/// <remarks>Do not attempt to modify the world inside this callback</remarks>
public delegate bool PreSolveFcn(ShapeID shapeIdA, ShapeID shapeIdB, Vector2 point, Vector2 normal, object context);
/// <summary>Prototype callback for overlap queries.
/// Called for each shape found in the query.
/// @see b2World_OverlapABB</summary>
/// <returns>false to terminate the query.</returns>
public delegate bool OverlapResultFcn(ShapeID shapeId, object context);
/// <summary>Prototype callback for ray and shape casts.
/// Called for each shape found in the query. You control how the ray cast
/// proceeds by returning a float:<br/>
/// return -1: ignore this shape and continue<br/>
/// return 0: terminate the ray cast<br/>
/// return fraction: clip the ray to this point<br/>
/// return 1: don't clip the ray and continue<br/>
/// A cast with initial overlap will return a zero fraction and a zero normal.<br/>
/// @see b2World_CastRay</summary>
/// <param name="shapeId">the shape hit by the ray</param>
/// <param name="point">the point of initial intersection</param>
/// <param name="normal">the normal vector at the point of intersection, zero for a shape cast with initial overlap</param>
/// <param name="fraction">the fraction along the ray at the point of intersection, zero for a shape cast with initial overlap</param>
/// <param name="context">the user context</param>
/// <returns>-1 to filter, 0 to terminate, fraction to clip the ray for closest hit, 1 to continue</returns>
public delegate float CastResultFcn(ShapeID shapeId, Vector2 point, Vector2 normal, float fraction, object context);
/// <summary>Used to collect collision planes for character movers.</summary>
/// <returns>true to continue gathering planes.</returns>
public delegate bool PlaneResultFcn(ShapeID shapeId, ref PlaneResult plane, object context);
/// <summary>These colors are used for debug draw and mostly match the named SVG colors.
/// See https://www.rapidtables.com/web/color/index.html
/// https://johndecember.com/html/spec/colorsvg.html
/// https://upload.wikimedia.org/wikipedia/commons/2/2b/SVG_Recognized_color_keyword_names.svg</summary>
public enum HexColor
{
    AliceBlue = 0xF0F8FF,
    AntiqueWhite = 0xFAEBD7,
    Aqua = 0x00FFFF,
    Aquamarine = 0x7FFFD4,
    Azure = 0xF0FFFF,
    Beige = 0xF5F5DC,
    Bisque = 0xFFE4C4,
    Black = 0x000000,
    BlanchedAlmond = 0xFFEBCD,
    Blue = 0x0000FF,
    BlueViolet = 0x8A2BE2,
    Brown = 0xA52A2A,
    Burlywood = 0xDEB887,
    CadetBlue = 0x5F9EA0,
    Chartreuse = 0x7FFF00,
    Chocolate = 0xD2691E,
    Coral = 0xFF7F50,
    CornflowerBlue = 0x6495ED,
    Cornsilk = 0xFFF8DC,
    Crimson = 0xDC143C,
    Cyan = 0x00FFFF,
    DarkBlue = 0x00008B,
    DarkCyan = 0x008B8B,
    DarkGoldenRod = 0xB8860B,
    DarkGray = 0xA9A9A9,
    DarkGreen = 0x006400,
    DarkKhaki = 0xBDB76B,
    DarkMagenta = 0x8B008B,
    DarkOliveGreen = 0x556B2F,
    DarkOrange = 0xFF8C00,
    DarkOrchid = 0x9932CC,
    DarkRed = 0x8B0000,
    DarkSalmon = 0xE9967A,
    DarkSeaGreen = 0x8FBC8F,
    DarkSlateBlue = 0x483D8B,
    DarkSlateGray = 0x2F4F4F,
    DarkTurquoise = 0x00CED1,
    DarkViolet = 0x9400D3,
    DeepPink = 0xFF1493,
    DeepSkyBlue = 0x00BFFF,
    DimGray = 0x696969,
    DodgerBlue = 0x1E90FF,
    FireBrick = 0xB22222,
    FloralWhite = 0xFFFAF0,
    ForestGreen = 0x228B22,
    Fuchsia = 0xFF00FF,
    Gainsboro = 0xDCDCDC,
    GhostWhite = 0xF8F8FF,
    Gold = 0xFFD700,
    GoldenRod = 0xDAA520,
    Gray = 0x808080,
    Green = 0x008000,
    GreenYellow = 0xADFF2F,
    HoneyDew = 0xF0FFF0,
    HotPink = 0xFF69B4,
    IndianRed = 0xCD5C5C,
    Indigo = 0x4B0082,
    Ivory = 0xFFFFF0,
    Khaki = 0xF0E68C,
    Lavender = 0xE6E6FA,
    LavenderBlush = 0xFFF0F5,
    LawnGreen = 0x7CFC00,
    LemonChiffon = 0xFFFACD,
    LightBlue = 0xADD8E6,
    LightCoral = 0xF08080,
    LightCyan = 0xE0FFFF,
    LightGoldenRodYellow = 0xFAFAD2,
    LightGray = 0xD3D3D3,
    LightGreen = 0x90EE90,
    LightPink = 0xFFB6C1,
    LightSalmon = 0xFFA07A,
    LightSeaGreen = 0x20B2AA,
    LightSkyBlue = 0x87CEFA,
    LightSlateGray = 0x778899,
    LightSteelBlue = 0xB0C4DE,
    LightYellow = 0xFFFFE0,
    Lime = 0x00FF00,
    LimeGreen = 0x32CD32,
    Linen = 0xFAF0E6,
    Magenta = 0xFF00FF,
    Maroon = 0x800000,
    MediumAquaMarine = 0x66CDAA,
    MediumBlue = 0x0000CD,
    MediumOrchid = 0xBA55D3,
    MediumPurple = 0x9370DB,
    MediumSeaGreen = 0x3CB371,
    MediumSlateBlue = 0x7B68EE,
    MediumSpringGreen = 0x00FA9A,
    MediumTurquoise = 0x48D1CC,
    MediumVioletRed = 0xC71585,
    MidnightBlue = 0x191970,
    MintCream = 0xF5FFFA,
    MistyRose = 0xFFE4E1,
    Moccasin = 0xFFE4B5,
    NavajoWhite = 0xFFDEAD,
    Navy = 0x000080,
    OldLace = 0xFDF5E6,
    Olive = 0x808000,
    OliveDrab = 0x6B8E23,
    Orange = 0xFFA500,
    OrangeRed = 0xFF4500,
    Orchid = 0xDA70D6,
    PaleGoldenRod = 0xEEE8AA,
    PaleGreen = 0x98FB98,
    PaleTurquoise = 0xAFEEEE,
    PaleVioletRed = 0xDB7093,
    PapayaWhip = 0xFFEFD5,
    PeachPuff = 0xFFDAB9,
    Peru = 0xCD853F,
    Pink = 0xFFC0CB,
    Plum = 0xDDA0DD,
    PowderBlue = 0xB0E0E6,
    Purple = 0x800080,
    RebeccaPurple = 0x663399,
    Red = 0xFF0000,
    RosyBrown = 0xBC8F8F,
    RoyalBlue = 0x4169E1,
    SaddleBrown = 0x8B4513,
    Salmon = 0xFA8072,
    SandyBrown = 0xF4A460,
    SeaGreen = 0x2E8B57,
    SeaShell = 0xFFF5EE,
    Sienna = 0xA0522D,
    Silver = 0xC0C0C0,
    SkyBlue = 0x87CEEB,
    SlateBlue = 0x6A5ACD,
    SlateGray = 0x708090,
    Snow = 0xFFFAFA,
    SpringGreen = 0x00FF7F,
    SteelBlue = 0x4682B4,
    Tan = 0xD2B48C,
    Teal = 0x008080,
    Thistle = 0xD8BFD8,
    Tomato = 0xFF6347,
    Turquoise = 0x40E0D0,
    Violet = 0xEE82EE,
    Wheat = 0xF5DEB3,
    White = 0xFFFFFF,
    WhiteSmoke = 0xF5F5F5,
    Yellow = 0xFFFF00,
    YellowGreen = 0x9ACD32,

    Box2DRed = 0xDC3132,
    Box2DBlue = 0x30AEBF,
    Box2DGreen = 0x8CC924,
    Box2DYellow = 0xFFEE8C
}
/// <summary>This struct holds callbacks you can implement to draw a Box2D world.
/// This structure should be zero initialized.</summary>
public partial class DebugDraw
{
    /// <summary>Draw a closed polygon provided in CCW order.</summary>
    public Action<Vector2[], HexColor, object> DrawPolygonFcn;
    /// <summary>Draw a solid closed polygon provided in CCW order.</summary>
    public Action<Transform, Vector2[], float, HexColor, object> DrawSolidPolygonFcn;
    /// <summary>Draw a circle.</summary>
    public Action<Vector2, float, HexColor, object> DrawCircleFcn;
    /// <summary>Draw a solid circle.</summary>
    public Action<Transform, float, HexColor, object> DrawSolidCircleFcn;
    /// <summary>Draw a solid capsule.</summary>
    public Action<Vector2, Vector2, float, HexColor, object> DrawSolidCapsuleFcn;
    /// <summary>Draw a line segment.</summary>
    public Action<Vector2, Vector2, HexColor, object> DrawSegmentFcn;
    /// <summary>Draw a transform. Choose your own length scale.</summary>
    public Action<Transform, object> DrawTransformFcn;
    /// <summary>Draw a point.</summary>
    public Action<Vector2, float, HexColor, object> DrawPointFcn;
    /// <summary>Draw a string in world space</summary>
    public Action<Vector2, string, HexColor, object> DrawStringFcn;
    /// <summary>Bounds to use if restricting drawing to a rectangular region</summary>
    public AABB drawingBounds = new(new(-float.MaxValue, -float.MaxValue), new(float.MaxValue, float.MaxValue));
    /// <summary>Option to draw shapes</summary>
    public bool drawShapes = true;
    /// <summary>Option to draw joints</summary>
    public bool drawJoints;
    /// <summary>Option to draw additional information for joints</summary>
    public bool drawJointExtras;
    /// <summary>Option to draw the bounding boxes for shapes</summary>
    public bool drawBounds;
    /// <summary>Option to draw the mass and center of mass of dynamic bodies</summary>
    public bool drawMass;
    /// <summary>Option to draw body names</summary>
    public bool drawBodyNames;
    /// <summary>Option to draw contact points</summary>
    public bool drawContacts;
    /// <summary>Option to visualize the graph coloring used for contacts and joints</summary>
    public bool drawGraphColors;
    /// <summary>Option to draw contact normals</summary>
    public bool drawContactNormals;
    /// <summary>Option to draw contact normal impulses</summary>
    public bool drawContactImpulses;
    /// <summary>Option to draw contact feature ids</summary>
    public bool drawContactFeatures;
    /// <summary>Option to draw contact friction impulses</summary>
    public bool drawFrictionImpulses;
    /// <summary>Option to draw islands as bounding boxes</summary>
    public bool drawIslands;
    /// <summary>User context that is passed as an argument to drawing callback functions</summary>
    public object context;
}