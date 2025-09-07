using System;
using System.Diagnostics;
using System.Reflection;

namespace Box2D;

[Flags] public enum BodyFlags
{
    /// <summary>This body has fixed translation along the x-axis</summary>
    LockLinearX = 1,
    /// <summary>This body has fixed translation along the y-axis</summary>
    LockLinearY = 2,
    /// <summary>This body has fixed rotation</summary>
    LockAngularZ = 4,
    /// <summary>This flag is used for debug draw</summary>
    IsFast = 8,
    /// <summary>This dynamic body does a final CCD pass against all body types, but not other bullets</summary>
    IsBullet = 0x10,
    /// <summary>This body was speed capped in the current time step</summary>
    IsSpeedCapped = 0x20,
    /// <summary>This body had a time of impact event in the current time step</summary>
    HadTimeOfImpact = 0x40,
    /// <summary>This body has no limit on angular velocity</summary>
    AllowFastRotation = 0x80,
    /// <summary>This body need's to have its AABB increased</summary>
    EnlargeBounds = 0x100,
    /// <summary>This body is dynamic so the solver should write to it.
    /// This prevents writing to kinematic bodies that causes a multithreaded sharing
    /// cache coherence problem even when the values are not changing.
    /// Used for b2BodyState flags.</summary>
    Dynamic = 0x200,
    /// <summary>All lock flags</summary>
    AllLocks = LockLinearX | LockLinearY | LockAngularZ,
}
public class Body
{
    public string name;
    public object userData;
    ///<summary>index of solver set stored in World<br/>
    /// may be B2_NULL_INDEX</summary>
    public int setIndex;

    ///<summary>body sim and state index within set<br/>
    /// may be B2_NULL_INDEX</summary>
    public int localIndex;

    ///<summary>[31 : contactId | 1 : edgeIndex]</summary>
    public int headContactKey;
    public int contactCount;

    ///<summary>todo maybe move this to the body sim</summary>
    public int headShapeId;
    public int shapeCount;

    public int headChainId;

    ///<summary>[31 : jointId | 1 : edgeIndex]</summary>
    public int headJointKey;
    public int jointCount;

    ///<summary>All enabled dynamic and kinematic bodies are in an island.</summary>
    public int islandId;

    ///<summary>doubly-linked island list</summary>
    public int islandPrev;
    public int islandNext;

    public float mass;

    ///<summary>Rotational inertia about the center of mass.</summary>
    public float inertia;

    public float sleepThreshold;
    public float sleepTime;

    ///<summary>this is used to adjust the fellAsleep flag in the body move array</summary>
    public int bodyMoveIndex;

    public int id;

    ///<summary>BodyFlags
    ///Important flags: locking, dynamic</summary>
    public BodyFlags flags;

    public BodyType type;

    ///<summary>This is monotonically advanced when a body is allocated in this slot<br/>
    /// Used to check for invalid BodyId</summary>
    public ushort generation;

    ///<summary>todo move into flags</summary>
    public bool enableSleep;
}
/// <summary>Body State<br/>
/// The body state is designed for fast conversion to and from SIMD via scatter-gather.
/// Only awake dynamic and kinematic bodies have a body state->
/// This is used in the performance critical constraint solver<br/>
///
/// The solver operates on the body state-> The body state array does not hold static bodies. Static bodies are shared
/// across worker threads. It would be okay to read their states, but writing to them would cause cache thrashing across
/// workers, even if the values don't change.
/// This causes some trouble when computing anchors. I rotate joint anchors using the body rotation every sub-step. For static
/// bodies the anchor doesn't rotate. Body A or B could be static and this can lead to lots of branching. This branching
/// should be minimized.<br/>
///
/// Solution 1:<br/>
/// Use delta rotations. This means anchors need to be prepared in world space. The delta rotation for static bodies will be
/// identity using a dummy state-> Base separation and angles need to be computed. Manifolds will be behind a frame, but that
/// is probably best if bodies move fast.<br/>
///
/// Solution 2:<br/>
/// Use full rotation. The anchors for static bodies will be in world space while the anchors for dynamic bodies will be in local
/// space. Potentially confusing and bug prone.<br/>
///
/// Note:<br/>
/// I rotate joint anchors each sub-step but not contact anchors. Joint stability improves a lot by rotating joint anchors
/// according to substep progress. Contacts have reduced stability when anchors are rotated during substeps, especially for
/// round shapes.</summary>
public struct BodyState
{
    public Vector2 linearVelocity = Vector2.Zero;
    public float angularVelocity = 0;
    /// <summary>BodyFlags</summary>
    public BodyFlags flags = 0;
    /// <summary>Using delta position reduces round-off error far from the origin</summary>
    public Vector2 deltaPosition = Vector2.Zero;
    /// <summary>Using delta rotation because I cannot access the full rotation on static bodies in
    /// the solver and must use zero delta rotation for static bodies (c,s) = (1,0)</summary>
    public Rotation deltaRotation = Rotation.Identity;
    public BodyState() { }
    public void LimitVelocity(float maxLinearSpeed)
    {
        float v2 = linearVelocity.LengthSquared();
        if (v2 > maxLinearSpeed * maxLinearSpeed) linearVelocity *= maxLinearSpeed / MathF.Sqrt(v2);
    }
}
/// <summary>Body simulation data used for integration of position and velocity
/// Transform data used for collision and solver preparation.</summary>
public record BodySim
{
    /// <summary>transform for body origin</summary>
    public Transform transform;

    /// <summary>center of mass position in world space</summary>
    public Vector2 center;

    /// <summary>previous rotation and COM for TOI</summary>
    public Rotation rotation0;
    public Vector2 center0;

    /// <summary>location of center of mass relative to the body origin</summary>
    public Vector2 localCenter;

    public Vector2 force;
    public float torque;

    /// <summary>inverse inertia</summary>
    public float invMass;
    public float invInertia;

    public float minExtent;
    public float maxExtent;
    public float linearDamping;
    public float angularDamping;
    public float gravityScale;

    /// <summary>Index of Body</summary>
    public int bodyId;

    /// <summary>BodyFlags</summary>
    public BodyFlags flags;
    public Sweep MakeSweep() => new() { c1 = center0, c2 = center, q1 = rotation0, q2 = transform.q, localCenter = localCenter };
}
public unsafe partial class World
{
    public Body GetBodyFullID(BodyID bodyID)
    {
        Debug.Assert(API.BodyAPI.Body_IsValid(bodyID));
        return bodies[bodyID.index1 - 1];
    }
    public Transform GetBodyTransformQuick(Body body) => solverSets[body.setIndex].bodySims[body.localIndex].transform;
    public Transform GetBodyTransform(int bodyId) => GetBodyTransformQuick(bodies[bodyId]);
    public BodyID MakeBodyID(int bodyId) => new() { index1 = bodyId + 1, world0 = this, generation = bodies[bodyId].generation };
    public BodySim GetBodySim(Body body) => solverSets[body.setIndex].bodySims[body.localIndex];
    public unsafe BodyState* GetBodyState(Body body) => body.setIndex == (int)SetType.Awake ? solverSets[(int)SetType.Awake].bodyStates.Data + body.localIndex : (BodyState*)null;
    public void CreateIslandForBody(int setIndex, Body body)
    {
        Debug.Assert(body.islandId == -1);
        Debug.Assert(body.islandPrev == -1);
        Debug.Assert(body.islandNext == -1);
        Debug.Assert(setIndex != (int)SetType.Disabled);
        Island island = CreateIsland(setIndex);
        body.islandId = island.islandId;
        island.headBody = body.id;
        island.tailBody = body.id;
        island.bodyCount = 1;
    }
    public void RemoveBodyFromIsland(Body body)
    {
        if (body.islandId == -1)
        {
            Debug.Assert(body.islandPrev == -1);
            Debug.Assert(body.islandNext == -1);
            return;
        }
        int islandId = body.islandId;
        Island island = islands[islandId];
        if (body.islandPrev != -1) bodies[body.islandPrev].islandNext = body.islandNext;
        if (body.islandNext != -1) bodies[body.islandNext].islandPrev = body.islandPrev;
        Debug.Assert(island.bodyCount > 0);
        island.bodyCount--;
        bool islandDestroyed = false;
        if (island.headBody == body.id)
        {
            island.headBody = body.islandNext;
            if (island.headBody == -1)
            {
                Debug.Assert(island.tailBody == body.id);
                Debug.Assert(island.bodyCount == 0);
                Debug.Assert(island.contactCount == 0);
                Debug.Assert(island.jointCount == 0);
                DestroyIsland(island.islandId);
                islandDestroyed = true;
            }
        }
        else if (island.tailBody == body.id) island.tailBody = body.islandPrev;
        if (!islandDestroyed) ValidateIsland(islandId);
        body.islandId = -1;
        body.islandPrev = -1;
        body.islandNext = -1;
    }
    public void DestroyBodyContacts(Body body, bool wakeBodies)
    {
        int edgeKey = body.headContactKey;
        while (edgeKey != -1)
        {
            int contactId = edgeKey >> 1;
            int edgeIndex = edgeKey & 1;
            Contact contact = contacts[contactId];
            edgeKey = edgeIndex == 1 ? contact.edge1.nextKey : contact.edge0.nextKey;
            DestroyContact(contact, wakeBodies);
        }
        ValidateSolverSets();
    }
    public bool WakeBody(Body body)
    {
        if (body.setIndex >= (int)SetType.FirstSleeping)
        {
            WakeSolverSet(body.setIndex);
            ValidateSolverSets();
            return true;
        }
        return false;
    }
    public void UpdateBodyMassData(Body body)
    {
        BodySim bodySim = GetBodySim(body);
        body.mass = 0;
        body.inertia = 0;
        bodySim.invMass = 0;
        bodySim.invInertia = 0;
        bodySim.localCenter = Vector2.Zero;
        bodySim.minExtent = Box2D.Huge;
        bodySim.maxExtent = 0;
        if (body.type != BodyType.Dynamic)
        {
            bodySim.center = bodySim.transform.p;
            bodySim.center0 = bodySim.center;

            //Added in C# (we need the local center for correct separation)
            /*{
                int shapeId = body.headShapeId;
                float mass = 0;
                while (shapeId != -1)
                {
                    Shape s = shapes[shapeId];
                    shapeId = s.nextShapeId;
                    MassData massData = s.ComputeMass();
                    mass += massData.mass;
                    bodySim.localCenter = Vector2.MulAdd(bodySim.localCenter, massData.mass, massData.center);
                }
                if (mass > 0) bodySim.localCenter = 1 / mass * bodySim.localCenter;
            }*/

            if (body.type == BodyType.Kinematic)
            {
                int shapeId = body.headShapeId;
                while (shapeId != -1)
                {
                    Shape s = shapes[shapeId];
                    ShapeExtent extent = s.ComputeExtent(Vector2.Zero);
                    bodySim.minExtent = Math.Min(bodySim.minExtent, extent.minExtent);
                    bodySim.maxExtent = Math.Max(bodySim.maxExtent, extent.maxExtent);
                    shapeId = s.nextShapeId;
                }
            }
            return;
        }
        int shapeCount = body.shapeCount;
        MassData[] masses = new MassData[shapeCount];
        Vector2 localCenter = Vector2.Zero;
        int shapeIndex = 0;
        {
            int shapeId = body.headShapeId;
            while (shapeId != -1)
            {
                Shape s = shapes[shapeId];
                shapeId = s.nextShapeId;
                if (s.density == 0) { masses[shapeIndex] = new(); continue; }
                MassData massData = s.ComputeMass();
                body.mass += massData.mass;
                localCenter = Vector2.MulAdd(localCenter, massData.mass, massData.center);
                masses[shapeIndex] = massData;
                shapeIndex++;
            }
            if (body.mass > 0)
            {
                bodySim.invMass = 1 / body.mass;
                localCenter = bodySim.invMass * localCenter;
            }
            for (shapeIndex = 0; shapeIndex < shapeCount; shapeIndex++)
            {
                MassData massData = masses[shapeIndex];
                if (massData.mass == 0) continue;
                Vector2 offset = localCenter - massData.center;
                float inertia = massData.rotationalInertia + massData.mass * Vector2.Dot(offset, offset);
                body.inertia += inertia;
            }
            masses = null;
            Debug.Assert(body.inertia >= 0);
            if (body.inertia > 0) bodySim.invInertia = 1 / body.inertia;
            else { body.inertia = 0; bodySim.invInertia = 0; }
            Vector2 oldCenter = bodySim.center;
            bodySim.localCenter = localCenter;
            bodySim.center = bodySim.transform.TransformPoint(bodySim.localCenter);
            bodySim.center0 = bodySim.center;
            BodyState* state = GetBodyState(body);
            if (state != null)
            {
                Vector2 deltaLinear = Vector2.CrossSV(state->angularVelocity, bodySim.center - oldCenter);
                state->linearVelocity += deltaLinear;
            }
            shapeId = body.headShapeId;
            while (shapeId != -1)
            {
                Shape s = shapes[shapeId];
                ShapeExtent extent = s.ComputeExtent(localCenter);
                bodySim.minExtent = Math.Min(bodySim.minExtent, extent.minExtent);
                bodySim.maxExtent = Math.Max(bodySim.maxExtent, extent.maxExtent);
                shapeId = s.nextShapeId;
            }
        }
    }
    public bool ShouldBodiesCollide(Body bodyA, Body bodyB)
    {
        if (bodyA.type != BodyType.Dynamic && bodyB.type != BodyType.Dynamic) return false;
        int jointKey, otherBodyId;
        if (bodyA.jointCount < bodyB.jointCount) { jointKey = bodyA.headJointKey; otherBodyId = bodyB.id; }
        else { jointKey = bodyB.headJointKey; otherBodyId = bodyA.id; }
        while (jointKey != -1)
        {
            int jointId = jointKey >> 1;
            int edgeIndex = jointKey & 1;
            int otherEdgeIndex = edgeIndex ^ 1;
            Joint joint = joints[jointId];
            if (!joint.collideConnected && (otherEdgeIndex == 1 ? joint.edge1.bodyId : joint.edge0.bodyId) == otherBodyId)
                return false;
            jointKey = edgeIndex == 1 ? joint.edge1.nextKey : joint.edge0.nextKey;
        }
        return true;
    }
}