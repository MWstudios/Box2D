using System;
using System.Diagnostics;

namespace Box2D;

[Flags] public enum ContactFlags
{
    /// <summary>Set when the solid shapes are touching.</summary>
    Touching = 0x00000001,

    /// <summary>Contact has a hit event</summary>
    HitEvent = 0x00000002,

    /// <summary>This contact wants contact events</summary>
    EnableContactEvents = 0x00000004,
};

/// <summary>A contact edge is used to connect bodies and contacts together
/// in a contact graph where each body is a node and each contact
/// is an edge. A contact edge belongs to a doubly linked list
/// maintained in each attached body. Each contact has two contact
/// edges, one for each attached body.</summary>
public struct ContactEdge
{
    public int bodyId;
    public int prevKey;
    public int nextKey;
}

/// <summary>Cold contact data. Used as a persistent handle and for persistent island
/// connectivity.</summary>
public class Contact
{
    /// <summary>index of simulation set stored in World
    /// B2_NULL_INDEX when slot is free</summary>
    public int setIndex;

    /// <summary>index into the constraint graph color array
    /// B2_NULL_INDEX for non-touching or sleeping contacts
    /// B2_NULL_INDEX when slot is free</summary>
    public int colorIndex;

    /// <summary>contact index within set or graph color
    /// B2_NULL_INDEX when slot is free</summary>
    public int localIndex;

    public ContactEdge edge0, edge1;
    public int shapeIdA;
    public int shapeIdB;
    public int contactId;

    /// <summary>A contact only belongs to an island if touching, otherwise B2_NULL_INDEX.</summary>
    public int islandPrev;
    public int islandNext;
    public int islandId;

    /// <summary>ContactFlags</summary>
    public ContactFlags flags;

    /// <summary>This is monotonically advanced when a contact is allocated in this slot
    /// Used to check for invalid ContactId</summary>
    public uint generation;
}

/// <summary>Shifted to be distinct from ContactFlags
[Flags] public enum ContactSimFlags
{
    /// <summary>Set when the shapes are touching</summary>
    Touching = 0x00010000,

    /// <summary>This contact no longer has overlapping AABBs</summary>
    Disjoint = 0x00020000,

    /// <summary>This contact started touching</summary>
    StartedTouching = 0x00040000,

    /// <summary>This contact stopped touching</summary>
    StoppedTouching = 0x00080000,

    /// <summary>This contact has a hit event</summary>
    EnableHitEvent = 0x00100000,

    /// <summary>This contact wants pre-solve events</summary>
    EnablePreSolveEvents = 0x00200000,
};

/// <summary>The class manages contact between two shapes. A contact exists for each overlapping
/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
/// that has no contact points.</summary>
public record class ContactSim
{
    public int contactId;

#if B2_VALIDATE
    public int bodyIdA;
    public int bodyIdB;
#endif

    public int bodySimIndexA;
    public int bodySimIndexB;

    public int shapeIdA;
    public int shapeIdB;

    public float invMassA;
    public float invIA;

    public float invMassB;
    public float invIB;

    public Manifold manifold;

    /// <summary>Mixed friction and restitution</summary>
    public float friction;
    public float restitution;
    public float rollingResistance;
    public float tangentSpeed;

    /// <summary>ContactSimFlags</summary>
    public ContactSimFlags simFlags;

    public SimplexCache cache;
}
public partial class World
{
    static ulong B2_SHAPE_PAIR_KEY(ulong K1, ulong K2) => K1 < K2 ? K1 << 32 | K2 : K2 << 32 | K1;
    public Contact GetContactFullId(ContactID contactId)
    {
        int id = contactId.index1 - 1;
        Contact contact = contacts[id];
        Debug.Assert(contact.contactId == id && contact.generation == contactId.generation);
        return contact;
    }
    public void CreateContact(Shape shapeA, Shape shapeB)
    {
        ShapeType type1 = shapeA.type, type2 = shapeB.type;
        Debug.Assert(0 <= type1 && (int)type1 < ContactRegister.s_registers.Length);
        Debug.Assert(0 <= type2 && (int)type2 < ContactRegister.s_registers.Length);
        if (ContactRegister.s_registers[(int)type1][(int)type2].fcn == null) return;
        if (!ContactRegister.s_registers[(int)type1][(int)type2].primary)
        {
            CreateContact(shapeB, shapeA); return;
        }
        Body bodyA = bodies[shapeA.bodyId], bodyB = bodies[shapeB.bodyId];
        Debug.Assert(bodyA.setIndex != (int)SetType.Disabled && bodyB.setIndex != (int)SetType.Disabled);
        Debug.Assert(bodyA.setIndex != (int)SetType.Static || bodyB.setIndex != (int)SetType.Static);
        int setIndex;
        if (bodyA.setIndex == (int)SetType.Awake || bodyB.setIndex == (int)SetType.Awake)
            setIndex = (int)SetType.Awake;
        else setIndex = (int)SetType.Disabled;
        SolverSet set = solverSets[setIndex];
        int contactId = contactIdPool.AllocId();
        if (contactId == contacts.Count)
            contacts.Add(new());
        int shapeIdA = shapeA.id, shapeIdB = shapeB.id;
        Contact contact = contacts[contactId];
        contact.contactId = contactId;
        contact.generation++;
        contact.setIndex = setIndex;
        contact.colorIndex = -1;
        contact.localIndex = set.contactSims.Count;
        contact.islandId = -1;
        contact.islandPrev = -1;
        contact.islandNext = -1;
        contact.shapeIdA = shapeIdA;
        contact.shapeIdB = shapeIdB;
        contact.flags = 0;
        Debug.Assert(shapeA.sensorIndex == -1 && shapeB.sensorIndex == -1);
        if (shapeA.enableContactEvents || shapeB.enableContactEvents)
            contact.flags |= ContactFlags.EnableContactEvents;
        {
            contact.edge0.bodyId = shapeA.bodyId;
            contact.edge0.prevKey = -1;
            contact.edge0.nextKey = bodyA.headContactKey;
            int keyA = contactId << 1;
            int headContactKey = bodyA.headContactKey;
            if (headContactKey != -1)
            {
                Contact headContact = contacts[headContactKey >> 1];
                if ((headContactKey & 1) == 1) headContact.edge1.prevKey = keyA;
                else headContact.edge0.prevKey = keyA;
            }
            bodyA.headContactKey = keyA;
            bodyA.contactCount++;
        }
        {
            contact.edge1.bodyId = shapeB.bodyId;
            contact.edge1.prevKey = -1;
            contact.edge1.nextKey = bodyB.headContactKey;
            int keyB = (contactId << 1) | 1;
            int headContactKey = bodyB.headContactKey;
            if (headContactKey != -1)
            {
                Contact headContact = contacts[headContactKey >> 1];
                if ((headContactKey & 1) == 1) headContact.edge1.prevKey = keyB;
                else headContact.edge0.prevKey = keyB;
            }
            bodyB.headContactKey = keyB;
            bodyB.contactCount++;
        }
        ulong pairKey = B2_SHAPE_PAIR_KEY((ulong)shapeIdA, (ulong)shapeIdB);
        broadPhase.pairSet.Add(pairKey);
        ContactSim contactSim = new(); set.contactSims.Add(contactSim);
        contactSim.contactId = contactId;
#if B2_VALIDATE
        contactSim.bodyIdA = shapeA.bodyId;
        contactSim.bodyIdB = shapeB.bodyId;
#endif
        contactSim.bodySimIndexA = -1;
        contactSim.bodySimIndexB = -1;
        contactSim.invMassA = 0;
        contactSim.invIA = 0;
        contactSim.invMassB = 0;
        contactSim.invIB = 0;
        contactSim.shapeIdA = shapeIdA;
        contactSim.shapeIdB = shapeIdB;
        contactSim.cache = new();
        contactSim.manifold = new();
        contactSim.friction = frictionCallback(shapeA.material.friction, shapeA.material.userMaterialId, shapeB.material.friction, shapeB.material.userMaterialId);
        contactSim.restitution = restitutionCallback(shapeA.material.restitution, shapeA.material.userMaterialId, shapeB.material.restitution, shapeB.material.userMaterialId);
        contactSim.tangentSpeed = 0;
        contactSim.simFlags = 0;
        if (shapeA.enablePreSolveEvents || shapeB.enablePreSolveEvents)
        {
            contactSim.simFlags |= ContactSimFlags.EnablePreSolveEvents;
        }
    }
    public void DestroyContact(Contact contact, bool wakeBodies)
    {
        ulong pairKey = B2_SHAPE_PAIR_KEY((ulong)contact.shapeIdA, (ulong)contact.shapeIdB);
        broadPhase.pairSet.Remove(pairKey);
        int bodyIdA = contact.edge0.bodyId, bodyIdB = contact.edge1.bodyId;
        Body bodyA = bodies[bodyIdA], bodyB = bodies[bodyIdB];
        bool touching = contact.flags.HasFlag(ContactFlags.Touching);
        if (touching && contact.flags.HasFlag(ContactFlags.EnableContactEvents))
        {
            Shape shapeA = shapes[contact.shapeIdA], shapeB = shapes[contact.shapeIdB];
            ShapeID shapeIdA = new() { index1 = shapeA.id + 1, world0 = this, generation = shapeA.generation };
            ShapeID shapeIdB = new() { index1 = shapeB.id + 1, world0 = this, generation = shapeB.generation };
            if (endEventArrayIndex == 1) contactEndEvents1.Add(new ContactEndTouchEvent
                {
                    shapeIdA = shapeIdA,
                    shapeIdB = shapeIdB,
                    contactId = new() { index1 = contact.contactId + 1, world0 = this, generation = contact.generation }
                });
            else contactEndEvents0.Add(new ContactEndTouchEvent
            {
                shapeIdA = shapeIdA,
                shapeIdB = shapeIdB,
                contactId = new() { index1 = contact.contactId + 1, world0 = this, generation = contact.generation }
            });
        }
        if (contact.edge0.prevKey != -1)
        {
            Contact prevContact = contacts[contact.edge0.prevKey >> 1];
            if ((contact.edge0.prevKey & 1) == 1) prevContact.edge1.nextKey = contact.edge0.nextKey;
            else prevContact.edge0.nextKey = contact.edge0.nextKey;
        }
        if (contact.edge0.nextKey != -1)
        {
            Contact nextContact = contacts[contact.edge0.nextKey >> 1];
            if ((contact.edge0.nextKey & 1) == 1) nextContact.edge1.prevKey = contact.edge0.prevKey;
            else nextContact.edge0.prevKey = contact.edge0.prevKey;
        }
        int contactId = contact.contactId;
        int edgeKeyA = contactId << 1;
        if (bodyA.headContactKey == edgeKeyA) bodyA.headContactKey = contact.edge0.nextKey;
        bodyA.contactCount--;
        if (contact.edge1.prevKey != -1)
        {
            Contact prevContact = contacts[contact.edge1.prevKey >> 1];
            if ((contact.edge1.prevKey & 1) == 1) prevContact.edge1.nextKey = contact.edge1.nextKey;
            else prevContact.edge0.nextKey = contact.edge1.nextKey;
        }
        if (contact.edge1.nextKey != -1)
        {
            Contact nextContact = contacts[contact.edge1.nextKey >> 1];
            if ((contact.edge1.nextKey & 1) == 1) nextContact.edge1.prevKey = contact.edge1.prevKey;
            else nextContact.edge0.prevKey = contact.edge1.prevKey;
        }
        int edgeKeyB = (contactId << 1) | 1;
        if (bodyB.headContactKey == edgeKeyB) bodyB.headContactKey = contact.edge1.nextKey;
        bodyB.contactCount--;

        if (contact.islandId != -1) UnlinkContact(contact);
        if (contact.colorIndex != -1)
        {
            Debug.Assert(contact.setIndex == (int)SetType.Awake);
            RemoveContactFromGraph(bodyIdA, bodyIdB, contact.colorIndex, contact.localIndex);
        }
        else
        {
            Debug.Assert(contact.setIndex != (int)SetType.Awake || !contact.flags.HasFlag(ContactFlags.Touching));
            SolverSet set = solverSets[contact.setIndex];
            int movedIndex = set.contactSims.RemoveSwap(contact.localIndex);
            if (movedIndex != -1)
            {
                ContactSim movedContactSim = set.contactSims[contact.localIndex];
                Contact movedContact = contacts[movedContactSim.contactId];
                movedContact.localIndex = contact.localIndex;
            }
        }
        contact.contactId = -1;
        contact.setIndex = -1;
        contact.colorIndex = -1;
        contact.localIndex = -1;
        contactIdPool.FreeId(contactId);
        if (wakeBodies && touching)
        {
            WakeBody(bodyA); WakeBody(bodyB);
        }
    }
    public ContactSim GetContactSim(Contact contact)
    {
        if (contact.setIndex == (int)SetType.Awake && contact.colorIndex != -1)
        {
            Debug.Assert(0 <= contact.colorIndex && contact.colorIndex <= Box2D.GraphColorCount);
            GraphColor color = constraintGraph.colors[contact.colorIndex];
            return color.contactSims[contact.localIndex];
        }
        SolverSet set = solverSets[contact.setIndex];
        return set.contactSims[contact.localIndex];
    }
    public bool UpdateContact(ContactSim contactSim, Shape shapeA, Transform transformA, Vector2 centerOffsetA,
        Shape shapeB, Transform transformB, Vector2 centerOffsetB)
    {
        Manifold oldManifold = contactSim.manifold;
        ManifoldFcn fcn = ContactRegister.s_registers[(int)shapeA.type][(int)shapeB.type].fcn;
        contactSim.manifold = fcn(shapeA, transformA, shapeB, transformB, ref contactSim.cache);
        contactSim.friction = frictionCallback(shapeA.material.friction, shapeA.material.userMaterialId, shapeB.material.friction, shapeB.material.userMaterialId);
        contactSim.restitution = restitutionCallback(shapeA.material.restitution, shapeA.material.userMaterialId, shapeB.material.restitution, shapeB.material.userMaterialId);
        if (shapeA.material.rollingResistance > 0 || shapeB.material.rollingResistance > 0)
        {
            contactSim.rollingResistance = Math.Max(shapeA.material.rollingResistance, shapeB.material.rollingResistance)
                * Math.Max(shapeA.GetRadius(), shapeB.GetRadius());
        }
        else contactSim.rollingResistance = 0;
        contactSim.tangentSpeed = shapeA.material.tangentSpeed + shapeB.material.tangentSpeed;
        int pointCount = contactSim.manifold.pointCount;
        bool touching = pointCount > 0;
        if (touching && preSolveFcn != null && contactSim.simFlags.HasFlag(ContactSimFlags.EnablePreSolveEvents))
        {
            ShapeID shapeIdA = new() { index1 = shapeA.id + 1, world0 = this, generation = shapeA.generation };
            ShapeID shapeIdB = new() { index1 = shapeB.id + 1, world0 = this, generation = shapeB.generation };
            ref Manifold manifold = ref contactSim.manifold;
            float bestSeparation = manifold.point0.separation;
            Vector2 bestPoint = manifold.point0.point;
            for (int i = 1; i < manifold.pointCount; i++)
            {
                float separation = manifold.point1.separation;
                if (separation < bestSeparation)
                {
                    bestSeparation = separation;
                    bestPoint = manifold.point1.point;
                }
            }
            touching = preSolveFcn(shapeIdA, shapeIdB, bestPoint, manifold.normal, preSolveContext);
            if (!touching)
            {
                pointCount = 0;
                manifold.pointCount = 0;
            }
        }
        if (!enableSpeculative && pointCount == 2)
        {
            if (contactSim.manifold.point0.separation > 1.5f * Box2D.LinearSlop)
            {
                contactSim.manifold.point0 = contactSim.manifold.point1;
                contactSim.manifold.pointCount = 1;
            }
            else if (contactSim.manifold.point0.separation > 1.5f * Box2D.LinearSlop)
            {
                contactSim.manifold.pointCount = 1;
            }
            pointCount = contactSim.manifold.pointCount;
        }
        if (touching && (shapeA.enableHitEvents || shapeB.enableHitEvents))
        {
            contactSim.simFlags |= ContactSimFlags.EnableHitEvent;
        }
        else contactSim.simFlags &= ~ContactSimFlags.EnableHitEvent;
        if (pointCount > 0)
            contactSim.manifold.rollingImpulse = oldManifold.rollingImpulse;

        int unmatchedCount = 0;
        for (int i = 0; i < pointCount; i++)
        {
            ref ManifoldPoint mp2 = ref contactSim.manifold.point0;
            if (i == 1) mp2 = ref contactSim.manifold.point1;
            mp2.anchorA -= centerOffsetA;
            mp2.anchorB -= centerOffsetB;
            mp2.tangentImpulse = 0;
            mp2.normalImpulse = 0;
            mp2.totalNormalImpulse = 0;
            mp2.normalVelocity = 0;
            mp2.persisted = false;
            ushort id2 = mp2.id;
            for (int j = 0; j < oldManifold.pointCount; j++)
            {
                ref ManifoldPoint mp1 = ref oldManifold.point0;
                if (j == 1) mp1 = ref oldManifold.point1;
                if (mp1.id == id2)
                {
                    mp2.normalImpulse = mp1.normalImpulse;
                    mp2.tangentImpulse = mp1.tangentImpulse;
                    mp2.persisted = true;
                    mp1.normalImpulse = 0;
                    mp1.tangentImpulse = 0;
                    break;
                }
            }
            unmatchedCount += mp2.persisted ? 0 : 1;
        }
        if (touching) contactSim.simFlags |= ContactSimFlags.Touching;
        else contactSim.simFlags &= ~ContactSimFlags.Touching;
        return touching;
    }
}
public delegate Manifold ManifoldFcn(Shape shapeA, Transform xfA, Shape shapeB, Transform xfB, ref SimplexCache cache);
public struct ContactRegister
{
    public ManifoldFcn fcn;
    public bool primary;
    public static ContactRegister[][] s_registers;
    static Manifold CircleManifold(Shape shapeA, Transform xfA, Shape shapeB, Transform xfB, ref SimplexCache cache) => Collision.CollideCircles((Circle)shapeA.shape, xfA, (Circle)shapeB.shape, xfB);
    static Manifold CapsuleAndCircleManifold(Shape shapeA, Transform xfA, Shape shapeB, Transform xfB, ref SimplexCache cache) => Collision.CollideCapsuleAndCircle((Capsule)shapeA.shape, xfA, (Circle)shapeB.shape, xfB);
    static Manifold CapsuleManifold(Shape shapeA, Transform xfA, Shape shapeB, Transform xfB, ref SimplexCache cache) => Collision.CollideCapsules((Capsule)shapeA.shape, xfA, (Capsule)shapeB.shape, xfB);
    static Manifold PolygonAndCircleManifold(Shape shapeA, Transform xfA, Shape shapeB, Transform xfB, ref SimplexCache cache) => Collision.CollidePolygonAndCircle((Polygon)shapeA.shape, xfA, (Circle)shapeB.shape, xfB);
    static Manifold PolygonAndCapsuleManifold(Shape shapeA, Transform xfA, Shape shapeB, Transform xfB, ref SimplexCache cache) => Collision.CollidePolygonAndCapsule((Polygon)shapeA.shape, xfA, (Capsule)shapeB.shape, xfB);
    static Manifold PolygonManifold(Shape shapeA, Transform xfA, Shape shapeB, Transform xfB, ref SimplexCache cache) => Collision.CollidePolygons((Polygon)shapeA.shape, xfA, (Polygon)shapeB.shape, xfB);
    static Manifold SegmentAndCircleManifold(Shape shapeA, Transform xfA, Shape shapeB, Transform xfB, ref SimplexCache cache) => Collision.CollideSegmentAndCircle((Segment)shapeA.shape, xfA, (Circle)shapeB.shape, xfB);
    static Manifold SegmentAndCapsuleManifold(Shape shapeA, Transform xfA, Shape shapeB, Transform xfB, ref SimplexCache cache) => Collision.CollideSegmentAndCapsule((Segment)shapeA.shape, xfA, (Capsule)shapeB.shape, xfB);
    static Manifold SegmentAndPolygonManifold(Shape shapeA, Transform xfA, Shape shapeB, Transform xfB, ref SimplexCache cache) => Collision.CollideSegmentAndPolygon((Segment)shapeA.shape, xfA, (Polygon)shapeB.shape, xfB);
    static Manifold ChainSegmentAndCircleManifold(Shape shapeA, Transform xfA, Shape shapeB, Transform xfB, ref SimplexCache cache) => Collision.CollideChainSegmentAndCircle((ChainSegment)shapeA.shape, xfA, (Circle)shapeB.shape, xfB);
    static Manifold ChainSegmentAndCapsuleManifold(Shape shapeA, Transform xfA, Shape shapeB, Transform xfB, ref SimplexCache cache) => Collision.CollideChainSegmentAndCapsule((ChainSegment)shapeA.shape, xfA, (Capsule)shapeB.shape, xfB, ref cache);
    static Manifold ChainSegmentAndPolygonManifold(Shape shapeA, Transform xfA, Shape shapeB, Transform xfB, ref SimplexCache cache) => Collision.CollideChainSegmentAndPolygon((ChainSegment)shapeA.shape, xfA, (Polygon)shapeB.shape, xfB, ref cache);
    public static void AddType(ManifoldFcn fcn, ShapeType type1, ShapeType type2)
    {
        Debug.Assert(0 <= type1 && (int)type1 <= s_registers.Length);
        Debug.Assert(0 <= type2 && (int)type2 <= s_registers.Length);
        s_registers[(int)type1][(int)type2].fcn = fcn;
        s_registers[(int)type1][(int)type2].primary = true;
        if (type1 != type2)
        {
            s_registers[(int)type2][(int)type1].fcn = fcn;
            s_registers[(int)type2][(int)type1].primary = false;
        }
    }
    static ContactRegister()
    {
        s_registers = new ContactRegister[Enum.GetValues<ShapeType>().Length][];
        for (int i = 0; i < s_registers.Length; i++) s_registers[i] = new ContactRegister[s_registers.Length];
        AddType(CircleManifold, ShapeType.Circle, ShapeType.Circle);
        AddType(CapsuleAndCircleManifold, ShapeType.Capsule, ShapeType.Circle);
        AddType(CapsuleManifold, ShapeType.Capsule, ShapeType.Capsule);
        AddType(PolygonAndCircleManifold, ShapeType.Polygon, ShapeType.Circle);
        AddType(PolygonAndCapsuleManifold, ShapeType.Polygon, ShapeType.Capsule);
        AddType(PolygonManifold, ShapeType.Polygon, ShapeType.Polygon);
        AddType(SegmentAndCircleManifold, ShapeType.Segment, ShapeType.Circle);
        AddType(SegmentAndCapsuleManifold, ShapeType.Segment, ShapeType.Capsule);
        AddType(SegmentAndPolygonManifold, ShapeType.Segment, ShapeType.Polygon);
        AddType(ChainSegmentAndCircleManifold, ShapeType.ChainSegment, ShapeType.Circle);
        AddType(ChainSegmentAndCapsuleManifold, ShapeType.ChainSegment, ShapeType.Capsule);
        AddType(ChainSegmentAndPolygonManifold, ShapeType.ChainSegment, ShapeType.Polygon);
    }
    public static Manifold ComputeManifold(Shape shapeA, Transform transformA, Shape shapeB, Transform transformB)
    {
        ManifoldFcn fcn = s_registers[(int)shapeA.type][(int)shapeB.type].fcn;
        SimplexCache cache = new();
        if (s_registers[(int)shapeA.type][(int)shapeB.type].primary) //???
            return fcn(shapeA, transformA, shapeB, transformB, ref cache);
        else return fcn(shapeB, transformB, shapeA, transformA, ref cache);
    }
}
