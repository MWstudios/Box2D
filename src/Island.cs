using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;

namespace Box2D;

/// <summary> Cached contact data stored in the island for fast contiguous iteration.
/// Avoids touching b2Contact during union-find in b2SplitIsland.</summary>
public struct ContactLink
{
    public int contactId, bodyIdA, bodyIdB;
}
/// <summary>Cached joint data stored in the island for fast contiguous iteration.</summary>
public struct JointLink
{
    public int jointId, bodyIdA, bodyIdB;
}

/// <summary>Deterministic solver<br/>
///<br/>
/// Collide all awake contacts<br/>
/// Use bit array to emit start/stop touching events in defined order, per thread. Try using contact index, assuming contacts are
/// created in a deterministic order. bit-wise OR together bit arrays and issue changes:<br/>
/// - start touching: merge islands - temporary linked list - mark root island dirty - wake all - largest island is root<br/>
/// - stop touching: increment constraintRemoveCount<br/>
/// Persistent island for awake bodies, joints, and contacts.
/// Contacts are touching.
/// Contacts and joints may connect to static bodies, but static bodies are not in the island.</summary>
/// https://en.wikipedia.org/wiki/Component_(graph_theory)
/// https://en.wikipedia.org/wiki/Dynamic_connectivity
public class Island
{
    /// <summary>index of solver set stored in b2World<br/>
    /// may be B2_NULL_INDEX</summary>
    public int setIndex;

    /// <summary>island index within set<br/>
    /// may be B2_NULL_INDEX</summary>
    public int localIndex;

    public int islandId;

    /// <summary>Union find<br/>
    /// todo this could go away if islands are merged immediately with b2LinkJoint and b2LinkContact</summary>
    public int parentIsland;

    /// <summary>Keeps track of how many contacts have been removed from this island.<br/>
    /// This is used to determine if an island is a candidate for splitting.</summary>
    public int constraintRemoveCount;

    /// <summary>I tried using a stack array for this but the data pointer goes out of
	/// sync when the world island array grows.</summary>
    public List<int> bodies;

    /// <summary>Contacts and joints that belong to this island. May connect to static
    /// bodies not in the island.
    /// Each link has the two body ids so that b2SplitIsland's union-find pass
    /// never needs to touch b2Contact/b2Joint.</summary>
    public List<ContactLink> contacts;
    public List<JointLink> joints;
}
public record IslandSim
{
    public int islandId;
}

public partial class World
{
    public Island CreateIsland(int setIndex)
    {
        Debug.Assert(setIndex == (int)SetType.Awake || setIndex >= (int)SetType.FirstSleeping);
        int islandId = islandIdPool.AllocId();
        if (islandId == islands.Count)
        {
            islands.Add(null);
        }
        else Debug.Assert(islands[islandId].setIndex == -1);
        SolverSet set = solverSets[setIndex];
        Island island = new()
        {
            setIndex = setIndex,
            localIndex = set.islandSims.Count,
            islandId = islandId,
            bodies = new(),
            contacts = new(),
            joints = new(),
            parentIsland = -1,
            constraintRemoveCount = 0
        };
        islands[islandId] = island;
        IslandSim islandSim = new(); set.islandSims.Add(islandSim);
        islandSim.islandId = islandId;
        return island;
    }
    public void DestroyIsland(int islandId)
    {
        if (splitIslandId == islandId) splitIslandId = -1;
        Island island = islands[islandId];
        SolverSet set = solverSets[island.setIndex];
        {
            int localIndex = island.localIndex;
            int lastIndex = set.islandSims.Count - 1;
            Debug.Assert(0 <= localIndex & localIndex <= lastIndex);
            int moveIslandId = set.islandSims[lastIndex].islandId;
            set.islandSims[localIndex] = set.islandSims[lastIndex];
            islands[moveIslandId].localIndex = localIndex;
            set.islandSims.RemoveAt(set.islandSims.Count - 1);
        }
        island.constraintRemoveCount = 0;
        island.localIndex = -1;
        island.islandId = -1;
        island.setIndex = -1;
        Debug.Assert(island.localIndex == -1);
        islandIdPool.FreeId(islandId);
    }
    public int MergeIslands(int islandIdA, int islandIdB)
    {
        if (islandIdA == islandIdB) return islandIdA;
        if (islandIdA == -1) { Debug.Assert(islandIdB != -1); return islandIdB; }
        if (islandIdB == -1) { Debug.Assert(islandIdA != -1); return islandIdA; }
        Island smallIsland = islands[islandIdA], bigIsland = islands[islandIdB];
        if (smallIsland.bodies.Count >= bigIsland.bodies.Count) (smallIsland, bigIsland) = (bigIsland, smallIsland);
        int bigIslandId = bigIsland.islandId;
        bigIsland.bodies.EnsureCapacity(bigIsland.bodies.Count + smallIsland.bodies.Count);
        for (int i = 0; i < smallIsland.bodies.Count; i++)
        {
            int bodyId = smallIsland.bodies[i];
            Body body = bodies[bodyId];
            Debug.Assert(body.islandId == smallIsland.islandId);
            body.islandId = bigIslandId;
            body.islandIndex = bigIsland.bodies.Count;
            bigIsland.bodies.Add(bodyId);
        }
        if (smallIsland.contacts.Count > 0)
        {
            bigIsland.contacts.EnsureCapacity(bigIsland.contacts.Count + smallIsland.contacts.Count);
            for (int i = 0; i < smallIsland.contacts.Count; i++)
            {
                ref ContactLink link = ref CollectionsMarshal.AsSpan(smallIsland.contacts)[i];
                Contact contact = contacts[link.contactId];
                contact.islandId = bigIslandId;
                contact.islandIndex = bigIsland.contacts.Count;
                bigIsland.contacts.Add(link);
            }
        }
        if (smallIsland.joints.Count > 0)
        {
            bigIsland.joints.EnsureCapacity(bigIsland.joints.Count + smallIsland.joints.Count);
            for (int i = 0; i < smallIsland.joints.Count; i++)
            {
                ref JointLink link = ref CollectionsMarshal.AsSpan(smallIsland.joints)[i];
                Joint joint = joints[link.jointId];
                joint.islandId = bigIslandId;
                joint.islandIndex = bigIsland.joints.Count;
                bigIsland.joints.Add(link);
            }
        }
        bigIsland.constraintRemoveCount += smallIsland.constraintRemoveCount;
        DestroyIsland(smallIsland.islandId);
        ValidateIsland(bigIslandId);
        return bigIslandId;
    }
    public void AddContactToIsland(int islandId, Contact contact)
    {
        Debug.Assert(contact.islandId == -1);
        Debug.Assert(contact.islandIndex == -1);
        Island island = islands[islandId];
        contact.islandId = islandId;
        contact.islandIndex = island.contacts.Count;
        island.contacts.Add(new() { contactId = contact.contactId, bodyIdA = contact.edge0.bodyId, bodyIdB = contact.edge1.bodyId });
        ValidateIsland(islandId);
    }
    /// <summary>Link a contact into an island.
    /// This performs union-find and path compression to join islands.
    /// https://en.wikipedia.org/wiki/Disjoint-set_data_structure</summary>
    public void LinkContact(Contact contact)
    {
        Debug.Assert(contact.flags.HasFlag(ContactFlags.Touching));
        int bodyIdA = contact.edge0.bodyId;
        int bodyIdB = contact.edge1.bodyId;
        Body bodyA = bodies[bodyIdA], bodyB = bodies[bodyIdB];
        Debug.Assert(bodyA.setIndex != (int)SetType.Disabled && bodyB.setIndex != (int)SetType.Disabled);
        Debug.Assert(bodyA.setIndex != (int)SetType.Static || bodyB.setIndex != (int)SetType.Static);
        if (bodyA.setIndex == (int)SetType.Awake && bodyB.setIndex >= (int)SetType.FirstSleeping)
            WakeSolverSet(bodyB.setIndex);
        if (bodyB.setIndex == (int)SetType.Awake && bodyA.setIndex >= (int)SetType.FirstSleeping)
            WakeSolverSet(bodyA.setIndex);
        int islandIdA = bodyA.islandId, islandIdB = bodyB.islandId;
        Debug.Assert(bodyA.setIndex != (int)SetType.Static || islandIdA == -1);
        Debug.Assert(bodyB.setIndex != (int)SetType.Static || islandIdB == -1);
        Debug.Assert(islandIdA != -1 || islandIdB != -1);
        AddContactToIsland(MergeIslands(islandIdA, islandIdB), contact);
    }
    /// <summary>This is called when a contact no longer has contact points or when a contact is destroyed.</summary>
    public void UnlinkContact(Contact contact)
    {
        Debug.Assert(contact.islandId != -1);
        int islandId = contact.islandId;
        Island island = islands[islandId];
        int removeIndex = contact.islandIndex;
        Debug.Assert(0 <= removeIndex && removeIndex < island.contacts.Count);
        Debug.Assert(island.contacts[removeIndex].contactId == contact.contactId);
        int movedIndex = island.contacts.RemoveSwap(removeIndex);
        if (movedIndex != -1)
        {
            ref ContactLink movedLink = ref CollectionsMarshal.AsSpan(island.contacts)[removeIndex];
            Contact movedContact = contacts[movedLink.contactId];
            Debug.Assert(movedContact.islandIndex == movedIndex);
            movedContact.islandIndex = removeIndex;
        }
        contact.islandId = -1;
        contact.islandIndex = -1;
        island.constraintRemoveCount++;
        ValidateIsland(islandId);
    }
    public void AddJointToIsland(int islandId, Joint joint)
    {
        Debug.Assert(joint.islandId == -1);
        Debug.Assert(joint.islandIndex == -1);
        Island island = islands[islandId];
        joint.islandId = islandId;
        joint.islandIndex = island.joints.Count;
        island.joints.Add(new() { jointId = joint.jointId, bodyIdA = joint.edge0.bodyId, bodyIdB = joint.edge1.bodyId });
        ValidateIsland(islandId);
    }
    public void LinkJoint(Joint joint)
    {
        Body bodyA = bodies[joint.edge0.bodyId], bodyB = bodies[joint.edge1.bodyId];
        Debug.Assert(bodyA.type == BodyType.Dynamic || bodyB.type == BodyType.Dynamic);
        if (bodyA.setIndex == (int)SetType.Awake && bodyB.setIndex >= (int)SetType.FirstSleeping)
            WakeSolverSet(bodyB.setIndex);
        else if (bodyB.setIndex == (int)SetType.Awake && bodyA.setIndex >= (int)SetType.FirstSleeping)
            WakeSolverSet(bodyA.setIndex);
        int islandIdA = bodyA.islandId, islandIdB = bodyB.islandId;
        Debug.Assert(islandIdA != -1 || islandIdB != -1);
        AddJointToIsland(MergeIslands(islandIdA, islandIdB), joint);
    }
    public void UnlinkJoint(Joint joint)
    {
        if (joint.islandId == -1) return;
        int islandId = joint.islandId;
        Island island = islands[islandId];
        int removeIndex = joint.islandIndex;
        Debug.Assert(0 <= removeIndex && removeIndex < island.joints.Count);
        Debug.Assert(island.joints[removeIndex].jointId == joint.jointId);
        int movedIndex = island.joints.RemoveSwap(removeIndex);
        if (movedIndex != -1)
        {
            ref JointLink movedLink = ref CollectionsMarshal.AsSpan(island.joints)[removeIndex];
            Joint movedJoint = joints[movedLink.jointId];
            Debug.Assert(movedJoint.islandIndex == movedIndex);
            movedJoint.islandIndex = removeIndex;
        }
        joint.islandId = -1;
        joint.islandIndex = -1;
        island.constraintRemoveCount++;
        ValidateIsland(islandId);
    }
    /// <summary>Find parent of a node. Use path halving to speed up further queries.</summary>
    static unsafe int IslandFindParent(int* parents, int node)
    {
        while (parents[node]!=node)
        {
            int grandParent = parents[parents[node]];
            parents[node] = grandParent;
            node = grandParent;
        }
        return node;
    }
    /// <summary>Connect the components containing node1 and node2.
    /// Uses rank to keep tree balanced. Tracks per-component contact and joint counts.</summary>
    static unsafe void IslandUnion(int* parents, int* ranks, int node1, int node2, int* contactCounts, int* jointCounts)
    {
        int root1 = IslandFindParent(parents, node1), root2 = IslandFindParent(parents, node2);
        if (root1 != root2)
        {
            if (ranks[root1] < ranks[root2])
            {
                parents[root1] = root2;
                contactCounts[root2] += contactCounts[root1];
                jointCounts[root2] += jointCounts[root1];
            }
            else if (ranks[root1] > ranks[root2])
            {
                parents[root2] = root1;
                contactCounts[root1] += contactCounts[root2];
                jointCounts[root1] += jointCounts[root2];
            }
            else
            {
                parents[root2] = root1;
                ranks[root1]++;
                contactCounts[root1] += contactCounts[root2];
                jointCounts[root1] += jointCounts[root2];
            }
        }
    }
    /// <summary>Possible optimizations:<br/>
    /// 1. use the body island id as the mark<br/>
    /// 2. start from the sleepy bodies and stop processing if a sleep body is connected to a non-sleepy body<br/>
    /// 3. use a sleepy flag on bodies to avoid velocity access</summary>
    public unsafe void SplitIsland(int baseId)
    {
        Island baseIsland = islands[baseId];
        int setIndex = baseIsland.setIndex;
        if (setIndex != (int)SetType.Awake) return;
        if (baseIsland.constraintRemoveCount == 0) return;
        ValidateIsland(baseId);
        int baseBodyCount = baseIsland.bodies.Count;
        var baseBodyIds = baseIsland.bodies;
        int baseBodyCapacity = baseIsland.bodies.Capacity;
        int baseContactCount = baseIsland.contacts.Count;
        var baseContacts = baseIsland.contacts;
        int baseContactCapacity = baseIsland.contacts.Capacity;
        int baseJointCount = baseIsland.joints.Count;
        var baseJoints = baseIsland.joints;
        int baseJointCapacity = baseIsland.joints.Capacity;
        int componentCount = 0;
        int[] _parents = new int[baseBodyCount], _contactCounts = new int[baseBodyCount],
            _jointCounts = new int[baseBodyCount], _ranks = new int[baseBodyCount];
        fixed (int* parents = _parents, contactCounts = _contactCounts, jointCounts = _jointCounts, ranks = _ranks)
        {
            for (int i = 0; i < baseBodyCount; i++) parents[i] = i;
            for (int i = 0; i < baseContactCount; i++)
            {
                int bodyIdA = baseContacts[i].bodyIdA, bodyIdB = baseContacts[i].bodyIdB;
                Debug.Assert(0 <= bodyIdA && bodyIdA < bodies.Count);
                Debug.Assert(0 <= bodyIdB && bodyIdB < bodies.Count);
                Body bodyA = bodies[bodyIdA], bodyB = bodies[bodyIdB];
                int islandIndexA = bodyA.islandIndex, islandIndexB = bodyB.islandIndex;
                if (islandIndexA != -1 && islandIndexB != -1)
                {
                    Debug.Assert(0 <= islandIndexA && islandIndexA < baseBodyCount);
                    Debug.Assert(0 <= islandIndexB && islandIndexB < baseBodyCount);
                    IslandUnion(parents, ranks, islandIndexA, islandIndexB, contactCounts, jointCounts);
                    int root = IslandFindParent(parents, islandIndexA);
                    contactCounts[root]++;
                }
                else
                {
                    int islandIndex = islandIndexA != -1 ? islandIndexA : islandIndexB;
                    int root = IslandFindParent(parents, islandIndex);
                    contactCounts[root]++;
                }
            }
            for (int i = 0; i < baseJointCount; i++)
            {
                int bodyIdA = baseJoints[i].bodyIdA, bodyIdB = baseJoints[i].bodyIdB;
                Debug.Assert(0 <= bodyIdA && bodyIdA < bodies.Count);
                Debug.Assert(0 <= bodyIdB && bodyIdB < bodies.Count);
                Body bodyA = bodies[bodyIdA], bodyB = bodies[bodyIdB];
                int islandIndexA = bodyA.islandIndex, islandIndexB = bodyB.islandIndex;
                if (islandIndexA != -1 && islandIndexB != -1)
                {
                    Debug.Assert(0 <= islandIndexA && islandIndexA < baseBodyCount);
                    Debug.Assert(0 <= islandIndexB && islandIndexB < baseBodyCount);
                    IslandUnion(parents, ranks, islandIndexA, islandIndexB, contactCounts, jointCounts);
                    int root = IslandFindParent(parents, islandIndexA);
                    jointCounts[root]++;
                }
                else
                {
                    int islandIndex = islandIndexA != -1 ? islandIndexA : islandIndexB;
                    int root = IslandFindParent(parents, islandIndex);
                    jointCounts[root]++;
                }
            }
            for (int i = 0; i < baseBodyCount; i++)
            {
                parents[i] = IslandFindParent(parents, i);
                if (parents[i] == i) componentCount++;
            }
            if (componentCount == 1)
            {
                baseIsland.constraintRemoveCount = 0;
                return;
            }
            int[] rootMap = new int[baseBodyCount], componentBodyCounts = new int[componentCount],
                componentContactCounts = new int[componentCount], componentJointCounts = new int[componentCount];
            for (int i = 0; i < rootMap.Length; i++) rootMap[i] = -1;
            int islandCount = 0;
            for (int i = 0; i < baseBodyCount; i++)
            {
                int rootIndex = _parents[i];
                if (rootMap[rootIndex] == -1)
                {
                    rootMap[rootIndex] = islandCount;
                    componentBodyCounts[islandCount] = 0;
                    componentContactCounts[islandCount] = _contactCounts[rootIndex];
                    componentJointCounts[islandCount] = _jointCounts[rootIndex];
                    islandCount++;
                }
                componentBodyCounts[rootMap[rootIndex]]++;
            }
            Debug.Assert(islandCount == componentCount);
            int[] islandIds = new int[islandCount];
            for (int i = 0; i < islandCount; i++)
            {
                Island newIsland = CreateIsland((int)SetType.Awake);
                islandIds[i] = newIsland.islandId;
                newIsland.bodies.EnsureCapacity(componentBodyCounts[i]);
                newIsland.contacts.EnsureCapacity(componentContactCounts[i]);
                newIsland.joints.EnsureCapacity(componentJointCounts[i]);
            }
            for (int i = 0; i < baseBodyCount; i++)
            {
                int bodyId = baseBodyIds[i];
                int root = IslandFindParent(parents, i);
                int newIslandId = islandIds[rootMap[root]];
                Body body = bodies[bodyId];
                Island newIsland = islands[newIslandId];
                body.islandId = newIslandId;
                body.islandIndex = newIsland.bodies.Count;
                Debug.Assert(newIsland.bodies.Count < newIsland.bodies.Capacity);
                newIsland.bodies.Add(bodyId);
            }
            for (int i = 0; i < baseContactCount; i++)
            {
                ref ContactLink link = ref CollectionsMarshal.AsSpan(baseContacts)[i];
                Contact contact = contacts[link.contactId];
                Body bodyA = bodies[link.bodyIdA], bodyB = bodies[link.bodyIdB];
                int targetIslandId = bodyA.islandId != -1 ? bodyA.islandId : bodyB.islandId;
                Island targetIsland = islands[targetIslandId];
                contact.islandId = targetIslandId;
                contact.islandIndex = targetIsland.contacts.Count;
                Debug.Assert(targetIsland.contacts.Count < targetIsland.contacts.Capacity);
                targetIsland.contacts.Add(link);
            }
            for (int i = 0; i < baseJointCount; i++)
            {
                ref JointLink link = ref CollectionsMarshal.AsSpan(baseJoints)[i];
                Joint joint = joints[link.jointId];
                Body bodyA = bodies[link.bodyIdA], bodyB = bodies[link.bodyIdB];
                int targetIslandId = bodyA.islandId != -1 ? bodyA.islandId : bodyB.islandId;
                Island targetIsland = islands[targetIslandId];
                joint.islandId = targetIslandId;
                joint.islandIndex = targetIsland.joints.Count;
                Debug.Assert(targetIsland.joints.Count < targetIsland.joints.Capacity);
                targetIsland.joints.Add(link);
            }
        }
        DestroyIsland(baseId);
    }
    /// <summary>Split an island because some contacts and/or joints have been removed.
    /// This is called during the constraint solve while islands are not being touched. This uses DFS and touches a lot of memory,
    /// so it can be quite slow.</summary>
    /// <remarks>Note: contacts/joints connected to static bodies must belong to an island but don't affect island connectivity<br/>
    /// Note: static bodies are never in an island<br/>
    /// Note: this task interacts with some allocators without locks under the assumption that no other tasks
    /// are interacting with these data structures.</remarks>
    public static void SplitIslandTask(object context)
    {
        World world = (World)context;
        Debug.Assert(world.splitIslandId != -1);
        world.SplitIsland(world.splitIslandId);
    }
    public void ValidateIsland(int islandId)
    {
#if B2_VALIDATE
        if (islandId == -1) return;
        Island island = islands[islandId];
        Debug.Assert(island.islandId == islandId);
        Debug.Assert(island.setIndex != -1);
        {
            Debug.Assert(island.bodies.Count > 0);
            Debug.Assert(island.bodies.Count <= bodyIdPool.GetIdCount());
            for (int i = 0; i < island.bodies.Count; i++)
            {
                Body body = bodies[island.bodies[i]];
                Debug.Assert(body.islandId == islandId);
                Debug.Assert(body.islandIndex == i);
                Debug.Assert(body.setIndex == island.setIndex);
            }
        }
        if (island.contacts.Count > 0)
        {
            Debug.Assert(island.contacts.Count <= contactIdPool.GetIdCount());
            for (int i = 0; i < island.contacts.Count; i++)
            {
                ref ContactLink link = ref CollectionsMarshal.AsSpan(island.contacts)[i];
                Contact contact = contacts[link.contactId];
                Debug.Assert(contact.setIndex == island.setIndex);
                Debug.Assert(contact.islandId == islandId);
                Debug.Assert(contact.islandIndex == i);
            }
        }
        if (island.joints.Count > 0)
        {
            Debug.Assert(island.joints.Count <= jointIdPool.GetIdCount());
            for (int i = 0; i < island.joints.Count; i++)
            {
                ref JointLink link = ref CollectionsMarshal.AsSpan(island.joints)[i];
                Joint joint = joints[link.jointId];
                Debug.Assert(joint.setIndex == island.setIndex);
                Debug.Assert(joint.islandId == islandId);
                Debug.Assert(joint.islandIndex == i);
            }
        }
#endif
    }
}