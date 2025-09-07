using System.Collections.Generic;
using System.Diagnostics;

namespace Box2D;

/// <summary>Deterministic solver<br/>
///<br/>
/// Collide all awake contacts<br/>
/// Use bit array to emit start/stop touching events in defined order, per thread. Try using contact index, assuming contacts are
/// created in a deterministic order. bit-wise OR together bit arrays and issue changes:<br/>
/// - start touching: merge islands - temporary linked list - mark root island dirty - wake all - largest island is root<br/>
/// - stop touching: increment constraintRemoveCount<br/>
/// Persistent island for awake bodies, joints, and contacts</summary>
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

    public int headBody;
    public int tailBody;
    public int bodyCount;

    public int headContact;
    public int tailContact;
    public int contactCount;

    public int headJoint;
    public int tailJoint;
    public int jointCount;

    /// <summary>Union find<br/>
    /// todo this could go away if islands are merged immediately with b2LinkJoint and b2LinkContact</summary>
    public int parentIsland;

    /// <summary>Keeps track of how many contacts have been removed from this island.<br/>
    /// This is used to determine if an island is a candidate for splitting.</summary>
    public int constraintRemoveCount;

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
            headBody = -1,
            tailBody = -1,
            bodyCount = 0,
            headContact = -1,
            tailContact = -1,
            contactCount = 0,
            headJoint = -1,
            tailJoint = -1,
            jointCount = 0,
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
        int movedIndex = set.islandSims.RemoveSwap(island.localIndex);
        if (movedIndex != -1)
        {
            IslandSim movedElement = set.islandSims[island.localIndex];
            int movedId = movedElement.islandId;
            Island movedIsland = islands[movedId];
            Debug.Assert(movedIsland.localIndex == movedIndex);
            movedIsland.localIndex = island.localIndex;
        }
        island.islandId = -1;
        island.setIndex = -1;
        island.localIndex = -1;
        islandIdPool.FreeId(islandId);
    }
    public int MergeIslands(int islandIdA, int islandIdB)
    {
        if (islandIdA == islandIdB) return islandIdA;
        if (islandIdA == -1) { Debug.Assert(islandIdB != -1); return islandIdB; }
        if (islandIdB == -1) { Debug.Assert(islandIdA != -1); return islandIdA; }
        Island islandA = islands[islandIdA], islandB = islands[islandIdB];
        Island big, small;
        if (islandA.bodyCount >= islandB.bodyCount) { big = islandA; small = islandB; }
        else { big = islandB; small = islandA; }
        int bigId = big.islandId;
        for (int bodyId = small.headBody; bodyId != -1;)
        { Body body = bodies[bodyId]; body.islandId = bigId; bodyId = body.islandNext; }
        for (int contactId = small.headContact; contactId != -1;)
        { Contact contact = contacts[contactId]; contact.islandId = bigId; contactId = contact.islandNext; }
        for (int jointId = small.headJoint; jointId != -1;)
        { Joint joint = joints[jointId]; joint.islandId = bigId; jointId = joint.islandNext; }
        Debug.Assert(big.tailBody != -1);
        Body tailBody = bodies[big.tailBody];
        Debug.Assert(tailBody.islandNext == -1);
        tailBody.islandNext = small.headBody;
        Debug.Assert(small.headBody != -1);
        Body headBody = bodies[small.headBody];
        Debug.Assert(headBody.islandPrev == -1);
        headBody.islandPrev = big.tailBody;
        big.tailBody = small.tailBody;
        big.bodyCount += small.bodyCount;
        if (big.headContact == -1)
        {
            Debug.Assert(big.tailContact == -1 && big.contactCount == 0);
            big.headContact = small.headContact;
            big.tailContact = small.tailContact;
            big.contactCount = small.contactCount;
        }
        else if (small.headContact != -1)
        {
            Debug.Assert(small.tailContact != -1 && small.contactCount > 0);
            Debug.Assert(big.tailContact != -1 && big.contactCount > 0);
            Contact tailContact = contacts[big.tailContact];
            Debug.Assert(tailContact.islandNext == -1);
            tailContact.islandNext = small.headContact;
            Contact headContact = contacts[small.headContact];
            Debug.Assert(headContact.islandPrev == -1);
            headContact.islandPrev = big.tailContact;
            big.tailContact = small.tailContact;
            big.contactCount += small.contactCount;
        }
        if (big.headJoint == -1)
        {
            Debug.Assert(big.tailJoint == -1 && big.jointCount == 0);
            big.headJoint = small.headJoint;
            big.tailJoint = small.tailJoint;
            big.jointCount = small.jointCount;
        }
        else if (small.headJoint != -1)
        {
            Debug.Assert(small.tailJoint != -1 && small.jointCount > 0);
            Debug.Assert(big.tailJoint != -1 && big.jointCount > 0);
            Joint tailJoint = joints[big.tailJoint];
            Debug.Assert(tailJoint.islandNext == -1);
            tailJoint.islandNext = small.headJoint;
            Joint headJoint = joints[small.headJoint];
            Debug.Assert(headJoint.islandPrev == -1);
            headJoint.islandPrev = big.tailJoint;
            big.tailJoint = small.tailJoint;
            big.jointCount += small.jointCount;
        }
        big.constraintRemoveCount += small.constraintRemoveCount;
        small.bodyCount = 0;
        small.contactCount = 0;
        small.jointCount = 0;
        small.headBody = -1;
        small.headContact = -1;
        small.headJoint = -1;
        small.tailBody = -1;
        small.tailContact = -1;
        small.tailJoint = -1;
        small.constraintRemoveCount = 0;
        DestroyIsland(small.islandId);
        ValidateIsland(bigId);
        return bigId;
    }
    public void AddContactToIsland(int islandId, Contact contact)
    {
        Debug.Assert(contact.islandId == -1);
        Debug.Assert(contact.islandPrev == -1);
        Debug.Assert(contact.islandNext == -1);
        Island island = islands[islandId];
        if (island.headContact != -1)
        {
            contact.islandNext = island.headContact;
            Contact headContact = contacts[island.headContact];
            headContact.islandPrev = contact.contactId;
        }
        island.headContact = contact.contactId;
        if (island.tailContact == -1) island.tailContact = island.headContact;
        island.contactCount++;
        contact.islandId = islandId;
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
        if (contact.islandPrev != -1)
        {
            Contact prevContact = contacts[contact.islandPrev];
            Debug.Assert(prevContact.islandNext == contact.contactId);
            prevContact.islandNext = contact.islandNext;
        }
        if (contact.islandNext != -1)
        {
            Contact nextContact = contacts[contact.islandNext];
            Debug.Assert(nextContact.islandPrev == contact.contactId);
            nextContact.islandPrev = contact.islandPrev;
        }
        if (island.headContact == contact.contactId) island.headContact = contact.islandNext;
        if (island.tailContact == contact.contactId) island.tailContact = contact.islandPrev;
        Debug.Assert(island.contactCount > 0);
        island.contactCount--;
        island.constraintRemoveCount++;
        contact.islandId = -1;
        contact.islandPrev = -1;
        contact.islandNext = -1;
        ValidateIsland(islandId);
    }
    public void AddJointToIsland(int islandId, Joint joint)
    {
        Debug.Assert(joint.islandId == -1);
        Debug.Assert(joint.islandPrev == -1);
        Debug.Assert(joint.islandNext == -1);
        Island island = islands[islandId];
        if (island.headJoint != -1)
        {
            joint.islandNext = island.headJoint;
            Joint headJoint = joints[island.headJoint];
            headJoint.islandPrev = joint.jointId;
        }
        island.headJoint = joint.jointId;
        if (island.tailJoint == -1) island.tailJoint = island.headJoint;
        island.jointCount++;
        joint.islandId = islandId;
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
        if (joint.islandPrev != -1)
        {
            Joint prevJoint = joints[joint.islandPrev];
            Debug.Assert(prevJoint.islandNext == joint.jointId);
            prevJoint.islandNext = joint.islandNext;
        }
        if (joint.islandNext != -1)
        {
            Joint nextJoint = joints[joint.islandNext];
            Debug.Assert(nextJoint.islandPrev == joint.jointId);
            nextJoint.islandPrev = joint.islandPrev;
        }
        if (island.headJoint == joint.jointId) island.headJoint = joint.islandNext;
        if (island.tailJoint == joint.jointId) island.tailJoint = joint.islandPrev;
        Debug.Assert(island.jointCount > 0);
        island.jointCount--;
        island.constraintRemoveCount++;
        joint.islandId = -1;
        joint.islandPrev = -1;
        joint.islandNext = -1;
        ValidateIsland(islandId);
    }
    /// <summary>Possible optimizations:<br/>
    /// 1. use the body island id as the mark<br/>
    /// 2. start from the sleepy bodies and stop processing if a sleep body is connected to a non-sleepy body<br/>
    /// 3. use a sleepy flag on bodies to avoid velocity access</summary>
    public void SplitIsland(int baseId)
    {
        Island baseIsland = islands[baseId];
        int setIndex = baseIsland.setIndex;
        if (setIndex != (int)SetType.Awake) return;
        if (baseIsland.constraintRemoveCount == 0) return;
        ValidateIsland(baseId);
        int bodyCount = baseIsland.bodyCount;
        Stack<int> stack = new(bodyCount);
        List<int> bodyIds = new(bodyCount);
        int index = 0, nextBody = baseIsland.headBody;
        while (nextBody != -1)
        {
            bodyIds.Add(nextBody); index++;
            Body body = bodies[nextBody];
            nextBody = body.islandNext;
        }
        Debug.Assert(index == bodyCount);
        for (int i = 0; i < bodyCount; i++)
        {
            int seedIndex = bodyIds[i];
            Body seed = bodies[seedIndex];
            Debug.Assert(seed.setIndex == setIndex);
            if (seed.islandId != baseId) continue;
            stack.Push(seedIndex);
            Island island = CreateIsland(setIndex);
            int islandId = island.islandId;
            seed.islandId = islandId;
            while (stack.Count > 0)
            {
                int bodyId = stack.Pop();
                Body body = bodies[bodyId];
                Debug.Assert(body.setIndex == (int)SetType.Awake);
                Debug.Assert(body.islandId == islandId);
                if (island.tailBody != -1)
                    bodies[island.tailBody].islandNext = bodyId;
                body.islandPrev = island.tailBody;
                body.islandNext = -1;
                island.tailBody = bodyId;
                if (island.headBody == -1)
                    island.headBody = bodyId;
                island.bodyCount++;
                int contactKey = body.headContactKey;
                while (contactKey != -1)
                {
                    int contactId = contactKey >> 1;
                    int edgeIndex = contactKey & 1;
                    Contact contact = contacts[contactId];
                    Debug.Assert(contact.contactId == contactId);
                    contactKey = edgeIndex == 1 ? contact.edge1.nextKey : contact.edge0.nextKey;
                    if (contact.islandId == islandId) continue;
                    if (!contact.flags.HasFlag(ContactFlags.Touching)) continue;
                    int otherEdgeIndex = edgeIndex ^ 1;
                    int otherBodyId = otherEdgeIndex == 1 ? contact.edge1.bodyId : contact.edge0.bodyId;
                    Body otherBody = bodies[otherBodyId];
                    if (otherBody.islandId != islandId && otherBody.setIndex != (int)SetType.Static)
                    {
                        Debug.Assert(stack.Count < bodyCount);
                        stack.Push(otherBodyId);
                        otherBody.islandId = islandId;
                    }
                    contact.islandId = islandId;
                    if (island.tailContact != -1)
                    {
                        Contact tailContact = contacts[island.tailContact];
                        tailContact.islandNext = contactId;
                    }
                    contact.islandPrev = island.tailContact;
                    contact.islandNext = -1;
                    island.tailContact = contactId;
                    if (island.headContact == -1) island.headContact = contactId;
                    island.contactCount++;
                }
                int jointKey = body.headJointKey;
                while (jointKey != -1)
                {
                    int jointId = jointKey >> 1;
                    int edgeIndex = jointKey & 1;
                    Joint joint = joints[jointId];
                    Debug.Assert(joint.jointId == jointId);
                    jointKey = edgeIndex == 1 ? joint.edge1.nextKey : joint.edge0.nextKey;
                    if (joint.islandId == islandId) continue;
                    if (joint.setIndex == (int)SetType.Disabled) continue;
                    int otherEdgeIndex = edgeIndex ^ 1;
                    int otherBodyId = otherEdgeIndex == 1 ? joint.edge1.bodyId : joint.edge0.bodyId;
                    Body otherBody = bodies[otherBodyId];
                    if (otherBody.setIndex == (int)SetType.Disabled) continue;
                    if (body.type != BodyType.Dynamic && otherBody.type != BodyType.Dynamic) continue;
                    if (otherBody.islandId != islandId && otherBody.setIndex == (int)SetType.Awake)
                    {
                        Debug.Assert(stack.Count < bodyCount);
                        stack.Push(otherBodyId);
                        otherBody.islandId = islandId;
                    }
                    joint.islandId = islandId;
                    if (island.tailJoint != -1)
                    {
                        Joint tailJoint = joints[island.tailJoint];
                        tailJoint.islandNext = jointId;
                    }
                    joint.islandPrev = island.tailJoint;
                    joint.islandNext = -1;
                    island.tailJoint = jointId;
                    if (island.headJoint == -1) island.headJoint = jointId;
                    island.jointCount++;
                }
            }
            ValidateIsland(islandId);
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
    public static void SplitIslandTask(int startIndex, int endIndex, uint threadIndex, object context)
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
        Debug.Assert(island.headBody != -1);
        {
            Debug.Assert(island.tailBody != -1);
            Debug.Assert(island.bodyCount > 0);
            if (island.bodyCount > 1) Debug.Assert(island.tailBody != island.headBody);
            int count = 0, bodyId = island.headBody;
            while (bodyId != -1)
            {
                Body body = bodies[bodyId];
                Debug.Assert(body.islandId == islandId);
                Debug.Assert(body.setIndex == island.setIndex);
                count++;
                if (count == island.bodyCount) Debug.Assert(bodyId == island.tailBody);
                bodyId = body.islandNext;
            }
            Debug.Assert(count == island.bodyCount);
        }
        if (island.headContact != -1)
        {
            Debug.Assert(island.tailContact != -1);
            Debug.Assert(island.contactCount > 0);
            if (island.contactCount > 1) Debug.Assert(island.tailContact != island.headContact);
            Debug.Assert(island.contactCount <= contactIdPool.GetIdCount());
            int count = 0, contactId = island.headContact;
            while (contactId != -1)
            {
                Contact contact = contacts[contactId];
                Debug.Assert(contact.setIndex == island.setIndex);
                Debug.Assert(contact.islandId == islandId);
                count++;
                if (count == island.contactCount) Debug.Assert(contactId == island.tailContact);
                contactId = contact.islandNext;
            }
            Debug.Assert(count == island.contactCount);
        }
        else
        {
            Debug.Assert(island.tailContact == -1);
            Debug.Assert(island.contactCount == 0);
        }
        if (island.headJoint != -1)
        {
            Debug.Assert(island.tailJoint != -1);
            Debug.Assert(island.jointCount > 0);
            if (island.jointCount > 1) Debug.Assert(island.tailJoint != island.headJoint);
            Debug.Assert(island.jointCount <= jointIdPool.GetIdCount());
            int count = 0, jointId = island.headJoint;
            while (jointId != -1)
            {
                Joint joint = joints[jointId];
                Debug.Assert(joint.setIndex == island.setIndex);
                count++;
                if (count == island.jointCount) Debug.Assert(jointId == island.tailJoint);
                jointId = joint.islandNext;
            }
            Debug.Assert(count == island.jointCount);
        }
        else
        {
            Debug.Assert(island.tailJoint == -1);
            Debug.Assert(island.jointCount == 0);
        }
#endif
    }
}