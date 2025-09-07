using System.Collections.Generic;
using System.Diagnostics;

namespace Box2D;

/// <summary>This holds solver set data. The following sets are used:<br/>
/// - static set for all static bodies an joints between static bodies<br/>
/// - active set for all active bodies with body states (no contacts or joints)<br/>
/// - disabled set for disabled bodies and their joints<br/>
/// - all further sets are sleeping island sets along with their contacts and joints<br/>
/// The purpose of solver sets is to achieve high memory locality.<br/>
/// https://www.youtube.com/watch?v=nZNd5FjSquk</summary>
public class SolverSet
{
    /// <summary>Body array. Empty for unused set.</summary>
    public List<BodySim> bodySims = new();

    /// <summary>Body state only exists for active set</summary>
    public PtrArray<BodyState> bodyStates = new(4);

    /// <summary>This holds sleeping/disabled joints. Empty for static/active set.</summary>
    public List<JointSim> jointSims = new();

    /// <summary>This holds all contacts for sleeping sets.
    /// This holds non-touching contacts for the awake set.</summary>
    public List<ContactSim> contactSims = new();

    /// <summary>The awake set has an array of islands. Sleeping sets normally have a single islands. However, joints
    /// created between sleeping sets causes the sets to merge, leaving them with multiple islands. These sleeping
    /// islands will be naturally merged with the set is woken.
    /// The static and disabled sets have no islands.
    /// Islands live in the solver sets to limit the number of islands that need to be considered for sleeping.</summary>
    public List<IslandSim> islandSims = new();

    /// <summary>Aligns with b2World::solverSetIdPool. Used to create a stable id for body/contact/joint/islands.</summary>
    public int setIndex;
}

public partial class World
{
    public void DestroySolverSet(int setIndex)
    {
        solverSetIdPool.FreeId(setIndex);
        solverSets[setIndex] = new() { setIndex = -1 };
    }
    /// <summary>Wake a solver set. Does not merge islands.
    /// Contacts can be in several places:<br/>
    /// 1. non-touching contacts in the disabled set<br/>
    /// 2. non-touching contacts already in the awake set<br/>
    /// 3. touching contacts in the sleeping set<br/>
    /// This handles contact types 1 and 3. Type 2 doesn't need any action.</summary>
    public void WakeSolverSet(int setIndex)
    {
        Debug.Assert(setIndex >= (int)SetType.FirstSleeping);
        SolverSet set = solverSets[setIndex];
        SolverSet awakeSet = solverSets[(int)SetType.Awake];
        SolverSet disabledSet = solverSets[(int)SetType.Disabled];
        for (int i = 0; i < set.bodySims.Count; i++)
        {
            BodySim simSrc = set.bodySims[i];
            Body body = bodies[simSrc.bodyId];
            body.setIndex = (int)SetType.Awake;
            body.localIndex = awakeSet.bodySims.Count;
            body.sleepTime = 0;
            BodySim simDst = simSrc with { };
            awakeSet.bodySims.Add(simDst);
            awakeSet.bodyStates.Add(new() { flags = body.flags });
            int contactKey = body.headContactKey;
            while (contactKey != -1)
            {
                int edgeIndex = contactKey & 1;
                int contactId = contactKey >> 1;
                Contact contact = contacts[contactId];
                contactKey = edgeIndex == 1 ? contact.edge1.nextKey : contact.edge0.nextKey;
                if (contact.setIndex != (int)SetType.Disabled)
                {
                    Debug.Assert(contact.setIndex == (int)SetType.Awake || contact.setIndex == setIndex);
                    continue;
                }
                int localIndex = contact.localIndex;
                ContactSim contactSim = disabledSet.contactSims[localIndex];
                Debug.Assert(!contact.flags.HasFlag(ContactFlags.Touching) && contactSim.manifold.pointCount == 0);
                contact.setIndex = (int)SetType.Awake;
                contact.localIndex = awakeSet.contactSims.Count;
                ContactSim awakeContactSim = contactSim with { };
                awakeSet.contactSims.Add(awakeContactSim);
                int movedLocalIndex = disabledSet.contactSims.RemoveSwap(localIndex);
                if (movedLocalIndex != -1)
                {
                    ContactSim movedContactSim = disabledSet.contactSims[localIndex];
                    Contact movedContact = contacts[movedContactSim.contactId];
                    Debug.Assert(movedContact.localIndex == movedLocalIndex);
                    movedContact.localIndex = localIndex;
                }
            }
        }
        {
            int contactCount = set.contactSims.Count;
            for (int i = 0; i < contactCount; i++)
            {
                ContactSim contactSim = set.contactSims[i];
                Contact contact = contacts[contactSim.contactId];
                Debug.Assert(contact.flags.HasFlag(ContactFlags.Touching));
                Debug.Assert(contactSim.simFlags.HasFlag(ContactSimFlags.Touching));
                Debug.Assert(contactSim.manifold.pointCount > 0);
                Debug.Assert(contact.setIndex == setIndex);
                AddContactToGraph(contactSim, contact);
                contact.setIndex = (int)SetType.Awake;
            }
        }
        {
            int jointCount = set.jointSims.Count;
            for (int i = 0; i < jointCount; i++)
            {
                JointSim jointSim = set.jointSims[i];
                Joint joint = joints[jointSim.jointId];
                Debug.Assert(joint.setIndex == setIndex);
                AddJointToGraph(jointSim, joint);
                joint.setIndex = (int)SetType.Awake;
            }
        }
        {
            int islandCount = set.islandSims.Count;
            for (int i = 0; i < islandCount; i++)
            {
                IslandSim islandSrc = set.islandSims[i];
                Island island = islands[islandSrc.islandId];
                island.setIndex = (int)SetType.Awake;
                island.localIndex = awakeSet.islandSims.Count;
                IslandSim islandDst = islandSrc with { };
                awakeSet.islandSims.Add(islandDst);
            }
        }
        DestroySolverSet(setIndex);
    }
    public void TrySleepIsland(int islandId)
    {
        Island island = islands[islandId];
        Debug.Assert(island.setIndex == (int)SetType.Awake);
        if (island.constraintRemoveCount > 0) return;
        int sleepSetId = solverSetIdPool.AllocId();
        if (sleepSetId == solverSets.Count) solverSets.Add(new() { setIndex = -1 });
        SolverSet sleepSet = solverSets[sleepSetId];
        SolverSet awakeSet = solverSets[(int)SetType.Awake];
        Debug.Assert(0 <= island.localIndex && island.localIndex < awakeSet.islandSims.Count);
        sleepSet.setIndex = sleepSetId;
        sleepSet.bodySims = new(island.bodyCount);
        sleepSet.contactSims = new(island.contactCount);
        sleepSet.jointSims = new(island.jointCount);
        {
            SolverSet disabledSet = solverSets[(int)SetType.Disabled];
            int bodyId = island.headBody;
            while (bodyId != -1)
            {
                Body body = bodies[bodyId];
                Debug.Assert(body.setIndex == (int)SetType.Awake);
                Debug.Assert(body.islandId == islandId);
                if (body.bodyMoveIndex != -1)
                {
                    ref BodyMoveEvent moveEvent = ref System.Runtime.InteropServices.CollectionsMarshal.AsSpan(bodyMoveEvents)[body.bodyMoveIndex];
                    Debug.Assert(moveEvent.bodyId.index1 - 1 == bodyId);
                    Debug.Assert(moveEvent.bodyId.generation == body.generation);
                    moveEvent.fellAsleep = true;
                    body.bodyMoveIndex = -1;
                }
                int awakeBodyIndex = body.localIndex;
                BodySim awakeSim = awakeSet.bodySims[awakeBodyIndex];
                int sleepBodyIndex = sleepSet.bodySims.Count;
                BodySim sleepBodySim = awakeSim with { };
                sleepSet.bodySims.Add(sleepBodySim);
                int movedIndex = awakeSet.bodySims.RemoveSwap(awakeBodyIndex);
                if (movedIndex != -1)
                {
                    BodySim movedSim = awakeSet.bodySims[awakeBodyIndex];
                    int movedId = movedSim.bodyId;
                    Body movedBody = bodies[movedId];
                    Debug.Assert(movedBody.localIndex == movedIndex);
                    movedBody.localIndex = awakeBodyIndex;
                }
                awakeSet.bodyStates.RemoveSwap(awakeBodyIndex);
                body.setIndex = sleepSetId;
                body.localIndex = sleepBodyIndex;
                int contactKey = body.headContactKey;
                while (contactKey != -1)
                {
                    int contactId = contactKey >> 1;
                    int edgeIndex = contactKey & 1;
                    Contact contact = contacts[contactId];
                    Debug.Assert(contact.setIndex == (int)SetType.Awake || contact.setIndex == (int)SetType.Disabled);
                    contactKey = edgeIndex == 1 ? contact.edge1.nextKey : contact.edge0.nextKey;
                    if (contact.setIndex == (int)SetType.Disabled) continue;
                    if (contact.colorIndex != -1)
                    {
                        Debug.Assert(contact.flags.HasFlag(ContactFlags.Touching));
                        continue;
                    }
                    int otherEdgeIndex = edgeIndex ^ 1;
                    int otherBodyId = otherEdgeIndex == 1 ? contact.edge1.bodyId : contact.edge0.bodyId;
                    Body otherBody = bodies[otherBodyId];
                    if (otherBody.setIndex == (int)SetType.Awake) continue;
                    int localIndex = contact.localIndex;
                    ContactSim contactSim = awakeSet.contactSims[localIndex];
                    Debug.Assert(contactSim.manifold.pointCount == 0);
                    Debug.Assert(!contact.flags.HasFlag(ContactFlags.Touching));
                    contact.setIndex = (int)SetType.Disabled;
                    contact.localIndex = disabledSet.contactSims.Count;
                    ContactSim disabledContactSim = contactSim with { };
                    disabledSet.contactSims.Add(disabledContactSim);
                    int movedLocalIndex = awakeSet.contactSims.RemoveSwap(localIndex);
                    if (movedLocalIndex != -1)
                    {
                        ContactSim movedContactSim = awakeSet.contactSims[localIndex];
                        Contact movedContact = contacts[movedContactSim.contactId];
                        Debug.Assert(movedContact.localIndex == movedLocalIndex);
                        movedContact.localIndex = localIndex;
                    }
                }
                bodyId = body.islandNext;
            }
        }
        {
            int contactId = island.headContact;
            while (contactId != -1)
            {
                Contact contact = contacts[contactId];
                Debug.Assert(contact.setIndex == (int)SetType.Awake);
                Debug.Assert(contact.islandId == islandId);
                int colorIndex = contact.colorIndex;
                Debug.Assert(0 <= colorIndex && colorIndex < Box2D.GraphColorCount);
                GraphColor color = constraintGraph.colors[colorIndex];
                if (colorIndex != Box2D.GraphColorCount - 1)
                {
                    color.bodySet.ClearBit(contact.edge0.bodyId);
                    color.bodySet.ClearBit(contact.edge1.bodyId);
                }
                int localIndex = contact.localIndex;
                ContactSim awakeContactSim = color.contactSims[localIndex];
                int sleepContactIndex = sleepSet.contactSims.Count;
                ContactSim sleepContactSim = awakeContactSim with { };
                sleepSet.contactSims.Add(sleepContactSim);
                int movedLocalIndex = color.contactSims.RemoveSwap(localIndex);
                if (movedLocalIndex != -1)
                {
                    ContactSim movedContactSim = color.contactSims[localIndex];
                    Contact movedContact = contacts[movedContactSim.contactId];
                    Debug.Assert(movedContact.localIndex == movedLocalIndex);
                    movedContact.localIndex = localIndex;
                }
                contact.setIndex = sleepSetId;
                contact.colorIndex = -1;
                contact.localIndex = sleepContactIndex;
                contactId = contact.islandNext;
            }
        }
        {
            int jointId = island.headJoint;
            while (jointId != -1)
            {
                Joint joint = joints[jointId];
                Debug.Assert(joint.setIndex == (int)SetType.Awake);
                Debug.Assert(joint.islandId == islandId);
                int colorIndex = joint.colorIndex;
                int localIndex = joint.localIndex;
                Debug.Assert(0 <= colorIndex && colorIndex < Box2D.GraphColorCount);
                GraphColor color = constraintGraph.colors[colorIndex];
                JointSim awakeJointSim = color.jointSims[localIndex];
                if (colorIndex != Box2D.GraphColorCount - 1)
                {
                    color.bodySet.ClearBit(joint.edge0.bodyId);
                    color.bodySet.ClearBit(joint.edge1.bodyId);
                }
                int sleepJointIndex = sleepSet.jointSims.Count;
                JointSim sleepJointSim = new(awakeJointSim);
                sleepSet.jointSims.Add(sleepJointSim);
                int movedIndex = color.jointSims.RemoveSwap(localIndex);
                if (movedIndex != -1)
                {
                    JointSim movedJointSim = color.jointSims[localIndex];
                    int movedId = movedJointSim.jointId;
                    Joint movedJoint = joints[movedId];
                    Debug.Assert(movedJoint.localIndex == movedIndex);
                    movedJoint.localIndex = localIndex;
                }
                joint.setIndex = sleepSetId;
                joint.colorIndex = -1;
                joint.localIndex = sleepJointIndex;
                jointId = joint.islandNext;
            }
        }
        {
            Debug.Assert(island.setIndex == (int)SetType.Awake);
            int islandIndex = island.localIndex;
            IslandSim sleepIsland = new(); sleepSet.islandSims.Add(sleepIsland);
            sleepIsland.islandId = islandId;
            int movedIslandIndex = awakeSet.islandSims.RemoveSwap(islandIndex);
            if (movedIslandIndex != -1)
            {
                IslandSim movedIslandSim = awakeSet.islandSims[islandIndex];
                int movedIslandId = movedIslandSim.islandId;
                Island movedIsland = islands[movedIslandId];
                Debug.Assert(movedIsland.localIndex == movedIslandIndex);
                movedIsland.localIndex = islandIndex;
            }
            island.setIndex = sleepSetId;
            island.localIndex = 0;
        }
        ValidateSolverSets();
    }
    public void MergeSolverSets(int setId1, int setId2)
    {
        Debug.Assert(setId1 >= (int)SetType.FirstSleeping);
        Debug.Assert(setId2 >= (int)SetType.FirstSleeping);
        SolverSet set1 = solverSets[setId1], set2 = solverSets[setId2];
        if (set1.bodySims.Count < set2.bodySims.Count)
            (set1, set2, setId1, setId2) = (set2, set1, setId2, setId1);
        for (int i = 0; i < set2.bodySims.Count; i++)
        {
            BodySim simSrc = set2.bodySims[i];
            Body body = bodies[simSrc.bodyId];
            body.setIndex = setId1;
            body.localIndex = set1.bodySims.Count;
            BodySim simDst = simSrc with { }; set1.bodySims.Add(simDst);
        }
        for (int i = 0; i < set2.contactSims.Count; i++)
        {
            ContactSim contactSrc = set2.contactSims[i];
            Contact contact = contacts[contactSrc.contactId];
            Debug.Assert(contact.setIndex == setId2);
            contact.setIndex = setId1;
            contact.localIndex = set1.contactSims.Count;
            ContactSim contactDst = contactSrc with { }; set1.contactSims.Add(contactDst);
        }
        for (int i = 0; i < set2.jointSims.Count; i++)
        {
            JointSim jointSrc = set2.jointSims[i];
            Joint joint = joints[jointSrc.jointId];
            Debug.Assert(joint.setIndex == setId2);
            joint.setIndex = setId1;
            joint.localIndex = set1.jointSims.Count;
            JointSim jointDst = new(jointSrc); set1.jointSims.Add(jointDst);
        }
        for (int i = 0; i < set2.islandSims.Count; i++)
        {
            IslandSim islandSrc = set2.islandSims[i];
            int islandId = islandSrc.islandId;
            Island island = islands[islandId];
            island.setIndex = setId1;
            island.localIndex = set1.islandSims.Count;
            IslandSim islandDst = islandSrc with { }; set1.islandSims.Add(islandDst);
        }
        DestroySolverSet(setId2);
        ValidateSolverSets();
    }
    public void TransferBody(SolverSet targetSet, SolverSet sourceSet, Body body)
    {
        if (targetSet == sourceSet) return;
        int sourceIndex = body.localIndex;
        BodySim sourceSim = sourceSet.bodySims[sourceIndex];
        int targetIndex = targetSet.bodySims.Count;
        BodySim targetSim = sourceSim with { }; targetSet.bodySims.Add(targetSim);
        targetSim.flags &= ~(BodyFlags.IsFast | BodyFlags.IsSpeedCapped | BodyFlags.HadTimeOfImpact);
        int movedIndex = sourceSet.bodySims.RemoveSwap(sourceIndex);
        if (movedIndex != -1)
        {
            BodySim movedSim = sourceSet.bodySims[sourceIndex];
            int movedId = movedSim.bodyId;
            Body movedBody = bodies[movedId];
            Debug.Assert(movedBody.localIndex == movedIndex);
            movedBody.localIndex = sourceIndex;
        }
        if (sourceSet.setIndex == (int)SetType.Awake) sourceSet.bodyStates.RemoveSwap(sourceIndex);
        else if (targetSet.setIndex == (int)SetType.Awake)
            targetSet.bodyStates.Add(new() { flags = body.flags });
        body.setIndex = targetSet.setIndex;
        body.localIndex = targetIndex;
    }
    public void TransferJoint(SolverSet targetSet, SolverSet sourceSet, Joint joint)
    {
        if (targetSet == sourceSet) return;
        int localIndex = joint.localIndex;
        int colorIndex = joint.colorIndex;
        JointSim sourceSim;
        if (sourceSet.setIndex == (int)SetType.Awake)
        {
            Debug.Assert(0 <= colorIndex && colorIndex < Box2D.GraphColorCount);
            GraphColor color = constraintGraph.colors[colorIndex];
            sourceSim = color.jointSims[localIndex];
        }
        else
        {
            Debug.Assert(colorIndex == -1);
            sourceSim = sourceSet.jointSims[localIndex];
        }
        if (targetSet.setIndex == (int)SetType.Awake)
        {
            AddJointToGraph(sourceSim, joint);
            joint.setIndex = (int)SetType.Awake;
        }
        else
        {
            joint.setIndex = targetSet.setIndex;
            joint.localIndex = targetSet.jointSims.Count;
            joint.colorIndex = -1;
            JointSim targetSim = new(sourceSim); targetSet.jointSims.Add(targetSim);
        }
        if (sourceSet.setIndex == (int)SetType.Awake)
            RemoveJointFromGraph(joint.edge0.bodyId, joint.edge1.bodyId, colorIndex, localIndex);
        else
        {
            int movedIndex = sourceSet.jointSims.RemoveSwap(localIndex);
            if (movedIndex != -1)
            {
                JointSim movedJointSim = sourceSet.jointSims[localIndex];
                int movedId = movedJointSim.jointId;
                Joint movedJoint = joints[movedId];
                movedJoint.localIndex = localIndex;
            }
        }
    }
}