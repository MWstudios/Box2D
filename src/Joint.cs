using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace Box2D;

/// <summary>A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.</summary>
public struct JointEdge
{
    public int bodyId;
    public int prevKey;
    public int nextKey;
}
public class Joint
{
    public object userData;

    // index of simulation set stored in b2World
    // B2_NULL_INDEX when slot is free
    public int setIndex;

    // index into the constraint graph color array, may be B2_NULL_INDEX for sleeping/disabled joints
    // B2_NULL_INDEX when slot is free
    public int colorIndex;

    // joint index within set or graph color
    // B2_NULL_INDEX when slot is free
    public int localIndex;

    public JointEdge edge0, edge1;

    public int jointId;
    public int islandId;
    public int islandPrev;
    public int islandNext;

    public float drawScale;

    public JointType type;

    // This is monotonically advanced when a body is allocated in this slot
    // Used to check for invalid b2JointId
    public ushort generation;

    public bool collideConnected;
}
public interface IJoint
{
    public void GetReaction(out float linearImpulse, out float angularImpulse)
    { linearImpulse = 0; angularImpulse = 0; }
    public float GetLinearSeparation(Transform xfA, Transform xfB, Vector2 dp) => 0;
    public float GetAngularSeparation(float relativeAngle) => 0;
    public Vector2 GetForce(World world, JointSim base_) => new();
    public float GetTorque(World world, JointSim base_) => 0;
    public void Prepare(JointSim joint, StepContext context) { }
    public void WarmStart(JointSim joint, StepContext context) { }
    public void Solve(JointSim joint, StepContext context, bool useBias) { }
    public void Draw(DebugDraw draw, JointSim jointSim, Transform transformA, Transform transformB,
        Vector2 pA, Vector2 pB, float drawSize, HexColor color)
    {
        draw.DrawSegmentFcn(transformA.p, pA, color, draw.context);
        draw.DrawSegmentFcn(pA, pB, color, draw.context);
        draw.DrawSegmentFcn(transformB.p, pB, color, draw.context);
    }
    public IJoint Copy();
}

/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
public record class JointSim
{
    public int jointId;

    public int bodyIdA;
    public int bodyIdB;

    public JointType type;

    public Transform localFrameA;
    public Transform localFrameB;

    public float invMassA, invMassB;
    public float invIA, invIB;

    public float constraintHertz;
    public float constraintDampingRatio;

    public Softness constraintSoftness;

    public float forceThreshold;
    public float torqueThreshold;

    public IJoint joint;

    public JointSim(JointSim other)
    {
        jointId = other.jointId;
        bodyIdA = other.bodyIdA;
        bodyIdB = other.bodyIdB;
        type = other.type;
        localFrameA = other.localFrameA;
        localFrameB = other.localFrameB;
        invMassA = other.invMassA;
        invMassB = other.invMassB;
        invIA = other.invIA;
        invIB = other.invIB;
        constraintHertz = other.constraintHertz;
        constraintDampingRatio = other.constraintDampingRatio;
        constraintSoftness = other.constraintSoftness;
        forceThreshold = other.forceThreshold;
        torqueThreshold = other.torqueThreshold;
        joint = other.joint?.Copy();
    }

    public void GetJointReaction(float invTimeStep, out float force, out float torque)
    {
        joint.GetReaction(out float linearImpulse, out float angularImpulse);
        force = linearImpulse * invTimeStep;
        torque = angularImpulse * invTimeStep;
    }
    public void PrepareJoint(StepContext context)
    {
        float hertz = Math.Min(constraintHertz, 0.25f * context.inv_h);
        constraintSoftness = new(hertz, constraintDampingRatio, context.h);
        joint.Prepare(this, context);
    }
    public void WarmStart(StepContext context) => joint.WarmStart(this, context);
    public void Solve(StepContext context, bool useBias) => joint.Solve(this, context, useBias);
}
public struct JointPair { public Joint joint; public JointSim jointSim; }
public partial class World
{
    public Joint GetJointFullID(JointID jointId)
    {
        int id = jointId.index1 - 1;
        Joint joint = joints[id];
        Debug.Assert(joint.jointId == id && joint.generation == jointId.generation);
        return joint;
    }
    public JointSim GetJointSim(Joint joint)
    {
        if (joint.setIndex == (int)SetType.Awake)
        {
            Debug.Assert(0 <= joint.colorIndex && joint.colorIndex <= Box2D.GraphColorCount);
            GraphColor color = constraintGraph.colors[joint.colorIndex];
            return color.jointSims[joint.localIndex];
        }
        SolverSet set = solverSets[joint.setIndex];
        return set.jointSims[joint.localIndex];
    }
    public void DestroyContactsBetweeenBodies(Body bodyA, Body bodyB)
    {
        int contactKey, otherBodyId;
        if (bodyA.contactCount < bodyB.contactCount)
        {
            contactKey = bodyA.headContactKey;
            otherBodyId = bodyB.id;
        }
        else
        {
            contactKey = bodyB.headContactKey;
            otherBodyId = bodyA.id;
        }
        bool wakeBodies = false;
        while (contactKey != -1)
        {
            int contactId = contactKey >> 1;
            int edgeIndex = contactKey & 1;
            Contact contact = contacts[contactId];
            contactKey = edgeIndex == 1 ? contact.edge1.nextKey : contact.edge0.nextKey;
            int otherEdgeIndex = edgeIndex ^ 1;
            if ((otherEdgeIndex == 1 ? contact.edge1.bodyId : contact.edge0.bodyId) == otherBodyId)
                DestroyContact(contact, wakeBodies);
        }
        ValidateSolverSets();
    }
    public JointPair CreateJoint(ref JointDef def, JointType type)
    {
        Debug.Assert(def.localFrameA.IsValid());
        Debug.Assert(def.localFrameB.IsValid());
        Debug.Assert(this == def.bodyIdA.world0);
        Debug.Assert(this == def.bodyIdB.world0);
        Debug.Assert(def.bodyIdA != def.bodyIdB);
        Body bodyA = GetBodyFullID(def.bodyIdA);
        Body bodyB = GetBodyFullID(def.bodyIdB);
        int bodyIdA = bodyA.id, bodyIdB = bodyB.id;
        int maxSetindex = Math.Max(bodyA.setIndex, bodyB.setIndex);
        int jointId = jointIdPool.AllocId();
        if (jointId == joints.Count)
        {
            joints.Add(new());
        }
        Joint joint = joints[jointId];
        joint.jointId = jointId;
        joint.userData = def.userData;
        joint.generation++;
        joint.setIndex = -1;
        joint.colorIndex = -1;
        joint.localIndex = -1;
        joint.islandId = -1;
        joint.islandPrev = -1;
        joint.islandNext = -1;
        joint.drawScale = def.drawScale;
        joint.type = type;
        joint.collideConnected = def.collideConnected;
        joint.edge0.bodyId = bodyIdA;
        joint.edge0.prevKey = -1;
        joint.edge0.nextKey = bodyA.headJointKey;
        int keyA = (jointId << 1);
        if (bodyA.headJointKey != -1)
        {
            Joint jointA = joints[bodyA.headJointKey >> 1];
            if ((bodyA.headJointKey & 1) != 0) jointA.edge1.prevKey = keyA;
            else jointA.edge0.prevKey = keyA;
        }
        bodyA.headJointKey = keyA;
        bodyA.jointCount++;
        joint.edge1.bodyId = bodyIdB;
        joint.edge1.prevKey = -1;
        joint.edge1.nextKey = bodyB.headJointKey;
        int keyB = (jointId << 1) | 1;
        if (bodyB.headJointKey != -1)
        {
            Joint jointB = joints[bodyB.headJointKey >> 1];
            if ((bodyA.headJointKey & 1) != 0) jointB.edge1.prevKey = keyB;
            else jointB.edge0.prevKey = keyB;
        }
        bodyB.headJointKey = keyB;
        bodyB.jointCount++;
        JointSim jointSim;
        if (bodyA.setIndex == (int)SetType.Disabled || bodyB.setIndex == (int)SetType.Disabled)
        {
            SolverSet set = solverSets[(int)SetType.Disabled];
            joint.setIndex = (int)SetType.Disabled;
            joint.localIndex = set.jointSims.Count;
            set.jointSims.Add(jointSim = new() { jointId = jointId, bodyIdA = bodyIdA, bodyIdB = bodyIdB });
        }
        else if (bodyA.type != BodyType.Dynamic && bodyB.type != BodyType.Dynamic)
        {
            SolverSet set = solverSets[(int)SetType.Static];
            joint.setIndex = (int)SetType.Static;
            joint.localIndex = set.jointSims.Count;
            set.jointSims.Add(jointSim = new() { jointId = jointId, bodyIdA = bodyIdA, bodyIdB = bodyIdB });
        }
        else if (bodyA.setIndex == (int)SetType.Awake || bodyB.setIndex == (int)SetType.Awake)
        {
            if (maxSetindex >= (int)SetType.FirstSleeping) WakeSolverSet(maxSetindex);
            joint.setIndex = (int)SetType.Awake;
            jointSim = CreateJointInGraph(joint, new() { jointId = jointId, bodyIdA = bodyIdA, bodyIdB = bodyIdB });
        }
        else
        {
            Debug.Assert(bodyA.setIndex >= (int)SetType.FirstSleeping || bodyB.setIndex >= (int)SetType.FirstSleeping);
            Debug.Assert(bodyA.setIndex != (int)SetType.Static || bodyB.setIndex != (int)SetType.Static);
            int setIndex = maxSetindex;
            SolverSet set = solverSets[setIndex];
            joint.setIndex = setIndex;
            joint.localIndex = set.jointSims.Count;
            set.jointSims.Add(jointSim = new() { jointId = jointId, bodyIdA = bodyIdA, bodyIdB = bodyIdB });
            if (bodyA.setIndex != bodyB.setIndex && bodyA.setIndex >= (int)SetType.FirstSleeping && bodyB.setIndex >= (int)SetType.FirstSleeping)
            {
                MergeSolverSets(bodyA.setIndex, bodyB.setIndex);
                Debug.Assert(bodyA.setIndex == bodyB.setIndex);
                setIndex = bodyA.setIndex;
                SolverSet mergedSet = solverSets[setIndex];
                jointSim = mergedSet.jointSims[joint.localIndex];
            }
            Debug.Assert(joint.setIndex == setIndex);
        }
        jointSim.localFrameA = def.localFrameA;
        jointSim.localFrameB = def.localFrameB;
        jointSim.type = type;
        jointSim.constraintHertz = def.constraintHertz;
        jointSim.constraintDampingRatio = def.constraintDampingRatio;
        jointSim.constraintSoftness = new()
        {
            biasRate = 0,
            massScale = 1,
            impulseScale = 0
        };
        Debug.Assert(float.IsFinite(def.forceThreshold) && def.forceThreshold >= 0);
        Debug.Assert(float.IsFinite(def.torqueThreshold) && def.torqueThreshold >= 0);
        jointSim.forceThreshold = def.forceThreshold;
        jointSim.torqueThreshold = def.torqueThreshold;
        Debug.Assert(jointSim.jointId == jointId);
        Debug.Assert(jointSim.bodyIdA == bodyIdA);
        Debug.Assert(jointSim.bodyIdB == bodyIdB);
        if (joint.setIndex > (int)SetType.Disabled) LinkJoint(joint);
        if (!def.collideConnected) DestroyContactsBetweeenBodies(bodyA, bodyB);
        ValidateSolverSets();
        return new() { joint = joint, jointSim = jointSim };
    }
    public void DestroyJointInternal(Joint joint, bool wakeBodies)
    {
        int jointId = joint.jointId;
        JointEdge edgeA = joint.edge0;
        JointEdge edgeB = joint.edge1;
        int idA = edgeA.bodyId, idB = edgeB.bodyId;
        Body bodyA = bodies[idA], bodyB = bodies[idB];
        if (edgeA.prevKey != -1)
        {
            Joint prevJoint = joints[edgeA.prevKey >> 1];
            if ((edgeA.prevKey & 1) != 0) prevJoint.edge1.nextKey = edgeA.nextKey;
            else prevJoint.edge0.nextKey = edgeA.nextKey;
        }
        if (edgeA.nextKey != -1)
        {
            Joint nextJoint = joints[edgeA.nextKey >> 1];
            if ((edgeA.nextKey & 1) != 0) nextJoint.edge1.prevKey = edgeA.prevKey;
            else nextJoint.edge0.prevKey = edgeA.prevKey;
        }
        int edgeKeyA = jointId << 1;
        if (bodyA.headJointKey == edgeKeyA) bodyA.headJointKey = edgeA.nextKey;
        bodyA.jointCount--;
        if (edgeB.prevKey != -1)
        {
            Joint prevJoint = joints[edgeB.prevKey >> 1];
            if ((edgeB.prevKey & 1) != 0) prevJoint.edge1.nextKey = edgeB.nextKey;
            else prevJoint.edge0.nextKey = edgeB.nextKey;
        }
        if (edgeB.nextKey != -1)
        {
            Joint nextJoint = joints[edgeB.nextKey >> 1];
            if ((edgeB.nextKey & 1) != 0) nextJoint.edge1.prevKey = edgeB.prevKey;
            else nextJoint.edge0.prevKey = edgeB.prevKey;
        }
        int edgeKeyB = (jointId << 1) | 1;
        if (bodyB.headJointKey == edgeKeyB) bodyB.headJointKey = edgeB.nextKey;
        bodyB.jointCount--;
        if (joint.islandId != -1)
        {
            Debug.Assert(joint.setIndex > (int)SetType.Disabled);
            UnlinkJoint(joint);
        }
        else Debug.Assert(joint.setIndex <= (int)SetType.Disabled);
        int setIndex = joint.setIndex;
        int localIndex = joint.localIndex;
        if (setIndex == (int)SetType.Awake)
            RemoveJointFromGraph(joint.edge0.bodyId, joint.edge1.bodyId, joint.colorIndex, localIndex);
        else
        {
            SolverSet set = solverSets[setIndex];
            int movedIndex = set.jointSims.RemoveSwap(localIndex);
            if (movedIndex != -1)
            {
                JointSim movedJointSim = set.jointSims[localIndex];
                int movedid = movedJointSim.jointId;
                Joint movedJoint = joints[movedid];
                Debug.Assert(movedJoint.localIndex == movedIndex);
                movedJoint.localIndex = localIndex;
            }
        }
        joint.setIndex = -1;
        joint.localIndex = -1;
        joint.colorIndex = -1;
        joint.jointId = -1;
        jointIdPool.FreeId(jointId);
        if (wakeBodies)
        {
            WakeBody(bodyA); WakeBody(bodyB);
        }
        ValidateSolverSets();
    }
    public Vector2 GetJointConstraintForce(Joint joint)
    {
        JointSim base_ = GetJointSim(joint); return base_.joint.GetForce(this, base_);
    }
    public float GetJointConstraintTorque(Joint joint)
    {
        JointSim base_ = GetJointSim(joint); return base_.joint.GetTorque(this, base_);
    }
}
public partial class StepContext
{
    public void PrepareOverflowJoints()
    {
        List<JointSim> joints = graph.colors[Box2D.GraphColorCount - 1].jointSims;
        for (int i = 0; i < joints.Count; i++) joints[i].PrepareJoint(this);
    }
    public void WarmStartOverflowJoints()
    {
        List<JointSim> joints = graph.colors[Box2D.GraphColorCount - 1].jointSims;
        for (int i = 0; i < joints.Count; i++) joints[i].WarmStart(this);
    }
    public void SolveOverflowJoints(bool useBias)
    {
        List<JointSim> joints = graph.colors[Box2D.GraphColorCount - 1].jointSims;
        for (int i = 0; i < joints.Count; i++) joints[i].Solve(this, useBias);
    }
}
public partial class DebugDraw
{
    public void DrawJoint(World world, Joint joint)
    {
        Body bodyA = world.bodies[joint.edge0.bodyId], bodyB = world.bodies[joint.edge1.bodyId];
        if (bodyA.setIndex == (int)SetType.Disabled || bodyB.setIndex == (int)SetType.Disabled) return;
        JointSim jointSim = world.GetJointSim(joint);
        Transform transformA = world.GetBodyTransformQuick(bodyA);
        Transform transformB = world.GetBodyTransformQuick(bodyB);
        Vector2 pA = transformA.TransformPoint(jointSim.localFrameA.p);
        Vector2 pB = transformB.TransformPoint(jointSim.localFrameB.p);
        jointSim.joint.Draw(this, jointSim, transformA, transformB, pA, pB, joint.drawScale, HexColor.DarkSeaGreen);
        if (drawGraphColors)
        {
            HexColor[] graphColors =
            [
                HexColor.Red, HexColor.Orange, HexColor.Yellow, HexColor.Green, HexColor.Cyan, HexColor.Blue,
                HexColor.Violet, HexColor.Pink, HexColor.Chocolate, HexColor.GoldenRod, HexColor.Coral, HexColor.RosyBrown,
                HexColor.Aqua, HexColor.Peru, HexColor.Lime, HexColor.Gold, HexColor.Plum, HexColor.Snow,
                HexColor.Teal, HexColor.Khaki, HexColor.Salmon, HexColor.PeachPuff, HexColor.HoneyDew, HexColor.Black
            ];
            int colorIndex = joint.colorIndex;
            if (colorIndex != -1)
                DrawPointFcn(Vector2.Lerp(pA, pB, 0.5f), 5, graphColors[colorIndex], context);
        }
        if (drawJointExtras)
        {
            Vector2 force = world.GetJointConstraintForce(joint);
            float torque = world.GetJointConstraintTorque(joint);
            Vector2 p = Vector2.Lerp(pA, pB, 0.5f);
            DrawSegmentFcn(p, Vector2.MulAdd(p, 0.001f, force), HexColor.Azure, context);
            DrawStringFcn(p, $"f = [{force.x}, {force.y}], t = {torque}", HexColor.Azure, context);
        }
    }
}