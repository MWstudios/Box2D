using System;
using System.Diagnostics;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics.X86;
using System.Runtime.Intrinsics.Arm;

namespace Box2D;

public class GraphColor
{
    /// <summary>This bitset is indexed by bodyId so this is over-sized to encompass static bodies
    /// however I never traverse these bits or use the bit count for anything
    /// This bitset is unused on the overflow color.
    /// todo consider having a uint_16 per body that tracks the graph color membership</summary>
    public BitSet bodySet;
    /// <summary>cache friendly arrays</summary>
    public List<ContactSim> contactSims = new();
    public List<JointSim> jointSims = new();
    /// <summary>transient</summary>
    public ContactConstraint[] overflowConstraints = null;
    public IContactConstraintsSIMD simdConstraints = null;
    public GraphColor() { }
}
public interface IContactConstraintsSIMD
{
    public static unsafe IContactConstraintsSIMD Alloc(int length)
    {
        if (Avx.IsSupported) return new ContactConstraintsAVX
        { simdConstraints = (ContactSolverAVX.ContactConstraintSIMD*)NativeMemory.AlignedAlloc((nuint)(sizeof(ContactSolverAVX.ContactConstraintSIMD) * length), 32), owns = true };
        if (AdvSimd.IsSupported) return new ContactConstraintsNeon
        { simdConstraints = (ContactSolverNeon.ContactConstraintSIMD*)NativeMemory.AlignedAlloc((nuint)(sizeof(ContactSolverNeon.ContactConstraintSIMD) * length), 32), owns = true };
        if (Sse.IsSupported) return new ContactConstraintsSSE
        { simdConstraints = (ContactSolverSSE.ContactConstraintSIMD*)NativeMemory.AlignedAlloc((nuint)(sizeof(ContactSolverSSE.ContactConstraintSIMD) * length), 32), owns = true };
        return new ContactConstraintsFloat
        { simdConstraints = (ContactSolverFloat.ContactConstraintSIMD*)NativeMemory.AlignedAlloc((nuint)(sizeof(ContactSolverFloat.ContactConstraintSIMD) * length), 32), owns = true };
    }
    public unsafe IContactConstraintsSIMD PointTo(int offset);
    public void Free() { }
}
public unsafe struct ContactConstraintsAVX : IContactConstraintsSIMD
{
    public ContactSolverAVX.ContactConstraintSIMD* simdConstraints;
    public bool owns;
    public IContactConstraintsSIMD PointTo(int offset) => new ContactConstraintsAVX { simdConstraints = simdConstraints + offset };
    public void Free() { if (owns) NativeMemory.AlignedFree(simdConstraints); }
}
public unsafe struct ContactConstraintsNeon : IContactConstraintsSIMD
{
    public ContactSolverNeon.ContactConstraintSIMD* simdConstraints;
    public bool owns;
    public IContactConstraintsSIMD PointTo(int offset) => new ContactConstraintsNeon { simdConstraints = simdConstraints + offset };
    public void Free() { if (owns) NativeMemory.AlignedFree(simdConstraints); }
}
public unsafe struct ContactConstraintsSSE : IContactConstraintsSIMD
{
    public ContactSolverSSE.ContactConstraintSIMD* simdConstraints;
    public bool owns;
    public IContactConstraintsSIMD PointTo(int offset) => new ContactConstraintsSSE { simdConstraints = simdConstraints + offset };
    public void Free() { if (owns) NativeMemory.AlignedFree(simdConstraints); }
}
public unsafe struct ContactConstraintsFloat : IContactConstraintsSIMD
{
    public ContactSolverFloat.ContactConstraintSIMD* simdConstraints;
    public bool owns;
    public IContactConstraintsSIMD PointTo(int offset) => new ContactConstraintsFloat { simdConstraints = simdConstraints + offset };
    public void Free() { if (owns) NativeMemory.AlignedFree(simdConstraints); }
}
public class ConstraintGraph
{
    /// <summary>including overflow at the end</summary>
    public GraphColor[] colors = new GraphColor[Box2D.GraphColorCount];
    public ConstraintGraph(int bodyCapacity)
    {
        Debug.Assert(Box2D.GraphColorCount >= 2, "must have at least two constraint graph colors");
        bodyCapacity = Math.Max(bodyCapacity, 8);
        for (int i = 0; i < Box2D.GraphColorCount - 1; i++)
        {
            colors[i] = new() { bodySet = new((uint)bodyCapacity) };
            colors[i].bodySet.SetBitCountAndClear(bodyCapacity);
        }
        colors[^1] = new() { bodySet = new(0) };
    }
    public void Destroy()
    {
        for (int i = 0; i < Box2D.GraphColorCount - 1; i++)
        {
            colors[i].bodySet.Destroy();
        }
    }
    public int AssignJointColor(int bodyIdA, int bodyIdB, BodyType typeA, BodyType typeB)
    {
        Debug.Assert(typeA == BodyType.Dynamic || typeB == BodyType.Dynamic);

        if (typeA != BodyType.Static && typeB != BodyType.Static)
        {
            for (int i = 0; i < Box2D.DynamicColorCount; i++)
            {
                GraphColor color = colors[i];
                if (color.bodySet.GetBit(bodyIdA) || color.bodySet.GetBit(bodyIdB))
                    continue;
                if (typeA == BodyType.Dynamic) color.bodySet.SetBitGrow(bodyIdA);
                if (typeB == BodyType.Dynamic) color.bodySet.SetBitGrow(bodyIdB);
                return i;
            }
        }
        else if (typeA == BodyType.Dynamic)
        {
            for (int i = Box2D.GraphColorCount - 2; i >= 1; i--)
            {
                GraphColor color = colors[i];
                if (color.bodySet.GetBit(bodyIdA)) continue;
                color.bodySet.SetBitGrow(bodyIdA);
                return i;
            }
        }
        else if (typeB == BodyType.Dynamic)
        {
            for (int i = Box2D.GraphColorCount - 2; i >= 1; i--)
            {
                GraphColor color = colors[i];
                if (color.bodySet.GetBit(bodyIdB)) continue;
                color.bodySet.SetBitGrow(bodyIdB);
                return i;
            }
        }
        return Box2D.GraphColorCount - 1;
    }
}
public partial class World
{
    /// <summary>Contacts are always created as non-touching. They get cloned into the constraint
    /// graph once they are found to be touching.
    /// todo maybe kinematic bodies should not go into graph</summary>
    public void AddContactToGraph(ContactSim contactSim, Contact contact)
    {
        Debug.Assert(contactSim.manifold.pointCount > 0);
        Debug.Assert(contactSim.simFlags.HasFlag(ContactSimFlags.Touching));
        Debug.Assert(contact.flags.HasFlag(ContactFlags.Touching));
        int colorIndex = Box2D.GraphColorCount - 1;
        int bodyIdA = contact.edge0.bodyId, bodyIdB = contact.edge1.bodyId;
        BodyType typeA = bodies[bodyIdA].type, typeB = bodies[bodyIdB].type;
        Debug.Assert(typeA == BodyType.Dynamic || typeB == BodyType.Dynamic);

        if (typeA != BodyType.Static && typeB != BodyType.Static)
        {
            for (int i = 0; i < Box2D.DynamicColorCount; i++)
            {
                GraphColor color = constraintGraph.colors[i];
                if (color.bodySet.GetBit(bodyIdA) || color.bodySet.GetBit(bodyIdB))
                    continue;
                if (typeA == BodyType.Dynamic) color.bodySet.SetBitGrow(bodyIdA);
                if (typeB == BodyType.Dynamic) color.bodySet.SetBitGrow(bodyIdB);
                colorIndex = i;
                break;
            }
        }
        else if (typeA == BodyType.Dynamic)
        {
            for (int i = Box2D.GraphColorCount - 2; i >= 1; i--)
            {
                GraphColor color = constraintGraph.colors[i];
                if (color.bodySet.GetBit(bodyIdA)) continue;
                color.bodySet.SetBitGrow(bodyIdA);
                colorIndex = i;
                break;
            }
        }
        else if (typeB == BodyType.Dynamic)
        {
            for (int i = Box2D.GraphColorCount - 2; i >= 1; i--)
            {
                GraphColor color = constraintGraph.colors[i];
                if (color.bodySet.GetBit(bodyIdB)) continue;
                color.bodySet.SetBitGrow(bodyIdB);
                colorIndex = i;
                break;
            }
        }

        {
            GraphColor color = constraintGraph.colors[colorIndex];
            contact.colorIndex = colorIndex;
            contact.localIndex = color.contactSims.Count;
            color.contactSims.Add(contactSim with { });
            ContactSim newContact = color.contactSims[^1];
            if (typeA == BodyType.Static)
            {
                newContact.bodySimIndexA = -1;
                newContact.invMassA = 0;
                newContact.invIA = 0;
            }
            else
            {
                Debug.Assert(bodies[bodyIdA].setIndex == (int)SetType.Awake);
                SolverSet awakeSet = solverSets[(int)SetType.Awake];
                int localIndex = bodies[bodyIdA].localIndex;
                newContact.bodySimIndexA = localIndex;
                BodySim bodySimA = awakeSet.bodySims[localIndex];
                newContact.invMassA = bodySimA.invMass;
                newContact.invIA = bodySimA.invInertia;
            }
            if (typeB == BodyType.Static)
            {
                newContact.bodySimIndexB = -1;
                newContact.invMassB = 0;
                newContact.invIB = 0;
            }
            else
            {
                Debug.Assert(bodies[bodyIdB].setIndex == (int)SetType.Awake);
                SolverSet awakeSet = solverSets[(int)SetType.Awake];
                int localIndex = bodies[bodyIdB].localIndex;
                newContact.bodySimIndexB = localIndex;
                BodySim bodySimB = awakeSet.bodySims[localIndex];
                newContact.invMassB = bodySimB.invMass;
                newContact.invIB = bodySimB.invInertia;
            }
        }
    }
    public void RemoveContactFromGraph(int bodyIdA, int bodyIdB, int colorIndex, int localIndex)
    {
        Debug.Assert(0 <= colorIndex && colorIndex < Box2D.GraphColorCount);
        GraphColor color = constraintGraph.colors[colorIndex];
        if (colorIndex != Box2D.GraphColorCount - 1)
        {
            color.bodySet.ClearBit(bodyIdA);
            color.bodySet.ClearBit(bodyIdB);
        }
        int movedIndex = color.contactSims.RemoveSwap(localIndex);
        if (movedIndex != -1)
        {
            ContactSim movedContactSim = color.contactSims[localIndex];
            int movedId = movedContactSim.contactId;
            Contact movedContact = contacts[movedId];
            Debug.Assert(movedContact.setIndex == (int)SetType.Awake);
            Debug.Assert(movedContact.colorIndex == colorIndex);
            Debug.Assert(movedContact.localIndex == movedIndex);
            movedContact.localIndex = localIndex;
        }
    }
    public JointSim CreateJointInGraph(Joint joint, JointSim template)
    {
        int bodyIdA = joint.edge0.bodyId, bodyIdB = joint.edge1.bodyId;
        Body bodyA = bodies[bodyIdA], bodyB = bodies[bodyIdB];
        int colorIndex = constraintGraph.AssignJointColor(bodyIdA, bodyIdB, bodyA.type, bodyB.type);
        JointSim jointSim = new(template); constraintGraph.colors[colorIndex].jointSims.Add(jointSim);
        joint.colorIndex = colorIndex;
        joint.localIndex = constraintGraph.colors[colorIndex].jointSims.Count - 1;
        return jointSim;
    }
    public void AddJointToGraph(JointSim jointSim, Joint joint)
    {
        JointSim jointDst = CreateJointInGraph(joint, jointSim);
    }
    public void RemoveJointFromGraph(int bodyIdA, int bodyIdB, int colorIndex, int localIndex)
    {
        Debug.Assert(0 <= colorIndex && colorIndex < Box2D.GraphColorCount);
        GraphColor color = constraintGraph.colors[colorIndex];
        if (colorIndex != Box2D.GraphColorCount - 1)
        {
            color.bodySet.ClearBit(bodyIdA);
            color.bodySet.ClearBit(bodyIdB);
        }
        int movedIndex = color.jointSims.RemoveSwap(localIndex);
        if (movedIndex != -1)
        {
            JointSim movedJointSim = color.jointSims[localIndex];
            int movedId = movedJointSim.jointId;
            Joint movedJoint = joints[movedId];
            Debug.Assert(movedJoint.setIndex == (int)SetType.Awake);
            Debug.Assert(movedJoint.colorIndex == colorIndex);
            Debug.Assert(movedJoint.localIndex == movedIndex);
            movedJoint.localIndex = localIndex;
        }
    }
}