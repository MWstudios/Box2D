using System.Diagnostics;

namespace Box2D;

public record class FilterJoint : IJoint
{
    public static JointID Create(WorldID worldId, ref FilterJointDef def)
    {
        Debug.Assert(def.internalValue == Box2D.SECRET_COOKIE);
        World world = worldId.index1;
        Debug.Assert(!world.locked);
        if (world.locked) return new();
        JointPair pair = world.CreateJoint(ref def.base_, JointType.Filter);
        pair.jointSim.joint = new FilterJoint();
        return new() { index1 = pair.jointSim.jointId + 1, world0 = world, generation = pair.joint.generation };
    }
    public IJoint Copy() => new FilterJoint(this);
}
