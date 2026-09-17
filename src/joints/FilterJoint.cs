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
    public void HashStateDeep(ref ulong hash) { }
    public void Draw(DebugDraw draw, JointSim jointSim, WorldTransform transformA, WorldTransform transformB,
        Position pA, Position pB, float drawScale, HexColor color)
    {
        draw.DrawPointFcn(pA, 8, HexColor.LightSkyBlue, draw.context);
        draw.DrawPointFcn(pB, 8, HexColor.LightSkyBlue, draw.context);
        if (draw.drawJointExtras) draw.DrawLineFcn(pA, pB, HexColor.Gold, draw.context);
    }
    public IJoint Copy() => new FilterJoint(this);
}
