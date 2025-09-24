namespace Box2D;

/// <summary>World id references a world instance. This should be treated as an opaque handle.</summary>
public struct WorldID
{
    public World index1; public uint padding; public uint generation;
    public bool IsNull() => index1 == null;
    public static bool operator ==(WorldID left, WorldID right) { return left.index1 == right.index1 && left.generation == right.generation; }
    public static bool operator !=(WorldID left, WorldID right) { return left.index1 != right.index1 || left.generation != right.generation; }
}
/// <summary>Body id references a body instance. This should be treated as an opaque handle.</summary>
public struct BodyID
{
    public int index1; public World world0; public uint generation;
    public bool IsNull() => index1 == 0;
    public static bool operator ==(BodyID left, BodyID right) { return left.index1 == right.index1 && left.world0 == right.world0 && left.generation == right.generation; }
    public static bool operator !=(BodyID left, BodyID right) { return left.index1 != right.index1 || left.world0 != right.world0 || left.generation != right.generation; }
}
/// <summary>Shape id references a shape instance. This should be treated as an opaque handle.</summary>
public struct ShapeID
{
    public int index1; public World world0; public uint generation;
    public bool IsNull() => index1 == 0;
    public static bool operator ==(ShapeID left, ShapeID right) { return left.index1 == right.index1 && left.world0 == right.world0 && left.generation == right.generation; }
    public static bool operator !=(ShapeID left, ShapeID right) { return left.index1 != right.index1 || left.world0 != right.world0 || left.generation != right.generation; }
}
/// <summary>Chain id references a chain instances. This should be treated as an opaque handle.</summary>
public struct ChainID
{
    public int index1; public World world0; public uint generation;
    public bool IsNull() => index1 == 0;
    public static bool operator ==(ChainID left, ChainID right) { return left.index1 == right.index1 && left.world0 == right.world0 && left.generation == right.generation; }
    public static bool operator !=(ChainID left, ChainID right) { return left.index1 != right.index1 || left.world0 != right.world0 || left.generation != right.generation; }
}
/// <summary>Joint id references a joint instance. This should be treated as an opaque handle.</summary>
public struct JointID
{
    public int index1; public World world0; public uint generation;
    public bool IsNull() => index1 == 0;
    public static bool operator ==(JointID left, JointID right) { return left.index1 == right.index1 && left.world0 == right.world0 && left.generation == right.generation; }
    public static bool operator !=(JointID left, JointID right) { return left.index1 != right.index1 || left.world0 != right.world0 || left.generation != right.generation; }
}
/// <summary>Contact id references a contact instance. This should be treated as an opaque handled.</summary>
public struct ContactID
{
    public int index1; public World world0; public uint generation;
    public bool IsNull() => index1 == 0;
    public static bool operator ==(ContactID left, ContactID right) { return left.index1 == right.index1 && left.world0 == right.world0 && left.generation == right.generation; }
    public static bool operator !=(ContactID left, ContactID right) { return left.index1 != right.index1 || left.world0 != right.world0 || left.generation != right.generation; }
}