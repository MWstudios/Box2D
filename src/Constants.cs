using System;

namespace Box2D;
public static partial class Box2D
{
    /// <summary>Used to detect bad values. Positions greater than about 16km will have precision
    /// problems, so 100km as a limit should be fine in all cases.</summary>
    public static float Huge = 100000 * LengthUnitsPerMeter;
    /// <summary>Maximum parallel workers. Used to size some static arrays.</summary>
    public static int MaxWorkers = 64;
    /// <summary>Maximum number of colors in the constraint graph. Constraints that cannot
    /// find a color are added to the overflow set which are solved single-threaded.</summary>
    public static int GraphColorCount = 24;
    /// <summary>A small length used as a collision and constraint tolerance. Usually it is
    /// chosen to be numerically significant, but visually insignificant. In meters.
    /// Normally this is 0.5cm.</summary>
    ///<remarks>modifying this can have a significant impact on stability</remarks>
    public static float LinearSlop = 0.005f * LengthUnitsPerMeter;
    /// <summary>The maximum rotation of a body per time step. This limit is very large and is used
    /// to prevent numerical problems. You shouldn't need to adjust this.</summary>
    /// <remarks>increasing this to 0.5f * b2_pi or greater will break continuous collision.</remarks>
    public static float MaxRotation = 0.25f * MathF.PI;
    /// <summary>Box2D uses limited speculative collision. This reduces jitter.
    ///Normally this is 2cm.</summary>
    ///<remarks>modifying this can have a significant impact on performance and stability</remarks>
    public static float SpeculativeDistance = 4 * LinearSlop;
    /// <summary>This is used to fatten AABBs in the dynamic tree. This allows proxies
    /// to move by a small amount without triggering a tree adjustment. This is in meters.
    /// Normally this is 5cm.</summary>
    /// <remarks> modifying this can have a significant impact on performance</remarks>
    public static float AABBMargin = 0.05f * LengthUnitsPerMeter;
    /// <summary>The time that a body must be still before it will go to sleep. In seconds.</summary>
    public static float TimeToSleep = 0.5f;
    /// <summary>This keeps constraints involving two dynamic bodies at a lower solver priority than constraints
    /// involving a dynamic and static bodies. This reduces tunneling due to push through.</summary>
    public static int DynamicColorCount = 20;
    [Flags] public enum TreeNodeFlags
    {
        Allocated = 1, Enlarged = 2, Leaf = 4
    }
}
