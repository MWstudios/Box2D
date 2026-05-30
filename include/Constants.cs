using System;

namespace Box2D;
public static partial class Box2D
{
    /// <summary>Box2D bases all length units on meters, but you may need different units for your game.
    /// You can set this value to use different units. This should be done at application startup
    /// and only modified once. Default value is 1.
    /// For example, if your game uses pixels for units you can use pixels for all length values
    /// sent to Box2D. There should be no extra cost. However, Box2D has some internal tolerances
    /// and thresholds that have been tuned for meters. By calling this function, Box2D is able
    /// to adjust those tolerances and thresholds to improve accuracy.
    /// A good rule of thumb is to pass the height of your player character to this function. So
    /// if your player character is 32 pixels high, then pass 32 to this function. Then you may
    /// confidently use pixels for all the length values sent to Box2D. All length values returned
    /// from Box2D will also be pixels because Box2D does not do any scaling internally.
    /// However, you are now on the hook for coming up with good values for gravity, density, and
    /// forces.</summary>
    /// <remarks>This must be modified before any calls to Box2D</remarks>
    public static float LengthUnitsPerMeter { get; set; } = 1;
    /// <summary>Used to detect bad values. Positions greater than about 16km will have precision
    /// problems, so 100km as a limit should be fine in all cases.</summary>
    public static float Huge = 100000 * LengthUnitsPerMeter;
    /// <summary>Maximum parallel workers. Used for some fixed size arrays.</summary>
    public static int MaxWorkers = 32;
    /// <summary>Maximum number of tasks queued per world step. b2EnqueueTaskCallback will never be called
    /// more than this per world step. This is related to B2_MAX_WORKERS. With 32 workers,
    /// the maximum observed task count is 130. This allows an external task system to use a fixed
    /// size array for Box2D task, which may help with creating stable user task pointers.</summary>
    public static int MaxTasks = 256;
    /// <summary>Maximum number of colors in the constraint graph. Constraints that cannot
    /// find a color are added to the overflow set which are solved single-threaded.
    /// The compound barrel benchmark has minor overflow with 24 colors</summary>
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
    /// <summary>The default contact recycling distance.</summary>
    public static float ContactRecycleDistance = 10 * LinearSlop;
    /// <summary>The default contact recycling world angle threshold. 0.98 ~= 11.5 degrees</summary>
    public static float ContactRecycleCosAngle = 0.98f;
    /// <summary>This is used to fatten AABBs in the dynamic tree. This allows proxies
    /// to move by a small amount without triggering a tree adjustment. This is in meters.
    /// Normally this is 5cm.</summary>
    /// <remarks> modifying this can have a significant impact on performance</remarks>
    public static float MaxAABBMargin = 0.05f * LengthUnitsPerMeter;
    /// <summary>For small objects the margin is limited to this fraction times the maximum extent</summary>
    public static float AABBMarginFraction = 0.125f;
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
