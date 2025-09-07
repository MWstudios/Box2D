# Box2D 3 + LiquidFun in C#

This is a near complete port of [Box2D 3](https://github.com/erincatto/box2d) (as of commit [5e6cedd](https://github.com/erincatto/box2d/commit/5e6cedd2fb874988829edf14c498dcf2b94d5166))
and [LiquidFun](https://github.com/google/liquidfun) into C#. Testing macros such as B2_VALIDATE, samples and other benchmarks are not included.

**Note: Due to a missing SIMD assembly instruction in C#, the ARM64 version is incomplete and will throw exeptions.** Let me know if you have a C# replacement for `vtrnq_f32()`.

This port also does not mean that porting to other languages (Java, Python) is now easier. Not everything could be replicated in purely managed environment; some pointer logic from C and native memory management is still left in the code.

A few changes have also been made:
- Struct unions have been turned into polymorphic classes.
- Normal C# arrays instead of arena allocations.
- Some LiquidFun functions are now multithreaded or have new AVX counterparts. The SIMD functions from the original have been excluded (I was unable to transcribe them to C#).
- LiquidFun also uses [HPCSharp](https://github.com/DragonSpit/HPCSharp) to sort arrays.
- All fields and methods in Box2D (except integrity checks) have been made public.
- To not get lost in the chaos, the original Box2D API has been moved to the `Box2D.API` namespace and split into classes. LiquidFun got a new one, `ParticleAPI`.
- Point-shape distance functions in LiquidFun have been replaced with Box2D's distance proxies (the polygon one was broken anyway). Create a circle shape with zero radius and an identity transform. Use `Distance.ShapeDistance()`for points outside shapes and `ContactRegister.ComputeManifold()` for points inside shapes where proxies do not work (output normal equals zero).
- Shape casting did not exist back in Box2D 2 and thus won't work on particles.
- There is no `ParticleSystemDef`. Instead call `ParticleAPI.CreateParticleSystem()` access the properties of `ParticleSystem`.
- `ParticleSystem.Draw()` has not been implemented. Use `ParticleSystem.PositionBuffer` to render particles.

There is also a bug where particles can get stuck in bodies and not move. I'm not sure if this was happening in the original.

# Building

This project requires .NET 9 and [HPCSharp](https://github.com/DragonSpit/HPCSharp) on NuGet. If you do not want HPCSharp to be included, go to particle/ParticleSystem.cs and replace all sorting methods with normal sorting.