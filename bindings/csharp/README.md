# Maul3D C# starter kit

One file, no dependencies: `Maul3D.cs` binds the core surface
(world, body, shape, joint, step, queries, snapshot) over P/Invoke.
It is a seed, not a package: copy it into your project and extend
it along the same pattern as your game needs more of the API.

## Build the native library

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release -DMAUL3D_BUILD_SHARED=ON
cmake --build build
```

Put the result next to your managed binary:

| Platform | File |
| --- | --- |
| Windows | `maul3d.dll` |
| Linux | `libmaul3d.so` |
| macOS | `libmaul3d.dylib` |

The `DllImport` name is `maul3d`; the runtime resolves the
platform prefix and suffix itself.

## Use

```csharp
Maul3D.Native.LayoutCheck(); // once at startup, before anything

var wd = Maul3D.Native.DefaultWorldDef();
var world = Maul3D.Native.CreateWorld(ref wd);

var bd = Maul3D.Native.DefaultBodyDef();
bd.Type = (int)Maul3D.BodyType.Dynamic;
bd.Position = new Maul3D.Pos3(0, 5, 0);
var body = Maul3D.Native.CreateBody(world, ref bd);

var sd = Maul3D.Native.DefaultShapeDef();
var ball = new Maul3D.Sphere { Radius = 0.5f };
Maul3D.Native.CreateSphereShape(body, ref sd, ref ball);

for (int i = 0; i < 60; i++)
{
    Maul3D.Native.Step(world, 1.0f / 60.0f, 4);
}
Console.WriteLine($"hash {Maul3D.Native.Hash(world):x16}");

// Rollback: snapshot, run ahead, restore, run again: same hash.
int size = Maul3D.Native.SnapshotSize(world);
var snap = new byte[size];
Maul3D.Native.Snapshot(world, snap, size);
// ... later ...
Maul3D.Native.Restore(world, snap, size);

Maul3D.Native.DestroyWorld(world);
```

## The rules that keep you safe

- Always start from the `Default*Def()` functions. Every def
  carries a cookie; a zeroed or hand-rolled def is refused loudly
  by design.
- Call `LayoutCheck()` at startup. It catches a struct mirror
  drifting from the C headers before it can corrupt memory.
- Bools inside defs are `byte` fields (C `_Bool` is one byte).
  Set them to 0 or 1.
- Ids are value types and stay valid across snapshot restore;
  they encode no pointers.
- Determinism holds through this binding: commands in, hashes
  out, the same on every platform the engine supports.
