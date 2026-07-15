# Maul3D Godot starter kit

A GDExtension skeleton that puts a Maul3D world in the scene tree
as `M3World3D` (a `Node3D`). It is a seed, not a plugin release:
the node creates its world on enter, steps it from
`_physics_process`, and exposes the minimum a rollback game needs
from GDScript:

- `create_dynamic_box(position, half_extents, density) -> int`
- `create_static_floor() -> int`
- `body_transform(body) -> Transform3D` (drive your meshes)
- `set_body_velocity(body, velocity)`
- `snapshot() -> PackedByteArray` / `restore(data) -> bool`
- `world_hash() -> String` (hex; compare across peers for desync)
- `substeps` property (default 4)

Body ids are the engine's 8-byte generation-tagged handles packed
into an int: opaque, stable across snapshot restore, never a
pointer.

## Build

```sh
# 1. Maul3D static library
cmake -B ../../build/release -DCMAKE_BUILD_TYPE=Release ../..
cmake --build ../../build/release

# 2. godot-cpp, matching your Godot version
git clone -b 4.2 https://github.com/godotengine/godot-cpp

# 3. the extension
scons platform=linux target=template_release
```

Copy `bin/` and `maul3d.gdextension` into your Godot project.

This kit is compile-checked against the Maul3D headers; building
the extension itself needs godot-cpp and scons, which are not part
of this repository's CI. Extend `M3World3D` along the same
pattern: every engine call you need is one bound method away, and
the C API is documented in `docs/manual.md`.

## Determinism note

Godot's `_physics_process` delta is fixed, but if you run rollback
netcode, step the world yourself from your netcode tick instead
and keep `_physics_process` for interpolation only. Snapshot,
journal, and hash behave exactly as the manual describes; the
binding adds nothing and takes nothing away.
