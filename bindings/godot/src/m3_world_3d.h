// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#ifndef MAUL3D_GODOT_M3_WORLD_3D_H
#define MAUL3D_GODOT_M3_WORLD_3D_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>

#include <maul3d/body.h>
#include <maul3d/shape.h>
#include <maul3d/world.h>

namespace maul3d_godot
{

// A Maul3D world as a Godot node: created on enter, destroyed on
// exit, stepped from _physics_process with Godot's fixed delta.
// Bodies hand back an opaque 64-bit id (the 8-byte m3BodyId,
// bit-cast) that GDScript carries as int. This is the seed of an
// integration, deliberately small: pose reads for rendering,
// snapshot and restore for rollback, hash for desync checks.
class M3World3D : public godot::Node3D
{
    GDCLASS(M3World3D, godot::Node3D)

  public:
    void _enter_tree() override;
    void _exit_tree() override;
    void _physics_process(double delta) override;

    int64_t create_dynamic_box(godot::Vector3 position, godot::Vector3 half_extents,
                               float density);
    int64_t create_static_floor();
    godot::Transform3D body_transform(int64_t body) const;
    void set_body_velocity(int64_t body, godot::Vector3 velocity);

    godot::PackedByteArray snapshot() const;
    bool restore(const godot::PackedByteArray& data);
    godot::String world_hash() const; // hex, for desync checks

    void set_substeps(int substeps);
    int get_substeps() const;

  protected:
    static void _bind_methods();

  private:
    m3WorldId world_ = {0, 0};
    int substeps_ = 4;
};

} // namespace maul3d_godot

#endif
