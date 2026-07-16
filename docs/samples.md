# Samples

Small, complete, honest: each of these compiles against the public
headers alone and prints something you can verify. Build the
library first (`cmake -B build && cmake --build build`), then
compile a sample with
`gcc sample.c -Iinclude build/libmaul3d.a -lm`.

## A falling stack, hashed

```c
#include <maul3d/body.h>
#include <maul3d/shape.h>
#include <maul3d/world.h>
#include <stdio.h>

int main(void)
{
    m3WorldDef wd = m3DefaultWorldDef();
    m3WorldId world = m3CreateWorld(&wd);

    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane floor = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3CreatePlaneShape(ground, &sd, &floor);

    for (int i = 0; i < 10; ++i)
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.type = m3_dynamicBody;
        bd.position = (m3Pos3){0.0, 1.0 + 1.05 * i, 0.0};
        m3BodyId box = m3CreateBody(world, &bd);
        m3CreateBoxShape(box, &sd, (m3Vec3){0.5f, 0.5f, 0.5f});
    }

    for (int i = 0; i < 300; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
    printf("world hash %016llx\n", (unsigned long long)m3World_Hash(world));
    m3DestroyWorld(world);
    return 0;
}
```

Run it twice, on two machines, on two architectures: the hash is
the same. That is the whole point of the engine.

## Snapshot, diverge, restore: the rollback loop

```c
    int32_t size = m3World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    m3World_Snapshot(world, snap, size);
    uint64_t then = m3World_Hash(world);

    // Simulate ahead: mispredicted input, wrong future.
    for (int i = 0; i < 30; ++i)
    {
        m3World_Step(world, 1.0f / 60.0f, 4);
    }

    // The corrected input arrives: rewind and replay.
    m3World_Restore(world, snap, size);
    if (m3World_Hash(world) != then)
    {
        // never happens; the suite proves it every commit
    }
    free(snap);
```

Restore is total: every body, joint, vehicle, soft body, contact,
and sleep timer returns to the bit. Ids survive; your game object
handles stay valid.

## A raycast car

```c
    m3BodyDef bd = m3DefaultBodyDef();
    bd.type = m3_dynamicBody;
    bd.position = (m3Pos3){0.0, 0.65, 0.0};
    m3BodyId chassis = m3CreateBody(world, &bd);
    m3ShapeDef cs = m3DefaultShapeDef();
    cs.density = 300.0f;
    m3CreateBoxShape(chassis, &cs, (m3Vec3){1.0f, 0.25f, 0.5f});

    m3VehicleDef vd = m3DefaultVehicleDef();
    vd.chassis = chassis;
    vd.wheelCount = 4;
    for (int w = 0; w < 4; ++w)
    {
        vd.wheels[w].anchor = (m3Vec3){(w & 1) ? 0.8f : -0.8f, -0.25f,
                                       (w & 2) ? 0.45f : -0.45f};
        vd.wheels[w].driven = true;
        vd.wheels[w].steerable = (w & 1) != 0;
    }
    m3VehicleId car = m3CreateVehicle(world, &vd);

    m3Vehicle_SetCommands(car, 1.0f, 0.0f, 0.0f); // throttle, steer, brake
```

Commands are journaled state: record the journal and the whole
drive replays bit-exact. Add a drivetrain
(`m3Vehicle_SetDrivetrain`) for gears, or leave the flat force.

## A tank

```c
    m3Vehicle_SetTankCommands(car, 1.0f, -1.0f, 0.0f); // left, right, brake
```

Opposite tracks pivot the hull in place; equal tracks run
straight. Stop before you pivot, like the real machine: at speed
the tires read a spin attempt as a rollover.

## A character on the stairs

```c
    m3CharacterDef cd = m3DefaultCharacterDef();
    cd.position = (m3Pos3){0.0, 2.0, 0.0};
    m3CharacterId hero = m3CreateCharacter(world, &cd);

    // per tick, gravity is the host's job, netcode style:
    m3Character_Move(hero, (m3Vec3){0.1f, -0.16f, 0.0f});
    m3World_Step(world, 1.0f / 60.0f, 4);
```

Collide and slide, stair steps, moving platforms, and a journaled
move command that rolls back with everything else.
