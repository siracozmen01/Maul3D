// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// m3replay: the replay studio's command line (9-1). Records the
// built-in demo storm to an .m3j container, prints container info,
// and verifies a container by replaying it bit-for-bit. File IO
// lives HERE, never in the library (the zero-dependency law).
//
//   m3replay record <out.m3j>
//   m3replay info   <file.m3j>
//   m3replay verify <file.m3j>

#include "maul3d/joint.h"
#include "maul3d/replay.h"
#include "maul3d/shape.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// The demo def is part of the tool's contract: verify recreates
// the same capacities before restoring the embedded snapshot.
static m3WorldDef DemoDef(void)
{
    m3WorldDef def = m3DefaultWorldDef();
    def.bodyCapacity = 64;
    def.shapeCapacity = 64;
    def.jointCapacity = 16;
    return def;
}

// A deterministic little storm: crates rain on a plane, a hinge
// door gets a motor and a drive target, materials shift mid-run.
static void DemoSession(m3WorldId world, int32_t steps)
{
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3ShapeId floor = m3CreatePlaneShape(ground, &sd, &fl);
    m3Shape_EnableHitEvents(floor, true);

    m3BodyId post;
    m3JointId hinge;
    {
        m3BodyDef bd = m3DefaultBodyDef();
        bd.position = (m3Pos3){-3.0, 2.0, 0.0};
        post = m3CreateBody(world, &bd);
        m3CreateBoxShape(post, &sd, (m3Vec3){0.2f, 2.0f, 0.2f});
        m3BodyDef dd = m3DefaultBodyDef();
        dd.type = m3_dynamicBody;
        dd.position = (m3Pos3){-2.0, 2.0, 0.0};
        m3BodyId door = m3CreateBody(world, &dd);
        m3CreateBoxShape(door, &sd, (m3Vec3){0.8f, 1.0f, 0.1f});
        m3JointDef jd = m3DefaultJointDef();
        jd.type = m3_revoluteJoint;
        jd.bodyA = post;
        jd.bodyB = door;
        jd.localAnchorA = (m3Vec3){0.2f, 0.0f, 0.0f};
        jd.localAnchorB = (m3Vec3){-0.8f, 0.0f, 0.0f};
        jd.localAxisA = (m3Vec3){0.0f, 1.0f, 0.0f};
        jd.localAxisB = (m3Vec3){0.0f, 1.0f, 0.0f};
        hinge = m3CreateJoint(&jd);
    }

    for (int32_t i = 0; i < steps; ++i)
    {
        if (i % 30 == 5 && i < 240)
        {
            m3BodyDef bd = m3DefaultBodyDef();
            bd.type = m3_dynamicBody;
            bd.position = (m3Pos3){-1.5 + 0.5 * (double)((i / 30) % 7), 6.0, 0.0};
            m3BodyId crate = m3CreateBody(world, &bd);
            m3CreateBoxShape(crate, &sd, (m3Vec3){0.4f, 0.4f, 0.4f});
        }
        if (i == 60)
        {
            m3Joint_SetMotor(hinge, true, 2.0f, 40.0f);
        }
        if (i == 120)
        {
            m3Joint_SetMotor(hinge, false, 0.0f, 0.0f);
            m3Joint_SetSpring(hinge, true, 5.0f, 0.9f);
            m3Joint_SetTargetAngle(hinge, -0.4f);
        }
        if (i == 150)
        {
            m3Shape_SetFriction(floor, 0.2f);
        }
        m3World_Step(world, 1.0f / 60.0f, 4);
    }
}

static int Record(const char* path)
{
    m3WorldDef def = DemoDef();
    m3WorldId world = m3CreateWorld(&def);

    int32_t snapBytes = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
    if (snap == NULL || m3World_Snapshot(world, snap, snapBytes) != snapBytes)
    {
        fprintf(stderr, "m3replay: initial snapshot failed\n");
        return 1;
    }

    int32_t journalCap = 4 * 1024 * 1024;
    uint8_t* journal = (uint8_t*)malloc((size_t)journalCap);
    if (journal == NULL || !m3World_JournalBegin(world, journal, journalCap))
    {
        fprintf(stderr, "m3replay: journal begin failed\n");
        return 1;
    }
    DemoSession(world, 300);
    int32_t journalBytes = m3World_JournalEnd(world);
    if (journalBytes < 0)
    {
        fprintf(stderr, "m3replay: journal overflowed\n");
        return 1;
    }
    uint64_t finalHash = m3World_Hash(world);

    int32_t need = m3ReplayEncodeSize(snapBytes, journalBytes);
    uint8_t* blob = (uint8_t*)malloc((size_t)need);
    int32_t wrote = m3ReplayEncode(snap, snapBytes, journal, journalBytes, finalHash, blob, need);
    if (wrote != need)
    {
        fprintf(stderr, "m3replay: encode failed\n");
        return 1;
    }
    FILE* f = fopen(path, "wb");
    if (f == NULL || fwrite(blob, 1, (size_t)wrote, f) != (size_t)wrote)
    {
        fprintf(stderr, "m3replay: cannot write %s\n", path);
        return 1;
    }
    fclose(f);
    printf("m3replay: recorded %s (%d bytes, hash %016llx)\n", path, wrote,
           (unsigned long long)finalHash);
    free(blob);
    free(journal);
    free(snap);
    m3DestroyWorld(world);
    return 0;
}

static uint8_t* ReadAll(const char* path, int32_t* outBytes)
{
    FILE* f = fopen(path, "rb");
    if (f == NULL)
    {
        return NULL;
    }
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (len <= 0)
    {
        fclose(f);
        return NULL;
    }
    uint8_t* data = (uint8_t*)malloc((size_t)len);
    if (data == NULL || fread(data, 1, (size_t)len, f) != (size_t)len)
    {
        fclose(f);
        free(data);
        return NULL;
    }
    fclose(f);
    *outBytes = (int32_t)len;
    return data;
}

static int Info(const char* path)
{
    int32_t bytes = 0;
    uint8_t* data = ReadAll(path, &bytes);
    m3ReplayView view;
    if (data == NULL || !m3ReplayDecode(data, bytes, &view))
    {
        fprintf(stderr, "m3replay: %s is not a valid M3J1 container\n", path);
        return 1;
    }
    printf("m3replay: %s\n  snapshot %d bytes\n  journal  %d bytes\n  ops      %d\n  steps    "
           "%d\n  hash     %016llx\n",
           path, view.snapshotBytes, view.journalBytes, view.info.opCount, view.info.stepCount,
           (unsigned long long)view.finalHash);
    free(data);
    return 0;
}

static int Verify(const char* path)
{
    int32_t bytes = 0;
    uint8_t* data = ReadAll(path, &bytes);
    m3ReplayView view;
    if (data == NULL || !m3ReplayDecode(data, bytes, &view))
    {
        fprintf(stderr, "m3replay: %s is not a valid M3J1 container\n", path);
        return 1;
    }
    m3WorldDef def = DemoDef();
    m3WorldId world = m3CreateWorld(&def);
    if (!m3World_Restore(world, view.snapshot, view.snapshotBytes))
    {
        fprintf(stderr, "m3replay: the embedded snapshot refused (config or capacity)\n");
        return 1;
    }
    if (!m3World_JournalReplay(world, view.journal, view.journalBytes))
    {
        fprintf(stderr, "m3replay: the journal refused\n");
        return 1;
    }
    uint64_t hash = m3World_Hash(world);
    printf("m3replay: replayed %d ops / %d steps, hash %016llx, recorded %016llx: %s\n",
           view.info.opCount, view.info.stepCount, (unsigned long long)hash,
           (unsigned long long)view.finalHash, hash == view.finalHash ? "MATCH" : "DIVERGED");
    m3DestroyWorld(world);
    free(data);
    return hash == view.finalHash ? 0 : 2;
}

int main(int argc, char** argv)
{
    if (argc == 3 && strcmp(argv[1], "record") == 0)
    {
        return Record(argv[2]);
    }
    if (argc == 3 && strcmp(argv[1], "info") == 0)
    {
        return Info(argv[2]);
    }
    if (argc == 3 && strcmp(argv[1], "verify") == 0)
    {
        return Verify(argv[2]);
    }
    fprintf(stderr, "usage: m3replay record|info|verify <file.m3j>\n");
    return 1;
}
