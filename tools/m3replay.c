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

static void DemoSessionEx(m3WorldId world, int32_t steps, int32_t injectStep);

// A deterministic little storm: crates rain on a plane, a hinge
// door gets a motor and a drive target, materials shift mid-run.
static void DemoSession(m3WorldId world, int32_t steps)
{
    DemoSessionEx(world, steps, -1);
}

// injectStep >= 0 slips one extra impulse before that step: the
// divergence the finder must pin to the exact step and body.
static void DemoSessionEx(m3WorldId world, int32_t steps, int32_t injectStep)
{
    m3BodyDef gd = m3DefaultBodyDef();
    m3BodyId ground = m3CreateBody(world, &gd);
    m3ShapeDef sd = m3DefaultShapeDef();
    m3Plane fl = {{0.0f, 1.0f, 0.0f}, 0.0f};
    m3ShapeId floor = m3CreatePlaneShape(ground, &sd, &fl);
    m3Shape_EnableHitEvents(floor, true);

    m3BodyId post;
    m3BodyId injectTarget;
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
        injectTarget = door;
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
        if (i == injectStep)
        {
            // The deliberate desync: a kick on the door's post
            // partner, journaled like everything else.
            m3Body_ApplyLinearImpulse(injectTarget, (m3Vec3){0.8f, 0.0f, 0.3f});
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

// --- The step scrubber (9-2) ------------------------------------------------
//
// A journal prefix cut at record boundaries is itself a valid
// journal, so the scrubber needs no engine additions: it slices
// the stream into per-step segments (the ops leading up to a step
// record, plus that step record), replays segments forward, and
// keyframes a snapshot every M3_SCRUB_INTERVAL steps. Seek =
// restore the nearest keyframe at or before the target, re-step
// forward. Bit-exact by the rollback contract, and the selftest
// PROVES it against straight-run prefix hashes.

#define M3_SCRUB_INTERVAL 30

typedef struct Scrub
{
    const uint8_t* journal;
    int32_t journalBytes;
    int32_t stepCount;
    int32_t* stepEnds; // byte offset just PAST step i's record
    uint8_t** keys;    // keyframe snapshots, one per interval mark
    int32_t* keySizes; // per-key bytes: snapshots are count-derived
                       // (10-3 meshes, 17-1 hulls) so the size
                       // GROWS with the world and is never one
                       // number for a whole session
    int32_t keyCount;
    m3WorldId world;
} Scrub;

// Walk the framing once, recording where each step's segment ends.
static bool ScrubIndex(Scrub* s)
{
    s->stepEnds = (int32_t*)malloc(sizeof(int32_t) * (size_t)(s->stepCount + 1));
    int32_t cursor = 0;
    int32_t step = 0;
    while (cursor < s->journalBytes)
    {
        int32_t op;
        int32_t payload;
        memcpy(&op, s->journal + cursor, 4);
        memcpy(&payload, s->journal + cursor + 4, 4);
        cursor += 8 + payload;
        if (op == 1)
        {
            s->stepEnds[step] = cursor;
            step += 1;
        }
    }
    return step == s->stepCount;
}

static bool ScrubBuild(Scrub* s, const m3ReplayView* view)
{
    s->journal = (const uint8_t*)view->journal;
    s->journalBytes = view->journalBytes;
    s->stepCount = view->info.stepCount;
    if (!ScrubIndex(s))
    {
        return false;
    }
    m3WorldDef def = DemoDef();
    s->world = m3CreateWorld(&def);
    if (!m3World_Restore(s->world, view->snapshot, view->snapshotBytes))
    {
        return false;
    }
    s->keyCount = s->stepCount / M3_SCRUB_INTERVAL + 1;
    s->keys = (uint8_t**)calloc((size_t)s->keyCount, sizeof(uint8_t*));
    s->keySizes = (int32_t*)calloc((size_t)s->keyCount, sizeof(int32_t));
    // Key 0 is the pre-step world (creates land inside segment 0).
    s->keySizes[0] = m3World_SnapshotSize(s->world);
    s->keys[0] = (uint8_t*)malloc((size_t)s->keySizes[0]);
    if (m3World_Snapshot(s->world, s->keys[0], s->keySizes[0]) != s->keySizes[0])
    {
        return false;
    }
    for (int32_t i = 0; i < s->stepCount; ++i)
    {
        int32_t from = i == 0 ? 0 : s->stepEnds[i - 1];
        int32_t to = s->stepEnds[i];
        if (!m3World_JournalReplay(s->world, s->journal + from, to - from))
        {
            return false;
        }
        if ((i + 1) % M3_SCRUB_INTERVAL == 0 && (i + 1) / M3_SCRUB_INTERVAL < s->keyCount)
        {
            int32_t k = (i + 1) / M3_SCRUB_INTERVAL;
            s->keySizes[k] = m3World_SnapshotSize(s->world);
            s->keys[k] = (uint8_t*)malloc((size_t)s->keySizes[k]);
            if (m3World_Snapshot(s->world, s->keys[k], s->keySizes[k]) != s->keySizes[k])
            {
                return false;
            }
        }
    }
    return true;
}

// Land the scrub world exactly on the state AFTER step N (N = 0
// means the pre-step world).
static bool ScrubSeek(Scrub* s, int32_t stepN)
{
    if (stepN < 0 || stepN > s->stepCount)
    {
        return false;
    }
    int32_t key = stepN / M3_SCRUB_INTERVAL;
    if (key >= s->keyCount)
    {
        key = s->keyCount - 1;
    }
    if (!m3World_Restore(s->world, s->keys[key], s->keySizes[key]))
    {
        return false;
    }
    for (int32_t i = key * M3_SCRUB_INTERVAL; i < stepN; ++i)
    {
        int32_t from = i == 0 ? 0 : s->stepEnds[i - 1];
        int32_t to = s->stepEnds[i];
        if (!m3World_JournalReplay(s->world, s->journal + from, to - from))
        {
            return false;
        }
    }
    return true;
}

// Straight-run prefix hash: a fresh world replays records 0..the
// end of step N in ONE call. The scrubber must always agree.
static bool PrefixHash(const m3ReplayView* view, const Scrub* s, int32_t stepN, uint64_t* out)
{
    m3WorldDef def = DemoDef();
    m3WorldId world = m3CreateWorld(&def);
    if (!m3World_Restore(world, view->snapshot, view->snapshotBytes))
    {
        return false;
    }
    int32_t bytes = stepN == 0 ? 0 : s->stepEnds[stepN - 1];
    if (bytes > 0 && !m3World_JournalReplay(world, view->journal, bytes))
    {
        return false;
    }
    *out = m3World_Hash(world);
    m3DestroyWorld(world);
    return true;
}

static void ScrubFree(Scrub* s)
{
    for (int32_t k = 0; k < s->keyCount; ++k)
    {
        free(s->keys[k]);
    }
    free(s->keys);
    free(s->keySizes);
    free(s->stepEnds);
    m3DestroyWorld(s->world);
}

static int Seek(const char* path, int32_t stepN)
{
    int32_t bytes = 0;
    uint8_t* data = ReadAll(path, &bytes);
    m3ReplayView view;
    if (data == NULL || !m3ReplayDecode(data, bytes, &view))
    {
        fprintf(stderr, "m3replay: %s is not a valid M3J1 container\n", path);
        return 1;
    }
    Scrub s;
    memset(&s, 0, sizeof(s));
    if (!ScrubBuild(&s, &view) || !ScrubSeek(&s, stepN))
    {
        fprintf(stderr, "m3replay: seek failed (step out of range or corrupt stream)\n");
        return 1;
    }
    uint64_t scrubbed = m3World_Hash(s.world);
    uint64_t straight = 0;
    if (!PrefixHash(&view, &s, stepN, &straight))
    {
        fprintf(stderr, "m3replay: straight-run check failed\n");
        return 1;
    }
    printf("m3replay: step %d scrub %016llx straight %016llx: %s\n", stepN,
           (unsigned long long)scrubbed, (unsigned long long)straight,
           scrubbed == straight ? "MATCH" : "DIVERGED");
    ScrubFree(&s);
    free(data);
    return scrubbed == straight ? 0 : 2;
}

static int Play(const char* path, int32_t fromStep, int32_t toStep)
{
    int32_t bytes = 0;
    uint8_t* data = ReadAll(path, &bytes);
    m3ReplayView view;
    if (data == NULL || !m3ReplayDecode(data, bytes, &view))
    {
        fprintf(stderr, "m3replay: %s is not a valid M3J1 container\n", path);
        return 1;
    }
    Scrub s;
    memset(&s, 0, sizeof(s));
    if (!ScrubBuild(&s, &view) || fromStep > toStep || !ScrubSeek(&s, fromStep))
    {
        fprintf(stderr, "m3replay: play range failed\n");
        return 1;
    }
    for (int32_t i = fromStep; i <= toStep && i <= s.stepCount; ++i)
    {
        printf("step %4d hash %016llx\n", i, (unsigned long long)m3World_Hash(s.world));
        if (i < s.stepCount)
        {
            int32_t from = i == 0 ? 0 : s.stepEnds[i - 1];
            m3World_JournalReplay(s.world, s.journal + from, s.stepEnds[i] - from);
        }
    }
    ScrubFree(&s);
    free(data);
    return 0;
}

// Record a demo session (optionally with the injected desync)
// into a malloc'd container.
static uint8_t* RecordBlob(int32_t steps, int32_t injectStep, int32_t* outBytes)
{
    m3WorldDef def = DemoDef();
    m3WorldId world = m3CreateWorld(&def);
    int32_t snapBytes = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
    m3World_Snapshot(world, snap, snapBytes);
    int32_t journalCap = 4 * 1024 * 1024;
    uint8_t* journal = (uint8_t*)malloc((size_t)journalCap);
    m3World_JournalBegin(world, journal, journalCap);
    DemoSessionEx(world, steps, injectStep);
    int32_t journalBytes = m3World_JournalEnd(world);
    uint64_t finalHash = m3World_Hash(world);
    int32_t need = m3ReplayEncodeSize(snapBytes, journalBytes);
    uint8_t* blob = (uint8_t*)malloc((size_t)need);
    m3ReplayEncode(snap, snapBytes, journal, journalBytes, finalHash, blob, need);
    m3DestroyWorld(world);
    free(journal);
    free(snap);
    *outBytes = need;
    return blob;
}

// The finder: binary search the first step whose prefix hash
// differs, verify the step before agrees, then diff the worlds.
static int DiffViews(const m3ReplayView* va, const m3ReplayView* vb)
{
    Scrub sa;
    Scrub sb;
    memset(&sa, 0, sizeof(sa));
    memset(&sb, 0, sizeof(sb));
    if (!ScrubBuild(&sa, va) || !ScrubBuild(&sb, vb))
    {
        fprintf(stderr, "m3replay: diff scrub build failed\n");
        return 1;
    }
    int32_t steps = sa.stepCount < sb.stepCount ? sa.stepCount : sb.stepCount;
    uint64_t ha;
    uint64_t hb;
    PrefixHash(va, &sa, steps, &ha);
    PrefixHash(vb, &sb, steps, &hb);
    if (ha == hb && sa.stepCount == sb.stepCount)
    {
        printf("m3replay: no divergence across %d steps\n", steps);
        ScrubFree(&sa);
        ScrubFree(&sb);
        return 0;
    }
    int32_t lo = 0; // known equal (step 0 = shared initial snapshot?)
    PrefixHash(va, &sa, 0, &ha);
    PrefixHash(vb, &sb, 0, &hb);
    if (ha != hb)
    {
        printf("m3replay: the initial snapshots already differ\n");
        ScrubFree(&sa);
        ScrubFree(&sb);
        return 2;
    }
    int32_t hi = steps; // known (or forced) different end
    while (hi - lo > 1)
    {
        int32_t mid = lo + (hi - lo) / 2;
        PrefixHash(va, &sa, mid, &ha);
        PrefixHash(vb, &sb, mid, &hb);
        if (ha == hb)
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
    }
    printf("m3replay: first divergent step %d\n", hi);
    ScrubSeek(&sa, hi);
    ScrubSeek(&sb, hi);
    m3BodyDiff rows[10];
    int32_t written = 0;
    int32_t total = m3World_DiffReport(sa.world, sb.world, rows, 10, &written);
    printf("m3replay: %d differing bodies, worst first:\n", total);
    for (int32_t i = 0; i < written; ++i)
    {
        printf("  body %4d  pos %.6f  rot %.6f  vel %.6f%s%s\n", rows[i].body.index1,
               (double)rows[i].positionError, (double)rows[i].rotationError,
               (double)rows[i].velocityError, rows[i].onlyInA ? "  ONLY-A" : "",
               rows[i].onlyInB ? "  ONLY-B" : "");
    }
    ScrubFree(&sa);
    ScrubFree(&sb);
    return 2;
}

static int Diff(const char* pathA, const char* pathB)
{
    int32_t bytesA = 0;
    int32_t bytesB = 0;
    uint8_t* dataA = ReadAll(pathA, &bytesA);
    uint8_t* dataB = ReadAll(pathB, &bytesB);
    m3ReplayView va;
    m3ReplayView vb;
    if (dataA == NULL || dataB == NULL || !m3ReplayDecode(dataA, bytesA, &va) ||
        !m3ReplayDecode(dataB, bytesB, &vb))
    {
        fprintf(stderr, "m3replay: diff needs two valid M3J1 containers\n");
        return 1;
    }
    int rc = DiffViews(&va, &vb);
    free(dataA);
    free(dataB);
    return rc;
}

// The CI gate for the finder: identical twins stay silent; an
// injected impulse is pinned to its exact step with the touched
// body reported first.
static int DiffTest(void)
{
    int32_t bytesA;
    int32_t bytesB;
    int32_t bytesC;
    uint8_t* a = RecordBlob(300, -1, &bytesA);
    uint8_t* b = RecordBlob(300, -1, &bytesB);
    uint8_t* c = RecordBlob(300, 150, &bytesC);
    m3ReplayView va;
    m3ReplayView vb;
    m3ReplayView vc;
    if (!m3ReplayDecode(a, bytesA, &va) || !m3ReplayDecode(b, bytesB, &vb) ||
        !m3ReplayDecode(c, bytesC, &vc))
    {
        fprintf(stderr, "difftest: decode failed\n");
        return 1;
    }
    printf("difftest: twins...\n");
    if (DiffViews(&va, &vb) != 0)
    {
        fprintf(stderr, "difftest: identical twins reported a divergence\n");
        return 1;
    }
    printf("difftest: injected...\n");
    Scrub sa;
    Scrub sc;
    memset(&sa, 0, sizeof(sa));
    memset(&sc, 0, sizeof(sc));
    if (!ScrubBuild(&sa, &va) || !ScrubBuild(&sc, &vc))
    {
        return 1;
    }
    // Recompute the first divergent step the same way diff does.
    int32_t lo = 0;
    int32_t hi = 300;
    uint64_t ha;
    uint64_t hc;
    while (hi - lo > 1)
    {
        int32_t mid = lo + (hi - lo) / 2;
        PrefixHash(&va, &sa, mid, &ha);
        PrefixHash(&vc, &sc, mid, &hc);
        if (ha == hc)
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
    }
    // The impulse lands before step 150's record: the first
    // divergent post-step state is step 151 (1-based prefix 151).
    if (hi != 151)
    {
        fprintf(stderr, "difftest: expected first divergence at 151, found %d\n", hi);
        return 1;
    }
    ScrubSeek(&sa, hi);
    ScrubSeek(&sc, hi);
    m3BodyDiff rows[4];
    int32_t written = 0;
    int32_t total = m3World_DiffReport(sa.world, sc.world, rows, 4, &written);
    if (total <= 0 || written <= 0)
    {
        fprintf(stderr, "difftest: no body diff at the divergent step\n");
        return 1;
    }
    // The kicked door is body index 3 (ground, post, door, then
    // rain), reported first by error magnitude.
    if (rows[0].body.index1 != 3)
    {
        fprintf(stderr, "difftest: expected the door (body 3) first, got %d\n",
                rows[0].body.index1);
        return 1;
    }
    ScrubFree(&sa);
    ScrubFree(&sc);
    free(a);
    free(b);
    free(c);
    printf("m3replay difftest: twins silent, injection pinned to step 151, door first\n");
    return 0;
}

// The CI gate: record the demo in memory, scrub-thrash it with a
// deterministic LCG, compare every landing against the straight
// run. No files, runs everywhere.
static int SelfTest(void)
{
    m3WorldDef def = DemoDef();
    m3WorldId world = m3CreateWorld(&def);
    int32_t snapBytes = m3World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)snapBytes);
    m3World_Snapshot(world, snap, snapBytes);
    int32_t journalCap = 4 * 1024 * 1024;
    uint8_t* journal = (uint8_t*)malloc((size_t)journalCap);
    m3World_JournalBegin(world, journal, journalCap);
    DemoSession(world, 300);
    int32_t journalBytes = m3World_JournalEnd(world);
    uint64_t finalHash = m3World_Hash(world);
    int32_t need = m3ReplayEncodeSize(snapBytes, journalBytes);
    uint8_t* blob = (uint8_t*)malloc((size_t)need);
    m3ReplayEncode(snap, snapBytes, journal, journalBytes, finalHash, blob, need);
    m3DestroyWorld(world);

    m3ReplayView view;
    if (!m3ReplayDecode(blob, need, &view))
    {
        fprintf(stderr, "selftest: decode failed\n");
        return 1;
    }
    Scrub s;
    memset(&s, 0, sizeof(s));
    if (!ScrubBuild(&s, &view))
    {
        fprintf(stderr, "selftest: scrub build failed\n");
        return 1;
    }
    // The forward build must end on the recorder's hash.
    if (m3World_Hash(s.world) != finalHash)
    {
        fprintf(stderr, "selftest: the forward pass diverged\n");
        return 1;
    }
    uint32_t rng = 20260708u;
    int32_t failures = 0;
    for (int32_t t = 0; t < 40; ++t)
    {
        rng = rng * 1664525u + 1013904223u;
        int32_t stepN = (int32_t)(rng % (uint32_t)(s.stepCount + 1));
        uint64_t straight;
        if (!ScrubSeek(&s, stepN) || !PrefixHash(&view, &s, stepN, &straight) ||
            m3World_Hash(s.world) != straight)
        {
            fprintf(stderr, "selftest: seek %d DIVERGED\n", stepN);
            failures += 1;
        }
    }
    ScrubFree(&s);
    free(blob);
    free(journal);
    free(snap);
    if (failures == 0)
    {
        printf("m3replay selftest: 40 random seeks, all on the straight-run hash\n");
        return 0;
    }
    return 1;
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
    if (argc == 4 && strcmp(argv[1], "seek") == 0)
    {
        return Seek(argv[2], atoi(argv[3]));
    }
    if (argc == 5 && strcmp(argv[1], "play") == 0)
    {
        return Play(argv[2], atoi(argv[3]), atoi(argv[4]));
    }
    if (argc == 2 && strcmp(argv[1], "selftest") == 0)
    {
        return SelfTest();
    }
    if (argc == 4 && strcmp(argv[1], "diff") == 0)
    {
        return Diff(argv[2], argv[3]);
    }
    if (argc == 2 && strcmp(argv[1], "difftest") == 0)
    {
        return DiffTest();
    }
    fprintf(stderr, "usage: m3replay record|info|verify <file.m3j> | seek <file> <N> | play "
                    "<file> <A> <B> | diff <a> <b> | selftest | difftest\n");
    return 1;
}
