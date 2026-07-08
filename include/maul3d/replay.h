// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#ifndef MAUL3D_REPLAY_H
#define MAUL3D_REPLAY_H

#include "maul3d/base.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /// The M3J1 replay container (phase 9): a pure-memory codec for
    /// [initial snapshot + journal] sessions. The LIBRARY never
    /// touches files; tools own IO (the zero-dependency law). The
    /// embedded snapshot self-validates on restore (config hash,
    /// format version, capacities), so the container only guards
    /// its own framing.
    ///
    /// Layout, little-endian: magic "M3J1" (4) | container version
    /// u32 (1) | snapshotBytes i32 | journalBytes i32 | stepCount
    /// i32 | opCount i32 | finalHash u64 | snapshot | journal.

    /// Counts from a raw journal byte stream: total records, and
    /// how many of them are steps. Refuses malformed framing.
    typedef struct m3JournalInfo
    {
        int32_t opCount;
        int32_t stepCount;
    } m3JournalInfo;

    M3_API bool m3JournalDescribe(const void* journal, int32_t bytes, m3JournalInfo* out);

    /// A decoded view into an encoded container: pointers alias the
    /// input buffer, nothing is copied.
    typedef struct m3ReplayView
    {
        const void* snapshot;
        int32_t snapshotBytes;
        const void* journal;
        int32_t journalBytes;
        uint64_t finalHash; // the recorder's end-of-session hash
        m3JournalInfo info;
    } m3ReplayView;

    /// Exact encoded size for the given payload sizes.
    M3_API int32_t m3ReplayEncodeSize(int32_t snapshotBytes, int32_t journalBytes);

    /// Encode a container. finalHash is the recorder's end hash
    /// (m3World_Hash after the session); verifiers compare against
    /// it. Returns bytes written, or 0 on refusal (bad sizes, small
    /// capacity, malformed journal).
    M3_API int32_t m3ReplayEncode(const void* snapshot, int32_t snapshotBytes, const void* journal,
                                  int32_t journalBytes, uint64_t finalHash, void* out,
                                  int32_t capacity);

    /// Decode and validate framing (magic, version, lengths, journal
    /// record walk). Returns false on any corruption; the view is
    /// untouched on refusal.
    M3_API bool m3ReplayDecode(const void* data, int32_t bytes, m3ReplayView* out);

#ifdef __cplusplus
}
#endif

#endif
