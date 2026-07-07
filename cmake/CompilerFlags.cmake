# Determinism-critical compiler flags. Every target that contains
# simulation code must call maul3d_apply_flags(). These flags are the
# build-time half of the determinism contract; the test suite verifies
# the promise at runtime.

# SIMD policy (phase 2a design): the canonical logical width is 4
# lanes on every backend (SSE2 on x64, NEON on arm64, scalar fallback),
# which is Box3D's choice and aids cross-platform bit identity. SSE2 is
# part of the x86-64 baseline, so no extra arch flags are needed.
# MAUL3D_SIMD=scalar forces the scalar fallback, and CI runs one scalar
# cell whose hashes must match the vector cells bit for bit.
set(MAUL3D_SIMD "auto" CACHE STRING "SIMD backend: auto or scalar")

function(maul3d_apply_flags target)
    # A host injecting fast-math through global flags would silently void
    # the determinism contract; refuse to configure at all.
    string(FIND "${CMAKE_C_FLAGS}" "fast-math" _m3_fastmath)
    string(FIND "${CMAKE_C_FLAGS}" "/fp:fast" _m3_fpfast)
    if(NOT _m3_fastmath EQUAL -1 OR NOT _m3_fpfast EQUAL -1)
        message(FATAL_ERROR "fast-math in CMAKE_C_FLAGS is incompatible with maul3d's determinism contract")
    endif()

    set_target_properties(${target} PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)

    if(MSVC)
        # /fp:precise alone permitted FMA contraction before VS2022 17.0
        # (and by default on arm64). Require a compiler that knows
        # /fp:contract so contraction can be switched off explicitly.
        if(MSVC_VERSION LESS 1930)
            message(FATAL_ERROR "MSVC >= VS2022 17.0 (1930) required: older versions cannot disable FP contraction")
        endif()
        target_compile_options(${target} PRIVATE /W4 /WX /fp:precise /fp:contract-)
    else()
        target_compile_options(${target} PRIVATE
            -ffp-contract=off -fno-trapping-math -fno-fast-math -fno-unsafe-math-optimizations
            -Wall -Wextra -Werror -Wshadow -Wdouble-promotion)
    endif()

    if(MAUL3D_SIMD STREQUAL "scalar")
        target_compile_definitions(${target} PRIVATE MAUL3D_SIMD_FORCE_SCALAR=1)
    endif()

    if(MAUL3D_SANITIZE)
        if(NOT MSVC)
            target_compile_options(${target} PRIVATE -fsanitize=address,undefined -fno-sanitize-recover=all)
            target_link_options(${target} PRIVATE -fsanitize=address,undefined)
        endif()
    endif()
endfunction()
