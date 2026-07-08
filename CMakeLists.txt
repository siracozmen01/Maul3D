cmake_minimum_required(VERSION 3.16)
project(maul3d VERSION 0.1.0 LANGUAGES C)

option(MAUL3D_SANITIZE "Build with ASAN/UBSAN" OFF)

include(cmake/CompilerFlags.cmake)

add_library(maul3d STATIC
    src/allocator.c
    src/body.c
    src/broad_phase.c
    src/core.c
    src/distance.c
    src/dynamic_tree.c
    src/hull.c
    src/manifold.c
    src/quickhull.c
    src/joint.c
    src/raycast.c
    src/math_functions.c
    src/shape.c
    src/snapshot.c
    src/solver.c
    src/world.c
)
target_include_directories(maul3d PUBLIC include)
target_include_directories(maul3d PRIVATE src)
maul3d_apply_flags(maul3d)
if(NOT MSVC)
    # sqrtf and remainderf, the only libm the bit law allows.
    target_link_libraries(maul3d PUBLIC m)
endif()

enable_testing()

find_package(Threads REQUIRED)

add_executable(test_determinism test/test_determinism.c)
target_link_libraries(test_determinism PRIVATE maul3d Threads::Threads)
maul3d_apply_flags(test_determinism)
add_test(NAME determinism COMMAND test_determinism)

add_executable(test_math test/test_math.c)
target_link_libraries(test_math PRIVATE maul3d)
maul3d_apply_flags(test_math)
add_test(NAME math COMMAND test_math)

add_executable(test_core test/test_core.c)
target_link_libraries(test_core PRIVATE maul3d)
target_include_directories(test_core PRIVATE src)
maul3d_apply_flags(test_core)
add_test(NAME core COMMAND test_core)

add_executable(test_world test/test_world.c)
target_link_libraries(test_world PRIVATE maul3d)
target_include_directories(test_world PRIVATE src)
maul3d_apply_flags(test_world)
add_test(NAME world COMMAND test_world)

add_executable(test_contacts test/test_contacts.c)
target_link_libraries(test_contacts PRIVATE maul3d)
target_include_directories(test_contacts PRIVATE src)
maul3d_apply_flags(test_contacts)
add_test(NAME contacts COMMAND test_contacts)

add_executable(test_distance test/test_distance.c)
target_link_libraries(test_distance PRIVATE maul3d)
target_include_directories(test_distance PRIVATE src)
maul3d_apply_flags(test_distance)
add_test(NAME distance COMMAND test_distance)

add_executable(test_solver test/test_solver.c)
target_link_libraries(test_solver PRIVATE maul3d)
maul3d_apply_flags(test_solver)
add_test(NAME solver COMMAND test_solver)

add_executable(test_quickhull test/test_quickhull.c)
target_link_libraries(test_quickhull PRIVATE maul3d)
target_include_directories(test_quickhull PRIVATE src)
maul3d_apply_flags(test_quickhull)
add_test(NAME quickhull COMMAND test_quickhull)

add_executable(test_queries test/test_queries.c)
target_link_libraries(test_queries PRIVATE maul3d)
maul3d_apply_flags(test_queries)
add_test(NAME queries COMMAND test_queries)

# The benchmark harness: built always (so it cannot rot), never a
# ctest gate (wall time is not deterministic). Run manually in
# Release for profiles; scenes print their hashes as a sanity anchor.
add_executable(maul3d_bench bench/bench_main.c)
target_link_libraries(maul3d_bench PRIVATE maul3d)
maul3d_apply_flags(maul3d_bench)

add_executable(test_joints test/test_joints.c)
target_link_libraries(test_joints PRIVATE maul3d)
maul3d_apply_flags(test_joints)
add_test(NAME joints COMMAND test_joints)

add_executable(test_hardening test/test_hardening.c)
target_link_libraries(test_hardening PRIVATE maul3d)
maul3d_apply_flags(test_hardening)
add_test(NAME hardening COMMAND test_hardening)

add_executable(test_journal test/test_journal.c)
target_link_libraries(test_journal PRIVATE maul3d)
maul3d_apply_flags(test_journal)
add_test(NAME journal COMMAND test_journal)
