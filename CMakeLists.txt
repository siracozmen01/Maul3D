cmake_minimum_required(VERSION 3.16)
project(maul3d VERSION 1.0.0 LANGUAGES C)

option(MAUL3D_SANITIZE "Build with ASAN/UBSAN" OFF)
option(MAUL3D_BUILD_SHARED "Build maul3d as a shared library" OFF)
option(MAUL3D_BUILD_TESTBED "Build the interactive raylib testbed" OFF)

include(cmake/CompilerFlags.cmake)
include(GNUInstallDirs)

if(MAUL3D_BUILD_SHARED)
    set(MAUL3D_LIB_KIND SHARED)
else()
    set(MAUL3D_LIB_KIND STATIC)
endif()

add_library(maul3d ${MAUL3D_LIB_KIND}
    src/allocator.c
    src/body.c
    src/broad_phase.c
    src/core.c
    src/distance.c
    src/draw.c
    src/dynamic_tree.c
    src/hull.c
    src/manifold.c
    src/mesh_bvh.c
    src/quickhull.c
    src/joint.c
    src/query.c
    src/raycast.c
    src/math_functions.c
    src/shape.c
    src/snapshot.c
    src/solver.c
    src/voxel.c
    src/world.c
)
target_include_directories(maul3d PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_include_directories(maul3d PRIVATE src)
maul3d_apply_flags(maul3d)
if(MAUL3D_BUILD_SHARED)
    # SONAME discipline: the SOVERSION tracks the major version (0
    # while the API is pre-freeze), the full version rides along.
    set_target_properties(maul3d PROPERTIES
        VERSION ${PROJECT_VERSION}
        SOVERSION ${PROJECT_VERSION_MAJOR}
        C_VISIBILITY_PRESET hidden)
    target_compile_definitions(maul3d PUBLIC MAUL3D_SHARED)
endif()
if(NOT MSVC)
    # sqrtf and remainderf, the only libm the bit law allows.
    target_link_libraries(maul3d PUBLIC m)
endif()

# The installable package (2d-4): headers, the library, an exported
# CMake target (maul3d::maul3d), and a pkg-config file. The consumer
# smoke test in test/consumer builds against the INSTALLED tree and
# gates CI.
install(DIRECTORY include/maul3d DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS maul3d EXPORT maul3dTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(EXPORT maul3dTargets
    NAMESPACE maul3d::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/maul3d)

include(CMakePackageConfigHelpers)
configure_package_config_file(cmake/maul3dConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/maul3dConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/maul3d)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/maul3dConfigVersion.cmake
    COMPATIBILITY SameMinorVersion)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/maul3dConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/maul3dConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/maul3d)

configure_file(cmake/maul3d.pc.in ${CMAKE_CURRENT_BINARY_DIR}/maul3d.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/maul3d.pc
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)

if(MAUL3D_BUILD_TESTBED)
    add_subdirectory(testbed)
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

# White-box suite: reaches internal symbols, which a
# hidden-visibility shared build does not export. Static
# builds gate it; the shared gate is the black-box suites
# plus the installed-package consumer.
if(NOT MAUL3D_BUILD_SHARED)
    add_executable(test_core test/test_core.c)
    target_link_libraries(test_core PRIVATE maul3d)
    target_include_directories(test_core PRIVATE src)
    maul3d_apply_flags(test_core)
    add_test(NAME core COMMAND test_core)
endif()

# White-box suite: reaches internal symbols, which a
# hidden-visibility shared build does not export. Static
# builds gate it; the shared gate is the black-box suites
# plus the installed-package consumer.
if(NOT MAUL3D_BUILD_SHARED)
    add_executable(test_world test/test_world.c)
    target_link_libraries(test_world PRIVATE maul3d)
    target_include_directories(test_world PRIVATE src)
    maul3d_apply_flags(test_world)
    add_test(NAME world COMMAND test_world)
endif()

# White-box suite: reaches internal symbols, which a
# hidden-visibility shared build does not export. Static
# builds gate it; the shared gate is the black-box suites
# plus the installed-package consumer.
if(NOT MAUL3D_BUILD_SHARED)
    add_executable(test_contacts test/test_contacts.c)
    target_link_libraries(test_contacts PRIVATE maul3d)
    target_include_directories(test_contacts PRIVATE src)
    maul3d_apply_flags(test_contacts)
    add_test(NAME contacts COMMAND test_contacts)
endif()

# White-box suite: reaches internal symbols, which a
# hidden-visibility shared build does not export. Static
# builds gate it; the shared gate is the black-box suites
# plus the installed-package consumer.
if(NOT MAUL3D_BUILD_SHARED)
    add_executable(test_distance test/test_distance.c)
    target_link_libraries(test_distance PRIVATE maul3d)
    target_include_directories(test_distance PRIVATE src)
    maul3d_apply_flags(test_distance)
    add_test(NAME distance COMMAND test_distance)
endif()

add_executable(test_solver test/test_solver.c)
target_link_libraries(test_solver PRIVATE maul3d)
maul3d_apply_flags(test_solver)
add_test(NAME solver COMMAND test_solver)

# White-box suite: reaches internal symbols, which a
# hidden-visibility shared build does not export. Static
# builds gate it; the shared gate is the black-box suites
# plus the installed-package consumer.
if(NOT MAUL3D_BUILD_SHARED)
    add_executable(test_quickhull test/test_quickhull.c)
    target_link_libraries(test_quickhull PRIVATE maul3d)
    target_include_directories(test_quickhull PRIVATE src)
    maul3d_apply_flags(test_quickhull)
    add_test(NAME quickhull COMMAND test_quickhull)
endif()

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

add_executable(test_draw test/test_draw.c)
target_link_libraries(test_draw PRIVATE maul3d)
maul3d_apply_flags(test_draw)
add_test(NAME draw COMMAND test_draw)

# White-box suites: static builds only (internal symbols).
if(NOT MAUL3D_BUILD_SHARED)
    add_executable(test_voxstorm test/test_voxstorm.c)
    target_link_libraries(test_voxstorm PRIVATE maul3d)
    target_include_directories(test_voxstorm PRIVATE src)
    maul3d_apply_flags(test_voxstorm)
    add_test(NAME voxstorm COMMAND test_voxstorm)

    add_executable(test_voxel test/test_voxel.c)
    target_link_libraries(test_voxel PRIVATE maul3d)
    target_include_directories(test_voxel PRIVATE src)
    maul3d_apply_flags(test_voxel)
    add_test(NAME voxel COMMAND test_voxel)
endif()

# White-box suite (allocator counters): static builds only, like
# the other internal-symbol suites.
if(NOT MAUL3D_BUILD_SHARED)
    add_executable(test_soak test/test_soak.c)
    target_link_libraries(test_soak PRIVATE maul3d)
    target_include_directories(test_soak PRIVATE src)
    maul3d_apply_flags(test_soak)
    add_test(NAME soak COMMAND test_soak)
endif()

add_executable(test_sequences test/test_sequences.c)
target_link_libraries(test_sequences PRIVATE maul3d)
maul3d_apply_flags(test_sequences)
add_test(NAME sequences COMMAND test_sequences)

add_executable(test_fuzz test/test_fuzz.c)
target_link_libraries(test_fuzz PRIVATE maul3d)
maul3d_apply_flags(test_fuzz)
add_test(NAME fuzz COMMAND test_fuzz)

# White-box suite: reaches internal symbols, which a
# hidden-visibility shared build does not export. Static
# builds gate it; the shared gate is the black-box suites
# plus the installed-package consumer.
if(NOT MAUL3D_BUILD_SHARED)
    add_executable(test_bvh test/test_bvh.c)
    target_link_libraries(test_bvh PRIVATE maul3d)
    target_include_directories(test_bvh PRIVATE src)
    maul3d_apply_flags(test_bvh)
    add_test(NAME bvh COMMAND test_bvh)
endif()
