cmake_minimum_required(VERSION 3.16)
project(maul3d VERSION 0.1.0 LANGUAGES C)

option(MAUL3D_SANITIZE "Build with ASAN/UBSAN" OFF)

include(cmake/CompilerFlags.cmake)

add_library(maul3d STATIC
    src/allocator.c
    src/body.c
    src/core.c
    src/math_functions.c
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

add_executable(test_determinism test/test_determinism.c)
target_link_libraries(test_determinism PRIVATE maul3d)
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
maul3d_apply_flags(test_world)
add_test(NAME world COMMAND test_world)

add_executable(test_journal test/test_journal.c)
target_link_libraries(test_journal PRIVATE maul3d)
maul3d_apply_flags(test_journal)
add_test(NAME journal COMMAND test_journal)
