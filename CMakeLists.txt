cmake_minimum_required(VERSION 3.16)
project(maul3d VERSION 0.1.0 LANGUAGES C)

option(MAUL3D_SANITIZE "Build with ASAN/UBSAN" OFF)

include(cmake/CompilerFlags.cmake)

add_library(maul3d STATIC
    src/core.c
)
target_include_directories(maul3d PUBLIC include)
target_include_directories(maul3d PRIVATE src)
maul3d_apply_flags(maul3d)

enable_testing()

add_executable(test_determinism test/test_determinism.c)
target_link_libraries(test_determinism PRIVATE maul3d)
maul3d_apply_flags(test_determinism)
add_test(NAME determinism COMMAND test_determinism)
