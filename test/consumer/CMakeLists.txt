# The package gate (2d-4): a separate project that knows nothing
# about the maul3d source tree and consumes only the INSTALLED
# package. If this builds, links, and runs, the package is real.
cmake_minimum_required(VERSION 3.16)
project(maul3d_consumer LANGUAGES C)

find_package(maul3d 0.3 REQUIRED CONFIG)

add_executable(consumer consumer.c)
target_link_libraries(consumer PRIVATE maul3d::maul3d)
