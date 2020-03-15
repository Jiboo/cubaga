cmake_minimum_required(VERSION 3.15)
project(cubaga)

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(deps/libhut)
add_subdirectory(deps/meshoptimizer)

add_library(mikktspace deps/ext-mikktspace/mikktspace.c)
target_include_directories(mikktspace PUBLIC deps/ext-mikktspace)

add_library(bc7enc16 deps/bc7enc16/bc7enc16.c)
target_include_directories(bc7enc16 PUBLIC deps/bc7enc16)

add_subdirectory(debugger)
add_subdirectory(packager)
add_subdirectory(viewer)
