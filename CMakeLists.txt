cmake_minimum_required(VERSION 3.16)
project(cubaga)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_COMPILE_WARNING_MODE HIGH)

set(HUT_ENABLE_PROFILING TRUE)
set(HUT_DEFINITIONS ${HUT_DEFINITIONS} -DHUT_ENABLE_VALIDATION)
set(HUT_DEFINITIONS ${HUT_DEFINITIONS} -DHUT_ENABLE_VALIDATION_DEBUG)
add_subdirectory(deps/libhut)

add_subdirectory(deps/meshoptimizer)

add_library(mikktspace deps/ext-mikktspace/mikktspace.c)
target_include_directories(mikktspace PUBLIC deps/ext-mikktspace)

add_library(bc7enc_rdo
    deps/bc7enc_rdo/bc7decomp.cpp deps/bc7enc_rdo/bc7decomp_ref.cpp deps/bc7enc_rdo/bc7enc.cpp deps/bc7enc_rdo/utils.cpp
    deps/bc7enc_rdo/ert.cpp deps/bc7enc_rdo/lodepng.cpp deps/bc7enc_rdo/rdo_bc_encoder.cpp deps/bc7enc_rdo/rgbcx.cpp)
target_include_directories(bc7enc_rdo PUBLIC deps/bc7enc_rdo)

add_subdirectory(pbrgen)
add_subdirectory(packager)
add_subdirectory(data)

add_subdirectory(viewer)
add_subdirectory(debugger)
