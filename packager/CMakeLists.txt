cmake_minimum_required(VERSION 3.15)
project(cubaga-packager)

set(CMAKE_CXX_STANDARD 20)

file(GLOB CUBAGA_PACKAGER_SRC *.cpp)
add_executable(cubaga-packager ${CUBAGA_PACKAGER_SRC} deps.c)
target_link_libraries(cubaga-packager meshoptimizer mikktspace bc7enc_rdo)
target_include_directories(cubaga-packager PUBLIC
    ../deps/cgltf
    ../deps/stb ${GLM_INCLUDE_DIRS})
target_compile_definitions(cubaga-packager PUBLIC
        -DGLM_FORCE_DEPTH_ZERO_TO_ONE
        -DGLM_FORCE_RADIANS
        -DGLM_FORCE_LEFT_HANDED
        -DCUBAGA_DEBUG)
