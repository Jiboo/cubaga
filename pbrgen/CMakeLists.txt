cmake_minimum_required(VERSION 3.15)
project(cubaga-pbr_gen)

set(CMAKE_CXX_STANDARD 20)

file(GLOB CUBAGA_PBRGEN_SHADERS *.frag *.vert)
hut_gen_spv_bundle(NAMESPACE cubaga_pbrgen_shaders OUTDIR ${CMAKE_CURRENT_BINARY_DIR} SHADER_SOURCES ${CUBAGA_PBRGEN_SHADERS})

add_executable(cubaga-pbrgen main.cpp deps.c ${HUT_BUNDLE_cubaga_pbrgen_shaders_FILES})
target_link_libraries(cubaga-pbrgen hut hut_ktx2 bc7enc_rdo)
target_include_directories(cubaga-pbrgen PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ../deps/stb)
target_compile_definitions(cubaga-pbrgen PRIVATE -DCUBAGA_DEBUG)
