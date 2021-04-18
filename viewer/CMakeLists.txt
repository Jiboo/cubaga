cmake_minimum_required(VERSION 3.15)
project(cubaga-viewer)

set(CMAKE_CXX_STANDARD 20)

file(GLOB VIEWER_SHADERS *.frag *.vert)
hut_gen_spv_bundle(NAMESPACE viewer_shaders OUTDIR ${CMAKE_CURRENT_BINARY_DIR} SHADER_SOURCES ${VIEWER_SHADERS})

add_executable(cubaga-viewer main.cpp ${HUT_BUNDLE_viewer_shaders_FILES})
target_link_libraries(cubaga-viewer hut hut_ktx2 hut_imgui_impl meshoptimizer cubaga-data)
target_include_directories(cubaga-viewer PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
