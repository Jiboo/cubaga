cmake_minimum_required(VERSION 3.15)
project(cubaga-viewer)

set(CMAKE_CXX_STANDARD 20)

file(GLOB VIEWER_SHADERS *.frag *.vert)
hut_gen_spv_bundle(NAMESPACE viewer_shaders OUTDIR ${CMAKE_CURRENT_BINARY_DIR} SHADER_SOURCES ${VIEWER_SHADERS})
set(viewer_shaders_cpp ${CMAKE_CURRENT_BINARY_DIR}/viewer_shaders.cpp)

add_executable(cubaga-viewer main.cpp ${viewer_shaders_cpp} ${HUT_BUNDLE_viewer_shaders_FILES})
target_link_libraries(cubaga-viewer hut hut_imgui_impl meshoptimizer)
target_include_directories(cubaga-viewer PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
