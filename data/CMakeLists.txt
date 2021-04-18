cmake_minimum_required(VERSION 3.16)
project(cubaga-data)

set(CUBAGA_ENVMAPS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/envmaps)
set(CUBAGA_ENVMAPS_GENDST_DIR ${CMAKE_CURRENT_BINARY_DIR}/genpbr/)

file(GLOB CUBAGA_ENVMAPS_KTX ${CUBAGA_ENVMAPS_SRC_DIR}/*.ktx2)
foreach(envmap_src_ktx ${CUBAGA_ENVMAPS_KTX})
  get_filename_component(envmap_irr_ktx ${envmap_src_ktx} NAME)
  string(REPLACE ".ktx2" ".irr.ktx2" envmap_irr_ktx "${envmap_irr_ktx}")
  set(envmap_irr_ktx ${CUBAGA_ENVMAPS_GENDST_DIR}/${envmap_irr_ktx})

  get_filename_component(envmap_pre_ktx ${envmap_src_ktx} NAME)
  string(REPLACE ".ktx2" ".pre.ktx2" envmap_pre_ktx "${envmap_pre_ktx}")
  set(envmap_pre_ktx ${CUBAGA_ENVMAPS_GENDST_DIR}/${envmap_pre_ktx})

  set(CUBAGA_GEN_ENVMAPS ${CUBAGA_GEN_ENVMAPS};${envmap_pre_ktx};${envmap_irr_ktx})
endforeach()

set(CUBAGA_GEN_BRDFLUT_KTX ${CUBAGA_ENVMAPS_GENDST_DIR}/brdflut.ktx2)
set(CUBAGA_GEN_DATA_CPP ${CMAKE_CURRENT_BINARY_DIR}/cubaga_pbr_data.cpp)

add_custom_command(OUTPUT ${CUBAGA_GEN_BRDFLUT_KTX} ${CUBAGA_GEN_ENVMAPS}
    COMMAND $<TARGET_FILE:cubaga-pbrgen> ${CUBAGA_ENVMAPS_GENDST_DIR} ${CUBAGA_ENVMAPS_SRC_DIR}
    DEPENDS cubaga-pbrgen ${CUBAGA_ENVMAPS_KTX}
    COMMENT "Generating PBR data textures...")

add_custom_command(OUTPUT ${CUBAGA_GEN_DATA_CPP}
    COMMAND $<TARGET_FILE:gen_res> cubaga_pbr_data ${CMAKE_CURRENT_BINARY_DIR} ${CUBAGA_GEN_BRDFLUT_KTX} ${CUBAGA_GEN_ENVMAPS}
    DEPENDS gen_res ${CUBAGA_GEN_BRDFLUT_KTX} ${CUBAGA_GEN_ENVMAPS}
    COMMENT "Generating PBR data headers...")

add_library(cubaga-data ${CUBAGA_GEN_DATA_CPP})
target_include_directories(cubaga-data PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
