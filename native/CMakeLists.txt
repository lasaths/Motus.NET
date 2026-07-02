cmake_minimum_required(VERSION 3.16)
project(motus_ompl_native C CXX)

add_library(motus_ompl_native SHARED)

target_include_directories(motus_ompl_native PUBLIC include)

option(MOTUS_USE_OMPL "Link against OMPL C++" OFF)
if(MOTUS_USE_OMPL)
    find_package(ompl REQUIRED)
    target_sources(motus_ompl_native PRIVATE src/motus_ompl_ompl.cpp)
    target_link_libraries(motus_ompl_native PRIVATE ompl::ompl)
    target_compile_definitions(motus_ompl_native PRIVATE MOTUS_HAS_OMPL=1)
else()
    target_sources(motus_ompl_native PRIVATE src/motus_ompl_stub.cpp)
endif()

set_target_properties(motus_ompl_native PROPERTIES
    CXX_STANDARD 17
    POSITION_INDEPENDENT_CODE ON
)
