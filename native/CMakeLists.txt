cmake_minimum_required(VERSION 3.16)
project(motus_native C CXX)

add_library(motus_native SHARED
    src/motus_error.cpp
)

target_include_directories(motus_native PUBLIC include)

option(MOTUS_USE_OMPL "Link against OMPL C++" OFF)
option(MOTUS_USE_FCL "Link against FCL C++" OFF)

if(MOTUS_USE_OMPL)
    find_package(ompl CONFIG QUIET)
    if(NOT ompl_FOUND)
        find_package(ompl REQUIRED)
    endif()
    target_sources(motus_native PRIVATE src/motus_ompl_ompl.cpp)
    target_compile_definitions(motus_native PRIVATE MOTUS_HAS_OMPL=1)
    if(TARGET ompl::ompl)
        target_link_libraries(motus_native PRIVATE ompl::ompl)
    else()
        target_link_libraries(motus_native PRIVATE ompl)
    endif()
else()
    target_sources(motus_native PRIVATE src/motus_ompl_stub.cpp)
endif()

if(MOTUS_USE_FCL)
    find_package(fcl CONFIG QUIET)
    if(NOT fcl_FOUND)
        find_package(PkgConfig REQUIRED)
        pkg_check_modules(FCL REQUIRED fcl)
        target_include_directories(motus_native PRIVATE ${FCL_INCLUDE_DIRS})
        target_link_libraries(motus_native PRIVATE ${FCL_LIBRARIES})
    else()
        target_link_libraries(motus_native PRIVATE fcl)
    endif()
    target_sources(motus_native PRIVATE src/motus_fcl_fcl.cpp)
    target_compile_definitions(motus_native PRIVATE MOTUS_HAS_FCL=1)
else()
    target_sources(motus_native PRIVATE src/motus_fcl_stub.cpp)
endif()

set_target_properties(motus_native PROPERTIES
    CXX_STANDARD 17
    POSITION_INDEPENDENT_CODE ON
    OUTPUT_NAME "motus_native"
)

# Legacy alias for older loader names
add_library(motus_ompl_native ALIAS motus_native)
