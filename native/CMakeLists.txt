cmake_minimum_required(VERSION 3.16)
project(motus_ompl_native C CXX)

add_library(motus_ompl_native SHARED)

target_include_directories(motus_ompl_native PUBLIC include)

option(MOTUS_USE_OMPL "Link against OMPL C++" OFF)
if(MOTUS_USE_OMPL)
    find_package(ompl CONFIG QUIET)
    if(NOT ompl_FOUND)
        find_package(ompl REQUIRED)
    endif()
    target_sources(motus_ompl_native PRIVATE src/motus_ompl_ompl.cpp)
    target_compile_definitions(motus_ompl_native PRIVATE MOTUS_HAS_OMPL=1)
    if(TARGET ompl::ompl)
        target_link_libraries(motus_ompl_native PRIVATE ompl::ompl)
    elseif(DEFINED OMPL_LIBRARIES)
        if(OMPL_INCLUDE_DIRS)
            target_include_directories(motus_ompl_native PRIVATE ${OMPL_INCLUDE_DIRS})
        endif()
        # Apt FindOMPL.cmake may list stale absolute Boost paths; link ompl by name.
        target_link_libraries(motus_ompl_native PRIVATE ompl)
    else()
        target_link_libraries(motus_ompl_native PRIVATE ompl)
    endif()
else()
    target_sources(motus_ompl_native PRIVATE src/motus_ompl_stub.cpp)
endif()

set_target_properties(motus_ompl_native PROPERTIES
    CXX_STANDARD 17
    POSITION_INDEPENDENT_CODE ON
)
