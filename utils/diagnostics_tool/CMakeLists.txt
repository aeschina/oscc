cmake_minimum_required(VERSION 2.8)

project(diagnostics-tool)

find_package(PkgConfig REQUIRED)

if(KIA_SOUL)
    add_definitions(-DKIA_SOUL)
elseif(KIA_SOUL_EV)
    add_definitions(-DKIA_SOUL_EV)
endif()

add_executable(
    diagnostics-tool
    src/main.c
    ../../api/src/oscc.c)

target_include_directories(
    diagnostics-tool
    PRIVATE
    ../../api/include)

target_link_libraries(
    diagnostics-tool
    PRIVATE)
