cmake_minimum_required(VERSION 3.15...3.29)
project(example LANGUAGES CXX)

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )

set(CMAKE_VERBOSE_MAKEFILE ON)

file(GLOB app_INCLUDE_DIR
"../../driver/include"
)

file(GLOB app_LIB_DIR
"../../driver/lib/${ARCHITECTURE}"
)

include_directories( ${app_INCLUDE_DIR} )

link_directories( ${app_LIB_DIR} )

add_compile_options(-fPIC)
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)
pybind11_add_module(limu_py src/wrapper.cpp)
install(TARGETS limu_py DESTINATION .)