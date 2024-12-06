cmake_minimum_required(VERSION 3.5.0)

PROJECT(demo)

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )

message( STATUS "Architecture: ${ARCHITECTURE}" )

find_package(Boost REQUIRED COMPONENTS system thread)

message("Boost version: ${Boost_VERSION}")

find_package(OpenCV REQUIRED)

message("OpenCV version: ${OpenCV_VERSION}")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package (Threads)

find_package(PCL REQUIRED)

file(GLOB app_INCLUDE_DIR
    "../../driver/include"
)

file(GLOB app_LIB_DIR
    "../../driver/lib/${ARCHITECTURE}"
)

file(GLOB app_SRC
    "src/*.cpp"
)

include_directories( ${app_INCLUDE_DIR} )

link_directories( ${app_LIB_DIR} )

ADD_EXECUTABLE( demo ${app_SRC})

TARGET_LINK_LIBRARIES( demo ${Boost_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} limu_driver )

