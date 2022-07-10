cmake_minimum_required(VERSION 2.8.3)
project(ArmourShoot)

#set(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")
#-DEIGEN_USE_MKL_ALL")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPATH=\"\\\"${PROJECT_SOURCE_DIR}\\\"\"")

set(GXI_API_DIR "/usr/lib/")

#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV REQUIRED)
find_package( Ceres REQUIRED)

#find_package(Ceres REQUIRED)
#include_directories(${CERES_INCLUDE_DIRS})

message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

include_directories(
        #${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/src/armour
        ${PROJECT_SOURCE_DIR}/src/IndusCamera
        ${PROJECT_SOURCE_DIR}/src/serial
        ${PROJECT_SOURCE_DIR}/src/
        ${PROJECT_SOURCE_DIR}/src/classifier
        ${PROJECT_SOURCE_DIR}/src/predictor
        ${PROJECT_SOURCE_DIR}/src/energy
        ${PROJECT_SOURCE_DIR}/src/base
        ${PROJECT_SOURCE_DIR}/src/topaimer
        ${PROJECT_SOURCE_DIR}/src/shooter
        ${PROJECT_SOURCE_DIR}/src/filter
        ${CERES_INCLUDE_DIRS}
)

add_subdirectory(src)

add_executable(ArmourShoot main.cpp record/Record.cpp record/Record.h )
target_link_libraries(ArmourShoot src)

