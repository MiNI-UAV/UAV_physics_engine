cmake_minimum_required(VERSION 3.5)
project(uav)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(BUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build)

file(GLOB SOURCES ${SOURCE_DIR}/*.cpp)

include_directories(${INCLUDE_DIR})
link_directories(${LIB_DIR})

link_directories("/usr/lib/x86_64-linux-gnu")

add_executable(uav ${SOURCES})
target_include_directories(uav PUBLIC include)
target_compile_features(uav PUBLIC cxx_std_20)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(uav Eigen3::Eigen)
find_package(cppzmq)
target_link_libraries(uav cppzmq)