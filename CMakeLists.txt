cmake_minimum_required(VERSION 3.5)
project(uav)

add_subdirectory(lib/UAV_common)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
set(CMAKE_CXX_STANDARD 20)

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(BUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build)

file(GLOB SOURCES ${SOURCE_DIR}/*.cpp)
file(GLOB SOURCES2 ${SOURCE_DIR}/aircrafts/*.cpp)

include_directories(${INCLUDE_DIR})
link_directories(${LIB_DIR})
link_directories(${CMAKE_SOURCE_DIR}/lib/UAV_common)

link_directories("/usr/lib/x86_64-linux-gnu")
link_directories("/usr/local/lib")
link_directories("/usr/local/include")

add_executable(uav ${SOURCES} ${SOURCES2})
set_property(TARGET uav PROPERTY CXX_STANDARD 20)
target_include_directories(uav PUBLIC include)
target_compile_features(uav PUBLIC cxx_std_20)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(uav Eigen3::Eigen)
find_package(cppzmq)
target_link_libraries(uav cppzmq)
find_package(cxxopts)
target_link_libraries(uav cxxopts::cxxopts)
target_link_libraries(uav common) 
target_include_directories(uav PRIVATE ${CMAKE_SOURCE_DIR}/lib/UAV_common/header)
