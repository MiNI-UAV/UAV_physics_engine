cmake_minimum_required(VERSION 3.5)
project(uav)

add_subdirectory(lib/UAV_common)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
set(CMAKE_CXX_STANDARD 20)

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(BUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build)

set(SOURCE_FILES
    ${SOURCE_DIR}/aircrafts/aircraft_comm.cpp
    ${SOURCE_DIR}/aircrafts/aircraft.cpp
    ${SOURCE_DIR}/aircrafts/aircraft_impulse.cpp
    ${SOURCE_DIR}/aircrafts/copter.hpp
    ${SOURCE_DIR}/aircrafts/rocket.hpp
    ${SOURCE_DIR}/atmosphere.cpp
    ${SOURCE_DIR}/atmosphere.hpp
    ${SOURCE_DIR}/control.cpp
    ${SOURCE_DIR}/control.hpp
    ${SOURCE_DIR}/defines.hpp
    ${SOURCE_DIR}/forces.cpp
    ${SOURCE_DIR}/forces.hpp
    ${SOURCE_DIR}/main.cpp
    ${SOURCE_DIR}/matrices.cpp
    ${SOURCE_DIR}/matrices.hpp
    ${SOURCE_DIR}/simulation.cpp
    ${SOURCE_DIR}/simulation.hpp
    ${SOURCE_DIR}/uav_state.cpp
    ${SOURCE_DIR}/uav_state.hpp
)


include_directories(${INCLUDE_DIR})
link_directories(${LIB_DIR}/UAV_common)

link_directories("/usr/lib/x86_64-linux-gnu")
link_directories("/usr/local/lib")
link_directories("/usr/local/include")

add_executable(uav ${SOURCE_FILES})
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
