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
    ${SOURCE_DIR}/aircraft/aircraft_comm.cpp
    ${SOURCE_DIR}/aircraft/aircraft.cpp
    ${SOURCE_DIR}/aircraft/aircraft.hpp
    ${SOURCE_DIR}/aircraft/aircraft_impulse.cpp
    ${SOURCE_DIR}/defines.hpp
    ${SOURCE_DIR}/dynamic/forces.cpp
    ${SOURCE_DIR}/dynamic/forces.hpp
    ${SOURCE_DIR}/dynamic/matrices.cpp
    ${SOURCE_DIR}/dynamic/matrices.hpp
    ${SOURCE_DIR}/main.cpp
    ${SOURCE_DIR}/simulation/atmosphere.cpp
    ${SOURCE_DIR}/simulation/atmosphere.hpp
    ${SOURCE_DIR}/simulation/control.cpp
    ${SOURCE_DIR}/simulation/control.hpp
    ${SOURCE_DIR}/simulation/simulation.cpp
    ${SOURCE_DIR}/simulation/simulation.hpp
    ${SOURCE_DIR}/simulation/uav_state.cpp
    ${SOURCE_DIR}/simulation/uav_state.hpp
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
