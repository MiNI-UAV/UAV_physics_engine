cmake_minimum_required(VERSION 3.5)
project(UAV_physics_engine) # this does not matter for our simple case
add_executable(main src/main.cpp src/constants.cpp) # this determines the executable name
target_include_directories(main PUBLIC include)
target_compile_features(main PUBLIC cxx_std_20) # we need C++20
add_subdirectory(lib/eigen)
link_directories(${CMAKE_SOURCE_DIR}/lib)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(main Eigen3::Eigen)
