cmake_minimum_required(VERSION 3.5)
project(tests) 

# Locate GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
 
# Link runTests with what we want to test and the GTest and pthread library
add_executable(mainTest mainTest.cpp)
target_link_libraries(mainTest ${GTEST_LIBRARIES} pthread)