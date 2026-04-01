cmake_minimum_required(VERSION 3.16.3)

# Allow standalone build: cmake -S tests/cpp -B tests/cpp/build
if(NOT PROJECT_SOURCE_DIR OR PROJECT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    project(flexiv_rt_safety_tests CXX)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
    enable_testing()
else()
    set(_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
endif()

find_package(GTest REQUIRED)

add_executable(test_rt_safety test_rt_safety.cpp)
target_include_directories(test_rt_safety PRIVATE ${_INCLUDE_DIR})
target_link_libraries(test_rt_safety GTest::gtest_main pthread)
add_test(NAME rt_safety COMMAND test_rt_safety)

add_executable(test_trajectory test_trajectory.cpp)
target_include_directories(test_trajectory PRIVATE ${_INCLUDE_DIR})
target_link_libraries(test_trajectory GTest::gtest_main pthread)
add_test(NAME trajectory COMMAND test_trajectory)
