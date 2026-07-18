# Host-side tests for the Kalman filter (built with native gcc, NOT the ARM toolchain)
#
#   cmake -S tests -B tests/build -G Ninja -DCMAKE_C_COMPILER=gcc
#   cmake --build tests/build
#   ctest --test-dir tests/build   (or run tests/build/test_kalman directly)
cmake_minimum_required(VERSION 3.22)
project(kalman_host_tests C)

set(CMAKE_C_STANDARD 11)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

add_library(kalman ${REPO_ROOT}/Core/Src/kalman.c)
target_include_directories(kalman PUBLIC ${REPO_ROOT}/Core/Inc)

add_executable(test_kalman test_kalman.c)
target_link_libraries(test_kalman kalman)

add_executable(replay_static replay_static.c)
target_link_libraries(replay_static kalman)

if(NOT MSVC)
    target_link_libraries(test_kalman m)
    target_link_libraries(replay_static m)
endif()

enable_testing()
add_test(NAME kalman_unit COMMAND test_kalman)
