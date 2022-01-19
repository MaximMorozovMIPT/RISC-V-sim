cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror" CACHE STRING "Default CXX options" FORCE)
set(CMAKE_CXX_STANDARD 20)

project(simulator CXX)

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_subdirectory(pipeline)

add_executable(sim main.cpp)

target_include_directories(sim PRIVATE ${PROJECT_SOURCE_DIR})

target_link_libraries(sim pipeline)


set(TEST_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/tests)

set(RUN_TESTS 
    ${TEST_FOLDER}/main.cpp
    ${TEST_FOLDER}/integration_test.cpp
    ${TEST_FOLDER}/instructions_test.cpp
    )

add_executable(run_tests ${RUN_TESTS})
target_link_libraries(run_tests pipeline ${GTEST_LIBRARIES} pthread)
target_include_directories(run_tests PRIVATE ${PROJECT_SOURCE_DIR})