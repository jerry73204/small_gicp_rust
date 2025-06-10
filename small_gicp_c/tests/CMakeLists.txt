cmake_minimum_required(VERSION 3.16)

# Add Unity test framework
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/Unity ${CMAKE_CURRENT_BINARY_DIR}/unity)

# Create Unity library target if not already created by the submodule
if(NOT TARGET unity)
    add_library(unity
        ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/Unity/src/unity.c
    )
    target_include_directories(unity PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/Unity/src
    )
    target_compile_definitions(unity PUBLIC
        UNITY_INCLUDE_DOUBLE=1
        UNITY_INCLUDE_FLOAT=1
        UNITY_SUPPORT_64=1
        UNITY_DOUBLE_PRECISION=1
    )
endif()

# Common test utilities
add_library(test_common STATIC
    test_common.c
)
target_include_directories(test_common PUBLIC .)
target_link_libraries(test_common PUBLIC small_gicp_c unity)

# Test data directory
set(TEST_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/data")
target_compile_definitions(test_common PUBLIC TEST_DATA_DIR="${TEST_DATA_DIR}")

# Helper function to create tests
function(add_c_test test_name source_file)
    add_executable(${test_name} ${source_file})
    target_link_libraries(${test_name} test_common m)  # Add math library
    add_test(NAME ${test_name} COMMAND ${test_name})

    # Set test properties
    set_tests_properties(${test_name} PROPERTIES
        TIMEOUT 60
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )

    # Add to global list of C wrapper tests
    set_property(GLOBAL APPEND PROPERTY C_WRAPPER_TESTS ${test_name})
endfunction()

# Automatically discover and add all test files
file(GLOB_RECURSE TEST_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/test_*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/integration/test_*.c"
)

# Create tests from discovered files
foreach(test_source ${TEST_SOURCES})
    # Get relative path and extract test name
    file(RELATIVE_PATH rel_path ${CMAKE_CURRENT_SOURCE_DIR} ${test_source})
    get_filename_component(test_name ${test_source} NAME_WE)

    # Add the test
    add_c_test(${test_name} ${rel_path})
    message(STATUS "Added C wrapper test: ${test_name}")
endforeach()

# Get the list of all C wrapper tests
get_property(ALL_C_TESTS GLOBAL PROPERTY C_WRAPPER_TESTS)

# Create a target to build all C wrapper tests
add_custom_target(c_wrapper_tests
    DEPENDS ${ALL_C_TESTS}
    COMMENT "Building all C wrapper tests"
)

# Custom target to run all tests
add_custom_target(run_c_tests
    COMMAND ${CMAKE_COMMAND} -E echo "Running C wrapper tests..."
    DEPENDS c_wrapper_tests
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Running all C wrapper tests"
)

# Add individual test run commands
foreach(test_name ${ALL_C_TESTS})
    add_custom_command(TARGET run_c_tests POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo "Running ${test_name}..."
        COMMAND ./${test_name}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Running ${test_name}"
    )
endforeach()
