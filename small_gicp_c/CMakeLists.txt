cmake_minimum_required(VERSION 3.16)
project(small_gicp_c VERSION 1.0.0 LANGUAGES C CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Options
option(BUILD_SHARED_LIBS "Build shared library" ON)
option(BUILD_EXAMPLES "Build example programs" ON)
option(BUILD_TESTS "Build test programs" ON)

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(OpenMP)
find_package(TBB)

# Add the parent directory to include path for small_gicp headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../small_gicp/include)

# Build the small_gicp helper library if not already built
if(NOT TARGET small_gicp)
    # Save our current BUILD_TESTS setting before modifying it for small_gicp
    set(SAVED_BUILD_TESTS ${BUILD_TESTS})

    # Disable PCL and other heavy dependencies for C wrapper
    set(BUILD_WITH_PCL OFF CACHE BOOL "Disable PCL for C wrapper" FORCE)
    set(BUILD_TESTS OFF CACHE BOOL "Disable C++ tests for C wrapper" FORCE)
    set(BUILD_EXAMPLES OFF CACHE BOOL "Disable C++ examples for C wrapper" FORCE)
    set(BUILD_BENCHMARKS OFF CACHE BOOL "Disable C++ benchmarks for C wrapper" FORCE)
    set(BUILD_WITH_FAST_GICP OFF CACHE BOOL "Disable fast_gicp for C wrapper" FORCE)
    set(BUILD_WITH_IRIDESCENCE OFF CACHE BOOL "Disable Iridescence for C wrapper" FORCE)

    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../small_gicp ${CMAKE_CURRENT_BINARY_DIR}/small_gicp)

    # Restore our BUILD_TESTS setting for the C wrapper
    set(BUILD_TESTS ${SAVED_BUILD_TESTS})
endif()

# Create the C wrapper library
add_library(small_gicp_c
    src/types.cpp
    src/points/point_cloud.cpp
    src/ann/kdtree.cpp
    src/ann/gaussian_voxelmap.cpp
    src/ann/incremental_voxelmap.cpp
    src/ann/unsafe_kdtree.cpp
    src/util/downsampling.cpp
    src/util/normal_estimation.cpp
    src/util/local_features.cpp
    src/util/sorting.cpp
    src/util/version.cpp
    src/registration/registration.cpp
    src/registration/advanced_registration.cpp
    src/registration/factors.cpp
    src/io/io.cpp
)

# Set include directories
target_include_directories(small_gicp_c
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../small_gicp/include
)

# Link dependencies
target_link_libraries(small_gicp_c
    PUBLIC
        Eigen3::Eigen
    PRIVATE
        small_gicp
)

# Add OpenMP if available
if(OpenMP_CXX_FOUND)
    target_link_libraries(small_gicp_c PUBLIC OpenMP::OpenMP_CXX)
endif()

# Add TBB if available
if(TBB_FOUND)
    target_link_libraries(small_gicp_c PUBLIC TBB::tbb)
endif()

# Set library properties
set_target_properties(small_gicp_c PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
)

# Build examples
if(BUILD_EXAMPLES)
    add_subdirectory(example)
endif()

# Build tests
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Installation
include(GNUInstallDirs)

install(TARGETS small_gicp_c
    EXPORT small_gicp_c-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install all headers
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.h"
)

# Export targets
install(EXPORT small_gicp_c-targets
    FILE small_gicp_c-targets.cmake
    NAMESPACE small_gicp_c::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/small_gicp_c
)

# Create and install package config file
include(CMakePackageConfigHelpers)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/small_gicp_c-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/small_gicp_c-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/small_gicp_c
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/small_gicp_c-config-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/small_gicp_c-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/small_gicp_c-config-version.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/small_gicp_c
)

# Create pkg-config file
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/small_gicp_c.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/small_gicp_c.pc
    @ONLY
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/small_gicp_c.pc
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
)
