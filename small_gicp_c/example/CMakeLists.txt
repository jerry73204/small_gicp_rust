cmake_minimum_required(VERSION 3.16)

# Basic registration example
add_executable(basic_registration_c basic_registration.c)
target_link_libraries(basic_registration_c small_gicp_c)

# Preprocessing example
add_executable(preprocessing_c preprocessing.c)
target_link_libraries(preprocessing_c small_gicp_c)

# KdTree example
add_executable(kdtree_c kdtree.c)
target_link_libraries(kdtree_c small_gicp_c m)

# Test for new covariance and parallel functionality
add_executable(test_covariance_parallel_c test_covariance_parallel.c)
target_link_libraries(test_covariance_parallel_c small_gicp_c m)

# Test for parallel utility functions
add_executable(test_parallel_utilities_c test_parallel_utilities.c)
target_link_libraries(test_parallel_utilities_c small_gicp_c m)

# Test for direct access to internal vectors
add_executable(test_direct_access_c test_direct_access.c)
target_link_libraries(test_direct_access_c small_gicp_c m)

# Test for advanced features (RegistrationHelper, LocalFeatures, Sorting)
add_executable(test_advanced_features_c test_advanced_features.c)
target_link_libraries(test_advanced_features_c small_gicp_c m)

# Test for incremental voxelmap operations
add_executable(test_incremental_voxelmap_c test_incremental_voxelmap.c)
target_link_libraries(test_incremental_voxelmap_c small_gicp_c m)

# Test for advanced ANN features (UnsafeKdTree, KNN settings, projections)
add_executable(test_advanced_ann_c test_advanced_ann.c)
target_link_libraries(test_advanced_ann_c small_gicp_c m)

# Test for direct factor access
add_executable(test_factor_access_c test_factor_access.c)
target_link_libraries(test_factor_access_c small_gicp_c m)

# Install examples
install(TARGETS 
    basic_registration_c
    preprocessing_c
    kdtree_c
    test_covariance_parallel_c
    test_parallel_utilities_c
    test_direct_access_c
    test_advanced_features_c
    test_incremental_voxelmap_c
    test_advanced_ann_c
    test_factor_access_c
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/examples
)