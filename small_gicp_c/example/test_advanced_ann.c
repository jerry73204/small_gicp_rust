#include <math.h>
#include <small_gicp_c.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// Helper function to create a test point cloud
small_gicp_error_t create_test_point_cloud(size_t num_points,
                                           small_gicp_point_cloud_t **cloud) {
  small_gicp_error_t error = small_gicp_point_cloud_create(cloud);
  if (error != SMALL_GICP_SUCCESS)
    return error;

  error = small_gicp_point_cloud_resize(*cloud, num_points);
  if (error != SMALL_GICP_SUCCESS)
    return error;

  // Create a test pattern (points on a sphere)
  srand(42); // Fixed seed for reproducibility
  for (size_t i = 0; i < num_points; i++) {
    double theta = 2.0 * M_PI * i / num_points;
    double phi = M_PI * (rand() % 100) / 100.0;
    double radius = 1.0 + 0.1 * sin(theta * 3);

    double x = radius * sin(phi) * cos(theta);
    double y = radius * sin(phi) * sin(theta);
    double z = radius * cos(phi);

    error = small_gicp_point_cloud_set_point(*cloud, i, x, y, z);
    if (error != SMALL_GICP_SUCCESS)
      return error;
  }

  return SMALL_GICP_SUCCESS;
}

int test_unsafe_kdtree_basic() {
  printf("Testing UnsafeKdTree basic functionality...\n");

  small_gicp_point_cloud_t *cloud = NULL;
  small_gicp_unsafe_kdtree_t *unsafe_kdtree = NULL;
  small_gicp_error_t error;

  // Create test cloud
  error = create_test_point_cloud(1000, &cloud);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create test cloud\n");
    return 1;
  }

  // Create UnsafeKdTree with default configuration
  printf("Creating UnsafeKdTree with default config...\n");
  error = small_gicp_unsafe_kdtree_create(cloud, &unsafe_kdtree);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create UnsafeKdTree\n");
    return 1;
  }

  // Test nearest neighbor search
  printf("Testing nearest neighbor search...\n");
  size_t index;
  double distance;
  error = small_gicp_unsafe_kdtree_nearest_neighbor_search(
      unsafe_kdtree, 0.0, 0.0, 1.0, &index, &distance);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Nearest neighbor search failed\n");
    return 1;
  }

  if (index != SIZE_MAX) {
    printf("Nearest neighbor found: index=%zu, distance=%.6f\n", index,
           distance);
  } else {
    printf("No nearest neighbor found\n");
  }

  // Test k-NN search
  printf("Testing k-NN search (k=5)...\n");
  const int k = 5;
  size_t indices[k];
  double distances[k];
  error = small_gicp_unsafe_kdtree_knn_search(unsafe_kdtree, 0.0, 0.0, 1.0, k,
                                              indices, distances);
  if (error != SMALL_GICP_SUCCESS) {
    printf("k-NN search failed\n");
    return 1;
  }

  printf("k-NN results:\n");
  for (int i = 0; i < k; i++) {
    if (indices[i] != SIZE_MAX) {
      printf("  [%d] index=%zu, distance=%.6f\n", i, indices[i], distances[i]);
    }
  }

  // Cleanup
  small_gicp_unsafe_kdtree_destroy(unsafe_kdtree);
  small_gicp_point_cloud_destroy(cloud);

  printf("UnsafeKdTree basic test passed!\n");
  return 0;
}

int test_projection_types() {
  printf("Testing different projection types...\n");

  small_gicp_point_cloud_t *cloud = NULL;
  small_gicp_unsafe_kdtree_t *axis_aligned_tree = NULL;
  small_gicp_unsafe_kdtree_t *normal_tree = NULL;
  small_gicp_error_t error;

  // Create test cloud
  error = create_test_point_cloud(500, &cloud);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create test cloud\n");
    return 1;
  }

  // Test AxisAligned projection
  printf("Testing AxisAligned projection...\n");
  small_gicp_kdtree_config_extended_t axis_config;
  small_gicp_create_default_kdtree_config_extended(&axis_config);
  axis_config.projection.type = SMALL_GICP_PROJECTION_AXIS_ALIGNED;
  axis_config.projection.max_scan_count = 64;
  axis_config.max_leaf_size = 10;

  error = small_gicp_unsafe_kdtree_create_with_extended_config(
      cloud, &axis_config, &axis_aligned_tree);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create AxisAligned UnsafeKdTree\n");
    return 1;
  }

  // Test Normal projection
  printf("Testing Normal projection...\n");
  small_gicp_kdtree_config_extended_t normal_config;
  small_gicp_create_default_kdtree_config_extended(&normal_config);
  normal_config.projection.type = SMALL_GICP_PROJECTION_NORMAL;
  normal_config.projection.max_scan_count = 128;
  normal_config.max_leaf_size = 15;

  error = small_gicp_unsafe_kdtree_create_with_extended_config(
      cloud, &normal_config, &normal_tree);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create Normal projection UnsafeKdTree\n");
    return 1;
  }

  // Compare search results
  printf("Comparing search results between projection types...\n");
  size_t axis_index, normal_index;
  double axis_distance, normal_distance;

  error = small_gicp_unsafe_kdtree_nearest_neighbor_search(
      axis_aligned_tree, 0.5, 0.5, 0.5, &axis_index, &axis_distance);
  if (error != SMALL_GICP_SUCCESS) {
    printf("AxisAligned search failed\n");
    return 1;
  }

  error = small_gicp_unsafe_kdtree_nearest_neighbor_search(
      normal_tree, 0.5, 0.5, 0.5, &normal_index, &normal_distance);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Normal projection search failed\n");
    return 1;
  }

  printf("AxisAligned: index=%zu, distance=%.6f\n", axis_index, axis_distance);
  printf("Normal: index=%zu, distance=%.6f\n", normal_index, normal_distance);

  // Cleanup
  small_gicp_unsafe_kdtree_destroy(axis_aligned_tree);
  small_gicp_unsafe_kdtree_destroy(normal_tree);
  small_gicp_point_cloud_destroy(cloud);

  printf("Projection types test passed!\n");
  return 0;
}

int test_knn_settings() {
  printf("Testing KNN settings with early termination...\n");

  small_gicp_point_cloud_t *cloud = NULL;
  small_gicp_unsafe_kdtree_t *kdtree = NULL;
  small_gicp_error_t error;

  // Create test cloud
  error = create_test_point_cloud(800, &cloud);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create test cloud\n");
    return 1;
  }

  // Create UnsafeKdTree
  error = small_gicp_unsafe_kdtree_create(cloud, &kdtree);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create UnsafeKdTree\n");
    return 1;
  }

  // Test exact search (epsilon = 0.0)
  printf("Testing exact search (epsilon=0.0)...\n");
  small_gicp_knn_setting_t exact_setting;
  small_gicp_create_default_knn_setting(&exact_setting);
  exact_setting.epsilon = 0.0;

  size_t exact_index;
  double exact_distance;
  error = small_gicp_unsafe_kdtree_nearest_neighbor_search_with_setting(
      kdtree, 0.0, 0.0, 0.0, &exact_setting, &exact_index, &exact_distance);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Exact search failed\n");
    return 1;
  }

  printf("Exact search result: index=%zu, distance=%.6f\n", exact_index,
         exact_distance);

  // Test approximate search (epsilon = 0.1)
  printf("Testing approximate search (epsilon=0.1)...\n");
  small_gicp_knn_setting_t approx_setting;
  small_gicp_create_default_knn_setting(&approx_setting);
  approx_setting.epsilon = 0.1;

  size_t approx_index;
  double approx_distance;
  error = small_gicp_unsafe_kdtree_nearest_neighbor_search_with_setting(
      kdtree, 0.0, 0.0, 0.0, &approx_setting, &approx_index, &approx_distance);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Approximate search failed\n");
    return 1;
  }

  printf("Approximate search result: index=%zu, distance=%.6f\n", approx_index,
         approx_distance);

  // Test k-NN with early termination
  printf("Testing k-NN with early termination...\n");
  const int k = 3;
  size_t indices[k];
  double distances[k];

  error = small_gicp_unsafe_kdtree_knn_search_with_setting(
      kdtree, 0.0, 0.0, 0.0, k, &approx_setting, indices, distances);
  if (error != SMALL_GICP_SUCCESS) {
    printf("k-NN with setting failed\n");
    return 1;
  }

  printf("k-NN with early termination results:\n");
  for (int i = 0; i < k; i++) {
    if (indices[i] != SIZE_MAX) {
      printf("  [%d] index=%zu, distance=%.6f\n", i, indices[i], distances[i]);
    }
  }

  // Cleanup
  small_gicp_unsafe_kdtree_destroy(kdtree);
  small_gicp_point_cloud_destroy(cloud);

  printf("KNN settings test passed!\n");
  return 0;
}

int test_regular_kdtree_enhancements() {
  printf("Testing enhanced regular KdTree with settings...\n");

  small_gicp_point_cloud_t *cloud = NULL;
  small_gicp_kdtree_t *kdtree = NULL;
  small_gicp_error_t error;

  // Create test cloud
  error = create_test_point_cloud(600, &cloud);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create test cloud\n");
    return 1;
  }

  // Create KdTree with extended configuration
  printf("Creating KdTree with extended configuration...\n");
  small_gicp_kdtree_config_extended_t config;
  small_gicp_create_default_kdtree_config_extended(&config);
  config.builder_type = SMALL_GICP_KDTREE_BUILDER_DEFAULT;
  config.max_leaf_size = 25;
  config.projection.type = SMALL_GICP_PROJECTION_AXIS_ALIGNED;
  config.projection.max_scan_count = 64;

  error =
      small_gicp_kdtree_create_with_extended_config(cloud, &config, &kdtree);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create enhanced KdTree\n");
    return 1;
  }

  // Test enhanced search methods
  printf("Testing enhanced search methods...\n");
  small_gicp_knn_setting_t setting;
  small_gicp_create_default_knn_setting(&setting);
  setting.epsilon = 0.05;

  size_t index;
  double distance;
  error = small_gicp_kdtree_nearest_neighbor_search_with_setting(
      kdtree, 0.2, 0.3, 0.4, &setting, &index, &distance);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Enhanced nearest neighbor search failed\n");
    return 1;
  }

  printf("Enhanced NN search: index=%zu, distance=%.6f\n", index, distance);

  // Test enhanced k-NN search
  const int k = 4;
  size_t indices[k];
  double distances[k];
  error = small_gicp_kdtree_knn_search_with_setting(
      kdtree, 0.2, 0.3, 0.4, k, &setting, indices, distances);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Enhanced k-NN search failed\n");
    return 1;
  }

  printf("Enhanced k-NN results:\n");
  for (int i = 0; i < k; i++) {
    if (indices[i] != SIZE_MAX) {
      printf("  [%d] index=%zu, distance=%.6f\n", i, indices[i], distances[i]);
    }
  }

  // Cleanup
  small_gicp_kdtree_destroy(kdtree);
  small_gicp_point_cloud_destroy(cloud);

  printf("Regular KdTree enhancements test passed!\n");
  return 0;
}

int test_parallel_builders() {
  printf("Testing parallel KdTree builders...\n");

  small_gicp_point_cloud_t *cloud = NULL;
  small_gicp_error_t error;

  // Create test cloud
  error = create_test_point_cloud(1500, &cloud);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create test cloud\n");
    return 1;
  }

  // Test different builder types
  small_gicp_kdtree_builder_type_t builders[] = {
      SMALL_GICP_KDTREE_BUILDER_DEFAULT,
#ifdef SMALL_GICP_HAS_OPENMP
      SMALL_GICP_KDTREE_BUILDER_OPENMP,
#endif
#ifdef SMALL_GICP_HAS_TBB
      SMALL_GICP_KDTREE_BUILDER_TBB
#endif
  };

  const char *builder_names[] = {"Default",
#ifdef SMALL_GICP_HAS_OPENMP
                                 "OpenMP",
#endif
#ifdef SMALL_GICP_HAS_TBB
                                 "TBB"
#endif
  };

  size_t num_builders = sizeof(builders) / sizeof(builders[0]);

  for (size_t i = 0; i < num_builders; i++) {
    printf("Testing %s builder...\n", builder_names[i]);

    small_gicp_kdtree_config_extended_t config;
    small_gicp_create_default_kdtree_config_extended(&config);
    config.builder_type = builders[i];
    config.num_threads = 2;
    config.max_leaf_size = 20;

    // Test UnsafeKdTree
    small_gicp_unsafe_kdtree_t *unsafe_kdtree = NULL;
    error = small_gicp_unsafe_kdtree_create_with_extended_config(
        cloud, &config, &unsafe_kdtree);
    if (error != SMALL_GICP_SUCCESS) {
      printf("Failed to create UnsafeKdTree with %s builder\n",
             builder_names[i]);
      return 1;
    }

    // Quick search test
    size_t index;
    double distance;
    error = small_gicp_unsafe_kdtree_nearest_neighbor_search(
        unsafe_kdtree, 0.0, 0.0, 0.0, &index, &distance);
    if (error != SMALL_GICP_SUCCESS) {
      printf("Search failed with %s builder\n", builder_names[i]);
      return 1;
    }

    printf("  %s builder: index=%zu, distance=%.6f\n", builder_names[i], index,
           distance);

    small_gicp_unsafe_kdtree_destroy(unsafe_kdtree);

    // Test regular KdTree
    small_gicp_kdtree_t *kdtree = NULL;
    error =
        small_gicp_kdtree_create_with_extended_config(cloud, &config, &kdtree);
    if (error != SMALL_GICP_SUCCESS) {
      printf("Failed to create KdTree with %s builder\n", builder_names[i]);
      return 1;
    }

    small_gicp_kdtree_destroy(kdtree);
  }

  // Cleanup
  small_gicp_point_cloud_destroy(cloud);

  printf("Parallel builders test passed!\n");
  return 0;
}

int main() {
  printf("Running tests for advanced ANN features\n\n");

  if (test_unsafe_kdtree_basic() != 0) {
    printf("UnsafeKdTree basic test failed!\n");
    return 1;
  }

  printf("\n");

  if (test_projection_types() != 0) {
    printf("Projection types test failed!\n");
    return 1;
  }

  printf("\n");

  if (test_knn_settings() != 0) {
    printf("KNN settings test failed!\n");
    return 1;
  }

  printf("\n");

  if (test_regular_kdtree_enhancements() != 0) {
    printf("Regular KdTree enhancements test failed!\n");
    return 1;
  }

  printf("\n");

  if (test_parallel_builders() != 0) {
    printf("Parallel builders test failed!\n");
    return 1;
  }

  printf("\nAll advanced ANN feature tests passed successfully!\n");
  return 0;
}
