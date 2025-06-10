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

  // Create a simple test pattern (random points in a unit cube)
  srand(42); // Fixed seed for reproducibility
  for (size_t i = 0; i < num_points; i++) {
    double x = (double)rand() / RAND_MAX * 2.0 - 1.0; // [-1, 1]
    double y = (double)rand() / RAND_MAX * 2.0 - 1.0; // [-1, 1]
    double z = (double)rand() / RAND_MAX * 2.0 - 1.0; // [-1, 1]

    error = small_gicp_point_cloud_set_point(*cloud, i, x, y, z);
    if (error != SMALL_GICP_SUCCESS)
      return error;

    // Add simple normals
    double norm = sqrt(x * x + y * y + z * z);
    if (norm > 1e-6) {
      error = small_gicp_point_cloud_set_normal(*cloud, i, x / norm, y / norm,
                                                z / norm);
      if (error != SMALL_GICP_SUCCESS)
        return error;
    }
  }

  return SMALL_GICP_SUCCESS;
}

int test_flat_container_points() {
  printf("Testing flat container with points only...\n");

  small_gicp_incremental_voxelmap_t *voxelmap = NULL;
  small_gicp_point_cloud_t *cloud = NULL;
  small_gicp_error_t error;

  // Create test cloud
  error = create_test_point_cloud(500, &cloud);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create test cloud\n");
    return 1;
  }

  // Create incremental voxelmap
  error = small_gicp_incremental_voxelmap_create(
      0.1, SMALL_GICP_VOXEL_FLAT_POINTS, &voxelmap);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create incremental voxelmap\n");
    return 1;
  }

  // Insert point cloud
  printf("Inserting point cloud...\n");
  error = small_gicp_incremental_voxelmap_insert(voxelmap, cloud);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to insert point cloud\n");
    return 1;
  }

  // Check voxelmap size
  size_t size, num_voxels;
  small_gicp_incremental_voxelmap_size(voxelmap, &size);
  small_gicp_incremental_voxelmap_num_voxels(voxelmap, &num_voxels);
  printf("Voxelmap contains %zu points in %zu voxels\n", size, num_voxels);

  // Test search operations
  printf("Testing nearest neighbor search...\n");
  size_t index;
  double distance;
  error = small_gicp_incremental_voxelmap_nearest_neighbor_search(
      voxelmap, 0.0, 0.0, 0.0, &index, &distance);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Nearest neighbor search failed with error code %d\n", error);
    return 1;
  }
  if (index == SIZE_MAX) {
    printf("No nearest neighbor found\n");
  } else {
    printf("Nearest neighbor: index=%zu, distance=%.6f\n", index, distance);
  }

  // Test k-NN search
  printf("Testing k-NN search (k=5)...\n");
  const int k = 5;
  size_t indices[k];
  double distances[k];
  error = small_gicp_incremental_voxelmap_knn_search(voxelmap, 0.0, 0.0, 0.0, k,
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

  // Test voxel coordinate calculations
  printf("Testing voxel coordinate calculations...\n");
  int coord_x, coord_y, coord_z;
  error = small_gicp_incremental_voxelmap_get_voxel_coords(
      voxelmap, 0.25, 0.35, 0.45, &coord_x, &coord_y, &coord_z);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to get voxel coordinates\n");
    return 1;
  }
  printf("Point (0.25, 0.35, 0.45) -> voxel coords (%d, %d, %d)\n", coord_x,
         coord_y, coord_z);

  // Test voxel existence check (may not be implemented)
  bool exists;
  error = small_gicp_incremental_voxelmap_has_voxel(voxelmap, coord_x, coord_y,
                                                    coord_z, &exists);
  if (error == SMALL_GICP_NOT_IMPLEMENTED) {
    printf("Voxel existence check not implemented (as expected)\n");
  } else if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to check voxel existence\n");
    return 1;
  } else {
    printf("Voxel (%d, %d, %d) exists: %s\n", coord_x, coord_y, coord_z,
           exists ? "true" : "false");
  }

  // Test voxel info extraction
  if (num_voxels > 0) {
    printf("Testing voxel info extraction...\n");
    small_gicp_voxel_info_t info;
    error = small_gicp_incremental_voxelmap_get_voxel_info(voxelmap, 0, &info);
    if (error != SMALL_GICP_SUCCESS) {
      printf("Failed to get voxel info\n");
      return 1;
    }
    printf("Voxel 0: center=(%.3f, %.3f, %.3f), coords=(%d, %d, %d), "
           "points=%zu, lru=%zu\n",
           info.x, info.y, info.z, info.coord_x, info.coord_y, info.coord_z,
           info.num_points, info.lru_counter);
  }

  // Cleanup
  small_gicp_incremental_voxelmap_destroy(voxelmap);
  small_gicp_point_cloud_destroy(cloud);

  printf("Flat container points test passed!\n");
  return 0;
}

int test_gaussian_voxels() {
  printf("Testing Gaussian voxels...\n");

  small_gicp_incremental_voxelmap_t *voxelmap = NULL;
  small_gicp_point_cloud_t *cloud = NULL;
  small_gicp_error_t error;

  // Create test cloud
  error = create_test_point_cloud(200, &cloud);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create test cloud\n");
    return 1;
  }

  // Create Gaussian voxelmap with custom configuration
  small_gicp_flat_container_config_t config;
  small_gicp_create_default_flat_container_config(&config);
  config.lru_horizon = 50.0;
  config.lru_clear_cycle = 5;

  error = small_gicp_incremental_voxelmap_create_with_config(
      0.2, SMALL_GICP_VOXEL_GAUSSIAN, &config, &voxelmap);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create Gaussian voxelmap\n");
    return 1;
  }

  // Insert point cloud
  printf("Inserting point cloud into Gaussian voxelmap...\n");
  error = small_gicp_incremental_voxelmap_insert(voxelmap, cloud);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to insert point cloud\n");
    return 1;
  }

  // Finalize Gaussian statistics
  printf("Finalizing Gaussian statistics...\n");
  error = small_gicp_incremental_voxelmap_finalize(voxelmap);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to finalize voxelmap\n");
    return 1;
  }

  // Check voxelmap size
  size_t size, num_voxels;
  small_gicp_incremental_voxelmap_size(voxelmap, &size);
  small_gicp_incremental_voxelmap_num_voxels(voxelmap, &num_voxels);
  printf("Gaussian voxelmap contains %zu points in %zu voxels\n", size,
         num_voxels);

  // Extract Gaussian statistics from first voxel
  if (num_voxels > 0) {
    printf("Testing Gaussian voxel statistics extraction...\n");
    double mean[4];
    double covariance[16];
    size_t num_points;

    error = small_gicp_incremental_voxelmap_get_gaussian_voxel(
        voxelmap, 0, mean, covariance, &num_points);
    if (error != SMALL_GICP_SUCCESS) {
      printf("Failed to get Gaussian voxel statistics\n");
      return 1;
    }

    printf("Gaussian voxel 0:\n");
    printf("  Mean: (%.6f, %.6f, %.6f)\n", mean[0], mean[1], mean[2]);
    printf("  Points: %zu\n", num_points);
    printf("  Covariance diagonal: [%.6f, %.6f, %.6f]\n", covariance[0],
           covariance[5], covariance[10]);
  }

  // Test search with different search offset patterns
  printf("Testing search with different offset patterns...\n");
  for (int pattern = 1; pattern <= 27; pattern *= 7) {
    error = small_gicp_incremental_voxelmap_set_search_offsets(
        voxelmap, (small_gicp_search_offset_pattern_t)pattern);
    if (error != SMALL_GICP_SUCCESS) {
      printf("Failed to set search offsets\n");
      return 1;
    }

    size_t index;
    double distance;
    error = small_gicp_incremental_voxelmap_nearest_neighbor_search(
        voxelmap, 0.0, 0.0, 0.0, &index, &distance);
    if (error != SMALL_GICP_SUCCESS) {
      printf("NN search failed with offset pattern %d\n", pattern);
      return 1;
    }
    printf("  Pattern %d: nearest at index %zu, distance %.6f\n", pattern,
           index, distance);
  }

  // Cleanup
  small_gicp_incremental_voxelmap_destroy(voxelmap);
  small_gicp_point_cloud_destroy(cloud);

  printf("Gaussian voxels test passed!\n");
  return 0;
}

int test_incremental_insertion() {
  printf("Testing incremental point insertion...\n");

  small_gicp_incremental_voxelmap_t *voxelmap = NULL;
  small_gicp_error_t error;

  // Create voxelmap with normals support
  error = small_gicp_incremental_voxelmap_create(
      0.15, SMALL_GICP_VOXEL_FLAT_NORMAL, &voxelmap);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create incremental voxelmap\n");
    return 1;
  }

  // Test individual point insertion
  printf("Inserting individual points...\n");
  for (int i = 0; i < 10; i++) {
    double x = i * 0.1;
    double y = i * 0.05;
    double z = i * 0.02;

    if (i % 2 == 0) {
      // Insert point with normal
      error = small_gicp_incremental_voxelmap_insert_point_with_normal(
          voxelmap, x, y, z, 0.0, 0.0, 1.0);
    } else {
      // Insert just point
      error = small_gicp_incremental_voxelmap_insert_point(voxelmap, x, y, z);
    }

    if (error != SMALL_GICP_SUCCESS) {
      printf("Failed to insert point %d\n", i);
      return 1;
    }
  }

  // Check size after individual insertions
  size_t size, num_voxels;
  small_gicp_incremental_voxelmap_size(voxelmap, &size);
  small_gicp_incremental_voxelmap_num_voxels(voxelmap, &num_voxels);
  printf("After individual insertion: %zu points in %zu voxels\n", size,
         num_voxels);

  // Test transformation matrix insertion
  printf("Testing insertion with transformation...\n");
  small_gicp_point_cloud_t *cloud = NULL;
  error = create_test_point_cloud(50, &cloud);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create test cloud for transformation\n");
    return 1;
  }

  // Create a transformation matrix (translation + rotation)
  double transform[16] = {
      0.866, -0.5,  0.0, 1.0, // Rotation around Z + translation X
      0.5,   0.866, 0.0, 2.0, //
      0.0,   0.0,   1.0, 3.0, // Translation Z
      0.0,   0.0,   0.0, 1.0  // Homogeneous
  };

  error = small_gicp_incremental_voxelmap_insert_with_transform(voxelmap, cloud,
                                                                transform);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to insert with transformation\n");
    return 1;
  }

  // Check size after transformation insertion
  small_gicp_incremental_voxelmap_size(voxelmap, &size);
  small_gicp_incremental_voxelmap_num_voxels(voxelmap, &num_voxels);
  printf("After transformation insertion: %zu points in %zu voxels\n", size,
         num_voxels);

  // Test manual LRU cleanup (may not be implemented)
  printf("Testing manual LRU cleanup...\n");
  error = small_gicp_incremental_voxelmap_lru_cleanup(voxelmap);
  if (error == SMALL_GICP_NOT_IMPLEMENTED) {
    printf("LRU cleanup not implemented (LRU is automatic)\n");
  } else if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to perform LRU cleanup\n");
    return 1;
  } else {
    small_gicp_incremental_voxelmap_size(voxelmap, &size);
    small_gicp_incremental_voxelmap_num_voxels(voxelmap, &num_voxels);
    printf("After LRU cleanup: %zu points in %zu voxels\n", size, num_voxels);
  }

  // Test clearing the voxelmap (may not be implemented)
  printf("Testing voxelmap clear...\n");
  error = small_gicp_incremental_voxelmap_clear(voxelmap);
  if (error == SMALL_GICP_NOT_IMPLEMENTED) {
    printf("Clear not implemented (create new voxelmap instead)\n");
  } else if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to clear voxelmap\n");
    return 1;
  } else {
    small_gicp_incremental_voxelmap_size(voxelmap, &size);
    small_gicp_incremental_voxelmap_num_voxels(voxelmap, &num_voxels);
    printf("After clear: %zu points in %zu voxels\n", size, num_voxels);
  }

  // Cleanup
  small_gicp_incremental_voxelmap_destroy(voxelmap);
  small_gicp_point_cloud_destroy(cloud);

  printf("Incremental insertion test passed!\n");
  return 0;
}

int test_configuration_management() {
  printf("Testing configuration management...\n");

  small_gicp_incremental_voxelmap_t *voxelmap = NULL;
  small_gicp_error_t error;

  // Create default configuration
  small_gicp_flat_container_config_t config;
  error = small_gicp_create_default_flat_container_config(&config);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create default config\n");
    return 1;
  }

  printf("Default config values:\n");
  printf("  min_sq_dist_in_cell: %.6f\n", config.min_sq_dist_in_cell);
  printf("  max_num_points_in_cell: %d\n", config.max_num_points_in_cell);
  printf("  lru_horizon: %.1f\n", config.lru_horizon);
  printf("  lru_clear_cycle: %d\n", config.lru_clear_cycle);

  // Modify configuration
  config.min_sq_dist_in_cell = 0.005;
  config.max_num_points_in_cell = 10;
  config.lru_horizon = 25.0;
  config.lru_clear_cycle = 3;

  // Create voxelmap with custom config
  error = small_gicp_incremental_voxelmap_create_with_config(
      0.1, SMALL_GICP_VOXEL_FLAT_POINTS, &config, &voxelmap);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create voxelmap with custom config\n");
    return 1;
  }

  // Get configuration back
  small_gicp_flat_container_config_t retrieved_config;
  error =
      small_gicp_incremental_voxelmap_get_config(voxelmap, &retrieved_config);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to get configuration\n");
    return 1;
  }

  printf("Retrieved config values:\n");
  printf("  min_sq_dist_in_cell: %.6f\n", retrieved_config.min_sq_dist_in_cell);
  printf("  max_num_points_in_cell: %d\n",
         retrieved_config.max_num_points_in_cell);
  printf("  lru_horizon: %.1f\n", retrieved_config.lru_horizon);
  printf("  lru_clear_cycle: %d\n", retrieved_config.lru_clear_cycle);

  // Update configuration (may not be implemented)
  retrieved_config.lru_horizon = 15.0;
  error = small_gicp_incremental_voxelmap_update_config(voxelmap,
                                                        &retrieved_config);
  if (error == SMALL_GICP_NOT_IMPLEMENTED) {
    printf(
        "Configuration update not implemented (config set at creation time)\n");
  } else if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to update configuration\n");
    return 1;
  } else {
    printf("Configuration updated successfully\n");
  }

  // Cleanup
  small_gicp_incremental_voxelmap_destroy(voxelmap);

  printf("Configuration management test passed!\n");
  return 0;
}

int main() {
  printf("Running tests for incremental voxelmap operations\n\n");

  if (test_flat_container_points() != 0) {
    printf("Flat container points test failed!\n");
    return 1;
  }

  printf("\n");

  if (test_gaussian_voxels() != 0) {
    printf("Gaussian voxels test failed!\n");
    return 1;
  }

  printf("\n");

  if (test_incremental_insertion() != 0) {
    printf("Incremental insertion test failed!\n");
    return 1;
  }

  printf("\n");

  if (test_configuration_management() != 0) {
    printf("Configuration management test failed!\n");
    return 1;
  }

  printf("\nAll incremental voxelmap tests passed successfully!\n");
  return 0;
}
