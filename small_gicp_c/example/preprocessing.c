#include <small_gicp_c.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv) {
  if (argc != 2) {
    fprintf(stderr, "Usage: %s <input.ply>\n", argv[0]);
    return 1;
  }

  small_gicp_error_t error;

  // Load point cloud
  printf("Loading point cloud...\n");
  small_gicp_point_cloud_t *cloud = NULL;
  error = small_gicp_load_ply(argv[1], &cloud);
  if (error != SMALL_GICP_SUCCESS) {
    fprintf(stderr, "Failed to load point cloud: %s\n",
            small_gicp_error_string(error));
    return 1;
  }

  size_t original_size;
  small_gicp_point_cloud_size(cloud, &original_size);
  printf("Original point cloud size: %zu\n", original_size);

  // Test voxel grid downsampling
  printf("\nTesting voxel grid downsampling...\n");
  double leaf_sizes[] = {0.1, 0.5, 1.0};

  for (int i = 0; i < 3; i++) {
    small_gicp_point_cloud_t *downsampled = NULL;
    error =
        small_gicp_voxelgrid_sampling(cloud, leaf_sizes[i], 4, &downsampled);

    if (error == SMALL_GICP_SUCCESS) {
      size_t downsampled_size;
      small_gicp_point_cloud_size(downsampled, &downsampled_size);
      printf("Leaf size %.1f: %zu points (%.1f%% of original)\n", leaf_sizes[i],
             downsampled_size, 100.0 * downsampled_size / original_size);

      // Save result
      char filename[256];
      snprintf(filename, sizeof(filename), "downsampled_%.1f.ply",
               leaf_sizes[i]);
      small_gicp_save_ply(filename, downsampled);

      small_gicp_point_cloud_destroy(downsampled);
    } else {
      fprintf(stderr, "Downsampling failed: %s\n",
              small_gicp_error_string(error));
    }
  }

  // Test random sampling
  printf("\nTesting random sampling...\n");
  size_t sample_counts[] = {1000, 5000, 10000};

  for (int i = 0; i < 3; i++) {
    if (sample_counts[i] > original_size)
      continue;

    small_gicp_point_cloud_t *sampled = NULL;
    error = small_gicp_random_sampling(cloud, sample_counts[i], &sampled);

    if (error == SMALL_GICP_SUCCESS) {
      size_t sampled_size;
      small_gicp_point_cloud_size(sampled, &sampled_size);
      printf("Random sampling %zu points: actual %zu\n", sample_counts[i],
             sampled_size);

      small_gicp_point_cloud_destroy(sampled);
    } else {
      fprintf(stderr, "Random sampling failed: %s\n",
              small_gicp_error_string(error));
    }
  }

  // Test normal estimation
  printf("\nTesting normal estimation...\n");

  // First downsample for faster processing
  small_gicp_point_cloud_t *working_cloud = NULL;
  error = small_gicp_voxelgrid_sampling(cloud, 0.5, 4, &working_cloud);
  if (error != SMALL_GICP_SUCCESS) {
    fprintf(stderr, "Failed to downsample for normal estimation\n");
    small_gicp_point_cloud_destroy(cloud);
    return 1;
  }

  // Create KdTree
  small_gicp_kdtree_t *kdtree = NULL;
  error = small_gicp_kdtree_create(working_cloud, 4, &kdtree);
  if (error != SMALL_GICP_SUCCESS) {
    fprintf(stderr, "Failed to create KdTree: %s\n",
            small_gicp_error_string(error));
    small_gicp_point_cloud_destroy(cloud);
    small_gicp_point_cloud_destroy(working_cloud);
    return 1;
  }

  // Estimate normals
  printf("Estimating normals with 20 neighbors...\n");
  error = small_gicp_estimate_normals(working_cloud, kdtree, 20, 4);
  if (error == SMALL_GICP_SUCCESS) {
    printf("Normals estimated successfully\n");

    // Check a few normals
    size_t size;
    small_gicp_point_cloud_size(working_cloud, &size);
    printf("\nFirst 5 point normals:\n");
    for (size_t i = 0; i < 5 && i < size; i++) {
      double x, y, z, nx, ny, nz;
      small_gicp_point_cloud_get_point(working_cloud, i, &x, &y, &z);
      small_gicp_point_cloud_get_normal(working_cloud, i, &nx, &ny, &nz);
      printf("Point %zu: (%.3f, %.3f, %.3f) -> Normal: (%.3f, %.3f, %.3f)\n", i,
             x, y, z, nx, ny, nz);
    }

    // Save with normals
    small_gicp_save_ply("with_normals.ply", working_cloud);
  } else {
    fprintf(stderr, "Normal estimation failed: %s\n",
            small_gicp_error_string(error));
  }

  // Test combined preprocessing
  printf("\nTesting combined preprocessing...\n");
  small_gicp_point_cloud_t *preprocessed = NULL;
  small_gicp_kdtree_t *preprocessed_tree = NULL;

  error = small_gicp_preprocess_points(cloud, 0.25, 10, 4, &preprocessed,
                                       &preprocessed_tree);
  if (error == SMALL_GICP_SUCCESS) {
    size_t preprocessed_size;
    small_gicp_point_cloud_size(preprocessed, &preprocessed_size);
    printf("Preprocessed cloud: %zu points (%.1f%% of original)\n",
           preprocessed_size, 100.0 * preprocessed_size / original_size);
    printf("Includes downsampling, KdTree construction, and normal/covariance "
           "estimation\n");

    small_gicp_point_cloud_destroy(preprocessed);
    small_gicp_kdtree_destroy(preprocessed_tree);
  } else {
    fprintf(stderr, "Preprocessing failed: %s\n",
            small_gicp_error_string(error));
  }

  // Clean up
  small_gicp_point_cloud_destroy(cloud);
  small_gicp_point_cloud_destroy(working_cloud);
  small_gicp_kdtree_destroy(kdtree);

  return 0;
}
