#include <small_gicp_c.h>
#include <stdio.h>
#include <stdlib.h>

void print_matrix(const char *name, const double *matrix) {
  printf("%s:\n", name);
  for (int i = 0; i < 4; i++) {
    printf("  [");
    for (int j = 0; j < 4; j++) {
      printf("%8.4f", matrix[i * 4 + j]);
      if (j < 3)
        printf(", ");
    }
    printf("]\n");
  }
}

int main(int argc, char **argv) {
  if (argc != 3) {
    fprintf(stderr, "Usage: %s <target.ply> <source.ply>\n", argv[0]);
    return 1;
  }

  small_gicp_error_t error;

  // Load point clouds
  printf("Loading point clouds...\n");
  small_gicp_point_cloud_t *target = NULL;
  small_gicp_point_cloud_t *source = NULL;

  error = small_gicp_load_ply(argv[1], &target);
  if (error != SMALL_GICP_SUCCESS) {
    fprintf(stderr, "Failed to load target: %s\n",
            small_gicp_error_string(error));
    return 1;
  }

  error = small_gicp_load_ply(argv[2], &source);
  if (error != SMALL_GICP_SUCCESS) {
    fprintf(stderr, "Failed to load source: %s\n",
            small_gicp_error_string(error));
    small_gicp_point_cloud_destroy(target);
    return 1;
  }

  size_t target_size, source_size;
  small_gicp_point_cloud_size(target, &target_size);
  small_gicp_point_cloud_size(source, &source_size);
  printf("Target points: %zu, Source points: %zu\n", target_size, source_size);

  // Preprocess point clouds
  printf("\nPreprocessing point clouds...\n");
  small_gicp_point_cloud_t *target_preprocessed = NULL;
  small_gicp_point_cloud_t *source_preprocessed = NULL;
  small_gicp_kdtree_t *target_tree = NULL;
  small_gicp_kdtree_t *source_tree = NULL;

  error = small_gicp_preprocess_points(target, 0.25, 10, 4,
                                       &target_preprocessed, &target_tree);
  if (error != SMALL_GICP_SUCCESS) {
    fprintf(stderr, "Failed to preprocess target: %s\n",
            small_gicp_error_string(error));
    goto cleanup;
  }

  error = small_gicp_preprocess_points(source, 0.25, 10, 4,
                                       &source_preprocessed, &source_tree);
  if (error != SMALL_GICP_SUCCESS) {
    fprintf(stderr, "Failed to preprocess source: %s\n",
            small_gicp_error_string(error));
    goto cleanup;
  }

  small_gicp_point_cloud_size(target_preprocessed, &target_size);
  small_gicp_point_cloud_size(source_preprocessed, &source_size);
  printf("After preprocessing - Target: %zu, Source: %zu\n", target_size,
         source_size);

  // Test different registration algorithms
  printf("\nRunning registration algorithms...\n");

  const char *algorithm_names[] = {"ICP", "Plane ICP", "GICP"};
  small_gicp_registration_type_t algorithms[] = {
      SMALL_GICP_ICP, SMALL_GICP_PLANE_ICP, SMALL_GICP_GICP};

  for (int i = 0; i < 3; i++) {
    printf("\n=== %s ===\n", algorithm_names[i]);

    small_gicp_registration_result_t result;
    error = small_gicp_align_preprocessed(
        target_preprocessed, source_preprocessed, target_tree, algorithms[i],
        NULL, // identity initial guess
        4,    // num_threads
        &result);

    if (error != SMALL_GICP_SUCCESS) {
      fprintf(stderr, "Registration failed: %s\n",
              small_gicp_error_string(error));
      continue;
    }

    printf("Converged: %s\n", result.converged ? "Yes" : "No");
    printf("Iterations: %d\n", result.iterations);
    printf("Inliers: %d\n", result.num_inliers);
    printf("Error: %f\n", result.error);
    print_matrix("Transformation", result.T_target_source);
  }

  // Test VGICP
  printf("\n=== VGICP ===\n");
  small_gicp_gaussian_voxelmap_t *target_voxelmap = NULL;
  error = small_gicp_gaussian_voxelmap_create(target_preprocessed, 1.0, 4,
                                              &target_voxelmap);
  if (error == SMALL_GICP_SUCCESS) {
    small_gicp_registration_result_t result;
    error = small_gicp_align_vgicp(target_voxelmap, source_preprocessed, NULL,
                                   4, &result);

    if (error == SMALL_GICP_SUCCESS) {
      printf("Converged: %s\n", result.converged ? "Yes" : "No");
      printf("Iterations: %d\n", result.iterations);
      printf("Inliers: %d\n", result.num_inliers);
      printf("Error: %f\n", result.error);
      print_matrix("Transformation", result.T_target_source);
    } else {
      fprintf(stderr, "VGICP registration failed: %s\n",
              small_gicp_error_string(error));
    }

    small_gicp_gaussian_voxelmap_destroy(target_voxelmap);
  } else {
    fprintf(stderr, "Failed to create voxelmap: %s\n",
            small_gicp_error_string(error));
  }

cleanup:
  // Clean up
  small_gicp_point_cloud_destroy(target);
  small_gicp_point_cloud_destroy(source);
  small_gicp_point_cloud_destroy(target_preprocessed);
  small_gicp_point_cloud_destroy(source_preprocessed);
  small_gicp_kdtree_destroy(target_tree);
  small_gicp_kdtree_destroy(source_tree);

  return 0;
}
