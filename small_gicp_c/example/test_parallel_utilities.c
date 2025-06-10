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

  // Create a simple test pattern (sphere)
  for (size_t i = 0; i < num_points; i++) {
    double theta = 2.0 * M_PI * i / num_points;
    double phi = M_PI * (i % 100) / 100.0;
    double radius = 1.0 + 0.1 * sin(theta * 5);

    double x = radius * sin(phi) * cos(theta);
    double y = radius * sin(phi) * sin(theta);
    double z = radius * cos(phi);

    error = small_gicp_point_cloud_set_point(*cloud, i, x, y, z);
    if (error != SMALL_GICP_SUCCESS)
      return error;
  }

  return SMALL_GICP_SUCCESS;
}

// Test parallel downsampling
int test_parallel_downsampling() {
  printf("Testing parallel downsampling...\n");

  small_gicp_point_cloud_t *cloud = NULL;
  small_gicp_point_cloud_t *downsampled_default = NULL;
  small_gicp_point_cloud_t *downsampled_omp = NULL;
  small_gicp_point_cloud_t *downsampled_tbb = NULL;
  small_gicp_error_t error;

  // Create test point cloud
  error = create_test_point_cloud(10000, &cloud);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create test point cloud\n");
    return 1;
  }

  double leaf_size = 0.1;
  int num_threads = 4;

  printf("Original cloud size: 10000 points\n");

  // Test default backend
  printf("Testing default backend...\n");
  clock_t start = clock();
  error = small_gicp_voxelgrid_sampling_with_backend(
      cloud, leaf_size, SMALL_GICP_DOWNSAMPLING_BACKEND_DEFAULT, 1,
      &downsampled_default);
  clock_t end = clock();
  double time_default = ((double)(end - start)) / CLOCKS_PER_SEC;

  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed default downsampling\n");
    return 1;
  }

  size_t size_default;
  small_gicp_point_cloud_size(downsampled_default, &size_default);
  printf("Default backend: %zu points, %.3f seconds\n", size_default,
         time_default);

  // Test OpenMP backend
  printf("Testing OpenMP backend...\n");
  start = clock();
  error = small_gicp_voxelgrid_sampling_with_backend(
      cloud, leaf_size, SMALL_GICP_DOWNSAMPLING_BACKEND_OPENMP, num_threads,
      &downsampled_omp);
  end = clock();
  double time_omp = ((double)(end - start)) / CLOCKS_PER_SEC;

  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed OpenMP downsampling\n");
    return 1;
  }

  size_t size_omp;
  small_gicp_point_cloud_size(downsampled_omp, &size_omp);
  printf("OpenMP backend: %zu points, %.3f seconds\n", size_omp, time_omp);

  // Test TBB backend
  printf("Testing TBB backend...\n");
  start = clock();
  error = small_gicp_voxelgrid_sampling_with_backend(
      cloud, leaf_size, SMALL_GICP_DOWNSAMPLING_BACKEND_TBB, num_threads,
      &downsampled_tbb);
  end = clock();
  double time_tbb = ((double)(end - start)) / CLOCKS_PER_SEC;

  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed TBB downsampling\n");
    return 1;
  }

  size_t size_tbb;
  small_gicp_point_cloud_size(downsampled_tbb, &size_tbb);
  printf("TBB backend: %zu points, %.3f seconds\n", size_tbb, time_tbb);

  // Test random sampling with seed
  printf("Testing random sampling with seed...\n");
  small_gicp_point_cloud_t *random1 = NULL, *random2 = NULL;
  error = small_gicp_random_sampling_with_seed(cloud, 1000, 42, &random1);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed random sampling 1\n");
    return 1;
  }

  error = small_gicp_random_sampling_with_seed(cloud, 1000, 42, &random2);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed random sampling 2\n");
    return 1;
  }

  // Verify reproducibility
  double x1, y1, z1, x2, y2, z2;
  small_gicp_point_cloud_get_point(random1, 0, &x1, &y1, &z1);
  small_gicp_point_cloud_get_point(random2, 0, &x2, &y2, &z2);

  if (fabs(x1 - x2) > 1e-9 || fabs(y1 - y2) > 1e-9 || fabs(z1 - z2) > 1e-9) {
    printf("Random sampling with same seed should be reproducible\n");
    return 1;
  }
  printf("Random sampling reproducibility test passed!\n");

  // Cleanup
  small_gicp_point_cloud_destroy(cloud);
  small_gicp_point_cloud_destroy(downsampled_default);
  small_gicp_point_cloud_destroy(downsampled_omp);
  small_gicp_point_cloud_destroy(downsampled_tbb);
  small_gicp_point_cloud_destroy(random1);
  small_gicp_point_cloud_destroy(random2);

  printf("Parallel downsampling test passed!\n");
  return 0;
}

// Test parallel normal estimation
int test_parallel_normal_estimation() {
  printf("Testing parallel normal estimation...\n");

  small_gicp_point_cloud_t *cloud = NULL;
  small_gicp_kdtree_t *kdtree = NULL;
  small_gicp_error_t error;

  // Create test point cloud
  error = create_test_point_cloud(1000, &cloud);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create test point cloud\n");
    return 1;
  }

  // Create KdTree
  error = small_gicp_kdtree_create(cloud, 4, &kdtree);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create KdTree\n");
    return 1;
  }

  int num_neighbors = 10;
  int num_threads = 4;

  // Test default backend
  printf("Testing default normal estimation...\n");
  error = small_gicp_estimate_normals_with_backend(
      cloud, kdtree, num_neighbors,
      SMALL_GICP_NORMAL_ESTIMATION_BACKEND_DEFAULT, 1);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed default normal estimation\n");
    return 1;
  }

  // Check if normals are computed
  double nx, ny, nz;
  error = small_gicp_point_cloud_get_normal(cloud, 0, &nx, &ny, &nz);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to get normal\n");
    return 1;
  }

  double normal_magnitude = sqrt(nx * nx + ny * ny + nz * nz);
  if (normal_magnitude < 0.5) { // Should be roughly unit length
    printf("Normal magnitude too small: %f\n", normal_magnitude);
    return 1;
  }
  printf("Default normal estimation completed (magnitude: %.3f)\n",
         normal_magnitude);

  // Test OpenMP backend
  printf("Testing OpenMP normal estimation...\n");
  error = small_gicp_estimate_normals_with_backend(
      cloud, kdtree, num_neighbors, SMALL_GICP_NORMAL_ESTIMATION_BACKEND_OPENMP,
      num_threads);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed OpenMP normal estimation\n");
    return 1;
  }
  printf("OpenMP normal estimation completed\n");

  // Test TBB backend
  printf("Testing TBB normal estimation...\n");
  error = small_gicp_estimate_normals_with_backend(
      cloud, kdtree, num_neighbors, SMALL_GICP_NORMAL_ESTIMATION_BACKEND_TBB,
      num_threads);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed TBB normal estimation\n");
    return 1;
  }
  printf("TBB normal estimation completed\n");

  // Test combined normal and covariance estimation
  printf("Testing combined normal and covariance estimation...\n");
  error = small_gicp_estimate_normals_covariances_with_backend(
      cloud, kdtree, num_neighbors, SMALL_GICP_NORMAL_ESTIMATION_BACKEND_OPENMP,
      num_threads);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed combined estimation\n");
    return 1;
  }

  // Check if covariances are computed
  double cov_matrix[16];
  error = small_gicp_point_cloud_get_covariance(cloud, 0, cov_matrix);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to get covariance\n");
    return 1;
  }

  // Check that covariance diagonal elements are positive
  if (cov_matrix[0] <= 0 || cov_matrix[5] <= 0 || cov_matrix[10] <= 0) {
    printf("Invalid covariance matrix\n");
    return 1;
  }
  printf("Combined estimation completed (cov[0,0]: %.6f)\n", cov_matrix[0]);

  // Cleanup
  small_gicp_kdtree_destroy(kdtree);
  small_gicp_point_cloud_destroy(cloud);

  printf("Parallel normal estimation test passed!\n");
  return 0;
}

int main() {
  printf("Running tests for parallel utility functions\n\n");

  if (test_parallel_downsampling() != 0) {
    printf("Parallel downsampling test failed!\n");
    return 1;
  }

  printf("\n");

  if (test_parallel_normal_estimation() != 0) {
    printf("Parallel normal estimation test failed!\n");
    return 1;
  }

  printf("\nAll parallel utility tests passed successfully!\n");
  return 0;
}
