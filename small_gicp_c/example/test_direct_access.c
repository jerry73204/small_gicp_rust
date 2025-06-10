#include <math.h>
#include <small_gicp_c.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int test_direct_points_access() {
  printf("Testing direct points access...\n");

  small_gicp_point_cloud_t *cloud = NULL;
  small_gicp_error_t error;

  // Create point cloud
  error = small_gicp_point_cloud_create(&cloud);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create point cloud\n");
    return 1;
  }

  // Test bulk point setting
  double test_points[] = {
      1.0, 2.0, 3.0, // Point 0
      4.0, 5.0, 6.0, // Point 1
      7.0, 8.0, 9.0  // Point 2
  };

  error = small_gicp_point_cloud_set_points_bulk(cloud, test_points, 3);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to set points bulk\n");
    return 1;
  }

  // Test direct access to points data
  double *points_data = NULL;
  size_t data_size = 0;
  error =
      small_gicp_point_cloud_get_points_data(cloud, &points_data, &data_size);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to get points data\n");
    return 1;
  }

  printf("Points data size: %zu (expected: 12)\n", data_size);
  if (data_size != 12) { // 3 points * 4 components
    printf("Unexpected data size\n");
    return 1;
  }

  // Verify data access (x, y, z, w format)
  for (int i = 0; i < 3; i++) {
    double x = points_data[i * 4 + 0];
    double y = points_data[i * 4 + 1];
    double z = points_data[i * 4 + 2];
    double w = points_data[i * 4 + 3];

    printf("Point %d: (%.1f, %.1f, %.1f, %.1f)\n", i, x, y, z, w);

    // Verify values
    if (fabs(x - test_points[i * 3 + 0]) > 1e-9 ||
        fabs(y - test_points[i * 3 + 1]) > 1e-9 ||
        fabs(z - test_points[i * 3 + 2]) > 1e-9 || fabs(w - 1.0) > 1e-9) {
      printf("Point %d data mismatch\n", i);
      return 1;
    }
  }

  // Test copy to array function
  double copy_array[9]; // 3 points * 3 components
  error = small_gicp_point_cloud_copy_points_to_array(cloud, copy_array, 9);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to copy points to array\n");
    return 1;
  }

  // Verify copied data
  for (int i = 0; i < 9; i++) {
    if (fabs(copy_array[i] - test_points[i]) > 1e-9) {
      printf("Copied data mismatch at index %d\n", i);
      return 1;
    }
  }

  small_gicp_point_cloud_destroy(cloud);
  printf("Direct points access test passed!\n");
  return 0;
}

int test_direct_normals_access() {
  printf("Testing direct normals access...\n");

  small_gicp_point_cloud_t *cloud = NULL;
  small_gicp_error_t error;

  // Create point cloud
  error = small_gicp_point_cloud_create(&cloud);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create point cloud\n");
    return 1;
  }

  // Set some points first
  error = small_gicp_point_cloud_resize(cloud, 2);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to resize point cloud\n");
    return 1;
  }

  // Test bulk normal setting
  double test_normals[] = {
      1.0, 0.0, 0.0, // Normal 0
      0.0, 1.0, 0.0  // Normal 1
  };

  error = small_gicp_point_cloud_set_normals_bulk(cloud, test_normals, 2);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to set normals bulk\n");
    return 1;
  }

  // Test direct access to normals data
  double *normals_data = NULL;
  size_t data_size = 0;
  error =
      small_gicp_point_cloud_get_normals_data(cloud, &normals_data, &data_size);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to get normals data\n");
    return 1;
  }

  printf("Normals data size: %zu (expected: 8)\n", data_size);
  if (data_size != 8) { // 2 normals * 4 components
    printf("Unexpected normals data size\n");
    return 1;
  }

  // Verify normals data (nx, ny, nz, 0 format)
  for (int i = 0; i < 2; i++) {
    double nx = normals_data[i * 4 + 0];
    double ny = normals_data[i * 4 + 1];
    double nz = normals_data[i * 4 + 2];
    double nw = normals_data[i * 4 + 3];

    printf("Normal %d: (%.1f, %.1f, %.1f, %.1f)\n", i, nx, ny, nz, nw);

    if (fabs(nx - test_normals[i * 3 + 0]) > 1e-9 ||
        fabs(ny - test_normals[i * 3 + 1]) > 1e-9 ||
        fabs(nz - test_normals[i * 3 + 2]) > 1e-9 || fabs(nw - 0.0) > 1e-9) {
      printf("Normal %d data mismatch\n", i);
      return 1;
    }
  }

  small_gicp_point_cloud_destroy(cloud);
  printf("Direct normals access test passed!\n");
  return 0;
}

int test_direct_covariances_access() {
  printf("Testing direct covariances access...\n");

  small_gicp_point_cloud_t *cloud = NULL;
  small_gicp_error_t error;

  // Create point cloud
  error = small_gicp_point_cloud_create(&cloud);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create point cloud\n");
    return 1;
  }

  // Set one point
  error = small_gicp_point_cloud_resize(cloud, 1);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to resize point cloud\n");
    return 1;
  }

  // Test bulk covariance setting (1 covariance = 16 values)
  double test_covariances[16] = {1.0, 0.1, 0.2, 0.0, 0.1, 2.0, 0.3, 0.0,
                                 0.2, 0.3, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0};

  error =
      small_gicp_point_cloud_set_covariances_bulk(cloud, test_covariances, 1);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to set covariances bulk\n");
    return 1;
  }

  // Test direct access to covariances data
  double *covariances_data = NULL;
  size_t data_size = 0;
  error = small_gicp_point_cloud_get_covariances_data(cloud, &covariances_data,
                                                      &data_size);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to get covariances data\n");
    return 1;
  }

  printf("Covariances data size: %zu (expected: 16)\n", data_size);
  if (data_size != 16) { // 1 covariance * 16 components
    printf("Unexpected covariances data size\n");
    return 1;
  }

  // Verify covariances data
  for (int i = 0; i < 16; i++) {
    if (fabs(covariances_data[i] - test_covariances[i]) > 1e-9) {
      printf("Covariance data mismatch at index %d: expected %.1f, got %.1f\n",
             i, test_covariances[i], covariances_data[i]);
      return 1;
    }
  }

  // Test copy to array function
  double copy_array[16];
  error =
      small_gicp_point_cloud_copy_covariances_to_array(cloud, copy_array, 16);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to copy covariances to array\n");
    return 1;
  }

  // Verify copied data
  for (int i = 0; i < 16; i++) {
    if (fabs(copy_array[i] - test_covariances[i]) > 1e-9) {
      printf("Copied covariance data mismatch at index %d\n", i);
      return 1;
    }
  }

  small_gicp_point_cloud_destroy(cloud);
  printf("Direct covariances access test passed!\n");
  return 0;
}

int test_empty_vectors() {
  printf("Testing empty vectors access...\n");

  small_gicp_point_cloud_t *cloud = NULL;
  small_gicp_error_t error;

  // Create empty point cloud
  error = small_gicp_point_cloud_create(&cloud);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create point cloud\n");
    return 1;
  }

  // Test accessing empty vectors
  double *data = NULL;
  size_t data_size = 0;

  // Points
  error = small_gicp_point_cloud_get_points_data(cloud, &data, &data_size);
  if (error != SMALL_GICP_SUCCESS || data != NULL || data_size != 0) {
    printf("Empty points vector access failed\n");
    return 1;
  }

  // Normals
  error = small_gicp_point_cloud_get_normals_data(cloud, &data, &data_size);
  if (error != SMALL_GICP_SUCCESS || data != NULL || data_size != 0) {
    printf("Empty normals vector access failed\n");
    return 1;
  }

  // Covariances
  error = small_gicp_point_cloud_get_covariances_data(cloud, &data, &data_size);
  if (error != SMALL_GICP_SUCCESS || data != NULL || data_size != 0) {
    printf("Empty covariances vector access failed\n");
    return 1;
  }

  small_gicp_point_cloud_destroy(cloud);
  printf("Empty vectors access test passed!\n");
  return 0;
}

int main() {
  printf("Running tests for direct access to internal vectors\n\n");

  if (test_direct_points_access() != 0) {
    printf("Direct points access test failed!\n");
    return 1;
  }

  printf("\n");

  if (test_direct_normals_access() != 0) {
    printf("Direct normals access test failed!\n");
    return 1;
  }

  printf("\n");

  if (test_direct_covariances_access() != 0) {
    printf("Direct covariances access test failed!\n");
    return 1;
  }

  printf("\n");

  if (test_empty_vectors() != 0) {
    printf("Empty vectors access test failed!\n");
    return 1;
  }

  printf("\nAll direct access tests passed successfully!\n");
  return 0;
}
