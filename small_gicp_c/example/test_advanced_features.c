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

int test_registration_helper() {
  printf("Testing RegistrationHelper functions...\n");

  small_gicp_point_cloud_t *target_cloud = NULL;
  small_gicp_point_cloud_t *source_cloud = NULL;
  small_gicp_point_cloud_t *preprocessed_target = NULL;
  small_gicp_point_cloud_t *preprocessed_source = NULL;
  small_gicp_kdtree_t *target_tree = NULL;
  small_gicp_kdtree_t *source_tree = NULL;
  small_gicp_gaussian_voxelmap_t *voxelmap = NULL;
  small_gicp_registration_helper_setting_t *setting = NULL;
  small_gicp_error_t error;

  // Create test clouds
  error = create_test_point_cloud(1000, &target_cloud);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create target cloud\n");
    return 1;
  }

  error = create_test_point_cloud(800, &source_cloud);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create source cloud\n");
    return 1;
  }

  // Test preprocessing
  printf("Testing preprocessing...\n");
  error = small_gicp_preprocess_points_helper(
      target_cloud, 0.1, 10, 4, &preprocessed_target, &target_tree);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to preprocess target cloud\n");
    return 1;
  }

  error = small_gicp_preprocess_points_helper(
      source_cloud, 0.1, 10, 4, &preprocessed_source, &source_tree);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to preprocess source cloud\n");
    return 1;
  }

  // Check preprocessed cloud sizes
  size_t target_size, source_size;
  small_gicp_point_cloud_size(preprocessed_target, &target_size);
  small_gicp_point_cloud_size(preprocessed_source, &source_size);
  printf("Preprocessed target size: %zu, source size: %zu\n", target_size,
         source_size);

  // Test Gaussian voxelmap creation
  printf("Testing Gaussian voxelmap creation...\n");
  error = small_gicp_create_gaussian_voxelmap_helper(preprocessed_target, 0.2,
                                                     &voxelmap);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create Gaussian voxelmap\n");
    return 1;
  }

  // Test default setting creation
  printf("Testing registration helper setting...\n");
  error = small_gicp_create_default_registration_helper_setting(&setting);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create default setting\n");
    return 1;
  }

  // Modify setting
  setting->type = SMALL_GICP_GICP;
  setting->max_iterations = 10;
  setting->verbose = false;

  // Test alignment with point clouds
  printf("Testing registration helper alignment...\n");
  small_gicp_registration_result_t result;
  error = small_gicp_align_helper(preprocessed_target, preprocessed_source,
                                  target_tree, NULL, setting, &result);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to align with helper\n");
    return 1;
  }

  printf("Registration result: converged=%s, iterations=%d, error=%.6f\n",
         result.converged ? "true" : "false", result.iterations, result.error);

  // Test VGICP alignment
  printf("Testing VGICP helper alignment...\n");
  small_gicp_registration_result_t vgicp_result;
  setting->type = SMALL_GICP_VGICP;
  error = small_gicp_align_helper_vgicp(voxelmap, preprocessed_source, NULL,
                                        setting, &vgicp_result);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed VGICP alignment with helper\n");
    return 1;
  }

  printf("VGICP result: converged=%s, iterations=%d, error=%.6f\n",
         vgicp_result.converged ? "true" : "false", vgicp_result.iterations,
         vgicp_result.error);

  // Cleanup
  small_gicp_point_cloud_destroy(target_cloud);
  small_gicp_point_cloud_destroy(source_cloud);
  small_gicp_point_cloud_destroy(preprocessed_target);
  small_gicp_point_cloud_destroy(preprocessed_source);
  small_gicp_kdtree_destroy(target_tree);
  small_gicp_kdtree_destroy(source_tree);
  small_gicp_gaussian_voxelmap_destroy(voxelmap);
  small_gicp_destroy_registration_helper_setting(setting);

  printf("RegistrationHelper test passed!\n");
  return 0;
}

int test_local_features() {
  printf("Testing local features (NormalSetter/CovarianceSetter)...\n");

  small_gicp_point_cloud_t *cloud = NULL;
  small_gicp_kdtree_t *kdtree = NULL;
  small_gicp_error_t error;

  // Create test cloud
  error = create_test_point_cloud(500, &cloud);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create test cloud\n");
    return 1;
  }

  // Create KdTree
  error = small_gicp_kdtree_create(cloud, 4, &kdtree);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to create KdTree\n");
    return 1;
  }

  // Test automatic local features estimation
  printf("Testing automatic normal estimation...\n");
  error = small_gicp_estimate_local_features_auto(
      cloud, 10, SMALL_GICP_SETTER_NORMAL,
      SMALL_GICP_LOCAL_FEATURES_BACKEND_DEFAULT, 1);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed automatic normal estimation\n");
    return 1;
  }

  // Check that normals were computed
  double nx, ny, nz;
  error = small_gicp_point_cloud_get_normal(cloud, 0, &nx, &ny, &nz);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to get computed normal\n");
    return 1;
  }

  double normal_magnitude = sqrt(nx * nx + ny * ny + nz * nz);
  printf("First normal magnitude: %.3f\n", normal_magnitude);

  // Test covariance estimation with OpenMP backend
  printf("Testing covariance estimation with OpenMP...\n");
  error = small_gicp_estimate_local_features_cloud(
      cloud, kdtree, 15, SMALL_GICP_SETTER_COVARIANCE,
      SMALL_GICP_LOCAL_FEATURES_BACKEND_OPENMP, 2);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed covariance estimation with OpenMP\n");
    return 1;
  }

  // Check covariance
  double cov_matrix[16];
  error = small_gicp_point_cloud_get_covariance(cloud, 0, cov_matrix);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to get computed covariance\n");
    return 1;
  }

  printf("First covariance diagonal: [%.6f, %.6f, %.6f]\n", cov_matrix[0],
         cov_matrix[5], cov_matrix[10]);

  // Test direct setter interface
  printf("Testing direct setter interface...\n");
  double eigenvectors[9] = {
      1.0, 0.0, 0.0, // First eigenvector (normal direction)
      0.0, 1.0, 0.0, // Second eigenvector
      0.0, 0.0, 1.0  // Third eigenvector
  };

  error = small_gicp_normal_setter_set(cloud, 100, eigenvectors);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed direct normal setter\n");
    return 1;
  }

  error = small_gicp_covariance_setter_set(cloud, 100, eigenvectors);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed direct covariance setter\n");
    return 1;
  }

  // Test single point estimation
  printf("Testing single point estimation...\n");
  error = small_gicp_estimate_local_features_single_point(
      cloud, kdtree, 200, 20, SMALL_GICP_SETTER_NORMAL_COVARIANCE);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed single point estimation\n");
    return 1;
  }

  // Test invalid setters
  printf("Testing invalid setters...\n");
  error = small_gicp_normal_setter_set_invalid(cloud, 300);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed invalid normal setter\n");
    return 1;
  }

  error = small_gicp_covariance_setter_set_invalid(cloud, 300);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed invalid covariance setter\n");
    return 1;
  }

  // Cleanup
  small_gicp_kdtree_destroy(kdtree);
  small_gicp_point_cloud_destroy(cloud);

  printf("Local features test passed!\n");
  return 0;
}

int test_sorting_utilities() {
  printf("Testing sorting utilities...\n");

  // Test sorting doubles
  printf("Testing double array sorting...\n");
  const size_t count = 1000;
  double *doubles = malloc(count * sizeof(double));
  if (!doubles) {
    printf("Failed to allocate memory for doubles\n");
    return 1;
  }

  // Fill with random values
  srand(42); // Fixed seed for reproducibility
  for (size_t i = 0; i < count; i++) {
    doubles[i] = (double)rand() / RAND_MAX;
  }

  small_gicp_error_t error =
      small_gicp_sort_doubles(doubles, count, SMALL_GICP_SORT_BACKEND_STD, 1);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to sort doubles\n");
    free(doubles);
    return 1;
  }

  // Verify sorted
  for (size_t i = 1; i < count; i++) {
    if (doubles[i] < doubles[i - 1]) {
      printf("Double array not properly sorted at index %zu\n", i);
      free(doubles);
      return 1;
    }
  }
  printf("Double array sorting successful\n");

  // Test OpenMP quick sort
  for (size_t i = 0; i < count; i++) {
    doubles[i] = (double)rand() / RAND_MAX;
  }

  error = small_gicp_sort_doubles(doubles, count,
                                  SMALL_GICP_SORT_BACKEND_OPENMP_QUICK, 2);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed OpenMP quick sort\n");
    free(doubles);
    return 1;
  }

  // Verify sorted
  for (size_t i = 1; i < count; i++) {
    if (doubles[i] < doubles[i - 1]) {
      printf("OpenMP quick sorted array not properly sorted at index %zu\n", i);
      free(doubles);
      return 1;
    }
  }
  printf("OpenMP quick sort successful\n");

  free(doubles);

  // Test sorting indices by values
  printf("Testing sorting indices by values...\n");
  const size_t values_count = 100;
  float *values = malloc(values_count * sizeof(float));
  size_t *indices = malloc(values_count * sizeof(size_t));

  if (!values || !indices) {
    printf("Failed to allocate memory for indices test\n");
    free(values);
    free(indices);
    return 1;
  }

  // Fill with random values
  for (size_t i = 0; i < values_count; i++) {
    values[i] = (float)rand() / RAND_MAX;
  }

  error = small_gicp_sort_indices_by_values_float(
      indices, values, values_count, SMALL_GICP_SORT_BACKEND_STD, 1);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to sort indices by values\n");
    free(values);
    free(indices);
    return 1;
  }

  // Verify indices are sorted by values
  for (size_t i = 1; i < values_count; i++) {
    if (values[indices[i]] < values[indices[i - 1]]) {
      printf("Indices not properly sorted by values at index %zu\n", i);
      free(values);
      free(indices);
      return 1;
    }
  }
  printf("Sorting indices by values successful\n");

  free(values);
  free(indices);

  // Test size_t sorting (which can use radix sort)
  printf("Testing size_t array sorting...\n");
  size_t *size_array = malloc(count * sizeof(size_t));
  if (!size_array) {
    printf("Failed to allocate memory for size_t array\n");
    return 1;
  }

  for (size_t i = 0; i < count; i++) {
    size_array[i] = rand() % 10000;
  }

  error =
      small_gicp_sort_size_t(size_array, count, SMALL_GICP_SORT_BACKEND_STD, 1);
  if (error != SMALL_GICP_SUCCESS) {
    printf("Failed to sort size_t array\n");
    free(size_array);
    return 1;
  }

  // Verify sorted
  for (size_t i = 1; i < count; i++) {
    if (size_array[i] < size_array[i - 1]) {
      printf("Size_t array not properly sorted at index %zu\n", i);
      free(size_array);
      return 1;
    }
  }
  printf("Size_t array sorting successful\n");

  free(size_array);

  printf("Sorting utilities test passed!\n");
  return 0;
}

int main() {
  printf("Running tests for advanced features\n\n");

  if (test_registration_helper() != 0) {
    printf("RegistrationHelper test failed!\n");
    return 1;
  }

  printf("\n");

  if (test_local_features() != 0) {
    printf("Local features test failed!\n");
    return 1;
  }

  printf("\n");

  if (test_sorting_utilities() != 0) {
    printf("Sorting utilities test failed!\n");
    return 1;
  }

  printf("\nAll advanced feature tests passed successfully!\n");
  return 0;
}
