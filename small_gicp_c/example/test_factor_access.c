#include <math.h>
#include <small_gicp_c.h>
#include <stdio.h>
#include <stdlib.h>

int main() {
  printf("=== Small GICP C Factor Utility Test ===\n\n");

  // Test points
  double source_point[3] = {1.0, 2.0, 3.0};
  double target_point[3] = {1.1, 2.1, 3.1};
  double target_normal[3] = {0.0, 0.0, 1.0};

  // Test covariance matrices (3x3 identity matrices)
  double source_cov[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  double target_cov[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

  double error;
  double weight;
  small_gicp_error_t result;

  // Test 1: ICP Error Computation
  printf("1. Testing ICP Error Computation\n");
  printf("   Source point: [%.3f, %.3f, %.3f]\n", source_point[0],
         source_point[1], source_point[2]);
  printf("   Target point: [%.3f, %.3f, %.3f]\n", target_point[0],
         target_point[1], target_point[2]);

  result = small_gicp_compute_icp_error(source_point, target_point, &error);
  if (result == SMALL_GICP_SUCCESS) {
    printf("   Error: %.6f\n", error);
  } else {
    printf("   ERROR: Failed to compute ICP error\n");
    return 1;
  }
  printf("   ✓ ICP error computation completed\n\n");

  // Test 2: Point-to-Plane Error Computation
  printf("2. Testing Point-to-Plane Error Computation\n");
  printf("   Source point: [%.3f, %.3f, %.3f]\n", source_point[0],
         source_point[1], source_point[2]);
  printf("   Target point: [%.3f, %.3f, %.3f]\n", target_point[0],
         target_point[1], target_point[2]);
  printf("   Target normal: [%.3f, %.3f, %.3f]\n", target_normal[0],
         target_normal[1], target_normal[2]);

  result = small_gicp_compute_point_to_plane_error(source_point, target_point,
                                                   target_normal, &error);
  if (result == SMALL_GICP_SUCCESS) {
    printf("   Error: %.6f\n", error);
  } else {
    printf("   ERROR: Failed to compute Point-to-Plane error\n");
    return 1;
  }
  printf("   ✓ Point-to-Plane error computation completed\n\n");

  // Test 3: GICP Error Computation
  printf("3. Testing GICP Error Computation\n");
  printf("   Source point: [%.3f, %.3f, %.3f]\n", source_point[0],
         source_point[1], source_point[2]);
  printf("   Target point: [%.3f, %.3f, %.3f]\n", target_point[0],
         target_point[1], target_point[2]);
  printf("   Using 3x3 identity covariance matrices\n");

  result = small_gicp_compute_gicp_error(source_point, target_point, source_cov,
                                         target_cov, &error);
  if (result == SMALL_GICP_SUCCESS) {
    printf("   Error: %.6f\n", error);
  } else {
    printf("   ERROR: Failed to compute GICP error\n");
    return 1;
  }
  printf("   ✓ GICP error computation completed\n\n");

  // Test 4: Robust Weight Computation
  printf("4. Testing Robust Weight Computation\n");
  printf("   Testing Huber kernel with c=1.0 and error=0.5\n");

  small_gicp_robust_kernel_t *kernel = NULL;
  result = small_gicp_create_robust_kernel(SMALL_GICP_ROBUST_KERNEL_HUBER, 1.0,
                                           &kernel);
  if (result != SMALL_GICP_SUCCESS) {
    printf("   ERROR: Failed to create robust kernel\n");
    return 1;
  }

  result = small_gicp_compute_robust_weight(kernel, 0.5, &weight);
  if (result == SMALL_GICP_SUCCESS) {
    printf("   Weight: %.6f\n", weight);
  } else {
    printf("   ERROR: Failed to compute robust weight\n");
    small_gicp_destroy_robust_kernel(kernel);
    return 1;
  }

  small_gicp_destroy_robust_kernel(kernel);
  printf("   ✓ Robust weight computation completed\n\n");

  // Test 5: Robust ICP Error Computation
  printf("5. Testing Robust ICP Error (Huber kernel)\n");
  printf("   Source point: [%.3f, %.3f, %.3f]\n", source_point[0],
         source_point[1], source_point[2]);
  printf("   Target point: [%.3f, %.3f, %.3f]\n", target_point[0],
         target_point[1], target_point[2]);
  printf("   Huber kernel with c=1.0\n");

  small_gicp_robust_kernel_t *huber_kernel = NULL;
  result = small_gicp_create_robust_kernel(SMALL_GICP_ROBUST_KERNEL_HUBER, 1.0,
                                           &huber_kernel);
  if (result != SMALL_GICP_SUCCESS) {
    printf("   ERROR: Failed to create Huber kernel\n");
    return 1;
  }

  result = small_gicp_compute_robust_icp_error(huber_kernel, source_point,
                                               target_point, &error, &weight);
  if (result == SMALL_GICP_SUCCESS) {
    printf("   Robust Error: %.6f\n", error);
    printf("   Weight: %.6f\n", weight);
  } else {
    printf("   ERROR: Failed to compute robust ICP error\n");
    small_gicp_destroy_robust_kernel(huber_kernel);
    return 1;
  }

  small_gicp_destroy_robust_kernel(huber_kernel);
  printf("   ✓ Robust ICP error computation completed\n\n");

  // Test 6: Robust Point-to-Plane Error Computation
  printf("6. Testing Robust Point-to-Plane Error (Cauchy kernel)\n");
  printf("   Source point: [%.3f, %.3f, %.3f]\n", source_point[0],
         source_point[1], source_point[2]);
  printf("   Target point: [%.3f, %.3f, %.3f]\n", target_point[0],
         target_point[1], target_point[2]);
  printf("   Target normal: [%.3f, %.3f, %.3f]\n", target_normal[0],
         target_normal[1], target_normal[2]);
  printf("   Cauchy kernel with c=1.0\n");

  small_gicp_robust_kernel_t *cauchy_kernel = NULL;
  result = small_gicp_create_robust_kernel(SMALL_GICP_ROBUST_KERNEL_CAUCHY, 1.0,
                                           &cauchy_kernel);
  if (result != SMALL_GICP_SUCCESS) {
    printf("   ERROR: Failed to create Cauchy kernel\n");
    return 1;
  }

  result = small_gicp_compute_robust_point_to_plane_error(
      cauchy_kernel, source_point, target_point, target_normal, &error,
      &weight);
  if (result == SMALL_GICP_SUCCESS) {
    printf("   Robust Error: %.6f\n", error);
    printf("   Weight: %.6f\n", weight);
  } else {
    printf("   ERROR: Failed to compute robust Point-to-Plane error\n");
    small_gicp_destroy_robust_kernel(cauchy_kernel);
    return 1;
  }

  small_gicp_destroy_robust_kernel(cauchy_kernel);
  printf("   ✓ Robust Point-to-Plane error computation completed\n\n");

  // Test 7: Robust GICP Error Computation
  printf("7. Testing Robust GICP Error (Huber kernel)\n");
  printf("   Source point: [%.3f, %.3f, %.3f]\n", source_point[0],
         source_point[1], source_point[2]);
  printf("   Target point: [%.3f, %.3f, %.3f]\n", target_point[0],
         target_point[1], target_point[2]);
  printf("   Using 3x3 identity covariance matrices\n");
  printf("   Huber kernel with c=1.0\n");

  small_gicp_robust_kernel_t *huber_kernel2 = NULL;
  result = small_gicp_create_robust_kernel(SMALL_GICP_ROBUST_KERNEL_HUBER, 1.0,
                                           &huber_kernel2);
  if (result != SMALL_GICP_SUCCESS) {
    printf("   ERROR: Failed to create Huber kernel\n");
    return 1;
  }

  result = small_gicp_compute_robust_gicp_error(huber_kernel2, source_point,
                                                target_point, source_cov,
                                                target_cov, &error, &weight);
  if (result == SMALL_GICP_SUCCESS) {
    printf("   Robust Error: %.6f\n", error);
    printf("   Weight: %.6f\n", weight);
  } else {
    printf("   ERROR: Failed to compute robust GICP error\n");
    small_gicp_destroy_robust_kernel(huber_kernel2);
    return 1;
  }

  small_gicp_destroy_robust_kernel(huber_kernel2);
  printf("   ✓ Robust GICP error computation completed\n\n");

  printf("=== All factor access tests completed successfully! ===\n");
  return 0;
}
