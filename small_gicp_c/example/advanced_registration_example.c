#include <math.h>
#include <small_gicp_c.h>
#include <stdio.h>
#include <stdlib.h>

// Helper function to create a simple point cloud
small_gicp_error_t create_test_cloud(int num_points,
                                     small_gicp_point_cloud_t **cloud) {
  small_gicp_error_t err = small_gicp_point_cloud_create(cloud);
  if (err != SMALL_GICP_SUCCESS)
    return err;

  err = small_gicp_point_cloud_resize(*cloud, num_points);
  if (err != SMALL_GICP_SUCCESS)
    return err;

  // Create a simple grid of points
  for (int i = 0; i < num_points; i++) {
    double x = (i % 10) * 0.1;
    double y = (i / 10) * 0.1;
    double z = 0.0;
    err = small_gicp_point_cloud_set_point(*cloud, i, x, y, z);
    if (err != SMALL_GICP_SUCCESS)
      return err;
  }

  return SMALL_GICP_SUCCESS;
}

int main() {
  printf("Small GICP Advanced Registration Example\\n");
  printf("========================================\\n\\n");

  // Create test point clouds
  small_gicp_point_cloud_t *target = NULL;
  small_gicp_point_cloud_t *source = NULL;

  if (create_test_cloud(100, &target) != SMALL_GICP_SUCCESS ||
      create_test_cloud(100, &source) != SMALL_GICP_SUCCESS) {
    printf("Failed to create test point clouds\\n");
    return 1;
  }

  // Preprocess point clouds
  small_gicp_point_cloud_t *target_preprocessed = NULL;
  small_gicp_point_cloud_t *source_preprocessed = NULL;
  small_gicp_kdtree_t *target_tree = NULL;
  small_gicp_kdtree_t *source_tree = NULL;

  if (small_gicp_preprocess_points(target, 0.05, 10, 1, &target_preprocessed,
                                   &target_tree) != SMALL_GICP_SUCCESS ||
      small_gicp_preprocess_points(source, 0.05, 10, 1, &source_preprocessed,
                                   &source_tree) != SMALL_GICP_SUCCESS) {
    printf("Failed to preprocess point clouds\\n");
    return 1;
  }

  printf("1. Testing Advanced Registration Configuration\\n");
  printf("---------------------------------------------\\n");

  // Create advanced configuration objects
  small_gicp_registration_setting_t *setting = NULL;
  small_gicp_termination_criteria_t *criteria = NULL;
  small_gicp_optimizer_setting_t *optimizer = NULL;
  small_gicp_correspondence_rejector_t *rejector = NULL;
  small_gicp_robust_kernel_t *robust_kernel = NULL;

  // Create default registration setting for GICP
  if (small_gicp_create_default_registration_setting(
          SMALL_GICP_REGISTRATION_TYPE_GICP, &setting) != SMALL_GICP_SUCCESS) {
    printf("Failed to create registration setting\\n");
    return 1;
  }
  printf("✓ Created default registration setting\\n");

  // Create custom termination criteria
  if (small_gicp_create_termination_criteria(1e-4, 0.05 * M_PI / 180.0,
                                             &criteria) != SMALL_GICP_SUCCESS) {
    printf("Failed to create termination criteria\\n");
    return 1;
  }
  printf("✓ Created custom termination criteria (trans_eps=1e-4, "
         "rot_eps=0.05°)\\n");

  // Create Gauss-Newton optimizer
  if (small_gicp_create_default_optimizer_setting(
          SMALL_GICP_OPTIMIZER_GAUSS_NEWTON, &optimizer) !=
      SMALL_GICP_SUCCESS) {
    printf("Failed to create optimizer setting\\n");
    return 1;
  }
  printf("✓ Created Gauss-Newton optimizer setting\\n");

  // Create distance-based correspondence rejector
  if (small_gicp_create_correspondence_rejector(
          SMALL_GICP_REJECTOR_DISTANCE, 0.1, &rejector) != SMALL_GICP_SUCCESS) {
    printf("Failed to create correspondence rejector\\n");
    return 1;
  }
  printf("✓ Created distance correspondence rejector (max_dist=0.1)\\n");

  // Create Huber robust kernel
  if (small_gicp_create_robust_kernel(SMALL_GICP_ROBUST_KERNEL_HUBER, 0.5,
                                      &robust_kernel) != SMALL_GICP_SUCCESS) {
    printf("Failed to create robust kernel\\n");
    return 1;
  }
  printf("✓ Created Huber robust kernel (c=0.5)\\n");

  printf("\\n2. Testing Advanced Registration Functions\\n");
  printf("------------------------------------------\\n");

  // Test advanced registration
  small_gicp_registration_result_extended_t result;
  if (small_gicp_align_advanced(
          target_preprocessed, source_preprocessed, target_tree,
          NULL, // identity initial guess
          setting, criteria, optimizer, rejector, robust_kernel,
          NULL, // no DOF restriction
          &result) == SMALL_GICP_SUCCESS) {
    printf("✓ Advanced registration completed\\n");
    printf("  - Converged: %s\\n", result.converged ? "Yes" : "No");
    printf("  - Iterations: %d\\n", result.iterations);
    printf("  - Error: %.6f\\n", result.error);
    printf("  - Inliers: %d\\n", result.num_inliers);
    printf("  - Information matrix H[0,0]: %.6f\\n", result.H[0]);
    printf("  - Information vector b[0]: %.6f\\n", result.b[0]);
  } else {
    printf("✗ Advanced registration failed\\n");
  }

  // Test registration with custom settings
  small_gicp_registration_result_extended_t result2;
  if (small_gicp_align_with_settings(target_preprocessed, source_preprocessed,
                                     target_tree, setting, NULL,
                                     &result2) == SMALL_GICP_SUCCESS) {
    printf("✓ Registration with custom settings completed\\n");
    printf("  - Converged: %s\\n", result2.converged ? "Yes" : "No");
    printf("  - Iterations: %d\\n", result2.iterations);
    printf("  - Error: %.6f\\n", result2.error);
  } else {
    printf("✗ Registration with custom settings failed\\n");
  }

  printf("\\n3. Testing Utility Functions\\n");
  printf("-----------------------------\\n");

  // Test creating different robust kernels
  small_gicp_robust_kernel_t *cauchy_kernel = NULL;
  if (small_gicp_create_robust_kernel(SMALL_GICP_ROBUST_KERNEL_CAUCHY, 1.0,
                                      &cauchy_kernel) == SMALL_GICP_SUCCESS) {
    printf("✓ Created Cauchy robust kernel\\n");
    small_gicp_destroy_robust_kernel(cauchy_kernel);
  }

  // Test creating Levenberg-Marquardt optimizer
  small_gicp_optimizer_setting_t *lm_optimizer = NULL;
  if (small_gicp_create_default_optimizer_setting(
          SMALL_GICP_OPTIMIZER_LEVENBERG_MARQUARDT, &lm_optimizer) ==
      SMALL_GICP_SUCCESS) {
    printf("✓ Created Levenberg-Marquardt optimizer\\n");
    small_gicp_destroy_optimizer_setting(lm_optimizer);
  }

  // Test DOF restriction
  small_gicp_restrict_dof_factor_t *dof_restriction = NULL;
  double rotation_mask[3] = {1.0, 1.0, 0.0};    // Allow rx, ry, restrict rz
  double translation_mask[3] = {1.0, 1.0, 1.0}; // Allow all translations
  if (small_gicp_create_dof_restriction(1e-3, rotation_mask, translation_mask,
                                        &dof_restriction) ==
      SMALL_GICP_SUCCESS) {
    printf("✓ Created DOF restriction (2D rotation)\\n");
    small_gicp_destroy_dof_restriction(dof_restriction);
  }

  printf("\\n4. Performance Comparison\\n");
  printf("-------------------------\\n");

  // Basic registration for comparison
  small_gicp_registration_result_t basic_result;
  if (small_gicp_align_preprocessed(target_preprocessed, source_preprocessed,
                                    target_tree,
                                    SMALL_GICP_REGISTRATION_TYPE_GICP, NULL, 1,
                                    &basic_result) == SMALL_GICP_SUCCESS) {
    printf("✓ Basic registration completed\\n");
    printf("  - Converged: %s\\n", basic_result.converged ? "Yes" : "No");
    printf("  - Iterations: %d\\n", basic_result.iterations);
    printf("  - Error: %.6f\\n", basic_result.error);
  }

  // Cleanup
  small_gicp_destroy_robust_kernel(robust_kernel);
  small_gicp_destroy_correspondence_rejector(rejector);
  small_gicp_destroy_optimizer_setting(optimizer);
  small_gicp_destroy_termination_criteria(criteria);
  small_gicp_destroy_registration_setting(setting);

  small_gicp_kdtree_destroy(source_tree);
  small_gicp_kdtree_destroy(target_tree);
  small_gicp_point_cloud_destroy(source_preprocessed);
  small_gicp_point_cloud_destroy(target_preprocessed);
  small_gicp_point_cloud_destroy(source);
  small_gicp_point_cloud_destroy(target);

  printf("\\nAdvanced registration example completed successfully!\\n");
  return 0;
}
