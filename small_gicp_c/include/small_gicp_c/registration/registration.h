#ifndef SMALL_GICP_C_REGISTRATION_REGISTRATION_H
#define SMALL_GICP_C_REGISTRATION_REGISTRATION_H

#include <small_gicp_c/types.h>

#ifdef __cplusplus
extern "C" {
#endif

// Advanced registration configuration structures

// Termination criteria for registration convergence
typedef struct {
  double translation_eps; // Translation tolerance [m]
  double rotation_eps;    // Rotation tolerance [rad]
} small_gicp_termination_criteria_t;

// Robust kernel for outlier rejection
typedef enum {
  SMALL_GICP_ROBUST_KERNEL_NONE = 0,
  SMALL_GICP_ROBUST_KERNEL_HUBER = 1,
  SMALL_GICP_ROBUST_KERNEL_CAUCHY = 2
} small_gicp_robust_kernel_type_t;

typedef struct {
  small_gicp_robust_kernel_type_t type;
  double c; // Kernel width parameter
} small_gicp_robust_kernel_t;

// Optimizer configuration
typedef enum {
  SMALL_GICP_OPTIMIZER_GAUSS_NEWTON = 0,
  SMALL_GICP_OPTIMIZER_LEVENBERG_MARQUARDT = 1
} small_gicp_optimizer_type_t;

typedef struct {
  small_gicp_optimizer_type_t type;
  bool verbose;
  int max_iterations;
  double lambda;            // For Gauss-Newton or init_lambda for LM
  int max_inner_iterations; // Only for Levenberg-Marquardt
  double lambda_factor;     // Only for Levenberg-Marquardt
} small_gicp_optimizer_setting_t;

// Correspondence rejection methods
typedef enum {
  SMALL_GICP_REJECTOR_NONE = 0,
  SMALL_GICP_REJECTOR_DISTANCE = 1
} small_gicp_correspondence_rejector_type_t;

typedef struct {
  small_gicp_correspondence_rejector_type_t type;
  double max_dist_sq; // Maximum squared distance for distance rejector
} small_gicp_correspondence_rejector_t;

// Parallel processing configuration
typedef enum {
  SMALL_GICP_REDUCTION_SERIAL = 0,
  SMALL_GICP_REDUCTION_OPENMP = 1,
  SMALL_GICP_REDUCTION_TBB = 2
} small_gicp_reduction_type_t;

// Degrees of freedom restriction
typedef struct {
  double lambda;              // Regularization parameter
  double rotation_mask[3];    // rx, ry, rz (1.0 = active, 0.0 = inactive)
  double translation_mask[3]; // tx, ty, tz (1.0 = active, 0.0 = inactive)
} small_gicp_restrict_dof_factor_t;

// Advanced registration settings
typedef struct {
  small_gicp_registration_type_t type;
  double voxel_resolution;
  double downsampling_resolution;
  double max_correspondence_distance;
  double rotation_eps;
  double translation_eps;
  int num_threads;
  int max_iterations;
  bool verbose;
} small_gicp_registration_setting_t;

// Extended registration result with information matrix
typedef struct {
  double T_target_source[16]; // 4x4 transformation matrix in row-major order
  bool converged;
  int iterations;
  int num_inliers;
  double H[36]; // 6x6 information matrix in row-major order
  double b[6];  // information vector
  double error;
} small_gicp_registration_result_extended_t;

// Complete preprocessing pipeline
small_gicp_error_t
small_gicp_preprocess_points(const small_gicp_point_cloud_t *cloud,
                             double downsampling_resolution, int num_neighbors,
                             int num_threads,
                             small_gicp_point_cloud_t **preprocessed_cloud,
                             small_gicp_kdtree_t **kdtree);

// Basic registration with automatic preprocessing
small_gicp_error_t small_gicp_align(
    const small_gicp_point_cloud_t *target,
    const small_gicp_point_cloud_t *source, small_gicp_registration_type_t type,
    const double
        *initial_guess, // 4x4 matrix in row-major order, NULL for identity
    int num_threads, small_gicp_registration_result_t *result);

// Registration with preprocessed point clouds
small_gicp_error_t
small_gicp_align_preprocessed(const small_gicp_point_cloud_t *target,
                              const small_gicp_point_cloud_t *source,
                              const small_gicp_kdtree_t *target_tree,
                              small_gicp_registration_type_t type,
                              const double *initial_guess, int num_threads,
                              small_gicp_registration_result_t *result);

// VGICP registration
small_gicp_error_t
small_gicp_align_vgicp(const small_gicp_gaussian_voxelmap_t *target_voxelmap,
                       const small_gicp_point_cloud_t *source,
                       const double *initial_guess, int num_threads,
                       small_gicp_registration_result_t *result);

// Advanced registration functions

// Advanced registration with full configuration
small_gicp_error_t small_gicp_align_advanced(
    const small_gicp_point_cloud_t *target,
    const small_gicp_point_cloud_t *source,
    const small_gicp_kdtree_t *target_tree, const double *initial_guess,
    const small_gicp_registration_setting_t *setting,
    const small_gicp_termination_criteria_t *criteria,
    const small_gicp_optimizer_setting_t *optimizer,
    const small_gicp_correspondence_rejector_t *rejector,
    const small_gicp_robust_kernel_t *robust_kernel,
    const small_gicp_restrict_dof_factor_t *dof_restriction,
    small_gicp_registration_result_extended_t *result);

// Registration with custom settings
small_gicp_error_t small_gicp_align_with_settings(
    const small_gicp_point_cloud_t *target,
    const small_gicp_point_cloud_t *source,
    const small_gicp_kdtree_t *target_tree,
    const small_gicp_registration_setting_t *setting,
    const double *initial_guess,
    small_gicp_registration_result_extended_t *result);

// Utility functions for advanced features

// Create default termination criteria
small_gicp_error_t small_gicp_create_default_termination_criteria(
    small_gicp_termination_criteria_t **criteria);

// Create custom termination criteria
small_gicp_error_t small_gicp_create_termination_criteria(
    double translation_eps, double rotation_eps,
    small_gicp_termination_criteria_t **criteria);

// Destroy termination criteria
small_gicp_error_t small_gicp_destroy_termination_criteria(
    small_gicp_termination_criteria_t *criteria);

// Create robust kernel
small_gicp_error_t
small_gicp_create_robust_kernel(small_gicp_robust_kernel_type_t type, double c,
                                small_gicp_robust_kernel_t **kernel);

// Destroy robust kernel
small_gicp_error_t
small_gicp_destroy_robust_kernel(small_gicp_robust_kernel_t *kernel);

// Create default optimizer setting
small_gicp_error_t small_gicp_create_default_optimizer_setting(
    small_gicp_optimizer_type_t type,
    small_gicp_optimizer_setting_t **optimizer);

// Create custom optimizer setting
small_gicp_error_t small_gicp_create_optimizer_setting(
    const small_gicp_optimizer_setting_t *setting,
    small_gicp_optimizer_setting_t **optimizer);

// Destroy optimizer setting
small_gicp_error_t
small_gicp_destroy_optimizer_setting(small_gicp_optimizer_setting_t *optimizer);

// Create correspondence rejector
small_gicp_error_t small_gicp_create_correspondence_rejector(
    small_gicp_correspondence_rejector_type_t type, double max_dist_sq,
    small_gicp_correspondence_rejector_t **rejector);

// Destroy correspondence rejector
small_gicp_error_t small_gicp_destroy_correspondence_rejector(
    small_gicp_correspondence_rejector_t *rejector);

// Create default registration setting
small_gicp_error_t small_gicp_create_default_registration_setting(
    small_gicp_registration_type_t type,
    small_gicp_registration_setting_t **setting);

// Destroy registration setting
small_gicp_error_t small_gicp_destroy_registration_setting(
    small_gicp_registration_setting_t *setting);

// Create DOF restriction factor
small_gicp_error_t small_gicp_create_dof_restriction(
    double lambda,
    const double *rotation_mask,    // 3 elements: rx, ry, rz
    const double *translation_mask, // 3 elements: tx, ty, tz
    small_gicp_restrict_dof_factor_t **dof_restriction);

// Destroy DOF restriction factor
small_gicp_error_t small_gicp_destroy_dof_restriction(
    small_gicp_restrict_dof_factor_t *dof_restriction);

// Set parallel reduction strategy
small_gicp_error_t
small_gicp_set_reduction_strategy(small_gicp_reduction_type_t type,
                                  int num_threads);

// RegistrationHelper functions

// RegistrationHelper setting structure
typedef struct {
  small_gicp_registration_type_t type;
  double voxel_resolution;
  double downsampling_resolution;
  double max_correspondence_distance;
  double rotation_eps;
  double translation_eps;
  int num_threads;
  int max_iterations;
  bool verbose;
} small_gicp_registration_helper_setting_t;

// Create default RegistrationHelper setting
small_gicp_error_t small_gicp_create_default_registration_helper_setting(
    small_gicp_registration_helper_setting_t **setting);

// Create custom RegistrationHelper setting
small_gicp_error_t small_gicp_create_registration_helper_setting(
    const small_gicp_registration_helper_setting_t *input_setting,
    small_gicp_registration_helper_setting_t **setting);

// Destroy RegistrationHelper setting
small_gicp_error_t small_gicp_destroy_registration_helper_setting(
    small_gicp_registration_helper_setting_t *setting);

// RegistrationHelper preprocessing functions
small_gicp_error_t small_gicp_preprocess_points_helper(
    const small_gicp_point_cloud_t *points, double downsampling_resolution,
    int num_neighbors, int num_threads,
    small_gicp_point_cloud_t **preprocessed_cloud,
    small_gicp_kdtree_t **kdtree);

// Create Gaussian voxelmap
small_gicp_error_t small_gicp_create_gaussian_voxelmap_helper(
    const small_gicp_point_cloud_t *points, double voxel_resolution,
    small_gicp_gaussian_voxelmap_t **voxelmap);

// RegistrationHelper align function with point clouds
small_gicp_error_t
small_gicp_align_helper(const small_gicp_point_cloud_t *target,
                        const small_gicp_point_cloud_t *source,
                        const small_gicp_kdtree_t *target_tree,
                        const double *init_T, // 4x4 matrix in row-major order
                        const small_gicp_registration_helper_setting_t *setting,
                        small_gicp_registration_result_t *result);

// RegistrationHelper align function with voxelmap
small_gicp_error_t small_gicp_align_helper_vgicp(
    const small_gicp_gaussian_voxelmap_t *target_voxelmap,
    const small_gicp_point_cloud_t *source,
    const double *init_T, // 4x4 matrix in row-major order
    const small_gicp_registration_helper_setting_t *setting,
    small_gicp_registration_result_t *result);

// Direct factor utility functions

// Point-to-point ICP error computation
small_gicp_error_t
small_gicp_compute_icp_error(const double *source_point, // 3D point [x, y, z]
                             const double *target_point, // 3D point [x, y, z]
                             double *error);             // Output: scalar error

// Point-to-plane ICP error computation
small_gicp_error_t small_gicp_compute_point_to_plane_error(
    const double *source_point,  // 3D point [x, y, z]
    const double *target_point,  // 3D point [x, y, z]
    const double *target_normal, // 3D normal [nx, ny, nz]
    double *error);              // Output: scalar error

// GICP error computation
small_gicp_error_t small_gicp_compute_gicp_error(
    const double *source_point, // 3D point [x, y, z]
    const double *target_point, // 3D point [x, y, z]
    const double *source_cov,   // 3x3 covariance matrix [row-major]
    const double *target_cov,   // 3x3 covariance matrix [row-major]
    double *error);             // Output: scalar error

// Robust kernel weight computation
small_gicp_error_t
small_gicp_compute_robust_weight(const small_gicp_robust_kernel_t *kernel,
                                 double error,    // Input: error value
                                 double *weight); // Output: robust weight

// Combined robust error computation
small_gicp_error_t small_gicp_compute_robust_icp_error(
    const small_gicp_robust_kernel_t *kernel,
    const double *source_point, // 3D point [x, y, z]
    const double *target_point, // 3D point [x, y, z]
    double *error,              // Output: weighted error
    double *weight);            // Output: robust weight [optional]

small_gicp_error_t small_gicp_compute_robust_point_to_plane_error(
    const small_gicp_robust_kernel_t *kernel,
    const double *source_point,  // 3D point [x, y, z]
    const double *target_point,  // 3D point [x, y, z]
    const double *target_normal, // 3D normal [nx, ny, nz]
    double *error,               // Output: weighted error
    double *weight);             // Output: robust weight [optional]

small_gicp_error_t small_gicp_compute_robust_gicp_error(
    const small_gicp_robust_kernel_t *kernel,
    const double *source_point, // 3D point [x, y, z]
    const double *target_point, // 3D point [x, y, z]
    const double *source_cov,   // 3x3 covariance matrix [row-major]
    const double *target_cov,   // 3x3 covariance matrix [row-major]
    double *error,              // Output: weighted error
    double *weight);            // Output: robust weight [optional]

#ifdef __cplusplus
}
#endif

#endif // SMALL_GICP_C_REGISTRATION_REGISTRATION_H
