#include "../common.h"
#include <small_gicp_c/registration/registration.h>

#include <small_gicp/factors/general_factor.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/factors/icp_factor.hpp>
#include <small_gicp/factors/plane_icp_factor.hpp>
#include <small_gicp/factors/robust_kernel.hpp>
#include <small_gicp/registration/optimizer.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/reduction_tbb.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/registration/rejector.hpp>
#include <small_gicp/registration/termination_criteria.hpp>

#define CHECK_NULL(ptr)                                                        \
  if (!(ptr))                                                                  \
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;

small_gicp_error_t small_gicp_align(const small_gicp_point_cloud_t *target,
                                    const small_gicp_point_cloud_t *source,
                                    small_gicp_registration_type_t type,
                                    const double *initial_guess,
                                    int num_threads,
                                    small_gicp_registration_result_t *result) {
  CHECK_NULL(target);
  CHECK_NULL(target->cloud);
  CHECK_NULL(source);
  CHECK_NULL(source->cloud);
  CHECK_NULL(result);

  TRY_CATCH_BEGIN
  Eigen::Isometry3d init_guess = Eigen::Isometry3d::Identity();
  if (initial_guess) {
    Eigen::Matrix4d mat;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        mat(i, j) = initial_guess[i * 4 + j];
      }
    }
    init_guess.matrix() = mat;
  }

  small_gicp::RegistrationSetting setting;
  setting.num_threads = num_threads;
  setting.type =
      static_cast<small_gicp::RegistrationSetting::RegistrationType>(type);

  // For direct alignment, we need to preprocess first
  auto [target_preprocessed, target_tree_temp] = small_gicp::preprocess_points(
      *target->cloud, setting.downsampling_resolution, 10, setting.num_threads);
  auto [source_preprocessed, source_tree_temp] = small_gicp::preprocess_points(
      *source->cloud, setting.downsampling_resolution, 10, setting.num_threads);

  small_gicp::RegistrationResult reg_result;
  if (setting.type == small_gicp::RegistrationSetting::VGICP) {
    auto target_voxelmap_temp = small_gicp::create_gaussian_voxelmap(
        *target_preprocessed, setting.voxel_resolution);
    reg_result = small_gicp::align(*target_voxelmap_temp, *source_preprocessed,
                                   init_guess, setting);
  } else {
    reg_result = small_gicp::align(*target_preprocessed, *source_preprocessed,
                                   *target_tree_temp, init_guess, setting);
  }

  // Copy transformation matrix
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      result->T_target_source[i * 4 + j] = reg_result.T_target_source(i, j);
    }
  }

  result->converged = reg_result.converged;
  result->iterations = reg_result.iterations;
  result->num_inliers = reg_result.num_inliers;
  result->error = reg_result.error;
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_align_preprocessed(const small_gicp_point_cloud_t *target,
                              const small_gicp_point_cloud_t *source,
                              const small_gicp_kdtree_t *target_tree,
                              small_gicp_registration_type_t type,
                              const double *initial_guess, int num_threads,
                              small_gicp_registration_result_t *result) {
  CHECK_NULL(target);
  CHECK_NULL(target->cloud);
  CHECK_NULL(source);
  CHECK_NULL(source->cloud);
  CHECK_NULL(target_tree);
  CHECK_NULL(target_tree->tree);
  CHECK_NULL(result);

  TRY_CATCH_BEGIN
  Eigen::Isometry3d init_guess = Eigen::Isometry3d::Identity();
  if (initial_guess) {
    Eigen::Matrix4d mat;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        mat(i, j) = initial_guess[i * 4 + j];
      }
    }
    init_guess.matrix() = mat;
  }

  small_gicp::RegistrationSetting setting;
  setting.num_threads = num_threads;
  setting.type =
      static_cast<small_gicp::RegistrationSetting::RegistrationType>(type);

  auto reg_result = small_gicp::align(*target->cloud, *source->cloud,
                                      *target_tree->tree, init_guess, setting);

  // Copy transformation matrix
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      result->T_target_source[i * 4 + j] = reg_result.T_target_source(i, j);
    }
  }

  result->converged = reg_result.converged;
  result->iterations = reg_result.iterations;
  result->num_inliers = reg_result.num_inliers;
  result->error = reg_result.error;
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_align_vgicp(const small_gicp_gaussian_voxelmap_t *target_voxelmap,
                       const small_gicp_point_cloud_t *source,
                       const double *initial_guess, int num_threads,
                       small_gicp_registration_result_t *result) {
  CHECK_NULL(target_voxelmap);
  CHECK_NULL(target_voxelmap->voxelmap);
  CHECK_NULL(source);
  CHECK_NULL(source->cloud);
  CHECK_NULL(result);

  TRY_CATCH_BEGIN
  Eigen::Isometry3d init_guess = Eigen::Isometry3d::Identity();
  if (initial_guess) {
    Eigen::Matrix4d mat;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        mat(i, j) = initial_guess[i * 4 + j];
      }
    }
    init_guess.matrix() = mat;
  }

  small_gicp::RegistrationSetting setting;
  setting.num_threads = num_threads;

  auto reg_result = small_gicp::align(*target_voxelmap->voxelmap,
                                      *source->cloud, init_guess, setting);

  // Copy transformation matrix
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      result->T_target_source[i * 4 + j] = reg_result.T_target_source(i, j);
    }
  }

  result->converged = reg_result.converged;
  result->iterations = reg_result.iterations;
  result->num_inliers = reg_result.num_inliers;
  result->error = reg_result.error;
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_preprocess_points(const small_gicp_point_cloud_t *cloud,
                             double downsampling_resolution, int num_neighbors,
                             int num_threads,
                             small_gicp_point_cloud_t **preprocessed_cloud,
                             small_gicp_kdtree_t **kdtree) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(preprocessed_cloud);
  CHECK_NULL(kdtree);

  TRY_CATCH_BEGIN
  auto [downsampled, tree] = small_gicp::preprocess_points(
      *cloud->cloud, downsampling_resolution, num_neighbors, num_threads);

  *preprocessed_cloud = new small_gicp_point_cloud;
  (*preprocessed_cloud)->cloud = downsampled;

  *kdtree = new small_gicp_kdtree;
  (*kdtree)->tree = tree;
  TRY_CATCH_END
}

// Advanced registration functions implementation

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
    small_gicp_registration_result_extended_t *result) {
  CHECK_NULL(target);
  CHECK_NULL(target->cloud);
  CHECK_NULL(source);
  CHECK_NULL(source->cloud);
  CHECK_NULL(target_tree);
  CHECK_NULL(target_tree->tree);
  CHECK_NULL(setting);
  CHECK_NULL(result);

  TRY_CATCH_BEGIN
  Eigen::Isometry3d init_guess = Eigen::Isometry3d::Identity();
  if (initial_guess) {
    Eigen::Matrix4d mat;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        mat(i, j) = initial_guess[i * 4 + j];
      }
    }
    init_guess.matrix() = mat;
  }

  // For now, use the basic registration with custom settings
  small_gicp::RegistrationSetting reg_setting;
  reg_setting.type =
      static_cast<small_gicp::RegistrationSetting::RegistrationType>(
          setting->type);
  reg_setting.voxel_resolution = setting->voxel_resolution;
  reg_setting.downsampling_resolution = setting->downsampling_resolution;
  reg_setting.max_correspondence_distance =
      setting->max_correspondence_distance;
  reg_setting.num_threads = setting->num_threads;
  reg_setting.max_iterations = setting->max_iterations;
  reg_setting.verbose = setting->verbose;

  // Apply custom termination criteria
  if (criteria) {
    reg_setting.rotation_eps = criteria->rotation_eps;
    reg_setting.translation_eps = criteria->translation_eps;
  } else {
    reg_setting.rotation_eps = setting->rotation_eps;
    reg_setting.translation_eps = setting->translation_eps;
  }

  auto reg_result =
      small_gicp::align(*target->cloud, *source->cloud, *target_tree->tree,
                        init_guess, reg_setting);

  // Copy results
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      result->T_target_source[i * 4 + j] = reg_result.T_target_source(i, j);
    }
  }

  result->converged = reg_result.converged;
  result->iterations = reg_result.iterations;
  result->num_inliers = reg_result.num_inliers;
  result->error = reg_result.error;

  // Copy information matrix and vector
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      result->H[i * 6 + j] = reg_result.H(i, j);
    }
    result->b[i] = reg_result.b(i);
  }

  TRY_CATCH_END
}

// RegistrationHelper implementations

small_gicp_error_t small_gicp_create_default_registration_helper_setting(
    small_gicp_registration_helper_setting_t **setting) {
  CHECK_NULL(setting);

  TRY_CATCH_BEGIN
  *setting = new small_gicp_registration_helper_setting_t;

  // Set defaults based on small_gicp::RegistrationSetting
  (*setting)->type = SMALL_GICP_GICP;
  (*setting)->voxel_resolution = 1.0;
  (*setting)->downsampling_resolution = 0.25;
  (*setting)->max_correspondence_distance = 1.0;
  (*setting)->rotation_eps = 0.1 * M_PI / 180.0;
  (*setting)->translation_eps = 1e-3;
  (*setting)->num_threads = 4;
  (*setting)->max_iterations = 20;
  (*setting)->verbose = false;
  TRY_CATCH_END
}

small_gicp_error_t small_gicp_create_registration_helper_setting(
    const small_gicp_registration_helper_setting_t *input_setting,
    small_gicp_registration_helper_setting_t **setting) {
  CHECK_NULL(input_setting);
  CHECK_NULL(setting);

  TRY_CATCH_BEGIN
  *setting = new small_gicp_registration_helper_setting_t;
  **setting = *input_setting; // Copy all fields
  TRY_CATCH_END
}

small_gicp_error_t small_gicp_destroy_registration_helper_setting(
    small_gicp_registration_helper_setting_t *setting) {
  if (setting) {
    delete setting;
  }
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t small_gicp_preprocess_points_helper(
    const small_gicp_point_cloud_t *points, double downsampling_resolution,
    int num_neighbors, int num_threads,
    small_gicp_point_cloud_t **preprocessed_cloud,
    small_gicp_kdtree_t **kdtree) {
  CHECK_NULL(points);
  CHECK_NULL(points->cloud);
  CHECK_NULL(preprocessed_cloud);
  CHECK_NULL(kdtree);

  TRY_CATCH_BEGIN
  auto result = small_gicp::preprocess_points(
      *points->cloud, downsampling_resolution, num_neighbors, num_threads);

  // Create preprocessed cloud wrapper
  *preprocessed_cloud = new small_gicp_point_cloud;
  (*preprocessed_cloud)->cloud = result.first;

  // Create kdtree wrapper
  *kdtree = new small_gicp_kdtree;
  (*kdtree)->tree = result.second;
  TRY_CATCH_END
}

small_gicp_error_t small_gicp_create_gaussian_voxelmap_helper(
    const small_gicp_point_cloud_t *points, double voxel_resolution,
    small_gicp_gaussian_voxelmap_t **voxelmap) {
  CHECK_NULL(points);
  CHECK_NULL(points->cloud);
  CHECK_NULL(voxelmap);

  TRY_CATCH_BEGIN
  auto vm =
      small_gicp::create_gaussian_voxelmap(*points->cloud, voxel_resolution);

  *voxelmap = new small_gicp_gaussian_voxelmap;
  (*voxelmap)->voxelmap = vm;
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_align_helper(const small_gicp_point_cloud_t *target,
                        const small_gicp_point_cloud_t *source,
                        const small_gicp_kdtree_t *target_tree,
                        const double *init_T,
                        const small_gicp_registration_helper_setting_t *setting,
                        small_gicp_registration_result_t *result) {
  CHECK_NULL(target);
  CHECK_NULL(target->cloud);
  CHECK_NULL(source);
  CHECK_NULL(source->cloud);
  CHECK_NULL(target_tree);
  CHECK_NULL(target_tree->tree);
  CHECK_NULL(setting);
  CHECK_NULL(result);

  TRY_CATCH_BEGIN
  // Convert initial transformation
  Eigen::Isometry3d init_guess = Eigen::Isometry3d::Identity();
  if (init_T) {
    Eigen::Matrix4d mat;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        mat(i, j) = init_T[i * 4 + j];
      }
    }
    init_guess.matrix() = mat;
  }

  // Convert setting
  small_gicp::RegistrationSetting cpp_setting;
  cpp_setting.type =
      static_cast<small_gicp::RegistrationSetting::RegistrationType>(
          setting->type);
  cpp_setting.voxel_resolution = setting->voxel_resolution;
  cpp_setting.downsampling_resolution = setting->downsampling_resolution;
  cpp_setting.max_correspondence_distance =
      setting->max_correspondence_distance;
  cpp_setting.rotation_eps = setting->rotation_eps;
  cpp_setting.translation_eps = setting->translation_eps;
  cpp_setting.num_threads = setting->num_threads;
  cpp_setting.max_iterations = setting->max_iterations;
  cpp_setting.verbose = setting->verbose;

  auto reg_result =
      small_gicp::align(*target->cloud, *source->cloud, *target_tree->tree,
                        init_guess, cpp_setting);

  // Copy results
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      result->T_target_source[i * 4 + j] = reg_result.T_target_source(i, j);
    }
  }

  result->converged = reg_result.converged;
  result->iterations = reg_result.iterations;
  result->num_inliers = reg_result.num_inliers;
  result->error = reg_result.error;
  TRY_CATCH_END
}

small_gicp_error_t small_gicp_align_helper_vgicp(
    const small_gicp_gaussian_voxelmap_t *target_voxelmap,
    const small_gicp_point_cloud_t *source, const double *init_T,
    const small_gicp_registration_helper_setting_t *setting,
    small_gicp_registration_result_t *result) {
  CHECK_NULL(target_voxelmap);
  CHECK_NULL(target_voxelmap->voxelmap);
  CHECK_NULL(source);
  CHECK_NULL(source->cloud);
  CHECK_NULL(setting);
  CHECK_NULL(result);

  TRY_CATCH_BEGIN
  // Convert initial transformation
  Eigen::Isometry3d init_guess = Eigen::Isometry3d::Identity();
  if (init_T) {
    Eigen::Matrix4d mat;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        mat(i, j) = init_T[i * 4 + j];
      }
    }
    init_guess.matrix() = mat;
  }

  // Convert setting
  small_gicp::RegistrationSetting cpp_setting;
  cpp_setting.type =
      static_cast<small_gicp::RegistrationSetting::RegistrationType>(
          setting->type);
  cpp_setting.voxel_resolution = setting->voxel_resolution;
  cpp_setting.downsampling_resolution = setting->downsampling_resolution;
  cpp_setting.max_correspondence_distance =
      setting->max_correspondence_distance;
  cpp_setting.rotation_eps = setting->rotation_eps;
  cpp_setting.translation_eps = setting->translation_eps;
  cpp_setting.num_threads = setting->num_threads;
  cpp_setting.max_iterations = setting->max_iterations;
  cpp_setting.verbose = setting->verbose;

  auto reg_result = small_gicp::align(*target_voxelmap->voxelmap,
                                      *source->cloud, init_guess, cpp_setting);

  // Copy results
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      result->T_target_source[i * 4 + j] = reg_result.T_target_source(i, j);
    }
  }

  result->converged = reg_result.converged;
  result->iterations = reg_result.iterations;
  result->num_inliers = reg_result.num_inliers;
  result->error = reg_result.error;
  TRY_CATCH_END
}
