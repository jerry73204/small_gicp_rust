#include <small_gicp_c/registration/registration.h>
#include "../common.h"

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