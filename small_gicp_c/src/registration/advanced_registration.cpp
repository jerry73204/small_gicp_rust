#include "../common.h"
#include <small_gicp_c/registration/registration.h>

#define CHECK_NULL(ptr)                                                        \
  if (!(ptr))                                                                  \
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;

// Utility functions implementation

small_gicp_error_t small_gicp_align_with_settings(
    const small_gicp_point_cloud_t *target,
    const small_gicp_point_cloud_t *source,
    const small_gicp_kdtree_t *target_tree,
    const small_gicp_registration_setting_t *setting,
    const double *initial_guess,
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

  small_gicp::RegistrationSetting reg_setting;
  reg_setting.type =
      static_cast<small_gicp::RegistrationSetting::RegistrationType>(
          setting->type);
  reg_setting.voxel_resolution = setting->voxel_resolution;
  reg_setting.downsampling_resolution = setting->downsampling_resolution;
  reg_setting.max_correspondence_distance =
      setting->max_correspondence_distance;
  reg_setting.rotation_eps = setting->rotation_eps;
  reg_setting.translation_eps = setting->translation_eps;
  reg_setting.num_threads = setting->num_threads;
  reg_setting.max_iterations = setting->max_iterations;
  reg_setting.verbose = setting->verbose;

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

small_gicp_error_t small_gicp_create_default_termination_criteria(
    small_gicp_termination_criteria_t **criteria) {
  CHECK_NULL(criteria);

  TRY_CATCH_BEGIN
  *criteria = new small_gicp_termination_criteria_t;
  (*criteria)->translation_eps = 1e-3;
  (*criteria)->rotation_eps = 0.1 * M_PI / 180.0;
  TRY_CATCH_END
}

small_gicp_error_t small_gicp_create_termination_criteria(
    double translation_eps, double rotation_eps,
    small_gicp_termination_criteria_t **criteria) {
  CHECK_NULL(criteria);

  TRY_CATCH_BEGIN
  *criteria = new small_gicp_termination_criteria_t;
  (*criteria)->translation_eps = translation_eps;
  (*criteria)->rotation_eps = rotation_eps;
  TRY_CATCH_END
}

small_gicp_error_t small_gicp_destroy_termination_criteria(
    small_gicp_termination_criteria_t *criteria) {
  if (criteria) {
    delete criteria;
  }
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t
small_gicp_create_robust_kernel(small_gicp_robust_kernel_type_t type, double c,
                                small_gicp_robust_kernel_t **kernel) {
  CHECK_NULL(kernel);

  TRY_CATCH_BEGIN
  *kernel = new small_gicp_robust_kernel_t;
  (*kernel)->type = type;
  (*kernel)->c = c;
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_destroy_robust_kernel(small_gicp_robust_kernel_t *kernel) {
  if (kernel) {
    delete kernel;
  }
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t small_gicp_create_default_optimizer_setting(
    small_gicp_optimizer_type_t type,
    small_gicp_optimizer_setting_t **optimizer) {
  CHECK_NULL(optimizer);

  TRY_CATCH_BEGIN
  *optimizer = new small_gicp_optimizer_setting_t;
  (*optimizer)->type = type;
  (*optimizer)->verbose = false;

  if (type == SMALL_GICP_OPTIMIZER_GAUSS_NEWTON) {
    (*optimizer)->max_iterations = 20;
    (*optimizer)->lambda = 1e-6;
    (*optimizer)->max_inner_iterations = 1;
    (*optimizer)->lambda_factor = 2.0;
  } else { // Levenberg-Marquardt
    (*optimizer)->max_iterations = 20;
    (*optimizer)->lambda = 1e-6;
    (*optimizer)->max_inner_iterations = 5;
    (*optimizer)->lambda_factor = 2.0;
  }
  TRY_CATCH_END
}

small_gicp_error_t small_gicp_create_optimizer_setting(
    const small_gicp_optimizer_setting_t *setting,
    small_gicp_optimizer_setting_t **optimizer) {
  CHECK_NULL(setting);
  CHECK_NULL(optimizer);

  TRY_CATCH_BEGIN
  *optimizer = new small_gicp_optimizer_setting_t;
  **optimizer = *setting;
  TRY_CATCH_END
}

small_gicp_error_t small_gicp_destroy_optimizer_setting(
    small_gicp_optimizer_setting_t *optimizer) {
  if (optimizer) {
    delete optimizer;
  }
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t small_gicp_create_correspondence_rejector(
    small_gicp_correspondence_rejector_type_t type, double max_dist_sq,
    small_gicp_correspondence_rejector_t **rejector) {
  CHECK_NULL(rejector);

  TRY_CATCH_BEGIN
  *rejector = new small_gicp_correspondence_rejector_t;
  (*rejector)->type = type;
  (*rejector)->max_dist_sq = max_dist_sq;
  TRY_CATCH_END
}

small_gicp_error_t small_gicp_destroy_correspondence_rejector(
    small_gicp_correspondence_rejector_t *rejector) {
  if (rejector) {
    delete rejector;
  }
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t small_gicp_create_default_registration_setting(
    small_gicp_registration_type_t type,
    small_gicp_registration_setting_t **setting) {
  CHECK_NULL(setting);

  TRY_CATCH_BEGIN
  *setting = new small_gicp_registration_setting_t;
  (*setting)->type = type;
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

small_gicp_error_t small_gicp_destroy_registration_setting(
    small_gicp_registration_setting_t *setting) {
  if (setting) {
    delete setting;
  }
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t small_gicp_create_dof_restriction(
    double lambda, const double *rotation_mask, const double *translation_mask,
    small_gicp_restrict_dof_factor_t **dof_restriction) {
  CHECK_NULL(rotation_mask);
  CHECK_NULL(translation_mask);
  CHECK_NULL(dof_restriction);

  TRY_CATCH_BEGIN
  *dof_restriction = new small_gicp_restrict_dof_factor_t;
  (*dof_restriction)->lambda = lambda;

  for (int i = 0; i < 3; ++i) {
    (*dof_restriction)->rotation_mask[i] = rotation_mask[i];
    (*dof_restriction)->translation_mask[i] = translation_mask[i];
  }
  TRY_CATCH_END
}

small_gicp_error_t small_gicp_destroy_dof_restriction(
    small_gicp_restrict_dof_factor_t *dof_restriction) {
  if (dof_restriction) {
    delete dof_restriction;
  }
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t
small_gicp_set_reduction_strategy(small_gicp_reduction_type_t type,
                                  int num_threads) {
  // This is a placeholder - in practice, reduction strategy would be set
  // per registration call rather than globally
  return SMALL_GICP_SUCCESS;
}
