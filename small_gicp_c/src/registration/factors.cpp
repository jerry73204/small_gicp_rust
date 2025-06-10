#include "../common.h"
#include <small_gicp_c/registration/registration.h>

#include <small_gicp/factors/robust_kernel.hpp>

#define CHECK_NULL(ptr)                                                        \
  if (!(ptr))                                                                  \
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;

// Point-to-point ICP error computation
small_gicp_error_t small_gicp_compute_icp_error(const double *source_point,
                                                const double *target_point,
                                                double *error) {
  CHECK_NULL(source_point);
  CHECK_NULL(target_point);
  CHECK_NULL(error);

  TRY_CATCH_BEGIN
  Eigen::Vector3d src_pt(source_point[0], source_point[1], source_point[2]);
  Eigen::Vector3d tgt_pt(target_point[0], target_point[1], target_point[2]);

  Eigen::Vector3d residual = tgt_pt - src_pt;
  *error = 0.5 * residual.squaredNorm();
  TRY_CATCH_END
}

// Point-to-plane ICP error computation
small_gicp_error_t small_gicp_compute_point_to_plane_error(
    const double *source_point, const double *target_point,
    const double *target_normal, double *error) {
  CHECK_NULL(source_point);
  CHECK_NULL(target_point);
  CHECK_NULL(target_normal);
  CHECK_NULL(error);

  TRY_CATCH_BEGIN
  Eigen::Vector3d src_pt(source_point[0], source_point[1], source_point[2]);
  Eigen::Vector3d tgt_pt(target_point[0], target_point[1], target_point[2]);
  Eigen::Vector3d tgt_nrm(target_normal[0], target_normal[1], target_normal[2]);

  Eigen::Vector3d residual = tgt_pt - src_pt;
  double plane_error = residual.dot(tgt_nrm);
  *error = 0.5 * plane_error * plane_error;
  TRY_CATCH_END
}

// GICP error computation
small_gicp_error_t small_gicp_compute_gicp_error(const double *source_point,
                                                 const double *target_point,
                                                 const double *source_cov,
                                                 const double *target_cov,
                                                 double *error) {
  CHECK_NULL(source_point);
  CHECK_NULL(target_point);
  CHECK_NULL(source_cov);
  CHECK_NULL(target_cov);
  CHECK_NULL(error);

  TRY_CATCH_BEGIN
  Eigen::Vector3d src_pt(source_point[0], source_point[1], source_point[2]);
  Eigen::Vector3d tgt_pt(target_point[0], target_point[1], target_point[2]);

  Eigen::Matrix3d src_cov_mat, tgt_cov_mat;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      src_cov_mat(i, j) = source_cov[i * 3 + j];
      tgt_cov_mat(i, j) = target_cov[i * 3 + j];
    }
  }

  Eigen::Vector3d residual = tgt_pt - src_pt;
  Eigen::Matrix3d combined_cov = src_cov_mat + tgt_cov_mat;

  // Compute GICP error: 0.5 * residual^T * (cov_src + cov_tgt)^-1 * residual
  Eigen::Matrix3d combined_cov_inv = combined_cov.inverse();
  *error = 0.5 * residual.transpose() * combined_cov_inv * residual;
  TRY_CATCH_END
}

// Robust kernel weight computation
small_gicp_error_t
small_gicp_compute_robust_weight(const small_gicp_robust_kernel_t *kernel,
                                 double error, double *weight) {
  CHECK_NULL(kernel);
  CHECK_NULL(weight);

  TRY_CATCH_BEGIN
  if (kernel->type == SMALL_GICP_ROBUST_KERNEL_HUBER) {
    small_gicp::Huber::Setting huber_setting;
    huber_setting.c = kernel->c;
    small_gicp::Huber huber_kernel(huber_setting);
    *weight = huber_kernel.weight(std::sqrt(
        2.0 * error)); // Convert from squared error to residual magnitude
  } else if (kernel->type == SMALL_GICP_ROBUST_KERNEL_CAUCHY) {
    small_gicp::Cauchy::Setting cauchy_setting;
    cauchy_setting.c = kernel->c;
    small_gicp::Cauchy cauchy_kernel(cauchy_setting);
    *weight = cauchy_kernel.weight(std::sqrt(
        2.0 * error)); // Convert from squared error to residual magnitude
  } else {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }
  TRY_CATCH_END
}

// Combined robust ICP error computation
small_gicp_error_t small_gicp_compute_robust_icp_error(
    const small_gicp_robust_kernel_t *kernel, const double *source_point,
    const double *target_point, double *error, double *weight) {
  CHECK_NULL(kernel);
  CHECK_NULL(source_point);
  CHECK_NULL(target_point);
  CHECK_NULL(error);

  TRY_CATCH_BEGIN
  // Compute base ICP error
  double base_error;
  small_gicp_error_t result =
      small_gicp_compute_icp_error(source_point, target_point, &base_error);
  if (result != SMALL_GICP_SUCCESS) {
    return result;
  }

  // Compute robust weight
  double robust_weight;
  result = small_gicp_compute_robust_weight(kernel, base_error, &robust_weight);
  if (result != SMALL_GICP_SUCCESS) {
    return result;
  }

  *error = robust_weight * base_error;
  if (weight) {
    *weight = robust_weight;
  }
  TRY_CATCH_END
}

// Combined robust point-to-plane error computation
small_gicp_error_t small_gicp_compute_robust_point_to_plane_error(
    const small_gicp_robust_kernel_t *kernel, const double *source_point,
    const double *target_point, const double *target_normal, double *error,
    double *weight) {
  CHECK_NULL(kernel);
  CHECK_NULL(source_point);
  CHECK_NULL(target_point);
  CHECK_NULL(target_normal);
  CHECK_NULL(error);

  TRY_CATCH_BEGIN
  // Compute base point-to-plane error
  double base_error;
  small_gicp_error_t result = small_gicp_compute_point_to_plane_error(
      source_point, target_point, target_normal, &base_error);
  if (result != SMALL_GICP_SUCCESS) {
    return result;
  }

  // Compute robust weight
  double robust_weight;
  result = small_gicp_compute_robust_weight(kernel, base_error, &robust_weight);
  if (result != SMALL_GICP_SUCCESS) {
    return result;
  }

  *error = robust_weight * base_error;
  if (weight) {
    *weight = robust_weight;
  }
  TRY_CATCH_END
}

// Combined robust GICP error computation
small_gicp_error_t small_gicp_compute_robust_gicp_error(
    const small_gicp_robust_kernel_t *kernel, const double *source_point,
    const double *target_point, const double *source_cov,
    const double *target_cov, double *error, double *weight) {
  CHECK_NULL(kernel);
  CHECK_NULL(source_point);
  CHECK_NULL(target_point);
  CHECK_NULL(source_cov);
  CHECK_NULL(target_cov);
  CHECK_NULL(error);

  TRY_CATCH_BEGIN
  // Compute base GICP error
  double base_error;
  small_gicp_error_t result = small_gicp_compute_gicp_error(
      source_point, target_point, source_cov, target_cov, &base_error);
  if (result != SMALL_GICP_SUCCESS) {
    return result;
  }

  // Compute robust weight
  double robust_weight;
  result = small_gicp_compute_robust_weight(kernel, base_error, &robust_weight);
  if (result != SMALL_GICP_SUCCESS) {
    return result;
  }

  *error = robust_weight * base_error;
  if (weight) {
    *weight = robust_weight;
  }
  TRY_CATCH_END
}
