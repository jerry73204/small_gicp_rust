#include "../common.h"
#include <small_gicp/util/normal_estimation.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
#include <small_gicp_c/util/local_features.h>

#ifdef SMALL_GICP_HAS_TBB
#include <small_gicp/util/normal_estimation_tbb.hpp>
#endif

#define CHECK_NULL(ptr)                                                        \
  if (!(ptr))                                                                  \
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;

small_gicp_error_t small_gicp_estimate_local_features_single_point(
    small_gicp_point_cloud_t *cloud, const small_gicp_kdtree_t *kdtree,
    size_t point_index, int num_neighbors,
    small_gicp_setter_type_t setter_type) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(kdtree);
  CHECK_NULL(kdtree->tree);

  if (point_index >= cloud->cloud->size()) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  TRY_CATCH_BEGIN
  switch (setter_type) {
  case SMALL_GICP_SETTER_NORMAL:
    small_gicp::estimate_local_features<
        small_gicp::NormalSetter<small_gicp::PointCloud>>(
        *cloud->cloud, *kdtree->tree, num_neighbors, point_index);
    break;
  case SMALL_GICP_SETTER_COVARIANCE:
    small_gicp::estimate_local_features<
        small_gicp::CovarianceSetter<small_gicp::PointCloud>>(
        *cloud->cloud, *kdtree->tree, num_neighbors, point_index);
    break;
  case SMALL_GICP_SETTER_NORMAL_COVARIANCE:
    small_gicp::estimate_local_features<
        small_gicp::NormalCovarianceSetter<small_gicp::PointCloud>>(
        *cloud->cloud, *kdtree->tree, num_neighbors, point_index);
    break;
  default:
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }
  TRY_CATCH_END
}

small_gicp_error_t small_gicp_estimate_local_features_cloud(
    small_gicp_point_cloud_t *cloud, const small_gicp_kdtree_t *kdtree,
    int num_neighbors, small_gicp_setter_type_t setter_type,
    small_gicp_local_features_backend_t backend, int num_threads) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(kdtree);
  CHECK_NULL(kdtree->tree);

  TRY_CATCH_BEGIN
  switch (backend) {
  case SMALL_GICP_LOCAL_FEATURES_BACKEND_DEFAULT:
    switch (setter_type) {
    case SMALL_GICP_SETTER_NORMAL:
      small_gicp::estimate_local_features<
          small_gicp::NormalSetter<small_gicp::PointCloud>>(
          *cloud->cloud, *kdtree->tree, num_neighbors);
      break;
    case SMALL_GICP_SETTER_COVARIANCE:
      small_gicp::estimate_local_features<
          small_gicp::CovarianceSetter<small_gicp::PointCloud>>(
          *cloud->cloud, *kdtree->tree, num_neighbors);
      break;
    case SMALL_GICP_SETTER_NORMAL_COVARIANCE:
      small_gicp::estimate_local_features<
          small_gicp::NormalCovarianceSetter<small_gicp::PointCloud>>(
          *cloud->cloud, *kdtree->tree, num_neighbors);
      break;
    default:
      return SMALL_GICP_ERROR_INVALID_ARGUMENT;
    }
    break;

  case SMALL_GICP_LOCAL_FEATURES_BACKEND_OPENMP:
    switch (setter_type) {
    case SMALL_GICP_SETTER_NORMAL:
      small_gicp::estimate_normals_omp(*cloud->cloud, *kdtree->tree,
                                       num_neighbors, num_threads);
      break;
    case SMALL_GICP_SETTER_COVARIANCE:
      small_gicp::estimate_covariances_omp(*cloud->cloud, *kdtree->tree,
                                           num_neighbors, num_threads);
      break;
    case SMALL_GICP_SETTER_NORMAL_COVARIANCE:
      small_gicp::estimate_normals_covariances_omp(*cloud->cloud, *kdtree->tree,
                                                   num_neighbors, num_threads);
      break;
    default:
      return SMALL_GICP_ERROR_INVALID_ARGUMENT;
    }
    break;

  case SMALL_GICP_LOCAL_FEATURES_BACKEND_TBB:
#ifdef SMALL_GICP_HAS_TBB
    switch (setter_type) {
    case SMALL_GICP_SETTER_NORMAL:
      small_gicp::estimate_normals_tbb(*cloud->cloud, *kdtree->tree,
                                       num_neighbors, num_threads);
      break;
    case SMALL_GICP_SETTER_COVARIANCE:
      small_gicp::estimate_covariances_tbb(*cloud->cloud, *kdtree->tree,
                                           num_neighbors, num_threads);
      break;
    case SMALL_GICP_SETTER_NORMAL_COVARIANCE:
      small_gicp::estimate_normals_covariances_tbb(*cloud->cloud, *kdtree->tree,
                                                   num_neighbors, num_threads);
      break;
    default:
      return SMALL_GICP_ERROR_INVALID_ARGUMENT;
    }
#else
    return SMALL_GICP_ERROR_INVALID_ARGUMENT; // TBB not available
#endif
    break;

  default:
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }
  TRY_CATCH_END
}

small_gicp_error_t small_gicp_estimate_local_features_auto(
    small_gicp_point_cloud_t *cloud, int num_neighbors,
    small_gicp_setter_type_t setter_type,
    small_gicp_local_features_backend_t backend, int num_threads) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);

  TRY_CATCH_BEGIN
  switch (backend) {
  case SMALL_GICP_LOCAL_FEATURES_BACKEND_DEFAULT:
    switch (setter_type) {
    case SMALL_GICP_SETTER_NORMAL:
      small_gicp::estimate_local_features<
          small_gicp::NormalSetter<small_gicp::PointCloud>>(*cloud->cloud,
                                                            num_neighbors);
      break;
    case SMALL_GICP_SETTER_COVARIANCE:
      small_gicp::estimate_local_features<
          small_gicp::CovarianceSetter<small_gicp::PointCloud>>(*cloud->cloud,
                                                                num_neighbors);
      break;
    case SMALL_GICP_SETTER_NORMAL_COVARIANCE:
      small_gicp::estimate_local_features<
          small_gicp::NormalCovarianceSetter<small_gicp::PointCloud>>(
          *cloud->cloud, num_neighbors);
      break;
    default:
      return SMALL_GICP_ERROR_INVALID_ARGUMENT;
    }
    break;

  case SMALL_GICP_LOCAL_FEATURES_BACKEND_OPENMP:
    switch (setter_type) {
    case SMALL_GICP_SETTER_NORMAL:
      small_gicp::estimate_normals_omp(*cloud->cloud, num_neighbors,
                                       num_threads);
      break;
    case SMALL_GICP_SETTER_COVARIANCE:
      small_gicp::estimate_covariances_omp(*cloud->cloud, num_neighbors,
                                           num_threads);
      break;
    case SMALL_GICP_SETTER_NORMAL_COVARIANCE:
      small_gicp::estimate_normals_covariances_omp(*cloud->cloud, num_neighbors,
                                                   num_threads);
      break;
    default:
      return SMALL_GICP_ERROR_INVALID_ARGUMENT;
    }
    break;

  case SMALL_GICP_LOCAL_FEATURES_BACKEND_TBB:
#ifdef SMALL_GICP_HAS_TBB
    switch (setter_type) {
    case SMALL_GICP_SETTER_NORMAL:
      small_gicp::estimate_normals_tbb(*cloud->cloud, num_neighbors,
                                       num_threads);
      break;
    case SMALL_GICP_SETTER_COVARIANCE:
      small_gicp::estimate_covariances_tbb(*cloud->cloud, num_neighbors,
                                           num_threads);
      break;
    case SMALL_GICP_SETTER_NORMAL_COVARIANCE:
      small_gicp::estimate_normals_covariances_tbb(*cloud->cloud, num_neighbors,
                                                   num_threads);
      break;
    default:
      return SMALL_GICP_ERROR_INVALID_ARGUMENT;
    }
#else
    return SMALL_GICP_ERROR_INVALID_ARGUMENT; // TBB not available
#endif
    break;

  default:
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }
  TRY_CATCH_END
}

// Direct setter interface implementations

small_gicp_error_t small_gicp_normal_setter_set(small_gicp_point_cloud_t *cloud,
                                                size_t point_index,
                                                const double *eigenvectors) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(eigenvectors);

  if (point_index >= cloud->cloud->size()) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  TRY_CATCH_BEGIN
  // Convert eigenvectors from row-major order to Eigen matrix
  Eigen::Matrix3d eig_mat;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      eig_mat(i, j) = eigenvectors[i * 3 + j];
    }
  }

  small_gicp::NormalSetter<small_gicp::PointCloud>::set(*cloud->cloud,
                                                        point_index, eig_mat);
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_covariance_setter_set(small_gicp_point_cloud_t *cloud,
                                 size_t point_index,
                                 const double *eigenvectors) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(eigenvectors);

  if (point_index >= cloud->cloud->size()) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  TRY_CATCH_BEGIN
  // Convert eigenvectors from row-major order to Eigen matrix
  Eigen::Matrix3d eig_mat;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      eig_mat(i, j) = eigenvectors[i * 3 + j];
    }
  }

  small_gicp::CovarianceSetter<small_gicp::PointCloud>::set(
      *cloud->cloud, point_index, eig_mat);
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_normal_covariance_setter_set(small_gicp_point_cloud_t *cloud,
                                        size_t point_index,
                                        const double *eigenvectors) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(eigenvectors);

  if (point_index >= cloud->cloud->size()) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  TRY_CATCH_BEGIN
  // Convert eigenvectors from row-major order to Eigen matrix
  Eigen::Matrix3d eig_mat;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      eig_mat(i, j) = eigenvectors[i * 3 + j];
    }
  }

  small_gicp::NormalCovarianceSetter<small_gicp::PointCloud>::set(
      *cloud->cloud, point_index, eig_mat);
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_normal_setter_set_invalid(small_gicp_point_cloud_t *cloud,
                                     size_t point_index) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);

  if (point_index >= cloud->cloud->size()) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  TRY_CATCH_BEGIN
  small_gicp::NormalSetter<small_gicp::PointCloud>::set_invalid(*cloud->cloud,
                                                                point_index);
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_covariance_setter_set_invalid(small_gicp_point_cloud_t *cloud,
                                         size_t point_index) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);

  if (point_index >= cloud->cloud->size()) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  TRY_CATCH_BEGIN
  small_gicp::CovarianceSetter<small_gicp::PointCloud>::set_invalid(
      *cloud->cloud, point_index);
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_normal_covariance_setter_set_invalid(small_gicp_point_cloud_t *cloud,
                                                size_t point_index) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);

  if (point_index >= cloud->cloud->size()) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  TRY_CATCH_BEGIN
  small_gicp::NormalCovarianceSetter<small_gicp::PointCloud>::set_invalid(
      *cloud->cloud, point_index);
  TRY_CATCH_END
}
