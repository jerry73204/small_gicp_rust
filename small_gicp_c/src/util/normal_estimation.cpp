#include "../common.h"
#include <small_gicp_c/util/normal_estimation.h>

#include <small_gicp/util/normal_estimation_omp.hpp>
#include <small_gicp/util/normal_estimation_tbb.hpp>

#define CHECK_NULL(ptr)                                                        \
  if (!(ptr))                                                                  \
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;

small_gicp_error_t
small_gicp_estimate_normals(small_gicp_point_cloud_t *cloud,
                            const small_gicp_kdtree_t *kdtree,
                            int num_neighbors, int num_threads) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(kdtree);
  CHECK_NULL(kdtree->tree);

  TRY_CATCH_BEGIN
  if (num_threads > 1) {
    small_gicp::estimate_normals_omp(*cloud->cloud, *kdtree->tree,
                                     num_neighbors, num_threads);
  } else {
    small_gicp::estimate_normals(*cloud->cloud, *kdtree->tree, num_neighbors);
  }
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_estimate_covariances(small_gicp_point_cloud_t *cloud,
                                const small_gicp_kdtree_t *kdtree,
                                int num_neighbors, int num_threads) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(kdtree);
  CHECK_NULL(kdtree->tree);

  TRY_CATCH_BEGIN
  if (num_threads > 1) {
    small_gicp::estimate_covariances_omp(*cloud->cloud, *kdtree->tree,
                                         num_neighbors, num_threads);
  } else {
    small_gicp::estimate_covariances(*cloud->cloud, *kdtree->tree,
                                     num_neighbors);
  }
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_estimate_normals_covariances(small_gicp_point_cloud_t *cloud,
                                        const small_gicp_kdtree_t *kdtree,
                                        int num_neighbors, int num_threads) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(kdtree);
  CHECK_NULL(kdtree->tree);

  TRY_CATCH_BEGIN
  if (num_threads > 1) {
    small_gicp::estimate_normals_covariances_omp(*cloud->cloud, *kdtree->tree,
                                                 num_neighbors, num_threads);
  } else {
    small_gicp::estimate_normals_covariances(*cloud->cloud, *kdtree->tree,
                                             num_neighbors);
  }
  TRY_CATCH_END
}

// Normal estimation with specific backend
small_gicp_error_t small_gicp_estimate_normals_with_backend(
    small_gicp_point_cloud_t *cloud, const small_gicp_kdtree_t *kdtree,
    int num_neighbors, small_gicp_normal_estimation_backend_t backend,
    int num_threads) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(kdtree);
  CHECK_NULL(kdtree->tree);

  TRY_CATCH_BEGIN
  switch (backend) {
  case SMALL_GICP_NORMAL_ESTIMATION_BACKEND_DEFAULT:
    small_gicp::estimate_normals(*cloud->cloud, *kdtree->tree, num_neighbors);
    break;
  case SMALL_GICP_NORMAL_ESTIMATION_BACKEND_OPENMP:
    if (num_threads <= 0)
      num_threads = 4;
    small_gicp::estimate_normals_omp(*cloud->cloud, *kdtree->tree,
                                     num_neighbors, num_threads);
    break;
  case SMALL_GICP_NORMAL_ESTIMATION_BACKEND_TBB:
    small_gicp::estimate_normals_tbb(*cloud->cloud, *kdtree->tree,
                                     num_neighbors);
    break;
  default:
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }
  TRY_CATCH_END
}

// Covariance estimation with specific backend
small_gicp_error_t small_gicp_estimate_covariances_with_backend(
    small_gicp_point_cloud_t *cloud, const small_gicp_kdtree_t *kdtree,
    int num_neighbors, small_gicp_normal_estimation_backend_t backend,
    int num_threads) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(kdtree);
  CHECK_NULL(kdtree->tree);

  TRY_CATCH_BEGIN
  switch (backend) {
  case SMALL_GICP_NORMAL_ESTIMATION_BACKEND_DEFAULT:
    small_gicp::estimate_covariances(*cloud->cloud, *kdtree->tree,
                                     num_neighbors);
    break;
  case SMALL_GICP_NORMAL_ESTIMATION_BACKEND_OPENMP:
    if (num_threads <= 0)
      num_threads = 4;
    small_gicp::estimate_covariances_omp(*cloud->cloud, *kdtree->tree,
                                         num_neighbors, num_threads);
    break;
  case SMALL_GICP_NORMAL_ESTIMATION_BACKEND_TBB:
    small_gicp::estimate_covariances_tbb(*cloud->cloud, *kdtree->tree,
                                         num_neighbors);
    break;
  default:
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }
  TRY_CATCH_END
}

// Combined normal and covariance estimation with specific backend
small_gicp_error_t small_gicp_estimate_normals_covariances_with_backend(
    small_gicp_point_cloud_t *cloud, const small_gicp_kdtree_t *kdtree,
    int num_neighbors, small_gicp_normal_estimation_backend_t backend,
    int num_threads) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(kdtree);
  CHECK_NULL(kdtree->tree);

  TRY_CATCH_BEGIN
  switch (backend) {
  case SMALL_GICP_NORMAL_ESTIMATION_BACKEND_DEFAULT:
    small_gicp::estimate_normals_covariances(*cloud->cloud, *kdtree->tree,
                                             num_neighbors);
    break;
  case SMALL_GICP_NORMAL_ESTIMATION_BACKEND_OPENMP:
    if (num_threads <= 0)
      num_threads = 4;
    small_gicp::estimate_normals_covariances_omp(*cloud->cloud, *kdtree->tree,
                                                 num_neighbors, num_threads);
    break;
  case SMALL_GICP_NORMAL_ESTIMATION_BACKEND_TBB:
    small_gicp::estimate_normals_covariances_tbb(*cloud->cloud, *kdtree->tree,
                                                 num_neighbors);
    break;
  default:
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }
  TRY_CATCH_END
}
