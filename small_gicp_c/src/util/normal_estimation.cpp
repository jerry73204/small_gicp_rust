#include "../common.h"
#include <small_gicp_c/util/normal_estimation.h>

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
