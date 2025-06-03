#include <small_gicp_c/ann/kdtree.h>
#include "../common.h"

#define CHECK_NULL(ptr)                                                        \
  if (!(ptr))                                                                  \
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;

small_gicp_error_t
small_gicp_kdtree_create(const small_gicp_point_cloud_t *cloud, int num_threads,
                         small_gicp_kdtree_t **kdtree) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(kdtree);

  TRY_CATCH_BEGIN
  *kdtree = new small_gicp_kdtree;

  if (num_threads > 1) {
    // Use the parallel KdTree builder for multi-threaded construction
    small_gicp::KdTreeBuilderOMP builder(num_threads);
    (*kdtree)->tree =
        std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(
            cloud->cloud, builder);
  } else {
    // Use default single-threaded builder
    (*kdtree)->tree =
        std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(
            cloud->cloud);
  }
  TRY_CATCH_END
}

small_gicp_error_t small_gicp_kdtree_destroy(small_gicp_kdtree_t *kdtree) {
  if (kdtree) {
    delete kdtree;
  }
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t
small_gicp_kdtree_nearest_neighbor_search(const small_gicp_kdtree_t *kdtree,
                                          double x, double y, double z,
                                          size_t *index, double *sq_dist) {
  CHECK_NULL(kdtree);
  CHECK_NULL(kdtree->tree);
  CHECK_NULL(index);
  CHECK_NULL(sq_dist);

  TRY_CATCH_BEGIN
  Eigen::Vector4d query(x, y, z, 1.0);
  size_t num_found =
      kdtree->tree->nearest_neighbor_search(query, index, sq_dist);

  if (num_found == 0) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_kdtree_knn_search(const small_gicp_kdtree_t *kdtree, double x,
                             double y, double z, int k, size_t *indices,
                             double *sq_dists) {
  CHECK_NULL(kdtree);
  CHECK_NULL(kdtree->tree);
  CHECK_NULL(indices);
  CHECK_NULL(sq_dists);

  TRY_CATCH_BEGIN
  Eigen::Vector4d query(x, y, z, 1.0);
  size_t num_found = kdtree->tree->knn_search(query, k, indices, sq_dists);

  if (num_found == 0) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }
  TRY_CATCH_END
}