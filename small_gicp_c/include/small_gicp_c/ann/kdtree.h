#ifndef SMALL_GICP_C_ANN_KDTREE_H
#define SMALL_GICP_C_ANN_KDTREE_H

#include <small_gicp_c/types.h>

#ifdef __cplusplus
extern "C" {
#endif

// KdTree creation and destruction
small_gicp_error_t
small_gicp_kdtree_create(const small_gicp_point_cloud_t *cloud, int num_threads,
                         small_gicp_kdtree_t **kdtree);

small_gicp_error_t 
small_gicp_kdtree_destroy(small_gicp_kdtree_t *kdtree);

// KdTree search operations
small_gicp_error_t
small_gicp_kdtree_nearest_neighbor_search(const small_gicp_kdtree_t *kdtree,
                                          double x, double y, double z,
                                          size_t *index, double *sq_dist);

small_gicp_error_t
small_gicp_kdtree_knn_search(const small_gicp_kdtree_t *kdtree, double x,
                             double y, double z, int k, size_t *indices,
                             double *sq_dists);

#ifdef __cplusplus
}
#endif

#endif // SMALL_GICP_C_ANN_KDTREE_H