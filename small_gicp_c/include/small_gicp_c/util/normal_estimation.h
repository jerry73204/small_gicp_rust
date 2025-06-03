#ifndef SMALL_GICP_C_UTIL_NORMAL_ESTIMATION_H
#define SMALL_GICP_C_UTIL_NORMAL_ESTIMATION_H

#include <small_gicp_c/types.h>

#ifdef __cplusplus
extern "C" {
#endif

// Normal estimation
small_gicp_error_t
small_gicp_estimate_normals(small_gicp_point_cloud_t *cloud,
                            const small_gicp_kdtree_t *kdtree,
                            int num_neighbors, int num_threads);

// Covariance estimation
small_gicp_error_t
small_gicp_estimate_covariances(small_gicp_point_cloud_t *cloud,
                                const small_gicp_kdtree_t *kdtree,
                                int num_neighbors, int num_threads);

// Combined normal and covariance estimation
small_gicp_error_t
small_gicp_estimate_normals_covariances(small_gicp_point_cloud_t *cloud,
                                        const small_gicp_kdtree_t *kdtree,
                                        int num_neighbors, int num_threads);

#ifdef __cplusplus
}
#endif

#endif // SMALL_GICP_C_UTIL_NORMAL_ESTIMATION_H