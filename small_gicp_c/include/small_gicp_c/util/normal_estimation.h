#ifndef SMALL_GICP_C_UTIL_NORMAL_ESTIMATION_H
#define SMALL_GICP_C_UTIL_NORMAL_ESTIMATION_H

#include <small_gicp_c/types.h>

#ifdef __cplusplus
extern "C" {
#endif

// Normal estimation backend types
typedef enum {
  SMALL_GICP_NORMAL_ESTIMATION_BACKEND_DEFAULT = 0,
  SMALL_GICP_NORMAL_ESTIMATION_BACKEND_OPENMP = 1,
  SMALL_GICP_NORMAL_ESTIMATION_BACKEND_TBB = 2
} small_gicp_normal_estimation_backend_t;

// Normal estimation (legacy, automatic backend selection)
small_gicp_error_t
small_gicp_estimate_normals(small_gicp_point_cloud_t *cloud,
                            const small_gicp_kdtree_t *kdtree,
                            int num_neighbors, int num_threads);

// Normal estimation with specific backend
small_gicp_error_t small_gicp_estimate_normals_with_backend(
    small_gicp_point_cloud_t *cloud, const small_gicp_kdtree_t *kdtree,
    int num_neighbors, small_gicp_normal_estimation_backend_t backend,
    int num_threads);

// Covariance estimation (legacy, automatic backend selection)
small_gicp_error_t
small_gicp_estimate_covariances(small_gicp_point_cloud_t *cloud,
                                const small_gicp_kdtree_t *kdtree,
                                int num_neighbors, int num_threads);

// Covariance estimation with specific backend
small_gicp_error_t small_gicp_estimate_covariances_with_backend(
    small_gicp_point_cloud_t *cloud, const small_gicp_kdtree_t *kdtree,
    int num_neighbors, small_gicp_normal_estimation_backend_t backend,
    int num_threads);

// Combined normal and covariance estimation (legacy, automatic backend
// selection)
small_gicp_error_t
small_gicp_estimate_normals_covariances(small_gicp_point_cloud_t *cloud,
                                        const small_gicp_kdtree_t *kdtree,
                                        int num_neighbors, int num_threads);

// Combined normal and covariance estimation with specific backend
small_gicp_error_t small_gicp_estimate_normals_covariances_with_backend(
    small_gicp_point_cloud_t *cloud, const small_gicp_kdtree_t *kdtree,
    int num_neighbors, small_gicp_normal_estimation_backend_t backend,
    int num_threads);

#ifdef __cplusplus
}
#endif

#endif // SMALL_GICP_C_UTIL_NORMAL_ESTIMATION_H
