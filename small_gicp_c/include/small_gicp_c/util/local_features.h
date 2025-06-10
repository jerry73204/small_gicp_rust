#ifndef SMALL_GICP_C_UTIL_LOCAL_FEATURES_H
#define SMALL_GICP_C_UTIL_LOCAL_FEATURES_H

#include <small_gicp_c/types.h>

#ifdef __cplusplus
extern "C" {
#endif

// Local feature estimation setter types
typedef enum {
  SMALL_GICP_SETTER_NORMAL = 0,
  SMALL_GICP_SETTER_COVARIANCE = 1,
  SMALL_GICP_SETTER_NORMAL_COVARIANCE = 2
} small_gicp_setter_type_t;

// Local feature estimation backends
typedef enum {
  SMALL_GICP_LOCAL_FEATURES_BACKEND_DEFAULT = 0,
  SMALL_GICP_LOCAL_FEATURES_BACKEND_OPENMP = 1,
  SMALL_GICP_LOCAL_FEATURES_BACKEND_TBB = 2
} small_gicp_local_features_backend_t;

// Estimate local features for a single point
small_gicp_error_t small_gicp_estimate_local_features_single_point(
    small_gicp_point_cloud_t *cloud, const small_gicp_kdtree_t *kdtree,
    size_t point_index, int num_neighbors,
    small_gicp_setter_type_t setter_type);

// Estimate local features for entire point cloud
small_gicp_error_t small_gicp_estimate_local_features_cloud(
    small_gicp_point_cloud_t *cloud, const small_gicp_kdtree_t *kdtree,
    int num_neighbors, small_gicp_setter_type_t setter_type,
    small_gicp_local_features_backend_t backend, int num_threads);

// Estimate local features without external kdtree
small_gicp_error_t small_gicp_estimate_local_features_auto(
    small_gicp_point_cloud_t *cloud, int num_neighbors,
    small_gicp_setter_type_t setter_type,
    small_gicp_local_features_backend_t backend, int num_threads);

// Direct setter interface - set normal for a point given eigenvectors
small_gicp_error_t small_gicp_normal_setter_set(
    small_gicp_point_cloud_t *cloud, size_t point_index,
    const double *eigenvectors); // 3x3 matrix in row-major order

// Direct setter interface - set covariance for a point given eigenvectors
small_gicp_error_t small_gicp_covariance_setter_set(
    small_gicp_point_cloud_t *cloud, size_t point_index,
    const double *eigenvectors); // 3x3 matrix in row-major order

// Direct setter interface - set normal and covariance for a point given
// eigenvectors
small_gicp_error_t small_gicp_normal_covariance_setter_set(
    small_gicp_point_cloud_t *cloud, size_t point_index,
    const double *eigenvectors); // 3x3 matrix in row-major order

// Direct setter interface - set invalid normal
small_gicp_error_t
small_gicp_normal_setter_set_invalid(small_gicp_point_cloud_t *cloud,
                                     size_t point_index);

// Direct setter interface - set invalid covariance
small_gicp_error_t
small_gicp_covariance_setter_set_invalid(small_gicp_point_cloud_t *cloud,
                                         size_t point_index);

// Direct setter interface - set invalid normal and covariance
small_gicp_error_t
small_gicp_normal_covariance_setter_set_invalid(small_gicp_point_cloud_t *cloud,
                                                size_t point_index);

#ifdef __cplusplus
}
#endif

#endif // SMALL_GICP_C_UTIL_LOCAL_FEATURES_H
