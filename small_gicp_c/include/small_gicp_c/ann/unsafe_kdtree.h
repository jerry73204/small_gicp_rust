#ifndef SMALL_GICP_C_ANN_UNSAFE_KDTREE_H
#define SMALL_GICP_C_ANN_UNSAFE_KDTREE_H

#include <small_gicp_c/ann/kdtree.h>
#include <small_gicp_c/points/point_cloud.h>
#include <small_gicp_c/types.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations
typedef struct small_gicp_unsafe_kdtree small_gicp_unsafe_kdtree_t;

// Projection types for KdTree construction
typedef enum {
  SMALL_GICP_PROJECTION_AXIS_ALIGNED = 0, // Split along X/Y/Z axis (fastest)
  SMALL_GICP_PROJECTION_NORMAL =
      1 // Split along normal direction (more accurate)
} small_gicp_projection_type_t;

// KNN search settings for early termination
typedef struct {
  double epsilon; // Early termination threshold (0.0 = exact search)
} small_gicp_knn_setting_t;

// Projection configuration
typedef struct {
  small_gicp_projection_type_t type; // Projection type
  int max_scan_count; // Maximum points to scan for axis selection (default:
                      // 128)
} small_gicp_projection_config_t;

// Extended KdTree builder configuration
typedef struct {
  small_gicp_kdtree_builder_type_t builder_type; // OpenMP, TBB, or default
  int num_threads;                               // Number of threads
  int max_leaf_size;                             // Maximum points per leaf node
  small_gicp_projection_config_t projection;     // Projection configuration
} small_gicp_kdtree_config_extended_t;

// Create UnsafeKdTree (caller must ensure point cloud lifetime exceeds KdTree
// lifetime)
small_gicp_error_t
small_gicp_unsafe_kdtree_create(const small_gicp_point_cloud_t *cloud,
                                small_gicp_unsafe_kdtree_t **kdtree);

// Create UnsafeKdTree with basic configuration
small_gicp_error_t small_gicp_unsafe_kdtree_create_with_config(
    const small_gicp_point_cloud_t *cloud,
    const small_gicp_kdtree_config_t *config,
    small_gicp_unsafe_kdtree_t **kdtree);

// Create UnsafeKdTree with extended configuration
small_gicp_error_t small_gicp_unsafe_kdtree_create_with_extended_config(
    const small_gicp_point_cloud_t *cloud,
    const small_gicp_kdtree_config_extended_t *config,
    small_gicp_unsafe_kdtree_t **kdtree);

// Destroy UnsafeKdTree
small_gicp_error_t
small_gicp_unsafe_kdtree_destroy(small_gicp_unsafe_kdtree_t *kdtree);

// Nearest neighbor search
small_gicp_error_t small_gicp_unsafe_kdtree_nearest_neighbor_search(
    const small_gicp_unsafe_kdtree_t *kdtree, double query_x, double query_y,
    double query_z, size_t *index, double *distance);

// Nearest neighbor search with custom settings
small_gicp_error_t
small_gicp_unsafe_kdtree_nearest_neighbor_search_with_setting(
    const small_gicp_unsafe_kdtree_t *kdtree, double query_x, double query_y,
    double query_z, const small_gicp_knn_setting_t *setting, size_t *index,
    double *distance);

// K-nearest neighbor search
small_gicp_error_t small_gicp_unsafe_kdtree_knn_search(
    const small_gicp_unsafe_kdtree_t *kdtree, double query_x, double query_y,
    double query_z, int k, size_t *indices, double *distances);

// K-nearest neighbor search with custom settings
small_gicp_error_t small_gicp_unsafe_kdtree_knn_search_with_setting(
    const small_gicp_unsafe_kdtree_t *kdtree, double query_x, double query_y,
    double query_z, int k, const small_gicp_knn_setting_t *setting,
    size_t *indices, double *distances);

// Create default KNN setting
small_gicp_error_t
small_gicp_create_default_knn_setting(small_gicp_knn_setting_t *setting);

// Create default projection configuration
small_gicp_error_t small_gicp_create_default_projection_config(
    small_gicp_projection_config_t *config);

// Create default extended KdTree configuration
small_gicp_error_t small_gicp_create_default_kdtree_config_extended(
    small_gicp_kdtree_config_extended_t *config);

// Update existing KdTree functions to support KNN settings

// Enhanced versions of existing KdTree functions with KNN settings
small_gicp_error_t small_gicp_kdtree_nearest_neighbor_search_with_setting(
    const small_gicp_kdtree_t *kdtree, double query_x, double query_y,
    double query_z, const small_gicp_knn_setting_t *setting, size_t *index,
    double *distance);

small_gicp_error_t small_gicp_kdtree_knn_search_with_setting(
    const small_gicp_kdtree_t *kdtree, double query_x, double query_y,
    double query_z, int k, const small_gicp_knn_setting_t *setting,
    size_t *indices, double *distances);

// Create KdTree with extended configuration (enhanced version of existing
// function)
small_gicp_error_t small_gicp_kdtree_create_with_extended_config(
    const small_gicp_point_cloud_t *cloud,
    const small_gicp_kdtree_config_extended_t *config,
    small_gicp_kdtree_t **kdtree);

#ifdef __cplusplus
}
#endif

#endif // SMALL_GICP_C_ANN_UNSAFE_KDTREE_H
