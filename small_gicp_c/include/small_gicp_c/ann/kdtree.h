#ifndef SMALL_GICP_C_ANN_KDTREE_H
#define SMALL_GICP_C_ANN_KDTREE_H

#include <small_gicp_c/types.h>

#ifdef __cplusplus
extern "C" {
#endif

// KdTree builder types
typedef enum {
  SMALL_GICP_KDTREE_BUILDER_DEFAULT = 0,
  SMALL_GICP_KDTREE_BUILDER_OPENMP = 1,
  SMALL_GICP_KDTREE_BUILDER_TBB = 2
} small_gicp_kdtree_builder_type_t;

// KdTree builder configuration
typedef struct {
  small_gicp_kdtree_builder_type_t builder_type;
  int num_threads;      // For OpenMP builder
  int max_leaf_size;    // Maximum number of points in leaf nodes
} small_gicp_kdtree_config_t;

// KdTree creation and destruction
small_gicp_error_t
small_gicp_kdtree_create(const small_gicp_point_cloud_t *cloud, int num_threads,
                         small_gicp_kdtree_t **kdtree);

// KdTree creation with specific builder type
small_gicp_error_t
small_gicp_kdtree_create_with_builder(const small_gicp_point_cloud_t *cloud,
                                      small_gicp_kdtree_builder_type_t builder_type,
                                      int num_threads,
                                      small_gicp_kdtree_t **kdtree);

// KdTree creation with full configuration
small_gicp_error_t
small_gicp_kdtree_create_with_config(const small_gicp_point_cloud_t *cloud,
                                     const small_gicp_kdtree_config_t *config,
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

// KdTree configuration helpers
small_gicp_error_t
small_gicp_kdtree_config_create_default(small_gicp_kdtree_config_t **config);

small_gicp_error_t
small_gicp_kdtree_config_create(small_gicp_kdtree_builder_type_t builder_type,
                               int num_threads, int max_leaf_size,
                               small_gicp_kdtree_config_t **config);

small_gicp_error_t
small_gicp_kdtree_config_destroy(small_gicp_kdtree_config_t *config);

#ifdef __cplusplus
}
#endif

#endif // SMALL_GICP_C_ANN_KDTREE_H