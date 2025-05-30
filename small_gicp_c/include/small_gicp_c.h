#ifndef SMALL_GICP_C_H
#define SMALL_GICP_C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>

// Error codes
typedef enum {
  SMALL_GICP_SUCCESS = 0,
  SMALL_GICP_ERROR_INVALID_ARGUMENT = -1,
  SMALL_GICP_ERROR_OUT_OF_MEMORY = -2,
  SMALL_GICP_ERROR_FILE_NOT_FOUND = -3,
  SMALL_GICP_ERROR_IO_ERROR = -4,
  SMALL_GICP_ERROR_EXCEPTION = -5
} small_gicp_error_t;

// Registration types
typedef enum {
  SMALL_GICP_ICP = 0,
  SMALL_GICP_PLANE_ICP = 1,
  SMALL_GICP_GICP = 2,
  SMALL_GICP_VGICP = 3
} small_gicp_registration_type_t;

// Opaque types
typedef struct small_gicp_point_cloud small_gicp_point_cloud_t;
typedef struct small_gicp_kdtree small_gicp_kdtree_t;
typedef struct small_gicp_gaussian_voxelmap small_gicp_gaussian_voxelmap_t;

// Registration result structure
typedef struct {
  double T_target_source[16]; // 4x4 transformation matrix in row-major order
  bool converged;
  int iterations;
  int num_inliers;
  double error;
} small_gicp_registration_result_t;

// Point cloud creation and destruction
small_gicp_error_t
small_gicp_point_cloud_create(small_gicp_point_cloud_t **cloud);
small_gicp_error_t
small_gicp_point_cloud_destroy(small_gicp_point_cloud_t *cloud);

// Point cloud data access
small_gicp_error_t
small_gicp_point_cloud_resize(small_gicp_point_cloud_t *cloud, size_t n);
small_gicp_error_t
small_gicp_point_cloud_size(const small_gicp_point_cloud_t *cloud,
                            size_t *size);
small_gicp_error_t
small_gicp_point_cloud_set_point(small_gicp_point_cloud_t *cloud, size_t index,
                                 double x, double y, double z);
small_gicp_error_t
small_gicp_point_cloud_get_point(const small_gicp_point_cloud_t *cloud,
                                 size_t index, double *x, double *y, double *z);
small_gicp_error_t
small_gicp_point_cloud_set_normal(small_gicp_point_cloud_t *cloud, size_t index,
                                  double nx, double ny, double nz);
small_gicp_error_t
small_gicp_point_cloud_get_normal(const small_gicp_point_cloud_t *cloud,
                                  size_t index, double *nx, double *ny,
                                  double *nz);

// Point cloud I/O
small_gicp_error_t small_gicp_load_ply(const char *filename,
                                       small_gicp_point_cloud_t **cloud);
small_gicp_error_t small_gicp_save_ply(const char *filename,
                                       const small_gicp_point_cloud_t *cloud);
small_gicp_error_t
small_gicp_load_points_from_array(const float *points, size_t num_points,
                                  small_gicp_point_cloud_t **cloud);

// Preprocessing functions
small_gicp_error_t
small_gicp_preprocess_points(const small_gicp_point_cloud_t *cloud,
                             double downsampling_resolution, int num_neighbors,
                             int num_threads,
                             small_gicp_point_cloud_t **preprocessed_cloud,
                             small_gicp_kdtree_t **kdtree);

// Downsampling
small_gicp_error_t
small_gicp_voxelgrid_sampling(const small_gicp_point_cloud_t *cloud,
                              double leaf_size, int num_threads,
                              small_gicp_point_cloud_t **downsampled);

small_gicp_error_t
small_gicp_random_sampling(const small_gicp_point_cloud_t *cloud,
                           size_t num_samples,
                           small_gicp_point_cloud_t **downsampled);

// Normal and covariance estimation
small_gicp_error_t
small_gicp_estimate_normals(small_gicp_point_cloud_t *cloud,
                            const small_gicp_kdtree_t *kdtree,
                            int num_neighbors, int num_threads);

small_gicp_error_t
small_gicp_estimate_covariances(small_gicp_point_cloud_t *cloud,
                                const small_gicp_kdtree_t *kdtree,
                                int num_neighbors, int num_threads);

small_gicp_error_t
small_gicp_estimate_normals_covariances(small_gicp_point_cloud_t *cloud,
                                        const small_gicp_kdtree_t *kdtree,
                                        int num_neighbors, int num_threads);

// KdTree creation and search
small_gicp_error_t
small_gicp_kdtree_create(const small_gicp_point_cloud_t *cloud, int num_threads,
                         small_gicp_kdtree_t **kdtree);
small_gicp_error_t small_gicp_kdtree_destroy(small_gicp_kdtree_t *kdtree);
small_gicp_error_t
small_gicp_kdtree_nearest_neighbor_search(const small_gicp_kdtree_t *kdtree,
                                          double x, double y, double z,
                                          size_t *index, double *sq_dist);
small_gicp_error_t
small_gicp_kdtree_knn_search(const small_gicp_kdtree_t *kdtree, double x,
                             double y, double z, int k, size_t *indices,
                             double *sq_dists);

// Gaussian voxel map for VGICP
small_gicp_error_t
small_gicp_gaussian_voxelmap_create(const small_gicp_point_cloud_t *cloud,
                                    double voxel_resolution, int num_threads,
                                    small_gicp_gaussian_voxelmap_t **voxelmap);
small_gicp_error_t
small_gicp_gaussian_voxelmap_destroy(small_gicp_gaussian_voxelmap_t *voxelmap);

// Registration functions
small_gicp_error_t small_gicp_align(
    const small_gicp_point_cloud_t *target,
    const small_gicp_point_cloud_t *source, small_gicp_registration_type_t type,
    const double
        *initial_guess, // 4x4 matrix in row-major order, NULL for identity
    int num_threads, small_gicp_registration_result_t *result);

small_gicp_error_t
small_gicp_align_preprocessed(const small_gicp_point_cloud_t *target,
                              const small_gicp_point_cloud_t *source,
                              const small_gicp_kdtree_t *target_tree,
                              small_gicp_registration_type_t type,
                              const double *initial_guess, int num_threads,
                              small_gicp_registration_result_t *result);

small_gicp_error_t
small_gicp_align_vgicp(const small_gicp_gaussian_voxelmap_t *target_voxelmap,
                       const small_gicp_point_cloud_t *source,
                       const double *initial_guess, int num_threads,
                       small_gicp_registration_result_t *result);

// Utility functions
small_gicp_error_t small_gicp_get_version(char *buffer, size_t buffer_size);
const char *small_gicp_error_string(small_gicp_error_t error);

#ifdef __cplusplus
}
#endif

#endif // SMALL_GICP_C_H
