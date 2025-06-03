#ifndef SMALL_GICP_C_REGISTRATION_REGISTRATION_H
#define SMALL_GICP_C_REGISTRATION_REGISTRATION_H

#include <small_gicp_c/types.h>

#ifdef __cplusplus
extern "C" {
#endif

// Complete preprocessing pipeline
small_gicp_error_t
small_gicp_preprocess_points(const small_gicp_point_cloud_t *cloud,
                             double downsampling_resolution, int num_neighbors,
                             int num_threads,
                             small_gicp_point_cloud_t **preprocessed_cloud,
                             small_gicp_kdtree_t **kdtree);

// Basic registration with automatic preprocessing
small_gicp_error_t small_gicp_align(
    const small_gicp_point_cloud_t *target,
    const small_gicp_point_cloud_t *source, 
    small_gicp_registration_type_t type,
    const double *initial_guess, // 4x4 matrix in row-major order, NULL for identity
    int num_threads, 
    small_gicp_registration_result_t *result);

// Registration with preprocessed point clouds
small_gicp_error_t
small_gicp_align_preprocessed(const small_gicp_point_cloud_t *target,
                              const small_gicp_point_cloud_t *source,
                              const small_gicp_kdtree_t *target_tree,
                              small_gicp_registration_type_t type,
                              const double *initial_guess, int num_threads,
                              small_gicp_registration_result_t *result);

// VGICP registration
small_gicp_error_t
small_gicp_align_vgicp(const small_gicp_gaussian_voxelmap_t *target_voxelmap,
                       const small_gicp_point_cloud_t *source,
                       const double *initial_guess, int num_threads,
                       small_gicp_registration_result_t *result);

#ifdef __cplusplus
}
#endif

#endif // SMALL_GICP_C_REGISTRATION_REGISTRATION_H