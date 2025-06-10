#ifndef SMALL_GICP_C_UTIL_DOWNSAMPLING_H
#define SMALL_GICP_C_UTIL_DOWNSAMPLING_H

#include <small_gicp_c/types.h>

#ifdef __cplusplus
extern "C" {
#endif

// Downsampling backend types
typedef enum {
  SMALL_GICP_DOWNSAMPLING_BACKEND_DEFAULT = 0,
  SMALL_GICP_DOWNSAMPLING_BACKEND_OPENMP = 1,
  SMALL_GICP_DOWNSAMPLING_BACKEND_TBB = 2
} small_gicp_downsampling_backend_t;

// Voxel grid downsampling (legacy, uses default backend)
small_gicp_error_t
small_gicp_voxelgrid_sampling(const small_gicp_point_cloud_t *cloud,
                              double leaf_size, int num_threads,
                              small_gicp_point_cloud_t **downsampled);

// Voxel grid downsampling with specific backend
small_gicp_error_t small_gicp_voxelgrid_sampling_with_backend(
    const small_gicp_point_cloud_t *cloud, double leaf_size,
    small_gicp_downsampling_backend_t backend, int num_threads,
    small_gicp_point_cloud_t **downsampled);

// Random downsampling
small_gicp_error_t
small_gicp_random_sampling(const small_gicp_point_cloud_t *cloud,
                           size_t num_samples,
                           small_gicp_point_cloud_t **downsampled);

// Random downsampling with seed for reproducibility
small_gicp_error_t
small_gicp_random_sampling_with_seed(const small_gicp_point_cloud_t *cloud,
                                     size_t num_samples, unsigned int seed,
                                     small_gicp_point_cloud_t **downsampled);

#ifdef __cplusplus
}
#endif

#endif // SMALL_GICP_C_UTIL_DOWNSAMPLING_H
