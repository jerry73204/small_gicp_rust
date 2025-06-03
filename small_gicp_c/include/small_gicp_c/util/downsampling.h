#ifndef SMALL_GICP_C_UTIL_DOWNSAMPLING_H
#define SMALL_GICP_C_UTIL_DOWNSAMPLING_H

#include <small_gicp_c/types.h>

#ifdef __cplusplus
extern "C" {
#endif

// Voxel grid downsampling
small_gicp_error_t
small_gicp_voxelgrid_sampling(const small_gicp_point_cloud_t *cloud,
                              double leaf_size, int num_threads,
                              small_gicp_point_cloud_t **downsampled);

// Random downsampling
small_gicp_error_t
small_gicp_random_sampling(const small_gicp_point_cloud_t *cloud,
                           size_t num_samples,
                           small_gicp_point_cloud_t **downsampled);

#ifdef __cplusplus
}
#endif

#endif // SMALL_GICP_C_UTIL_DOWNSAMPLING_H