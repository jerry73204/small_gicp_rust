#ifndef SMALL_GICP_C_ANN_GAUSSIAN_VOXELMAP_H
#define SMALL_GICP_C_ANN_GAUSSIAN_VOXELMAP_H

#include <small_gicp_c/types.h>

#ifdef __cplusplus
extern "C" {
#endif

// Gaussian voxel map creation and destruction
small_gicp_error_t
small_gicp_gaussian_voxelmap_create(const small_gicp_point_cloud_t *cloud,
                                    double voxel_resolution, int num_threads,
                                    small_gicp_gaussian_voxelmap_t **voxelmap);

small_gicp_error_t
small_gicp_gaussian_voxelmap_destroy(small_gicp_gaussian_voxelmap_t *voxelmap);

#ifdef __cplusplus
}
#endif

#endif // SMALL_GICP_C_ANN_GAUSSIAN_VOXELMAP_H