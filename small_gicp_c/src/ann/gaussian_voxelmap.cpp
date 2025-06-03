#include <small_gicp_c/ann/gaussian_voxelmap.h>
#include "../common.h"

#define CHECK_NULL(ptr)                                                        \
  if (!(ptr))                                                                  \
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;

small_gicp_error_t
small_gicp_gaussian_voxelmap_create(const small_gicp_point_cloud_t *cloud,
                                    double voxel_resolution, int num_threads,
                                    small_gicp_gaussian_voxelmap_t **voxelmap) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(voxelmap);

  TRY_CATCH_BEGIN
  *voxelmap = new small_gicp_gaussian_voxelmap;
  (*voxelmap)->voxelmap =
      small_gicp::create_gaussian_voxelmap(*cloud->cloud, voxel_resolution);
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_gaussian_voxelmap_destroy(small_gicp_gaussian_voxelmap_t *voxelmap) {
  if (voxelmap) {
    delete voxelmap;
  }
  return SMALL_GICP_SUCCESS;
}