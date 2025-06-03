#include <small_gicp_c/util/downsampling.h>
#include "../common.h"

#define CHECK_NULL(ptr)                                                        \
  if (!(ptr))                                                                  \
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;

small_gicp_error_t
small_gicp_voxelgrid_sampling(const small_gicp_point_cloud_t *cloud,
                              double leaf_size, int num_threads,
                              small_gicp_point_cloud_t **downsampled) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(downsampled);

  TRY_CATCH_BEGIN
  auto result = (num_threads > 1)
                    ? small_gicp::voxelgrid_sampling_omp(*cloud->cloud,
                                                         leaf_size, num_threads)
                    : small_gicp::voxelgrid_sampling(*cloud->cloud, leaf_size);

  *downsampled = new small_gicp_point_cloud;
  (*downsampled)->cloud = result;
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_random_sampling(const small_gicp_point_cloud_t *cloud,
                           size_t num_samples,
                           small_gicp_point_cloud_t **downsampled) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(downsampled);

  TRY_CATCH_BEGIN
  std::mt19937 rng(42); // Fixed seed for reproducibility
  auto result = small_gicp::random_sampling(*cloud->cloud, num_samples, rng);

  *downsampled = new small_gicp_point_cloud;
  (*downsampled)->cloud = result;
  TRY_CATCH_END
}