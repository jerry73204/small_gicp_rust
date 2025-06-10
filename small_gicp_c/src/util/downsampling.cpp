#include "../common.h"
#include <small_gicp_c/util/downsampling.h>

#include <random>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/downsampling_tbb.hpp>

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

// Voxel grid downsampling with specific backend
small_gicp_error_t small_gicp_voxelgrid_sampling_with_backend(
    const small_gicp_point_cloud_t *cloud, double leaf_size,
    small_gicp_downsampling_backend_t backend, int num_threads,
    small_gicp_point_cloud_t **downsampled) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(downsampled);

  TRY_CATCH_BEGIN
  std::shared_ptr<small_gicp::PointCloud> result;

  switch (backend) {
  case SMALL_GICP_DOWNSAMPLING_BACKEND_DEFAULT:
    result = small_gicp::voxelgrid_sampling(*cloud->cloud, leaf_size);
    break;
  case SMALL_GICP_DOWNSAMPLING_BACKEND_OPENMP:
    if (num_threads <= 0)
      num_threads = 4;
    result = small_gicp::voxelgrid_sampling_omp(*cloud->cloud, leaf_size,
                                                num_threads);
    break;
  case SMALL_GICP_DOWNSAMPLING_BACKEND_TBB:
    result = small_gicp::voxelgrid_sampling_tbb(*cloud->cloud, leaf_size);
    break;
  default:
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  *downsampled = new small_gicp_point_cloud;
  (*downsampled)->cloud = result;
  TRY_CATCH_END
}

// Random downsampling with seed for reproducibility
small_gicp_error_t
small_gicp_random_sampling_with_seed(const small_gicp_point_cloud_t *cloud,
                                     size_t num_samples, unsigned int seed,
                                     small_gicp_point_cloud_t **downsampled) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(downsampled);

  TRY_CATCH_BEGIN
  std::mt19937 rng(seed);
  auto result = small_gicp::random_sampling(*cloud->cloud, num_samples, rng);

  *downsampled = new small_gicp_point_cloud;
  (*downsampled)->cloud = result;
  TRY_CATCH_END
}
