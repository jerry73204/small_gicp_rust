#include <small_gicp_c/points/point_cloud.h>
#include "../common.h"

#define CHECK_NULL(ptr)                                                        \
  if (!(ptr))                                                                  \
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;

// Point cloud creation and destruction
small_gicp_error_t
small_gicp_point_cloud_create(small_gicp_point_cloud_t **cloud) {
  CHECK_NULL(cloud);

  TRY_CATCH_BEGIN
  *cloud = new small_gicp_point_cloud;
  (*cloud)->cloud = std::make_shared<small_gicp::PointCloud>();
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_point_cloud_destroy(small_gicp_point_cloud_t *cloud) {
  if (cloud) {
    delete cloud;
  }
  return SMALL_GICP_SUCCESS;
}

// Point cloud data access
small_gicp_error_t
small_gicp_point_cloud_resize(small_gicp_point_cloud_t *cloud, size_t n) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);

  TRY_CATCH_BEGIN
  cloud->cloud->resize(n);
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_point_cloud_size(const small_gicp_point_cloud_t *cloud,
                            size_t *size) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(size);

  *size = cloud->cloud->size();
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t
small_gicp_point_cloud_set_point(small_gicp_point_cloud_t *cloud, size_t index,
                                 double x, double y, double z) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);

  if (index >= cloud->cloud->size()) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  cloud->cloud->point(index) = Eigen::Vector4d(x, y, z, 1.0);
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t
small_gicp_point_cloud_get_point(const small_gicp_point_cloud_t *cloud,
                                 size_t index, double *x, double *y,
                                 double *z) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(x);
  CHECK_NULL(y);
  CHECK_NULL(z);

  if (index >= cloud->cloud->size()) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  const auto &point = cloud->cloud->point(index);
  *x = point.x();
  *y = point.y();
  *z = point.z();
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t
small_gicp_point_cloud_set_normal(small_gicp_point_cloud_t *cloud, size_t index,
                                  double nx, double ny, double nz) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);

  if (index >= cloud->cloud->size()) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  cloud->cloud->normal(index) = Eigen::Vector4d(nx, ny, nz, 0.0);
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t
small_gicp_point_cloud_get_normal(const small_gicp_point_cloud_t *cloud,
                                  size_t index, double *nx, double *ny,
                                  double *nz) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(nx);
  CHECK_NULL(ny);
  CHECK_NULL(nz);

  if (index >= cloud->cloud->size()) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  const auto &normal = cloud->cloud->normal(index);
  *nx = normal.x();
  *ny = normal.y();
  *nz = normal.z();
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t
small_gicp_load_points_from_array(const float *points, size_t num_points,
                                  small_gicp_point_cloud_t **cloud) {
  CHECK_NULL(points);
  CHECK_NULL(cloud);

  TRY_CATCH_BEGIN
  *cloud = new small_gicp_point_cloud;
  (*cloud)->cloud = std::make_shared<small_gicp::PointCloud>();
  (*cloud)->cloud->resize(num_points);

  for (size_t i = 0; i < num_points; i++) {
    (*cloud)->cloud->point(i) = Eigen::Vector4d(
        static_cast<double>(points[i * 3 + 0]),
        static_cast<double>(points[i * 3 + 1]),
        static_cast<double>(points[i * 3 + 2]), 1.0);
  }
  TRY_CATCH_END
}