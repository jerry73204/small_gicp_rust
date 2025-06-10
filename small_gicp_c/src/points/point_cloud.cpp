#include "../common.h"
#include <small_gicp_c/points/point_cloud.h>

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

// Covariance operations
small_gicp_error_t
small_gicp_point_cloud_set_covariance(small_gicp_point_cloud_t *cloud,
                                      size_t index, const double *cov_matrix) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(cov_matrix);

  if (index >= cloud->cloud->size()) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  TRY_CATCH_BEGIN
  // Convert from row-major order to Eigen matrix
  Eigen::Matrix4d &cov = cloud->cloud->cov(index);
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      cov(i, j) = cov_matrix[i * 4 + j];
    }
  }
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_point_cloud_get_covariance(const small_gicp_point_cloud_t *cloud,
                                      size_t index, double *cov_matrix) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(cov_matrix);

  if (index >= cloud->cloud->size()) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  TRY_CATCH_BEGIN
  const Eigen::Matrix4d &cov = cloud->cloud->cov(index);
  // Convert to row-major order
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      cov_matrix[i * 4 + j] = cov(i, j);
    }
  }
  TRY_CATCH_END
}

// Check if point cloud has data
small_gicp_error_t
small_gicp_point_cloud_has_points(const small_gicp_point_cloud_t *cloud,
                                  bool *has_points) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(has_points);

  *has_points = small_gicp::traits::has_points(*cloud->cloud);
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t
small_gicp_point_cloud_has_normals(const small_gicp_point_cloud_t *cloud,
                                   bool *has_normals) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(has_normals);

  *has_normals = small_gicp::traits::has_normals(*cloud->cloud);
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t
small_gicp_point_cloud_has_covariances(const small_gicp_point_cloud_t *cloud,
                                       bool *has_covariances) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(has_covariances);

  *has_covariances = small_gicp::traits::has_covs(*cloud->cloud);
  return SMALL_GICP_SUCCESS;
}

small_gicp_error_t
small_gicp_point_cloud_empty(const small_gicp_point_cloud_t *cloud,
                             bool *is_empty) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(is_empty);

  *is_empty = cloud->cloud->empty();
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
    (*cloud)->cloud->point(i) =
        Eigen::Vector4d(static_cast<double>(points[i * 3 + 0]),
                        static_cast<double>(points[i * 3 + 1]),
                        static_cast<double>(points[i * 3 + 2]), 1.0);
  }
  TRY_CATCH_END
}

// Direct access to internal vectors (bulk operations)
small_gicp_error_t
small_gicp_point_cloud_get_points_data(const small_gicp_point_cloud_t *cloud,
                                       double **points_data,
                                       size_t *data_size) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(points_data);
  CHECK_NULL(data_size);

  TRY_CATCH_BEGIN
  const auto &points = cloud->cloud->points;
  if (points.empty()) {
    *points_data = nullptr;
    *data_size = 0;
  } else {
    // Return pointer to first element data (4 doubles per point: x, y, z, w)
    *points_data = const_cast<double *>(points[0].data());
    *data_size = points.size() * 4; // 4 components per point
  }
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_point_cloud_get_normals_data(const small_gicp_point_cloud_t *cloud,
                                        double **normals_data,
                                        size_t *data_size) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(normals_data);
  CHECK_NULL(data_size);

  TRY_CATCH_BEGIN
  const auto &normals = cloud->cloud->normals;
  if (normals.empty()) {
    *normals_data = nullptr;
    *data_size = 0;
  } else {
    // Return pointer to first element data (4 doubles per normal: nx, ny, nz,
    // 0)
    *normals_data = const_cast<double *>(normals[0].data());
    *data_size = normals.size() * 4; // 4 components per normal
  }
  TRY_CATCH_END
}

small_gicp_error_t small_gicp_point_cloud_get_covariances_data(
    const small_gicp_point_cloud_t *cloud, double **covariances_data,
    size_t *data_size) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(covariances_data);
  CHECK_NULL(data_size);

  TRY_CATCH_BEGIN
  const auto &covariances = cloud->cloud->covs;
  if (covariances.empty()) {
    *covariances_data = nullptr;
    *data_size = 0;
  } else {
    // Return pointer to first element data (16 doubles per covariance: 4x4
    // matrix)
    *covariances_data = const_cast<double *>(covariances[0].data());
    *data_size = covariances.size() * 16; // 16 components per covariance matrix
  }
  TRY_CATCH_END
}

// Bulk point operations
small_gicp_error_t
small_gicp_point_cloud_set_points_bulk(small_gicp_point_cloud_t *cloud,
                                       const double *points_data,
                                       size_t num_points) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(points_data);

  TRY_CATCH_BEGIN
  cloud->cloud->resize(num_points);

  for (size_t i = 0; i < num_points; i++) {
    // Input format: x, y, z per point (3 components)
    cloud->cloud->point(i) =
        Eigen::Vector4d(points_data[i * 3 + 0], points_data[i * 3 + 1],
                        points_data[i * 3 + 2], 1.0);
  }
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_point_cloud_set_normals_bulk(small_gicp_point_cloud_t *cloud,
                                        const double *normals_data,
                                        size_t num_points) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(normals_data);

  if (num_points != cloud->cloud->size()) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  TRY_CATCH_BEGIN
  cloud->cloud->normals.resize(num_points);

  for (size_t i = 0; i < num_points; i++) {
    // Input format: nx, ny, nz per normal (3 components)
    cloud->cloud->normal(i) =
        Eigen::Vector4d(normals_data[i * 3 + 0], normals_data[i * 3 + 1],
                        normals_data[i * 3 + 2], 0.0);
  }
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_point_cloud_set_covariances_bulk(small_gicp_point_cloud_t *cloud,
                                            const double *covariances_data,
                                            size_t num_points) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(covariances_data);

  if (num_points != cloud->cloud->size()) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  TRY_CATCH_BEGIN
  cloud->cloud->covs.resize(num_points);

  for (size_t i = 0; i < num_points; i++) {
    // Input format: 4x4 matrix in row-major order (16 components per
    // covariance)
    Eigen::Matrix4d &cov = cloud->cloud->cov(i);
    for (int row = 0; row < 4; row++) {
      for (int col = 0; col < 4; col++) {
        cov(row, col) = covariances_data[i * 16 + row * 4 + col];
      }
    }
  }
  TRY_CATCH_END
}

// Copy data from internal vectors to user-provided arrays
small_gicp_error_t small_gicp_point_cloud_copy_points_to_array(
    const small_gicp_point_cloud_t *cloud, double *points_array,
    size_t array_size) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(points_array);

  size_t required_size = cloud->cloud->size() * 3; // x, y, z per point
  if (array_size < required_size) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  TRY_CATCH_BEGIN
  for (size_t i = 0; i < cloud->cloud->size(); i++) {
    const auto &point = cloud->cloud->point(i);
    points_array[i * 3 + 0] = point.x();
    points_array[i * 3 + 1] = point.y();
    points_array[i * 3 + 2] = point.z();
  }
  TRY_CATCH_END
}

small_gicp_error_t small_gicp_point_cloud_copy_normals_to_array(
    const small_gicp_point_cloud_t *cloud, double *normals_array,
    size_t array_size) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(normals_array);

  size_t required_size =
      cloud->cloud->normals.size() * 3; // nx, ny, nz per normal
  if (array_size < required_size) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  TRY_CATCH_BEGIN
  for (size_t i = 0; i < cloud->cloud->normals.size(); i++) {
    const auto &normal = cloud->cloud->normal(i);
    normals_array[i * 3 + 0] = normal.x();
    normals_array[i * 3 + 1] = normal.y();
    normals_array[i * 3 + 2] = normal.z();
  }
  TRY_CATCH_END
}

small_gicp_error_t small_gicp_point_cloud_copy_covariances_to_array(
    const small_gicp_point_cloud_t *cloud, double *covariances_array,
    size_t array_size) {
  CHECK_NULL(cloud);
  CHECK_NULL(cloud->cloud);
  CHECK_NULL(covariances_array);

  size_t required_size =
      cloud->cloud->covs.size() * 16; // 4x4 matrix per covariance
  if (array_size < required_size) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  TRY_CATCH_BEGIN
  for (size_t i = 0; i < cloud->cloud->covs.size(); i++) {
    const Eigen::Matrix4d &cov = cloud->cloud->cov(i);
    // Copy to row-major order
    for (int row = 0; row < 4; row++) {
      for (int col = 0; col < 4; col++) {
        covariances_array[i * 16 + row * 4 + col] = cov(row, col);
      }
    }
  }
  TRY_CATCH_END
}
