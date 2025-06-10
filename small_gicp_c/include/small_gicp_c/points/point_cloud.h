#ifndef SMALL_GICP_C_POINTS_POINT_CLOUD_H
#define SMALL_GICP_C_POINTS_POINT_CLOUD_H

#include <small_gicp_c/types.h>

#ifdef __cplusplus
extern "C" {
#endif

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

// Covariance operations (4x4 matrix in row-major order)
small_gicp_error_t
small_gicp_point_cloud_set_covariance(small_gicp_point_cloud_t *cloud,
                                      size_t index, const double *cov_matrix);

small_gicp_error_t
small_gicp_point_cloud_get_covariance(const small_gicp_point_cloud_t *cloud,
                                      size_t index, double *cov_matrix);

// Check if point cloud has data
small_gicp_error_t
small_gicp_point_cloud_has_points(const small_gicp_point_cloud_t *cloud,
                                  bool *has_points);

small_gicp_error_t
small_gicp_point_cloud_has_normals(const small_gicp_point_cloud_t *cloud,
                                   bool *has_normals);

small_gicp_error_t
small_gicp_point_cloud_has_covariances(const small_gicp_point_cloud_t *cloud,
                                       bool *has_covariances);

small_gicp_error_t
small_gicp_point_cloud_empty(const small_gicp_point_cloud_t *cloud,
                             bool *is_empty);

// Point cloud data loading from array
small_gicp_error_t
small_gicp_load_points_from_array(const float *points, size_t num_points,
                                  small_gicp_point_cloud_t **cloud);

// Direct access to internal vectors (bulk operations)
small_gicp_error_t
small_gicp_point_cloud_get_points_data(const small_gicp_point_cloud_t *cloud,
                                       double **points_data, size_t *data_size);

small_gicp_error_t
small_gicp_point_cloud_get_normals_data(const small_gicp_point_cloud_t *cloud,
                                        double **normals_data,
                                        size_t *data_size);

small_gicp_error_t small_gicp_point_cloud_get_covariances_data(
    const small_gicp_point_cloud_t *cloud, double **covariances_data,
    size_t *data_size);

// Bulk point operations
small_gicp_error_t
small_gicp_point_cloud_set_points_bulk(small_gicp_point_cloud_t *cloud,
                                       const double *points_data,
                                       size_t num_points);

small_gicp_error_t
small_gicp_point_cloud_set_normals_bulk(small_gicp_point_cloud_t *cloud,
                                        const double *normals_data,
                                        size_t num_points);

small_gicp_error_t
small_gicp_point_cloud_set_covariances_bulk(small_gicp_point_cloud_t *cloud,
                                            const double *covariances_data,
                                            size_t num_points);

// Copy data from internal vectors to user-provided arrays
small_gicp_error_t small_gicp_point_cloud_copy_points_to_array(
    const small_gicp_point_cloud_t *cloud, double *points_array,
    size_t array_size);

small_gicp_error_t small_gicp_point_cloud_copy_normals_to_array(
    const small_gicp_point_cloud_t *cloud, double *normals_array,
    size_t array_size);

small_gicp_error_t small_gicp_point_cloud_copy_covariances_to_array(
    const small_gicp_point_cloud_t *cloud, double *covariances_array,
    size_t array_size);

#ifdef __cplusplus
}
#endif

#endif // SMALL_GICP_C_POINTS_POINT_CLOUD_H
