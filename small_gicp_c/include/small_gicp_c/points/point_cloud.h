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
small_gicp_point_cloud_size(const small_gicp_point_cloud_t *cloud, size_t *size);

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

// Point cloud data loading from array
small_gicp_error_t
small_gicp_load_points_from_array(const float *points, size_t num_points,
                                  small_gicp_point_cloud_t **cloud);

#ifdef __cplusplus
}
#endif

#endif // SMALL_GICP_C_POINTS_POINT_CLOUD_H