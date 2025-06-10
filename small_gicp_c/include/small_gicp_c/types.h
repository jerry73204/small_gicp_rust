#ifndef SMALL_GICP_C_TYPES_H
#define SMALL_GICP_C_TYPES_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Error codes
typedef enum {
  SMALL_GICP_SUCCESS = 0,
  SMALL_GICP_ERROR_INVALID_ARGUMENT = -1,
  SMALL_GICP_ERROR_OUT_OF_MEMORY = -2,
  SMALL_GICP_ERROR_FILE_NOT_FOUND = -3,
  SMALL_GICP_ERROR_IO_ERROR = -4,
  SMALL_GICP_ERROR_EXCEPTION = -5,
  SMALL_GICP_NOT_IMPLEMENTED = -6
} small_gicp_error_t;

// Registration types
typedef enum {
  SMALL_GICP_ICP = 0,
  SMALL_GICP_PLANE_ICP = 1,
  SMALL_GICP_GICP = 2,
  SMALL_GICP_VGICP = 3
} small_gicp_registration_type_t;

// Opaque types - forward declarations
typedef struct small_gicp_point_cloud small_gicp_point_cloud_t;
typedef struct small_gicp_kdtree small_gicp_kdtree_t;
typedef struct small_gicp_gaussian_voxelmap small_gicp_gaussian_voxelmap_t;

// Registration result structure
typedef struct {
  double T_target_source[16]; // 4x4 transformation matrix in row-major order
  bool converged;
  int iterations;
  int num_inliers;
  double error;
} small_gicp_registration_result_t;

// Utility functions
const char *small_gicp_error_string(small_gicp_error_t error);

#ifdef __cplusplus
}
#endif

#endif // SMALL_GICP_C_TYPES_H