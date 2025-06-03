#ifndef SMALL_GICP_C_IO_IO_H
#define SMALL_GICP_C_IO_IO_H

#include <small_gicp_c/types.h>

#ifdef __cplusplus
extern "C" {
#endif

// Point cloud I/O operations
small_gicp_error_t small_gicp_load_ply(const char *filename,
                                       small_gicp_point_cloud_t **cloud);

small_gicp_error_t small_gicp_save_ply(const char *filename,
                                       const small_gicp_point_cloud_t *cloud);

#ifdef __cplusplus
}
#endif

#endif // SMALL_GICP_C_IO_IO_H