#ifndef SMALL_GICP_C_UTIL_VERSION_H
#define SMALL_GICP_C_UTIL_VERSION_H

#include <small_gicp_c/types.h>

#ifdef __cplusplus
extern "C" {
#endif

// Utility functions
small_gicp_error_t small_gicp_get_version(char *buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif // SMALL_GICP_C_UTIL_VERSION_H