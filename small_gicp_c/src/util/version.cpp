#include <small_gicp_c/util/version.h>
#include "../common.h"

#include <cstring>

#define CHECK_NULL(ptr)                                                        \
  if (!(ptr))                                                                  \
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;

small_gicp_error_t small_gicp_get_version(char *buffer, size_t buffer_size) {
  CHECK_NULL(buffer);

  const char *version = "1.0.0";
  size_t len = strlen(version);

  if (buffer_size <= len) {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }

  strcpy(buffer, version);
  return SMALL_GICP_SUCCESS;
}