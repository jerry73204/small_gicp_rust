#include <small_gicp_c/types.h>

const char *small_gicp_error_string(small_gicp_error_t error) {
  switch (error) {
  case SMALL_GICP_SUCCESS:
    return "Success";
  case SMALL_GICP_ERROR_INVALID_ARGUMENT:
    return "Invalid argument";
  case SMALL_GICP_ERROR_OUT_OF_MEMORY:
    return "Out of memory";
  case SMALL_GICP_ERROR_FILE_NOT_FOUND:
    return "File not found";
  case SMALL_GICP_ERROR_IO_ERROR:
    return "I/O error";
  case SMALL_GICP_ERROR_EXCEPTION:
    return "Exception occurred";
  default:
    return "Unknown error";
  }
}