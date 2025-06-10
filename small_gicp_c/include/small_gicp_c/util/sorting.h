#ifndef SMALL_GICP_C_UTIL_SORTING_H
#define SMALL_GICP_C_UTIL_SORTING_H

#include <small_gicp_c/types.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Sorting backend types
typedef enum {
  SMALL_GICP_SORT_BACKEND_STD = 0,
  SMALL_GICP_SORT_BACKEND_OPENMP_MERGE = 1,
  SMALL_GICP_SORT_BACKEND_OPENMP_QUICK = 2,
  SMALL_GICP_SORT_BACKEND_TBB_PARALLEL = 3,
  SMALL_GICP_SORT_BACKEND_TBB_RADIX = 4
} small_gicp_sort_backend_t;

// Comparison function type for custom sorting
typedef int (*small_gicp_compare_func_t)(const void *a, const void *b);

// Key extraction function type for radix sort (returns uint64_t key)
typedef uint64_t (*small_gicp_key_func_t)(const void *element);

// Sort array of doubles
small_gicp_error_t small_gicp_sort_doubles(double *array, size_t count,
                                           small_gicp_sort_backend_t backend,
                                           int num_threads);

// Sort array of floats
small_gicp_error_t small_gicp_sort_floats(float *array, size_t count,
                                          small_gicp_sort_backend_t backend,
                                          int num_threads);

// Sort array of 32-bit integers
small_gicp_error_t small_gicp_sort_ints(int *array, size_t count,
                                        small_gicp_sort_backend_t backend,
                                        int num_threads);

// Sort array of 64-bit integers
small_gicp_error_t small_gicp_sort_longs(long long *array, size_t count,
                                         small_gicp_sort_backend_t backend,
                                         int num_threads);

// Sort array of size_t
small_gicp_error_t small_gicp_sort_size_t(size_t *array, size_t count,
                                          small_gicp_sort_backend_t backend,
                                          int num_threads);

// Generic sort with custom comparison function
small_gicp_error_t
small_gicp_sort_generic(void *array, size_t count, size_t element_size,
                        small_gicp_compare_func_t compare_func,
                        small_gicp_sort_backend_t backend, int num_threads);

// TBB radix sort for unsigned integers with custom key function
small_gicp_error_t small_gicp_radix_sort_tbb(void *array, size_t count,
                                             size_t element_size,
                                             small_gicp_key_func_t key_func);

// Parallel sort index array based on values
small_gicp_error_t small_gicp_sort_indices_by_values_double(
    size_t *indices, const double *values, size_t count,
    small_gicp_sort_backend_t backend, int num_threads);

small_gicp_error_t small_gicp_sort_indices_by_values_float(
    size_t *indices, const float *values, size_t count,
    small_gicp_sort_backend_t backend, int num_threads);

#ifdef __cplusplus
}
#endif

#endif // SMALL_GICP_C_UTIL_SORTING_H
