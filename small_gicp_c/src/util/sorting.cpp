#include "../common.h"
#include <small_gicp/util/sort_omp.hpp>
#include <small_gicp_c/util/sorting.h>

#ifdef SMALL_GICP_HAS_TBB
#include <small_gicp/util/sort_tbb.hpp>
#include <tbb/parallel_sort.h>
#endif

#include <algorithm>
#include <cstdlib>

#define CHECK_NULL(ptr)                                                        \
  if (!(ptr))                                                                  \
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;

// Helper function to get comparison function for different types
template <typename T> struct DefaultComparison {
  static int compare(const void *a, const void *b) {
    const T *ta = static_cast<const T *>(a);
    const T *tb = static_cast<const T *>(b);
    if (*ta < *tb)
      return -1;
    if (*ta > *tb)
      return 1;
    return 0;
  }
};

template <typename T>
small_gicp_error_t sort_array_impl(T *array, size_t count,
                                   small_gicp_sort_backend_t backend,
                                   int num_threads) {
  if (count == 0)
    return SMALL_GICP_SUCCESS;

  TRY_CATCH_BEGIN
  switch (backend) {
  case SMALL_GICP_SORT_BACKEND_STD:
    std::sort(array, array + count);
    break;

  case SMALL_GICP_SORT_BACKEND_OPENMP_MERGE:
    small_gicp::merge_sort_omp(array, array + count, std::less<T>(),
                               num_threads);
    break;

  case SMALL_GICP_SORT_BACKEND_OPENMP_QUICK:
    small_gicp::quick_sort_omp(array, array + count, std::less<T>(),
                               num_threads);
    break;

  case SMALL_GICP_SORT_BACKEND_TBB_PARALLEL:
#ifdef SMALL_GICP_HAS_TBB
    tbb::parallel_sort(array, array + count);
#else
    return SMALL_GICP_ERROR_INVALID_ARGUMENT; // TBB not available
#endif
    break;

  case SMALL_GICP_SORT_BACKEND_TBB_RADIX:
    // Radix sort only works for unsigned integer types
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;

  default:
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }
  TRY_CATCH_END
}

small_gicp_error_t small_gicp_sort_doubles(double *array, size_t count,
                                           small_gicp_sort_backend_t backend,
                                           int num_threads) {
  CHECK_NULL(array);
  return sort_array_impl(array, count, backend, num_threads);
}

small_gicp_error_t small_gicp_sort_floats(float *array, size_t count,
                                          small_gicp_sort_backend_t backend,
                                          int num_threads) {
  CHECK_NULL(array);
  return sort_array_impl(array, count, backend, num_threads);
}

small_gicp_error_t small_gicp_sort_ints(int *array, size_t count,
                                        small_gicp_sort_backend_t backend,
                                        int num_threads) {
  CHECK_NULL(array);
  return sort_array_impl(array, count, backend, num_threads);
}

small_gicp_error_t small_gicp_sort_longs(long long *array, size_t count,
                                         small_gicp_sort_backend_t backend,
                                         int num_threads) {
  CHECK_NULL(array);
  return sort_array_impl(array, count, backend, num_threads);
}

small_gicp_error_t small_gicp_sort_size_t(size_t *array, size_t count,
                                          small_gicp_sort_backend_t backend,
                                          int num_threads) {
  CHECK_NULL(array);

  if (count == 0)
    return SMALL_GICP_SUCCESS;

  TRY_CATCH_BEGIN
  switch (backend) {
  case SMALL_GICP_SORT_BACKEND_STD:
    std::sort(array, array + count);
    break;

  case SMALL_GICP_SORT_BACKEND_OPENMP_MERGE:
    small_gicp::merge_sort_omp(array, array + count, std::less<size_t>(),
                               num_threads);
    break;

  case SMALL_GICP_SORT_BACKEND_OPENMP_QUICK:
    small_gicp::quick_sort_omp(array, array + count, std::less<size_t>(),
                               num_threads);
    break;

  case SMALL_GICP_SORT_BACKEND_TBB_PARALLEL:
#ifdef SMALL_GICP_HAS_TBB
    tbb::parallel_sort(array, array + count);
#else
    return SMALL_GICP_ERROR_INVALID_ARGUMENT; // TBB not available
#endif
    break;

  case SMALL_GICP_SORT_BACKEND_TBB_RADIX:
#ifdef SMALL_GICP_HAS_TBB
    // Use identity key function for size_t (already unsigned)
    small_gicp::radix_sort_tbb(array, array + count,
                               [](const size_t &x) { return x; });
#else
    return SMALL_GICP_ERROR_INVALID_ARGUMENT; // TBB not available
#endif
    break;

  default:
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }
  TRY_CATCH_END
}

small_gicp_error_t
small_gicp_sort_generic(void *array, size_t count, size_t element_size,
                        small_gicp_compare_func_t compare_func,
                        small_gicp_sort_backend_t backend, int num_threads) {
  CHECK_NULL(array);
  CHECK_NULL(compare_func);

  if (count == 0)
    return SMALL_GICP_SUCCESS;

  TRY_CATCH_BEGIN
  switch (backend) {
  case SMALL_GICP_SORT_BACKEND_STD:
    std::qsort(array, count, element_size, compare_func);
    break;

  case SMALL_GICP_SORT_BACKEND_OPENMP_MERGE:
  case SMALL_GICP_SORT_BACKEND_OPENMP_QUICK:
  case SMALL_GICP_SORT_BACKEND_TBB_PARALLEL:
  case SMALL_GICP_SORT_BACKEND_TBB_RADIX:
    // Generic sorting with custom comparison is not directly supported
    // by the parallel implementations in small_gicp. Fall back to std::qsort.
    std::qsort(array, count, element_size, compare_func);
    break;

  default:
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }
  TRY_CATCH_END
}

small_gicp_error_t small_gicp_radix_sort_tbb(void *array, size_t count,
                                             size_t element_size,
                                             small_gicp_key_func_t key_func) {
  CHECK_NULL(array);
  CHECK_NULL(key_func);

#ifdef SMALL_GICP_HAS_TBB
  if (count == 0)
    return SMALL_GICP_SUCCESS;

  TRY_CATCH_BEGIN
  // This is a simplified wrapper. The actual implementation would need
  // to handle different element sizes properly. For now, we only support
  // common cases where element_size matches standard types.

  if (element_size == sizeof(size_t)) {
    size_t *typed_array = static_cast<size_t *>(array);
    small_gicp::radix_sort_tbb(
        typed_array, typed_array + count,
        [key_func](const size_t &x) { return key_func(&x); });
  } else if (element_size == sizeof(uint64_t)) {
    uint64_t *typed_array = static_cast<uint64_t *>(array);
    small_gicp::radix_sort_tbb(
        typed_array, typed_array + count,
        [key_func](const uint64_t &x) { return key_func(&x); });
  } else if (element_size == sizeof(uint32_t)) {
    uint32_t *typed_array = static_cast<uint32_t *>(array);
    small_gicp::radix_sort_tbb(
        typed_array, typed_array + count,
        [key_func](const uint32_t &x) { return key_func(&x); });
  } else {
    return SMALL_GICP_ERROR_INVALID_ARGUMENT; // Unsupported element size
  }
  TRY_CATCH_END
#else
  return SMALL_GICP_ERROR_INVALID_ARGUMENT; // TBB not available
#endif
}

template <typename T>
small_gicp_error_t
sort_indices_by_values_impl(size_t *indices, const T *values, size_t count,
                            small_gicp_sort_backend_t backend,
                            int num_threads) {
  if (count == 0)
    return SMALL_GICP_SUCCESS;

  TRY_CATCH_BEGIN
  // Initialize indices
  for (size_t i = 0; i < count; i++) {
    indices[i] = i;
  }

  // Define comparison function for indices based on values
  auto compare = [values](size_t a, size_t b) { return values[a] < values[b]; };

  switch (backend) {
  case SMALL_GICP_SORT_BACKEND_STD:
    std::sort(indices, indices + count, compare);
    break;

  case SMALL_GICP_SORT_BACKEND_OPENMP_MERGE:
    small_gicp::merge_sort_omp(indices, indices + count, compare, num_threads);
    break;

  case SMALL_GICP_SORT_BACKEND_OPENMP_QUICK:
    small_gicp::quick_sort_omp(indices, indices + count, compare, num_threads);
    break;

  case SMALL_GICP_SORT_BACKEND_TBB_PARALLEL:
#ifdef SMALL_GICP_HAS_TBB
    tbb::parallel_sort(indices, indices + count, compare);
#else
    return SMALL_GICP_ERROR_INVALID_ARGUMENT; // TBB not available
#endif
    break;

  case SMALL_GICP_SORT_BACKEND_TBB_RADIX:
    // Radix sort is not suitable for sorting indices by values
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;

  default:
    return SMALL_GICP_ERROR_INVALID_ARGUMENT;
  }
  TRY_CATCH_END
}

small_gicp_error_t small_gicp_sort_indices_by_values_double(
    size_t *indices, const double *values, size_t count,
    small_gicp_sort_backend_t backend, int num_threads) {
  CHECK_NULL(indices);
  CHECK_NULL(values);
  return sort_indices_by_values_impl(indices, values, count, backend,
                                     num_threads);
}

small_gicp_error_t small_gicp_sort_indices_by_values_float(
    size_t *indices, const float *values, size_t count,
    small_gicp_sort_backend_t backend, int num_threads) {
  CHECK_NULL(indices);
  CHECK_NULL(values);
  return sort_indices_by_values_impl(indices, values, count, backend,
                                     num_threads);
}
