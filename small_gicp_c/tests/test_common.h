#ifndef TEST_COMMON_H
#define TEST_COMMON_H

#include <math.h>
#include <small_gicp_c.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unity.h>

#ifdef __cplusplus
extern "C" {
#endif

// Test tolerances
#define TEST_TOLERANCE_FLOAT 1e-6
#define TEST_TOLERANCE_DOUBLE 1e-12
#define TEST_TOLERANCE_ANGLE 1e-3

// Test data paths
extern const char *get_test_data_path(const char *filename);

// Custom assertions
void assert_point_near(double x1, double y1, double z1, double x2, double y2,
                       double z2, double tol);
void assert_vector_near(const double *v1, const double *v2, int size,
                        double tol);
void assert_matrix_near(const double *m1, const double *m2, int rows, int cols,
                        double tol);
void assert_transformation_valid(const double *T);
void assert_rotation_matrix_valid(const double *R);

// Point cloud utilities
void generate_random_points(small_gicp_point_cloud_t *cloud, int n,
                            unsigned int seed);
void generate_grid_points(small_gicp_point_cloud_t *cloud, int nx, int ny,
                          int nz, double spacing);
void add_gaussian_noise(small_gicp_point_cloud_t *cloud, double sigma,
                        unsigned int seed);
void transform_point_cloud(small_gicp_point_cloud_t *cloud,
                           const double *transformation);

// File utilities
int file_exists(const char *path);
int load_test_point_cloud(const char *filename,
                          small_gicp_point_cloud_t **cloud);

// Timing utilities
typedef struct {
  clock_t start_time;
} test_timer_t;

void timer_start(test_timer_t *timer);
double timer_elapsed_ms(test_timer_t *timer);

// Memory tracking utilities
void memory_tracking_start(void);
size_t memory_tracking_get_peak(void);
int memory_tracking_check_leaks(void);

// Test helper macros
#define TEST_ASSERT_SMALL_GICP_SUCCESS(error)                                  \
  TEST_ASSERT_EQUAL_MESSAGE(SMALL_GICP_SUCCESS, error,                         \
                            "Expected SMALL_GICP_SUCCESS")

#define TEST_ASSERT_POINT_NEAR(x1, y1, z1, x2, y2, z2, tol)                    \
  assert_point_near(x1, y1, z1, x2, y2, z2, tol)

#define TEST_ASSERT_VECTOR_NEAR(v1, v2, size, tol)                             \
  assert_vector_near(v1, v2, size, tol)

#define TEST_ASSERT_MATRIX_NEAR(m1, m2, rows, cols, tol)                       \
  assert_matrix_near(m1, m2, rows, cols, tol)

#define TEST_ASSERT_TRANSFORMATION_VALID(T) assert_transformation_valid(T)

#define TEST_ASSERT_FILE_EXISTS(path)                                          \
  TEST_ASSERT_TRUE_MESSAGE(file_exists(path), "File does not exist: " path)

// Benchmark macros
#define BENCHMARK_START(timer) timer_start(&timer)
#define BENCHMARK_END(timer, name, max_time_ms)                                \
  do {                                                                         \
    double elapsed = timer_elapsed_ms(&timer);                                 \
    printf("[BENCHMARK] %s: %.3f ms\n", name, elapsed);                        \
    TEST_ASSERT_LESS_THAN_MESSAGE(max_time_ms, elapsed,                        \
                                  "Benchmark failed: " name " took too long"); \
  } while (0)

#ifdef __cplusplus
}
#endif

#endif // TEST_COMMON_H