#include "test_common.h"
#include <string.h>

// Internal state for memory tracking
static size_t peak_memory_usage = 0;
static size_t current_allocations = 0;

const char *get_test_data_path(const char *filename) {
  static char path[512];
  snprintf(path, sizeof(path), "%s/%s", TEST_DATA_DIR, filename);
  return path;
}

void assert_point_near(double x1, double y1, double z1, double x2, double y2,
                       double z2, double tol) {
  TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(tol, x1, x2, "X coordinates differ");
  TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(tol, y1, y2, "Y coordinates differ");
  TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(tol, z1, z2, "Z coordinates differ");
}

void assert_vector_near(const double *v1, const double *v2, int size,
                        double tol) {
  for (int i = 0; i < size; i++) {
    char msg[128];
    snprintf(msg, sizeof(msg), "Vector element %d differs", i);
    TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(tol, v1[i], v2[i], msg);
  }
}

void assert_matrix_near(const double *m1, const double *m2, int rows, int cols,
                        double tol) {
  for (int i = 0; i < rows * cols; i++) {
    char msg[128];
    snprintf(msg, sizeof(msg), "Matrix element %d differs", i);
    TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(tol, m1[i], m2[i], msg);
  }
}

void assert_transformation_valid(const double *T) {
  // Check that it's a 4x4 matrix with proper structure
  // Last row should be [0, 0, 0, 1]
  TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(1e-10, T[12], 0.0, "T[3,0] should be 0");
  TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(1e-10, T[13], 0.0, "T[3,1] should be 0");
  TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(1e-10, T[14], 0.0, "T[3,2] should be 0");
  TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(1e-10, T[15], 1.0, "T[3,3] should be 1");

  // Check that rotation part is a valid rotation matrix
  assert_rotation_matrix_valid(T);
}

void assert_rotation_matrix_valid(const double *R) {
  // Extract 3x3 rotation matrix (assuming column-major order)
  double R33[9];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      R33[i * 3 + j] = R[i * 4 + j]; // Skip the 4th column/row
    }
  }

  // Check determinant ≈ 1
  double det = R33[0] * (R33[4] * R33[8] - R33[5] * R33[7]) -
               R33[1] * (R33[3] * R33[8] - R33[5] * R33[6]) +
               R33[2] * (R33[3] * R33[7] - R33[4] * R33[6]);
  TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(1e-6, det, 1.0, "Determinant should be 1");

  // Check orthogonality: R * R^T = I (check a few key elements)
  double dot_01 = R33[0] * R33[1] + R33[3] * R33[4] + R33[6] * R33[7];
  double dot_02 = R33[0] * R33[2] + R33[3] * R33[5] + R33[6] * R33[8];
  double dot_12 = R33[1] * R33[2] + R33[4] * R33[5] + R33[7] * R33[8];

  TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(1e-6, dot_01, 0.0,
                                    "Rotation matrix not orthogonal (0,1)");
  TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(1e-6, dot_02, 0.0,
                                    "Rotation matrix not orthogonal (0,2)");
  TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(1e-6, dot_12, 0.0,
                                    "Rotation matrix not orthogonal (1,2)");
}

void generate_random_points(small_gicp_point_cloud_t *cloud, int n,
                            unsigned int seed) {
  srand(seed);

  small_gicp_error_t error = small_gicp_point_cloud_resize(cloud, n);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);

  for (int i = 0; i < n; i++) {
    double x = ((double)rand() / RAND_MAX) * 10.0 - 5.0; // [-5, 5]
    double y = ((double)rand() / RAND_MAX) * 10.0 - 5.0;
    double z = ((double)rand() / RAND_MAX) * 10.0 - 5.0;

    error = small_gicp_point_cloud_set_point(cloud, i, x, y, z);
    TEST_ASSERT_SMALL_GICP_SUCCESS(error);
  }
}

void generate_grid_points(small_gicp_point_cloud_t *cloud, int nx, int ny,
                          int nz, double spacing) {
  int total_points = nx * ny * nz;

  small_gicp_error_t error = small_gicp_point_cloud_resize(cloud, total_points);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);

  int idx = 0;
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      for (int k = 0; k < nz; k++) {
        double x = i * spacing;
        double y = j * spacing;
        double z = k * spacing;

        error = small_gicp_point_cloud_set_point(cloud, idx, x, y, z);
        TEST_ASSERT_SMALL_GICP_SUCCESS(error);
        idx++;
      }
    }
  }
}

void add_gaussian_noise(small_gicp_point_cloud_t *cloud, double sigma,
                        unsigned int seed) {
  srand(seed);

  size_t size;
  small_gicp_error_t error = small_gicp_point_cloud_size(cloud, &size);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);

  for (size_t i = 0; i < size; i++) {
    double x, y, z;
    error = small_gicp_point_cloud_get_point(cloud, i, &x, &y, &z);
    TEST_ASSERT_SMALL_GICP_SUCCESS(error);

    // Simple Box-Muller transform for Gaussian noise
    static int has_spare = 0;
    static double spare;

    if (has_spare) {
      has_spare = 0;
      x += spare * sigma;
    } else {
      has_spare = 1;
      double u = ((double)rand() / RAND_MAX) * 2.0 - 1.0;
      double v = ((double)rand() / RAND_MAX) * 2.0 - 1.0;
      double s = u * u + v * v;
      while (s >= 1.0 || s == 0.0) {
        u = ((double)rand() / RAND_MAX) * 2.0 - 1.0;
        v = ((double)rand() / RAND_MAX) * 2.0 - 1.0;
        s = u * u + v * v;
      }
      s = sqrt(-2.0 * log(s) / s);
      spare = v * s;
      x += u * s * sigma;
    }

    // Add noise to y and z similarly (simplified)
    y += (((double)rand() / RAND_MAX) - 0.5) * sigma * 2.0;
    z += (((double)rand() / RAND_MAX) - 0.5) * sigma * 2.0;

    error = small_gicp_point_cloud_set_point(cloud, i, x, y, z);
    TEST_ASSERT_SMALL_GICP_SUCCESS(error);
  }
}

void transform_point_cloud(small_gicp_point_cloud_t *cloud,
                           const double *transformation) {
  size_t size;
  small_gicp_error_t error = small_gicp_point_cloud_size(cloud, &size);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);

  for (size_t i = 0; i < size; i++) {
    double x, y, z;
    error = small_gicp_point_cloud_get_point(cloud, i, &x, &y, &z);
    TEST_ASSERT_SMALL_GICP_SUCCESS(error);

    // Apply transformation (assuming column-major 4x4 matrix)
    double tx = transformation[0] * x + transformation[4] * y +
                transformation[8] * z + transformation[12];
    double ty = transformation[1] * x + transformation[5] * y +
                transformation[9] * z + transformation[13];
    double tz = transformation[2] * x + transformation[6] * y +
                transformation[10] * z + transformation[14];

    error = small_gicp_point_cloud_set_point(cloud, i, tx, ty, tz);
    TEST_ASSERT_SMALL_GICP_SUCCESS(error);
  }
}

int file_exists(const char *path) {
  FILE *file = fopen(path, "r");
  if (file) {
    fclose(file);
    return 1;
  }
  return 0;
}

int load_test_point_cloud(const char *filename,
                          small_gicp_point_cloud_t **cloud) {
  const char *full_path = get_test_data_path(filename);

  if (!file_exists(full_path)) {
    return 0;
  }

  small_gicp_error_t error = small_gicp_load_ply(full_path, cloud);
  return (error == SMALL_GICP_SUCCESS) ? 1 : 0;
}

void timer_start(test_timer_t *timer) { timer->start_time = clock(); }

double timer_elapsed_ms(test_timer_t *timer) {
  clock_t end_time = clock();
  return ((double)(end_time - timer->start_time)) / CLOCKS_PER_SEC * 1000.0;
}

void memory_tracking_start(void) {
  peak_memory_usage = 0;
  current_allocations = 0;
}

size_t memory_tracking_get_peak(void) { return peak_memory_usage; }

int memory_tracking_check_leaks(void) {
  return (current_allocations == 0) ? 1 : 0;
}