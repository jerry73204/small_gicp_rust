#include "test_common.h"

static small_gicp_point_cloud_t *test_cloud = NULL;

void setUp(void) {
  // Create a new point cloud for each test
  small_gicp_error_t error = small_gicp_point_cloud_create(&test_cloud);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);
  TEST_ASSERT_NOT_NULL(test_cloud);
}

void tearDown(void) {
  // Clean up after each test
  if (test_cloud) {
    small_gicp_point_cloud_destroy(test_cloud);
    test_cloud = NULL;
  }
}

void test_point_cloud_creation(void) {
  size_t size;
  small_gicp_error_t error = small_gicp_point_cloud_size(test_cloud, &size);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);
  TEST_ASSERT_EQUAL(0, size);

  bool empty;
  error = small_gicp_point_cloud_empty(test_cloud, &empty);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);
  TEST_ASSERT_TRUE(empty);
}

void test_point_cloud_resize(void) {
  // Test resizing to a specific size
  small_gicp_error_t error = small_gicp_point_cloud_resize(test_cloud, 100);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);

  size_t size;
  error = small_gicp_point_cloud_size(test_cloud, &size);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);
  TEST_ASSERT_EQUAL(100, size);

  bool empty;
  error = small_gicp_point_cloud_empty(test_cloud, &empty);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);
  TEST_ASSERT_FALSE(empty);

  // Test resizing to zero
  error = small_gicp_point_cloud_resize(test_cloud, 0);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);

  error = small_gicp_point_cloud_size(test_cloud, &size);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);
  TEST_ASSERT_EQUAL(0, size);
}

void test_point_operations(void) {
  // Resize to hold some points
  small_gicp_error_t error = small_gicp_point_cloud_resize(test_cloud, 3);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);

  // Set points
  error = small_gicp_point_cloud_set_point(test_cloud, 0, 1.0, 2.0, 3.0);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);

  error = small_gicp_point_cloud_set_point(test_cloud, 1, 4.0, 5.0, 6.0);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);

  error = small_gicp_point_cloud_set_point(test_cloud, 2, 7.0, 8.0, 9.0);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);

  // Get points and verify
  double x, y, z;
  error = small_gicp_point_cloud_get_point(test_cloud, 0, &x, &y, &z);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);
  TEST_ASSERT_POINT_NEAR(1.0, 2.0, 3.0, x, y, z, TEST_TOLERANCE_DOUBLE);

  error = small_gicp_point_cloud_get_point(test_cloud, 1, &x, &y, &z);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);
  TEST_ASSERT_POINT_NEAR(4.0, 5.0, 6.0, x, y, z, TEST_TOLERANCE_DOUBLE);

  error = small_gicp_point_cloud_get_point(test_cloud, 2, &x, &y, &z);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);
  TEST_ASSERT_POINT_NEAR(7.0, 8.0, 9.0, x, y, z, TEST_TOLERANCE_DOUBLE);
}

void test_normal_operations(void) {
  // Resize to hold some points
  small_gicp_error_t error = small_gicp_point_cloud_resize(test_cloud, 2);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);

  // Set normals
  error = small_gicp_point_cloud_set_normal(test_cloud, 0, 1.0, 0.0, 0.0);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);

  error = small_gicp_point_cloud_set_normal(test_cloud, 1, 0.0, 1.0, 0.0);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);

  // Get normals and verify
  double nx, ny, nz;
  error = small_gicp_point_cloud_get_normal(test_cloud, 0, &nx, &ny, &nz);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);
  TEST_ASSERT_POINT_NEAR(1.0, 0.0, 0.0, nx, ny, nz, TEST_TOLERANCE_DOUBLE);

  error = small_gicp_point_cloud_get_normal(test_cloud, 1, &nx, &ny, &nz);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);
  TEST_ASSERT_POINT_NEAR(0.0, 1.0, 0.0, nx, ny, nz, TEST_TOLERANCE_DOUBLE);
}

void test_boundary_conditions(void) {
  // Test accessing points in empty cloud
  double x, y, z;
  small_gicp_error_t error =
      small_gicp_point_cloud_get_point(test_cloud, 0, &x, &y, &z);
  TEST_ASSERT_NOT_EQUAL(SMALL_GICP_SUCCESS, error);

  // Test setting point in empty cloud
  error = small_gicp_point_cloud_set_point(test_cloud, 0, 1.0, 2.0, 3.0);
  TEST_ASSERT_NOT_EQUAL(SMALL_GICP_SUCCESS, error);

  // Resize and test out-of-bounds access
  error = small_gicp_point_cloud_resize(test_cloud, 5);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);

  // Valid access
  error = small_gicp_point_cloud_set_point(test_cloud, 4, 1.0, 2.0, 3.0);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);

  // Invalid access (beyond size)
  error = small_gicp_point_cloud_get_point(test_cloud, 5, &x, &y, &z);
  TEST_ASSERT_NOT_EQUAL(SMALL_GICP_SUCCESS, error);

  error = small_gicp_point_cloud_set_point(test_cloud, 5, 1.0, 2.0, 3.0);
  TEST_ASSERT_NOT_EQUAL(SMALL_GICP_SUCCESS, error);
}

void test_large_point_cloud(void) {
  const size_t large_size = 10000;
  test_timer_t timer;

  // Benchmark large resize
  BENCHMARK_START(timer);
  small_gicp_error_t error =
      small_gicp_point_cloud_resize(test_cloud, large_size);
  BENCHMARK_END(timer, "Large resize", 100.0); // Should complete in < 100ms

  TEST_ASSERT_SMALL_GICP_SUCCESS(error);

  size_t size;
  error = small_gicp_point_cloud_size(test_cloud, &size);
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);
  TEST_ASSERT_EQUAL(large_size, size);

  // Test setting points in large cloud
  BENCHMARK_START(timer);
  for (size_t i = 0; i < 1000; i++) { // Just test first 1000
    error = small_gicp_point_cloud_set_point(test_cloud, i, (double)i,
                                             (double)i * 2, (double)i * 3);
    TEST_ASSERT_SMALL_GICP_SUCCESS(error);
  }
  BENCHMARK_END(timer, "Setting 1000 points", 50.0);
}

void test_null_pointer_handling(void) {
  // Test null pointer arguments
  size_t size;
  small_gicp_error_t error = small_gicp_point_cloud_size(NULL, &size);
  TEST_ASSERT_NOT_EQUAL(SMALL_GICP_SUCCESS, error);

  error = small_gicp_point_cloud_size(test_cloud, NULL);
  TEST_ASSERT_NOT_EQUAL(SMALL_GICP_SUCCESS, error);

  double x, y, z;
  error = small_gicp_point_cloud_get_point(NULL, 0, &x, &y, &z);
  TEST_ASSERT_NOT_EQUAL(SMALL_GICP_SUCCESS, error);

  error = small_gicp_point_cloud_set_point(NULL, 0, 1.0, 2.0, 3.0);
  TEST_ASSERT_NOT_EQUAL(SMALL_GICP_SUCCESS, error);
}

int main(void) {
  UNITY_BEGIN();

  RUN_TEST(test_point_cloud_creation);
  RUN_TEST(test_point_cloud_resize);
  RUN_TEST(test_point_operations);
  RUN_TEST(test_normal_operations);
  RUN_TEST(test_boundary_conditions);
  RUN_TEST(test_large_point_cloud);
  RUN_TEST(test_null_pointer_handling);

  return UNITY_END();
}