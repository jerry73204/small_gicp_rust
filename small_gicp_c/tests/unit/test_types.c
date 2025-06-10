#include "test_common.h"

void setUp(void) {
  // Called before each test
}

void tearDown(void) {
  // Called after each test
}

void test_error_codes_are_defined(void) {
  // Test that all expected error codes are defined
  TEST_ASSERT_EQUAL(0, SMALL_GICP_SUCCESS);
  TEST_ASSERT_NOT_EQUAL(SMALL_GICP_SUCCESS, SMALL_GICP_ERROR_INVALID_ARGUMENT);
  TEST_ASSERT_NOT_EQUAL(SMALL_GICP_SUCCESS, SMALL_GICP_ERROR_OUT_OF_MEMORY);
  TEST_ASSERT_NOT_EQUAL(SMALL_GICP_SUCCESS, SMALL_GICP_ERROR_FILE_NOT_FOUND);
  TEST_ASSERT_NOT_EQUAL(SMALL_GICP_SUCCESS, SMALL_GICP_ERROR_IO_ERROR);
  TEST_ASSERT_NOT_EQUAL(SMALL_GICP_SUCCESS, SMALL_GICP_ERROR_EXCEPTION);
}

void test_registration_types_are_defined(void) {
  // Test that registration type enum values are defined
  TEST_ASSERT_TRUE(SMALL_GICP_ICP >= 0);
  TEST_ASSERT_TRUE(SMALL_GICP_PLANE_ICP >= 0);
  TEST_ASSERT_TRUE(SMALL_GICP_GICP >= 0);
  TEST_ASSERT_TRUE(SMALL_GICP_VGICP >= 0);

  // Test that they are different values
  TEST_ASSERT_NOT_EQUAL(SMALL_GICP_ICP, SMALL_GICP_PLANE_ICP);
  TEST_ASSERT_NOT_EQUAL(SMALL_GICP_ICP, SMALL_GICP_GICP);
  TEST_ASSERT_NOT_EQUAL(SMALL_GICP_ICP, SMALL_GICP_VGICP);
}

void test_optimizer_types_are_defined(void) {
  // Test that optimizer type enum values are defined
  TEST_ASSERT_TRUE(SMALL_GICP_OPTIMIZER_GAUSS_NEWTON >= 0);
  TEST_ASSERT_TRUE(SMALL_GICP_OPTIMIZER_LEVENBERG_MARQUARDT >= 0);

  TEST_ASSERT_NOT_EQUAL(SMALL_GICP_OPTIMIZER_GAUSS_NEWTON,
                        SMALL_GICP_OPTIMIZER_LEVENBERG_MARQUARDT);
}

void test_reduction_types_are_defined(void) {
  // Test that parallel reduction enum values are defined
  TEST_ASSERT_TRUE(SMALL_GICP_REDUCTION_SERIAL >= 0);
  TEST_ASSERT_TRUE(SMALL_GICP_REDUCTION_OPENMP >= 0);
  TEST_ASSERT_TRUE(SMALL_GICP_REDUCTION_TBB >= 0);

  TEST_ASSERT_NOT_EQUAL(SMALL_GICP_REDUCTION_SERIAL,
                        SMALL_GICP_REDUCTION_OPENMP);
  TEST_ASSERT_NOT_EQUAL(SMALL_GICP_REDUCTION_SERIAL, SMALL_GICP_REDUCTION_TBB);
}

void test_basic_constants(void) {
  // Test some basic size/dimension constants if they exist
  // This is mainly a compile-time check that headers are included correctly
  TEST_ASSERT_TRUE(sizeof(small_gicp_point_cloud_t *) > 0);
  TEST_ASSERT_TRUE(sizeof(small_gicp_kdtree_t *) > 0);
  TEST_ASSERT_TRUE(sizeof(small_gicp_gaussian_voxelmap_t *) > 0);
}

void test_version_info(void) {
  // Test that version information is available
  char version_buffer[256];
  small_gicp_error_t error =
      small_gicp_get_version(version_buffer, sizeof(version_buffer));
  TEST_ASSERT_SMALL_GICP_SUCCESS(error);
  TEST_ASSERT_TRUE(strlen(version_buffer) > 0);

  printf("small_gicp_c version: %s\n", version_buffer);
}

int main(void) {
  UNITY_BEGIN();

  RUN_TEST(test_error_codes_are_defined);
  RUN_TEST(test_registration_types_are_defined);
  RUN_TEST(test_optimizer_types_are_defined);
  RUN_TEST(test_reduction_types_are_defined);
  RUN_TEST(test_basic_constants);
  RUN_TEST(test_version_info);

  return UNITY_END();
}