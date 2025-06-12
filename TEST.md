# small_gicp C Wrapper Testing Plan

This document outlines the comprehensive testing strategy for the small_gicp C wrapper, based on the testing patterns established in the C++ codebase.

## Overview

The C++ codebase uses Google Test framework with comprehensive coverage across all modules. For the C wrapper, we will create analogous tests using a C-compatible testing framework while maintaining similar test coverage and patterns.

## C++ Test Reference Mapping

This test plan is directly based on the upstream C++ test suite with the following mapping:

| C Wrapper Test             | C++ Reference                                  | Description                                       |
|----------------------------|------------------------------------------------|---------------------------------------------------|
| `test_types.c`             | N/A                                            | C-specific error handling and type validation     |
| `test_point_cloud.c`       | `points_test.cpp`                              | Point cloud data structures and operations        |
| `test_kdtree.c`            | `kdtree_test.cpp`, `kdtree_synthetic_test.cpp` | KdTree construction and k-NN search               |
| `test_voxelmap.c`          | `kdtree_test.cpp` (voxel sections)             | IncrementalVoxelMap and GaussianVoxelMap          |
| `test_downsampling.c`      | `downsampling_test.cpp`                        | Voxel grid and random sampling                    |
| `test_normal_estimation.c` | `normal_estimation_test.cpp`                   | Normal and covariance estimation                  |
| `test_registration.c`      | `registration_test.cpp`                        | All registration algorithms (ICP, GICP, VGICP)    |
| `test_io.c`                | Helper functions in tests                      | PLY file I/O operations                           |
| `test_memory.c`            | N/A                                            | C-specific memory management and handle lifecycle |
| `test_utilities.c`         | `vector_test.cpp`, `sort_*_test.cpp`           | Internal utility functions                        |
| Integration tests          | `helper_test.cpp`                              | High-level workflows and preprocessing            |

**Total C++ Tests Analyzed**: 10 test files covering comprehensive functionality
**Key Reference**: `registration_test.cpp` is the most comprehensive test with all algorithms and parallel variants

## Testing Framework Selection

### Recommended: Unity Test Framework
- **Rationale**: Lightweight, pure C, minimal dependencies, widely used in embedded systems
- **Alternative 1**: Check - More feature-rich but heavier dependency
- **Alternative 2**: Custom minimal test macros - Maximum control but more maintenance

### Framework Features Needed
- [x] Basic assertions (equality, near equality for floats)
- [x] Test fixtures (setup/teardown)
- [x] Test grouping/suites
- [x] Memory leak detection integration
- [x] Performance timing
- [x] Parameterized tests (can be simulated with loops)

## Test Structure

### Directory Layout
```
small_gicp_c/
├── tests/
│   ├── CMakeLists.txt
│   ├── test_main.c
│   ├── test_common.h
│   ├── test_common.c
│   ├── data/                    # Symlink to small_gicp/data
│   ├── unit/
│   │   ├── test_types.c
│   │   ├── test_point_cloud.c
│   │   ├── test_kdtree.c
│   │   ├── test_voxelmap.c
│   │   ├── test_downsampling.c
│   │   ├── test_normal_estimation.c
│   │   ├── test_registration.c
│   │   ├── test_io.c
│   │   └── test_memory.c
│   └── integration/
│       ├── test_preprocessing_pipeline.c
│       ├── test_registration_pipeline.c
│       └── test_parallel_backends.c
```

## Test Categories and Coverage

### 1. **Core Types Tests** (`test_types.c`)
**C++ Reference**: No direct equivalent (C++ uses exceptions and templates)
- [ ] Error code handling
- [ ] Type size verification
- [ ] Enum value consistency
- [ ] Handle validity checking

### 2. **Point Cloud Tests** (`test_point_cloud.c`)
**C++ Reference**: `points_test.cpp`
- Tests point cloud traits system for various point types
- Tests `PointCloud` class and PCL point cloud compatibility
- Validates point, normal, and covariance data access/modification
- Tests resizing and data integrity operations

#### Basic Operations
- [ ] Creation and destruction
- [ ] Memory leak verification
- [ ] Resize operations (grow/shrink) *(ref: `points_test.cpp` resizing tests)*
- [ ] Empty point cloud handling
- [ ] Boundary conditions (size 0, 1, large sizes)

#### Point Operations
- [ ] Individual point get/set *(ref: `points_test.cpp` point access tests)*
- [ ] Bulk point operations
- [ ] Points data pointer access *(ref: `points_test.cpp` data access)*
- [ ] Invalid index handling
- [ ] NaN/Inf value handling

#### Normal Operations
- [ ] Individual normal get/set *(ref: `points_test.cpp` normal access tests)*
- [ ] Bulk normal operations
- [ ] Normals data pointer access *(ref: `points_test.cpp` normal data access)*
- [ ] Normal vector normalization

#### Covariance Operations
- [ ] Individual covariance get/set *(ref: `points_test.cpp` covariance access tests)*
- [ ] Bulk covariance operations
- [ ] Covariance data pointer access *(ref: `points_test.cpp` covariance data access)*
- [ ] Symmetric matrix verification
- [ ] Positive semi-definite verification

#### Data Validation
- [ ] `has_points` functionality *(ref: `points_test.cpp` trait system)*
- [ ] `has_normals` functionality *(ref: `points_test.cpp` trait system)*
- [ ] `has_covariances` functionality *(ref: `points_test.cpp` trait system)*

### 3. **KdTree Tests** (`test_kdtree.c`)
**C++ Reference**: `kdtree_test.cpp`, `kdtree_synthetic_test.cpp`
- Tests KdTree implementations with different builders (serial, TBB, OMP)
- Tests voxel-based spatial data structures (IncrementalVoxelMap, GaussianVoxelMap)
- Validates k-nearest neighbor and nearest neighbor search accuracy
- Compares against brute-force ground truth
- Synthetic data generation with various distributions

#### Construction
- [ ] Basic KdTree creation *(ref: `kdtree_test.cpp` KdTree construction)*
- [ ] Creation with different builder types *(ref: `kdtree_test.cpp` serial/TBB/OMP builders)*
- [ ] Creation with custom max_leaf_size
- [ ] Empty point cloud handling *(ref: `kdtree_test.cpp` edge cases)*
- [ ] Large point cloud performance *(ref: `kdtree_synthetic_test.cpp` varying sizes)*

#### Search Operations
- [ ] Nearest neighbor search accuracy *(ref: `kdtree_test.cpp` nearest neighbor validation)*
- [ ] K-nearest neighbors search *(ref: `kdtree_test.cpp`, `kdtree_synthetic_test.cpp` k-NN with k=1,2,3,5,10,20)*
- [ ] Radius search
- [ ] Edge cases (query point in cloud, far from cloud) *(ref: `kdtree_synthetic_test.cpp` edge case testing)*
- [ ] Performance comparison between builders *(ref: `kdtree_test.cpp` serial/TBB/OMP comparison)*

#### Advanced Features
- [ ] UnsafeKdTree operations
- [ ] Custom projection types *(ref: `kdtree_synthetic_test.cpp` normal projection variants)*
- [ ] KNN settings (early termination)
- [ ] Thread safety verification *(ref: `kdtree_test.cpp` parallel builder testing)*

### 4. **Voxel Map Tests** (`test_voxelmap.c`)
**C++ Reference**: `kdtree_test.cpp` (voxel map sections)
- Tests IncrementalVoxelMap (IVOX) and GaussianVoxelMap (GVOX)
- Validates k-nearest neighbor search accuracy
- Compares against brute-force ground truth

#### Gaussian Voxel Map
- [ ] Creation with different resolutions *(ref: `kdtree_test.cpp` GVOX creation)*
- [ ] Voxel size validation
- [ ] Point cloud conversion accuracy *(ref: `kdtree_test.cpp` GVOX k-NN validation)*

#### Incremental Voxel Map
- [ ] Point insertion (single/bulk) *(ref: `kdtree_test.cpp` IVOX insertion)*
- [ ] LRU cache behavior
- [ ] Search with different offsets *(ref: `kdtree_test.cpp` IVOX k-NN search)*
- [ ] Update with transformation
- [ ] Memory usage patterns

### 5. **Downsampling Tests** (`test_downsampling.c`)
**C++ Reference**: `downsampling_test.cpp`
- Tests voxel grid downsampling (serial, TBB, OMP variants)
- Tests random sampling algorithms
- Validates against PCL's VoxelGrid filter
- Tests both native PointCloud and PCL point cloud types
- Tests empty point cloud handling

#### Voxel Grid Sampling
- [ ] Different leaf sizes *(ref: `downsampling_test.cpp` voxel grid with various leaf sizes)*
- [ ] Point distribution preservation *(ref: `downsampling_test.cpp` sampling correctness)*
- [ ] Normal/covariance preservation *(ref: `downsampling_test.cpp` attribute preservation)*
- [ ] Edge cases (empty cloud, single point) *(ref: `downsampling_test.cpp` edge case handling)*
- [ ] Parallel backend comparison *(ref: `downsampling_test.cpp` serial/TBB/OMP variants)*

#### Random Sampling
- [ ] Sampling ratios *(ref: `downsampling_test.cpp` random sampling)*
- [ ] Reproducibility with seeds
- [ ] Distribution uniformity *(ref: `downsampling_test.cpp` sampling uniqueness)*
- [ ] Performance scaling

### 6. **Normal Estimation Tests** (`test_normal_estimation.c`)
**C++ Reference**: `normal_estimation_test.cpp`
- Tests normal estimation algorithms (serial, TBB, OMP variants)
- Tests covariance estimation algorithms
- Tests combined normal-covariance estimation
- Validates against PCL's normal estimation
- Tests both native PointCloud and PCL point cloud types

#### Normal Estimation
- [ ] Different k values *(ref: `normal_estimation_test.cpp` various k parameters)*
- [ ] Geometric accuracy *(ref: `normal_estimation_test.cpp` normal validation)*
- [ ] Consistency check *(ref: `normal_estimation_test.cpp` vs PCL comparison)*
- [ ] Parallel backend comparison *(ref: `normal_estimation_test.cpp` serial/TBB/OMP variants)*
- [ ] Edge points handling *(ref: `normal_estimation_test.cpp` edge case testing)*

#### Covariance Estimation
- [ ] Covariance computation accuracy *(ref: `normal_estimation_test.cpp` covariance validation)*
- [ ] Eigenvalue/eigenvector verification
- [ ] Parallel backend comparison *(ref: `normal_estimation_test.cpp` parallel covariance)*

#### Combined Estimation
- [ ] Normal and covariance consistency *(ref: `normal_estimation_test.cpp` combined estimation)*
- [ ] Performance vs separate calls *(ref: `normal_estimation_test.cpp` performance comparison)*

### 7. **Registration Tests** (`test_registration.c`)
**C++ Reference**: `registration_test.cpp` (most comprehensive test)
- Tests all registration variants: ICP, Point-to-Plane ICP, GICP, VGICP
- Tests robust variants (Huber-GICP, Cauchy-GICP)
- Tests serial, TBB, and OpenMP parallel execution
- Tests PCL compatibility interface (RegistrationPCL)
- Validates transformation accuracy against ground truth
- Tests forward/backward registration scenarios

#### Basic Registration
- [ ] ICP convergence *(ref: `registration_test.cpp` ICP tests)*
- [ ] Plane-ICP convergence *(ref: `registration_test.cpp` Point-to-Plane ICP)*
- [ ] GICP convergence *(ref: `registration_test.cpp` GICP tests)*
- [ ] VGICP convergence *(ref: `registration_test.cpp` VGICP tests)*
- [ ] Known transformation recovery *(ref: `registration_test.cpp` ground truth validation)*

#### Configuration
- [ ] Termination criteria *(ref: `registration_test.cpp` convergence criteria)*
- [ ] Optimizer settings
- [ ] Correspondence rejection
- [ ] Robust kernels *(ref: `registration_test.cpp` Huber/Cauchy variants)*
- [ ] DOF restrictions

#### Results Validation
- [ ] Transformation matrix correctness *(ref: `registration_test.cpp` transformation validation)*
- [ ] Rotation matrix properties *(ref: `registration_test.cpp` rotation validation)*
- [ ] Information matrix (extended results) *(ref: `registration_test.cpp` Hessian validation)*
- [ ] Convergence status *(ref: `registration_test.cpp` convergence checking)*

#### Error Cases
- [ ] Non-convergence handling
- [ ] Empty point cloud
- [ ] Insufficient correspondences

### 8. **I/O Tests** (`test_io.c`)
**C++ Reference**: No direct equivalent (uses helper functions in tests)
- Real data loading from `small_gicp/data/` directory
- Ground truth transformation files

- [ ] PLY file loading
- [ ] PLY file saving
- [ ] Round-trip accuracy
- [ ] Error handling (missing files, corrupted data)
- [ ] Large file handling

### 9. **Memory Management Tests** (`test_memory.c`)
**C++ Reference**: No direct equivalent (C++ uses RAII and smart pointers)
- C wrapper specific testing for handle-based resource management

- [ ] Allocation/deallocation patterns
- [ ] Handle lifecycle verification
- [ ] Stress testing (many allocations)
- [ ] Memory leak detection (Valgrind integration)
- [ ] Double-free prevention
- [ ] Null pointer handling

### 10. **Utility Functions Tests** (`test_utilities.c`)
**C++ Reference**: `vector_test.cpp`, `sort_omp_test.cpp`, `sort_tbb_test.cpp`
- Tests utility functions used internally by the C wrapper
- Tests vector operations, hashing, and sorting algorithms
- Tests fast_floor and other optimized operations

#### Vector and Hash Operations
- [ ] 3D integer vector hashing *(ref: `vector_test.cpp` XORVector3iHash)*
- [ ] Fast floor operations *(ref: `vector_test.cpp` fast_floor function)*
- [ ] Hash consistency validation

#### Sorting Operations (if exposed in C wrapper)
- [ ] Radix sort for various integer types *(ref: `sort_tbb_test.cpp`)*
- [ ] Merge sort operations *(ref: `sort_omp_test.cpp`)*
- [ ] Quick sort operations *(ref: `sort_omp_test.cpp`)*
- [ ] Key-value pair sorting with custom extractors

### 11. **Integration Tests**
**C++ Reference**: `helper_test.cpp`
- Tests high-level registration helpers and preprocessing utilities
- Tests `preprocess_points`, `create_gaussian_voxelmap`, and `align` functions
- Validates preprocessing pipeline (downsampling + normal estimation)
- Tests both raw Eigen vectors and PointCloud inputs

#### Preprocessing Pipeline (`test_preprocessing_pipeline.c`)
- [ ] Complete preprocessing workflow *(ref: `helper_test.cpp` preprocess_points)*
- [ ] Result consistency *(ref: `helper_test.cpp` preprocessing validation)*
- [ ] Performance benchmarking

#### Registration Pipeline (`test_registration_pipeline.c`)
- [ ] Full registration workflows *(ref: `helper_test.cpp` align functions)*
- [ ] Real-world data scenarios *(ref: `helper_test.cpp` with real data)*
- [ ] Multi-stage registration *(ref: `helper_test.cpp` forward/backward registration)*

#### Parallel Backends (`test_parallel_backends.c`)
- [ ] OpenMP vs TBB vs Sequential consistency *(ref: multiple test files with parallel variants)*
- [ ] Thread scaling *(ref: performance tests across all modules)*
- [ ] Race condition detection

## Test Data

### Synthetic Data Generation
**C++ Reference**: `kdtree_synthetic_test.cpp`
- Point grids with known properties *(ref: `kdtree_synthetic_test.cpp` grid generation)*
- Transformed point clouds with ground truth *(ref: `kdtree_synthetic_test.cpp` various distributions)*
- Noise injection utilities
- Special cases (planar, spherical, random) *(ref: `kdtree_synthetic_test.cpp` uniform/normal distributions)*

### Real Data
**C++ Reference**: All test files use `small_gicp/data/`
- Use same PLY files as C++ tests (`small_gicp/data/`) *(ref: data loading in all test files)*
- Target and source point clouds *(ref: `registration_test.cpp`, `helper_test.cpp`)*
- Ground truth transformations *(ref: `T_target_source.txt`)*

## Performance Benchmarks

### Timing Requirements
- [ ] KdTree construction: < 100ms for 10k points
- [ ] Nearest neighbor search: < 0.1ms per query
- [ ] Registration: < 1s for typical clouds
- [ ] Downsampling: < 50ms for 100k points

### Memory Requirements
- [ ] Point cloud overhead: < 10% over raw data
- [ ] KdTree memory: < 2x point data
- [ ] No memory leaks in 1000x stress tests

## Test Utilities (`test_common.h/c`)

### Helper Functions
```c
// Assertion helpers
void assert_point_near(double x1, double y1, double z1, 
                      double x2, double y2, double z2, double tol);
void assert_matrix_near(const double* m1, const double* m2, 
                       int size, double tol);
void assert_transformation_valid(const double* T);

// Data generation
void generate_random_points(small_gicp_point_cloud_t* cloud, int n);
void generate_grid_points(small_gicp_point_cloud_t* cloud, 
                         int nx, int ny, int nz);
void add_noise(small_gicp_point_cloud_t* cloud, double sigma);
void transform_cloud(small_gicp_point_cloud_t* cloud, const double* T);

// File utilities
const char* get_test_data_path(const char* filename);
int file_exists(const char* path);

// Timing utilities
typedef struct { double start; } timer_t;
void timer_start(timer_t* timer);
double timer_elapsed(timer_t* timer);

// Memory tracking
void memory_tracking_start();
size_t memory_tracking_get_current();
int memory_tracking_check_leaks();
```

## CI/CD Integration

### Build Matrix
- Compilers: GCC, Clang, MSVC
- C Standards: C99, C11
- Configurations: Debug, Release, RelWithDebInfo
- Optional features: With/without OpenMP, With/without TBB

### Test Execution
```bash
# Run all tests
ctest --output-on-failure

# Run with memory checking
ctest -T memcheck

# Run specific test suite
./test_point_cloud

# Run with verbose output
./test_registration --verbose
```

### Coverage Requirements
- Line coverage: > 90%
- Function coverage: 100%
- Branch coverage: > 80%

## Implementation Timeline

### Phase 1: Foundation (Week 1)
- [x] Set up Unity test framework
- [x] Create test directory structure
- [x] Implement test utilities
- [x] Basic point cloud tests

### Phase 2: Core Features (Week 2)
- [ ] KdTree tests
- [ ] Downsampling tests
- [ ] Normal estimation tests
- [ ] Basic registration tests

### Phase 3: Advanced Features (Week 3)
- [ ] Voxel map tests
- [ ] Advanced registration tests
- [ ] I/O tests
- [ ] Memory management tests

### Phase 4: Integration & Polish (Week 4)
- [ ] Integration tests
- [ ] Performance benchmarks
- [ ] CI/CD setup
- [ ] Documentation

## Success Criteria

1. **Coverage**: All public APIs have corresponding tests
2. **Reliability**: Tests pass consistently across platforms
3. **Performance**: No performance regressions vs C++
4. **Memory Safety**: Zero memory leaks detected
5. **Maintainability**: Clear test structure and documentation

## Notes for Implementation

1. **Test Independence**: Each test should be independent and not rely on execution order
2. **Resource Cleanup**: Always clean up allocated resources in teardown
3. **Error Paths**: Test both success and failure cases
4. **Platform Differences**: Account for floating-point differences across platforms
5. **Parallel Testing**: Ensure tests can run in parallel without interference

## Example Test Pattern

```c
// test_point_cloud.c
#include "unity.h"
#include "test_common.h"
#include <small_gicp_c.h>

static small_gicp_point_cloud_t* test_cloud = NULL;

void setUp(void) {
    small_gicp_error_t error = small_gicp_point_cloud_create(&test_cloud);
    TEST_ASSERT_EQUAL(SMALL_GICP_SUCCESS, error);
}

void tearDown(void) {
    if (test_cloud) {
        small_gicp_point_cloud_destroy(test_cloud);
        test_cloud = NULL;
    }
}

void test_point_cloud_creation(void) {
    TEST_ASSERT_NOT_NULL(test_cloud);
    
    size_t size = 0;
    small_gicp_error_t error = small_gicp_point_cloud_size(test_cloud, &size);
    TEST_ASSERT_EQUAL(SMALL_GICP_SUCCESS, error);
    TEST_ASSERT_EQUAL(0, size);
}

void test_point_cloud_resize(void) {
    small_gicp_error_t error = small_gicp_point_cloud_resize(test_cloud, 100);
    TEST_ASSERT_EQUAL(SMALL_GICP_SUCCESS, error);
    
    size_t size = 0;
    error = small_gicp_point_cloud_size(test_cloud, &size);
    TEST_ASSERT_EQUAL(SMALL_GICP_SUCCESS, error);
    TEST_ASSERT_EQUAL(100, size);
}

// ... more tests ...

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_point_cloud_creation);
    RUN_TEST(test_point_cloud_resize);
    // ... register more tests ...
    return UNITY_END();
}
```

This comprehensive testing plan ensures the C wrapper maintains the same quality and reliability standards as the original C++ implementation.
