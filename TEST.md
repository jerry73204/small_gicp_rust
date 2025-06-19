# Rust API Testing Framework

This document outlines the comprehensive testing strategy for the small_gicp Rust API that leverages upstream C++ test fixtures and maintains correspondence with C++ tests for validation and regression prevention.

## Overview

The testing framework reuses test fixtures from the upstream C++ codebase (`small_gicp/`) to ensure:
- **Consistency**: Rust and C++ implementations produce equivalent results on identical data
- **Regression Prevention**: Changes don't break compatibility with upstream behavior  
- **CI/CD Compatibility**: Tests run reliably in automated environments
- **Traceability**: Clear mapping between Rust tests and their C++ counterparts

## Test Data and Fixtures

### Shared Test Data
All test data is sourced from `small_gicp/data/`:
- **Point Clouds**: `target.ply`, `source.ply` - Real point cloud data for registration tests
- **Ground Truth**: `T_target_source.txt` - Known transformation matrix for validation
- **Synthetic Data**: Generated procedurally for specific test scenarios

### Test Data Access Pattern
```rust
// Example: Load shared test data in Rust tests
fn load_test_data() -> (PointCloud, PointCloud, Matrix4<f64>) {
    let target = PointCloud::load_ply("small_gicp/data/target.ply")?;
    let source = PointCloud::load_ply("small_gicp/data/source.ply")?;
    let ground_truth = load_transformation_matrix("small_gicp/data/T_target_source.txt")?;
    (target, source, ground_truth)
}
```

## Test Mapping Strategy

### 1. Direct C++ Correspondence Tests
Each major C++ test class has a corresponding Rust test module:

| C++ Test File                | Rust Test Module                                 | Purpose                           | Reference   |
|------------------------------|--------------------------------------------------|-----------------------------------|-------------|
| `kdtree_test.cpp`            | `tests/kdtree_correspondence_test.rs`            | KdTree operations validation      | Lines 20-50 |
| `registration_test.cpp`      | `tests/registration_correspondence_test.rs`      | Registration algorithm validation | Lines 25-50 |
| `downsampling_test.cpp`      | `tests/downsampling_correspondence_test.rs`      | Downsampling accuracy validation  | Lines 18-50 |
| `normal_estimation_test.cpp` | `tests/normal_estimation_correspondence_test.rs` | Normal estimation accuracy        | -           |
| `points_test.cpp`            | `tests/point_cloud_correspondence_test.rs`       | Point cloud operations            | -           |

### 2. Test Case Mapping
Each Rust test explicitly references its C++ counterpart:

```rust
/// Corresponds to KdTreeTest::SetUp() in kdtree_test.cpp:22-41
/// Tests k-nearest neighbor search accuracy against brute force reference
#[test]
fn test_knn_search_correspondence() {
    // Reference: kdtree_test.cpp lines 22-41
    // Implementation mirrors C++ test setup and validation
}
```

### 3. Fixture Replication
Rust tests replicate C++ test fixtures exactly:

```rust
// Corresponds to DownsamplingTest::SetUp() in downsampling_test.cpp:20-41
struct DownsamplingTestFixture {
    points: PointCloud,           // From data/target.ply
    resolutions: Vec<f64>,        // [0.1, 0.5, 1.0]
    downsampled_reference: Vec<PointCloud>, // PCL reference results
}
```

## Test Categories

### 1. Accuracy Tests
**Purpose**: Verify Rust results match C++ results within tolerance
**Pattern**: Load same data, run same algorithm, compare outputs
**Example**:
```rust
#[test]
fn test_voxel_downsampling_accuracy() {
    // Reference: downsampling_test.cpp:45-50
    let fixture = DownsamplingTestFixture::new();
    for (i, &resolution) in fixture.resolutions.iter().enumerate() {
        let rust_result = Preprocessing::voxel_downsample(&fixture.points, resolution, 1);
        let cpp_reference = &fixture.downsampled_reference[i];
        
        // Verify point count matches within tolerance
        assert_points_equivalent(&rust_result, cpp_reference, 1e-6);
    }
}
```

### 2. Performance Parity Tests
**Purpose**: Ensure Rust wrapper doesn't introduce significant overhead
**Pattern**: Benchmark identical operations, compare timing
**Example**:
```rust
#[test]
fn test_kdtree_performance_parity() {
    // Reference: kdtree_test.cpp timing patterns
    let fixture = KdTreeTestFixture::new();
    let start = Instant::now();
    let tree = KdTree::new(&fixture.points, 1).unwrap();
    let rust_build_time = start.elapsed();
    
    // Should be within 20% of C++ equivalent (accounting for FFI overhead)
    assert!(rust_build_time.as_millis() < cpp_reference_time_ms * 120 / 100);
}
```

### 3. Edge Case Tests
**Purpose**: Verify robust handling of boundary conditions
**Pattern**: Test empty clouds, single points, degenerate cases
**Example**:
```rust
#[test]
fn test_empty_cloud_handling() {
    // Reference: points_test.cpp edge case patterns
    let empty_cloud = PointCloud::new().unwrap();
    
    // All operations should handle empty input gracefully
    assert!(KdTree::new(&empty_cloud, 1).is_err());
    assert_eq!(Preprocessing::voxel_downsample(&empty_cloud, 1.0, 1).size(), 0);
}
```

### 4. API Contract Tests
**Purpose**: Verify Rust API contracts match C++ behavior
**Pattern**: Test parameter validation, error conditions, state management
**Example**:
```rust
#[test]
fn test_parameter_validation() {
    // Reference: C++ constructor/method parameter validation
    let cloud = load_test_cloud();
    
    // Invalid parameters should produce consistent errors
    assert!(KdTree::new(&cloud, 0).is_err());  // Invalid thread count
    assert!(Preprocessing::voxel_downsample(&cloud, -1.0, 1).size() == 0); // Invalid voxel size
}
```

## Test Organization

### Directory Structure

The test infrastructure is organized within the `small-gicp-rust` package directory using a flat structure with naming conventions:

```
small-gicp-rust/
├── tests/                                  # Integration tests directory
│   ├── common/                            # Shared test utilities module
│   │   ├── mod.rs                        # Module declarations
│   │   ├── data_loader.rs                # C++ test data loading utilities
│   │   └── comparison_utils.rs           # Result comparison utilities
│   ├── correspondence_*.rs               # C++ correspondence tests
│   │   ├── correspondence_kdtree_test.rs
│   │   ├── correspondence_registration_test.rs
│   │   ├── correspondence_downsampling_test.rs
│   │   ├── correspondence_point_cloud_test.rs
│   │   └── correspondence_normal_estimation_test.rs
│   ├── integration_*.rs                   # End-to-end workflow tests
│   │   ├── integration_full_pipeline_test.rs
│   │   └── integration_performance_test.rs
│   ├── regression_*.rs                    # Regression prevention tests
│   │   └── regression_api_stability_test.rs
│   └── [existing test files]              # Pre-existing integration tests
│       ├── basic_integration_test.rs
│       ├── borrowed_kdtree_test.rs
│       ├── covariance_direct_access_tests.rs
│       └── trait_tests.rs
```

**Note**: The test organization uses Cargo conventions:
- Each `.rs` file directly in `tests/` is compiled as a separate test binary
- Test files are prefixed by category (correspondence_, integration_, regression_)
- The `common/` module is shared across all tests using `#[path = "common/mod.rs"]`
- This flat structure ensures all tests are automatically discovered by Cargo

### How to Use the Test Infrastructure

All test files follow the same pattern for importing common utilities:

```rust
// tests/correspondence_example_test.rs
#[path = "common/mod.rs"]
mod common;

use common::prelude::*;

#[test]
fn test_example_correspondence() {
    // Use test fixtures
    let data_loader = TestDataLoader::default();
    let cloud = data_loader.load_target_cloud().unwrap();
    
    // Use comparison utilities
    let result = assert_values_equivalent(1.0, 1.0001, DEFAULT_TOLERANCE, 0.01);
    assert!(result.passed);
}
```

### Running Test Categories

With the flat structure and naming conventions, you can easily run specific test categories:

```bash
# Run all correspondence tests
cargo test correspondence_

# Run all integration tests  
cargo test integration_

# Run all regression tests
cargo test regression_

# Run a specific test file
cargo test --test correspondence_kdtree_test

# Run with ignored tests (placeholders)
cargo test correspondence_ -- --ignored
```

### Test Utilities
```rust
// tests/fixtures/comparison_utils.rs
pub fn assert_points_equivalent(rust: &PointCloud, cpp_ref: &PointCloud, tolerance: f64) {
    assert_eq!(rust.size(), cpp_ref.size());
    for i in 0..rust.size() {
        let r_point = rust.point_at(i);
        let c_point = cpp_ref.point_at(i);
        assert!((r_point - c_point).norm() < tolerance);
    }
}

pub fn load_cpp_test_results(test_name: &str) -> TestResults {
    // Load pre-computed C++ results for comparison
}
```

## CI/CD Integration

### Test Execution Strategy
```yaml
# .github/workflows/test.yml
- name: Run Correspondence Tests
  run: |
    # Ensure test data is available
    git submodule update --init small_gicp
    
    # Run Rust tests with C++ reference data
    cargo test --test "*_correspondence_test" -- --test-threads=1
    
    # Generate test coverage report
    cargo tarpaulin --out xml --output-dir coverage/
```

### Environment Requirements
- **Data Availability**: `small_gicp/data/` must be accessible
- **Deterministic Results**: Tests use fixed seeds for reproducibility
- **Resource Constraints**: Tests scale appropriately for CI resources
- **Platform Independence**: Tests work across Linux, macOS, Windows

## Validation Methodology

### 1. Numerical Accuracy
- **Point Coordinates**: Within 1e-6 absolute tolerance
- **Transformation Matrices**: Within 1e-9 for rotation, 1e-6 for translation
- **Distances**: Within 1e-8 relative tolerance
- **Counts**: Exact match (point counts, neighbor counts)

### 2. Algorithmic Equivalence
- **Search Results**: Same nearest neighbors found (order may vary)
- **Convergence**: Same final registration result within tolerance
- **Edge Cases**: Identical handling of degenerate inputs

### 3. Performance Benchmarks
- **Build Times**: Tree construction within 200% of C++ time
- **Query Times**: Search operations within 150% of C++ time
- **Memory Usage**: Within 120% of C++ memory footprint

## Test Documentation

### Test Annotations
Every test includes:
```rust
/// **C++ Reference**: `kdtree_test.cpp:75-95`
/// **Purpose**: Validates k-NN search returns correct neighbors in correct order
/// **Fixture**: Uses `data/target.ply` with 20-NN queries
/// **Tolerance**: Point distances within 1e-8, indices exact match
#[test]
fn test_knn_search_order() {
    // Test implementation
}
```

### Cross-Reference Index
Maintain `CROSS_REFERENCE.md` mapping:
- Rust test → C++ test location
- Test data dependencies
- Expected tolerances
- Known differences/limitations

## Maintenance Strategy

### 1. Upstream Sync
- **Quarterly Review**: Check for new C++ tests to port
- **Version Tracking**: Maintain compatibility with upstream changes
- **Data Updates**: Sync test data when upstream changes

### 2. Test Evolution
- **Add Coverage**: Create Rust tests for new C++ functionality
- **Deprecation**: Mark tests when C++ equivalents are removed
- **Enhancement**: Improve test robustness and coverage

### 3. Failure Analysis
- **Tolerance Tuning**: Adjust based on empirical variance
- **Platform Differences**: Account for floating-point behavior
- **Upstream Changes**: Update when C++ behavior changes

## Progress Tracking

### Overall Progress Dashboard

| Component                   | Status       | Tests Implemented | Tests Passing | Coverage | Notes                              |
|-----------------------------|--------------|-------------------|---------------|----------|------------------------------------|
| **Test Infrastructure**     | 🟢 Complete  | 5/5               | 5/5           | 100%     | Foundation ready                   |
| **Unit Tests**              |              |                   |               |          |                                    |
| - KdTree Unit Tests         | ⏳ Pending   | 0/8               | 0/8           | 0%       | `src/kdtree/tests.rs`              |
| - Preprocessing Unit Tests  | ⏳ Pending   | 0/11              | 0/11          | 0%       | `src/preprocessing/tests.rs`       |
| - Point Cloud Unit Tests    | ⏳ Pending   | 0/7               | 0/7           | 0%       | `src/point_cloud/tests.rs`         |
| - VoxelMap Unit Tests       | ⏳ Pending   | 0/5               | 0/5           | 0%       | `src/voxelmap/tests.rs`            |
| **Integration Tests**       |              |                   |               |          |                                    |
| - Registration Integration  | ⏳ Pending   | 0/15              | 0/15          | 0%       | `tests/integration_registration_test.rs` |
| - Full Pipeline Tests       | ⏳ Pending   | 0/5               | 0/5           | 0%       | `tests/integration_full_pipeline_test.rs` |
| - Performance Tests         | ⏳ Pending   | 0/8               | 0/8           | 0%       | `tests/integration_performance_test.rs` |
| **CI/CD Integration**       | 🟢 Complete  | 3/3               | 3/3           | 100%     | GitHub Actions configured          |

**Legend**: 🟢 Complete | 🔄 In Progress | ⏳ Pending | 🔴 Failed | ⚠️ Issues

### Test Infrastructure Progress

| Task                              | Status       | Assignee | Due Date | Notes                                              |
|-----------------------------------|--------------|----------|----------|---------------------------------------------------|
| **Data Loading Utilities**        | 🟢 Complete  | -        | Week 1   | `small-gicp-rust/tests/fixtures/data_loader.rs`   |
| **Comparison Utilities**          | 🟢 Complete  | -        | Week 1   | `small-gicp-rust/tests/fixtures/comparison_utils.rs` |
| **Test Fixtures Module**          | 🟢 Complete  | -        | Week 1   | `small-gicp-rust/tests/fixtures/mod.rs`           |
| **CI Configuration**              | 🟢 Complete  | -        | Week 1   | `.github/workflows/test.yml`                       |
| **Cross-Reference Documentation** | 🟢 Complete  | -        | Week 1   | `CROSS_REFERENCE.md`                               |

### Correspondence Tests Progress

#### KdTree Tests (`tests/correspondence_kdtree_test.rs`)

| Test Function                                                                                                  | C++ Reference                                                            | Status     | Implementation Progress | Notes                        |
|----------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------|------------|-------------------------|------------------------------|
| `test_kdtree_build_accuracy`                                                                                   | `kdtree_test.cpp:22-41`                                                  | ⏳ Pending | 0%                      | Tree construction validation |
| `test_knn_search_correspondence`                                                                               | `Code` kdtree_test.cpp:44-70` | ⏳ Pending | 0% | K-NN search accuracy | |            |                         |                              |
| | `test_radius_search_correspondence` | `kdtree_test.cpp:72-95` | ⏳ Pending | 0% | Radius search validation | |                                                                          |            |                         |                              |
| | `test_nearest_neighbor_correspondence` | `kdtree_test.cpp:98-115` | ⏳ Pending | 0% | Single NN search |     |                                                                          |            |                         |                              |
| | `test_kdtree_parallel_backends` | `kdtree_test.cpp:118-140` | ⏳ Pending | 0% | OpenMP/TBB validation |      |                                                                          |            |                         |                              |
| | `test_kdtree_memory_efficiency` | `kdtree_test.cpp:143-160` | ⏳ Pending | 0% | Memory usage validation |    |                                                                          |            |                         |                              |
| | `test_kdtree_large_datasets` | `kdtree_test.cpp:163-180` | ⏳ Pending | 0% | Scalability testing |           |                                                                          |            |                         |                              |
| | `test_kdtree_edge_cases` | `kdtree_test.cpp:183-200`                                                         | ⏳ Pending                                                               | 0%         | Boundary conditions     |                              |

#### Downsampling Tests (`tests/correspondence_downsampling_test.rs`)

| Test Function                          | C++ Reference                   | Status     | Implementation Progress | Notes                      |
|----------------------------------------|---------------------------------|------------|-------------------------|----------------------------|
| `test_voxel_downsampling_accuracy`     | `downsampling_test.cpp:45-60`   | ⏳ Pending | 0%                      | Voxel grid accuracy        |
| `test_random_downsampling_accuracy`    | `downsampling_test.cpp:63-78`   | ⏳ Pending | 0%                      | Random sampling validation |
| `test_downsampling_parallel_backends`  | `downsampling_test.cpp:81-98`   | ⏳ Pending | 0%                      | OpenMP/TBB validation      |
| `test_downsampling_edge_cases`         | `downsampling_test.cpp:101-118` | ⏳ Pending | 0%                      | Empty/single point clouds  |
| `test_downsampling_resolution_scaling` | `downsampling_test.cpp:121-138` | ⏳ Pending | 0%                      | Multi-resolution testing   |
| `test_downsampling_memory_efficiency`  | `downsampling_test.cpp:141-158` | ⏳ Pending | 0%                      | Memory usage validation    |

#### Registration Tests (`tests/correspondence_registration_test.rs`)

| Test Function                         | C++ Reference                   | Status     | Implementation Progress | Notes                      |
|---------------------------------------|---------------------------------|------------|-------------------------|----------------------------|
| `test_icp_registration_accuracy`      | `registration_test.cpp:27-50`   | ⏳ Pending | 0%                      | ICP algorithm validation   |
| `test_point_to_plane_icp_accuracy`    | `registration_test.cpp:53-75`   | ⏳ Pending | 0%                      | Point-to-plane ICP         |
| `test_gicp_registration_accuracy`     | `registration_test.cpp:78-100`  | ⏳ Pending | 0%                      | GICP algorithm validation  |
| `test_vgicp_registration_accuracy`    | `registration_test.cpp:103-125` | ⏳ Pending | 0%                      | VGICP algorithm validation |
| `test_registration_convergence`       | `registration_test.cpp:128-150` | ⏳ Pending | 0%                      | Convergence criteria       |
| `test_registration_robust_kernels`    | `registration_test.cpp:153-175` | ⏳ Pending | 0%                      | Outlier rejection          |
| `test_registration_initial_guess`     | `registration_test.cpp:178-200` | ⏳ Pending | 0%                      | Initial transformation     |
| `test_registration_termination`       | `registration_test.cpp:203-225` | ⏳ Pending | 0%                      | Termination criteria       |
| `test_registration_parallel_backends` | `registration_test.cpp:228-250` | ⏳ Pending | 0%                      | OpenMP/TBB validation      |
| `test_registration_memory_efficiency` | `registration_test.cpp:253-275` | ⏳ Pending | 0%                      | Memory usage               |
| `test_registration_edge_cases`        | `registration_test.cpp:278-300` | ⏳ Pending | 0%                      | Degenerate cases           |
| `test_registration_real_world_data`   | `registration_test.cpp:303-325` | ⏳ Pending | 0%                      | Real dataset validation    |

#### Point Cloud Tests (`tests/correspondence_point_cloud_test.rs`)

| Test Function                      | C++ Reference             | Status     | Implementation Progress | Notes                     |
|------------------------------------|---------------------------|------------|-------------------------|---------------------------|
| `test_point_cloud_creation`        | `points_test.cpp:25-45`   | ⏳ Pending | 0%                      | Cloud construction        |
| `test_point_access_patterns`       | `points_test.cpp:48-68`   | ⏳ Pending | 0%                      | Point access methods      |
| `test_normal_covariance_storage`   | `points_test.cpp:71-91`   | ⏳ Pending | 0%                      | Attribute storage         |
| `test_point_cloud_transformations` | `points_test.cpp:94-114`  | ⏳ Pending | 0%                      | Transformation operations |
| `test_point_cloud_io_operations`   | `points_test.cpp:117-137` | ⏳ Pending | 0%                      | File I/O validation       |
| `test_point_cloud_memory_layout`   | `points_test.cpp:140-160` | ⏳ Pending | 0%                      | Memory efficiency         |
| `test_point_cloud_edge_cases`      | `points_test.cpp:163-183` | ⏳ Pending | 0%                      | Boundary conditions       |

#### Normal Estimation Tests (`tests/correspondence_normal_estimation_test.rs`)

| Test Function                         | C++ Reference                        | Status     | Implementation Progress | Notes                       |
|---------------------------------------|--------------------------------------|------------|-------------------------|-----------------------------|
| `test_normal_estimation_accuracy`     | `normal_estimation_test.cpp:30-50`   | ⏳ Pending | 0%                      | Normal computation accuracy |
| `test_covariance_estimation_accuracy` | `normal_estimation_test.cpp:53-73`   | ⏳ Pending | 0%                      | Covariance computation      |
| `test_estimation_parallel_backends`   | `normal_estimation_test.cpp:76-96`   | ⏳ Pending | 0%                      | OpenMP/TBB validation       |
| `test_estimation_neighbor_selection`  | `normal_estimation_test.cpp:99-119`  | ⏳ Pending | 0%                      | Neighbor count effects      |
| `test_estimation_edge_cases`          | `normal_estimation_test.cpp:122-142` | ⏳ Pending | 0%                      | Sparse/dense clouds         |

### Performance Tests Progress

| Test Category                | Total Tests | Implemented | Passing | Target Performance  | Notes                    |
|------------------------------|-------------|-------------|---------|---------------------|--------------------------|
| **KdTree Build Performance** | 2           | 0           | 0       | <200% of C++ time   | Tree construction timing |
| **Search Performance**       | 3           | 0           | 0       | <150% of C++ time   | Query operation timing   |
| **Registration Performance** | 2           | 0           | 0       | <200% of C++ time   | Algorithm timing         |
| **Memory Usage**             | 1           | 0           | 0       | <120% of C++ memory | Memory profiling         |

### CI/CD Integration Progress

| Component                   | Status     | Implementation Progress | Notes                        |
|-----------------------------|------------|-------------------------|------------------------------|
| **GitHub Actions Workflow** | ⏳ Pending | 0%                      | `.github/workflows/test.yml` |
| **Test Data Management**    | ⏳ Pending | 0%                      | Submodule integration        |
| **Coverage Reporting**      | ⏳ Pending | 0%                      | Tarpaulin integration        |

## Implementation Timeline

### Phase 1: Core Correspondence Tests (Week 1-2)
- [ ] Set up test data loading utilities
- [ ] Implement KdTree correspondence tests
- [ ] Implement downsampling correspondence tests
- [ ] Create CI pipeline integration

### Phase 2: Registration Tests (Week 3)
- [ ] Implement registration correspondence tests
- [ ] Add performance parity tests
- [ ] Create comprehensive error condition tests

### Phase 3: Edge Cases and Polish (Week 4)
- [ ] Add edge case and robustness tests
- [ ] Implement regression test suite
- [ ] Documentation and cross-reference creation
- [ ] CI/CD optimization and reliability improvements

## Success Metrics

- **Coverage**: 95% of C++ test scenarios have Rust equivalents
- **Accuracy**: All tests pass with defined tolerances
- **Reliability**: <1% flaky test rate in CI
- **Performance**: Rust API within 2x of C++ performance
- **Maintenance**: Test updates completed within 1 week of upstream changes

---

*This testing framework ensures the Rust API maintains high fidelity with the proven C++ implementation while providing confidence for ongoing development and maintenance.*
