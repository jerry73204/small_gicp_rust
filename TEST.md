# Rust Testing Framework for small_gicp

This document describes the comprehensive testing strategy for the small_gicp Rust API, ensuring compatibility with the C++ implementation through systematic testing of all components.

## Overview

The Rust testing framework mirrors the C++ test structure while following Rust conventions:
- **Unit tests** are located in `src/{module}/tests.rs` files
- **Integration tests** are located in `src/tests/` 
- **Test fixtures** use symlinks to C++ test data in `small_gicp/data/`
- **Test utilities** provide common functionality for validation

## Test Organization

### Directory Structure

```
small-gicp-rust/
├── src/
│   ├── kdtree/
│   │   ├── mod.rs
│   │   └── tests.rs                    # Unit tests for KdTree
│   ├── preprocessing/
│   │   ├── mod.rs
│   │   └── tests.rs                    # Unit tests for downsampling & normal estimation
│   ├── point_cloud/
│   │   ├── mod.rs
│   │   └── tests.rs                    # Unit tests for PointCloud
│   ├── registration/
│   │   ├── mod.rs
│   │   └── tests.rs                    # Unit tests for registration algorithms
│   ├── voxelmap/
│   │   ├── mod.rs
│   │   └── tests.rs                    # Unit tests for VoxelMap
│   └── tests/                          # Integration tests
│       ├── mod.rs
│       ├── registration_test.rs        # Full registration workflows
│       ├── helper_test.rs              # High-level API tests
│       └── common/                     # Shared test utilities
│           ├── mod.rs
│           ├── fixtures.rs             # Test fixture management
│           └── validation.rs           # Comparison utilities
├── data/                               # Symlinks to C++ test data
│   ├── source.ply -> ../../small_gicp/data/source.ply
│   ├── target.ply -> ../../small_gicp/data/target.ply
│   └── T_target_source.txt -> ../../small_gicp/data/T_target_source.txt
└── Cargo.toml
```

### Test Data Management

Test data is shared with the C++ implementation through symlinks:

```bash
# Create symlinks to C++ test data
cd small-gicp-rust
mkdir -p data
ln -s ../../small_gicp/data/source.ply data/source.ply
ln -s ../../small_gicp/data/target.ply data/target.ply
ln -s ../../small_gicp/data/T_target_source.txt data/T_target_source.txt
```

## C++ to Rust Test Mapping

### Unit Tests

| C++ Test File                | Rust Location                | Test Focus                                      |
|------------------------------|------------------------------|-------------------------------------------------|
| `kdtree_test.cpp`            | `src/kdtree/tests.rs`        | KdTree construction, k-NN search, radius search |
| `downsampling_test.cpp`      | `src/preprocessing/tests.rs` | Voxel grid & random downsampling                |
| `normal_estimation_test.cpp` | `src/preprocessing/tests.rs` | Normal & covariance estimation                  |
| `points_test.cpp`            | `src/point_cloud/tests.rs`   | Point cloud traits & operations                 |
| `kdtree_synthetic_test.cpp`  | `src/kdtree/tests.rs`        | Synthetic data distributions                    |

### Integration Tests

| C++ Test File           | Rust Location                    | Test Focus                      |
|-------------------------|----------------------------------|---------------------------------|
| `registration_test.cpp` | `src/tests/registration_test.rs` | Complete registration pipelines |
| `helper_test.cpp`       | `src/tests/helper_test.rs`       | High-level convenience APIs     |

## Test Implementation Status

### Unit Tests Progress

#### KdTree Tests (`src/kdtree/tests.rs`)
| Test Case                    | C++ Reference               | Status     | Progress | Notes                               |
|------------------------------|-----------------------------|------------|----------|-------------------------------------|
| `test_load_check`            | `kdtree_test.cpp:LoadCheck` | ⏳ Pending | 0%       | Verify test data loading            |
| `test_empty_kdtree`          | `kdtree_test.cpp:EmptyTest` | ⏳ Pending | 0%       | Empty point cloud handling          |
| `test_knn_search`            | `kdtree_test.cpp:KnnTest`   | ⏳ Pending | 0%       | k-NN validation against brute force |
| `test_radius_search`         | `kdtree_test.cpp`           | ⏳ Pending | 0%       | Radius search validation            |
| `test_parallel_construction` | `kdtree_test.cpp`           | ⏳ Pending | 0%       | Multi-threaded tree building        |
| `test_synthetic_uniform`     | `kdtree_synthetic_test.cpp` | ⏳ Pending | 0%       | Uniform distribution                |
| `test_synthetic_normal`      | `kdtree_synthetic_test.cpp` | ⏳ Pending | 0%       | Gaussian distribution               |
| `test_synthetic_clustered`   | `kdtree_synthetic_test.cpp` | ⏳ Pending | 0%       | Clustered points                    |

#### Preprocessing Tests (`src/preprocessing/tests.rs`)
| Test Case                    | C++ Reference                                     | Status     | Progress | Notes                    |
|------------------------------|---------------------------------------------------|------------|----------|--------------------------|
| **Downsampling**             |                                                   |            |          |                          |
| `test_empty_downsample`      | `downsampling_test.cpp:EmptyTest`                 | ⏳ Pending | 0%       | Empty cloud downsampling |
| `test_voxel_downsample`      | `downsampling_test.cpp:DownsampleTest`            | ⏳ Pending | 0%       | Voxel grid sampling      |
| `test_random_downsample`     | `downsampling_test.cpp:RandomSamplingTest`        | ⏳ Pending | 0%       | Random sampling          |
| **Normal Estimation**        |                                                   |            |          |                          |
| `test_empty_normals`         | `normal_estimation_test.cpp:EmptyTest`            | ⏳ Pending | 0%       | Empty cloud normals      |
| `test_normal_estimation`     | `normal_estimation_test.cpp:NormalEstimationTest` | ⏳ Pending | 0%       | Normal computation       |
| `test_covariance_estimation` | `normal_estimation_test.cpp`                      | ⏳ Pending | 0%       | Covariance matrices      |

#### Point Cloud Tests (`src/point_cloud/tests.rs`)
| Test Case                    | C++ Reference     | Status     | Progress | Notes                |
|------------------------------|-------------------|------------|----------|----------------------|
| `test_traits_implementation` | `points_test.cpp` | ⏳ Pending | 0%       | Trait compliance     |
| `test_point_access`          | `points_test.cpp` | ⏳ Pending | 0%       | Point accessors      |
| `test_normal_access`         | `points_test.cpp` | ⏳ Pending | 0%       | Normal accessors     |
| `test_covariance_access`     | `points_test.cpp` | ⏳ Pending | 0%       | Covariance accessors |
| `test_resize_operations`     | `points_test.cpp` | ⏳ Pending | 0%       | Dynamic resizing     |
| `test_ply_io`                | N/A               | ⏳ Pending | 0%       | PLY file I/O         |

#### VoxelMap Tests (`src/voxelmap/tests.rs`)
| Test Case                       | C++ Reference                      | Status     | Progress | Notes                 |
|---------------------------------|------------------------------------|------------|----------|-----------------------|
| `test_incremental_construction` | `kdtree_test.cpp:KnnTest`          | ⏳ Pending | 0%       | Incremental voxel map |
| `test_gaussian_voxels`          | `kdtree_test.cpp:KnnTest`          | ⏳ Pending | 0%       | Gaussian voxel map    |
| `test_voxel_queries`            | `helper_test.cpp:GaussianVoxelMap` | ⏳ Pending | 0%       | Spatial queries       |

### Integration Tests Progress

#### Registration Tests (`src/tests/registration_test.rs`)
| Test Case                  | C++ Reference                                       | Status     | Progress | Notes                 |
|----------------------------|-----------------------------------------------------|------------|----------|-----------------------|
| `test_icp_registration`    | `registration_test.cpp:RegistrationTest/ICP`        | ⏳ Pending | 0%       | Basic ICP             |
| `test_plane_icp`           | `registration_test.cpp:RegistrationTest/PLANE_ICP`  | ⏳ Pending | 0%       | Point-to-plane ICP    |
| `test_gicp`                | `registration_test.cpp:RegistrationTest/GICP`       | ⏳ Pending | 0%       | GICP algorithm        |
| `test_vgicp`               | `registration_test.cpp:RegistrationTest/VGICP`      | ⏳ Pending | 0%       | Voxelized GICP        |
| `test_robust_registration` | `registration_test.cpp:RegistrationTest/HUBER_GICP` | ⏳ Pending | 0%       | Robust kernels        |
| `test_forward_backward`    | `registration_test.cpp`                             | ⏳ Pending | 0%       | Transform consistency |

#### Helper Tests (`src/tests/helper_test.rs`)
| Test Case                | C++ Reference                      | Status     | Progress | Notes                  |
|--------------------------|------------------------------------|------------|----------|------------------------|
| `test_preprocess_points` | `helper_test.cpp:Preprocess`       | ⏳ Pending | 0%       | Preprocessing pipeline |
| `test_create_voxelmap`   | `helper_test.cpp:GaussianVoxelMap` | ⏳ Pending | 0%       | Voxel map creation     |
| `test_align_points`      | `helper_test.cpp:Align`            | ⏳ Pending | 0%       | High-level alignment   |

## Test Fixtures and Utilities

### Common Test Fixtures

```rust
// src/tests/common/fixtures.rs

/// Standard test fixture matching C++ KdTreeTest
pub struct KdTreeTestFixture {
    pub points: PointCloud,
    pub queries: Vec<Point3<f64>>,
    pub ground_truth_knn: Vec<Vec<(usize, f64)>>,
}

/// Standard test fixture matching C++ RegistrationTest
pub struct RegistrationTestFixture {
    pub source: PointCloud,
    pub target: PointCloud,
    pub ground_truth: Matrix4<f64>,
    pub target_tree: KdTree,
    pub source_tree: KdTree,
}

/// Load standard test data
pub fn load_test_data() -> Result<(PointCloud, PointCloud, Matrix4<f64>)> {
    let source = PointCloud::load_ply("data/source.ply")?;
    let target = PointCloud::load_ply("data/target.ply")?;
    let transform = load_transformation_matrix("data/T_target_source.txt")?;
    Ok((source, target, transform))
}
```

### Validation Utilities

```rust
// src/tests/common/validation.rs

/// Compare transformations with C++ tolerances
pub fn assert_transform_equal(
    actual: &Matrix4<f64>,
    expected: &Matrix4<f64>,
    rotation_tol_deg: f64,  // Default: 2.5°
    translation_tol: f64,    // Default: 0.2
) -> Result<()>;

/// Validate k-NN results against brute force
pub fn validate_knn_results(
    tree_results: &[(usize, f64)],
    brute_force: &[(usize, f64)],
    k: usize,
) -> Result<()>;

/// Brute force k-NN for validation
pub fn brute_force_knn(
    points: &PointCloud,
    query: &Point3<f64>,
    k: usize,
) -> Vec<(usize, f64)>;
```

## Test Parameters

### Default Test Parameters (matching C++)
- **Downsampling resolution**: 0.3 (registration), 0.5 (KdTree), 0.25 (normals)
- **Normal estimation neighbors**: 20
- **KdTree test queries**: 50 queries × 3 types = 150 total
- **Registration tolerance**: 2.5° rotation, 0.2 translation
- **Thread counts**: 1, 4, 8 (for parallel tests)

### Test Data Generation

```rust
/// Generate synthetic point distributions
pub fn generate_uniform_points(n: usize, bounds: Bounds) -> PointCloud;
pub fn generate_normal_points(n: usize, mean: Point3<f64>, stddev: f64) -> PointCloud;
pub fn generate_clustered_points(n_clusters: usize, points_per_cluster: usize) -> PointCloud;
```

## Running Tests

```bash
# Run all unit tests
cargo test --lib

# Run specific module tests
cargo test --lib kdtree::tests
cargo test --lib preprocessing::tests

# Run integration tests
cargo test --test '*'

# Run with test data validation
cargo test -- --nocapture

# Run benchmarks
cargo bench
```

## Implementation Guidelines

1. **Test Structure**: Each test should reference the corresponding C++ test
2. **Validation**: Use the same tolerances and validation methods as C++
3. **Fixtures**: Reuse C++ test data through symlinks
4. **Parallelization**: Test both single and multi-threaded variants
5. **Edge Cases**: Always test empty inputs and boundary conditions

## Progress Summary

| Component         | Total Tests | Implemented | Passing | Coverage |
|-------------------|-------------|-------------|---------|----------|
| **KdTree**        | 8           | 0           | 0       | 0%       |
| **Preprocessing** | 6           | 0           | 0       | 0%       |
| **Point Cloud**   | 6           | 0           | 0       | 0%       |
| **VoxelMap**      | 3           | 0           | 0       | 0%       |
| **Registration**  | 6           | 0           | 0       | 0%       |
| **Helpers**       | 3           | 0           | 0       | 0%       |
| **Total**         | **32**      | **0**       | **0**   | **0%**   |

---

*This testing framework ensures the Rust implementation maintains exact compatibility with the C++ implementation while following Rust best practices.*
