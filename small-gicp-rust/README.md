# small-gicp-rust

[![Crates.io](https://img.shields.io/crates/v/small-gicp-rust.svg)](https://crates.io/crates/small-gicp-rust)
[![Documentation](https://docs.rs/small-gicp-rust/badge.svg)](https://docs.rs/small-gicp-rust)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

Safe Rust bindings for the [small_gicp](https://github.com/koide3/small_gicp) point cloud registration library.

This crate provides a high-level, memory-safe interface to the small_gicp C++ library for 3D point cloud registration algorithms including ICP, GICP, and VGICP.

## Features

- **Point cloud operations**: Load, save, and manipulate 3D point clouds
- **KdTree spatial indexing**: Efficient nearest neighbor search
- **Point cloud preprocessing**: Downsampling, normal estimation, and covariance computation
- **Registration algorithms**: ICP, Plane ICP, GICP, and VGICP
- **Thread safety**: All operations can be parallelized
- **Memory safety**: Automatic resource management with RAII
- **Integration with nalgebra**: Uses nalgebra for linear algebra types

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
small-gicp-rust = "0.1"
nalgebra = "0.32"
```

## Quick Start

```rust
use small_gicp_rust::prelude::*;
use nalgebra::Point3;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create point clouds
    let target_points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
    ];
    let source_points = vec![
        Point3::new(0.1, 0.1, 0.0),
        Point3::new(1.1, 0.1, 0.0),
        Point3::new(0.1, 1.1, 0.0),
        Point3::new(1.1, 1.1, 0.0),
    ];

    let target = PointCloud::from_points(&target_points)?;
    let source = PointCloud::from_points(&source_points)?;

    // Perform registration
    let settings = RegistrationSettings::default();
    let result = register(&target, &source, &settings)?;

    println!("Registration converged: {}", result.converged);
    println!("Final error: {:.6}", result.error);
    println!("Translation: {:?}", result.translation());
    
    Ok(())
}
```

## Examples

### Basic Registration

```rust
use small_gicp_rust::prelude::*;
use nalgebra::Point3;

// Create point clouds from vectors of points
let target = PointCloud::from_points(&target_points)?;
let source = PointCloud::from_points(&source_points)?;

// Configure registration settings
let settings = RegistrationSettings {
    registration_type: RegistrationType::Gicp,
    num_threads: 4,
    initial_guess: None,
};

// Perform registration
let result = register(&target, &source, &settings)?;

// Access results
println!("Converged: {}", result.converged);
println!("Error: {:.6}", result.error);
println!("Iterations: {}", result.iterations);

// Get transformation as nalgebra types
let transformation = result.transformation;
let translation = result.translation();
let rotation = result.rotation();
```

### Point Cloud Preprocessing

```rust
use small_gicp_rust::prelude::*;

// Load point cloud from file
let cloud = PointCloud::from_ply("input.ply")?;

// Configure preprocessing
let settings = PreprocessingSettings {
    downsampling: Some(DownsamplingMethod::VoxelGrid { leaf_size: 0.05 }),
    num_neighbors_normals: 20,
    estimate_covariances: true,
    num_threads: 4,
};

// Preprocess the cloud
let result = preprocess_point_cloud(&cloud, &settings, true)?;

// Use the preprocessed cloud and KdTree
let preprocessed_cloud = result.cloud;
let kdtree = result.kdtree.unwrap();

// Save the preprocessed cloud
preprocessed_cloud.save_ply("output.ply")?;
```

### Manual Preprocessing Steps

```rust
use small_gicp_rust::prelude::*;

let cloud = PointCloud::from_ply("input.ply")?;

// Step 1: Downsample
let downsampled = voxelgrid_sampling(&cloud, 0.05, 4)?;
println!("Downsampled: {} → {} points", cloud.len(), downsampled.len());

// Step 2: Build KdTree
let kdtree = KdTree::new(&downsampled, 4)?;

// Step 3: Estimate normals and covariances
let mut processed = downsampled;
estimate_normals_and_covariances(&mut processed, &kdtree, 20, 4)?;
```

### Different Registration Algorithms

```rust
use small_gicp_rust::prelude::*;

let algorithms = [
    RegistrationType::Icp,        // Point-to-point ICP
    RegistrationType::PlaneIcp,   // Point-to-plane ICP  
    RegistrationType::Gicp,       // Generalized ICP
    RegistrationType::Vgicp,      // Voxelized Generalized ICP
];

for algorithm in algorithms {
    let settings = RegistrationSettings {
        registration_type: algorithm,
        num_threads: 4,
        initial_guess: None,
    };
    
    let result = register(&target, &source, &settings)?;
    println!("{:?}: error = {:.6}", algorithm, result.error);
}
```

### VGICP Registration

```rust
use small_gicp_rust::prelude::*;

// Create Gaussian voxel map for target
let target_voxelmap = GaussianVoxelMap::new(&target, 0.1, 4)?;

let settings = RegistrationSettings {
    registration_type: RegistrationType::Vgicp,
    num_threads: 4,
    initial_guess: None,
};

// Register using voxel map
let result = register_vgicp(&target_voxelmap, &source, &settings)?;
```

### KdTree Operations

```rust
use small_gicp_rust::prelude::*;
use nalgebra::Point3;

let kdtree = KdTree::new(&cloud, 4)?;
let query_point = Point3::new(0.5, 0.5, 0.5);

// Nearest neighbor search
let (index, sq_distance) = kdtree.nearest_neighbor(query_point)?;

// K nearest neighbors
let neighbors = kdtree.knn_search(query_point, 10)?;

// Radius search
let radius_neighbors = kdtree.radius_search(query_point, 0.1, 20)?;
```

## Registration Algorithms

This crate supports several point cloud registration algorithms:

- **ICP (Iterative Closest Point)**: Basic point-to-point registration
- **Plane ICP**: Point-to-plane registration, better for planar surfaces  
- **GICP (Generalized ICP)**: Uses local surface covariances for better accuracy
- **VGICP (Voxelized GICP)**: Memory-efficient variant of GICP using voxel maps

## API Documentation

### Core Types

- `PointCloud`: 3D point cloud with optional normals
- `KdTree`: Spatial index for efficient nearest neighbor search
- `RegistrationSettings`: Configuration for registration algorithms
- `RegistrationResult`: Results of registration including transformation
- `PreprocessingSettings`: Configuration for preprocessing operations

### Key Functions

- `register()`: Complete registration with automatic preprocessing
- `register_preprocessed()`: Registration with manually preprocessed clouds
- `register_vgicp()`: VGICP registration using voxel maps
- `preprocess_point_cloud()`: Complete preprocessing pipeline
- `voxelgrid_sampling()`: Voxel grid downsampling
- `random_sampling()`: Random downsampling
- `estimate_normals()`: Surface normal estimation

## Performance

The library supports multi-threading for all major operations:

- Point cloud preprocessing (downsampling, normal estimation)
- KdTree construction  
- Registration algorithms

Set the `num_threads` parameter to control parallelization (1 for single-threaded).

## Examples

Run the included examples:

```bash
# Basic registration demonstration
cargo run --example basic_registration

# Preprocessing operations
cargo run --example preprocessing_demo
```

## Testing

Run the test suite:

```bash
# Unit tests
cargo test

# Integration tests  
cargo test --test integration_tests

# All tests
cargo test --all
```

## Dependencies

This crate depends on:

- `small-gicp-sys`: Low-level FFI bindings to the C++ library
- `nalgebra`: Linear algebra types (Point3, Vector3, Isometry3, etc.)
- `thiserror`: Error handling

## License

This crate is licensed under the MIT License. See [LICENSE](LICENSE) for details.

The underlying small_gicp C++ library has its own license terms.

## Contributing

Contributions are welcome! Please see the main repository for contribution guidelines.

## Changelog

### 0.1.0

- Initial release
- Support for ICP, GICP, and VGICP registration
- Point cloud preprocessing utilities
- KdTree spatial indexing
- Safe Rust API with automatic memory management
- Integration with nalgebra for linear algebra types