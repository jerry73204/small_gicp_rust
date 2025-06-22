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
- **Registration algorithms**: ICP, Point-to-Plane ICP, GICP, and VGICP
- **Thread safety**: All operations can be parallelized
- **Memory safety**: Automatic resource management with RAII
- **Voxel-based registration**: VGICP for large-scale point clouds

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
small-gicp-rust = "0.1"
```

## Quick Start

```rust
use small_gicp::prelude::*;

fn main() -> Result<()> {
    // Create point clouds (in practice, load from PLY files or sensors)
    let mut source = PointCloud::new()?;
    let mut target = PointCloud::new()?;
    
    // Add points to the clouds
    for i in 0..100 {
        let angle = i as f64 * 0.1;
        target.add_point(angle.cos(), angle.sin(), 0.0);
        // Source is slightly transformed (rotated and translated)
        source.add_point(angle.cos() + 0.1, angle.sin() + 0.1, 0.0);
    }
    
    // Build KdTree for efficient correspondence search
    let target_tree = KdTree::new(&target)?;
    
    // Configure ICP settings
    let settings = IcpSettings {
        max_iterations: 50,
        max_correspondence_distance: 1.0,
        ..Default::default()
    };
    
    // Perform ICP registration
    let result = align_icp(&source, &target, &target_tree, None, settings)?;
    
    println!("Converged: {}", result.converged);
    println!("Iterations: {}", result.iterations);
    println!("Final error: {:.6}", result.error);
    
    // Extract the transformation
    let transform = &result.t_target_source;
    println!("Translation: {:?}", transform.translation.vector);
    
    Ok(())
}
```

## Registration Methods

### ICP (Iterative Closest Point)

Basic point-to-point registration. Fast but requires good initial alignment.

```rust
use small_gicp::prelude::*;

let target_tree = KdTree::new(&target)?;
let result = align_icp(&source, &target, &target_tree, None, IcpSettings::default())?;
```

### Point-to-Plane ICP

Uses surface normals for better convergence on smooth surfaces.

```rust
use small_gicp::prelude::*;

// Preprocess to compute normals
let (processed_target, target_tree) = preprocess_points(&target, 0.1, 20, 4)?;
let (processed_source, _) = preprocess_points(&source, 0.1, 20, 4)?;

let result = align_plane_icp(
    &processed_source,
    &processed_target,
    &target_tree,
    None,
    PlaneIcpSettings::default()
)?;
```

### GICP (Generalized ICP)

Uses local covariance information for robust registration.

```rust
use small_gicp::prelude::*;

// Preprocess to compute covariances
let (processed_target, target_tree) = preprocess_points(&target, 0.1, 20, 4)?;
let (processed_source, source_tree) = preprocess_points(&source, 0.1, 20, 4)?;

let result = align_gicp(
    &processed_source,
    &processed_target,
    &source_tree,
    &target_tree,
    None,
    GicpSettings::default()
)?;
```

### VGICP (Voxelized GICP)

Efficient registration for large point clouds using voxel maps.

```rust
use small_gicp::prelude::*;

// Create voxel map from target cloud
let voxel_resolution = 0.5;
let target_voxelmap = create_gaussian_voxelmap(&target, voxel_resolution)?;

let result = align_vgicp(
    &source,
    &target_voxelmap,
    None,
    VgicpSettings {
        voxel_resolution,
        ..Default::default()
    }
)?;
```

## Examples

### Basic Registration Example

```rust
use small_gicp::prelude::*;
use rand::distributions::{Distribution, Uniform};

// Generate sample point clouds
let mut source = PointCloud::new()?;
let mut target = PointCloud::new()?;

// Create a simple 3D pattern
for i in 0..50 {
    for j in 0..50 {
        let x = i as f64 * 0.1;
        let y = j as f64 * 0.1;
        let z = (x * x + y * y).sin() * 0.5;
        target.add_point(x, y, z);
    }
}

// Apply a transformation to create the source cloud
let rotation = nalgebra::Vector3::z() * 0.3; // 0.3 radians around Z
let translation = nalgebra::Vector3::new(0.5, 0.5, 0.1);
let transform = nalgebra::Isometry3::new(translation, rotation);

// Transform target points to create source
for i in 0..target.len() {
    let p = target.point(i);
    let pt = nalgebra::Point3::new(p[0], p[1], p[2]);
    let transformed = transform * pt;
    source.add_point(transformed.x, transformed.y, transformed.z);
}

// Perform registration to recover the transformation
let target_tree = KdTree::new(&target)?;
let result = align_icp(&source, &target, &target_tree, None, IcpSettings::default())?;

println!("Registration result:");
println!("  Converged: {}", result.converged);
println!("  Iterations: {}", result.iterations);
println!("  Error: {:.6}", result.error);
```

### Preprocessing Pipeline

```rust
use small_gicp::prelude::*;

// Load a point cloud
let cloud = PointCloud::new()?;
// ... add points ...

// Complete preprocessing pipeline
let (processed_cloud, kdtree) = preprocess_points(
    &cloud,
    0.1,    // Downsampling resolution
    20,     // Number of neighbors for normal estimation
    4       // Number of threads
)?;

println!("Original points: {}", cloud.len());
println!("Processed points: {}", processed_cloud.len());
println!("Has normals: {}", processed_cloud.has_normals());
println!("Has covariances: {}", processed_cloud.has_covariances());
```

### Custom Registration Settings

```rust
use small_gicp::prelude::*;

// Create custom settings for fine control
let settings = IcpSettings {
    max_iterations: 100,
    max_correspondence_distance: 2.0,
    rotation_eps: 0.001,
    translation_eps: 0.001,
    num_threads: 8,
};

let target_tree = KdTree::new(&target)?;
let result = align_icp(&source, &target, &target_tree, None, settings)?;
```

### Working with Voxel Maps

```rust
use small_gicp::prelude::*;
use nalgebra::Point3;

// Create a voxel map
let mut voxelmap = GaussianVoxelMap::new(0.5); // 0.5m voxel size

// Insert point cloud
voxelmap.insert(&cloud)?;

// Query operations
let query_point = Point3::new(1.0, 2.0, 3.0);

// Find nearest voxel
if let Some((index, distance)) = voxelmap.nearest_neighbor_search(&query_point) {
    println!("Nearest voxel index: {}, distance: {:.3}", index, distance.sqrt());
}

// Get voxel statistics
let stats = voxelmap.statistics();
println!("Voxels: {}, Total points: {}", stats.num_voxels, stats.total_points);
```

## Best Practices

1. **Preprocessing**: Always downsample dense point clouds for better performance
2. **Initial Alignment**: Provide a rough initial transformation for large misalignments
3. **Parameter Tuning**: Adjust `max_correspondence_distance` based on your data
4. **Method Selection**:
   - Use ICP for fast, simple registration with good initial alignment
   - Use Plane ICP when you have surface normals available
   - Use GICP for highest accuracy with structured scenes
   - Use VGICP for large-scale point clouds (>100k points)

## API Documentation

See the [full API documentation](https://docs.rs/small-gicp-rust) for detailed information about all types and functions.

### Key Types

- `PointCloud`: 3D point cloud with optional normals and covariances
- `KdTree`: Spatial index for efficient nearest neighbor search
- `GaussianVoxelMap`: Voxel map for VGICP registration
- `RegistrationResult`: Results including transformation and convergence info

### Registration Functions

- `align_icp()`: Basic ICP registration
- `align_plane_icp()`: Point-to-plane ICP with normals
- `align_gicp()`: Generalized ICP with covariances
- `align_vgicp()`: Voxelized GICP for large clouds

### Preprocessing Functions

- `preprocess_points()`: Complete preprocessing pipeline
- `create_gaussian_voxelmap()`: Create voxel map for VGICP

## Performance

The library supports multi-threading for all major operations. Set the `num_threads` parameter in registration settings to control parallelization.

## License

This crate is licensed under the MIT License, same as the original small_gicp library.

## Contributing

Contributions are welcome! Please see the main repository for contribution guidelines.