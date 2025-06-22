# small_gicp_rust

Rust bindings for the [small_gicp](https://github.com/koide3/small_gicp) point cloud registration library.

## Overview

This project provides a safe, idiomatic Rust interface to small_gicp, a high-performance C++ library for point cloud registration algorithms (ICP, GICP, VGICP).

- **small-gicp-sys**: Low-level FFI bindings to the C++ library
- **small-gicp**: High-level Rust API with safety and ergonomics

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
small-gicp = "0.1"
```

## Quick Example

```rust
use small_gicp::prelude::*;

fn main() -> Result<()> {
    // Create sample point clouds
    let mut source = PointCloud::new()?;
    let mut target = PointCloud::new()?;
    
    // Add points (in practice, load from files or sensors)
    for i in 0..100 {
        let angle = i as f64 * 0.1;
        target.add_point(angle.cos(), angle.sin(), 0.0);
        // Source is slightly transformed
        source.add_point(angle.cos() + 0.1, angle.sin() + 0.1, 0.0);
    }
    
    // Build KdTree for target
    let target_tree = KdTree::new(&target)?;
    
    // Align point clouds using ICP
    let result = align_icp(
        &source,
        &target,
        &target_tree,
        None,
        IcpSettings::default()
    )?;
    
    println!("Converged: {}", result.converged);
    println!("Error: {:.6}", result.error);
    println!("Transform: {:?}", result.t_target_source);
    
    Ok(())
}
```

## Registration Methods

- **ICP**: Fast point-to-point alignment
- **Plane ICP**: Uses surface normals for better convergence
- **GICP**: Probabilistic approach using local covariances
- **VGICP**: Efficient voxel-based variant for large point clouds

See the [documentation](https://docs.rs/small-gicp) for detailed examples of each method.

## License

MIT License - same as the original small_gicp library.