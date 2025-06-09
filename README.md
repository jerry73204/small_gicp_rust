# small_gicp_rust

Rust bindings and C wrapper for the [small_gicp](https://github.com/koide3/small_gicp) point cloud registration library.

## Overview

This project provides Rust access to small_gicp, a fast C++ library for point cloud registration algorithms (ICP, GICP, VGICP). It includes:

- **C wrapper** (`small_gicp_c/`) - Complete C API for the C++ library
- **Rust sys crate** (`small-gicp-sys/`) - Low-level FFI bindings
- **Rust high-level API** (`small-gicp-rust/`) - Safe, ergonomic Rust interface

## Architecture

```
small_gicp_rust/
├── small_gicp/              # Original C++ library (submodule)
├── small_gicp_c/            # C wrapper API
│   ├── include/             # C headers
│   ├── src/                 # C++ implementation
│   └── example/             # C usage examples
├── small-gicp-sys/          # Rust FFI bindings (sys crate)
└── small-gicp-rust/         # High-level Rust API
    ├── src/
    ├── examples/
    └── tests/
```

## Building

### Prerequisites
- CMake 3.16+
- C++17 compiler
- Eigen3
- OpenMP (optional)
- Intel TBB (optional)

### C Wrapper
```bash
cd small_gicp_c
./build.sh
```

### Rust
```bash
cargo build --all-targets
```

## C API Coverage

The C wrapper provides ~85% coverage of the C++ API:

| Module | Coverage | Features |
|--------|----------|----------|
| Point Cloud | 90% | Points, normals, covariances, validation |
| Registration | 80% | ICP, GICP, VGICP, advanced settings |
| KdTree | 85% | Parallel builders (OpenMP, TBB), configuration |
| Utilities | 65% | Downsampling, normal estimation |
| I/O | 40% | PLY file support |

See [PROGRESS.md](small_gicp_c/PROGRESS.md) for detailed API coverage.

## Usage

### C API
```c
#include <small_gicp_c.h>

// Create point clouds
small_gicp_point_cloud_t *target, *source;
small_gicp_point_cloud_create(&target);
small_gicp_point_cloud_create(&source);

// Load data and perform registration
small_gicp_registration_result_t result;
small_gicp_align(target, source, SMALL_GICP_GICP, NULL, 4, &result);
```

### Rust API
```rust
use small_gicp_rust::{PointCloud, align, RegistrationType};

let target = PointCloud::from_points(&target_points)?;
let source = PointCloud::from_points(&source_points)?;

let result = align(&target, &source, RegistrationType::GICP)?;
println!("Transformation: {:?}", result.transformation);
```

## Features

- **Point Cloud Operations**: Create, manipulate, and validate point clouds
- **Registration Algorithms**: ICP, Point-to-Plane ICP, GICP, VGICP
- **Parallel Processing**: OpenMP and TBB support for performance
- **Preprocessing**: Downsampling, normal/covariance estimation
- **Advanced Configuration**: Custom termination criteria, robust kernels, DOF restrictions

## License

This project follows the same license as the original small_gicp library (MIT).
