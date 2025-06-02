# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

small_gicp is a header-only C++ library for efficient and parallelized point cloud registration algorithms (ICP, Point-to-Plane ICP, GICP, VGICP). It is a refined version of fast_gicp, offering 2x performance improvement with minimal dependencies (Eigen + bundled nanoflann and Sophus).

**Important Note**: Despite the repository name "small_gicp_rust", this currently contains only the C++ implementation. The Rust port appears to be planned for the future.

## Build Commands

### C++ Build
```bash
# Create build directory
mkdir build && cd build

# Configure (with all features)
cmake .. -DBUILD_HELPER=ON -DBUILD_TESTS=ON -DBUILD_EXAMPLES=ON -DBUILD_BENCHMARKS=ON

# Build
make -j$(nproc)

# Run tests
ctest

# Run specific test
./src/test/kdtree_test
```

### Python Build
```bash
# Install from source
pip install .

# Development install
pip install -e .

# Run Python tests
pytest src/test/python_test.py
```

## Linting and Code Quality

### C++ Code
```bash
# No specific linting setup found - follow general C++ best practices
# Code uses C++17 standard
```

### Python Code
```bash
# Run Python tests
pytest src/test/python_test.py
```

## Architecture Overview

### Core Components

1. **Point Cloud Representations** (`include/small_gicp/points/`)
   - Flexible point cloud interface supporting Eigen vectors, custom structs, and PCL point types
   - Traits system for compile-time type checking

2. **Nearest Neighbor Search** (`include/small_gicp/ann/`)
   - KdTree implementations with parallel variants (OpenMP, TBB)
   - Voxel-based spatial hashing for large-scale data

3. **Registration Algorithms** (`include/small_gicp/registration/`)
   - Core registration framework with customizable components
   - Supports ICP, Point-to-Plane ICP, GICP, VGICP
   - Optimizer with Levenberg-Marquardt solver

4. **Preprocessing Utilities** (`include/small_gicp/util/`)
   - Parallel downsampling algorithms
   - Normal/covariance estimation
   - All utilities have OpenMP and TBB variants

### Design Patterns

- **Header-only library**: Most functionality in headers for easy integration
- **Template-based design**: Allows working with various point types without runtime overhead
- **Parallel variants**: Functions typically have sequential, OpenMP, and TBB implementations
- **PCL compatibility**: Drop-in replacement for PCL registration when PCL is available

### Python Bindings

The Python bindings (`src/python/`) expose core functionality:
- Point cloud loading and preprocessing
- KdTree construction and queries
- Registration algorithms
- Voxel grid mapping

## Testing Approach

- **C++ Tests**: Google Test framework in `src/test/*_test.cpp`
- **Python Tests**: pytest in `src/test/python_test.py`
- **Benchmarks**: Performance benchmarks in `src/benchmark/`
- **Examples**: Working examples in `src/example/` serve as integration tests

## Performance Considerations

- The library is optimized for parallel execution - prefer OpenMP/TBB variants for large point clouds
- Voxel-based downsampling is faster than random sampling for large datasets
- Registration convergence criteria can be tuned via `TerminationCriteria` struct