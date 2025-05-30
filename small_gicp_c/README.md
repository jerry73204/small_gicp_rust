# small_gicp C Wrapper

This is a C wrapper for the small_gicp C++ library, providing a pure C interface for point cloud registration algorithms.

## Features

- Point cloud loading and saving (PLY format)
- Point cloud preprocessing (downsampling, normal estimation)
- Multiple registration algorithms:
  - ICP (Iterative Closest Point)
  - Point-to-Plane ICP
  - GICP (Generalized ICP)
  - VGICP (Voxelized GICP)
- KdTree for efficient nearest neighbor search
- Thread-safe operations with OpenMP support

## Building

### Requirements

- CMake 3.16 or higher
- C11 compatible compiler
- C++17 compatible compiler
- Eigen3
- OpenMP (optional, for parallelization)

### Build Instructions

```bash
cd small_gicp_c
mkdir build
cd build
cmake .. -DBUILD_EXAMPLES=ON
make -j$(nproc)
```

### Installation

```bash
sudo make install
```

This will install:
- Headers to `/usr/local/include/`
- Libraries to `/usr/local/lib/`
- CMake config files to `/usr/local/lib/cmake/small_gicp_c/`
- pkg-config file to `/usr/local/lib/pkgconfig/`

## Usage

### Basic Example

```c
#include <small_gicp_c.h>

int main() {
    // Load point clouds
    small_gicp_point_cloud_t* target = NULL;
    small_gicp_point_cloud_t* source = NULL;
    
    small_gicp_load_ply("target.ply", &target);
    small_gicp_load_ply("source.ply", &source);
    
    // Perform registration
    small_gicp_registration_result_t result;
    small_gicp_align(target, source, SMALL_GICP_GICP, NULL, 4, &result);
    
    // Print results
    printf("Converged: %s\n", result.converged ? "Yes" : "No");
    printf("Error: %f\n", result.error);
    
    // Clean up
    small_gicp_point_cloud_destroy(target);
    small_gicp_point_cloud_destroy(source);
    
    return 0;
}
```

### Linking with CMake

```cmake
find_package(small_gicp_c REQUIRED)
target_link_libraries(your_target small_gicp_c::small_gicp_c)
```

### Linking with pkg-config

```bash
gcc your_program.c `pkg-config --cflags --libs small_gicp_c` -o your_program
```

## API Reference

### Error Handling

All functions return `small_gicp_error_t`:
- `SMALL_GICP_SUCCESS` - Operation successful
- `SMALL_GICP_ERROR_INVALID_ARGUMENT` - Invalid argument provided
- `SMALL_GICP_ERROR_OUT_OF_MEMORY` - Memory allocation failed
- `SMALL_GICP_ERROR_FILE_NOT_FOUND` - File not found
- `SMALL_GICP_ERROR_IO_ERROR` - I/O error occurred
- `SMALL_GICP_ERROR_EXCEPTION` - C++ exception caught

Use `small_gicp_error_string()` to get human-readable error descriptions.

### Main Functions

See `include/small_gicp_c.h` for the complete API documentation.

## Examples

Three example programs are provided:

1. **basic_registration** - Demonstrates all registration algorithms
2. **preprocessing** - Shows downsampling and normal estimation
3. **kdtree** - KdTree construction and nearest neighbor queries

Run examples from the build directory:

```bash
./example/basic_registration ../small_gicp/data/target.ply ../small_gicp/data/source.ply
./example/preprocessing ../small_gicp/data/target.ply
./example/kdtree
```

## License

This wrapper is provided under the same license as the small_gicp library.