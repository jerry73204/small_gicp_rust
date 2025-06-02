# small-gicp-sys

Raw FFI bindings to the small_gicp C API.

This crate provides low-level bindings to the small_gicp point cloud registration library through its C wrapper API. These bindings are automatically generated using bindgen.

## Dependencies

Before using this crate, you need to have the small_gicp_c library installed. You have two options:

### Option 1: Build from source (recommended)

Enable the `build-from-source` feature in your `Cargo.toml`:

```toml
[dependencies]
small-gicp-sys = { version = "0.1", features = ["build-from-source"] }
```

This will automatically build the C wrapper library during the cargo build process.

### Option 2: Use system library

Install the small_gicp_c library system-wide:

```bash
cd ../small_gicp_c
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make
sudo make install
```

Then use the crate without any features:

```toml
[dependencies]
small-gicp-sys = "0.1"
```

## Usage

This crate provides raw FFI bindings and is inherently unsafe. All functions must be called within an `unsafe` block:

```rust
use small_gicp_sys::*;
use std::ptr;

unsafe {
    // Create a point cloud
    let mut cloud: *mut small_gicp_point_cloud_t = ptr::null_mut();
    let result = small_gicp_point_cloud_create(&mut cloud);
    
    if result == small_gicp_error_t::SMALL_GICP_SUCCESS {
        // Use the point cloud...
        
        // Don't forget to clean up
        small_gicp_point_cloud_destroy(cloud);
    }
}
```

## Safety

This crate exposes raw C APIs and is inherently unsafe. Users must ensure:

- Proper initialization and cleanup of resources
- Valid pointers are passed to functions
- Array bounds are respected
- Thread safety requirements are met

Consider using a higher-level safe wrapper crate for production use.

## Features

- `build-from-source`: Build the small_gicp_c library from source instead of using a system installation

## License

This crate is licensed under the MIT license, matching the small_gicp library.