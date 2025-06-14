//! # small-gicp-cxx
//!
//! Direct C++ bindings for the small_gicp library using cxx.
//!
//! This crate provides safe Rust bindings to the small_gicp C++ library,
//! offering efficient point cloud registration algorithms including ICP, GICP, and VGICP.
//!
//! ## Example
//!
//! ```rust,no_run
//! use small_gicp_cxx::{KdTree, PointCloud, Registration, RegistrationSettingsBuilder};
//!
//! // Create point clouds
//! let mut source = PointCloud::new();
//! let mut target = PointCloud::new();
//!
//! // ... add points to clouds ...
//!
//! // Build KdTree for target
//! let target_tree = KdTree::build(&target, 4);
//!
//! // Configure registration
//! let settings = RegistrationSettingsBuilder::new()
//!     .max_iterations(50)
//!     .num_threads(4)
//!     .build();
//!
//! // Perform ICP registration
//! let result = Registration::icp(&source, &target, &target_tree, None, Some(settings));
//!
//! println!("Converged: {}", result.converged);
//! println!("Iterations: {}", result.iterations);
//! println!("Error: {}", result.error);
//! ```

#![warn(missing_docs)]

mod ffi;
mod incremental_voxel_map;
mod kdtree;
mod point_cloud;
mod registration;
mod unsafe_kdtree;
mod voxel_map;

pub use incremental_voxel_map::{IncrementalVoxelMap, IncrementalVoxelMapBuilder};
pub use kdtree::{KdTree, KdTreeBuilder};
pub use point_cloud::PointCloud;
pub use registration::{Registration, RegistrationSettingsBuilder, TransformExt};
pub use unsafe_kdtree::{UnsafeKdTree, UnsafeKdTreeBuilder};
pub use voxel_map::{VoxelMap, VoxelMapBuilder};

// Re-export FFI types that users need
pub use ffi::{Point3d, RegistrationResult, RegistrationSettings, Transform};
