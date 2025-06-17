//! # small-gicp-cxx
//!
//! Direct C++ bindings for the small_gicp library using cxx.
//!
//! This crate provides safe Rust bindings to the small_gicp C++ library,
//! offering efficient point cloud registration algorithms including ICP, GICP, and VGICP.
//!
//! ## Features
//!
//! - **Point Cloud Operations**: Create, manipulate, and transform point clouds
//! - **Bulk Operations**: High-performance bulk data setting for large point clouds
//! - **Preprocessing**: Voxel grid downsampling, normal estimation, covariance estimation
//! - **Registration**: ICP, Point-to-Plane ICP, GICP, VGICP algorithms
//! - **Spatial Data Structures**: KdTree, UnsafeKdTree, GaussianVoxelMap, IncrementalVoxelMap
//! - **Transformations**: Rigid body transformations with utility functions
//! - **I/O Support**: Load and save point clouds in PLY format
//!
//! ## Example
//!
//! ```rust,no_run
//! use small_gicp_cxx::{
//!     KdTree, PointCloud, Preprocessing, Registration, RegistrationSettingsBuilder, Transform,
//! };
//!
//! // Create point clouds using bulk operations for better performance
//! let points = vec![
//!     1.0, 2.0, 3.0, 1.0, // Point 1 (x, y, z, w)
//!     4.0, 5.0, 6.0, 1.0, // Point 2 (x, y, z, w)
//! ];
//! let mut source = PointCloud::from_points_bulk(&points);
//! let mut target = PointCloud::new();
//! target.add_point(0.0, 0.0, 0.0);
//! target.add_point(1.0, 1.0, 1.0);
//!
//! // Apply transformation
//! let transform = Transform::translation(1.0, 0.0, 0.0);
//! source.transform(&transform);
//!
//! // Preprocess point clouds
//! let processed_source = Preprocessing::preprocess_for_registration(&source, 0.1, 10, 1);
//! let processed_target = Preprocessing::preprocess_for_registration(&target, 0.1, 10, 1);
//!
//! // Build KdTree for target
//! let target_tree = KdTree::build(&processed_target, 4);
//!
//! // Configure registration
//! let settings = RegistrationSettingsBuilder::new()
//!     .max_iterations(50)
//!     .num_threads(4)
//!     .build();
//!
//! // Perform ICP registration
//! let result = Registration::icp(
//!     &processed_source,
//!     &processed_target,
//!     &target_tree,
//!     None,
//!     Some(settings),
//! );
//!
//! println!("Converged: {}", result.converged);
//! println!("Iterations: {}", result.iterations);
//! println!("Error: {}", result.error);
//! ```

#![warn(missing_docs)]

mod ffi;
mod incremental_voxel_map;
mod io;
mod kdtree;
mod point_cloud;
mod preprocessing;
mod registration;
mod transform;
mod unsafe_kdtree;
mod voxel_map;

pub use incremental_voxel_map::{IncrementalVoxelMap, IncrementalVoxelMapBuilder};
pub use io::{Io, PointCloudIoExt};
pub use kdtree::{KdTree, KdTreeBuilder};
pub use point_cloud::PointCloud;
pub use preprocessing::{Preprocessing, PreprocessingBuilder};
pub use registration::{Registration, RegistrationSettingsBuilder, TransformExt};
pub use unsafe_kdtree::{UnsafeKdTree, UnsafeKdTreeBuilder};
pub use voxel_map::{VoxelMap, VoxelMapBuilder};

// Re-export FFI types that users need
pub use ffi::ffi::{
    GaussianVoxelData, KdTreeSettings, Point3d, RegistrationResult, RegistrationSettings,
    RegistrationType, Transform, VoxelInfoData,
};
