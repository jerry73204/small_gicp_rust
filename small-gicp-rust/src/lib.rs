//! Pure Rust API for the small_gicp point cloud registration library.
//!
//! This crate provides a high-level, idiomatic Rust interface for point cloud
//! registration algorithms including ICP, GICP, and VGICP. It uses the small-gicp-sys
//! crate internally for the actual C++ implementation.
//!
//! # Features
//!
//! - **Point cloud operations**: Load, save, and manipulate 3D point clouds
//! - **KdTree spatial indexing**: Efficient nearest neighbor search
//! - **Point cloud preprocessing**: Downsampling, normal estimation, and covariance computation
//! - **Registration algorithms**: ICP, Plane ICP, GICP, and VGICP
//! - **Thread safety**: All operations can be parallelized
//! - **Memory safety**: Automatic resource management with RAII
//! - **Voxel-based registration**: VGICP for large-scale point clouds
//!
//! # Quick Start
//!
//! ```rust,no_run
//! use small_gicp::prelude::*;
//!
//! # fn main() -> Result<()> {
//! // Create point clouds (in practice, load from PLY files or sensors)
//! let mut source = PointCloud::new()?;
//! let mut target = PointCloud::new()?;
//!
//! // Add points to the clouds
//! for i in 0..100 {
//!     let angle = i as f64 * 0.1;
//!     target.add_point(angle.cos(), angle.sin(), 0.0);
//!     // Source is slightly transformed (rotated and translated)
//!     source.add_point(angle.cos() + 0.1, angle.sin() + 0.1, 0.0);
//! }
//!
//! // Build KdTree for efficient correspondence search
//! let target_tree = KdTree::new(&target)?;
//!
//! // Configure ICP settings
//! let settings = IcpSettings {
//!     max_iterations: 50,
//!     max_correspondence_distance: 1.0,
//!     ..Default::default()
//! };
//!
//! // Perform ICP registration
//! let result = align_icp(&source, &target, &target_tree, None, settings)?;
//!
//! println!("Converged: {}", result.converged);
//! println!("Iterations: {}", result.iterations);
//! println!("Final error: {:.6}", result.error);
//!
//! // Extract the transformation
//! let transform = &result.t_target_source;
//! println!("Translation: {:?}", transform.translation.vector);
//! # Ok(())
//! # }
//! ```
//!
//! # Registration Methods
//!
//! ## ICP (Iterative Closest Point)
//!
//! Basic point-to-point registration. Fast but requires good initial alignment.
//!
//! ```rust,no_run
//! # use small_gicp::prelude::*;
//! # fn main() -> Result<()> {
//! # let source = PointCloud::new()?;
//! # let target = PointCloud::new()?;
//! let target_tree = KdTree::new(&target)?;
//! let result = align_icp(&source, &target, &target_tree, None, IcpSettings::default())?;
//! # Ok(())
//! # }
//! ```
//!
//! ## Point-to-Plane ICP
//!
//! Uses surface normals for better convergence on smooth surfaces.
//!
//! ```rust,no_run
//! # use small_gicp::prelude::*;
//! # fn main() -> Result<()> {
//! # let source = PointCloud::new()?;
//! # let target = PointCloud::new()?;
//! // Preprocess to compute normals
//! let (processed_target, target_tree) = preprocess_points(&target, 0.1, 20, 4)?;
//! let (processed_source, _) = preprocess_points(&source, 0.1, 20, 4)?;
//!
//! let result = align_plane_icp(
//!     &processed_source,
//!     &processed_target,
//!     &target_tree,
//!     None,
//!     PlaneIcpSettings::default(),
//! )?;
//! # Ok(())
//! # }
//! ```
//!
//! ## GICP (Generalized ICP)
//!
//! Uses local covariance information for robust registration.
//!
//! ```rust,no_run
//! # use small_gicp::prelude::*;
//! # fn main() -> Result<()> {
//! # let source = PointCloud::new()?;
//! # let target = PointCloud::new()?;
//! // Preprocess to compute covariances
//! let (processed_target, target_tree) = preprocess_points(&target, 0.1, 20, 4)?;
//! let (processed_source, source_tree) = preprocess_points(&source, 0.1, 20, 4)?;
//!
//! let result = align_gicp(
//!     &processed_source,
//!     &processed_target,
//!     &source_tree,
//!     &target_tree,
//!     None,
//!     GicpSettings::default(),
//! )?;
//! # Ok(())
//! # }
//! ```
//!
//! ## VGICP (Voxelized GICP)
//!
//! Efficient registration for large point clouds using voxel maps.
//!
//! ```rust,no_run
//! # use small_gicp::prelude::*;
//! # fn main() -> Result<()> {
//! # let source = PointCloud::new()?;
//! # let target = PointCloud::new()?;
//! // Create voxel map from target cloud
//! let voxel_resolution = 0.5;
//! let target_voxelmap = create_gaussian_voxelmap(&target, voxel_resolution)?;
//!
//! let result = align_vgicp(
//!     &source,
//!     &target_voxelmap,
//!     None,
//!     VgicpSettings {
//!         voxel_resolution,
//!         ..Default::default()
//!     },
//! )?;
//! # Ok(())
//! # }
//! ```
//!
//! # Best Practices
//!
//! 1. **Preprocessing**: Always downsample dense point clouds for better performance
//! 2. **Initial Alignment**: Provide a rough initial transformation for large misalignments
//! 3. **Parameter Tuning**: Adjust `max_correspondence_distance` based on your data
//! 4. **Method Selection**:
//!    - Use ICP for fast, simple registration with good initial alignment
//!    - Use Plane ICP when you have surface normals available
//!    - Use GICP for highest accuracy with structured scenes
//!    - Use VGICP for large-scale point clouds (>100k points)

pub mod config;
pub mod error;
pub mod kdtree;
pub mod point_cloud;
pub mod preprocessing;
pub mod registration;
pub mod traits;
pub mod voxelmap;

#[cfg(test)]
// Re-export the most commonly used types
pub use config::{
    CorrespondenceRejectorConfig,
    CovarianceEstimationConfig,
    DofRestrictionConfig,
    DownsamplingBackend,
    DownsamplingConfig,
    FlatContainerConfig,
    GaussNewtonConfig,
    GaussianVoxelMapConfig,
    KdTreeBuilderType,
    KdTreeConfig,
    KnnConfig,
    LevenbergMarquardtConfig,
    LocalFeatureEstimationConfig,
    LocalFeatureSetterType,
    LocalFeaturesBackend,
    NormalEstimationBackend,
    NormalEstimationConfig,
    OptimizerConfig,
    ParallelBackend,
    PreprocessingConfig as ConfigPreprocessingConfig,
    ProjectionConfig,
    ProjectionType,
    RandomSamplingConfig,
    RegistrationConfig,
    TerminationConfig,
    VoxelGridConfig,
    // TODO: Uncomment when robust kernels are exposed in C++ registration_helper API
    // RobustKernelConfig,
};
pub use error::{Result, SmallGicpError};
pub use kdtree::{BorrowedKdTree, KdTree};
pub use point_cloud::PointCloud;
pub use preprocessing::Preprocessing;
// Re-export registration functions and types
pub use registration::{
    // High-level API
    align,
    // Low-level API
    align_gicp,
    align_icp,
    align_plane_icp,
    align_vgicp,

    // Common types
    create_gaussian_voxelmap,
    preprocess_points,
    // Settings types
    GicpSettings,
    IcpSettings,
    PlaneIcpSettings,
    PreprocessingConfig,
    RegistrationMethod,

    RegistrationResult,

    VgicpSettings,
    // TODO: Uncomment when robust kernels are exposed in C++ registration_helper API
    // RobustKernel, RobustKernelType,
};
pub use traits::{
    Covariance4, MutablePointCloudTrait, Normal4, Point4, PointCloudTrait, SpatialSearchTree,
};
pub use voxelmap::{
    GaussianVoxel, GaussianVoxelMap, SearchOffsetPattern, VoxelContainerType, VoxelInfo,
    VoxelMapStatistics,
};

/// Convenience module for glob imports.
pub mod prelude {
    pub use crate::{
        config::{
            CorrespondenceRejectorConfig,
            CovarianceEstimationConfig,
            DofRestrictionConfig,
            DownsamplingBackend,
            DownsamplingConfig,
            FlatContainerConfig,
            GaussNewtonConfig,
            GaussianVoxelMapConfig,
            KdTreeBuilderType,
            KdTreeConfig,
            KnnConfig,
            LevenbergMarquardtConfig,
            LocalFeatureEstimationConfig,
            LocalFeatureSetterType,
            LocalFeaturesBackend,
            NormalEstimationBackend,
            NormalEstimationConfig,
            OptimizerConfig,
            ParallelBackend,
            PreprocessingConfig as ConfigPreprocessingConfig,
            ProjectionConfig,
            ProjectionType,
            RandomSamplingConfig,
            RegistrationConfig,
            TerminationConfig,
            VoxelGridConfig,
            // TODO: Uncomment when robust kernels are exposed in C++ registration_helper API
            // RobustKernelConfig,
        },
        error::{Result, SmallGicpError},
        kdtree::{BorrowedKdTree, KdTree},
        point_cloud::PointCloud,
        preprocessing::Preprocessing,
        // Registration functions and types
        registration::{
            // High-level API
            align,
            // Low-level API
            align_gicp,
            align_icp,
            align_plane_icp,
            align_vgicp,

            // Common types
            create_gaussian_voxelmap,
            preprocess_points,
            // Settings types
            GicpSettings,
            IcpSettings,
            PlaneIcpSettings,
            PreprocessingConfig,
            RegistrationMethod,

            RegistrationResult,

            VgicpSettings,
            // TODO: Uncomment when robust kernels are exposed in C++ registration_helper API
            // RobustKernel, RobustKernelType,
        },
        traits::{
            Covariance4, MutablePointCloudTrait, Normal4, Point4, PointCloudTrait,
            SpatialSearchTree,
        },
        voxelmap::{
            GaussianVoxel, GaussianVoxelMap, SearchOffsetPattern, VoxelContainerType, VoxelInfo,
            VoxelMapStatistics,
        },
    };
}
