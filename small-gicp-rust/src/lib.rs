//! Pure Rust API for the small_gicp point cloud registration library.
//!
//! This crate provides a high-level, idiomatic Rust interface for point cloud
//! registration algorithms including ICP, GICP, and VGICP. It uses the small-gicp-cxx
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
//! - **Generic programming**: Trait-based design for custom point cloud types
//!
//! # Quick Start
//!
//! ```rust,no_run
//! use nalgebra::Point3;
//! use small_gicp_rust::prelude::*;
//!
//! # fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
//! // TODO: This API is not yet implemented
//! // The following shows the intended interface:
//!
//! // Create point clouds
//! let target_points = vec![
//!     Point3::new(0.0, 0.0, 0.0),
//!     Point3::new(1.0, 0.0, 0.0),
//!     Point3::new(0.0, 1.0, 0.0),
//! ];
//! let source_points = vec![
//!     Point3::new(0.1, 0.1, 0.0),
//!     Point3::new(1.1, 0.1, 0.0),
//!     Point3::new(0.1, 1.1, 0.0),
//! ];
//!
//! let target = PointCloud::from_points(&target_points)?;
//! let source = PointCloud::from_points(&source_points)?;
//!
//! // Perform registration
//! let settings = RegistrationSettings::default();
//! let result = register(&target, &source, &settings)?;
//!
//! println!("Registration converged: {}", result.converged);
//! println!("Final error: {:.6}", result.error);
//! # Ok(())
//! # }
//! ```

pub mod config;
pub mod error;
pub mod kdtree;
pub(crate) mod kdtree_internal;
pub mod point_cloud;
pub mod preprocessing;
pub mod registration;
pub mod traits;
pub mod voxelmap;

// Re-export the most commonly used types
pub use config::{
    CorrespondenceRejectorConfig, CovarianceEstimationConfig, DofRestrictionConfig,
    DownsamplingBackend, DownsamplingConfig, FlatContainerConfig, GaussNewtonConfig,
    GaussianVoxelMapConfig, IncrementalVoxelMapConfig, KdTreeBuilderType, KdTreeConfig, KnnConfig,
    LevenbergMarquardtConfig, LocalFeatureEstimationConfig, LocalFeatureSetterType,
    LocalFeaturesBackend, NormalEstimationBackend, NormalEstimationConfig, OptimizerConfig,
    ParallelBackend, PreprocessingConfig, ProjectionConfig, ProjectionType, RandomSamplingConfig,
    RegistrationConfig, RobustKernelConfig, RobustKernelType, TerminationConfig, VoxelGridConfig,
};
pub use error::{Result, SmallGicpError};
pub use kdtree::{KdTree, KdTreeStrategy, UnsafeKdTree};
pub use point_cloud::PointCloud;
pub use preprocessing::Preprocessing;
pub use registration::{
    register, register_advanced, register_preprocessed, register_vgicp, DofRestriction,
    ExtendedRegistrationResult, GaussianVoxelMap, RegistrationResult, RegistrationSettings,
    RegistrationType, RobustKernel,
};
pub use traits::{Covariance4, MutablePointCloudTrait, Normal4, Point4, PointCloudTrait};
pub use voxelmap::{
    GaussianVoxel, IncrementalVoxelMap, SearchOffsetPattern, VoxelContainerType, VoxelInfo,
};

/// Convenience module for glob imports.
pub mod prelude {
    pub use crate::{
        config::{
            CorrespondenceRejectorConfig, CovarianceEstimationConfig, DofRestrictionConfig,
            DownsamplingBackend, DownsamplingConfig, FlatContainerConfig, GaussNewtonConfig,
            GaussianVoxelMapConfig, IncrementalVoxelMapConfig, KdTreeBuilderType, KdTreeConfig,
            KnnConfig, LevenbergMarquardtConfig, LocalFeatureEstimationConfig,
            LocalFeatureSetterType, LocalFeaturesBackend, NormalEstimationBackend,
            NormalEstimationConfig, OptimizerConfig, ParallelBackend, PreprocessingConfig,
            ProjectionConfig, ProjectionType, RandomSamplingConfig, RegistrationConfig,
            RobustKernelConfig, RobustKernelType, TerminationConfig, VoxelGridConfig,
        },
        error::{Result, SmallGicpError},
        kdtree::{KdTree, KdTreeStrategy, UnsafeKdTree},
        point_cloud::PointCloud,
        preprocessing::Preprocessing,
        registration::{
            register, register_advanced, register_preprocessed, register_vgicp, DofRestriction,
            ExtendedRegistrationResult, GaussianVoxelMap, RegistrationResult, RegistrationSettings,
            RegistrationType, RobustKernel,
        },
        traits::{Covariance4, MutablePointCloudTrait, Normal4, Point4, PointCloudTrait},
        voxelmap::{
            GaussianVoxel, IncrementalVoxelMap, SearchOffsetPattern, VoxelContainerType, VoxelInfo,
        },
    };
}
