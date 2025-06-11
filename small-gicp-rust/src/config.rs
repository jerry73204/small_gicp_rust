//! Configuration structures for small_gicp algorithms.
//!
//! This module provides typed configuration for various point cloud processing algorithms.
//! All configuration structs implement `Default` with values matching the C++ defaults.

use std::f64::consts::PI;

/// Get the default number of threads based on available parallelism.
/// Falls back to 4 if unable to determine available parallelism.
fn default_num_threads() -> usize {
    std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(4)
}

/// Builder type for KdTree construction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KdTreeBuilderType {
    /// Default single-threaded builder.
    Default,
    /// OpenMP parallel builder.
    OpenMp,
    /// TBB (Threading Building Blocks) parallel builder.
    Tbb,
}

impl Default for KdTreeBuilderType {
    fn default() -> Self {
        Self::Default
    }
}

/// Projection type for KdTree splitting strategy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProjectionType {
    /// Split along coordinate axes (X/Y/Z) - fastest approach.
    AxisAligned,
    /// Split along normal direction - more accurate for surfaces.
    Normal,
}

impl Default for ProjectionType {
    fn default() -> Self {
        Self::AxisAligned
    }
}

/// Configuration for KdTree construction.
#[derive(Debug, Clone)]
pub struct KdTreeConfig {
    /// Builder type for parallel processing.
    pub builder_type: KdTreeBuilderType,
    /// Number of threads to use (applies to OpenMP builder).
    pub num_threads: usize,
    /// Maximum number of points in a leaf node.
    pub max_leaf_size: i32,
    /// Projection settings for axis selection.
    pub projection: ProjectionConfig,
}

impl Default for KdTreeConfig {
    fn default() -> Self {
        Self {
            builder_type: KdTreeBuilderType::default(),
            num_threads: default_num_threads(),
            max_leaf_size: 20, // from C++ default kdtree.hpp:129
            projection: ProjectionConfig::default(),
        }
    }
}

/// Configuration for projection axis selection in KdTree.
#[derive(Debug, Clone)]
pub struct ProjectionConfig {
    /// Projection type for splitting strategy.
    pub projection_type: ProjectionType,
    /// Maximum number of points to use for axis search.
    pub max_scan_count: i32,
}

impl Default for ProjectionConfig {
    fn default() -> Self {
        Self {
            projection_type: ProjectionType::default(),
            max_scan_count: 128, // from C++ default projection.hpp:13
        }
    }
}

/// Configuration for KNN search operations.
#[derive(Debug, Clone)]
pub struct KnnConfig {
    /// Early termination threshold for search.
    pub epsilon: f64,
}

impl Default for KnnConfig {
    fn default() -> Self {
        Self {
            epsilon: 0.0, // from C++ default knn_result.hpp:22
        }
    }
}

/// Configuration for normal estimation.
#[derive(Debug, Clone)]
pub struct NormalEstimationConfig {
    /// Number of neighbors to use for normal estimation.
    pub num_neighbors: i32,
    /// Number of threads to use for parallel processing.
    pub num_threads: usize,
}

impl Default for NormalEstimationConfig {
    fn default() -> Self {
        Self {
            num_neighbors: 20, // from C++ default normal_estimation.hpp:118
            num_threads: default_num_threads(),
        }
    }
}

/// Configuration for covariance estimation.
#[derive(Debug, Clone)]
pub struct CovarianceEstimationConfig {
    /// Number of neighbors to use for covariance estimation.
    pub num_neighbors: i32,
    /// Number of threads to use for parallel processing.
    pub num_threads: usize,
}

impl Default for CovarianceEstimationConfig {
    fn default() -> Self {
        Self {
            num_neighbors: 20, // from C++ default normal_estimation.hpp:139
            num_threads: default_num_threads(),
        }
    }
}

/// Configuration for voxel grid downsampling.
#[derive(Debug, Clone)]
pub struct VoxelGridConfig {
    /// Size of each voxel (leaf size).
    pub leaf_size: f64,
    /// Number of threads to use for parallel processing.
    pub num_threads: usize,
}

impl Default for VoxelGridConfig {
    fn default() -> Self {
        Self {
            leaf_size: 0.25,
            num_threads: default_num_threads(),
        }
    }
}

/// Configuration for random sampling.
#[derive(Debug, Clone)]
pub struct RandomSamplingConfig {
    /// Number of points to sample.
    pub num_samples: usize,
    /// Random seed for reproducible results (None for random seed).
    pub seed: Option<u64>,
}

impl Default for RandomSamplingConfig {
    fn default() -> Self {
        Self {
            num_samples: 1000,
            seed: None,
        }
    }
}

/// Configuration for correspondence rejection.
#[derive(Debug, Clone)]
pub struct CorrespondenceRejectorConfig {
    /// Maximum squared distance between corresponding points.
    pub max_dist_sq: f64,
}

impl Default for CorrespondenceRejectorConfig {
    fn default() -> Self {
        Self {
            max_dist_sq: 1.0, // from C++ default rejector.hpp:20
        }
    }
}

/// Configuration for registration termination criteria.
#[derive(Debug, Clone)]
pub struct TerminationConfig {
    /// Translation tolerance for convergence (in meters).
    pub translation_eps: f64,
    /// Rotation tolerance for convergence (in radians).
    pub rotation_eps: f64,
}

impl Default for TerminationConfig {
    fn default() -> Self {
        Self {
            translation_eps: 1e-3,          // from C++ default termination_criteria.hpp:19
            rotation_eps: 0.1 * PI / 180.0, // from C++ default termination_criteria.hpp:20 (0.1 degrees)
        }
    }
}

/// Configuration for Gauss-Newton optimizer.
#[derive(Debug, Clone)]
pub struct GaussNewtonConfig {
    /// Whether to print debug messages.
    pub verbose: bool,
    /// Maximum number of optimization iterations.
    pub max_iterations: i32,
    /// Damping factor (increasing this makes optimization slow but stable).
    pub lambda: f64,
}

impl Default for GaussNewtonConfig {
    fn default() -> Self {
        Self {
            verbose: false,     // from C++ default optimizer.hpp:65
            max_iterations: 20, // from C++ default optimizer.hpp:66
            lambda: 1e-6,       // from C++ default optimizer.hpp:67
        }
    }
}

/// Configuration for Levenberg-Marquardt optimizer.
#[derive(Debug, Clone)]
pub struct LevenbergMarquardtConfig {
    /// Whether to print debug messages.
    pub verbose: bool,
    /// Maximum number of optimization iterations.
    pub max_iterations: i32,
    /// Maximum number of inner iterations (lambda trials).
    pub max_inner_iterations: i32,
    /// Initial lambda (damping factor).
    pub init_lambda: f64,
    /// Lambda increase factor.
    pub lambda_factor: f64,
}

impl Default for LevenbergMarquardtConfig {
    fn default() -> Self {
        Self {
            verbose: false,           // from C++ default optimizer.hpp:150
            max_iterations: 20,       // from C++ default optimizer.hpp:151
            max_inner_iterations: 10, // from C++ default optimizer.hpp:152
            init_lambda: 1e-3,        // from C++ default optimizer.hpp:153
            lambda_factor: 10.0,      // from C++ default optimizer.hpp:154
        }
    }
}

/// Optimizer configuration that can be either Gauss-Newton or Levenberg-Marquardt.
#[derive(Debug, Clone)]
pub enum OptimizerConfig {
    /// Gauss-Newton optimizer configuration.
    GaussNewton(GaussNewtonConfig),
    /// Levenberg-Marquardt optimizer configuration.
    LevenbergMarquardt(LevenbergMarquardtConfig),
}

impl Default for OptimizerConfig {
    fn default() -> Self {
        Self::LevenbergMarquardt(LevenbergMarquardtConfig::default())
    }
}

/// Robust kernel types for outlier rejection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RobustKernelType {
    /// No robust kernel (standard least squares).
    None,
    /// Huber robust kernel - good balance between robustness and efficiency.
    Huber,
    /// Cauchy robust kernel - more aggressive outlier rejection.
    Cauchy,
}

impl Default for RobustKernelType {
    fn default() -> Self {
        Self::None
    }
}

/// Configuration for robust kernel outlier rejection.
#[derive(Debug, Clone)]
pub struct RobustKernelConfig {
    /// Type of robust kernel to use.
    pub kernel_type: RobustKernelType,
    /// Scaling parameter for the robust kernel.
    /// For Huber: threshold for switching between quadratic and linear regions.
    /// For Cauchy: scale parameter controlling influence of outliers.
    pub scale_parameter: f64,
}

impl Default for RobustKernelConfig {
    fn default() -> Self {
        Self {
            kernel_type: RobustKernelType::default(),
            scale_parameter: 1.0, // Typical default scale from C wrapper examples
        }
    }
}

/// Configuration for DOF (Degrees of Freedom) restrictions during registration.
/// This allows constraining the optimization to specific translation and rotation axes.
#[derive(Debug, Clone)]
pub struct DofRestrictionConfig {
    /// Factor that controls the strength of DOF restrictions.
    /// Higher values enforce stronger constraints.
    pub restriction_factor: f64,
    /// Mask for rotation constraints [rx, ry, rz].
    /// 1.0 = allow rotation, 0.0 = restrict rotation on that axis.
    pub rotation_mask: [f64; 3],
    /// Mask for translation constraints [tx, ty, tz].
    /// 1.0 = allow translation, 0.0 = restrict translation on that axis.
    pub translation_mask: [f64; 3],
}

impl Default for DofRestrictionConfig {
    fn default() -> Self {
        Self {
            restriction_factor: 1e-3,          // From C wrapper examples
            rotation_mask: [1.0, 1.0, 1.0],    // Allow all rotations by default
            translation_mask: [1.0, 1.0, 1.0], // Allow all translations by default
        }
    }
}

impl DofRestrictionConfig {
    /// Create a configuration that constrains to 2D registration (no rotation around Z-axis).
    pub fn planar_2d() -> Self {
        Self {
            restriction_factor: 1e-3,
            rotation_mask: [1.0, 1.0, 0.0], // Allow rx, ry, restrict rz
            translation_mask: [1.0, 1.0, 1.0], // Allow all translations
        }
    }

    /// Create a configuration that only allows Z-axis rotation (yaw).
    pub fn yaw_only() -> Self {
        Self {
            restriction_factor: 1e-3,
            rotation_mask: [0.0, 0.0, 1.0], // Restrict rx, ry, allow rz
            translation_mask: [1.0, 1.0, 1.0], // Allow all translations
        }
    }

    /// Create a configuration that constrains to only XY translation.
    pub fn xy_translation_only() -> Self {
        Self {
            restriction_factor: 1e-3,
            rotation_mask: [0.0, 0.0, 0.0], // Restrict all rotations
            translation_mask: [1.0, 1.0, 0.0], // Allow tx, ty, restrict tz
        }
    }
}

/// Configuration for Gaussian voxel map creation.
#[derive(Debug, Clone)]
pub struct GaussianVoxelMapConfig {
    /// Size of each voxel.
    pub voxel_resolution: f64,
    /// Number of threads to use for parallel processing.
    pub num_threads: usize,
}

impl Default for GaussianVoxelMapConfig {
    fn default() -> Self {
        Self {
            voxel_resolution: 1.0,
            num_threads: default_num_threads(),
        }
    }
}

/// Configuration for point cloud preprocessing.
#[derive(Debug, Clone)]
pub struct PreprocessingConfig {
    /// Downsampling configuration.
    pub downsampling: Option<VoxelGridConfig>,
    /// Normal estimation configuration.
    pub normal_estimation: Option<NormalEstimationConfig>,
    /// Covariance estimation configuration.
    pub covariance_estimation: Option<CovarianceEstimationConfig>,
    /// KdTree construction configuration.
    pub kdtree: KdTreeConfig,
}

impl Default for PreprocessingConfig {
    fn default() -> Self {
        Self {
            downsampling: Some(VoxelGridConfig::default()),
            normal_estimation: Some(NormalEstimationConfig::default()),
            covariance_estimation: Some(CovarianceEstimationConfig::default()),
            kdtree: KdTreeConfig::default(),
        }
    }
}

/// Comprehensive registration configuration.
#[derive(Debug, Clone)]
pub struct RegistrationConfig {
    /// Termination criteria for convergence.
    pub termination: TerminationConfig,
    /// Correspondence rejector configuration.
    pub correspondence_rejector: CorrespondenceRejectorConfig,
    /// Optimizer configuration.
    pub optimizer: OptimizerConfig,
    /// Robust kernel configuration for outlier rejection.
    pub robust_kernel: RobustKernelConfig,
    /// DOF restriction configuration for constrained registration.
    pub dof_restriction: Option<DofRestrictionConfig>,
    /// Preprocessing configuration (used if input clouds are not preprocessed).
    pub preprocessing: Option<PreprocessingConfig>,
    /// Number of threads for parallel processing.
    pub num_threads: usize,
}

impl Default for RegistrationConfig {
    fn default() -> Self {
        Self {
            termination: TerminationConfig::default(),
            correspondence_rejector: CorrespondenceRejectorConfig::default(),
            optimizer: OptimizerConfig::default(),
            robust_kernel: RobustKernelConfig::default(),
            dof_restriction: None, // No DOF restrictions by default
            preprocessing: Some(PreprocessingConfig::default()),
            num_threads: default_num_threads(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_configs() {
        // Test that all default configs can be created without panicking
        let _kdtree_config = KdTreeConfig::default();
        let _normal_config = NormalEstimationConfig::default();
        let _cov_config = CovarianceEstimationConfig::default();
        let _voxel_config = VoxelGridConfig::default();
        let _termination_config = TerminationConfig::default();
        let _optimizer_config = OptimizerConfig::default();
        let _registration_config = RegistrationConfig::default();
    }

    #[test]
    fn test_config_values() {
        let normal_config = NormalEstimationConfig::default();
        assert_eq!(normal_config.num_neighbors, 20);

        let termination_config = TerminationConfig::default();
        assert_eq!(termination_config.translation_eps, 1e-3);
        assert!((termination_config.rotation_eps - 0.1 * PI / 180.0).abs() < 1e-10);

        let cov_config = CovarianceEstimationConfig::default();
        assert_eq!(cov_config.num_neighbors, 20);
    }
}
