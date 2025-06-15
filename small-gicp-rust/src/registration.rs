//! Point cloud registration algorithms and utilities.

use crate::{
    config::{DofRestrictionConfig, RobustKernelConfig, RobustKernelType},
    error::{Result, SmallGicpError},
    kdtree::KdTree,
    point_cloud::PointCloud,
};
use nalgebra::{Isometry3, Matrix4};

/// Settings for point cloud registration algorithms.
#[derive(Debug, Clone)]
pub struct RegistrationSettings {
    /// Maximum number of iterations.
    pub max_iterations: u32,
    /// Convergence threshold for rotation (radians).
    pub rotation_epsilon: f64,
    /// Convergence threshold for translation (meters).
    pub transformation_epsilon: f64,
    /// Maximum correspondence distance.
    pub max_correspondence_distance: f64,
    /// Number of threads to use (0 = auto).
    pub num_threads: u32,
    /// Type of registration algorithm to use.
    pub registration_type: RegistrationType,
    /// Initial guess for the transformation.
    pub initial_guess: Option<Isometry3<f64>>,
}

impl Default for RegistrationSettings {
    fn default() -> Self {
        Self {
            max_iterations: 50,
            rotation_epsilon: 1e-6,
            transformation_epsilon: 1e-6,
            max_correspondence_distance: 1.0,
            num_threads: 0,
            registration_type: RegistrationType::Icp,
            initial_guess: None,
        }
    }
}

impl RegistrationSettings {
    /// Create new registration settings with the given registration type.
    pub fn new(registration_type: RegistrationType) -> Self {
        Self {
            registration_type,
            ..Self::default()
        }
    }

    /// Set the number of threads.
    pub fn with_num_threads(mut self, num_threads: u32) -> Self {
        self.num_threads = num_threads;
        self
    }

    /// Set the initial guess transformation.
    pub fn with_initial_guess(mut self, initial_guess: Option<Isometry3<f64>>) -> Self {
        self.initial_guess = initial_guess;
        self
    }

    /// Set the maximum number of iterations.
    pub fn with_max_iterations(mut self, max_iterations: u32) -> Self {
        self.max_iterations = max_iterations;
        self
    }

    /// Set the maximum correspondence distance.
    pub fn with_max_correspondence_distance(mut self, distance: f64) -> Self {
        self.max_correspondence_distance = distance;
        self
    }
}

/// Result of a point cloud registration.
#[derive(Debug, Clone)]
pub struct RegistrationResult {
    /// Final transformation matrix.
    pub transformation: Matrix4<f64>,
    /// Whether the registration converged.
    pub converged: bool,
    /// Number of iterations performed.
    pub iterations: u32,
    /// Final registration error.
    pub error: f64,
    /// Number of inlier correspondences.
    pub num_inliers: usize,
}

impl RegistrationResult {
    /// Extract the translation part of the transformation.
    pub fn translation(&self) -> nalgebra::Vector3<f64> {
        todo!("Extract translation from transformation matrix")
    }

    /// Extract the rotation part of the transformation.
    pub fn rotation(&self) -> nalgebra::UnitQuaternion<f64> {
        todo!("Extract rotation from transformation matrix")
    }

    /// Transform a point using the registration result transformation.
    pub fn transform_point(&self, point: nalgebra::Point3<f64>) -> nalgebra::Point3<f64> {
        todo!("Transform point using self.transformation matrix")
    }
}

/// Extended registration result with additional information.
#[derive(Debug, Clone)]
pub struct ExtendedRegistrationResult {
    /// Basic registration result.
    pub result: RegistrationResult,
    /// Information matrix (inverse covariance).
    pub information_matrix: Matrix4<f64>,
    /// Number of valid correspondences.
    pub num_correspondences: usize,
}

/// Type of registration algorithm to use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegistrationType {
    /// Point-to-point ICP.
    Icp,
    /// Point-to-plane ICP.
    PlaneIcp,
    /// Generalized ICP.
    Gicp,
    /// Voxelized GICP.
    Vgicp,
}

/// Robust kernel for outlier rejection.
#[derive(Debug, Clone)]
pub struct RobustKernel {
    /// Type of robust kernel.
    pub kernel_type: RobustKernelType,
    /// Kernel parameter (threshold).
    pub parameter: f64,
}

impl RobustKernel {
    /// Create a new robust kernel from type and parameter.
    pub fn with_type(kernel_type: RobustKernelType, parameter: f64) -> Self {
        Self {
            kernel_type,
            parameter,
        }
    }

    /// Create a new robust kernel from configuration.
    pub fn from_config(config: &RobustKernelConfig) -> Result<Self> {
        if config.kernel_type == RobustKernelType::None {
            return Err(SmallGicpError::InvalidParameter {
                param: "kernel_type",
                value: "None is not a valid kernel type".to_string(),
            });
        }
        Ok(Self {
            kernel_type: config.kernel_type,
            parameter: config.scale_parameter,
        })
    }

    /// Create a Huber robust kernel with the given threshold.
    pub fn huber(threshold: f64) -> Result<Self> {
        Ok(Self {
            kernel_type: RobustKernelType::Huber,
            parameter: threshold,
        })
    }

    /// Compute the weight for a given error value.
    pub fn compute_weight(&self, error: f64) -> Result<f64> {
        todo!("Implement robust kernel weight computation using small-gicp-cxx")
    }

    /// Create a Cauchy robust kernel with the given threshold.
    pub fn cauchy(threshold: f64) -> Result<Self> {
        Ok(Self {
            kernel_type: RobustKernelType::Cauchy,
            parameter: threshold,
        })
    }

    /// Create a new robust kernel from configuration (for tests).
    pub fn new(config: &RobustKernelConfig) -> Result<Self> {
        Self::from_config(config)
    }
}

/// Degrees of freedom restriction for registration.
#[derive(Debug, Clone)]
pub struct DofRestriction {
    /// Mask for translation DOF (x, y, z).
    pub translation_mask: [bool; 3],
    /// Mask for rotation DOF (roll, pitch, yaw).
    pub rotation_mask: [bool; 3],
}

impl DofRestriction {
    /// Create a new DOF restriction with custom masks.
    pub fn with_masks(translation_mask: [bool; 3], rotation_mask: [bool; 3]) -> Self {
        Self {
            translation_mask,
            rotation_mask,
        }
    }

    /// Create a new DOF restriction from configuration.
    pub fn from_config(config: &DofRestrictionConfig) -> Result<Self> {
        // Convert f64 masks to bool masks (> 0.5 = true)
        let translation_mask = [
            config.translation_mask[0] > 0.5,
            config.translation_mask[1] > 0.5,
            config.translation_mask[2] > 0.5,
        ];
        let rotation_mask = [
            config.rotation_mask[0] > 0.5,
            config.rotation_mask[1] > 0.5,
            config.rotation_mask[2] > 0.5,
        ];
        Ok(Self {
            translation_mask,
            rotation_mask,
        })
    }

    /// No DOF restrictions (6-DOF registration).
    pub fn none() -> Self {
        Self {
            translation_mask: [true, true, true],
            rotation_mask: [true, true, true],
        }
    }

    /// Restrict to 2D planar motion (x, y, yaw).
    pub fn planar_2d() -> Result<Self> {
        Ok(Self {
            translation_mask: [true, true, false],
            rotation_mask: [false, false, true],
        })
    }

    /// Restrict to yaw rotation only.
    pub fn yaw_only() -> Result<Self> {
        Ok(Self {
            translation_mask: [true, true, true],
            rotation_mask: [false, false, true],
        })
    }

    /// Restrict to XY translation only.
    pub fn xy_translation_only() -> Result<Self> {
        Ok(Self {
            translation_mask: [true, true, false],
            rotation_mask: [false, false, false],
        })
    }

    /// Create a new DOF restriction from configuration (for tests).
    pub fn new(config: &DofRestrictionConfig) -> Result<Self> {
        Self::from_config(config)
    }
}

/// Gaussian voxel map for VGICP registration.
pub struct GaussianVoxelMap {
    // TODO: Wrap small_gicp_cxx::VoxelMap
    inner: small_gicp_cxx::VoxelMap,
}

impl GaussianVoxelMap {
    /// Create a new Gaussian voxel map from a point cloud and config.
    pub fn new<T>(cloud: &PointCloud, config: &T) -> Result<Self> {
        todo!("Implement using small_gicp_cxx::VoxelMap with point cloud and config")
    }

    /// Create a new Gaussian voxel map with voxel size.
    pub fn with_voxel_size(voxel_size: f64) -> Self {
        todo!("Implement using small_gicp_cxx::VoxelMap::new(voxel_size)")
    }

    /// Insert a point cloud into the voxel map.
    pub fn insert(&mut self, cloud: &PointCloud) -> Result<()> {
        todo!("Implement using inner.insert(cloud.inner())")
    }

    /// Get the number of voxels.
    pub fn size(&self) -> usize {
        todo!("Implement using inner.size()")
    }
}

/// Perform point cloud registration using ICP algorithm.
pub fn register(
    target: &PointCloud,
    source: &PointCloud,
    settings: &RegistrationSettings,
) -> Result<RegistrationResult> {
    todo!("Implement using small_gicp_cxx registration functions")
}

/// Perform point cloud registration with a pre-built target KdTree.
pub fn register_with_kdtree(
    target: &PointCloud,
    source: &PointCloud,
    target_tree: &KdTree,
    settings: &RegistrationSettings,
) -> Result<RegistrationResult> {
    todo!("Implement using small_gicp_cxx registration functions with KdTree")
}

/// Perform advanced registration with additional options.
pub fn register_advanced(
    target: &PointCloud,
    source: &PointCloud,
    registration_type: RegistrationType,
    initial_guess: Option<&Isometry3<f64>>,
    robust_kernel: Option<&RobustKernel>,
    dof_restriction: Option<&DofRestriction>,
    settings: &RegistrationSettings,
) -> Result<ExtendedRegistrationResult> {
    todo!("Implement using small_gicp_cxx advanced registration functions")
}

/// Perform registration using preprocessed point clouds.
pub fn register_preprocessed(
    target: &PointCloud,
    source: &PointCloud,
    target_tree: &KdTree,
    settings: &RegistrationSettings,
) -> Result<RegistrationResult> {
    todo!("Implement using small_gicp_cxx registration functions")
}

/// Perform VGICP registration using a Gaussian voxel map.
pub fn register_vgicp(
    target_voxelmap: &GaussianVoxelMap,
    source: &PointCloud,
    initial_guess: Option<&Isometry3<f64>>,
    settings: &RegistrationSettings,
) -> Result<RegistrationResult> {
    todo!("Implement using small_gicp_cxx VGICP registration")
}

/// Estimate the transformation between two point clouds using correspondences.
pub fn estimate_transformation(
    source_points: &[(f64, f64, f64)],
    target_points: &[(f64, f64, f64)],
) -> Result<Matrix4<f64>> {
    todo!("Implement transformation estimation from correspondences")
}

/// Compute registration error for a given transformation.
pub fn compute_error(
    source: &PointCloud,
    target: &PointCloud,
    transformation: &Matrix4<f64>,
    max_correspondence_distance: f64,
) -> Result<f64> {
    todo!("Implement error computation using small_gicp_cxx functions")
}

/// Find correspondences between two point clouds.
pub fn find_correspondences(
    source: &PointCloud,
    target: &PointCloud,
    target_tree: &KdTree,
    max_distance: f64,
) -> Result<Vec<(usize, usize, f64)>> {
    todo!("Implement correspondence finding using KdTree search")
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    #[test]
    fn test_registration_settings_default() {
        let settings = RegistrationSettings::default();
        assert_eq!(settings.max_iterations, 50);
        assert_eq!(settings.num_threads, 0);
    }

    #[test]
    fn test_dof_restriction() {
        let none = DofRestriction::none();
        assert!(none.translation_mask.iter().all(|&x| x));
        assert!(none.rotation_mask.iter().all(|&x| x));

        let planar = DofRestriction::planar_2d().unwrap();
        assert_eq!(planar.translation_mask, [true, true, false]);
        assert_eq!(planar.rotation_mask, [false, false, true]);
    }

    #[test]
    fn test_registration_type() {
        let reg_type = RegistrationType::Icp;
        assert_eq!(reg_type, RegistrationType::Icp);
        assert_ne!(reg_type, RegistrationType::Gicp);
    }

    #[test]
    fn test_robust_kernel() {
        let kernel = RobustKernel {
            kernel_type: RobustKernelType::Huber,
            parameter: 1.0,
        };
        assert_eq!(kernel.kernel_type, RobustKernelType::Huber);
        assert_eq!(kernel.parameter, 1.0);
    }

    // TODO: Add integration tests once implementation is complete
    // #[test]
    // fn test_basic_registration() {
    //     let target_points = vec![
    //         Point3::new(0.0, 0.0, 0.0),
    //         Point3::new(1.0, 0.0, 0.0),
    //         Point3::new(0.0, 1.0, 0.0),
    //     ];
    //     let source_points = vec![
    //         Point3::new(0.1, 0.1, 0.0),
    //         Point3::new(1.1, 0.1, 0.0),
    //         Point3::new(0.1, 1.1, 0.0),
    //     ];
    //
    //     let target = PointCloud::from_points(&target_points).unwrap();
    //     let source = PointCloud::from_points(&source_points).unwrap();
    //     let settings = RegistrationSettings::default();
    //
    //     let result = register(&target, &source, &settings).unwrap();
    //     assert!(result.converged);
    //     assert!(result.error < 0.1);
    // }
}
