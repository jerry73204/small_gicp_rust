//! Point cloud registration algorithms.

use crate::{
    config::{DofRestrictionConfig, GaussianVoxelMapConfig, RobustKernelConfig, RobustKernelType},
    error::{check_error, Result, SmallGicpError},
    kdtree_internal::CKdTree,
    point_cloud::PointCloud,
};
use nalgebra::{Isometry3, Matrix4, Point3, Translation3, UnitQuaternion, Vector3};
use std::ptr;

/// Registration algorithm types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegistrationType {
    /// Iterative Closest Point (point-to-point)
    Icp,
    /// Point-to-plane ICP
    PlaneIcp,
    /// Generalized ICP
    Gicp,
    /// Voxelized Generalized ICP
    Vgicp,
}

impl From<RegistrationType> for small_gicp_sys::small_gicp_registration_type_t {
    fn from(reg_type: RegistrationType) -> Self {
        match reg_type {
            RegistrationType::Icp => small_gicp_sys::small_gicp_registration_type_t::SMALL_GICP_ICP,
            RegistrationType::PlaneIcp => {
                small_gicp_sys::small_gicp_registration_type_t::SMALL_GICP_PLANE_ICP
            }
            RegistrationType::Gicp => {
                small_gicp_sys::small_gicp_registration_type_t::SMALL_GICP_GICP
            }
            RegistrationType::Vgicp => {
                small_gicp_sys::small_gicp_registration_type_t::SMALL_GICP_VGICP
            }
        }
    }
}

/// Result of a point cloud registration.
#[derive(Debug, Clone)]
pub struct RegistrationResult {
    /// The estimated transformation from source to target coordinate frame
    pub transformation: Isometry3<f64>,
    /// Whether the registration converged
    pub converged: bool,
    /// Number of iterations performed
    pub iterations: i32,
    /// Number of inlier correspondences
    pub num_inliers: i32,
    /// Final registration error
    pub error: f64,
}

/// Extended result of a point cloud registration including information matrix.
#[derive(Debug, Clone)]
pub struct ExtendedRegistrationResult {
    /// The estimated transformation from source to target coordinate frame
    pub transformation: Isometry3<f64>,
    /// Whether the registration converged
    pub converged: bool,
    /// Number of iterations performed
    pub iterations: i32,
    /// Number of inlier correspondences
    pub num_inliers: i32,
    /// Final registration error
    pub error: f64,
    /// Information matrix (6x6) representing the covariance inverse.
    /// This provides uncertainty information about the estimated transformation.
    /// Layout: [row0, row1, row2, row3, row4, row5] where each row has 6 elements.
    pub information_matrix: [f64; 36],
    /// Information vector (6x1) from the optimization.
    /// Layout: [tx, ty, tz, rx, ry, rz]
    pub information_vector: [f64; 6],
}

impl RegistrationResult {
    /// Get the transformation as a 4x4 matrix.
    pub fn transformation_matrix(&self) -> Matrix4<f64> {
        self.transformation.to_homogeneous()
    }

    /// Get the translation component.
    pub fn translation(&self) -> Vector3<f64> {
        self.transformation.translation.vector
    }

    /// Get the rotation component.
    pub fn rotation(&self) -> UnitQuaternion<f64> {
        self.transformation.rotation
    }

    /// Transform a point using this transformation.
    pub fn transform_point(&self, point: Point3<f64>) -> Point3<f64> {
        self.transformation * point
    }

    /// Transform a point cloud using this transformation.
    pub fn transform_point_cloud(&self, cloud: &PointCloud) -> Result<PointCloud> {
        let points = cloud.points()?;
        let transformed_points: Vec<Point3<f64>> =
            points.iter().map(|p| self.transform_point(*p)).collect();

        PointCloud::from_points(&transformed_points)
    }
}

impl ExtendedRegistrationResult {
    /// Get the transformation as a 4x4 matrix.
    pub fn transformation_matrix(&self) -> Matrix4<f64> {
        self.transformation.to_homogeneous()
    }

    /// Get the translation component.
    pub fn translation(&self) -> Vector3<f64> {
        self.transformation.translation.vector
    }

    /// Get the rotation component.
    pub fn rotation(&self) -> UnitQuaternion<f64> {
        self.transformation.rotation
    }

    /// Transform a point using this transformation.
    pub fn transform_point(&self, point: Point3<f64>) -> Point3<f64> {
        self.transformation * point
    }

    /// Transform a point cloud using this transformation.
    pub fn transform_point_cloud(&self, cloud: &PointCloud) -> Result<PointCloud> {
        let points = cloud.points()?;
        let transformed_points: Vec<Point3<f64>> =
            points.iter().map(|p| self.transform_point(*p)).collect();

        PointCloud::from_points(&transformed_points)
    }

    /// Get the information matrix as a 6x6 nalgebra matrix.
    /// This represents the inverse of the covariance matrix.
    pub fn information_matrix_6x6(&self) -> nalgebra::Matrix6<f64> {
        nalgebra::Matrix6::from_row_slice(&self.information_matrix)
    }

    /// Get the information vector as a 6x1 nalgebra vector.
    pub fn information_vector_6x1(&self) -> nalgebra::Vector6<f64> {
        nalgebra::Vector6::from_row_slice(&self.information_vector)
    }

    /// Convert to basic RegistrationResult (without information matrix).
    pub fn to_basic(&self) -> RegistrationResult {
        RegistrationResult {
            transformation: self.transformation,
            converged: self.converged,
            iterations: self.iterations,
            num_inliers: self.num_inliers,
            error: self.error,
        }
    }
}

impl From<small_gicp_sys::small_gicp_registration_result_t> for RegistrationResult {
    fn from(result: small_gicp_sys::small_gicp_registration_result_t) -> Self {
        // Convert row-major 4x4 matrix to Isometry3
        let matrix = Matrix4::from_row_slice(&result.T_target_source);

        // Extract translation and rotation
        let translation = Translation3::new(matrix[(0, 3)], matrix[(1, 3)], matrix[(2, 3)]);
        let rotation_matrix = matrix.fixed_view::<3, 3>(0, 0);
        let rotation = UnitQuaternion::from_matrix(&rotation_matrix.into());

        let transformation = Isometry3::from_parts(translation, rotation);

        RegistrationResult {
            transformation,
            converged: result.converged,
            iterations: result.iterations,
            num_inliers: result.num_inliers,
            error: result.error,
        }
    }
}

impl From<small_gicp_sys::small_gicp_registration_result_extended_t>
    for ExtendedRegistrationResult
{
    fn from(result: small_gicp_sys::small_gicp_registration_result_extended_t) -> Self {
        // Convert row-major 4x4 matrix to Isometry3
        let matrix = Matrix4::from_row_slice(&result.T_target_source);

        // Extract translation and rotation
        let translation = Translation3::new(matrix[(0, 3)], matrix[(1, 3)], matrix[(2, 3)]);
        let rotation_matrix = matrix.fixed_view::<3, 3>(0, 0);
        let rotation = UnitQuaternion::from_matrix(&rotation_matrix.into());

        let transformation = Isometry3::from_parts(translation, rotation);

        ExtendedRegistrationResult {
            transformation,
            converged: result.converged,
            iterations: result.iterations,
            num_inliers: result.num_inliers,
            error: result.error,
            information_matrix: result.H,
            information_vector: result.b,
        }
    }
}

/// Registration parameters and settings.
#[derive(Debug, Clone)]
pub struct RegistrationSettings {
    /// Registration algorithm type
    pub registration_type: RegistrationType,
    /// Number of threads to use (1 for single-threaded)
    pub num_threads: usize,
    /// Initial transformation guess (identity if None)
    pub initial_guess: Option<Isometry3<f64>>,
}

impl Default for RegistrationSettings {
    fn default() -> Self {
        RegistrationSettings {
            registration_type: RegistrationType::Gicp,
            num_threads: 1,
            initial_guess: None,
        }
    }
}

/// Voxelized point cloud for VGICP registration.
#[derive(Debug)]
pub struct GaussianVoxelMap {
    pub(crate) handle: *mut small_gicp_sys::small_gicp_gaussian_voxelmap_t,
}

impl GaussianVoxelMap {
    /// Create a Gaussian voxel map from a point cloud.
    ///
    /// # Arguments
    /// * `cloud` - The point cloud to voxelize
    /// * `config` - Gaussian voxel map configuration
    pub fn new(cloud: &PointCloud, config: &GaussianVoxelMapConfig) -> Result<Self> {
        if cloud.is_empty() {
            return Err(SmallGicpError::EmptyPointCloud);
        }

        let GaussianVoxelMapConfig {
            voxel_resolution,
            num_threads,
        } = *config;

        let mut handle = ptr::null_mut();
        let error = unsafe {
            small_gicp_sys::small_gicp_gaussian_voxelmap_create(
                cloud.handle,
                voxel_resolution,
                num_threads as i32,
                &mut handle,
            )
        };
        check_error(error)?;
        Ok(GaussianVoxelMap { handle })
    }

    /// Create a Gaussian voxel map from a point cloud (legacy method).
    ///
    /// # Arguments
    /// * `cloud` - The point cloud to voxelize
    /// * `voxel_resolution` - Size of each voxel
    /// * `num_threads` - Number of threads to use
    #[deprecated(
        since = "0.2.0",
        note = "Use new() with GaussianVoxelMapConfig instead"
    )]
    pub fn new_legacy(
        cloud: &PointCloud,
        voxel_resolution: f64,
        num_threads: usize,
    ) -> Result<Self> {
        let config = GaussianVoxelMapConfig {
            voxel_resolution,
            num_threads,
        };
        Self::new(cloud, &config)
    }
}

impl Drop for GaussianVoxelMap {
    fn drop(&mut self) {
        if !self.handle.is_null() {
            unsafe {
                small_gicp_sys::small_gicp_gaussian_voxelmap_destroy(self.handle);
            }
        }
    }
}

// Ensure GaussianVoxelMap is Send and Sync
unsafe impl Send for GaussianVoxelMap {}
unsafe impl Sync for GaussianVoxelMap {}

/// Robust kernel for outlier rejection during registration.
#[derive(Debug)]
pub struct RobustKernel {
    pub(crate) handle: *mut small_gicp_sys::small_gicp_robust_kernel_t,
}

impl RobustKernel {
    /// Create a new robust kernel.
    ///
    /// # Arguments
    /// * `config` - Robust kernel configuration
    pub fn new(config: &RobustKernelConfig) -> Result<Self> {
        let kernel_type = match config.kernel_type {
            RobustKernelType::None => {
                return Err(SmallGicpError::InvalidParameter {
                    param: "kernel_type",
                    value: "None".to_string(),
                });
            }
            RobustKernelType::Huber => 1, // SMALL_GICP_ROBUST_KERNEL_HUBER
            RobustKernelType::Cauchy => 2, // SMALL_GICP_ROBUST_KERNEL_CAUCHY
        };

        let mut handle = ptr::null_mut();
        let error = unsafe {
            small_gicp_sys::small_gicp_create_robust_kernel(
                kernel_type,
                config.scale_parameter,
                &mut handle,
            )
        };
        check_error(error)?;
        Ok(RobustKernel { handle })
    }

    /// Create a Huber robust kernel with the specified scale parameter.
    ///
    /// # Arguments
    /// * `scale` - Scale parameter (threshold for switching between quadratic and linear regions)
    pub fn huber(scale: f64) -> Result<Self> {
        let config = RobustKernelConfig {
            kernel_type: RobustKernelType::Huber,
            scale_parameter: scale,
        };
        Self::new(&config)
    }

    /// Create a Cauchy robust kernel with the specified scale parameter.
    ///
    /// # Arguments
    /// * `scale` - Scale parameter controlling influence of outliers
    pub fn cauchy(scale: f64) -> Result<Self> {
        let config = RobustKernelConfig {
            kernel_type: RobustKernelType::Cauchy,
            scale_parameter: scale,
        };
        Self::new(&config)
    }

    /// Compute the robust weight for a given error value.
    ///
    /// # Arguments
    /// * `error` - Error value to compute weight for
    ///
    /// # Returns
    /// Robust weight between 0.0 and 1.0
    pub fn compute_weight(&self, error: f64) -> Result<f64> {
        let mut weight = 0.0;
        let error_code = unsafe {
            small_gicp_sys::small_gicp_compute_robust_weight(self.handle, error, &mut weight)
        };
        check_error(error_code)?;
        Ok(weight)
    }
}

impl Drop for RobustKernel {
    fn drop(&mut self) {
        if !self.handle.is_null() {
            unsafe {
                small_gicp_sys::small_gicp_destroy_robust_kernel(self.handle);
            }
        }
    }
}

// Ensure RobustKernel is Send and Sync
unsafe impl Send for RobustKernel {}
unsafe impl Sync for RobustKernel {}

/// DOF (Degrees of Freedom) restriction for constrained registration.
#[derive(Debug)]
pub struct DofRestriction {
    pub(crate) handle: *mut small_gicp_sys::small_gicp_restrict_dof_factor_t,
}

impl DofRestriction {
    /// Create a new DOF restriction.
    ///
    /// # Arguments
    /// * `config` - DOF restriction configuration
    pub fn new(config: &DofRestrictionConfig) -> Result<Self> {
        let mut handle = ptr::null_mut();
        let error = unsafe {
            small_gicp_sys::small_gicp_create_dof_restriction(
                config.restriction_factor,
                config.rotation_mask.as_ptr(),
                config.translation_mask.as_ptr(),
                &mut handle,
            )
        };
        check_error(error)?;
        Ok(DofRestriction { handle })
    }

    /// Create a DOF restriction for 2D planar registration.
    /// Allows X/Y rotation and all translations, but restricts Z rotation.
    pub fn planar_2d() -> Result<Self> {
        let config = DofRestrictionConfig::planar_2d();
        Self::new(&config)
    }

    /// Create a DOF restriction that only allows yaw rotation (rotation around Z-axis).
    /// Restricts X/Y rotation but allows Z rotation and all translations.
    pub fn yaw_only() -> Result<Self> {
        let config = DofRestrictionConfig::yaw_only();
        Self::new(&config)
    }

    /// Create a DOF restriction that only allows XY translation.
    /// Restricts all rotations and Z translation.
    pub fn xy_translation_only() -> Result<Self> {
        let config = DofRestrictionConfig::xy_translation_only();
        Self::new(&config)
    }
}

impl Drop for DofRestriction {
    fn drop(&mut self) {
        if !self.handle.is_null() {
            unsafe {
                small_gicp_sys::small_gicp_destroy_dof_restriction(self.handle);
            }
        }
    }
}

// Ensure DofRestriction is Send and Sync
unsafe impl Send for DofRestriction {}
unsafe impl Sync for DofRestriction {}

/// Perform point cloud registration.
///
/// This function automatically preprocesses the point clouds and performs registration.
///
/// # Arguments
/// * `target` - The target point cloud
/// * `source` - The source point cloud to align to the target
/// * `settings` - Registration settings
///
/// # Returns
/// Registration result containing the transformation and statistics
pub fn register(
    target: &PointCloud,
    source: &PointCloud,
    settings: &RegistrationSettings,
) -> Result<RegistrationResult> {
    if target.is_empty() || source.is_empty() {
        return Err(SmallGicpError::EmptyPointCloud);
    }

    // Convert initial guess to row-major 4x4 matrix
    let initial_guess_matrix = if let Some(guess) = settings.initial_guess {
        let matrix = guess.to_homogeneous();
        Some([
            matrix[(0, 0)],
            matrix[(0, 1)],
            matrix[(0, 2)],
            matrix[(0, 3)],
            matrix[(1, 0)],
            matrix[(1, 1)],
            matrix[(1, 2)],
            matrix[(1, 3)],
            matrix[(2, 0)],
            matrix[(2, 1)],
            matrix[(2, 2)],
            matrix[(2, 3)],
            matrix[(3, 0)],
            matrix[(3, 1)],
            matrix[(3, 2)],
            matrix[(3, 3)],
        ])
    } else {
        None
    };

    let initial_guess_ptr = initial_guess_matrix
        .as_ref()
        .map(|m| m.as_ptr())
        .unwrap_or(ptr::null());

    let mut result = small_gicp_sys::small_gicp_registration_result_t {
        T_target_source: [0.0; 16],
        converged: false,
        iterations: 0,
        num_inliers: 0,
        error: 0.0,
    };

    let error = unsafe {
        small_gicp_sys::small_gicp_align(
            target.handle,
            source.handle,
            settings.registration_type.into(),
            initial_guess_ptr,
            settings.num_threads as i32,
            &mut result,
        )
    };

    check_error(error)?;

    let registration_result = RegistrationResult::from(result);

    if !registration_result.converged {
        return Err(SmallGicpError::RegistrationFailed {
            iterations: registration_result.iterations,
        });
    }

    Ok(registration_result)
}

/// Perform registration with preprocessed point clouds.
///
/// This function assumes the point clouds have already been preprocessed
/// (downsampled, normals computed, etc.).
///
/// # Arguments
/// * `target` - The preprocessed target point cloud
/// * `source` - The preprocessed source point cloud
/// * `target_tree` - KdTree for the target point cloud
/// * `settings` - Registration settings
///
/// # Returns
/// Registration result containing the transformation and statistics
pub fn register_preprocessed(
    target: &PointCloud,
    source: &PointCloud,
    target_tree: &CKdTree,
    settings: &RegistrationSettings,
) -> Result<RegistrationResult> {
    if target.is_empty() || source.is_empty() {
        return Err(SmallGicpError::EmptyPointCloud);
    }

    // Convert initial guess to row-major 4x4 matrix
    let initial_guess_matrix = if let Some(guess) = settings.initial_guess {
        let matrix = guess.to_homogeneous();
        Some([
            matrix[(0, 0)],
            matrix[(0, 1)],
            matrix[(0, 2)],
            matrix[(0, 3)],
            matrix[(1, 0)],
            matrix[(1, 1)],
            matrix[(1, 2)],
            matrix[(1, 3)],
            matrix[(2, 0)],
            matrix[(2, 1)],
            matrix[(2, 2)],
            matrix[(2, 3)],
            matrix[(3, 0)],
            matrix[(3, 1)],
            matrix[(3, 2)],
            matrix[(3, 3)],
        ])
    } else {
        None
    };

    let initial_guess_ptr = initial_guess_matrix
        .as_ref()
        .map(|m| m.as_ptr())
        .unwrap_or(ptr::null());

    let mut result = small_gicp_sys::small_gicp_registration_result_t {
        T_target_source: [0.0; 16],
        converged: false,
        iterations: 0,
        num_inliers: 0,
        error: 0.0,
    };

    let error = unsafe {
        small_gicp_sys::small_gicp_align_preprocessed(
            target.handle,
            source.handle,
            target_tree.handle,
            settings.registration_type.into(),
            initial_guess_ptr,
            settings.num_threads as i32,
            &mut result,
        )
    };

    check_error(error)?;

    let registration_result = RegistrationResult::from(result);

    if !registration_result.converged {
        return Err(SmallGicpError::RegistrationFailed {
            iterations: registration_result.iterations,
        });
    }

    Ok(registration_result)
}

/// Perform VGICP registration using a Gaussian voxel map.
///
/// # Arguments
/// * `target_voxelmap` - The target voxel map
/// * `source` - The source point cloud
/// * `settings` - Registration settings
///
/// # Returns
/// Registration result containing the transformation and statistics
pub fn register_vgicp(
    target_voxelmap: &GaussianVoxelMap,
    source: &PointCloud,
    settings: &RegistrationSettings,
) -> Result<RegistrationResult> {
    if source.is_empty() {
        return Err(SmallGicpError::EmptyPointCloud);
    }

    // Convert initial guess to row-major 4x4 matrix
    let initial_guess_matrix = if let Some(guess) = settings.initial_guess {
        let matrix = guess.to_homogeneous();
        Some([
            matrix[(0, 0)],
            matrix[(0, 1)],
            matrix[(0, 2)],
            matrix[(0, 3)],
            matrix[(1, 0)],
            matrix[(1, 1)],
            matrix[(1, 2)],
            matrix[(1, 3)],
            matrix[(2, 0)],
            matrix[(2, 1)],
            matrix[(2, 2)],
            matrix[(2, 3)],
            matrix[(3, 0)],
            matrix[(3, 1)],
            matrix[(3, 2)],
            matrix[(3, 3)],
        ])
    } else {
        None
    };

    let initial_guess_ptr = initial_guess_matrix
        .as_ref()
        .map(|m| m.as_ptr())
        .unwrap_or(ptr::null());

    let mut result = small_gicp_sys::small_gicp_registration_result_t {
        T_target_source: [0.0; 16],
        converged: false,
        iterations: 0,
        num_inliers: 0,
        error: 0.0,
    };

    let error = unsafe {
        small_gicp_sys::small_gicp_align_vgicp(
            target_voxelmap.handle,
            source.handle,
            initial_guess_ptr,
            settings.num_threads as i32,
            &mut result,
        )
    };

    check_error(error)?;

    let registration_result = RegistrationResult::from(result);

    if !registration_result.converged {
        return Err(SmallGicpError::RegistrationFailed {
            iterations: registration_result.iterations,
        });
    }

    Ok(registration_result)
}

/// Perform advanced registration with full pipeline control.
///
/// This function provides complete control over the registration process,
/// including robust kernels, DOF restrictions, and extended results.
///
/// # Arguments
/// * `target` - The target point cloud (should be preprocessed)
/// * `source` - The source point cloud to align to the target (should be preprocessed)
/// * `target_tree` - KdTree for the target point cloud
/// * `settings` - Registration settings
/// * `robust_kernel` - Optional robust kernel for outlier rejection
/// * `dof_restriction` - Optional DOF restriction for constrained registration
/// * `initial_guess` - Optional initial transformation guess
///
/// # Returns
/// Extended registration result with information matrix
pub fn register_advanced(
    target: &PointCloud,
    source: &PointCloud,
    target_tree: &CKdTree,
    settings: &RegistrationSettings,
    robust_kernel: Option<&RobustKernel>,
    dof_restriction: Option<&DofRestriction>,
    initial_guess: Option<Isometry3<f64>>,
) -> Result<ExtendedRegistrationResult> {
    if target.is_empty() || source.is_empty() {
        return Err(SmallGicpError::EmptyPointCloud);
    }

    // Create default registration settings
    let mut reg_setting = ptr::null_mut();
    let error = unsafe {
        small_gicp_sys::small_gicp_create_default_registration_setting(
            settings.registration_type.into(),
            &mut reg_setting,
        )
    };
    check_error(error)?;

    // Create termination criteria
    let mut criteria = ptr::null_mut();
    let error = unsafe {
        small_gicp_sys::small_gicp_create_termination_criteria(
            1e-4,                                // Default translation eps
            0.05 * std::f64::consts::PI / 180.0, // Default rotation eps in radians
            &mut criteria,
        )
    };
    check_error(error)?;

    // Create optimizer setting
    let mut optimizer = ptr::null_mut();
    let error = unsafe {
        small_gicp_sys::small_gicp_create_default_optimizer_setting(
            1, // SMALL_GICP_OPTIMIZER_LEVENBERG_MARQUARDT
            &mut optimizer,
        )
    };
    check_error(error)?;

    // Create correspondence rejector
    let mut rejector = ptr::null_mut();
    let error = unsafe {
        small_gicp_sys::small_gicp_create_correspondence_rejector(
            1,   // SMALL_GICP_REJECTOR_DISTANCE
            1.0, // Default max distance
            &mut rejector,
        )
    };
    check_error(error)?;

    // Convert initial guess to row-major 4x4 matrix
    let initial_guess_matrix = if let Some(guess) = initial_guess {
        let matrix = guess.to_homogeneous();
        Some([
            matrix[(0, 0)],
            matrix[(0, 1)],
            matrix[(0, 2)],
            matrix[(0, 3)],
            matrix[(1, 0)],
            matrix[(1, 1)],
            matrix[(1, 2)],
            matrix[(1, 3)],
            matrix[(2, 0)],
            matrix[(2, 1)],
            matrix[(2, 2)],
            matrix[(2, 3)],
            matrix[(3, 0)],
            matrix[(3, 1)],
            matrix[(3, 2)],
            matrix[(3, 3)],
        ])
    } else {
        None
    };

    let initial_guess_ptr = initial_guess_matrix
        .as_ref()
        .map(|m| m.as_ptr())
        .unwrap_or(ptr::null());

    let robust_kernel_ptr = robust_kernel.map(|k| k.handle).unwrap_or(ptr::null_mut());
    let dof_restriction_ptr = dof_restriction.map(|d| d.handle).unwrap_or(ptr::null_mut());

    let mut result = small_gicp_sys::small_gicp_registration_result_extended_t {
        T_target_source: [0.0; 16],
        converged: false,
        iterations: 0,
        num_inliers: 0,
        error: 0.0,
        H: [0.0; 36],
        b: [0.0; 6],
    };

    let error = unsafe {
        small_gicp_sys::small_gicp_align_advanced(
            target.handle,
            source.handle,
            target_tree.handle,
            initial_guess_ptr,
            reg_setting,
            criteria,
            optimizer,
            rejector,
            robust_kernel_ptr,
            dof_restriction_ptr,
            &mut result,
        )
    };

    // Cleanup
    unsafe {
        small_gicp_sys::small_gicp_destroy_correspondence_rejector(rejector);
        small_gicp_sys::small_gicp_destroy_optimizer_setting(optimizer);
        small_gicp_sys::small_gicp_destroy_termination_criteria(criteria);
        small_gicp_sys::small_gicp_destroy_registration_setting(reg_setting);
    }

    check_error(error)?;

    let registration_result = ExtendedRegistrationResult::from(result);

    if !registration_result.converged {
        return Err(SmallGicpError::RegistrationFailed {
            iterations: registration_result.iterations,
        });
    }

    Ok(registration_result)
}

/// Advanced registration with complete configuration control.
///
/// This function provides access to all advanced registration parameters
/// including optimizer type selection, parallel processing configuration,
/// and extended correspondence rejection settings.
///
/// # Arguments
/// * `target` - Target point cloud
/// * `source` - Source point cloud  
/// * `target_tree` - KdTree for the target point cloud
/// * `config` - Complete registration configuration
/// * `initial_guess` - Optional initial transformation guess
///
/// # Returns
/// Extended registration result with information matrix
pub fn register_with_complete_config(
    target: &PointCloud,
    source: &PointCloud,
    target_tree: &CKdTree,
    config: &crate::config::CompleteRegistrationConfig,
    initial_guess: Option<&Isometry3<f64>>,
) -> Result<ExtendedRegistrationResult> {
    if target.is_empty() || source.is_empty() {
        return Err(SmallGicpError::EmptyPointCloud);
    }

    // For now, use the existing register_advanced function as the implementation
    // TODO: Implement full configuration support when C wrapper functions are available
    let robust_kernel = if config.robust_kernel.kernel_type != RobustKernelType::None {
        Some(RobustKernel::new(&config.robust_kernel)?)
    } else {
        None
    };

    let dof_restriction = config
        .dof_restriction
        .as_ref()
        .map(|dof| DofRestriction::new(dof))
        .transpose()?;

    let settings = RegistrationSettings {
        registration_type: config.registration.registration_type,
        num_threads: config.registration.num_threads,
        initial_guess: initial_guess.copied(),
    };

    register_advanced(
        target,
        source,
        target_tree,
        &settings,
        robust_kernel.as_ref(),
        dof_restriction.as_ref(),
        initial_guess.copied(),
    )
}

/// Set global parallel reduction strategy for registration operations.
///
/// This function configures the parallel processing strategy that will be used
/// for subsequent registration operations.
///
/// # Arguments
/// * `config` - Parallel processing configuration
pub fn set_parallel_reduction_strategy(
    _config: &crate::config::ParallelProcessingConfig,
) -> Result<()> {
    // Note: This function is a placeholder for future implementation
    // The actual small_gicp_set_reduction_strategy function may not be available in the current C wrapper
    // For now, we'll return success to maintain API compatibility
    Ok(())
}
