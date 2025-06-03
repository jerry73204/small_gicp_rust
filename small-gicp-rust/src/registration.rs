//! Point cloud registration algorithms.

use crate::{
    config::GaussianVoxelMapConfig,
    error::{check_error, Result, SmallGicpError},
    kdtree::KdTree,
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
    target_tree: &KdTree,
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
