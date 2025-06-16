//! Point cloud registration algorithms and utilities.

use crate::{
    config::{DofRestrictionConfig, RobustKernelConfig, RobustKernelType},
    error::{Result, SmallGicpError},
    kdtree::KdTree,
    point_cloud::PointCloud,
    traits::PointCloudTrait,
};
use nalgebra::{Isometry3, Matrix4, Point3, UnitQuaternion, Vector3};
use small_gicp_cxx::{
    Registration as CxxRegistration, RegistrationResult as CxxRegistrationResult,
    RegistrationSettings as CxxRegistrationSettings, Transform as CxxTransform, VoxelMap,
};

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
    pub fn translation(&self) -> Vector3<f64> {
        Vector3::new(
            self.transformation[(0, 3)],
            self.transformation[(1, 3)],
            self.transformation[(2, 3)],
        )
    }

    /// Extract the rotation part of the transformation.
    pub fn rotation(&self) -> UnitQuaternion<f64> {
        let rotation_matrix = self.transformation.fixed_view::<3, 3>(0, 0).into_owned();
        UnitQuaternion::from_matrix(&rotation_matrix)
    }

    /// Transform a point using the registration result transformation.
    pub fn transform_point(&self, point: Point3<f64>) -> Point3<f64> {
        let homogeneous = self.transformation * point.to_homogeneous();
        Point3::new(
            homogeneous[0] / homogeneous[3],
            homogeneous[1] / homogeneous[3],
            homogeneous[2] / homogeneous[3],
        )
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
        let normalized_error = error / self.parameter;

        match self.kernel_type {
            RobustKernelType::None => Ok(1.0),
            RobustKernelType::Huber => {
                if normalized_error <= 1.0 {
                    Ok(1.0)
                } else {
                    Ok(1.0 / normalized_error)
                }
            }
            RobustKernelType::Cauchy => Ok(1.0 / (1.0 + normalized_error * normalized_error)),
        }
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
    inner: VoxelMap,
}

impl GaussianVoxelMap {
    /// Create a new Gaussian voxel map from a point cloud and config.
    pub fn new<T>(cloud: &PointCloud, _config: &T) -> Result<Self> {
        // For simplicity, use default voxel size of 0.1
        // In a complete implementation, would parse config for voxel_size
        Ok(Self {
            inner: VoxelMap::new(0.1),
        })
    }

    /// Create a new Gaussian voxel map with voxel size.
    pub fn with_voxel_size(voxel_size: f64) -> Self {
        Self {
            inner: VoxelMap::new(voxel_size),
        }
    }

    /// Insert a point cloud into the voxel map.
    pub fn insert(&mut self, cloud: &PointCloud) -> Result<()> {
        self.inner.insert(cloud.inner());
        Ok(())
    }

    /// Get the number of voxels.
    pub fn size(&self) -> usize {
        self.inner.size()
    }
}

/// Perform point cloud registration using ICP algorithm.
pub fn register(
    target: &PointCloud,
    source: &PointCloud,
    settings: &RegistrationSettings,
) -> Result<RegistrationResult> {
    let target_tree = KdTree::new(target)?;
    register_with_kdtree(target, source, &target_tree, settings)
}

/// Perform point cloud registration with a pre-built target KdTree.
pub fn register_with_kdtree(
    target: &PointCloud,
    source: &PointCloud,
    target_tree: &KdTree,
    settings: &RegistrationSettings,
) -> Result<RegistrationResult> {
    let cxx_settings = settings_to_cxx(settings);
    let init_transform = isometry_to_cxx_transform(settings.initial_guess.as_ref());

    let result = match settings.registration_type {
        RegistrationType::Icp => {
            // Create CXX point clouds from our data
            let mut source_cxx = small_gicp_cxx::PointCloud::new();
            let source_points = source.points_data();
            source_cxx.set_points_bulk(&source_points);

            let mut target_cxx = small_gicp_cxx::PointCloud::new();
            let target_points = target.points_data();
            target_cxx.set_points_bulk(&target_points);

            let target_tree_cxx = small_gicp_cxx::KdTree::build(&target_cxx, 1);

            CxxRegistration::icp(
                &source_cxx,
                &target_cxx,
                &target_tree_cxx,
                Some(init_transform),
                Some(cxx_settings),
            )
        }
        RegistrationType::PlaneIcp => {
            // Create CXX point clouds from our data
            let mut source_cxx = small_gicp_cxx::PointCloud::new();
            let source_points = source.points_data();
            source_cxx.set_points_bulk(&source_points);
            if source.has_normals() {
                let source_normals = source.normals_data();
                source_cxx.set_normals_bulk(&source_normals);
            }

            let mut target_cxx = small_gicp_cxx::PointCloud::new();
            let target_points = target.points_data();
            target_cxx.set_points_bulk(&target_points);
            if target.has_normals() {
                let target_normals = target.normals_data();
                target_cxx.set_normals_bulk(&target_normals);
            }

            let target_tree_cxx = small_gicp_cxx::KdTree::build(&target_cxx, 1);

            CxxRegistration::point_to_plane_icp(
                &source_cxx,
                &target_cxx,
                &target_tree_cxx,
                Some(init_transform),
                Some(cxx_settings),
            )
        }
        RegistrationType::Gicp => {
            return Err(SmallGicpError::InvalidParameter {
                param: "registration_type",
                value: "GICP not yet implemented - requires covariance handling".to_string(),
            });
        }
        RegistrationType::Vgicp => {
            return Err(SmallGicpError::InvalidParameter {
                param: "registration_type",
                value: "VGICP requires a target voxel map, use register_vgicp instead".to_string(),
            });
        }
    };

    Ok(cxx_result_to_rust(result))
}

/// Perform advanced registration with additional options.
pub fn register_advanced(
    target: &PointCloud,
    source: &PointCloud,
    registration_type: RegistrationType,
    initial_guess: Option<&Isometry3<f64>>,
    _robust_kernel: Option<&RobustKernel>,
    _dof_restriction: Option<&DofRestriction>,
    settings: &RegistrationSettings,
) -> Result<ExtendedRegistrationResult> {
    // Create modified settings with the specified registration type and initial guess
    let mut modified_settings = settings.clone();
    modified_settings.registration_type = registration_type;
    modified_settings.initial_guess = initial_guess.cloned();

    // For now, ignore robust_kernel and dof_restriction (would need CXX support)
    let result = register(target, source, &modified_settings)?;

    // Create extended result with placeholder information matrix
    let information_matrix = Matrix4::identity(); // Placeholder
    let num_correspondences = 0; // Would need to be computed from actual registration

    Ok(ExtendedRegistrationResult {
        result,
        information_matrix,
        num_correspondences,
    })
}

/// Perform registration using preprocessed point clouds.
pub fn register_preprocessed(
    target: &PointCloud,
    source: &PointCloud,
    target_tree: &KdTree,
    settings: &RegistrationSettings,
) -> Result<RegistrationResult> {
    // This is the same as register_with_kdtree for preprocessed clouds
    register_with_kdtree(target, source, target_tree, settings)
}

/// Perform VGICP registration using a Gaussian voxel map.
pub fn register_vgicp(
    target_voxelmap: &GaussianVoxelMap,
    source: &PointCloud,
    initial_guess: Option<&Isometry3<f64>>,
    settings: &RegistrationSettings,
) -> Result<RegistrationResult> {
    let cxx_settings = settings_to_cxx(settings);
    let init_transform = isometry_to_cxx_transform(initial_guess);

    // Create CXX point cloud from our data
    let mut source_cxx = small_gicp_cxx::PointCloud::new();
    let source_points = source.points_data();
    source_cxx.set_points_bulk(&source_points);
    if source.has_covariances() {
        let source_covs = source.covariances_data();
        source_cxx.set_covariances_bulk(&source_covs);
    }

    let result = CxxRegistration::vgicp(
        &source_cxx,
        &target_voxelmap.inner,
        Some(init_transform),
        Some(cxx_settings),
    );

    Ok(cxx_result_to_rust(result))
}

/// Estimate the transformation between two point clouds using correspondences.
pub fn estimate_transformation(
    source_points: &[(f64, f64, f64)],
    target_points: &[(f64, f64, f64)],
) -> Result<Matrix4<f64>> {
    if source_points.len() != target_points.len() {
        return Err(SmallGicpError::InvalidParameter {
            param: "points",
            value: "Source and target point arrays must have the same length".to_string(),
        });
    }

    if source_points.len() < 3 {
        return Err(SmallGicpError::InvalidParameter {
            param: "points",
            value: "At least 3 point correspondences are required".to_string(),
        });
    }

    // Simple centroid-based transformation estimation
    // In a complete implementation, this would use SVD or other robust methods
    let source_centroid = {
        let sum = source_points.iter().fold((0.0, 0.0, 0.0), |acc, p| {
            (acc.0 + p.0, acc.1 + p.1, acc.2 + p.2)
        });
        let n = source_points.len() as f64;
        (sum.0 / n, sum.1 / n, sum.2 / n)
    };

    let target_centroid = {
        let sum = target_points.iter().fold((0.0, 0.0, 0.0), |acc, p| {
            (acc.0 + p.0, acc.1 + p.1, acc.2 + p.2)
        });
        let n = target_points.len() as f64;
        (sum.0 / n, sum.1 / n, sum.2 / n)
    };

    // For simplicity, return translation-only transformation
    let translation = (
        target_centroid.0 - source_centroid.0,
        target_centroid.1 - source_centroid.1,
        target_centroid.2 - source_centroid.2,
    );

    let mut transform = Matrix4::identity();
    transform[(0, 3)] = translation.0;
    transform[(1, 3)] = translation.1;
    transform[(2, 3)] = translation.2;

    Ok(transform)
}

/// Compute registration error for a given transformation.
pub fn compute_error(
    source: &PointCloud,
    target: &PointCloud,
    transformation: &Matrix4<f64>,
    max_correspondence_distance: f64,
) -> Result<f64> {
    let target_tree = KdTree::new(target)?;
    let correspondences =
        find_correspondences(source, target, &target_tree, max_correspondence_distance)?;

    if correspondences.is_empty() {
        return Ok(f64::INFINITY);
    }

    let mut total_error = 0.0;
    for (source_idx, target_idx, _distance) in &correspondences {
        let source_point_tuple = source.get_point(*source_idx)?;
        let target_point_tuple = target.get_point(*target_idx)?;

        let source_point = Point3::new(
            source_point_tuple.0,
            source_point_tuple.1,
            source_point_tuple.2,
        );
        let target_point = Point3::new(
            target_point_tuple.0,
            target_point_tuple.1,
            target_point_tuple.2,
        );

        // Transform source point
        let source_homogeneous = source_point.to_homogeneous();
        let transformed_homogeneous = transformation * source_homogeneous;
        let transformed_source = Point3::new(
            transformed_homogeneous[0] / transformed_homogeneous[3],
            transformed_homogeneous[1] / transformed_homogeneous[3],
            transformed_homogeneous[2] / transformed_homogeneous[3],
        );

        // Compute squared distance
        let diff = transformed_source - target_point;
        total_error += diff.norm_squared();
    }

    Ok(total_error / correspondences.len() as f64)
}

/// Find correspondences between two point clouds.
pub fn find_correspondences(
    source: &PointCloud,
    target: &PointCloud,
    target_tree: &KdTree,
    max_distance: f64,
) -> Result<Vec<(usize, usize, f64)>> {
    let mut correspondences = Vec::new();

    for i in 0..source.size() {
        let source_point_tuple = source.get_point(i)?;
        let source_point = Point3::new(
            source_point_tuple.0,
            source_point_tuple.1,
            source_point_tuple.2,
        );
        if let Some((target_idx, distance)) = target_tree.nearest_neighbor(&source_point) {
            if distance <= max_distance * max_distance {
                // distance is squared
                correspondences.push((i, target_idx, distance.sqrt()));
            }
        }
    }

    Ok(correspondences)
}

// Helper functions for type conversions

/// Convert nalgebra Matrix4 to CXX Transform
fn matrix_to_cxx_transform(matrix: &Matrix4<f64>) -> CxxTransform {
    let mut transform_matrix = [0.0; 16];
    for i in 0..4 {
        for j in 0..4 {
            transform_matrix[i * 4 + j] = matrix[(i, j)];
        }
    }
    CxxTransform {
        matrix: transform_matrix,
    }
}

/// Convert CXX Transform to nalgebra Matrix4
fn cxx_transform_to_matrix(transform: &CxxTransform) -> Matrix4<f64> {
    let mut matrix = Matrix4::zeros();
    for i in 0..4 {
        for j in 0..4 {
            matrix[(i, j)] = transform.matrix[i * 4 + j];
        }
    }
    matrix
}

/// Convert Rust RegistrationSettings to CXX RegistrationSettings
fn settings_to_cxx(settings: &RegistrationSettings) -> CxxRegistrationSettings {
    CxxRegistrationSettings {
        max_iterations: settings.max_iterations as i32,
        rotation_epsilon: settings.rotation_epsilon,
        transformation_epsilon: settings.transformation_epsilon,
        max_correspondence_distance: settings.max_correspondence_distance,
        num_threads: settings.num_threads as i32,
    }
}

/// Convert CXX RegistrationResult to Rust RegistrationResult
fn cxx_result_to_rust(result: CxxRegistrationResult) -> RegistrationResult {
    RegistrationResult {
        transformation: cxx_transform_to_matrix(&result.transformation),
        converged: result.converged,
        iterations: result.iterations as u32,
        error: result.error,
        num_inliers: 0, // CXX result doesn't include this, set to 0
    }
}

/// Convert optional Isometry3 to CXX Transform (identity if None)
fn isometry_to_cxx_transform(iso: Option<&Isometry3<f64>>) -> CxxTransform {
    match iso {
        Some(iso) => matrix_to_cxx_transform(&iso.to_homogeneous()),
        None => CxxTransform::default(), // Identity transform
    }
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
