//! Simple registration API that directly wraps C++ registration_helper.hpp functions.

use crate::{
    error::Result, kdtree::KdTree, point_cloud::PointCloud, voxelmap::IncrementalVoxelMap,
};
use nalgebra::Isometry3;

/// Registration type enum matching C++ RegistrationSetting::RegistrationType
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegistrationType {
    /// Standard ICP (Iterative Closest Point)
    ICP,
    /// Point-to-plane ICP
    PlaneICP,
    /// Generalized ICP (distribution-to-distribution)
    GICP,
    /// Voxelized GICP
    VGICP,
}

/// Registration settings matching C++ RegistrationSetting
#[derive(Debug, Clone)]
pub struct RegistrationSetting {
    /// Registration type
    pub reg_type: RegistrationType,
    /// Voxel resolution for VGICP
    pub voxel_resolution: f64,
    /// Downsample resolution (used in Eigen-based interface)
    pub downsampling_resolution: f64,
    /// Maximum correspondence distance
    pub max_correspondence_distance: f64,
    /// Rotation tolerance for convergence check [rad]
    pub rotation_eps: f64,
    /// Translation tolerance for convergence check
    pub translation_eps: f64,
    /// Number of threads
    pub num_threads: i32,
    /// Maximum number of iterations
    pub max_iterations: i32,
    /// Verbose mode
    pub verbose: bool,
}

impl Default for RegistrationSetting {
    fn default() -> Self {
        Self {
            reg_type: RegistrationType::GICP,
            voxel_resolution: 1.0,
            downsampling_resolution: 0.25,
            max_correspondence_distance: 1.0,
            rotation_eps: 0.1 * std::f64::consts::PI / 180.0, // 0.1 degrees
            translation_eps: 1e-3,
            num_threads: 4,
            max_iterations: 20,
            verbose: false,
        }
    }
}

/// Registration result matching C++ RegistrationResult
#[derive(Debug, Clone)]
pub struct RegistrationResult {
    /// Whether the registration converged
    pub converged: bool,
    /// Number of iterations performed
    pub iterations: i32,
    /// Final transformation T_target_source
    pub t_target_source: Isometry3<f64>,
    /// Final error
    pub error: f64,
}

/// Align preprocessed point clouds
///
/// This function matches the C++ signature:
/// ```cpp
/// RegistrationResult align(
///   const PointCloud& target,
///   const PointCloud& source,
///   const KdTree<PointCloud>& target_tree,
///   const Eigen::Isometry3d& init_T = Eigen::Isometry3d::Identity(),
///   const RegistrationSetting& setting = RegistrationSetting());
/// ```
pub fn align(
    target: &PointCloud,
    source: &PointCloud,
    target_tree: &KdTree,
    init_t: Option<Isometry3<f64>>,
    setting: Option<RegistrationSetting>,
) -> Result<RegistrationResult> {
    let setting = setting.unwrap_or_default();
    let init_t = init_t.unwrap_or_else(Isometry3::identity);

    // Convert to CXX types
    let source_cxx = source.inner();
    let target_cxx = target.inner();
    let target_tree_cxx = target_tree.inner();

    // Convert settings to FFI
    let ffi_settings = small_gicp_sys::RegistrationSettings {
        max_iterations: setting.max_iterations,
        rotation_epsilon: setting.rotation_eps,
        transformation_epsilon: setting.translation_eps,
        max_correspondence_distance: setting.max_correspondence_distance,
        num_threads: setting.num_threads,
    };

    // Convert initial transformation
    let init_transform = {
        let matrix = init_t.to_homogeneous();
        let mut transform_array = [0.0; 16];
        for i in 0..4 {
            for j in 0..4 {
                transform_array[i * 4 + j] = matrix[(i, j)];
            }
        }
        small_gicp_sys::Transform {
            matrix: transform_array,
        }
    };

    // Call appropriate registration function based on type
    let ffi_result = match setting.reg_type {
        RegistrationType::ICP => small_gicp_sys::Registration::icp(
            source_cxx,
            target_cxx,
            target_tree_cxx,
            Some(init_transform),
            Some(ffi_settings),
        ),
        RegistrationType::PlaneICP => {
            // Ensure target has normals
            if !target.has_normals() {
                return Err(crate::error::SmallGicpError::InvalidArgument(
                    "Target point cloud must have normals for Point-to-Plane ICP".to_string(),
                ));
            }

            small_gicp_sys::Registration::point_to_plane_icp(
                source_cxx,
                target_cxx,
                target_tree_cxx,
                Some(init_transform),
                Some(ffi_settings),
            )
        }
        RegistrationType::GICP => {
            // Ensure both clouds have covariances
            if !source.has_covariances() {
                return Err(crate::error::SmallGicpError::InvalidArgument(
                    "Source point cloud must have covariances for GICP".to_string(),
                ));
            }
            if !target.has_covariances() {
                return Err(crate::error::SmallGicpError::InvalidArgument(
                    "Target point cloud must have covariances for GICP".to_string(),
                ));
            }

            // Need to build source tree for GICP
            let source_tree = small_gicp_sys::KdTree::build(source_cxx, setting.num_threads);

            small_gicp_sys::Registration::gicp(
                source_cxx,
                target_cxx,
                &source_tree,
                target_tree_cxx,
                Some(init_transform),
                Some(ffi_settings),
            )
        }
        RegistrationType::VGICP => {
            return Err(crate::error::SmallGicpError::InvalidArgument(
                "VGICP requires voxel map. Use align_voxelmap() instead.".to_string(),
            ));
        }
    };

    // Convert result
    Ok(convert_registration_result(ffi_result))
}

/// Align preprocessed point clouds with VGICP
///
/// This function matches the C++ signature:
/// ```cpp
/// RegistrationResult align(
///   const GaussianVoxelMap& target,
///   const PointCloud& source,
///   const Eigen::Isometry3d& init_T = Eigen::Isometry3d::Identity(),
///   const RegistrationSetting& setting = RegistrationSetting());
/// ```
pub fn align_voxelmap(
    _target: &IncrementalVoxelMap,
    source: &PointCloud,
    init_t: Option<Isometry3<f64>>,
    setting: Option<RegistrationSetting>,
) -> Result<RegistrationResult> {
    let setting = setting.unwrap_or_default();
    let init_t = init_t.unwrap_or_else(Isometry3::identity);

    // For VGICP, we need to create a voxel map from small-gicp-sys
    // Since IncrementalVoxelMap uses small-gicp-sys internally, we need to
    // work through the CXX interface
    let _source_cxx = source.inner();

    // Convert settings to FFI
    let _ffi_settings = small_gicp_sys::RegistrationSettings {
        max_iterations: setting.max_iterations,
        rotation_epsilon: setting.rotation_eps,
        transformation_epsilon: setting.translation_eps,
        max_correspondence_distance: setting.max_correspondence_distance,
        num_threads: setting.num_threads,
    };

    // Convert initial transformation
    let _init_transform = {
        let matrix = init_t.to_homogeneous();
        let mut transform_array = [0.0; 16];
        for i in 0..4 {
            for j in 0..4 {
                transform_array[i * 4 + j] = matrix[(i, j)];
            }
        }
        small_gicp_sys::Transform {
            matrix: transform_array,
        }
    };

    // Create a CXX voxel map and insert the target data
    let _target_voxelmap = small_gicp_sys::VoxelMap::new(setting.voxel_resolution);
    // We need to convert IncrementalVoxelMap to a point cloud first
    // For now, return an error as we need to implement this conversion
    return Err(crate::error::SmallGicpError::NotImplemented(
        "Direct voxelmap alignment not yet implemented. Please use align() with RegistrationType::VGICP and ensure target is converted to PointCloud.".to_string(),
    ));

    // Once implemented:
    // let ffi_result = small_gicp_sys::Registration::vgicp(
    //     source_cxx,
    //     &target_voxelmap,
    //     Some(init_transform),
    //     Some(ffi_settings),
    // );
    // Ok(convert_registration_result(ffi_result))
}

/// Preprocess point cloud (downsampling, kdtree creation, and normal and covariance estimation).
///
/// This function matches the C++ signature:
/// ```cpp
/// std::pair<PointCloud::Ptr, std::shared_ptr<KdTree<PointCloud>>>
/// preprocess_points(const PointCloud& points, double downsampling_resolution,
///                   int num_neighbors = 10, int num_threads = 4);
/// ```
pub fn preprocess_points(
    points: &PointCloud,
    downsampling_resolution: f64,
    num_neighbors: usize,
    num_threads: i32,
) -> Result<(PointCloud, KdTree)> {
    use crate::preprocessing::Preprocessing;

    // Downsample
    let downsampled = if downsampling_resolution > 0.0 {
        Preprocessing::voxel_downsample(points, downsampling_resolution, num_threads as usize)
    } else {
        points.clone()
    };

    // Create KdTree
    let kdtree = KdTree::new(&downsampled)?;

    // Estimate normals and covariances
    let mut processed = downsampled;
    Preprocessing::estimate_normals(&mut processed, num_neighbors, num_threads as usize)?;
    Preprocessing::estimate_covariances(&mut processed, num_neighbors, num_threads as usize)?;

    Ok((processed, kdtree))
}

/// Create an incremental Gaussian voxel map.
///
/// This function matches the C++ signature:
/// ```cpp
/// GaussianVoxelMap::Ptr create_gaussian_voxelmap(const PointCloud& points, double voxel_resolution);
/// ```
pub fn create_gaussian_voxelmap(
    points: &PointCloud,
    voxel_resolution: f64,
) -> Result<IncrementalVoxelMap> {
    let mut voxelmap = IncrementalVoxelMap::new(voxel_resolution);
    voxelmap.insert(points)?;
    voxelmap.finalize();
    Ok(voxelmap)
}

// Helper function to convert FFI result to Rust result
fn convert_registration_result(ffi: small_gicp_sys::RegistrationResult) -> RegistrationResult {
    // Convert transformation matrix
    let mut matrix = nalgebra::Matrix4::zeros();
    for i in 0..4 {
        for j in 0..4 {
            matrix[(i, j)] = ffi.transformation.matrix[i * 4 + j];
        }
    }

    // Extract rotation and translation
    let rotation =
        nalgebra::Rotation3::from_matrix_unchecked(matrix.fixed_view::<3, 3>(0, 0).into());
    let translation = nalgebra::Translation3::new(matrix[(0, 3)], matrix[(1, 3)], matrix[(2, 3)]);
    let transformation = Isometry3::from_parts(
        translation,
        nalgebra::UnitQuaternion::from_rotation_matrix(&rotation),
    );

    RegistrationResult {
        converged: ffi.converged,
        iterations: ffi.iterations,
        t_target_source: transformation,
        error: ffi.error,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_registration_setting_default() {
        let setting = RegistrationSetting::default();
        assert_eq!(setting.reg_type, RegistrationType::GICP);
        assert_eq!(setting.voxel_resolution, 1.0);
        assert_eq!(setting.max_iterations, 20);
    }

    #[test]
    fn test_registration_types() {
        assert_eq!(RegistrationType::ICP, RegistrationType::ICP);
        assert_ne!(RegistrationType::ICP, RegistrationType::GICP);
    }

    #[test]
    fn test_align_voxelmap_not_implemented() {
        // TODO: Implement align_voxelmap when IncrementalVoxelMap to VoxelMap conversion is ready
        let voxelmap = crate::voxelmap::IncrementalVoxelMap::new(0.1);
        let mut source = crate::point_cloud::PointCloud::new().unwrap();
        source.add_point(0.0, 0.0, 0.0);

        // This should return NotImplemented error
        let result = align_voxelmap(&voxelmap, &source, None, None);
        assert!(result.is_err());
        match result {
            Err(crate::error::SmallGicpError::NotImplemented(msg)) => {
                assert!(msg.contains("Direct voxelmap alignment not yet implemented"));
            }
            _ => panic!("Expected NotImplemented error, got: {:?}", result),
        }
    }

    #[test]
    #[ignore = "VGICP registration not yet fully implemented"]
    fn test_vgicp_registration() {
        // TODO: Complete VGICP registration test when align_voxelmap is implemented
        // This test requires:
        // 1. IncrementalVoxelMap to VoxelMap conversion
        // 2. Full VGICP implementation in align_voxelmap
        todo!("VGICP registration test - waiting for align_voxelmap implementation");
    }
}
