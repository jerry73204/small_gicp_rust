//! Point cloud preprocessing utilities.

use crate::error::{check_error, Result, SmallGicpError};
use crate::kdtree::KdTree;
use crate::point_cloud::PointCloud;
use std::ptr;

/// Downsampling methods for point clouds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DownsamplingMethod {
    /// Voxel grid sampling - keeps one point per voxel
    VoxelGrid { leaf_size: f64 },
    /// Random sampling - randomly selects specified number of points
    Random { num_samples: usize },
}

/// Settings for point cloud preprocessing.
#[derive(Debug, Clone)]
pub struct PreprocessingSettings {
    /// Downsampling method to apply
    pub downsampling: Option<DownsamplingMethod>,
    /// Number of neighbors for normal estimation (0 to skip)
    pub num_neighbors_normals: i32,
    /// Whether to estimate covariances for GICP
    pub estimate_covariances: bool,
    /// Number of threads to use (1 for single-threaded)
    pub num_threads: usize,
}

impl Default for PreprocessingSettings {
    fn default() -> Self {
        PreprocessingSettings {
            downsampling: Some(DownsamplingMethod::VoxelGrid { leaf_size: 0.05 }),
            num_neighbors_normals: 20,
            estimate_covariances: true,
            num_threads: 1,
        }
    }
}

/// Result of preprocessing operations.
#[derive(Debug)]
pub struct PreprocessingResult {
    /// The preprocessed point cloud
    pub cloud: PointCloud,
    /// KdTree built from the preprocessed cloud (if requested)
    pub kdtree: Option<KdTree>,
}

/// Perform complete preprocessing of a point cloud.
///
/// This function applies downsampling, normal estimation, and covariance computation
/// based on the provided settings.
///
/// # Arguments
/// * `cloud` - The input point cloud
/// * `settings` - Preprocessing settings
/// * `build_kdtree` - Whether to build a KdTree for the result
///
/// # Returns
/// Preprocessing result with the processed cloud and optional KdTree
pub fn preprocess_point_cloud(
    cloud: &PointCloud,
    settings: &PreprocessingSettings,
    build_kdtree: bool,
) -> Result<PreprocessingResult> {
    if cloud.is_empty() {
        return Err(SmallGicpError::EmptyPointCloud);
    }

    // Step 1: Apply downsampling if requested
    let mut processed_cloud = if let Some(downsampling) = &settings.downsampling {
        match downsampling {
            DownsamplingMethod::VoxelGrid { leaf_size } => {
                voxelgrid_sampling(cloud, *leaf_size, settings.num_threads)?
            }
            DownsamplingMethod::Random { num_samples } => random_sampling(cloud, *num_samples)?,
        }
    } else {
        cloud.clone()
    };

    // Step 2: Build KdTree if needed for normal estimation or if requested
    let kdtree = if settings.num_neighbors_normals > 0 || build_kdtree {
        Some(KdTree::new(&processed_cloud, settings.num_threads)?)
    } else {
        None
    };

    // Step 3: Estimate normals and/or covariances if requested
    if settings.num_neighbors_normals > 0 {
        if let Some(ref tree) = kdtree {
            if settings.estimate_covariances {
                estimate_normals_and_covariances(
                    &mut processed_cloud,
                    tree,
                    settings.num_neighbors_normals,
                    settings.num_threads,
                )?;
            } else {
                estimate_normals(
                    &mut processed_cloud,
                    tree,
                    settings.num_neighbors_normals,
                    settings.num_threads,
                )?;
            }
        }
    }

    Ok(PreprocessingResult {
        cloud: processed_cloud,
        kdtree: if build_kdtree { kdtree } else { None },
    })
}

/// Convenience function for complete preprocessing with automatic KdTree creation.
///
/// This is equivalent to calling `preprocess_point_cloud` with `build_kdtree = true`.
///
/// # Arguments
/// * `cloud` - The input point cloud
/// * `downsampling_resolution` - Voxel size for downsampling
/// * `num_neighbors` - Number of neighbors for normal estimation
/// * `num_threads` - Number of threads to use
///
/// # Returns
/// Preprocessing result with both processed cloud and KdTree
pub fn preprocess_points(
    cloud: &PointCloud,
    downsampling_resolution: f64,
    num_neighbors: i32,
    num_threads: usize,
) -> Result<PreprocessingResult> {
    if cloud.is_empty() {
        return Err(SmallGicpError::EmptyPointCloud);
    }

    let mut preprocessed_cloud_handle = ptr::null_mut();
    let mut kdtree_handle = ptr::null_mut();

    let error = unsafe {
        small_gicp_sys::small_gicp_preprocess_points(
            cloud.handle,
            downsampling_resolution,
            num_neighbors,
            num_threads as i32,
            &mut preprocessed_cloud_handle,
            &mut kdtree_handle,
        )
    };

    check_error(error)?;

    let preprocessed_cloud = PointCloud {
        handle: preprocessed_cloud_handle,
    };

    let kdtree = if !kdtree_handle.is_null() {
        Some(KdTree {
            handle: kdtree_handle,
        })
    } else {
        None
    };

    Ok(PreprocessingResult {
        cloud: preprocessed_cloud,
        kdtree,
    })
}

/// Apply voxel grid downsampling to a point cloud.
///
/// # Arguments
/// * `cloud` - The input point cloud
/// * `leaf_size` - Size of each voxel (in meters)
/// * `num_threads` - Number of threads to use
///
/// # Returns
/// A new downsampled point cloud
pub fn voxelgrid_sampling(
    cloud: &PointCloud,
    leaf_size: f64,
    num_threads: usize,
) -> Result<PointCloud> {
    if cloud.is_empty() {
        return Err(SmallGicpError::EmptyPointCloud);
    }

    if leaf_size <= 0.0 {
        return Err(SmallGicpError::InvalidArgument(
            "Leaf size must be positive".to_string(),
        ));
    }

    let mut downsampled_handle = ptr::null_mut();
    let error = unsafe {
        small_gicp_sys::small_gicp_voxelgrid_sampling(
            cloud.handle,
            leaf_size,
            num_threads as i32,
            &mut downsampled_handle,
        )
    };

    check_error(error)?;

    Ok(PointCloud {
        handle: downsampled_handle,
    })
}

/// Apply random downsampling to a point cloud.
///
/// # Arguments
/// * `cloud` - The input point cloud
/// * `num_samples` - Number of points to sample
///
/// # Returns
/// A new randomly sampled point cloud
pub fn random_sampling(cloud: &PointCloud, num_samples: usize) -> Result<PointCloud> {
    if cloud.is_empty() {
        return Err(SmallGicpError::EmptyPointCloud);
    }

    if num_samples == 0 {
        return Err(SmallGicpError::InvalidArgument(
            "Number of samples must be positive".to_string(),
        ));
    }

    if num_samples >= cloud.len() {
        // Return a clone if we're sampling more points than available
        return Ok(cloud.clone());
    }

    let mut downsampled_handle = ptr::null_mut();
    let error = unsafe {
        small_gicp_sys::small_gicp_random_sampling(
            cloud.handle,
            num_samples,
            &mut downsampled_handle,
        )
    };

    check_error(error)?;

    Ok(PointCloud {
        handle: downsampled_handle,
    })
}

/// Estimate normals for a point cloud using a KdTree.
///
/// # Arguments
/// * `cloud` - The point cloud to estimate normals for (modified in-place)
/// * `kdtree` - KdTree for neighborhood search
/// * `num_neighbors` - Number of neighbors to use for estimation
/// * `num_threads` - Number of threads to use
pub fn estimate_normals(
    cloud: &mut PointCloud,
    kdtree: &KdTree,
    num_neighbors: i32,
    num_threads: usize,
) -> Result<()> {
    if cloud.is_empty() {
        return Err(SmallGicpError::EmptyPointCloud);
    }

    if num_neighbors <= 0 {
        return Err(SmallGicpError::InvalidArgument(
            "Number of neighbors must be positive".to_string(),
        ));
    }

    let error = unsafe {
        small_gicp_sys::small_gicp_estimate_normals(
            cloud.handle,
            kdtree.handle,
            num_neighbors,
            num_threads as i32,
        )
    };

    check_error(error)
}

/// Estimate covariances for a point cloud using a KdTree.
///
/// This is required for GICP registration.
///
/// # Arguments
/// * `cloud` - The point cloud to estimate covariances for (modified in-place)
/// * `kdtree` - KdTree for neighborhood search
/// * `num_neighbors` - Number of neighbors to use for estimation
/// * `num_threads` - Number of threads to use
pub fn estimate_covariances(
    cloud: &mut PointCloud,
    kdtree: &KdTree,
    num_neighbors: i32,
    num_threads: usize,
) -> Result<()> {
    if cloud.is_empty() {
        return Err(SmallGicpError::EmptyPointCloud);
    }

    if num_neighbors <= 0 {
        return Err(SmallGicpError::InvalidArgument(
            "Number of neighbors must be positive".to_string(),
        ));
    }

    let error = unsafe {
        small_gicp_sys::small_gicp_estimate_covariances(
            cloud.handle,
            kdtree.handle,
            num_neighbors,
            num_threads as i32,
        )
    };

    check_error(error)
}

/// Estimate both normals and covariances for a point cloud.
///
/// This is a more efficient combined operation than calling both functions separately.
///
/// # Arguments
/// * `cloud` - The point cloud to process (modified in-place)
/// * `kdtree` - KdTree for neighborhood search
/// * `num_neighbors` - Number of neighbors to use for estimation
/// * `num_threads` - Number of threads to use
pub fn estimate_normals_and_covariances(
    cloud: &mut PointCloud,
    kdtree: &KdTree,
    num_neighbors: i32,
    num_threads: usize,
) -> Result<()> {
    if cloud.is_empty() {
        return Err(SmallGicpError::EmptyPointCloud);
    }

    if num_neighbors <= 0 {
        return Err(SmallGicpError::InvalidArgument(
            "Number of neighbors must be positive".to_string(),
        ));
    }

    let error = unsafe {
        small_gicp_sys::small_gicp_estimate_normals_covariances(
            cloud.handle,
            kdtree.handle,
            num_neighbors,
            num_threads as i32,
        )
    };

    check_error(error)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    fn create_test_cloud() -> PointCloud {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.0, 0.0, 1.0),
            Point3::new(0.0, 1.0, 1.0),
            Point3::new(1.0, 1.0, 1.0),
        ];
        PointCloud::from_points(&points).unwrap()
    }

    #[test]
    fn test_voxelgrid_sampling() {
        let cloud = create_test_cloud();
        let downsampled = voxelgrid_sampling(&cloud, 0.5, 1).unwrap();

        // Should have fewer points after downsampling
        assert!(downsampled.len() <= cloud.len());
        assert!(!downsampled.is_empty());
    }

    #[test]
    fn test_random_sampling() {
        let cloud = create_test_cloud();
        let num_samples = 4;
        let downsampled = random_sampling(&cloud, num_samples).unwrap();

        assert_eq!(downsampled.len(), num_samples);
    }

    #[test]
    fn test_preprocess_points() {
        let cloud = create_test_cloud();
        let result = preprocess_points(&cloud, 0.5, 10, 1).unwrap();

        assert!(!result.cloud.is_empty());
        assert!(result.kdtree.is_some());
    }

    #[test]
    fn test_preprocessing_settings() {
        let cloud = create_test_cloud();
        let settings = PreprocessingSettings::default();
        let result = preprocess_point_cloud(&cloud, &settings, true).unwrap();

        assert!(!result.cloud.is_empty());
        assert!(result.kdtree.is_some());
    }

    #[test]
    fn test_normal_estimation() {
        let cloud = create_test_cloud();
        let mut preprocessed_cloud = cloud.clone();
        let kdtree = KdTree::new(&preprocessed_cloud, 1).unwrap();

        estimate_normals(&mut preprocessed_cloud, &kdtree, 5, 1).unwrap();

        // Verify that normals were estimated (they should not be zero)
        let normal = preprocessed_cloud.get_normal(0).unwrap();
        assert!(normal.magnitude() > 0.0);
    }
}
