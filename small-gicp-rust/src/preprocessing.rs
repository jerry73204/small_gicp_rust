//! Point cloud preprocessing utilities.

use crate::{
    error::{check_error, Result, SmallGicpError},
    kdtree::KdTree,
    point_cloud::PointCloud,
};
use std::ptr;

/// Get the default number of threads based on available parallelism.
/// Falls back to 4 if unable to determine available parallelism.
fn default_num_threads() -> usize {
    std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(4)
}

/// Downsampling methods for point clouds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DownsamplingMethod {
    /// Voxel grid sampling - keeps one point per voxel
    VoxelGrid { leaf_size: f64 },
    /// Random sampling - randomly selects specified number of points
    Random { num_samples: usize },
}

/// Result of preprocessing operations.
#[derive(Debug)]
pub struct PreprocessingResult {
    /// The preprocessed point cloud
    pub cloud: PointCloud,
    /// KdTree built from the preprocessed cloud (if requested)
    pub kdtree: KdTree,
}

// Legacy PreprocessorConfig for backward compatibility
#[derive(Debug, Clone)]
pub struct PreprocessorConfig {
    pub downsampling_resolution: f64,
    pub num_neighbors: usize,
    pub num_threads: usize,
}

impl Default for PreprocessorConfig {
    fn default() -> Self {
        PreprocessorConfig {
            downsampling_resolution: 0.25,
            num_neighbors: 10,
            num_threads: default_num_threads(),
        }
    }
}

impl PointCloud {
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
    pub fn preprocess_points(&self, config: &PreprocessorConfig) -> Result<PreprocessingResult> {
        let PreprocessorConfig {
            downsampling_resolution,
            num_neighbors,
            num_threads,
        } = *config;

        if self.is_empty() {
            return Err(SmallGicpError::EmptyPointCloud);
        }

        let mut preprocessed_cloud_handle = ptr::null_mut();
        let mut kdtree_handle = ptr::null_mut();

        let error = unsafe {
            small_gicp_sys::small_gicp_preprocess_points(
                self.handle,
                downsampling_resolution,
                num_neighbors as i32,
                num_threads as i32,
                &mut preprocessed_cloud_handle,
                &mut kdtree_handle,
            )
        };

        check_error(error)?;

        let preprocessed_cloud = PointCloud {
            handle: preprocessed_cloud_handle,
        };

        assert!(!kdtree_handle.is_null());
        let kdtree = KdTree {
            handle: kdtree_handle,
        };

        Ok(PreprocessingResult {
            cloud: preprocessed_cloud,
            kdtree,
        })
    }
}

// Re-export from config module for backward compatibility
pub use crate::config::{
    CovarianceEstimationConfig, NormalEstimationConfig, VoxelGridConfig as VoxelGridSamplingConfig,
    VoxelGridConfig,
};

impl PointCloud {
    /// Apply voxel grid downsampling to a point cloud.
    ///
    /// # Arguments
    /// * `config` - Voxel grid sampling configuration
    ///
    /// # Returns
    /// A new downsampled point cloud
    pub fn voxelgrid_sampling(&self, config: &VoxelGridConfig) -> Result<PointCloud> {
        let VoxelGridConfig {
            leaf_size,
            num_threads,
        } = *config;

        if self.is_empty() {
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
                self.handle,
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
}

impl PointCloud {
    /// Apply random downsampling to a point cloud.
    ///
    /// # Arguments
    /// * `cloud` - The input point cloud
    /// * `num_samples` - Number of points to sample
    ///
    /// # Returns
    /// A new randomly sampled point cloud
    pub fn random_sampling(&self, num_samples: usize) -> Result<PointCloud> {
        if self.is_empty() {
            return Err(SmallGicpError::EmptyPointCloud);
        }

        if num_samples == 0 {
            return Err(SmallGicpError::InvalidArgument(
                "Number of samples must be positive".to_string(),
            ));
        }

        if num_samples >= self.len() {
            // Return a clone if we're sampling more points than available
            return Ok(self.clone());
        }

        let mut downsampled_handle = ptr::null_mut();
        let error = unsafe {
            small_gicp_sys::small_gicp_random_sampling(
                self.handle,
                num_samples,
                &mut downsampled_handle,
            )
        };

        check_error(error)?;

        Ok(PointCloud {
            handle: downsampled_handle,
        })
    }
}

/// Estimate normals for a point cloud using a KdTree.
///
/// # Arguments
/// * `cloud` - The point cloud to estimate normals for (modified in-place)
/// * `kdtree` - KdTree for neighborhood search
/// * `config` - Configuration for normal estimation
pub fn estimate_normals(
    cloud: &mut PointCloud,
    kdtree: &KdTree,
    config: &NormalEstimationConfig,
) -> Result<()> {
    let NormalEstimationConfig {
        num_neighbors,
        num_threads,
    } = *config;
    if cloud.is_empty() {
        return Err(SmallGicpError::EmptyPointCloud);
    }

    if num_neighbors == 0 {
        return Err(SmallGicpError::InvalidArgument(
            "Number of neighbors must be positive".to_string(),
        ));
    }

    let error = unsafe {
        small_gicp_sys::small_gicp_estimate_normals(
            cloud.handle,
            kdtree.handle,
            num_neighbors as i32,
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
/// * `config` - Configuration for covariance estimation
pub fn estimate_covariances(
    cloud: &mut PointCloud,
    kdtree: &KdTree,
    config: &CovarianceEstimationConfig,
) -> Result<()> {
    let CovarianceEstimationConfig {
        num_neighbors,
        num_threads,
    } = *config;
    if cloud.is_empty() {
        return Err(SmallGicpError::EmptyPointCloud);
    }

    if num_neighbors == 0 {
        return Err(SmallGicpError::InvalidArgument(
            "Number of neighbors must be positive".to_string(),
        ));
    }

    let error = unsafe {
        small_gicp_sys::small_gicp_estimate_covariances(
            cloud.handle,
            kdtree.handle,
            num_neighbors as i32,
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
/// * `config` - Configuration for normal and covariance estimation
pub fn estimate_normals_and_covariances(
    cloud: &mut PointCloud,
    kdtree: &KdTree,
    config: &NormalEstimationConfig,
) -> Result<()> {
    let NormalEstimationConfig {
        num_neighbors,
        num_threads,
    } = *config;
    if cloud.is_empty() {
        return Err(SmallGicpError::EmptyPointCloud);
    }

    if num_neighbors == 0 {
        return Err(SmallGicpError::InvalidArgument(
            "Number of neighbors must be positive".to_string(),
        ));
    }

    let error = unsafe {
        small_gicp_sys::small_gicp_estimate_normals_covariances(
            cloud.handle,
            kdtree.handle,
            num_neighbors as i32,
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
        let downsampled = cloud
            .voxelgrid_sampling(&VoxelGridConfig {
                leaf_size: 0.5,
                num_threads: 1,
            })
            .unwrap();

        // Should have fewer points after downsampling
        assert!(downsampled.len() <= cloud.len());
        assert!(!downsampled.is_empty());
    }

    #[test]
    fn test_random_sampling() {
        let cloud = create_test_cloud();
        let num_samples = 4;
        let downsampled = cloud.random_sampling(num_samples).unwrap();

        assert_eq!(downsampled.len(), num_samples);
    }

    #[test]
    fn test_preprocess_points() {
        let cloud = create_test_cloud();
        let result = cloud
            .preprocess_points(&PreprocessorConfig {
                downsampling_resolution: 0.5,
                num_neighbors: 10,
                num_threads: 1,
            })
            .unwrap();
        assert!(!result.cloud.is_empty());
    }

    #[test]
    fn test_normal_estimation() {
        let cloud = create_test_cloud();
        let mut preprocessed_cloud = cloud.clone();
        let kdtree_config = crate::config::KdTreeConfig::default();
        let kdtree = KdTree::new(&preprocessed_cloud, &kdtree_config).unwrap();

        estimate_normals(
            &mut preprocessed_cloud,
            &kdtree,
            &NormalEstimationConfig {
                num_neighbors: 5,
                num_threads: 1,
            },
        )
        .unwrap();

        // Verify that normals were estimated (they should not be zero)
        let normal = preprocessed_cloud.get_normal(0).unwrap();
        assert!(normal.magnitude() > 0.0);
    }
}
