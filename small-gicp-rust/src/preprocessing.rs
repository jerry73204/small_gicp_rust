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
    CovarianceEstimationConfig, DownsamplingBackend, LocalFeatureEstimationConfig,
    LocalFeatureSetterType, LocalFeaturesBackend, NormalEstimationBackend, NormalEstimationConfig,
    RandomSamplingConfig, VoxelGridConfig as VoxelGridSamplingConfig, VoxelGridConfig,
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
            backend,
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
            small_gicp_sys::small_gicp_voxelgrid_sampling_with_backend(
                self.handle,
                leaf_size,
                backend.into(),
                num_threads as i32,
                &mut downsampled_handle,
            )
        };

        check_error(error)?;

        Ok(PointCloud {
            handle: downsampled_handle,
        })
    }

    /// Apply voxel grid downsampling with default backend (legacy version).
    ///
    /// # Arguments
    /// * `leaf_size` - Voxel size for downsampling
    /// * `num_threads` - Number of threads to use
    ///
    /// # Returns
    /// A new downsampled point cloud
    pub fn voxelgrid_sampling_simple(
        &self,
        leaf_size: f64,
        num_threads: usize,
    ) -> Result<PointCloud> {
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

    /// Apply random downsampling with configuration support.
    ///
    /// # Arguments
    /// * `config` - Random sampling configuration
    ///
    /// # Returns
    /// A new randomly sampled point cloud
    pub fn random_sampling_with_config(&self, config: &RandomSamplingConfig) -> Result<PointCloud> {
        let RandomSamplingConfig { num_samples, seed } = *config;

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
        let error = match seed {
            Some(seed_val) => unsafe {
                small_gicp_sys::small_gicp_random_sampling_with_seed(
                    self.handle,
                    num_samples,
                    seed_val as u32,
                    &mut downsampled_handle,
                )
            },
            None => unsafe {
                small_gicp_sys::small_gicp_random_sampling(
                    self.handle,
                    num_samples,
                    &mut downsampled_handle,
                )
            },
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
        backend,
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
        small_gicp_sys::small_gicp_estimate_normals_with_backend(
            cloud.handle,
            kdtree.handle,
            num_neighbors as i32,
            backend.into(),
            num_threads as i32,
        )
    };

    check_error(error)
}

/// Estimate normals using automatic backend selection (legacy version).
///
/// # Arguments
/// * `cloud` - The point cloud to estimate normals for (modified in-place)
/// * `kdtree` - KdTree for neighborhood search
/// * `num_neighbors` - Number of neighbors for estimation
/// * `num_threads` - Number of threads to use
pub fn estimate_normals_simple(
    cloud: &mut PointCloud,
    kdtree: &KdTree,
    num_neighbors: i32,
    num_threads: usize,
) -> Result<()> {
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
/// * `config` - Configuration for covariance estimation
pub fn estimate_covariances(
    cloud: &mut PointCloud,
    kdtree: &KdTree,
    config: &CovarianceEstimationConfig,
) -> Result<()> {
    let CovarianceEstimationConfig {
        num_neighbors,
        backend,
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
        small_gicp_sys::small_gicp_estimate_covariances_with_backend(
            cloud.handle,
            kdtree.handle,
            num_neighbors as i32,
            backend.into(),
            num_threads as i32,
        )
    };

    check_error(error)
}

/// Estimate covariances using automatic backend selection (legacy version).
///
/// # Arguments
/// * `cloud` - The point cloud to estimate covariances for (modified in-place)
/// * `kdtree` - KdTree for neighborhood search
/// * `num_neighbors` - Number of neighbors for estimation
/// * `num_threads` - Number of threads to use
pub fn estimate_covariances_simple(
    cloud: &mut PointCloud,
    kdtree: &KdTree,
    num_neighbors: i32,
    num_threads: usize,
) -> Result<()> {
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
/// * `config` - Configuration for normal and covariance estimation
pub fn estimate_normals_and_covariances(
    cloud: &mut PointCloud,
    kdtree: &KdTree,
    config: &NormalEstimationConfig,
) -> Result<()> {
    let NormalEstimationConfig {
        num_neighbors,
        backend,
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
        small_gicp_sys::small_gicp_estimate_normals_covariances_with_backend(
            cloud.handle,
            kdtree.handle,
            num_neighbors as i32,
            backend.into(),
            num_threads as i32,
        )
    };

    check_error(error)
}

/// Estimate both normals and covariances using automatic backend selection (legacy version).
///
/// # Arguments
/// * `cloud` - The point cloud to process (modified in-place)
/// * `kdtree` - KdTree for neighborhood search
/// * `num_neighbors` - Number of neighbors for estimation
/// * `num_threads` - Number of threads to use
pub fn estimate_normals_and_covariances_simple(
    cloud: &mut PointCloud,
    kdtree: &KdTree,
    num_neighbors: i32,
    num_threads: usize,
) -> Result<()> {
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
            num_neighbors,
            num_threads as i32,
        )
    };

    check_error(error)
}

/// Estimate local features for a single point in a point cloud.
///
/// # Arguments
/// * `cloud` - The point cloud to process (modified in-place)
/// * `kdtree` - KdTree for neighborhood search
/// * `point_index` - Index of the point to estimate features for
/// * `num_neighbors` - Number of neighbors for estimation
/// * `setter_type` - Type of features to estimate
pub fn estimate_local_features_single_point(
    cloud: &mut PointCloud,
    kdtree: &KdTree,
    point_index: usize,
    num_neighbors: i32,
    setter_type: LocalFeatureSetterType,
) -> Result<()> {
    if cloud.is_empty() {
        return Err(SmallGicpError::EmptyPointCloud);
    }

    if point_index >= cloud.len() {
        return Err(SmallGicpError::InvalidArgument(
            "Point index out of bounds".to_string(),
        ));
    }

    if num_neighbors == 0 {
        return Err(SmallGicpError::InvalidArgument(
            "Number of neighbors must be positive".to_string(),
        ));
    }

    let error = unsafe {
        small_gicp_sys::small_gicp_estimate_local_features_single_point(
            cloud.handle,
            kdtree.handle,
            point_index,
            num_neighbors,
            setter_type.into(),
        )
    };

    check_error(error)
}

/// Estimate local features for an entire point cloud using advanced configuration.
///
/// # Arguments
/// * `cloud` - The point cloud to process (modified in-place)
/// * `kdtree` - KdTree for neighborhood search
/// * `config` - Configuration for local feature estimation
pub fn estimate_local_features_cloud(
    cloud: &mut PointCloud,
    kdtree: &KdTree,
    config: &LocalFeatureEstimationConfig,
) -> Result<()> {
    let LocalFeatureEstimationConfig {
        setter_type,
        backend,
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
        small_gicp_sys::small_gicp_estimate_local_features_cloud(
            cloud.handle,
            kdtree.handle,
            num_neighbors,
            setter_type.into(),
            backend.into(),
            num_threads as i32,
        )
    };

    check_error(error)
}

/// Estimate local features for an entire point cloud without external KdTree.
///
/// This function automatically builds a KdTree internally, which can be more
/// efficient for one-time processing but less efficient if you need the KdTree
/// for other operations.
///
/// # Arguments
/// * `cloud` - The point cloud to process (modified in-place)
/// * `config` - Configuration for local feature estimation
pub fn estimate_local_features_auto(
    cloud: &mut PointCloud,
    config: &LocalFeatureEstimationConfig,
) -> Result<()> {
    let LocalFeatureEstimationConfig {
        setter_type,
        backend,
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
        small_gicp_sys::small_gicp_estimate_local_features_auto(
            cloud.handle,
            num_neighbors,
            setter_type.into(),
            backend.into(),
            num_threads as i32,
        )
    };

    check_error(error)
}

/// Direct setter interface - set normal for a point given eigenvectors.
///
/// # Arguments
/// * `cloud` - The point cloud to modify
/// * `point_index` - Index of the point to set normal for
/// * `eigenvectors` - 3x3 matrix in row-major order
pub fn set_normal_direct(
    cloud: &mut PointCloud,
    point_index: usize,
    eigenvectors: &[f64; 9],
) -> Result<()> {
    if cloud.is_empty() {
        return Err(SmallGicpError::EmptyPointCloud);
    }

    if point_index >= cloud.len() {
        return Err(SmallGicpError::InvalidArgument(
            "Point index out of bounds".to_string(),
        ));
    }

    let error = unsafe {
        small_gicp_sys::small_gicp_normal_setter_set(
            cloud.handle,
            point_index,
            eigenvectors.as_ptr(),
        )
    };

    check_error(error)
}

/// Direct setter interface - set covariance for a point given eigenvectors.
///
/// # Arguments
/// * `cloud` - The point cloud to modify
/// * `point_index` - Index of the point to set covariance for
/// * `eigenvectors` - 3x3 matrix in row-major order
pub fn set_covariance_direct(
    cloud: &mut PointCloud,
    point_index: usize,
    eigenvectors: &[f64; 9],
) -> Result<()> {
    if cloud.is_empty() {
        return Err(SmallGicpError::EmptyPointCloud);
    }

    if point_index >= cloud.len() {
        return Err(SmallGicpError::InvalidArgument(
            "Point index out of bounds".to_string(),
        ));
    }

    let error = unsafe {
        small_gicp_sys::small_gicp_covariance_setter_set(
            cloud.handle,
            point_index,
            eigenvectors.as_ptr(),
        )
    };

    check_error(error)
}

/// Direct setter interface - set normal and covariance for a point given eigenvectors.
///
/// # Arguments
/// * `cloud` - The point cloud to modify
/// * `point_index` - Index of the point to set features for
/// * `eigenvectors` - 3x3 matrix in row-major order
pub fn set_normal_covariance_direct(
    cloud: &mut PointCloud,
    point_index: usize,
    eigenvectors: &[f64; 9],
) -> Result<()> {
    if cloud.is_empty() {
        return Err(SmallGicpError::EmptyPointCloud);
    }

    if point_index >= cloud.len() {
        return Err(SmallGicpError::InvalidArgument(
            "Point index out of bounds".to_string(),
        ));
    }

    let error = unsafe {
        small_gicp_sys::small_gicp_normal_covariance_setter_set(
            cloud.handle,
            point_index,
            eigenvectors.as_ptr(),
        )
    };

    check_error(error)
}

/// Set invalid normal for a point (marks as unreliable).
///
/// # Arguments
/// * `cloud` - The point cloud to modify
/// * `point_index` - Index of the point to mark as invalid
pub fn set_normal_invalid(cloud: &mut PointCloud, point_index: usize) -> Result<()> {
    if cloud.is_empty() {
        return Err(SmallGicpError::EmptyPointCloud);
    }

    if point_index >= cloud.len() {
        return Err(SmallGicpError::InvalidArgument(
            "Point index out of bounds".to_string(),
        ));
    }

    let error =
        unsafe { small_gicp_sys::small_gicp_normal_setter_set_invalid(cloud.handle, point_index) };

    check_error(error)
}

/// Set invalid covariance for a point (marks as unreliable).
///
/// # Arguments
/// * `cloud` - The point cloud to modify
/// * `point_index` - Index of the point to mark as invalid
pub fn set_covariance_invalid(cloud: &mut PointCloud, point_index: usize) -> Result<()> {
    if cloud.is_empty() {
        return Err(SmallGicpError::EmptyPointCloud);
    }

    if point_index >= cloud.len() {
        return Err(SmallGicpError::InvalidArgument(
            "Point index out of bounds".to_string(),
        ));
    }

    let error = unsafe {
        small_gicp_sys::small_gicp_covariance_setter_set_invalid(cloud.handle, point_index)
    };

    check_error(error)
}

/// Set invalid normal and covariance for a point (marks as unreliable).
///
/// # Arguments
/// * `cloud` - The point cloud to modify
/// * `point_index` - Index of the point to mark as invalid
pub fn set_normal_covariance_invalid(cloud: &mut PointCloud, point_index: usize) -> Result<()> {
    if cloud.is_empty() {
        return Err(SmallGicpError::EmptyPointCloud);
    }

    if point_index >= cloud.len() {
        return Err(SmallGicpError::InvalidArgument(
            "Point index out of bounds".to_string(),
        ));
    }

    let error = unsafe {
        small_gicp_sys::small_gicp_normal_covariance_setter_set_invalid(cloud.handle, point_index)
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
                backend: DownsamplingBackend::Default,
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
                backend: NormalEstimationBackend::Default,
                num_threads: 1,
            },
        )
        .unwrap();

        // Verify that normals were estimated (they should not be zero)
        let normal = preprocessed_cloud.get_normal(0).unwrap();
        assert!(normal.magnitude() > 0.0);
    }

    #[test]
    fn test_random_sampling_with_seed() {
        let cloud = create_test_cloud();
        let config = RandomSamplingConfig {
            num_samples: 4,
            seed: Some(42),
        };

        let downsampled1 = cloud.random_sampling_with_config(&config).unwrap();
        let downsampled2 = cloud.random_sampling_with_config(&config).unwrap();

        assert_eq!(downsampled1.len(), 4);
        assert_eq!(downsampled2.len(), 4);

        // With the same seed, results should be identical
        for i in 0..4 {
            let p1 = downsampled1.get_point(i).unwrap();
            let p2 = downsampled2.get_point(i).unwrap();
            assert!((p1 - p2).magnitude() < 1e-10);
        }
    }

    #[test]
    fn test_backend_aware_normal_estimation() {
        let cloud = create_test_cloud();
        let mut preprocessed_cloud = cloud.clone();
        let kdtree_config = crate::config::KdTreeConfig::default();
        let kdtree = KdTree::new(&preprocessed_cloud, &kdtree_config).unwrap();

        // Test with OpenMP backend
        estimate_normals(
            &mut preprocessed_cloud,
            &kdtree,
            &NormalEstimationConfig {
                num_neighbors: 5,
                backend: NormalEstimationBackend::OpenMp,
                num_threads: 2,
            },
        )
        .unwrap();

        let normal = preprocessed_cloud.get_normal(0).unwrap();
        assert!(normal.magnitude() > 0.0);
    }

    #[test]
    fn test_local_features_estimation() {
        let cloud = create_test_cloud();
        let mut preprocessed_cloud = cloud.clone();
        let kdtree_config = crate::config::KdTreeConfig::default();
        let kdtree = KdTree::new(&preprocessed_cloud, &kdtree_config).unwrap();

        let config = LocalFeatureEstimationConfig {
            setter_type: LocalFeatureSetterType::NormalCovariance,
            backend: LocalFeaturesBackend::Default,
            num_neighbors: 5,
            num_threads: 1,
        };

        estimate_local_features_cloud(&mut preprocessed_cloud, &kdtree, &config).unwrap();

        // Verify that both normals and covariances were estimated
        let normal = preprocessed_cloud.get_normal(0).unwrap();
        let covariance = preprocessed_cloud.get_covariance(0).unwrap();

        assert!(normal.magnitude() > 0.0);
        assert!(covariance.trace() > 0.0);
    }

    #[test]
    fn test_local_features_auto_estimation() {
        let cloud = create_test_cloud();
        let mut preprocessed_cloud = cloud.clone();

        let config = LocalFeatureEstimationConfig {
            setter_type: LocalFeatureSetterType::Normal,
            backend: LocalFeaturesBackend::Default,
            num_neighbors: 5,
            num_threads: 1,
        };

        estimate_local_features_auto(&mut preprocessed_cloud, &config).unwrap();

        let normal = preprocessed_cloud.get_normal(0).unwrap();
        assert!(normal.magnitude() > 0.0);
    }

    #[test]
    fn test_direct_setter_interface() {
        let cloud = create_test_cloud();
        let mut preprocessed_cloud = cloud.clone();

        // Create a simple eigenvector matrix (identity-like)
        let eigenvectors = [
            1.0, 0.0, 0.0, // First eigenvector
            0.0, 1.0, 0.0, // Second eigenvector
            0.0, 0.0, 1.0, // Third eigenvector (normal)
        ];

        set_normal_direct(&mut preprocessed_cloud, 0, &eigenvectors).unwrap();
        let normal = preprocessed_cloud.get_normal(0).unwrap();

        // Normal should have been set successfully (magnitude should be > 0)
        assert!(normal.magnitude() > 0.0);
    }

    #[test]
    fn test_invalid_setters() {
        let cloud = create_test_cloud();
        let mut preprocessed_cloud = cloud.clone();

        // Test setting invalid normal
        set_normal_invalid(&mut preprocessed_cloud, 0).unwrap();

        // Test setting invalid covariance
        set_covariance_invalid(&mut preprocessed_cloud, 0).unwrap();

        // Test setting invalid normal and covariance together
        set_normal_covariance_invalid(&mut preprocessed_cloud, 1).unwrap();
    }

    #[test]
    fn test_backend_aware_voxelgrid_sampling() {
        let cloud = create_test_cloud();

        // Test different backends
        let backends = [
            DownsamplingBackend::Default,
            DownsamplingBackend::OpenMp,
            DownsamplingBackend::Tbb,
        ];

        for backend in backends {
            let config = VoxelGridConfig {
                leaf_size: 0.5,
                backend,
                num_threads: 2,
            };

            let downsampled = cloud.voxelgrid_sampling(&config).unwrap();
            assert!(downsampled.len() <= cloud.len());
            assert!(!downsampled.is_empty());
        }
    }
}
