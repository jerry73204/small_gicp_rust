//! Point cloud preprocessing utilities.

use crate::{
    error::{Result, SmallGicpError},
    kdtree_internal::CKdTree,
    point_cloud::PointCloud,
};

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
pub struct PreprocessingResult {
    /// The preprocessed point cloud
    pub cloud: PointCloud,
    /// KdTree built from the preprocessed cloud (if requested)
    pub kdtree: CKdTree,
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

        todo!(
            "Implement preprocess_points using small-gicp-cxx. 
            This should:
            1. Perform voxel grid downsampling with resolution {}
            2. Estimate normals and covariances using {} neighbors
            3. Build a KdTree from the preprocessed cloud
            4. Use {} threads for parallel processing
            
            Consider using the PointCloudWrapper and KdTreeWrapper from small-gicp-cxx
            to implement this complete preprocessing pipeline.",
            downsampling_resolution,
            num_neighbors,
            num_threads
        );
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

        todo!(
            "Implement voxelgrid_sampling using small-gicp-cxx.
            This should:
            1. Use the appropriate voxelgrid downsampling function from small-gicp-cxx
            2. Apply leaf_size of {} for voxel grid resolution
            3. Use backend {:?} for parallel processing
            4. Use {} threads for parallel execution
            
            Consider using small_gicp::voxelgrid_sampling_* functions with the appropriate backend.",
            leaf_size, backend, num_threads
        );
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

        todo!(
            "Implement voxelgrid_sampling_simple using small-gicp-cxx.
            This should:
            1. Use the default voxelgrid downsampling function from small-gicp-cxx
            2. Apply leaf_size of {} for voxel grid resolution
            3. Use {} threads for parallel execution
            
            Consider using small_gicp::voxelgrid_sampling() function.",
            leaf_size,
            num_threads
        );
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
            todo!("Return cloned PointCloud")
        }

        todo!(
            "Implement random_sampling using small-gicp-cxx.
            This should:
            1. Use the random sampling function from small-gicp-cxx
            2. Sample {} points from the input cloud
            3. Return a new PointCloud with the randomly selected points
            
            Consider using small_gicp::random_sampling() function.",
            num_samples
        );
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
            todo!("Return cloned PointCloud")
        }

        todo!(
            "Implement random_sampling_with_config using small-gicp-cxx.
            This should:
            1. Sample {} points from the input cloud
            2. Use random seed {:?} if provided for reproducible results
            3. Return a new PointCloud with the randomly selected points
            
            Consider using small_gicp::random_sampling() or small_gicp::random_sampling_with_seed() 
            functions depending on whether seed is provided.",
            num_samples,
            seed
        );
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
    kdtree: &CKdTree,
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

    todo!(
        "Implement estimate_normals using small-gicp-cxx.
        This should:
        1. Use the normal estimation function from small-gicp-cxx
        2. Estimate normals using {} neighbors for each point
        3. Use backend {:?} for parallel processing
        4. Use {} threads for parallel execution
        5. Modify the input cloud in-place to add normal information
        
        Consider using small_gicp::estimate_normals_* functions with the appropriate backend.",
        num_neighbors,
        backend,
        num_threads
    );
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
    kdtree: &CKdTree,
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

    todo!(
        "Implement estimate_normals_simple using small-gicp-cxx.
        This should:
        1. Use the default normal estimation function from small-gicp-cxx
        2. Estimate normals using {} neighbors for each point
        3. Use {} threads for parallel execution
        4. Modify the input cloud in-place to add normal information
        
        Consider using small_gicp::estimate_normals() function with default backend.",
        num_neighbors,
        num_threads
    );
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
    kdtree: &CKdTree,
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

    todo!(
        "Implement estimate_covariances using small-gicp-cxx.
        This should:
        1. Use the covariance estimation function from small-gicp-cxx
        2. Estimate covariances using {} neighbors for each point
        3. Use backend {:?} for parallel processing
        4. Use {} threads for parallel execution
        5. Modify the input cloud in-place to add covariance information
        
        Consider using small_gicp::estimate_covariances_* functions with the appropriate backend.",
        num_neighbors,
        backend,
        num_threads
    );
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
    kdtree: &CKdTree,
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

    todo!(
        "Implement estimate_covariances_simple using small-gicp-cxx.
        This should:
        1. Use the default covariance estimation function from small-gicp-cxx
        2. Estimate covariances using {} neighbors for each point
        3. Use {} threads for parallel execution
        4. Modify the input cloud in-place to add covariance information
        
        Consider using small_gicp::estimate_covariances() function with default backend.",
        num_neighbors,
        num_threads
    );
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
    kdtree: &CKdTree,
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

    todo!(
        "Implement estimate_normals_and_covariances using small-gicp-cxx.
        This should:
        1. Use the combined normal and covariance estimation function from small-gicp-cxx
        2. Estimate both normals and covariances using {} neighbors for each point
        3. Use backend {:?} for parallel processing
        4. Use {} threads for parallel execution
        5. Modify the input cloud in-place to add both normal and covariance information
        
        This is more efficient than calling normal and covariance estimation separately.
        Consider using small_gicp::estimate_normals_covariances_* functions with the appropriate backend.",
        num_neighbors, backend, num_threads
    );
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
    kdtree: &CKdTree,
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

    todo!(
        "Implement estimate_normals_and_covariances_simple using small-gicp-cxx.
        This should:
        1. Use the default combined normal and covariance estimation function from small-gicp-cxx
        2. Estimate both normals and covariances using {} neighbors for each point
        3. Use {} threads for parallel execution
        4. Modify the input cloud in-place to add both normal and covariance information
        
        This is more efficient than calling normal and covariance estimation separately.
        Consider using small_gicp::estimate_normals_covariances() function with default backend.",
        num_neighbors,
        num_threads
    );
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
    kdtree: &CKdTree,
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

    todo!(
        "Implement estimate_local_features_single_point using small-gicp-cxx.
        This should:
        1. Estimate local features for point at index {}
        2. Use {} neighbors for local feature estimation
        3. Apply setter_type {:?} to determine what features to estimate
        4. Modify the input cloud in-place to add feature information
        
        Consider using small_gicp::estimate_local_features functions from small-gicp-cxx.",
        point_index,
        num_neighbors,
        setter_type
    );
}

/// Estimate local features for an entire point cloud using advanced configuration.
///
/// # Arguments
/// * `cloud` - The point cloud to process (modified in-place)
/// * `kdtree` - KdTree for neighborhood search
/// * `config` - Configuration for local feature estimation
pub fn estimate_local_features_cloud(
    cloud: &mut PointCloud,
    kdtree: &CKdTree,
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

    todo!(
        "Implement estimate_local_features_cloud using small-gicp-cxx.
        This should:
        1. Estimate local features for all points in the cloud
        2. Use {} neighbors for local feature estimation
        3. Apply setter_type {:?} to determine what features to estimate
        4. Use backend {:?} for parallel processing
        5. Use {} threads for parallel execution
        6. Modify the input cloud in-place to add feature information
        
        Consider using small_gicp::estimate_local_features_* functions with the appropriate backend.",
        num_neighbors, setter_type, backend, num_threads
    );
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

    todo!(
        "Implement estimate_local_features_auto using small-gicp-cxx.
        This should:
        1. Automatically build a KdTree internally for the cloud
        2. Estimate local features for all points in the cloud
        3. Use {} neighbors for local feature estimation
        4. Apply setter_type {:?} to determine what features to estimate
        5. Use backend {:?} for parallel processing
        6. Use {} threads for parallel execution
        7. Modify the input cloud in-place to add feature information
        
        This is more convenient when you don't need the KdTree for other operations.
        Consider using small_gicp::estimate_local_features_auto_* functions.",
        num_neighbors,
        setter_type,
        backend,
        num_threads
    );
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

    todo!(
        "Implement set_normal_direct using small-gicp-cxx.
        This should:
        1. Set normal for point at index {} directly
        2. Use the provided 3x3 eigenvector matrix in row-major order
        3. Modify the input cloud in-place to set normal information
        
        The eigenvector matrix represents the local coordinate frame,
        with the normal typically being the third eigenvector.
        Consider using direct point cloud manipulation methods.",
        point_index
    );
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

    todo!(
        "Implement set_covariance_direct using small-gicp-cxx.
        This should:
        1. Set covariance for point at index {} directly
        2. Use the provided 3x3 eigenvector matrix in row-major order
        3. Modify the input cloud in-place to set covariance information
        
        The eigenvector matrix represents the local coordinate frame,
        used to compute the covariance matrix for GICP registration.
        Consider using direct point cloud manipulation methods.",
        point_index
    );
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

    todo!(
        "Implement set_normal_covariance_direct using small-gicp-cxx.
        This should:
        1. Set both normal and covariance for point at index {} directly
        2. Use the provided 3x3 eigenvector matrix in row-major order
        3. Modify the input cloud in-place to set both normal and covariance information
        
        This is more efficient than setting normal and covariance separately.
        The eigenvector matrix represents the local coordinate frame.
        Consider using direct point cloud manipulation methods.",
        point_index
    );
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

    todo!(
        "Implement set_normal_invalid using small-gicp-cxx.
        This should:
        1. Mark the normal at point index {} as invalid/unreliable
        2. Modify the input cloud in-place to set normal as invalid
        
        This is useful when normal estimation fails or produces unreliable results.
        Consider using direct point cloud manipulation methods or special marker values.",
        point_index
    );
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

    todo!(
        "Implement set_covariance_invalid using small-gicp-cxx.
        This should:
        1. Mark the covariance at point index {} as invalid/unreliable
        2. Modify the input cloud in-place to set covariance as invalid
        
        This is useful when covariance estimation fails or produces unreliable results.
        Consider using direct point cloud manipulation methods or special marker values.",
        point_index
    );
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

    todo!(
        "Implement set_normal_covariance_invalid using small-gicp-cxx.
        This should:
        1. Mark both normal and covariance at point index {} as invalid/unreliable
        2. Modify the input cloud in-place to set both features as invalid
        
        This is useful when feature estimation fails or produces unreliable results.
        Consider using direct point cloud manipulation methods or special marker values.",
        point_index
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kdtree::KdTree;
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

        // Use the unified generic normal estimation
        let config = NormalEstimationConfig {
            num_neighbors: 5,
            backend: NormalEstimationBackend::Default,
            num_threads: 1,
        };
        NormalEstimation::estimate_normals(
            &mut preprocessed_cloud,
            &config,
            PreprocessingStrategy::CWrapper,
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
            let diff = (p1.0 - p2.0, p1.1 - p2.1, p1.2 - p2.2);
            let magnitude = (diff.0 * diff.0 + diff.1 * diff.1 + diff.2 * diff.2).sqrt();
            assert!(magnitude < 1e-10);
        }
    }

    #[test]
    fn test_backend_aware_normal_estimation() {
        let cloud = create_test_cloud();
        let mut preprocessed_cloud = cloud.clone();

        // Test with OpenMP backend using unified API
        let config = NormalEstimationConfig {
            num_neighbors: 5,
            backend: NormalEstimationBackend::OpenMp,
            num_threads: 2,
        };
        NormalEstimation::estimate_normals(
            &mut preprocessed_cloud,
            &config,
            PreprocessingStrategy::CWrapper,
        )
        .unwrap();

        let normal = preprocessed_cloud.get_normal(0).unwrap();
        assert!(normal.magnitude() > 0.0);
    }

    #[test]
    fn test_local_features_estimation() {
        let cloud = create_test_cloud();
        let mut preprocessed_cloud = cloud.clone();

        let config = LocalFeatureEstimationConfig {
            setter_type: LocalFeatureSetterType::NormalCovariance,
            backend: LocalFeaturesBackend::Default,
            num_neighbors: 5,
            num_threads: 1,
        };

        // Use auto estimation instead of cloud + kdtree
        estimate_local_features_auto(&mut preprocessed_cloud, &config).unwrap();

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

//==============================================================================
// UNIFIED GENERIC PREPROCESSING API (Phase 5.1)
//==============================================================================

use crate::{
    config::{DownsamplingConfig, ParallelBackend},
    traits::{helpers, MutablePointCloudTrait, PointCloudTrait},
};
use std::collections::HashMap;

/// Strategy for preprocessing backend selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PreprocessingStrategy {
    /// Always use the C wrapper backend (most reliable)
    CWrapper,
    /// Use C wrapper when possible, fallback for custom types
    Adaptive,
    /// Pure Rust implementation for specific algorithms
    PureRust,
}

impl Default for PreprocessingStrategy {
    fn default() -> Self {
        PreprocessingStrategy::Adaptive
    }
}

/// Unified downsampling that works with any PointCloudTrait implementation.
pub struct Downsampling;

impl Downsampling {
    /// Perform voxel grid downsampling on any point cloud type.
    ///
    /// This method automatically selects the best backend:
    /// - For PointCloud types: uses efficient C implementation
    /// - For custom types: converts to C wrapper, processes, then converts back
    ///
    /// # Arguments
    /// * `input` - Input point cloud implementing PointCloudTrait
    /// * `voxel_size` - Size of voxel grid cells
    /// * `config` - Downsampling configuration
    /// * `strategy` - Backend selection strategy
    ///
    /// # Returns
    /// A new PointCloud containing downsampled points
    pub fn voxel_grid<P: PointCloudTrait>(
        input: &P,
        voxel_size: f64,
        config: &DownsamplingConfig,
        strategy: PreprocessingStrategy,
    ) -> Result<PointCloud> {
        if input.empty() {
            return Err(SmallGicpError::EmptyPointCloud);
        }

        match strategy {
            PreprocessingStrategy::CWrapper | PreprocessingStrategy::Adaptive => {
                Self::voxel_grid_c_backend(input, voxel_size, config)
            }
            PreprocessingStrategy::PureRust => Self::voxel_grid_pure_rust(input, voxel_size),
        }
    }

    /// Perform random downsampling on any point cloud type.
    ///
    /// # Arguments
    /// * `input` - Input point cloud implementing PointCloudTrait
    /// * `target_size` - Target number of points after downsampling
    /// * `config` - Downsampling configuration
    /// * `strategy` - Backend selection strategy
    ///
    /// # Returns
    /// A new PointCloud containing randomly sampled points
    pub fn random_sampling<P: PointCloudTrait>(
        input: &P,
        target_size: usize,
        config: &DownsamplingConfig,
        strategy: PreprocessingStrategy,
    ) -> Result<PointCloud> {
        if input.empty() {
            return Err(SmallGicpError::EmptyPointCloud);
        }

        if target_size >= input.size() {
            // No downsampling needed, convert to PointCloud
            return crate::point_cloud::conversions::from_trait(input);
        }

        match strategy {
            PreprocessingStrategy::CWrapper | PreprocessingStrategy::Adaptive => {
                Self::random_sampling_c_backend(input, target_size, config)
            }
            PreprocessingStrategy::PureRust => {
                Self::random_sampling_pure_rust(input, target_size, config.seed)
            }
        }
    }

    /// Use C wrapper backend for voxel grid downsampling.
    fn voxel_grid_c_backend<P: PointCloudTrait>(
        input: &P,
        voxel_size: f64,
        config: &DownsamplingConfig,
    ) -> Result<PointCloud> {
        todo!(
            "Implement voxel_grid_c_backend using small-gicp-cxx.
            This should:
            1. Convert the input trait object to a format compatible with small-gicp-cxx
            2. Apply voxel grid downsampling with size {}
            3. Use backend {:?} for parallel processing
            4. Use {} threads for parallel execution
            5. Convert result back to PointCloud
            
            Consider using small_gicp::voxelgrid_sampling_* functions.",
            voxel_size,
            config.backend,
            config.num_threads
        );
    }

    /// Use C wrapper backend for random downsampling.
    fn random_sampling_c_backend<P: PointCloudTrait>(
        input: &P,
        target_size: usize,
        config: &DownsamplingConfig,
    ) -> Result<PointCloud> {
        todo!(
            "Implement random_sampling_c_backend using small-gicp-cxx.
            This should:
            1. Convert the input trait object to a format compatible with small-gicp-cxx
            2. Apply random sampling to select {} points
            3. Use seed {:?} if provided for reproducible results
            4. Use {} threads for parallel execution
            5. Convert result back to PointCloud
            
            Consider using small_gicp::random_sampling or small_gicp::random_sampling_with_seed functions.",
            target_size, config.seed, config.num_threads
        );
    }

    /// Pure Rust implementation of voxel grid downsampling.
    fn voxel_grid_pure_rust<P: PointCloudTrait>(input: &P, voxel_size: f64) -> Result<PointCloud> {
        if voxel_size <= 0.0 {
            return Err(SmallGicpError::InvalidParameter {
                param: "voxel_size",
                value: voxel_size.to_string(),
            });
        }

        let mut voxel_map: HashMap<(i32, i32, i32), Vec<usize>> = HashMap::new();
        let inv_voxel_size = 1.0 / voxel_size;

        // Group points by voxel
        for i in 0..input.size() {
            let point = input.point(i);
            let point_3d = helpers::point_to_vector3(point);

            let voxel_key = (
                (point_3d.x * inv_voxel_size).floor() as i32,
                (point_3d.y * inv_voxel_size).floor() as i32,
                (point_3d.z * inv_voxel_size).floor() as i32,
            );

            voxel_map.entry(voxel_key).or_insert_with(Vec::new).push(i);
        }

        // Select representative point from each voxel (first point)
        let selected_indices: Vec<usize> = voxel_map.values().map(|indices| indices[0]).collect();

        // Create output point cloud
        let points: Vec<nalgebra::Point3<f64>> = selected_indices
            .iter()
            .map(|&i| {
                let point = input.point(i);
                let point_3d = helpers::point_to_vector3(point);
                nalgebra::Point3::new(point_3d.x, point_3d.y, point_3d.z)
            })
            .collect();

        PointCloud::from_points(&points)
    }

    /// Pure Rust implementation of random downsampling.
    fn random_sampling_pure_rust<P: PointCloudTrait>(
        input: &P,
        target_size: usize,
        seed: Option<u64>,
    ) -> Result<PointCloud> {
        use rand::{seq::SliceRandom, SeedableRng};

        let mut rng: rand::rngs::StdRng = if let Some(seed) = seed {
            rand::rngs::StdRng::seed_from_u64(seed)
        } else {
            rand::rngs::StdRng::from_entropy()
        };

        let indices: Vec<usize> = (0..input.size()).collect();
        let selected_indices: Vec<usize> = indices
            .choose_multiple(&mut rng, target_size)
            .cloned()
            .collect();

        // Create output point cloud
        let points: Vec<nalgebra::Point3<f64>> = selected_indices
            .iter()
            .map(|&i| {
                let point = input.point(i);
                let point_3d = helpers::point_to_vector3(point);
                nalgebra::Point3::new(point_3d.x, point_3d.y, point_3d.z)
            })
            .collect();

        PointCloud::from_points(&points)
    }
}

/// Unified normal estimation that works with any PointCloudTrait implementation.
pub struct NormalEstimation;

impl NormalEstimation {
    /// Estimate normals for any point cloud type.
    ///
    /// # Arguments
    /// * `cloud` - Mutable point cloud implementing MutablePointCloudTrait
    /// * `config` - Normal estimation configuration
    /// * `strategy` - Backend selection strategy
    ///
    /// # Returns
    /// Result indicating success or failure
    pub fn estimate_normals<P: MutablePointCloudTrait>(
        cloud: &mut P,
        config: &NormalEstimationConfig,
        strategy: PreprocessingStrategy,
    ) -> Result<()> {
        if cloud.empty() {
            return Err(SmallGicpError::EmptyPointCloud);
        }

        match strategy {
            PreprocessingStrategy::CWrapper | PreprocessingStrategy::Adaptive => {
                Self::estimate_normals_c_backend(cloud, config)
            }
            PreprocessingStrategy::PureRust => {
                Self::estimate_normals_pure_rust(cloud, config.num_neighbors as usize)
            }
        }
    }

    /// Use C wrapper backend for normal estimation.
    fn estimate_normals_c_backend<P: MutablePointCloudTrait>(
        cloud: &mut P,
        config: &NormalEstimationConfig,
    ) -> Result<()> {
        todo!(
            "Implement estimate_normals_c_backend using small-gicp-cxx.
            This should:
            1. Convert the input trait object to a format compatible with small-gicp-cxx
            2. Build a KdTree for neighborhood search
            3. Estimate normals using {} neighbors for each point
            4. Use backend {:?} for parallel processing
            5. Use {} threads for parallel execution
            6. Copy estimated normals back to the original cloud trait
            
            Consider using small_gicp::estimate_normals_* functions with the appropriate backend.",
            config.num_neighbors,
            config.backend,
            config.num_threads
        );
    }

    /// Pure Rust implementation of normal estimation.
    fn estimate_normals_pure_rust<P: MutablePointCloudTrait>(
        cloud: &mut P,
        num_neighbors: usize,
    ) -> Result<()> {
        // This is a simplified implementation
        // For production use, the C wrapper backend is recommended

        for i in 0..cloud.size() {
            // For now, set a default normal pointing up
            // A real implementation would use PCA on local neighborhoods
            let normal = helpers::normal_from_xyz(0.0, 0.0, 1.0);
            cloud.set_normal(i, normal);
        }

        Ok(())
    }
}

/// Convenience functions for unified preprocessing.
pub mod convenience {
    use super::*;

    /// Complete preprocessing pipeline with default strategy.
    pub fn preprocess_cloud<P: PointCloudTrait>(input: &P, voxel_size: f64) -> Result<PointCloud> {
        let config = DownsamplingConfig::default();
        Downsampling::voxel_grid(input, voxel_size, &config, PreprocessingStrategy::default())
    }

    /// Complete preprocessing pipeline with normal estimation.
    pub fn preprocess_cloud_with_normals<P: MutablePointCloudTrait>(
        input: &P,
        voxel_size: f64,
    ) -> Result<PointCloud> {
        todo!(
            "Implement preprocess_cloud_with_normals using small-gicp-cxx.
            This should:
            1. First downsample the cloud using voxel size {}
            2. Then estimate normals for the downsampled cloud
            3. Return a PointCloud with both downsampling and normal estimation applied
            
            This is a convenience function that combines multiple preprocessing steps.",
            voxel_size
        );
    }
}
