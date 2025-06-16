//! Point cloud preprocessing utilities.
//!
//! This module provides thin wrappers around the C++ preprocessing functions.
//! The actual preprocessing functionality is implemented through the CXX bridge.

use crate::{error::Result, point_cloud::PointCloud};

/// Preprocessing functions for point clouds
pub struct Preprocessing;

impl Preprocessing {
    /// Downsample a point cloud using voxel grid filtering
    ///
    /// This method reduces the point cloud density by creating a regular 3D grid
    /// and keeping only one representative point per voxel.
    ///
    /// # Arguments
    /// * `cloud` - Input point cloud
    /// * `voxel_size` - Size of each voxel in meters
    /// * `num_threads` - Number of threads for parallel processing
    ///
    /// # Returns
    /// A new downsampled point cloud
    pub fn voxel_downsample(cloud: &PointCloud, voxel_size: f64, num_threads: usize) -> PointCloud {
        let cxx_cloud = cloud.clone().into_cxx();
        let downsampled = small_gicp_cxx::Preprocessing::voxel_downsample(
            &cxx_cloud,
            voxel_size,
            num_threads as i32,
        );
        PointCloud::from_cxx(downsampled)
    }

    /// Downsample a point cloud using random sampling
    ///
    /// This method randomly selects a specified number of points from the input cloud.
    ///
    /// # Arguments
    /// * `cloud` - Input point cloud
    /// * `num_samples` - Number of points to sample
    ///
    /// # Returns
    /// A new randomly sampled point cloud
    pub fn random_downsample(cloud: &PointCloud, num_samples: usize) -> PointCloud {
        let cxx_cloud = cloud.clone().into_cxx();
        let downsampled = small_gicp_cxx::Preprocessing::random_downsample(&cxx_cloud, num_samples);
        PointCloud::from_cxx(downsampled)
    }

    /// Estimate normals for all points in the cloud
    ///
    /// This method computes surface normals using local neighborhood analysis.
    /// The normals are stored directly in the point cloud.
    ///
    /// # Arguments
    /// * `cloud` - Point cloud to estimate normals for (will be modified)
    /// * `num_neighbors` - Number of neighbors to use for normal estimation
    /// * `num_threads` - Number of threads for parallel processing
    pub fn estimate_normals(
        cloud: &mut PointCloud,
        num_neighbors: usize,
        num_threads: usize,
    ) -> Result<()> {
        let mut cxx_cloud = cloud.clone().into_cxx();
        small_gicp_cxx::Preprocessing::estimate_normals(
            &mut cxx_cloud,
            num_neighbors as i32,
            num_threads as i32,
        );

        // Copy the results back to the Rust point cloud
        *cloud = PointCloud::from_cxx(cxx_cloud);
        Ok(())
    }

    /// Estimate covariances for all points in the cloud
    ///
    /// This method computes local surface covariance matrices for each point.
    /// The covariances are stored directly in the point cloud and are required
    /// for GICP registration.
    ///
    /// # Arguments
    /// * `cloud` - Point cloud to estimate covariances for (will be modified)
    /// * `num_neighbors` - Number of neighbors to use for covariance estimation
    /// * `num_threads` - Number of threads for parallel processing
    pub fn estimate_covariances(
        cloud: &mut PointCloud,
        num_neighbors: usize,
        num_threads: usize,
    ) -> Result<()> {
        let mut cxx_cloud = cloud.clone().into_cxx();
        small_gicp_cxx::Preprocessing::estimate_covariances(
            &mut cxx_cloud,
            num_neighbors as i32,
            num_threads as i32,
        );

        // Copy the results back to the Rust point cloud
        *cloud = PointCloud::from_cxx(cxx_cloud);
        Ok(())
    }

    /// Complete preprocessing pipeline for point cloud registration
    ///
    /// This convenience method performs downsampling, normal estimation, and
    /// covariance estimation in one call.
    ///
    /// # Arguments
    /// * `cloud` - Input point cloud
    /// * `voxel_size` - Voxel size for downsampling
    /// * `num_neighbors` - Number of neighbors for normal/covariance estimation
    /// * `num_threads` - Number of threads for parallel processing
    ///
    /// # Returns
    /// A new preprocessed point cloud ready for registration
    pub fn preprocess_for_registration(
        cloud: &PointCloud,
        voxel_size: f64,
        num_neighbors: usize,
        num_threads: usize,
    ) -> PointCloud {
        let cxx_cloud = cloud.clone().into_cxx();
        let processed = small_gicp_cxx::Preprocessing::preprocess_for_registration(
            &cxx_cloud,
            voxel_size,
            num_neighbors as i32,
            num_threads as i32,
        );
        PointCloud::from_cxx(processed)
    }
}

/// Estimate local features automatically using the given configuration.
pub fn estimate_local_features_auto(
    cloud: &mut PointCloud,
    config: &crate::config::LocalFeatureEstimationConfig,
) -> Result<()> {
    use crate::config::LocalFeatureSetterType;

    match config.setter_type {
        LocalFeatureSetterType::Normal => Preprocessing::estimate_normals(
            cloud,
            config.num_neighbors as usize,
            config.num_threads,
        ),
        LocalFeatureSetterType::Covariance => Preprocessing::estimate_covariances(
            cloud,
            config.num_neighbors as usize,
            config.num_threads,
        ),
        LocalFeatureSetterType::NormalCovariance => {
            Preprocessing::estimate_normals(
                cloud,
                config.num_neighbors as usize,
                config.num_threads,
            )?;
            Preprocessing::estimate_covariances(
                cloud,
                config.num_neighbors as usize,
                config.num_threads,
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_voxel_downsampling() {
        let mut cloud = PointCloud::new().unwrap();
        // Add a dense grid of points
        for i in 0..10 {
            for j in 0..10 {
                cloud.add_point(i as f64 * 0.01, j as f64 * 0.01, 0.0);
            }
        }

        let downsampled = Preprocessing::voxel_downsample(&cloud, 0.05, 1);

        // Downsampled cloud should have fewer points
        assert!(downsampled.len() < cloud.len());
        assert!(downsampled.len() > 0);
    }

    #[test]
    fn test_random_downsampling() {
        let mut cloud = PointCloud::new().unwrap();
        for i in 0..100 {
            cloud.add_point(i as f64, 0.0, 0.0);
        }

        let downsampled = Preprocessing::random_downsample(&cloud, 50);
        assert_eq!(downsampled.len(), 50);
    }

    #[test]
    fn test_normal_estimation() {
        let mut cloud = PointCloud::new().unwrap();
        // Create a simple planar surface with more points to ensure normal estimation works
        for i in 0..20 {
            for j in 0..20 {
                cloud.add_point(i as f64 * 0.1, j as f64 * 0.1, 0.0);
            }
        }

        Preprocessing::estimate_normals(&mut cloud, 5, 1).unwrap();

        // Check that normals data is available
        assert!(cloud.has_normals());
    }

    #[test]
    fn test_covariance_estimation() {
        let mut cloud = PointCloud::new().unwrap();
        // Create a simple planar surface
        for i in 0..20 {
            for j in 0..20 {
                cloud.add_point(i as f64 * 0.1, j as f64 * 0.1, 0.0);
            }
        }

        Preprocessing::estimate_covariances(&mut cloud, 5, 1).unwrap();

        // Check that covariances data is available
        assert!(cloud.has_covariances());
    }

    #[test]
    fn test_preprocess_for_registration() {
        let mut cloud = PointCloud::new().unwrap();
        for i in 0..30 {
            for j in 0..30 {
                cloud.add_point(i as f64 * 0.01, j as f64 * 0.01, 0.0);
            }
        }

        let processed = Preprocessing::preprocess_for_registration(&cloud, 0.05, 10, 1);

        // Should have fewer points due to downsampling
        assert!(processed.len() < cloud.len());
        assert!(processed.len() > 0);
        // Should have normals and covariances
        assert!(processed.has_normals());
        assert!(processed.has_covariances());
    }
}
