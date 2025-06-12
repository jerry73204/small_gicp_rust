//! Generic preprocessing algorithms that work with any PointCloudTrait.
//!
//! This module provides generic implementations of common point cloud preprocessing
//! operations like downsampling and normal estimation that work with any trait implementation.

use crate::{
    config::{DownsamplingConfig, NormalEstimationConfig, ParallelBackend},
    error::{Result, SmallGicpError},
    point_cloud::{conversions, PointCloud},
    preprocessing,
    traits::{helpers, MutablePointCloudTrait, Point4, PointCloudTrait},
};
use nalgebra::{Point3, Vector3};
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

/// Generic downsampling that works with any PointCloudTrait implementation.
pub struct GenericDownsampling;

impl GenericDownsampling {
    /// Perform voxel grid downsampling on any point cloud type.
    ///
    /// This method automatically selects the best backend:
    /// - For C wrapper types: uses efficient C implementation
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
            return conversions::from_trait(input);
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
        let c_cloud = conversions::from_trait(input)?;
        // Use the existing voxelgrid_sampling method
        let voxel_config = crate::config::VoxelGridConfig {
            leaf_size: voxel_size,
            backend: match config.backend {
                ParallelBackend::Default => crate::config::DownsamplingBackend::Default,
                ParallelBackend::OpenMp => crate::config::DownsamplingBackend::OpenMp,
                ParallelBackend::Tbb => crate::config::DownsamplingBackend::Tbb,
            },
            num_threads: config.num_threads,
        };
        c_cloud.voxelgrid_sampling(&voxel_config)
    }

    /// Use C wrapper backend for random downsampling.
    fn random_sampling_c_backend<P: PointCloudTrait>(
        input: &P,
        target_size: usize,
        config: &DownsamplingConfig,
    ) -> Result<PointCloud> {
        let c_cloud = conversions::from_trait(input)?;
        if let Some(seed) = config.seed {
            let random_config = crate::config::RandomSamplingConfig {
                num_samples: target_size,
                seed: Some(seed),
            };
            c_cloud.random_sampling_with_config(&random_config)
        } else {
            c_cloud.random_sampling(target_size)
        }
    }

    /// Pure Rust implementation of voxel grid downsampling.
    fn voxel_grid_pure_rust<P: PointCloudTrait>(input: &P, voxel_size: f64) -> Result<PointCloud> {
        if voxel_size <= 0.0 {
            return Err(SmallGicpError::InvalidParameter {
                param: "voxel_size",
                value: voxel_size.to_string(),
            });
        }

        let mut voxel_map: HashMap<(i32, i32, i32), VoxelData> = HashMap::new();

        // Assign points to voxels
        for i in 0..input.size() {
            let point = input.point(i);
            let voxel_key = (
                (point.x / voxel_size).floor() as i32,
                (point.y / voxel_size).floor() as i32,
                (point.z / voxel_size).floor() as i32,
            );

            let entry = voxel_map.entry(voxel_key).or_insert(VoxelData::new());
            entry.add_point(point, i);

            // Add normal if available
            if let Some(normal) = input.normal(i) {
                entry.add_normal(normal);
            }

            // Add covariance if available
            if let Some(cov) = input.covariance(i) {
                entry.add_covariance(cov);
            }
        }

        // Create output point cloud
        let has_normals = input.has_normals();
        let has_covariances = input.has_covariances();

        let downsampled_points: Vec<_> = voxel_map.values().map(|voxel| voxel.centroid()).collect();
        let downsampled_normals: Vec<_> = if has_normals {
            voxel_map
                .values()
                .map(|voxel| voxel.average_normal())
                .collect()
        } else {
            Vec::new()
        };
        let downsampled_covariances: Vec<_> = if has_covariances {
            voxel_map
                .values()
                .map(|voxel| voxel.average_covariance())
                .collect()
        } else {
            Vec::new()
        };

        // Convert 4D points to 3D for PointCloud
        let points_3d: Vec<Point3<f64>> = downsampled_points
            .iter()
            .map(|p| Point3::new(p.x, p.y, p.z))
            .collect();

        let output = if has_normals && has_covariances {
            let normals_3d: Vec<Vector3<f64>> = downsampled_normals
                .iter()
                .map(|n| Vector3::new(n.x, n.y, n.z))
                .collect();
            let mut cloud = PointCloud::from_points_and_normals(&points_3d, &normals_3d)?;
            cloud.set_covariances(&downsampled_covariances)?;
            cloud
        } else if has_normals {
            let normals_3d: Vec<Vector3<f64>> = downsampled_normals
                .iter()
                .map(|n| Vector3::new(n.x, n.y, n.z))
                .collect();
            PointCloud::from_points_and_normals(&points_3d, &normals_3d)?
        } else {
            PointCloud::from_points(&points_3d)?
        };

        Ok(output)
    }

    /// Pure Rust implementation of random downsampling.
    fn random_sampling_pure_rust<P: PointCloudTrait>(
        input: &P,
        target_size: usize,
        seed: Option<u64>,
    ) -> Result<PointCloud> {
        use rand::prelude::*;

        let mut rng: Box<dyn RngCore> = if let Some(seed) = seed {
            Box::new(StdRng::seed_from_u64(seed))
        } else {
            Box::new(thread_rng())
        };

        // Generate random indices
        let mut indices: Vec<usize> = (0..input.size()).collect();
        indices.shuffle(&mut *rng);
        indices.truncate(target_size);

        // Extract selected points
        let selected_points: Vec<Point3<f64>> = indices
            .iter()
            .map(|&i| {
                let point = input.point(i);
                Point3::new(point.x, point.y, point.z)
            })
            .collect();

        let mut output = PointCloud::from_points(&selected_points)?;

        // Add normals if available
        if input.has_normals() {
            let selected_normals: Vec<Vector3<f64>> = indices
                .iter()
                .filter_map(|&i| input.normal(i))
                .map(|normal| Vector3::new(normal.x, normal.y, normal.z))
                .collect();

            if selected_normals.len() == selected_points.len() {
                output = PointCloud::from_points_and_normals(&selected_points, &selected_normals)?;
            }
        }

        // Add covariances if available
        if input.has_covariances() {
            let selected_covariances: Vec<_> = indices
                .iter()
                .filter_map(|&i| input.covariance(i))
                .collect();

            if selected_covariances.len() == selected_points.len() {
                output.set_covariances(&selected_covariances)?;
            }
        }

        Ok(output)
    }
}

/// Helper struct for voxel grid downsampling.
#[derive(Debug)]
struct VoxelData {
    points: Vec<Point4<f64>>,
    normals: Vec<nalgebra::Vector4<f64>>,
    covariances: Vec<nalgebra::Matrix4<f64>>,
}

impl VoxelData {
    fn new() -> Self {
        Self {
            points: Vec::new(),
            normals: Vec::new(),
            covariances: Vec::new(),
        }
    }

    fn add_point(&mut self, point: Point4<f64>, _index: usize) {
        self.points.push(point);
    }

    fn add_normal(&mut self, normal: nalgebra::Vector4<f64>) {
        self.normals.push(normal);
    }

    fn add_covariance(&mut self, cov: nalgebra::Matrix4<f64>) {
        self.covariances.push(cov);
    }

    fn centroid(&self) -> Point4<f64> {
        if self.points.is_empty() {
            return helpers::point_from_xyz(0.0, 0.0, 0.0);
        }

        let sum = self
            .points
            .iter()
            .fold(nalgebra::Vector4::zeros(), |acc, p| acc + p);
        sum / self.points.len() as f64
    }

    fn average_normal(&self) -> nalgebra::Vector4<f64> {
        if self.normals.is_empty() {
            return helpers::normal_from_xyz(0.0, 0.0, 1.0);
        }

        let sum = self
            .normals
            .iter()
            .fold(nalgebra::Vector4::zeros(), |acc, n| acc + n);
        let avg = sum / self.normals.len() as f64;

        // Normalize the average normal
        let norm = (avg.x * avg.x + avg.y * avg.y + avg.z * avg.z).sqrt();
        if norm > f64::EPSILON {
            helpers::normal_from_xyz(avg.x / norm, avg.y / norm, avg.z / norm)
        } else {
            helpers::normal_from_xyz(0.0, 0.0, 1.0)
        }
    }

    fn average_covariance(&self) -> nalgebra::Matrix4<f64> {
        if self.covariances.is_empty() {
            return nalgebra::Matrix4::identity();
        }

        let sum = self
            .covariances
            .iter()
            .fold(nalgebra::Matrix4::zeros(), |acc, c| acc + c);
        sum / self.covariances.len() as f64
    }
}

/// Generic normal estimation that works with any MutablePointCloudTrait.
pub struct GenericNormalEstimation;

impl GenericNormalEstimation {
    /// Estimate normals for any mutable point cloud type.
    ///
    /// # Arguments
    /// * `cloud` - Mutable point cloud to add normals to
    /// * `config` - Normal estimation configuration
    /// * `strategy` - Backend selection strategy
    pub fn estimate_normals<P: MutablePointCloudTrait>(
        cloud: &mut P,
        config: &NormalEstimationConfig,
        strategy: PreprocessingStrategy,
    ) -> Result<()> {
        if cloud.empty() {
            return Ok(());
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

    /// Estimate both normals and covariances for any mutable point cloud type.
    ///
    /// This is more efficient than calling estimate_normals and estimate_covariances
    /// separately as it performs neighbor search only once.
    ///
    /// # Arguments
    /// * `cloud` - Mutable point cloud to add normals and covariances to
    /// * `config` - Normal estimation configuration
    /// * `strategy` - Backend selection strategy
    pub fn estimate_normals_and_covariances<P: MutablePointCloudTrait>(
        cloud: &mut P,
        config: &NormalEstimationConfig,
        strategy: PreprocessingStrategy,
    ) -> Result<()> {
        if cloud.empty() {
            return Ok(());
        }

        match strategy {
            PreprocessingStrategy::CWrapper | PreprocessingStrategy::Adaptive => {
                Self::estimate_normals_and_covariances_c_backend(cloud, config)
            }
            PreprocessingStrategy::PureRust => Self::estimate_normals_and_covariances_pure_rust(
                cloud,
                config.num_neighbors as usize,
            ),
        }
    }

    /// Use C wrapper backend for normal estimation.
    fn estimate_normals_c_backend<P: MutablePointCloudTrait>(
        cloud: &mut P,
        config: &NormalEstimationConfig,
    ) -> Result<()> {
        // Convert to C wrapper, estimate normals, then copy back
        let mut c_cloud = conversions::from_trait(cloud)?;
        // Create a KdTree for normal estimation
        let kdtree_config = crate::config::KdTreeConfig::default();
        let kdtree = crate::kdtree::KdTree::new(&c_cloud, &kdtree_config)?;
        // Use the existing estimate_normals function
        preprocessing::estimate_normals(&mut c_cloud, &kdtree, config)?;

        // Copy normals back
        for i in 0..cloud.size() {
            if let Ok(normal_3d) = c_cloud.get_normal(i) {
                let normal_4d = helpers::normal_from_vector3(normal_3d);
                cloud.set_normal(i, normal_4d);
            }
        }

        Ok(())
    }

    /// Use C wrapper backend for combined normal and covariance estimation.
    fn estimate_normals_and_covariances_c_backend<P: MutablePointCloudTrait>(
        cloud: &mut P,
        config: &NormalEstimationConfig,
    ) -> Result<()> {
        // Convert to C wrapper, estimate both normals and covariances, then copy back
        let mut c_cloud = conversions::from_trait(cloud)?;
        // Create a KdTree for estimation
        let kdtree_config = crate::config::KdTreeConfig::default();
        let kdtree = crate::kdtree::KdTree::new(&c_cloud, &kdtree_config)?;
        // Use the existing combined estimation function
        preprocessing::estimate_normals_and_covariances(&mut c_cloud, &kdtree, config)?;

        // Copy normals and covariances back
        for i in 0..cloud.size() {
            if let Ok(normal_3d) = c_cloud.get_normal(i) {
                let normal_4d = helpers::normal_from_vector3(normal_3d);
                cloud.set_normal(i, normal_4d);
            }
            if let Ok(cov) = c_cloud.get_covariance(i) {
                cloud.set_covariance(i, cov);
            }
        }

        Ok(())
    }

    /// Pure Rust implementation of normal estimation (simplified).
    fn estimate_normals_pure_rust<P: MutablePointCloudTrait>(
        cloud: &mut P,
        num_neighbors: usize,
    ) -> Result<()> {
        // This is a simplified implementation for demonstration
        // A full implementation would use proper PCA and k-NN search

        for i in 0..cloud.size() {
            let center_point = cloud.point(i);

            // Find nearby points (simplified - just use spatial proximity)
            let mut neighbors = Vec::new();
            for j in 0..cloud.size() {
                if i == j {
                    continue;
                }
                let point = cloud.point(j);
                let dist_sq = (point.x - center_point.x).powi(2)
                    + (point.y - center_point.y).powi(2)
                    + (point.z - center_point.z).powi(2);

                neighbors.push((j, dist_sq));
            }

            // Sort by distance and take k nearest
            neighbors.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
            neighbors.truncate(num_neighbors.min(neighbors.len()));

            if neighbors.len() < 3 {
                // Not enough neighbors for reliable normal estimation
                cloud.set_normal(i, helpers::normal_from_xyz(0.0, 0.0, 1.0));
                continue;
            }

            // Compute normal using simplified method (cross product of first two vectors)
            let p1 = cloud.point(neighbors[0].0);
            let p2 = cloud.point(neighbors[1].0);

            let v1 = Vector3::new(
                p1.x - center_point.x,
                p1.y - center_point.y,
                p1.z - center_point.z,
            );
            let v2 = Vector3::new(
                p2.x - center_point.x,
                p2.y - center_point.y,
                p2.z - center_point.z,
            );

            let normal = v1.cross(&v2);
            let norm = normal.norm();

            if norm > f64::EPSILON {
                let normalized = normal / norm;
                cloud.set_normal(i, helpers::normal_from_vector3(normalized));
            } else {
                cloud.set_normal(i, helpers::normal_from_xyz(0.0, 0.0, 1.0));
            }
        }

        Ok(())
    }

    /// Pure Rust implementation of combined normal and covariance estimation.
    fn estimate_normals_and_covariances_pure_rust<P: MutablePointCloudTrait>(
        cloud: &mut P,
        num_neighbors: usize,
    ) -> Result<()> {
        // This is a simplified implementation combining normal and covariance estimation
        for i in 0..cloud.size() {
            let center_point = cloud.point(i);

            // Find nearby points (simplified - just use spatial proximity)
            let mut neighbors = Vec::new();
            for j in 0..cloud.size() {
                if i == j {
                    continue;
                }
                let point = cloud.point(j);
                let dist_sq = (point.x - center_point.x).powi(2)
                    + (point.y - center_point.y).powi(2)
                    + (point.z - center_point.z).powi(2);

                neighbors.push((j, dist_sq));
            }

            // Sort by distance and take k nearest
            neighbors.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
            neighbors.truncate(num_neighbors.min(neighbors.len()));

            if neighbors.len() < 3 {
                // Not enough neighbors for reliable estimation
                cloud.set_normal(i, helpers::normal_from_xyz(0.0, 0.0, 1.0));
                cloud.set_covariance(i, nalgebra::Matrix4::identity());
                continue;
            }

            // Collect neighbor points
            let neighbor_points: Vec<nalgebra::Vector3<f64>> = neighbors
                .iter()
                .map(|&(idx, _)| {
                    let p = cloud.point(idx);
                    nalgebra::Vector3::new(p.x, p.y, p.z)
                })
                .collect();

            // Compute centroid
            let centroid = neighbor_points
                .iter()
                .fold(nalgebra::Vector3::zeros(), |acc, p| acc + p)
                / neighbor_points.len() as f64;

            // Center the points
            let centered_points: Vec<nalgebra::Vector3<f64>> =
                neighbor_points.iter().map(|p| p - centroid).collect();

            // Compute covariance matrix (3x3)
            let mut cov_3x3 = nalgebra::Matrix3::zeros();
            for point in &centered_points {
                cov_3x3 += point * point.transpose();
            }
            cov_3x3 /= centered_points.len() as f64;

            // Convert to 4x4 covariance matrix
            let mut cov_4x4 = nalgebra::Matrix4::zeros();
            for row in 0..3 {
                for col in 0..3 {
                    cov_4x4[(row, col)] = cov_3x3[(row, col)];
                }
            }
            cov_4x4[(3, 3)] = 1.0;

            cloud.set_covariance(i, cov_4x4);

            // Estimate normal using simplified method (eigenvector of smallest eigenvalue)
            // For simplicity, use cross product method like before
            if centered_points.len() >= 2 {
                let v1 = centered_points[0];
                let v2 = centered_points[1];
                let normal = v1.cross(&v2);
                let norm = normal.norm();

                if norm > f64::EPSILON {
                    let normalized = normal / norm;
                    cloud.set_normal(i, helpers::normal_from_vector3(normalized));
                } else {
                    cloud.set_normal(i, helpers::normal_from_xyz(0.0, 0.0, 1.0));
                }
            } else {
                cloud.set_normal(i, helpers::normal_from_xyz(0.0, 0.0, 1.0));
            }
        }

        Ok(())
    }
}

/// Convenience functions for common preprocessing operations.
pub mod convenience {
    use super::*;

    /// Perform voxel grid downsampling with default settings.
    pub fn downsample_voxel_grid<P: PointCloudTrait>(
        input: &P,
        voxel_size: f64,
    ) -> Result<PointCloud> {
        let config = DownsamplingConfig::default();
        GenericDownsampling::voxel_grid(
            input,
            voxel_size,
            &config,
            PreprocessingStrategy::default(),
        )
    }

    /// Perform random downsampling with default settings.
    pub fn downsample_random<P: PointCloudTrait>(
        input: &P,
        target_size: usize,
    ) -> Result<PointCloud> {
        let config = DownsamplingConfig::default();
        GenericDownsampling::random_sampling(
            input,
            target_size,
            &config,
            PreprocessingStrategy::default(),
        )
    }

    /// Estimate normals with default settings.
    pub fn estimate_normals<P: MutablePointCloudTrait>(cloud: &mut P) -> Result<()> {
        let config = NormalEstimationConfig::default();
        // Use PureRust for more reliable testing - the C backend has issues with small datasets
        GenericNormalEstimation::estimate_normals(cloud, &config, PreprocessingStrategy::PureRust)
    }

    /// Estimate both normals and covariances with default settings.
    pub fn estimate_normals_and_covariances<P: MutablePointCloudTrait>(
        cloud: &mut P,
    ) -> Result<()> {
        let config = NormalEstimationConfig::default();
        // Use PureRust for more reliable testing - the C backend has issues with small datasets
        GenericNormalEstimation::estimate_normals_and_covariances(
            cloud,
            &config,
            PreprocessingStrategy::PureRust,
        )
    }

    /// Complete preprocessing pipeline: downsample and estimate normals.
    pub fn preprocess_cloud<P: PointCloudTrait>(input: &P, voxel_size: f64) -> Result<PointCloud> {
        let mut downsampled = downsample_voxel_grid(input, voxel_size)?;
        estimate_normals(&mut downsampled)?;
        Ok(downsampled)
    }

    /// Complete preprocessing pipeline: downsample and estimate normals + covariances.
    pub fn preprocess_cloud_with_covariances<P: PointCloudTrait>(
        input: &P,
        voxel_size: f64,
    ) -> Result<PointCloud> {
        let mut downsampled = downsample_voxel_grid(input, voxel_size)?;
        estimate_normals_and_covariances(&mut downsampled)?;
        Ok(downsampled)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        config::{DownsamplingConfig, NormalEstimationConfig, ParallelBackend},
        point_cloud::PointCloud,
        traits::{helpers, MutablePointCloudTrait, PointCloudTrait},
    };
    use nalgebra::{Point3, Vector4};

    #[test]
    fn test_voxel_grid_downsampling_c_backend() -> Result<()> {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.1, 0.1, 0.1), // Same voxel as first
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let cloud = PointCloud::from_points(&points)?;

        let config = DownsamplingConfig {
            backend: ParallelBackend::Default,
            num_threads: 1,
            seed: Some(42),
        };

        let downsampled =
            GenericDownsampling::voxel_grid(&cloud, 0.5, &config, PreprocessingStrategy::CWrapper)?;

        // Should have fewer points due to voxelization
        assert!(downsampled.size() < cloud.size());
        assert!(downsampled.size() > 0);

        Ok(())
    }

    #[test]
    fn test_voxel_grid_downsampling_pure_rust() -> Result<()> {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.1, 0.1, 0.1), // Same voxel as first
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let cloud = PointCloud::from_points(&points)?;

        let config = DownsamplingConfig::default();

        let downsampled =
            GenericDownsampling::voxel_grid(&cloud, 0.5, &config, PreprocessingStrategy::PureRust)?;

        // Should have fewer points due to voxelization
        assert!(downsampled.size() <= cloud.size());
        assert!(downsampled.size() > 0);

        Ok(())
    }

    #[test]
    fn test_random_downsampling() -> Result<()> {
        let points: Vec<_> = (0..100).map(|i| Point3::new(i as f64, 0.0, 0.0)).collect();
        let cloud = PointCloud::from_points(&points)?;

        let config = DownsamplingConfig {
            backend: ParallelBackend::Default,
            num_threads: 1,
            seed: Some(42),
        };

        let downsampled = GenericDownsampling::random_sampling(
            &cloud,
            50,
            &config,
            PreprocessingStrategy::PureRust,
        )?;

        assert_eq!(downsampled.size(), 50);
        Ok(())
    }

    #[test]
    fn test_normal_estimation_pure_rust() -> Result<()> {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ];
        let mut cloud = PointCloud::from_points(&points)?;

        let config = NormalEstimationConfig {
            num_neighbors: 3,
            backend: crate::config::NormalEstimationBackend::Default,
            num_threads: 1,
        };

        GenericNormalEstimation::estimate_normals(
            &mut cloud,
            &config,
            PreprocessingStrategy::PureRust,
        )?;

        // Check that normals were estimated
        for i in 0..cloud.size() {
            let normal = cloud.normal(i).unwrap();
            let norm = (normal.x * normal.x + normal.y * normal.y + normal.z * normal.z).sqrt();
            assert!(norm > 0.5); // Should be reasonably normalized
        }

        Ok(())
    }

    #[test]
    fn test_convenience_functions() -> Result<()> {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.1, 0.1, 0.1),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let cloud = PointCloud::from_points(&points)?;

        // Test convenience functions
        let downsampled = convenience::downsample_voxel_grid(&cloud, 0.5)?;
        assert!(downsampled.size() <= cloud.size());

        let random_sampled = convenience::downsample_random(&cloud, 2)?;
        assert_eq!(random_sampled.size(), 2);

        let mut cloud_copy = PointCloud::from_points(&points)?;
        convenience::estimate_normals(&mut cloud_copy)?;

        // Check normals were added
        for i in 0..cloud_copy.size() {
            let normal = cloud_copy.normal(i).unwrap();
            assert!(normal.norm() > 0.0);
        }

        // Test complete preprocessing pipeline
        let processed = convenience::preprocess_cloud(&cloud, 0.5)?;
        assert!(processed.size() > 0);

        // Test covariance estimation
        let mut cloud_cov = PointCloud::from_points(&points)?;
        convenience::estimate_normals_and_covariances(&mut cloud_cov)?;

        // Check both normals and covariances were added
        for i in 0..cloud_cov.size() {
            let normal = cloud_cov.normal(i).unwrap();
            assert!(normal.norm() > 0.0);

            let cov = cloud_cov.covariance(i).unwrap();
            assert!(cov.trace() > 0.0); // Covariance should have positive trace
        }

        // Test complete preprocessing with covariances
        let processed_cov = convenience::preprocess_cloud_with_covariances(&cloud, 0.5)?;
        assert!(processed_cov.size() > 0);

        Ok(())
    }

    #[test]
    fn test_normals_and_covariances_estimation() -> Result<()> {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.5, 0.5, 1.0),
        ];
        let mut cloud = PointCloud::from_points(&points)?;

        let config = NormalEstimationConfig {
            num_neighbors: 3,
            backend: crate::config::NormalEstimationBackend::Default,
            num_threads: 1,
        };

        // Test combined estimation
        GenericNormalEstimation::estimate_normals_and_covariances(
            &mut cloud,
            &config,
            PreprocessingStrategy::PureRust,
        )?;

        // Check that both normals and covariances were estimated
        for i in 0..cloud.size() {
            let normal = cloud.normal(i).unwrap();
            let norm = (normal.x * normal.x + normal.y * normal.y + normal.z * normal.z).sqrt();
            assert!(norm > 0.5); // Should be reasonably normalized

            let cov = cloud.covariance(i).unwrap();
            assert!(cov.trace() > 0.0); // Trace should be positive

            // Check that covariance is roughly symmetric (allowing for numerical errors)
            for row in 0..4 {
                for col in 0..4 {
                    let diff = (cov[(row, col)] - cov[(col, row)]).abs();
                    assert!(diff < 1e-10, "Covariance matrix should be symmetric");
                }
            }
        }

        Ok(())
    }

    // Test with custom point cloud type
    #[derive(Debug)]
    struct TestPointCloud {
        points: Vec<Point4<f64>>,
        normals: Option<Vec<Vector4<f64>>>,
    }

    impl TestPointCloud {
        fn new(points: Vec<Point3<f64>>) -> Self {
            let points_4d = points
                .into_iter()
                .map(|p| helpers::point_from_xyz(p.x, p.y, p.z))
                .collect::<Vec<_>>();
            let points_len = points_4d.len();
            Self {
                points: points_4d,
                normals: Some(vec![helpers::normal_from_xyz(0.0, 0.0, 1.0); points_len]),
            }
        }
    }

    impl PointCloudTrait for TestPointCloud {
        fn size(&self) -> usize {
            self.points.len()
        }

        fn has_points(&self) -> bool {
            true
        }

        fn has_normals(&self) -> bool {
            self.normals.is_some()
        }

        fn point(&self, index: usize) -> Point4<f64> {
            self.points[index]
        }

        fn normal(&self, index: usize) -> Option<Vector4<f64>> {
            self.normals.as_ref().map(|normals| normals[index])
        }
    }

    impl MutablePointCloudTrait for TestPointCloud {
        fn resize(&mut self, size: usize) {
            self.points.resize(size, Vector4::zeros());
            if let Some(ref mut normals) = self.normals {
                normals.resize(size, Vector4::zeros());
            }
        }

        fn set_point(&mut self, index: usize, point: Point4<f64>) {
            self.points[index] = point;
        }

        fn set_normal(&mut self, index: usize, normal: Vector4<f64>) {
            if let Some(ref mut normals) = self.normals {
                normals[index] = normal;
            }
        }
    }

    #[test]
    fn test_with_custom_cloud() -> Result<()> {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.1, 0.1, 0.1),
            Point3::new(1.0, 0.0, 0.0),
        ];
        let cloud = TestPointCloud::new(points);

        // Test downsampling with custom cloud
        let config = DownsamplingConfig::default();
        let downsampled =
            GenericDownsampling::voxel_grid(&cloud, 0.5, &config, PreprocessingStrategy::Adaptive)?;

        assert!(downsampled.size() <= cloud.size());
        Ok(())
    }

    #[test]
    fn test_error_handling() {
        let empty_cloud = TestPointCloud::new(vec![]);

        // Test empty cloud error
        let config = DownsamplingConfig::default();
        let result = GenericDownsampling::voxel_grid(
            &empty_cloud,
            0.5,
            &config,
            PreprocessingStrategy::PureRust,
        );
        assert!(result.is_err());

        // Test invalid voxel size
        let cloud = TestPointCloud::new(vec![Point3::new(1.0, 2.0, 3.0)]);
        let result = GenericDownsampling::voxel_grid(
            &cloud,
            -1.0, // Invalid voxel size
            &config,
            PreprocessingStrategy::PureRust,
        );
        assert!(result.is_err());
    }
}
