//! Generic KdTree implementation that works with any PointCloudTrait.
//!
//! This module provides a generic KdTree that can work with any implementation
//! of PointCloudTrait, automatically choosing the best backend strategy.

use crate::{
    config::{KdTreeConfig, KnnConfig},
    error::{Result, SmallGicpError},
    kdtree::{KdTree as CKdTree, UnsafeKdTree},
    point_cloud::conversions,
    traits::{helpers, PointCloudTrait},
};
use nalgebra::Point3;
use std::marker::PhantomData;

/// Strategy for KdTree backend selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KdTreeStrategy {
    /// Always use the C wrapper backend (most reliable)
    CWrapper,
    /// Use C wrapper when possible, fallback for custom types
    Adaptive,
    /// Future: Pure Rust implementation (not yet implemented)
    #[allow(dead_code)]
    PureRust,
}

impl Default for KdTreeStrategy {
    fn default() -> Self {
        KdTreeStrategy::Adaptive
    }
}

/// A generic KdTree that works with any PointCloudTrait implementation.
///
/// This struct provides a unified interface for nearest neighbor search
/// that automatically selects the best backend based on the point cloud type.
///
/// # Examples
///
/// ```rust
/// use nalgebra::Point3;
/// use small_gicp_rust::{
///     config::KdTreeConfig, generic::GenericKdTree, point_cloud::PointCloud,
///     traits::PointCloudTrait,
/// };
///
/// // Works with C wrapper PointCloud
/// let cloud = PointCloud::from_points(&[Point3::new(1.0, 2.0, 3.0)])?;
/// let kdtree = GenericKdTree::new(&cloud, &KdTreeConfig::default())?;
/// let (index, distance) = kdtree.nearest_neighbor(Point3::new(1.0, 2.0, 3.0))?;
///
/// // Also works with custom implementations
/// let custom_cloud = MyCustomPointCloud::new();
/// let kdtree = GenericKdTree::new(&custom_cloud, &KdTreeConfig::default())?;
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
#[derive(Debug)]
pub struct GenericKdTree<'a, P: PointCloudTrait> {
    backend: KdTreeBackend,
    phantom: PhantomData<&'a P>,
}

/// Internal backend storage for different KdTree implementations.
#[derive(Debug)]
enum KdTreeBackend {
    /// Uses the C wrapper KdTree with owned point cloud data
    CWrapper(CKdTree),
    /// Uses the unsafe C wrapper KdTree with borrowed data
    UnsafeCWrapper(UnsafeKdTree),
    /// Future: Pure Rust implementation
    #[allow(dead_code)]
    PureRust,
}

impl<'a, P: PointCloudTrait> GenericKdTree<'a, P> {
    /// Create a new generic KdTree from any point cloud type.
    ///
    /// This constructor automatically selects the best backend strategy:
    /// - For C wrapper PointCloud: uses direct C backend
    /// - For custom types: converts to C wrapper then uses C backend
    ///
    /// # Arguments
    /// * `cloud` - The point cloud to build the tree from
    /// * `config` - KdTree configuration
    ///
    /// # Returns
    /// A generic KdTree that provides unified access to nearest neighbor search
    pub fn new(cloud: &'a P, config: &KdTreeConfig) -> Result<Self> {
        Self::new_with_strategy(cloud, config, KdTreeStrategy::default())
    }

    /// Create a new generic KdTree with explicit backend strategy.
    ///
    /// # Arguments
    /// * `cloud` - The point cloud to build the tree from
    /// * `config` - KdTree configuration
    /// * `strategy` - Backend selection strategy
    pub fn new_with_strategy(
        cloud: &'a P,
        config: &KdTreeConfig,
        strategy: KdTreeStrategy,
    ) -> Result<Self> {
        if cloud.empty() {
            return Err(SmallGicpError::EmptyPointCloud);
        }

        let backend = match strategy {
            KdTreeStrategy::CWrapper => Self::create_c_wrapper_backend(cloud, config)?,
            KdTreeStrategy::Adaptive => Self::create_adaptive_backend(cloud, config)?,
            KdTreeStrategy::PureRust => {
                return Err(SmallGicpError::NotImplemented(
                    "Pure Rust KdTree not yet implemented".to_string(),
                ))
            }
        };

        Ok(GenericKdTree {
            backend,
            phantom: PhantomData,
        })
    }

    /// Create a KdTree using the C wrapper backend.
    ///
    /// This method converts any point cloud to the C wrapper format,
    /// which provides maximum compatibility and performance.
    fn create_c_wrapper_backend(cloud: &P, config: &KdTreeConfig) -> Result<KdTreeBackend> {
        // Convert to C wrapper format
        let c_cloud = conversions::from_trait(cloud)?;
        let kdtree = CKdTree::new(&c_cloud, config)?;
        Ok(KdTreeBackend::CWrapper(kdtree))
    }

    /// Create a KdTree using adaptive backend selection.
    ///
    /// This tries to use the most efficient backend for the given point cloud type.
    fn create_adaptive_backend(cloud: &P, config: &KdTreeConfig) -> Result<KdTreeBackend> {
        // For now, adaptive strategy always uses C wrapper conversion
        // In the future, this could detect if the cloud is already a C wrapper
        // and use it directly, or use different strategies for different types
        Self::create_c_wrapper_backend(cloud, config)
    }

    /// Find the nearest neighbor to a query point.
    ///
    /// # Arguments
    /// * `query` - The query point (3D coordinates)
    ///
    /// # Returns
    /// A tuple of (point_index, squared_distance)
    pub fn nearest_neighbor(&self, query: Point3<f64>) -> Result<(usize, f64)> {
        match &self.backend {
            KdTreeBackend::CWrapper(kdtree) => kdtree.nearest_neighbor(query),
            KdTreeBackend::UnsafeCWrapper(kdtree) => kdtree.nearest_neighbor(query),
            KdTreeBackend::PureRust => Err(SmallGicpError::NotImplemented(
                "Pure Rust KdTree not yet implemented".to_string(),
            )),
        }
    }

    /// Find the nearest neighbor with custom search settings.
    ///
    /// # Arguments
    /// * `query` - The query point (3D coordinates)
    /// * `settings` - KNN search settings for early termination
    ///
    /// # Returns
    /// A tuple of (point_index, squared_distance)
    pub fn nearest_neighbor_with_settings(
        &self,
        query: Point3<f64>,
        settings: &KnnConfig,
    ) -> Result<(usize, f64)> {
        match &self.backend {
            KdTreeBackend::CWrapper(kdtree) => {
                kdtree.nearest_neighbor_with_settings(query, settings)
            }
            KdTreeBackend::UnsafeCWrapper(kdtree) => {
                kdtree.nearest_neighbor_with_settings(query, settings)
            }
            KdTreeBackend::PureRust => Err(SmallGicpError::NotImplemented(
                "Pure Rust KdTree not yet implemented".to_string(),
            )),
        }
    }

    /// Find k nearest neighbors to a query point.
    ///
    /// # Arguments
    /// * `query` - The query point (3D coordinates)
    /// * `k` - Number of neighbors to find
    ///
    /// # Returns
    /// A vector of tuples (point_index, squared_distance) sorted by distance
    pub fn knn_search(&self, query: Point3<f64>, k: usize) -> Result<Vec<(usize, f64)>> {
        match &self.backend {
            KdTreeBackend::CWrapper(kdtree) => kdtree.knn_search(query, k),
            KdTreeBackend::UnsafeCWrapper(kdtree) => kdtree.knn_search(query, k),
            KdTreeBackend::PureRust => Err(SmallGicpError::NotImplemented(
                "Pure Rust KdTree not yet implemented".to_string(),
            )),
        }
    }

    /// Find k nearest neighbors with custom search settings.
    ///
    /// # Arguments
    /// * `query` - The query point (3D coordinates)
    /// * `k` - Number of neighbors to find
    /// * `settings` - KNN search settings for early termination
    ///
    /// # Returns
    /// A vector of tuples (point_index, squared_distance) sorted by distance
    pub fn knn_search_with_settings(
        &self,
        query: Point3<f64>,
        k: usize,
        settings: &KnnConfig,
    ) -> Result<Vec<(usize, f64)>> {
        match &self.backend {
            KdTreeBackend::CWrapper(kdtree) => kdtree.knn_search_with_settings(query, k, settings),
            KdTreeBackend::UnsafeCWrapper(kdtree) => {
                kdtree.knn_search_with_settings(query, k, settings)
            }
            KdTreeBackend::PureRust => Err(SmallGicpError::NotImplemented(
                "Pure Rust KdTree not yet implemented".to_string(),
            )),
        }
    }

    /// Find all neighbors within a given radius.
    ///
    /// # Arguments
    /// * `query` - The query point (3D coordinates)
    /// * `radius` - The search radius
    /// * `max_neighbors` - Maximum number of neighbors to return
    ///
    /// # Returns
    /// A vector of tuples (point_index, squared_distance) within the radius
    pub fn radius_search(
        &self,
        query: Point3<f64>,
        radius: f64,
        max_neighbors: usize,
    ) -> Result<Vec<(usize, f64)>> {
        match &self.backend {
            KdTreeBackend::CWrapper(kdtree) => kdtree.radius_search(query, radius, max_neighbors),
            KdTreeBackend::UnsafeCWrapper(kdtree) => {
                kdtree.radius_search(query, radius, max_neighbors)
            }
            KdTreeBackend::PureRust => Err(SmallGicpError::NotImplemented(
                "Pure Rust KdTree not yet implemented".to_string(),
            )),
        }
    }

    /// Get information about the backend being used.
    pub fn backend_info(&self) -> &'static str {
        match &self.backend {
            KdTreeBackend::CWrapper(_) => "C Wrapper (Owned)",
            KdTreeBackend::UnsafeCWrapper(_) => "C Wrapper (Unsafe)",
            KdTreeBackend::PureRust => "Pure Rust",
        }
    }
}

/// Generic algorithms that work with any PointCloudTrait + KdTree combination.
pub mod algorithms {
    use super::*;
    use crate::traits::{Point4, PointCloudTrait};

    /// Find the closest point in the cloud to a query using trait-based access.
    ///
    /// This demonstrates how generic algorithms can combine trait-based point cloud
    /// access with efficient KdTree search.
    ///
    /// # Arguments
    /// * `cloud` - Any point cloud implementing PointCloudTrait
    /// * `kdtree` - A generic KdTree built from the same cloud
    /// * `query` - The query point as a 4D vector
    ///
    /// # Returns
    /// A tuple of (point_index, actual_point, squared_distance)
    pub fn find_closest_point_generic<P: PointCloudTrait>(
        cloud: &P,
        kdtree: &GenericKdTree<P>,
        query: Point4<f64>,
    ) -> Result<(usize, Point4<f64>, f64)> {
        let query_3d = helpers::point_to_vector3(query);
        let query_point3 = Point3::new(query_3d.x, query_3d.y, query_3d.z);

        let (index, sq_distance) = kdtree.nearest_neighbor(query_point3)?;
        let actual_point = cloud.point(index);

        Ok((index, actual_point, sq_distance))
    }

    /// Compute k-nearest neighbors and return both indices and actual points.
    ///
    /// # Arguments
    /// * `cloud` - Any point cloud implementing PointCloudTrait
    /// * `kdtree` - A generic KdTree built from the same cloud
    /// * `query` - The query point as a 4D vector
    /// * `k` - Number of neighbors to find
    ///
    /// # Returns
    /// A vector of tuples (point_index, actual_point, squared_distance)
    pub fn find_knn_points_generic<P: PointCloudTrait>(
        cloud: &P,
        kdtree: &GenericKdTree<P>,
        query: Point4<f64>,
        k: usize,
    ) -> Result<Vec<(usize, Point4<f64>, f64)>> {
        let query_3d = helpers::point_to_vector3(query);
        let query_point3 = Point3::new(query_3d.x, query_3d.y, query_3d.z);

        let knn_results = kdtree.knn_search(query_point3, k)?;

        let results = knn_results
            .into_iter()
            .map(|(index, sq_distance)| {
                let actual_point = cloud.point(index);
                (index, actual_point, sq_distance)
            })
            .collect();

        Ok(results)
    }

    /// Filter points by distance using generic trait + KdTree combination.
    ///
    /// This shows how generic algorithms can efficiently process large datasets
    /// by combining trait-based iteration with spatial search.
    ///
    /// # Arguments
    /// * `cloud` - Any point cloud implementing PointCloudTrait
    /// * `kdtree` - A generic KdTree built from the same cloud
    /// * `query_points` - Query points to search around
    /// * `max_distance` - Maximum distance threshold
    ///
    /// # Returns
    /// A vector of point indices that are within max_distance of any query point
    pub fn find_points_within_distance<P: PointCloudTrait>(
        cloud: &P,
        kdtree: &GenericKdTree<P>,
        query_points: &[Point4<f64>],
        max_distance: f64,
    ) -> Result<Vec<usize>> {
        let mut result_indices = std::collections::HashSet::new();
        let max_neighbors = cloud.size().min(1000); // Reasonable limit

        for query in query_points {
            let query_3d = helpers::point_to_vector3(*query);
            let query_point3 = Point3::new(query_3d.x, query_3d.y, query_3d.z);

            let nearby_points = kdtree.radius_search(query_point3, max_distance, max_neighbors)?;

            for (index, _) in nearby_points {
                result_indices.insert(index);
            }
        }

        Ok(result_indices.into_iter().collect())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        config::KdTreeConfig,
        point_cloud::PointCloud,
        traits::{helpers, MutablePointCloudTrait, Point4, PointCloudTrait},
    };
    use nalgebra::{Point3, Vector4};

    // Test with C wrapper PointCloud
    #[test]
    fn test_generic_kdtree_with_c_wrapper() -> Result<()> {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ];
        let cloud = PointCloud::from_points(&points)?;

        let config = KdTreeConfig::default();
        let kdtree = GenericKdTree::new(&cloud, &config)?;

        // Test nearest neighbor
        let query = Point3::new(0.1, 0.1, 0.0);
        let (index, sq_distance) = kdtree.nearest_neighbor(query)?;

        assert_eq!(index, 0);
        assert!(sq_distance < 0.1);

        // Test k-nearest neighbors
        let knn_results = kdtree.knn_search(query, 2)?;
        assert_eq!(knn_results.len(), 2);
        assert_eq!(knn_results[0].0, 0); // Closest should be first

        // Test radius search
        let radius_results = kdtree.radius_search(query, 1.5, 10)?;
        assert_eq!(radius_results.len(), 4); // All points within radius

        println!("Generic KdTree backend: {}", kdtree.backend_info());
        Ok(())
    }

    // Test with custom implementation
    #[derive(Debug)]
    struct TestPointCloud {
        points: Vec<Point4<f64>>,
    }

    impl TestPointCloud {
        fn new(points: Vec<Point3<f64>>) -> Self {
            let points_4d = points
                .into_iter()
                .map(|p| helpers::point_from_xyz(p.x, p.y, p.z))
                .collect();
            Self { points: points_4d }
        }
    }

    impl PointCloudTrait for TestPointCloud {
        fn size(&self) -> usize {
            self.points.len()
        }

        fn has_points(&self) -> bool {
            true
        }

        fn point(&self, index: usize) -> Point4<f64> {
            self.points[index]
        }
    }

    impl MutablePointCloudTrait for TestPointCloud {
        fn resize(&mut self, size: usize) {
            self.points.resize(size, Vector4::zeros());
        }

        fn set_point(&mut self, index: usize, point: Point4<f64>) {
            self.points[index] = point;
        }
    }

    #[test]
    fn test_generic_kdtree_with_custom_cloud() -> Result<()> {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
            Point3::new(2.0, 2.0, 0.0),
        ];
        let cloud = TestPointCloud::new(points);

        let config = KdTreeConfig::default();
        let kdtree = GenericKdTree::new(&cloud, &config)?;

        // Test nearest neighbor
        let query = Point3::new(0.1, 0.1, 0.0);
        let (index, sq_distance) = kdtree.nearest_neighbor(query)?;

        assert_eq!(index, 0);
        assert!(sq_distance < 0.1);

        println!("Custom cloud KdTree backend: {}", kdtree.backend_info());
        Ok(())
    }

    #[test]
    fn test_generic_algorithms() -> Result<()> {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let cloud = PointCloud::from_points(&points)?;
        let kdtree = GenericKdTree::new(&cloud, &KdTreeConfig::default())?;

        // Test closest point algorithm
        let query = helpers::point_from_xyz(0.1, 0.1, 0.0);
        let (index, actual_point, sq_distance) =
            algorithms::find_closest_point_generic(&cloud, &kdtree, query)?;

        assert_eq!(index, 0);
        assert!((actual_point.x - 0.0).abs() < f64::EPSILON);
        assert!(sq_distance < 0.1);

        // Test KNN algorithm
        let knn_results = algorithms::find_knn_points_generic(&cloud, &kdtree, query, 2)?;
        assert_eq!(knn_results.len(), 2);
        assert_eq!(knn_results[0].0, 0);

        // Test distance filtering
        let query_points = vec![helpers::point_from_xyz(0.5, 0.5, 0.0)];
        let nearby_indices =
            algorithms::find_points_within_distance(&cloud, &kdtree, &query_points, 1.0)?;
        assert!(!nearby_indices.is_empty());

        Ok(())
    }

    #[test]
    fn test_strategy_selection() -> Result<()> {
        let points = vec![Point3::new(1.0, 2.0, 3.0)];
        let cloud = PointCloud::from_points(&points)?;
        let config = KdTreeConfig::default();

        // Test explicit C wrapper strategy
        let kdtree_c = GenericKdTree::new_with_strategy(&cloud, &config, KdTreeStrategy::CWrapper)?;
        assert!(kdtree_c.backend_info().contains("C Wrapper"));

        // Test adaptive strategy
        let kdtree_adaptive =
            GenericKdTree::new_with_strategy(&cloud, &config, KdTreeStrategy::Adaptive)?;
        assert!(kdtree_adaptive.backend_info().contains("C Wrapper"));

        // Test unsupported strategy
        let result = GenericKdTree::new_with_strategy(&cloud, &config, KdTreeStrategy::PureRust);
        assert!(result.is_err());

        Ok(())
    }

    #[test]
    fn test_empty_cloud_error() {
        let cloud = TestPointCloud::new(vec![]);
        let config = KdTreeConfig::default();

        let result = GenericKdTree::new(&cloud, &config);
        assert!(result.is_err());
        assert!(matches!(
            result.unwrap_err(),
            SmallGicpError::EmptyPointCloud
        ));
    }
}
