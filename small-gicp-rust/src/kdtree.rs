//! KdTree spatial indexing for efficient nearest neighbor search.

use crate::{
    error::{Result, SmallGicpError},
    point_cloud::PointCloud,
    traits::PointCloudTrait,
};
use nalgebra::Point3;
use tracing::{debug, trace};

/// A KdTree for efficient nearest neighbor search in point clouds.
pub struct KdTree {
    // TODO: Wrap small_gicp_cxx::KdTree
    inner: small_gicp_cxx::KdTree,
}

impl KdTree {
    /// Build a KdTree from a point cloud.
    pub fn new(cloud: &PointCloud) -> Result<Self> {
        debug!("Building KdTree with 1 thread");
        let cxx_kdtree = small_gicp_cxx::KdTree::build(cloud.inner(), 1);
        Ok(Self { inner: cxx_kdtree })
    }

    /// Build a KdTree from a point cloud with specified number of threads.
    pub fn new_parallel(cloud: &PointCloud, num_threads: usize) -> Result<Self> {
        debug!("Building KdTree with {} threads", num_threads);
        let cxx_kdtree = small_gicp_cxx::KdTree::build(cloud.inner(), num_threads as i32);
        Ok(Self { inner: cxx_kdtree })
    }

    /// Build a KdTree from any point cloud that implements PointCloudTrait.
    pub fn from_trait<P: PointCloudTrait>(cloud: &P) -> Result<Self> {
        debug!("Building KdTree from trait implementation");
        // Convert to PointCloud first
        let mut pc = PointCloud::new()?;
        for i in 0..cloud.size() {
            let point = cloud.point(i);
            pc.add_point(point.x, point.y, point.z);
        }
        Self::new(&pc)
    }

    /// Find the nearest neighbor to a query point.
    pub fn nearest_neighbor(&self, point: &Point3<f64>) -> Option<(usize, f64)> {
        trace!(
            "Searching nearest neighbor for point ({}, {}, {})",
            point.x,
            point.y,
            point.z
        );
        let query = small_gicp_cxx::Point3d {
            x: point.x,
            y: point.y,
            z: point.z,
        };
        let result = self.inner.nearest_neighbor_with_distance(query);
        if result.index == usize::MAX {
            None
        } else {
            Some((result.index, result.squared_distance))
        }
    }

    /// Find k nearest neighbors to a query point.
    pub fn knn_search(&self, point: &Point3<f64>, k: usize) -> Vec<(usize, f64)> {
        trace!(
            "KNN search for point ({}, {}, {}) with k={}",
            point.x,
            point.y,
            point.z,
            k
        );
        let query = small_gicp_cxx::Point3d {
            x: point.x,
            y: point.y,
            z: point.z,
        };
        let result = self.inner.knn_search_with_distances(query, k);
        result
            .indices
            .into_iter()
            .zip(result.squared_distances.into_iter())
            .collect()
    }

    /// Find all neighbors within a given radius.
    pub fn radius_search(&self, point: &Point3<f64>, radius: f64) -> Vec<(usize, f64)> {
        trace!(
            "Radius search for point ({}, {}, {}) with radius={}",
            point.x,
            point.y,
            point.z,
            radius
        );
        let query = small_gicp_cxx::Point3d {
            x: point.x,
            y: point.y,
            z: point.z,
        };
        let result = self.inner.radius_search_with_distances(query, radius);
        result
            .indices
            .into_iter()
            .zip(result.squared_distances.into_iter())
            .collect()
    }

    /// Find the nearest neighbor with maximum distance constraint.
    pub fn nearest_neighbor_with_max_distance(
        &self,
        point: &Point3<f64>,
        max_distance: f64,
    ) -> Option<(usize, f64)> {
        debug!("Nearest neighbor search with max_distance={}", max_distance);
        let result = self.nearest_neighbor(point)?;
        if result.1.sqrt() <= max_distance {
            Some(result)
        } else {
            None
        }
    }

    /// Find the nearest neighbor with configuration settings.
    pub fn nearest_neighbor_with_settings(
        &self,
        point: &Point3<f64>,
        settings: &crate::config::KnnConfig,
    ) -> Option<(usize, f64)> {
        debug!("Nearest neighbor search with epsilon={}", settings.epsilon);
        // For now, epsilon is used as a max distance threshold
        if settings.epsilon > 0.0 {
            self.nearest_neighbor_with_max_distance(point, settings.epsilon)
        } else {
            self.nearest_neighbor(point)
        }
    }

    /// Access the underlying small-gicp-cxx KdTree.
    /// This is for internal use only and should not be exposed to users.
    pub(crate) fn inner(&self) -> &small_gicp_cxx::KdTree {
        &self.inner
    }

    /// Convert from a small-gicp-cxx KdTree.
    /// This is for internal use only and should not be exposed to users.
    pub(crate) fn from_cxx(inner: small_gicp_cxx::KdTree) -> Self {
        Self { inner }
    }

    /// Convert to a small-gicp-cxx KdTree.
    /// This is for internal use only and should not be exposed to users.
    pub(crate) fn into_cxx(self) -> small_gicp_cxx::KdTree {
        self.inner
    }

    /// Get backend information string.
    pub fn backend_info(&self) -> &'static str {
        "small-gicp-cxx KdTree"
    }
}

/// An unsafe KdTree variant for high-performance applications.
///
/// This provides better performance at the cost of some safety guarantees.
/// Use with caution in performance-critical applications.
pub struct UnsafeKdTree {
    // TODO: Wrap small_gicp_cxx::UnsafeKdTree
    inner: small_gicp_cxx::UnsafeKdTree,
}

impl UnsafeKdTree {
    /// Build an UnsafeKdTree from a point cloud.
    pub fn new(cloud: &PointCloud) -> Result<Self> {
        debug!("Building UnsafeKdTree with 1 thread");
        let cxx_kdtree = small_gicp_cxx::UnsafeKdTree::build(cloud.inner(), 1);
        Ok(Self { inner: cxx_kdtree })
    }

    /// Build an UnsafeKdTree from a point cloud with specified number of threads.
    pub fn new_parallel(cloud: &PointCloud, num_threads: usize) -> Result<Self> {
        debug!("Building UnsafeKdTree with {} threads", num_threads);
        let cxx_kdtree = small_gicp_cxx::UnsafeKdTree::build(cloud.inner(), num_threads as i32);
        Ok(Self { inner: cxx_kdtree })
    }

    /// Find the nearest neighbor to a query point.
    ///
    /// # Safety
    /// This method performs unsafe operations for better performance.
    /// The caller must ensure the point cloud used to build this tree is still valid.
    pub unsafe fn nearest_neighbor(&self, point: &Point3<f64>) -> Option<(usize, f64)> {
        trace!(
            "Unsafe nearest neighbor search for point ({}, {}, {})",
            point.x,
            point.y,
            point.z
        );
        let query = small_gicp_cxx::Point3d {
            x: point.x,
            y: point.y,
            z: point.z,
        };
        let result = self.inner.nearest_neighbor_with_distance(query);
        if result.index == usize::MAX {
            None
        } else {
            Some((result.index, result.squared_distance))
        }
    }

    /// Find k nearest neighbors to a query point.
    ///
    /// # Safety
    /// This method performs unsafe operations for better performance.
    /// The caller must ensure the point cloud used to build this tree is still valid.
    pub unsafe fn knn_search(&self, point: &Point3<f64>, k: usize) -> Vec<(usize, f64)> {
        trace!(
            "Unsafe KNN search for point ({}, {}, {}) with k={}",
            point.x,
            point.y,
            point.z,
            k
        );
        let query = small_gicp_cxx::Point3d {
            x: point.x,
            y: point.y,
            z: point.z,
        };
        let result = self.inner.knn_search_with_distances(query, k);
        result
            .indices
            .into_iter()
            .zip(result.squared_distances.into_iter())
            .collect()
    }

    /// Find all neighbors within a given radius.
    ///
    /// # Safety
    /// This method performs unsafe operations for better performance.
    /// The caller must ensure the point cloud used to build this tree is still valid.
    pub unsafe fn radius_search(&self, point: &Point3<f64>, radius: f64) -> Vec<(usize, f64)> {
        trace!(
            "Unsafe radius search for point ({}, {}, {}) with radius={}",
            point.x,
            point.y,
            point.z,
            radius
        );
        let query = small_gicp_cxx::Point3d {
            x: point.x,
            y: point.y,
            z: point.z,
        };
        let result = self.inner.radius_search_with_distances(query, radius);
        result
            .indices
            .into_iter()
            .zip(result.squared_distances.into_iter())
            .collect()
    }

    /// Access the underlying small-gicp-cxx UnsafeKdTree.
    /// This is for internal use only and should not be exposed to users.
    pub(crate) fn inner(&self) -> &small_gicp_cxx::UnsafeKdTree {
        &self.inner
    }

    /// Convert from a small-gicp-cxx UnsafeKdTree.
    /// This is for internal use only and should not be exposed to users.
    pub(crate) fn from_cxx(inner: small_gicp_cxx::UnsafeKdTree) -> Self {
        Self { inner }
    }

    /// Convert to a small-gicp-cxx UnsafeKdTree.
    /// This is for internal use only and should not be exposed to users.
    pub(crate) fn into_cxx(self) -> small_gicp_cxx::UnsafeKdTree {
        self.inner
    }
}

/// Strategy for KdTree construction and search.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KdTreeStrategy {
    /// Standard KdTree with safety guarantees.
    Safe,
    /// Unsafe KdTree for maximum performance.
    Unsafe,
}

/// Builder for configuring KdTree construction.
pub struct KdTreeBuilder {
    strategy: KdTreeStrategy,
    num_threads: usize,
}

impl KdTreeBuilder {
    /// Create a new KdTree builder with default settings.
    pub fn new() -> Self {
        Self {
            strategy: KdTreeStrategy::Safe,
            num_threads: 1,
        }
    }

    /// Set the KdTree strategy.
    pub fn strategy(mut self, strategy: KdTreeStrategy) -> Self {
        self.strategy = strategy;
        self
    }

    /// Set the number of threads for parallel construction.
    pub fn num_threads(mut self, num_threads: usize) -> Self {
        self.num_threads = num_threads;
        self
    }

    /// Build a KdTree from a point cloud.
    pub fn build(self, cloud: &PointCloud) -> Result<KdTree> {
        match self.strategy {
            KdTreeStrategy::Safe => {
                if self.num_threads == 1 {
                    KdTree::new(cloud)
                } else {
                    KdTree::new_parallel(cloud, self.num_threads)
                }
            }
            KdTreeStrategy::Unsafe => {
                // Build an UnsafeKdTree and wrap it in a safe interface
                debug!("Building KdTree using unsafe strategy");
                let unsafe_tree = if self.num_threads == 1 {
                    UnsafeKdTree::new(cloud)?
                } else {
                    UnsafeKdTree::new_parallel(cloud, self.num_threads)?
                };

                // For now, we convert to a regular KdTree since our API doesn't expose unsafe operations
                // In the future, we could have a wrapper that provides safe access to unsafe tree
                if self.num_threads == 1 {
                    KdTree::new(cloud)
                } else {
                    KdTree::new_parallel(cloud, self.num_threads)
                }
            }
        }
    }
}

impl Default for KdTreeBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    #[test]
    fn test_kdtree_builder() {
        let builder = KdTreeBuilder::new();
        assert_eq!(builder.strategy, KdTreeStrategy::Safe);
        assert_eq!(builder.num_threads, 1);

        let builder = builder.strategy(KdTreeStrategy::Unsafe).num_threads(4);
        assert_eq!(builder.strategy, KdTreeStrategy::Unsafe);
        assert_eq!(builder.num_threads, 4);
    }

    #[test]
    fn test_kdtree_strategy() {
        assert_eq!(KdTreeStrategy::Safe, KdTreeStrategy::Safe);
        assert_ne!(KdTreeStrategy::Safe, KdTreeStrategy::Unsafe);
    }

    // TODO: Add integration tests once implementation is complete
    // #[test]
    // fn test_kdtree_construction() {
    //     let points = vec![
    //         Point3::new(0.0, 0.0, 0.0),
    //         Point3::new(1.0, 0.0, 0.0),
    //         Point3::new(0.0, 1.0, 0.0),
    //         Point3::new(1.0, 1.0, 0.0),
    //     ];
    //     let cloud = PointCloud::from_points(&points).unwrap();
    //     let kdtree = KdTree::new(&cloud).unwrap();
    //
    //     let query = Point3::new(0.5, 0.5, 0.0);
    //     let result = kdtree.nearest_neighbor(&query);
    //     assert!(result.is_some());
    // }

    // #[test]
    // fn test_knn_search() {
    //     let points = vec![
    //         Point3::new(0.0, 0.0, 0.0),
    //         Point3::new(1.0, 0.0, 0.0),
    //         Point3::new(0.0, 1.0, 0.0),
    //         Point3::new(1.0, 1.0, 0.0),
    //     ];
    //     let cloud = PointCloud::from_points(&points).unwrap();
    //     let kdtree = KdTree::new(&cloud).unwrap();
    //
    //     let query = Point3::new(0.5, 0.5, 0.0);
    //     let results = kdtree.knn_search(&query, 2);
    //     assert_eq!(results.len(), 2);
    // }

    // #[test]
    // fn test_radius_search() {
    //     let points = vec![
    //         Point3::new(0.0, 0.0, 0.0),
    //         Point3::new(1.0, 0.0, 0.0),
    //         Point3::new(0.0, 1.0, 0.0),
    //         Point3::new(1.0, 1.0, 0.0),
    //     ];
    //     let cloud = PointCloud::from_points(&points).unwrap();
    //     let kdtree = KdTree::new(&cloud).unwrap();
    //
    //     let query = Point3::new(0.5, 0.5, 0.0);
    //     let results = kdtree.radius_search(&query, 1.0);
    //     assert!(results.len() > 0);
    // }
}

/// Generic algorithms for working with KdTrees and point clouds.
pub mod algorithms {
    use super::*;
    use crate::{error::Result, traits::PointCloudTrait};

    /// Generic nearest neighbor search function.
    pub fn nearest_neighbor<P: PointCloudTrait>(
        kdtree: &KdTree,
        cloud: &P,
        query_point: &Point3<f64>,
    ) -> Option<(usize, f64)> {
        debug!("Generic nearest neighbor search");
        kdtree.nearest_neighbor(query_point)
    }

    /// Generic k-nearest neighbor search function.
    pub fn knn_search<P: PointCloudTrait>(
        kdtree: &KdTree,
        cloud: &P,
        query_point: &Point3<f64>,
        k: usize,
    ) -> Vec<(usize, f64)> {
        debug!("Generic KNN search with k={}", k);
        kdtree.knn_search(query_point, k)
    }

    /// Generic radius search function.
    pub fn radius_search<P: PointCloudTrait>(
        kdtree: &KdTree,
        cloud: &P,
        query_point: &Point3<f64>,
        radius: f64,
    ) -> Vec<(usize, f64)> {
        debug!("Generic radius search with radius={}", radius);
        kdtree.radius_search(query_point, radius)
    }

    /// Build correspondences between two point clouds.
    pub fn build_correspondences<S: PointCloudTrait, T: PointCloudTrait>(
        source: &S,
        target: &T,
        target_kdtree: &KdTree,
        max_distance: f64,
    ) -> Vec<(usize, usize, f64)> {
        debug!(
            "Building correspondences with max_distance={}",
            max_distance
        );
        let mut correspondences = Vec::new();
        let max_dist_sq = max_distance * max_distance;

        for i in 0..source.size() {
            let source_point_4d = source.point(i);
            let source_point = Point3::new(source_point_4d.x, source_point_4d.y, source_point_4d.z);
            if let Some((target_idx, sq_dist)) = target_kdtree.nearest_neighbor(&source_point) {
                if sq_dist <= max_dist_sq {
                    correspondences.push((i, target_idx, sq_dist.sqrt()));
                }
            }
        }

        correspondences
    }

    /// Find the closest point in a point cloud to a query point.
    pub fn find_closest_point_generic<P: PointCloudTrait>(
        kdtree: &KdTree,
        cloud: &P,
        query_point: &Point3<f64>,
    ) -> Result<(usize, Point3<f64>, f64)> {
        debug!("Finding closest point");
        if let Some((index, sq_dist)) = kdtree.nearest_neighbor(query_point) {
            let point_4d = cloud.point(index);
            let point = Point3::new(point_4d.x, point_4d.y, point_4d.z);
            Ok((index, point, sq_dist.sqrt()))
        } else {
            Err(SmallGicpError::IndexOutOfBounds {
                index: 0,
                size: cloud.size(),
            })
        }
    }

    /// Find k nearest points in a point cloud to a query point.
    pub fn find_knn_points_generic<P: PointCloudTrait>(
        kdtree: &KdTree,
        cloud: &P,
        query_point: &Point3<f64>,
        k: usize,
    ) -> Result<Vec<(usize, Point3<f64>, f64)>> {
        debug!("Finding {} nearest points", k);
        let knn_results = kdtree.knn_search(query_point, k);
        let mut results = Vec::with_capacity(knn_results.len());

        for (index, sq_dist) in knn_results {
            let point_4d = cloud.point(index);
            let point = Point3::new(point_4d.x, point_4d.y, point_4d.z);
            results.push((index, point, sq_dist.sqrt()));
        }

        Ok(results)
    }
}
