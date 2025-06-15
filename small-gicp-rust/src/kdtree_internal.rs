//! KdTree for efficient nearest neighbor search.

use crate::{
    config::{KdTreeConfig, KnnConfig},
    error::{Result, SmallGicpError},
    point_cloud::PointCloud,
};
use nalgebra::Point3;

/// C wrapper KdTree for efficient nearest neighbor search in point clouds.
///
/// This is the internal C wrapper implementation. Users should prefer the generic `KdTree<P>`
/// which provides a unified interface for all point cloud types.
pub(crate) struct CKdTree {
    pub(crate) handle: small_gicp_cxx::KdTree,
}

impl CKdTree {
    /// Create a new KdTree from a point cloud.
    ///
    /// # Arguments
    /// * `cloud` - The point cloud to build the tree from
    /// * `config` - KdTree configuration
    pub fn new(cloud: &PointCloud, config: &KdTreeConfig) -> Result<Self> {
        if cloud.is_empty() {
            return Err(SmallGicpError::EmptyPointCloud);
        }

        let handle = small_gicp_cxx::KdTree::build(cloud.inner(), config.num_threads as i32);

        Ok(CKdTree { handle })
    }

    /// Find the nearest neighbor to a query point.
    ///
    /// # Arguments
    /// * `query` - The query point
    ///
    /// # Returns
    /// A tuple of (point_index, squared_distance)
    pub fn nearest_neighbor(&self, query: Point3<f64>) -> Result<(usize, f64)> {
        if let Some(index) = self.handle.nearest_neighbor(query.x, query.y, query.z) {
            // TODO: Get the actual squared distance from the C++ side
            // For now, return 0.0 as a placeholder
            Ok((index, 0.0))
        } else {
            Err(SmallGicpError::InvalidArgument(
                "No nearest neighbor found".to_string(),
            ))
        }
    }

    /// Find the nearest neighbor to a query point with custom search settings.
    ///
    /// # Arguments
    /// * `query` - The query point
    /// * `settings` - KNN search settings for early termination
    ///
    /// # Returns
    /// A tuple of (point_index, squared_distance)
    pub fn nearest_neighbor_with_settings(
        &self,
        query: Point3<f64>,
        _settings: &KnnConfig,
    ) -> Result<(usize, f64)> {
        // For now, ignore settings and use the simple version
        self.nearest_neighbor(query)
    }

    /// Find k nearest neighbors to a query point.
    ///
    /// # Arguments
    /// * `query` - The query point
    /// * `k` - Number of neighbors to find
    ///
    /// # Returns
    /// A vector of tuples (point_index, squared_distance) sorted by distance
    pub fn knn_search(&self, query: Point3<f64>, k: usize) -> Result<Vec<(usize, f64)>> {
        if k == 0 {
            return Ok(Vec::new());
        }

        let indices = self.handle.knn_search(query.x, query.y, query.z, k);

        // TODO: Get the actual squared distances from the C++ side
        // For now, return indices with 0.0 distances as placeholders
        let results: Vec<(usize, f64)> = indices.into_iter().map(|idx| (idx, 0.0)).collect();

        Ok(results)
    }

    /// Find k nearest neighbors to a query point with custom search settings.
    ///
    /// # Arguments
    /// * `query` - The query point
    /// * `k` - Number of neighbors to find
    /// * `settings` - KNN search settings for early termination
    ///
    /// # Returns
    /// A vector of tuples (point_index, squared_distance) sorted by distance
    pub fn knn_search_with_settings(
        &self,
        query: Point3<f64>,
        k: usize,
        _settings: &KnnConfig,
    ) -> Result<Vec<(usize, f64)>> {
        // For now, ignore settings and use the simple version
        self.knn_search(query, k)
    }

    /// Find all neighbors within a given radius.
    ///
    /// # Arguments
    /// * `query` - The query point
    /// * `radius` - The search radius
    /// * `max_neighbors` - Maximum number of neighbors to return (ignored for now)
    ///
    /// # Returns
    /// A vector of tuples (point_index, squared_distance) within the radius
    pub fn radius_search(
        &self,
        query: Point3<f64>,
        radius: f64,
        _max_neighbors: usize,
    ) -> Result<Vec<(usize, f64)>> {
        let indices = self.handle.radius_search(query.x, query.y, query.z, radius);

        // TODO: Get the actual squared distances from the C++ side
        // For now, return indices with 0.0 distances as placeholders
        let results: Vec<(usize, f64)> = indices.into_iter().map(|idx| (idx, 0.0)).collect();

        Ok(results)
    }
}

// Ensure CKdTree is Send and Sync
unsafe impl Send for CKdTree {}
unsafe impl Sync for CKdTree {}

/// A high-performance KdTree that uses raw pointers for zero-copy operations.
///
/// # Safety
/// The caller must ensure that the point cloud used to create this tree
/// remains valid for the entire lifetime of the UnsafeKdTree.
pub struct UnsafeKdTree {
    pub(crate) handle: small_gicp_cxx::UnsafeKdTree,
}

impl UnsafeKdTree {
    /// Create a new UnsafeKdTree from a point cloud.
    ///
    /// # Arguments
    /// * `cloud` - The point cloud to build the tree from
    /// * `config` - KdTree configuration
    ///
    /// # Safety
    /// The caller must ensure that `cloud` remains valid for the entire lifetime
    /// of the UnsafeKdTree. The point cloud data is accessed via raw pointers.
    pub unsafe fn new(cloud: &PointCloud, config: &KdTreeConfig) -> Result<Self> {
        if cloud.is_empty() {
            return Err(SmallGicpError::EmptyPointCloud);
        }

        let handle = small_gicp_cxx::UnsafeKdTree::build(cloud.inner(), config.num_threads as i32);

        Ok(UnsafeKdTree { handle })
    }

    /// Create a new UnsafeKdTree with basic configuration.
    ///
    /// # Arguments
    /// * `cloud` - The point cloud to build the tree from
    ///
    /// # Safety
    /// The caller must ensure that `cloud` remains valid for the entire lifetime
    /// of the UnsafeKdTree.
    pub unsafe fn new_basic(cloud: &PointCloud) -> Result<Self> {
        if cloud.is_empty() {
            return Err(SmallGicpError::EmptyPointCloud);
        }

        let handle = small_gicp_cxx::UnsafeKdTree::build(cloud.inner(), 1);
        Ok(UnsafeKdTree { handle })
    }

    /// Find the nearest neighbor to a query point.
    ///
    /// # Arguments
    /// * `query` - The query point
    ///
    /// # Returns
    /// A tuple of (point_index, squared_distance)
    pub fn nearest_neighbor(&self, query: Point3<f64>) -> Result<(usize, f64)> {
        if let Some(index) = self.handle.nearest_neighbor(query.x, query.y, query.z) {
            // TODO: Get the actual squared distance from the C++ side
            // For now, return 0.0 as a placeholder
            Ok((index, 0.0))
        } else {
            Err(SmallGicpError::InvalidArgument(
                "No nearest neighbor found".to_string(),
            ))
        }
    }

    /// Find the nearest neighbor to a query point with custom search settings.
    ///
    /// # Arguments
    /// * `query` - The query point
    /// * `settings` - KNN search settings for early termination
    ///
    /// # Returns
    /// A tuple of (point_index, squared_distance)
    pub fn nearest_neighbor_with_settings(
        &self,
        query: Point3<f64>,
        _settings: &KnnConfig,
    ) -> Result<(usize, f64)> {
        // For now, ignore settings and use the simple version
        self.nearest_neighbor(query)
    }

    /// Find k nearest neighbors to a query point.
    ///
    /// # Arguments
    /// * `query` - The query point
    /// * `k` - Number of neighbors to find
    ///
    /// # Returns
    /// A vector of tuples (point_index, squared_distance) sorted by distance
    pub fn knn_search(&self, query: Point3<f64>, k: usize) -> Result<Vec<(usize, f64)>> {
        if k == 0 {
            return Ok(Vec::new());
        }

        let indices = self.handle.knn_search(query.x, query.y, query.z, k);

        // TODO: Get the actual squared distances from the C++ side
        // For now, return indices with 0.0 distances as placeholders
        let results: Vec<(usize, f64)> = indices.into_iter().map(|idx| (idx, 0.0)).collect();

        Ok(results)
    }

    /// Find k nearest neighbors to a query point with custom search settings.
    ///
    /// # Arguments
    /// * `query` - The query point
    /// * `k` - Number of neighbors to find
    /// * `settings` - KNN search settings for early termination
    ///
    /// # Returns
    /// A vector of tuples (point_index, squared_distance) sorted by distance
    pub fn knn_search_with_settings(
        &self,
        query: Point3<f64>,
        k: usize,
        _settings: &KnnConfig,
    ) -> Result<Vec<(usize, f64)>> {
        // For now, ignore settings and use the simple version
        self.knn_search(query, k)
    }

    /// Find all neighbors within a given radius.
    ///
    /// # Arguments
    /// * `query` - The query point
    /// * `radius` - The search radius
    /// * `max_neighbors` - Maximum number of neighbors to return (ignored for now)
    ///
    /// # Returns
    /// A vector of tuples (point_index, squared_distance) within the radius
    pub fn radius_search(
        &self,
        query: Point3<f64>,
        radius: f64,
        _max_neighbors: usize,
    ) -> Result<Vec<(usize, f64)>> {
        let indices = self.handle.radius_search(query.x, query.y, query.z, radius);

        // TODO: Get the actual squared distances from the C++ side
        // For now, return indices with 0.0 distances as placeholders
        let results: Vec<(usize, f64)> = indices.into_iter().map(|idx| (idx, 0.0)).collect();

        Ok(results)
    }
}

// Ensure UnsafeKdTree is Send and Sync (with appropriate safety caveats)
unsafe impl Send for UnsafeKdTree {}
unsafe impl Sync for UnsafeKdTree {}
