//! KdTree spatial indexing for efficient nearest neighbor search.
//!
//! This module provides two KdTree implementations:
//!
//! - [`KdTree`]: Owns a copy of the point cloud data internally
//! - [`BorrowedKdTree`]: Zero-copy reference to an existing [`PointCloud`]
//!
//! # Choosing the Right Type
//!
//! Use [`KdTree`] when:
//! - You need the tree to outlive the source point cloud
//! - You want a simple, self-contained tree structure
//! - The point cloud might be modified after tree construction
//!
//! Use [`BorrowedKdTree`] when:
//! - You want zero-copy operations for better performance
//! - The point cloud will outlive the tree
//! - You need to avoid memory duplication
//!
//! Both types work exclusively with the built-in [`PointCloud`] type.
//! Users with custom point cloud types should convert to [`PointCloud`] first.
//!
//! [`PointCloud`]: crate::point_cloud::PointCloud

use crate::{
    error::{Result, SmallGicpError},
    point_cloud::PointCloud,
    traits::SpatialSearchTree,
};
use nalgebra::{Point3, Vector3};
use std::marker::PhantomData;
use tracing::{debug, trace};

/// A KdTree for efficient nearest neighbor search in point clouds.
///
/// This type works exclusively with the built-in `PointCloud` type and
/// owns a copy of the data internally. For custom point cloud types or
/// zero-copy operations, use `BorrowedKdTree` instead.
///
/// # Examples
///
/// ```rust,no_run
/// use nalgebra::Point3;
/// use small_gicp::{KdTree, PointCloud};
///
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// let mut cloud = PointCloud::new()?;
/// cloud.add_point(0.0, 0.0, 0.0);
/// cloud.add_point(1.0, 0.0, 0.0);
///
/// let kdtree = KdTree::new(&cloud)?;
/// let query = Point3::new(0.5, 0.0, 0.0);
///
/// if let Some((index, distance)) = kdtree.nearest_neighbor(&query) {
///     println!(
///         "Nearest point index: {}, distance: {}",
///         index,
///         distance.sqrt()
///     );
/// }
/// # Ok(())
/// # }
/// ```
pub struct KdTree {
    inner: small_gicp_sys::KdTree,
}

impl KdTree {
    /// Build a KdTree from a point cloud.
    pub fn new(cloud: &PointCloud) -> Result<Self> {
        if cloud.is_empty() {
            return Err(SmallGicpError::EmptyPointCloud);
        }
        debug!("Building KdTree with 1 thread");
        let cxx_kdtree = small_gicp_sys::KdTree::build(cloud.inner(), 1);
        Ok(Self { inner: cxx_kdtree })
    }

    /// Build a KdTree from a point cloud with specified number of threads.
    pub fn new_parallel(cloud: &PointCloud, num_threads: usize) -> Result<Self> {
        if cloud.is_empty() {
            return Err(SmallGicpError::EmptyPointCloud);
        }
        debug!("Building KdTree with {} threads", num_threads);
        let cxx_kdtree = small_gicp_sys::KdTree::build(cloud.inner(), num_threads as i32);
        Ok(Self { inner: cxx_kdtree })
    }

    /// Find the nearest neighbor to a query point.
    pub fn nearest_neighbor(&self, point: &Point3<f64>) -> Option<(usize, f64)> {
        trace!(
            "Searching nearest neighbor for point ({}, {}, {})",
            point.x,
            point.y,
            point.z
        );
        let query = small_gicp_sys::Point3d {
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
        let query = small_gicp_sys::Point3d {
            x: point.x,
            y: point.y,
            z: point.z,
        };
        let result = self.inner.knn_search_with_distances(query, k);
        result
            .indices
            .into_iter()
            .zip(result.squared_distances)
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
        let query = small_gicp_sys::Point3d {
            x: point.x,
            y: point.y,
            z: point.z,
        };
        let result = self.inner.radius_search_with_distances(query, radius);
        result
            .indices
            .into_iter()
            .zip(result.squared_distances)
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

    /// Access the underlying small-gicp-sys KdTree.
    /// This is for internal use only and should not be exposed to users.
    pub(crate) fn inner(&self) -> &small_gicp_sys::KdTree {
        &self.inner
    }

    /// Convert from a small-gicp-sys KdTree.
    /// This is for internal use only and should not be exposed to users.
    pub(crate) fn from_cxx(inner: small_gicp_sys::KdTree) -> Self {
        Self { inner }
    }

    /// Convert to a small-gicp-sys KdTree.
    /// This is for internal use only and should not be exposed to users.
    pub(crate) fn into_cxx(self) -> small_gicp_sys::KdTree {
        self.inner
    }

    /// Get the number of points in the tree.
    pub fn size(&self) -> usize {
        self.inner.size()
    }

    /// Get backend information string.
    pub fn backend_info(&self) -> &'static str {
        "small-gicp-sys KdTree"
    }
}

impl std::fmt::Debug for KdTree {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("KdTree")
            .field("backend", &self.backend_info())
            .finish()
    }
}

impl SpatialSearchTree for KdTree {
    fn len(&self) -> usize {
        self.size()
    }

    fn nearest_neighbor(&self, query: &Vector3<f64>) -> Option<(usize, f64)> {
        let point = Point3::new(query.x, query.y, query.z);
        self.nearest_neighbor(&point)
    }

    fn knn_search(&self, query: &Vector3<f64>, k: usize) -> Vec<(usize, f64)> {
        let point = Point3::new(query.x, query.y, query.z);
        self.knn_search(&point, k)
    }

    fn radius_search(&self, query: &Vector3<f64>, radius: f64) -> Vec<(usize, f64)> {
        let point = Point3::new(query.x, query.y, query.z);
        self.radius_search(&point, radius)
    }
}

/// A zero-copy KdTree that borrows a PointCloud without copying.
///
/// This provides memory-efficient spatial search by constructing a KdTree
/// that holds a reference to an existing PointCloud, avoiding any data copying.
///
/// # Lifetime Management
///
/// The borrowed KdTree is tied to the lifetime of the original PointCloud,
/// ensuring memory safety while enabling zero-copy operations.
///
/// # Examples
///
/// ```rust,no_run
/// use nalgebra::Vector3;
/// use small_gicp::{BorrowedKdTree, PointCloud};
///
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// let mut cloud = PointCloud::new()?;
/// cloud.add_point(0.0, 0.0, 0.0);
/// cloud.add_point(1.0, 0.0, 0.0);
///
/// // Create a zero-copy KdTree
/// let borrowed_tree = BorrowedKdTree::new(&cloud)?;
///
/// // Perform search operations
/// let query = Vector3::new(0.5, 0.0, 0.0);
/// if let Some((index, distance)) = borrowed_tree.nearest_neighbor(&query) {
///     println!(
///         "Nearest neighbor: index={}, distance={}",
///         index,
///         distance.sqrt()
///     );
/// }
/// # Ok(())
/// # }
/// ```
pub struct BorrowedKdTree<'a> {
    inner: small_gicp_sys::UnsafeKdTree,
    _phantom: PhantomData<&'a PointCloud>,
}

impl<'a> BorrowedKdTree<'a> {
    /// Create a BorrowedKdTree from a PointCloud with single-threaded construction.
    ///
    /// This is a zero-copy operation that creates a KdTree holding a reference
    /// to the original PointCloud data.
    ///
    /// # Arguments
    ///
    /// * `cloud` - The PointCloud to borrow
    ///
    /// # Returns
    ///
    /// A new BorrowedKdTree that references the PointCloud data.
    pub fn new(cloud: &'a PointCloud) -> Result<Self> {
        Self::new_parallel(cloud, 1)
    }

    /// Create a BorrowedKdTree from a PointCloud with parallel construction.
    ///
    /// This is a zero-copy operation that creates a KdTree holding a reference
    /// to the original PointCloud data.
    ///
    /// # Arguments
    ///
    /// * `cloud` - The PointCloud to borrow
    /// * `num_threads` - Number of threads to use for construction
    ///
    /// # Returns
    ///
    /// A new BorrowedKdTree that references the PointCloud data.
    pub fn new_parallel(cloud: &'a PointCloud, num_threads: usize) -> Result<Self> {
        debug!("Building BorrowedKdTree with {} threads", num_threads);

        if cloud.is_empty() {
            return Err(SmallGicpError::EmptyPointCloud);
        }

        // Build the UnsafeKdTree using the C++ PointCloud reference
        let inner = small_gicp_sys::UnsafeKdTree::build(cloud.inner(), num_threads as i32);

        Ok(Self {
            inner,
            _phantom: PhantomData,
        })
    }

    /// Get the number of points in the tree.
    pub fn size(&self) -> usize {
        self.inner.size()
    }

    /// Check if the tree is empty.
    pub fn is_empty(&self) -> bool {
        self.size() == 0
    }

    /// Access the underlying UnsafeKdTree for internal use.
    pub(crate) fn inner(&self) -> &small_gicp_sys::UnsafeKdTree {
        &self.inner
    }
}

impl std::fmt::Debug for BorrowedKdTree<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("BorrowedKdTree")
            .field("size", &self.size())
            .finish()
    }
}

impl SpatialSearchTree for BorrowedKdTree<'_> {
    fn len(&self) -> usize {
        self.size()
    }

    fn nearest_neighbor(&self, query: &Vector3<f64>) -> Option<(usize, f64)> {
        trace!(
            "BorrowedKdTree nearest neighbor search for point ({}, {}, {})",
            query.x,
            query.y,
            query.z
        );

        let query_point = small_gicp_sys::Point3d {
            x: query.x,
            y: query.y,
            z: query.z,
        };

        let result = self.inner.nearest_neighbor_with_distance(query_point);
        if result.index == usize::MAX {
            None
        } else {
            Some((result.index, result.squared_distance))
        }
    }

    fn knn_search(&self, query: &Vector3<f64>, k: usize) -> Vec<(usize, f64)> {
        trace!(
            "BorrowedKdTree KNN search for point ({}, {}, {}) with k={}",
            query.x,
            query.y,
            query.z,
            k
        );

        let query_point = small_gicp_sys::Point3d {
            x: query.x,
            y: query.y,
            z: query.z,
        };

        let result = self.inner.knn_search_with_distances(query_point, k);
        result
            .indices
            .into_iter()
            .zip(result.squared_distances)
            .collect()
    }

    fn radius_search(&self, query: &Vector3<f64>, radius: f64) -> Vec<(usize, f64)> {
        trace!(
            "BorrowedKdTree radius search for point ({}, {}, {}) with radius={}",
            query.x,
            query.y,
            query.z,
            radius
        );

        let query_point = small_gicp_sys::Point3d {
            x: query.x,
            y: query.y,
            z: query.z,
        };

        let result = self.inner.radius_search_with_distances(query_point, radius);
        result
            .indices
            .into_iter()
            .zip(result.squared_distances)
            .collect()
    }
}

/// Algorithms for working with KdTrees and point clouds.
pub mod algorithms {
    use super::*;
    use crate::error::Result;

    /// Build correspondences between two point clouds.
    pub fn build_correspondences(
        source: &PointCloud,
        _target: &PointCloud,
        target_kdtree: &KdTree,
        max_distance: f64,
    ) -> Vec<(usize, usize, f64)> {
        debug!(
            "Building correspondences with max_distance={}",
            max_distance
        );
        let mut correspondences = Vec::new();
        let max_dist_sq = max_distance * max_distance;

        for i in 0..source.len() {
            let source_point = source.point_at(i).unwrap();
            if let Some((target_idx, sq_dist)) = target_kdtree.nearest_neighbor(&source_point) {
                if sq_dist <= max_dist_sq {
                    correspondences.push((i, target_idx, sq_dist.sqrt()));
                }
            }
        }

        correspondences
    }

    /// Find the closest point in a point cloud to a query point.
    pub fn find_closest_point(
        kdtree: &KdTree,
        cloud: &PointCloud,
        query_point: &Point3<f64>,
    ) -> Result<(usize, Point3<f64>, f64)> {
        debug!("Finding closest point");
        if let Some((index, sq_dist)) = kdtree.nearest_neighbor(query_point) {
            let point = cloud.point_at(index)?;
            Ok((index, point, sq_dist.sqrt()))
        } else {
            Err(SmallGicpError::IndexOutOfBounds(0, cloud.len()))
        }
    }

    /// Find k nearest points in a point cloud to a query point.
    pub fn find_knn_points(
        kdtree: &KdTree,
        cloud: &PointCloud,
        query_point: &Point3<f64>,
        k: usize,
    ) -> Result<Vec<(usize, Point3<f64>, f64)>> {
        debug!("Finding {} nearest points", k);
        let knn_results = kdtree.knn_search(query_point, k);
        let mut results = Vec::with_capacity(knn_results.len());

        for (index, sq_dist) in knn_results {
            let point = cloud.point_at(index)?;
            results.push((index, point, sq_dist.sqrt()));
        }

        Ok(results)
    }
}

#[cfg(test)]
mod tests;
