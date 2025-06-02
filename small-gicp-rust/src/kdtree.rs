//! KdTree for efficient nearest neighbor search.

use crate::error::{check_error, Result, SmallGicpError};
use crate::point_cloud::PointCloud;
use nalgebra::Point3;
use std::ptr;

/// A KdTree for efficient nearest neighbor search in point clouds.
#[derive(Debug)]
pub struct KdTree {
    pub(crate) handle: *mut small_gicp_sys::small_gicp_kdtree_t,
}

impl KdTree {
    /// Create a new KdTree from a point cloud.
    ///
    /// # Arguments
    /// * `cloud` - The point cloud to build the tree from
    /// * `num_threads` - Number of threads to use for construction (1 for single-threaded)
    pub fn new(cloud: &PointCloud, num_threads: usize) -> Result<Self> {
        if cloud.is_empty() {
            return Err(SmallGicpError::EmptyPointCloud);
        }

        let mut handle = ptr::null_mut();
        let error = unsafe {
            small_gicp_sys::small_gicp_kdtree_create(cloud.handle, num_threads as i32, &mut handle)
        };
        check_error(error)?;
        Ok(KdTree { handle })
    }

    /// Find the nearest neighbor to a query point.
    ///
    /// # Arguments
    /// * `query` - The query point
    ///
    /// # Returns
    /// A tuple of (point_index, squared_distance)
    pub fn nearest_neighbor(&self, query: Point3<f64>) -> Result<(usize, f64)> {
        let mut index = 0;
        let mut sq_dist = 0.0;

        let error = unsafe {
            small_gicp_sys::small_gicp_kdtree_nearest_neighbor_search(
                self.handle,
                query.x,
                query.y,
                query.z,
                &mut index,
                &mut sq_dist,
            )
        };

        check_error(error)?;
        Ok((index, sq_dist))
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

        let mut indices = vec![0usize; k];
        let mut sq_dists = vec![0.0f64; k];

        let error = unsafe {
            small_gicp_sys::small_gicp_kdtree_knn_search(
                self.handle,
                query.x,
                query.y,
                query.z,
                k as i32,
                indices.as_mut_ptr(),
                sq_dists.as_mut_ptr(),
            )
        };

        check_error(error)?;

        // Combine indices and distances, then sort by distance
        let mut results: Vec<(usize, f64)> =
            indices.into_iter().zip(sq_dists.into_iter()).collect();

        // Results should already be sorted, but ensure it
        results.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        Ok(results)
    }

    /// Find all neighbors within a given radius.
    ///
    /// # Arguments
    /// * `query` - The query point
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
        // Use knn_search and filter by radius
        let results = self.knn_search(query, max_neighbors)?;
        let radius_sq = radius * radius;

        Ok(results
            .into_iter()
            .take_while(|(_, sq_dist)| *sq_dist <= radius_sq)
            .collect())
    }
}

impl Drop for KdTree {
    fn drop(&mut self) {
        if !self.handle.is_null() {
            unsafe {
                small_gicp_sys::small_gicp_kdtree_destroy(self.handle);
            }
        }
    }
}

// Ensure KdTree is Send and Sync
unsafe impl Send for KdTree {}
unsafe impl Sync for KdTree {}
