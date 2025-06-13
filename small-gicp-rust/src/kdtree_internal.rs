//! KdTree for efficient nearest neighbor search.

use crate::{
    config::{KdTreeBuilderType, KdTreeConfig, KnnConfig, ProjectionType},
    error::{check_error, Result, SmallGicpError},
    point_cloud::PointCloud,
};
use nalgebra::Point3;
use std::ptr;

/// C wrapper KdTree for efficient nearest neighbor search in point clouds.
///
/// This is the internal C wrapper implementation. Users should prefer the generic `KdTree<P>`
/// which provides a unified interface for all point cloud types.
#[derive(Debug)]
pub(crate) struct CKdTree {
    pub(crate) handle: *mut small_gicp_sys::small_gicp_kdtree_t,
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

        let mut handle = ptr::null_mut();

        // Convert enum values to C types
        let builder_type = match config.builder_type {
            KdTreeBuilderType::Default => 0, // SMALL_GICP_KDTREE_BUILDER_DEFAULT
            KdTreeBuilderType::OpenMp => 1,  // SMALL_GICP_KDTREE_BUILDER_OPENMP
            KdTreeBuilderType::Tbb => 2,     // SMALL_GICP_KDTREE_BUILDER_TBB
        };

        let projection_type = match config.projection.projection_type {
            ProjectionType::AxisAligned => 0, // SMALL_GICP_PROJECTION_AXIS_ALIGNED
            ProjectionType::Normal => 1,      // SMALL_GICP_PROJECTION_NORMAL
        };

        // Use the extended configuration if we have advanced features
        if config.builder_type != KdTreeBuilderType::Default
            || config.projection.projection_type != ProjectionType::AxisAligned
            || config.max_leaf_size != 20
        {
            // Create extended config
            let c_config = small_gicp_sys::small_gicp_kdtree_config_extended_t {
                builder_type,
                num_threads: config.num_threads as i32,
                max_leaf_size: config.max_leaf_size,
                projection: small_gicp_sys::small_gicp_projection_config_t {
                    type_: projection_type,
                    max_scan_count: config.projection.max_scan_count,
                },
            };

            let error = unsafe {
                small_gicp_sys::small_gicp_kdtree_create_with_extended_config(
                    cloud.handle,
                    &c_config,
                    &mut handle,
                )
            };
            check_error(error)?;
        } else {
            // Use simple configuration for basic cases
            let error = unsafe {
                small_gicp_sys::small_gicp_kdtree_create_with_builder(
                    cloud.handle,
                    builder_type,
                    config.num_threads as i32,
                    &mut handle,
                )
            };
            check_error(error)?;
        }

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
        settings: &KnnConfig,
    ) -> Result<(usize, f64)> {
        let mut index = 0;
        let mut sq_dist = 0.0;

        let c_settings = small_gicp_sys::small_gicp_knn_setting_t {
            epsilon: settings.epsilon,
        };

        let error = unsafe {
            small_gicp_sys::small_gicp_kdtree_nearest_neighbor_search_with_setting(
                self.handle,
                query.x,
                query.y,
                query.z,
                &c_settings,
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
        settings: &KnnConfig,
    ) -> Result<Vec<(usize, f64)>> {
        if k == 0 {
            return Ok(Vec::new());
        }

        let mut indices = vec![0usize; k];
        let mut sq_dists = vec![0.0f64; k];

        let c_settings = small_gicp_sys::small_gicp_knn_setting_t {
            epsilon: settings.epsilon,
        };

        let error = unsafe {
            small_gicp_sys::small_gicp_kdtree_knn_search_with_setting(
                self.handle,
                query.x,
                query.y,
                query.z,
                k as i32,
                &c_settings,
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

impl Drop for CKdTree {
    fn drop(&mut self) {
        if !self.handle.is_null() {
            unsafe {
                small_gicp_sys::small_gicp_kdtree_destroy(self.handle);
            }
        }
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
#[derive(Debug)]
pub struct UnsafeKdTree {
    pub(crate) handle: *mut small_gicp_sys::small_gicp_unsafe_kdtree_t,
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

        let mut handle = ptr::null_mut();

        // Convert enum values to C types
        let builder_type = match config.builder_type {
            KdTreeBuilderType::Default => 0, // SMALL_GICP_KDTREE_BUILDER_DEFAULT
            KdTreeBuilderType::OpenMp => 1,  // SMALL_GICP_KDTREE_BUILDER_OPENMP
            KdTreeBuilderType::Tbb => 2,     // SMALL_GICP_KDTREE_BUILDER_TBB
        };

        let projection_type = match config.projection.projection_type {
            ProjectionType::AxisAligned => 0, // SMALL_GICP_PROJECTION_AXIS_ALIGNED
            ProjectionType::Normal => 1,      // SMALL_GICP_PROJECTION_NORMAL
        };

        // Use the extended configuration if we have advanced features
        if config.builder_type != KdTreeBuilderType::Default
            || config.projection.projection_type != ProjectionType::AxisAligned
            || config.max_leaf_size != 20
        {
            // Create extended config
            let c_config = small_gicp_sys::small_gicp_kdtree_config_extended_t {
                builder_type,
                num_threads: config.num_threads as i32,
                max_leaf_size: config.max_leaf_size,
                projection: small_gicp_sys::small_gicp_projection_config_t {
                    type_: projection_type,
                    max_scan_count: config.projection.max_scan_count,
                },
            };

            let error = small_gicp_sys::small_gicp_unsafe_kdtree_create_with_extended_config(
                cloud.handle,
                &c_config,
                &mut handle,
            );
            check_error(error)?;
        } else {
            // Use simple configuration for basic cases
            let error = small_gicp_sys::small_gicp_unsafe_kdtree_create_with_config(
                cloud.handle,
                &small_gicp_sys::small_gicp_kdtree_config_t {
                    builder_type,
                    num_threads: config.num_threads as i32,
                    max_leaf_size: config.max_leaf_size,
                },
                &mut handle,
            );
            check_error(error)?;
        }

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

        let mut handle = ptr::null_mut();
        let error = small_gicp_sys::small_gicp_unsafe_kdtree_create(cloud.handle, &mut handle);
        check_error(error)?;
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
        let mut index = 0;
        let mut sq_dist = 0.0;

        let error = unsafe {
            small_gicp_sys::small_gicp_unsafe_kdtree_nearest_neighbor_search(
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
        settings: &KnnConfig,
    ) -> Result<(usize, f64)> {
        let mut index = 0;
        let mut sq_dist = 0.0;

        let c_settings = small_gicp_sys::small_gicp_knn_setting_t {
            epsilon: settings.epsilon,
        };

        let error = unsafe {
            small_gicp_sys::small_gicp_unsafe_kdtree_nearest_neighbor_search_with_setting(
                self.handle,
                query.x,
                query.y,
                query.z,
                &c_settings,
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
            small_gicp_sys::small_gicp_unsafe_kdtree_knn_search(
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
        settings: &KnnConfig,
    ) -> Result<Vec<(usize, f64)>> {
        if k == 0 {
            return Ok(Vec::new());
        }

        let mut indices = vec![0usize; k];
        let mut sq_dists = vec![0.0f64; k];

        let c_settings = small_gicp_sys::small_gicp_knn_setting_t {
            epsilon: settings.epsilon,
        };

        let error = unsafe {
            small_gicp_sys::small_gicp_unsafe_kdtree_knn_search_with_setting(
                self.handle,
                query.x,
                query.y,
                query.z,
                k as i32,
                &c_settings,
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

impl Drop for UnsafeKdTree {
    fn drop(&mut self) {
        if !self.handle.is_null() {
            unsafe {
                small_gicp_sys::small_gicp_unsafe_kdtree_destroy(self.handle);
            }
        }
    }
}

// Ensure UnsafeKdTree is Send and Sync (with appropriate safety caveats)
unsafe impl Send for UnsafeKdTree {}
unsafe impl Sync for UnsafeKdTree {}
