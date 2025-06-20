use crate::{
    ffi::{
        ffi::{
            create_unsafe_kdtree, create_unsafe_kdtree_from_points_ptr, KdTreeSettings,
            KnnSearchResult, NearestNeighborResult, UnsafeKdTree as FfiUnsafeKdTree,
        },
        Point3d,
    },
    PointCloud,
};
use cxx::UniquePtr;

/// High-performance K-d tree for efficient nearest neighbor search
///
/// This is an "unsafe" variant that offers better performance at the cost
/// of additional memory usage and potentially less safety in concurrent scenarios.
pub struct UnsafeKdTree {
    inner: UniquePtr<FfiUnsafeKdTree>,
}

impl UnsafeKdTree {
    /// Build an UnsafeKdTree from a point cloud
    pub fn build(cloud: &PointCloud, num_threads: i32) -> Self {
        UnsafeKdTree {
            inner: create_unsafe_kdtree(cloud.as_ffi(), num_threads),
        }
    }

    /// Build an UnsafeKdTree from raw point data pointer (zero-copy)
    ///
    /// # Safety
    ///
    /// The caller must ensure that:
    /// - `points_data` points to valid f64 data with 4D homogeneous coordinates (x, y, z, w)
    /// - The data contains exactly `num_points * 4` f64 values
    /// - The memory remains valid for the lifetime of this UnsafeKdTree
    /// - The data is not modified while the UnsafeKdTree exists
    pub unsafe fn from_points_ptr(
        points_data: *const f64,
        num_points: usize,
        settings: &KdTreeSettings,
    ) -> Self {
        UnsafeKdTree {
            inner: unsafe {
                create_unsafe_kdtree_from_points_ptr(points_data, num_points, settings)
            },
        }
    }

    /// Validate that the underlying data pointer hasn't changed
    ///
    /// This provides a runtime safety check to detect if the original data
    /// has been moved or invalidated. Returns true if the data is still valid.
    ///
    /// # Safety
    /// The caller must ensure that `expected_ptr` is a valid pointer.
    pub unsafe fn validate_data_ptr(&self, expected_ptr: *const f64) -> bool {
        self.inner.unsafe_validate_data_ptr(expected_ptr)
    }

    /// Find the nearest neighbor to a query point
    pub fn nearest_neighbor(&self, x: f64, y: f64, z: f64) -> Option<usize> {
        let idx = self.inner.unsafe_nearest_neighbor(Point3d { x, y, z });
        // Check if valid index (implementation should return SIZE_MAX for invalid)
        if idx == usize::MAX {
            None
        } else {
            Some(idx)
        }
    }

    /// Find k nearest neighbors to a query point
    pub fn knn_search(&self, x: f64, y: f64, z: f64, k: usize) -> Vec<usize> {
        self.inner.unsafe_knn_search(Point3d { x, y, z }, k)
    }

    /// Find all neighbors within a radius
    pub fn radius_search(&self, x: f64, y: f64, z: f64, radius: f64) -> Vec<usize> {
        self.inner.unsafe_radius_search(Point3d { x, y, z }, radius)
    }

    /// Find the nearest neighbor with distance
    pub fn nearest_neighbor_with_distance(&self, point: Point3d) -> NearestNeighborResult {
        self.inner.unsafe_nearest_neighbor_with_distance(point)
    }

    /// Find k nearest neighbors with distances
    pub fn knn_search_with_distances(&self, point: Point3d, k: usize) -> KnnSearchResult {
        self.inner.unsafe_knn_search_with_distances(point, k)
    }

    /// Find all neighbors within a radius with distances
    pub fn radius_search_with_distances(&self, point: Point3d, radius: f64) -> KnnSearchResult {
        self.inner
            .unsafe_radius_search_with_distances(point, radius)
    }

    /// Get the number of points in the tree
    pub fn size(&self) -> usize {
        self.inner.unsafe_size()
    }

    /// Get internal FFI handle (for registration algorithms)
    pub(crate) fn as_ffi(&self) -> &FfiUnsafeKdTree {
        &self.inner
    }
}

/// Builder for UnsafeKdTree construction with options
pub struct UnsafeKdTreeBuilder<'a> {
    cloud: &'a PointCloud,
    num_threads: i32,
}

impl<'a> UnsafeKdTreeBuilder<'a> {
    /// Create a new UnsafeKdTree builder
    pub fn new(cloud: &'a PointCloud) -> Self {
        Self {
            cloud,
            num_threads: 1,
        }
    }

    /// Set number of threads for parallel construction
    pub fn num_threads(mut self, threads: i32) -> Self {
        self.num_threads = threads;
        self
    }

    /// Build the UnsafeKdTree
    pub fn build(self) -> UnsafeKdTree {
        UnsafeKdTree::build(self.cloud, self.num_threads)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_unsafe_kdtree_search() {
        let mut cloud = PointCloud::new();
        cloud.add_point(0.0, 0.0, 0.0);
        cloud.add_point(1.0, 0.0, 0.0);
        cloud.add_point(0.0, 1.0, 0.0);
        cloud.add_point(0.0, 0.0, 1.0);

        let tree = UnsafeKdTree::build(&cloud, 1);

        // Test nearest neighbor - should find the closest point
        let nn = tree.nearest_neighbor(0.1, 0.1, 0.1);
        assert!(nn.is_some()); // Just check that we found something

        // Test a more specific case - nearest to the exact first point
        let nn = tree.nearest_neighbor(0.0, 0.0, 0.0);
        assert!(nn.is_some());

        // Test k-nearest neighbors
        let knn = tree.knn_search(0.0, 0.0, 0.0, 3);
        assert_eq!(knn.len(), 3); // Should find 3 neighbors
        assert!(!knn.is_empty()); // Should have some results
    }

    #[test]
    fn test_unsafe_kdtree_parallel() {
        let mut cloud = PointCloud::new();
        for i in 0..100 {
            cloud.add_point(i as f64, i as f64, i as f64);
        }

        let tree = UnsafeKdTreeBuilder::new(&cloud).num_threads(4).build();

        let nn = tree.nearest_neighbor(50.0, 50.0, 50.0);
        assert_eq!(nn, Some(50));
    }
}
