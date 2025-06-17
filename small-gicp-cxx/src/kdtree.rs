use crate::{
    ffi::{
        ffi::{create_kdtree, KdTree as FfiKdTree, KnnSearchResult, NearestNeighborResult},
        Point3d,
    },
    PointCloud,
};
use cxx::UniquePtr;

/// K-d tree for efficient nearest neighbor search
pub struct KdTree {
    inner: UniquePtr<FfiKdTree>,
}

// SAFETY: The underlying C++ KdTree is designed to be thread-safe for read operations.
// Multiple threads can safely perform searches simultaneously.
unsafe impl Send for KdTree {}
unsafe impl Sync for KdTree {}

impl KdTree {
    /// Build a K-d tree from a point cloud
    pub fn build(cloud: &PointCloud, num_threads: i32) -> Self {
        KdTree {
            inner: create_kdtree(cloud.as_ffi(), num_threads),
        }
    }

    /// Find the nearest neighbor to a query point
    pub fn nearest_neighbor(&self, x: f64, y: f64, z: f64) -> Option<usize> {
        let idx = self.inner.nearest_neighbor(Point3d { x, y, z });
        // Check if valid index (implementation should return SIZE_MAX for invalid)
        if idx == usize::MAX {
            None
        } else {
            Some(idx)
        }
    }

    /// Find k nearest neighbors to a query point
    pub fn knn_search(&self, x: f64, y: f64, z: f64, k: usize) -> Vec<usize> {
        self.inner.knn_search(Point3d { x, y, z }, k)
    }

    /// Find all neighbors within a radius
    pub fn radius_search(&self, x: f64, y: f64, z: f64, radius: f64) -> Vec<usize> {
        self.inner.radius_search(Point3d { x, y, z }, radius)
    }

    /// Find the nearest neighbor with distance
    pub fn nearest_neighbor_with_distance(&self, point: Point3d) -> NearestNeighborResult {
        self.inner.nearest_neighbor_with_distance(point)
    }

    /// Find k nearest neighbors with distances
    pub fn knn_search_with_distances(&self, point: Point3d, k: usize) -> KnnSearchResult {
        self.inner.knn_search_with_distances(point, k)
    }

    /// Find all neighbors within a radius with distances
    pub fn radius_search_with_distances(&self, point: Point3d, radius: f64) -> KnnSearchResult {
        self.inner.radius_search_with_distances(point, radius)
    }

    /// Get the number of points in the tree
    pub fn size(&self) -> usize {
        self.inner.size()
    }

    /// Get internal FFI handle
    pub(crate) fn as_ffi(&self) -> &FfiKdTree {
        &self.inner
    }
}

/// Builder for K-d tree construction with options
pub struct KdTreeBuilder<'a> {
    cloud: &'a PointCloud,
    num_threads: i32,
}

impl<'a> KdTreeBuilder<'a> {
    /// Create a new K-d tree builder
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

    /// Build the K-d tree
    pub fn build(self) -> KdTree {
        KdTree::build(self.cloud, self.num_threads)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kdtree_search() {
        let mut cloud = PointCloud::new();
        cloud.add_point(0.0, 0.0, 0.0);
        cloud.add_point(1.0, 0.0, 0.0);
        cloud.add_point(0.0, 1.0, 0.0);
        cloud.add_point(0.0, 0.0, 1.0);

        let tree = KdTree::build(&cloud, 1);

        // Test nearest neighbor - should find the closest point
        let nn = tree.nearest_neighbor(0.1, 0.1, 0.1);
        assert!(nn.is_some()); // Just check that we found something

        // Test k-nearest neighbors
        let knn = tree.knn_search(0.0, 0.0, 0.0, 3);
        assert_eq!(knn.len(), 3); // Should find 3 neighbors
        assert!(!knn.is_empty()); // Should have some results
    }
}
