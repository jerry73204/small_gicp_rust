use crate::ffi::{
    ffi::{create_point_cloud, PointCloud as FfiPointCloud},
    Point3d,
};
use cxx::UniquePtr;
use std::pin::Pin;

/// A 3D point cloud with optional normals and covariances
pub struct PointCloud {
    inner: UniquePtr<FfiPointCloud>,
}

impl PointCloud {
    /// Create a new empty point cloud
    pub fn new() -> Self {
        PointCloud {
            inner: create_point_cloud(),
        }
    }

    /// Get the number of points in the cloud
    pub fn len(&self) -> usize {
        self.inner.size()
    }

    /// Check if the point cloud is empty
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Reserve capacity for n points
    pub fn reserve(&mut self, n: usize) {
        self.inner.pin_mut().reserve(n);
    }

    /// Resize the point cloud to n points
    pub fn resize(&mut self, n: usize) {
        self.inner.pin_mut().resize(n);
    }

    /// Clear all points from the cloud
    pub fn clear(&mut self) {
        self.inner.pin_mut().clear();
    }

    /// Add a point to the cloud
    pub fn add_point(&mut self, x: f64, y: f64, z: f64) {
        self.inner.pin_mut().add_point(Point3d { x, y, z });
    }

    /// Set a point at the given index
    pub fn set_point(&mut self, index: usize, x: f64, y: f64, z: f64) {
        self.inner.pin_mut().set_point(index, Point3d { x, y, z });
    }

    /// Get a point at the given index
    pub fn get_point(&self, index: usize) -> Option<(f64, f64, f64)> {
        if index >= self.len() {
            return None;
        }
        let point = self.inner.get_point(index);
        Some((point.x, point.y, point.z))
    }

    /// Get raw point data as a slice of f64 (x, y, z, 1.0) for each point
    pub fn points_data(&self) -> &[f64] {
        self.inner.points_data()
    }

    /// Get raw normal data as a slice of f64 (nx, ny, nz, 0.0) for each normal
    pub fn normals_data(&self) -> &[f64] {
        self.inner.normals_data()
    }

    /// Get raw covariance data as a slice of f64 (16 values per 4x4 matrix)
    pub fn covs_data(&self) -> &[f64] {
        self.inner.covs_data()
    }

    /// Estimate normals for all points
    pub fn estimate_normals(&mut self, num_neighbors: i32, num_threads: i32) {
        self.inner
            .pin_mut()
            .estimate_normals(num_neighbors, num_threads);
    }

    /// Estimate covariances for all points
    pub fn estimate_covariances(&mut self, num_neighbors: i32, num_threads: i32) {
        self.inner
            .pin_mut()
            .estimate_covariances(num_neighbors, num_threads);
    }

    /// Downsample the point cloud using voxel grid filtering
    pub fn voxel_downsample(&self, voxel_size: f64, num_threads: i32) -> Self {
        PointCloud {
            inner: self.inner.voxel_downsample(voxel_size, num_threads),
        }
    }

    /// Create from a slice of points (x, y, z)
    pub fn from_points(points: &[(f64, f64, f64)]) -> Self {
        let mut cloud = Self::new();
        cloud.reserve(points.len());
        for &(x, y, z) in points {
            cloud.add_point(x, y, z);
        }
        cloud
    }

    /// Get internal FFI handle
    pub(crate) fn as_ffi(&self) -> &FfiPointCloud {
        &self.inner
    }

    #[allow(dead_code)]
    pub(crate) fn as_ffi_mut(&mut self) -> Pin<&mut FfiPointCloud> {
        self.inner.pin_mut()
    }
}

impl Default for PointCloud {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point_cloud_basic() {
        let mut cloud = PointCloud::new();
        assert_eq!(cloud.len(), 0);
        assert!(cloud.is_empty());

        cloud.add_point(1.0, 2.0, 3.0);
        assert_eq!(cloud.len(), 1);
        assert!(!cloud.is_empty());

        let point = cloud.get_point(0).unwrap();
        assert_eq!(point, (1.0, 2.0, 3.0));

        cloud.clear();
        assert!(cloud.is_empty());
    }
}
