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

    /// Set multiple points at once for better performance
    /// Data should be in format [x1, y1, z1, 1.0, x2, y2, z2, 1.0, ...]
    pub fn set_points_bulk(&mut self, points: &[f64]) {
        if points.len() % 4 != 0 {
            panic!("Points data must be a multiple of 4 (x, y, z, w)");
        }
        self.inner.pin_mut().set_points_bulk(points);
    }

    /// Set multiple normals at once for better performance
    /// Data should be in format [nx1, ny1, nz1, 0.0, nx2, ny2, nz2, 0.0, ...]
    pub fn set_normals_bulk(&mut self, normals: &[f64]) {
        if normals.len() % 4 != 0 {
            panic!("Normals data must be a multiple of 4 (nx, ny, nz, 0)");
        }
        self.inner.pin_mut().set_normals_bulk(normals);
    }

    /// Set multiple covariances at once for better performance
    /// Data should be in format [c11, c12, c13, c14, c21, ...] (row-major 4x4 matrices)
    pub fn set_covariances_bulk(&mut self, covariances: &[f64]) {
        if covariances.len() % 16 != 0 {
            panic!("Covariances data must be a multiple of 16 (4x4 matrix)");
        }
        self.inner.pin_mut().set_covariances_bulk(covariances);
    }

    /// Apply a rigid transformation to all points in the cloud (in-place)
    pub fn transform(&mut self, transform: &crate::ffi::Transform) {
        self.inner.pin_mut().transform(transform);
    }

    /// Create a new transformed point cloud without modifying the original
    pub fn transformed(&self, transform: &crate::ffi::Transform) -> Self {
        PointCloud {
            inner: self.inner.transformed(transform),
        }
    }

    /// Create from bulk point data (more efficient than individual add_point calls)
    /// Data should be in format [x1, y1, z1, 1.0, x2, y2, z2, 1.0, ...]
    pub fn from_points_bulk(points: &[f64]) -> Self {
        let mut cloud = Self::new();
        cloud.set_points_bulk(points);
        cloud
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

    /// Create from FFI handle (for internal use by preprocessing functions)
    pub(crate) fn from_ffi(inner: UniquePtr<FfiPointCloud>) -> Self {
        PointCloud { inner }
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

    #[test]
    fn test_bulk_operations() {
        let mut cloud = PointCloud::new();

        // Test bulk point setting
        let points = vec![
            1.0, 2.0, 3.0, 1.0, // Point 1
            4.0, 5.0, 6.0, 1.0, // Point 2
        ];
        cloud.set_points_bulk(&points);
        assert_eq!(cloud.len(), 2);

        assert_eq!(cloud.get_point(0).unwrap(), (1.0, 2.0, 3.0));
        assert_eq!(cloud.get_point(1).unwrap(), (4.0, 5.0, 6.0));

        // Test bulk normal setting
        let normals = vec![
            0.0, 0.0, 1.0, 0.0, // Normal 1
            1.0, 0.0, 0.0, 0.0, // Normal 2
        ];
        cloud.set_normals_bulk(&normals);

        // Test creating from bulk data
        let cloud2 = PointCloud::from_points_bulk(&points);
        assert_eq!(cloud2.len(), 2);
        assert_eq!(cloud2.get_point(0).unwrap(), (1.0, 2.0, 3.0));
    }

    #[test]
    fn test_transformation() {
        let mut cloud = PointCloud::new();
        cloud.add_point(1.0, 0.0, 0.0);
        cloud.add_point(0.0, 1.0, 0.0);

        // Test translation
        let translation = crate::Transform::translation(1.0, 2.0, 3.0);
        let transformed = cloud.transformed(&translation);

        assert_eq!(transformed.get_point(0).unwrap(), (2.0, 2.0, 3.0));
        assert_eq!(transformed.get_point(1).unwrap(), (1.0, 3.0, 3.0));

        // Original should be unchanged
        assert_eq!(cloud.get_point(0).unwrap(), (1.0, 0.0, 0.0));

        // Test in-place transformation
        cloud.transform(&translation);
        assert_eq!(cloud.get_point(0).unwrap(), (2.0, 2.0, 3.0));
    }

    #[test]
    #[should_panic(expected = "Points data must be a multiple of 4")]
    fn test_bulk_operations_invalid_size() {
        let mut cloud = PointCloud::new();
        let invalid_points = vec![1.0, 2.0, 3.0]; // Missing 4th component
        cloud.set_points_bulk(&invalid_points);
    }
}
