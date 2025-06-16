//! Generic traits for point cloud operations.
//!
//! This module provides trait-based abstractions that enable generic programming
//! over different point cloud types, similar to the C++ template-based design
//! of small_gicp.
//!
//! # Design Overview
//!
//! The trait hierarchy mirrors the C++ traits system:
//! - `PointCloudTrait` - Read-only interface for any point cloud type
//! - `MutablePointCloudTrait` - Extends with mutation capabilities
//!
//! All traits use 4D vectors (x,y,z,1) for points and (nx,ny,nz,0) for normals
//! to match the C++ SIMD optimization design.

use nalgebra::{Matrix4, Vector4};
use std::fmt::Debug;

/// Type alias for 4D point vectors (x, y, z, 1.0).
///
/// The fourth component is always 1.0 for homogeneous coordinates,
/// matching the C++ library's SIMD optimization design.
pub type Point4<T> = Vector4<T>;

/// Type alias for 4D normal vectors (nx, ny, nz, 0.0).
///
/// The fourth component is always 0.0 for direction vectors,
/// matching the C++ library's SIMD optimization design.
pub type Normal4<T> = Vector4<T>;

/// Type alias for 4x4 covariance matrices.
pub type Covariance4<T> = Matrix4<T>;

/// Read-only trait for point cloud access.
///
/// This trait provides a unified interface for accessing point cloud data,
/// regardless of the underlying storage type. It matches the C++ traits
/// system for compile-time polymorphism.
///
/// # Type Requirements
///
/// - Points are represented as 4D vectors (x, y, z, 1.0)
/// - Normals are represented as 4D vectors (nx, ny, nz, 0.0)  
/// - Covariances are represented as 4x4 matrices
///
/// # Examples
///
/// ```rust
/// use nalgebra::Vector4;
/// use small_gicp::traits::PointCloudTrait;
///
/// fn count_points_with_normals<P: PointCloudTrait>(cloud: &P) -> usize {
///     if cloud.has_normals() {
///         cloud.size()
///     } else {
///         0
///     }
/// }
/// ```
pub trait PointCloudTrait: Debug + Send + Sync {
    /// Returns the number of points in the cloud.
    fn size(&self) -> usize;

    /// Returns true if the cloud is empty.
    fn empty(&self) -> bool {
        self.size() == 0
    }

    /// Returns true if the cloud has point data.
    ///
    /// This should always return true for valid point clouds.
    fn has_points(&self) -> bool;

    /// Returns true if the cloud has normal vectors.
    fn has_normals(&self) -> bool {
        false
    }

    /// Returns true if the cloud has covariance matrices.
    fn has_covariances(&self) -> bool {
        false
    }

    /// Returns the point at the given index.
    ///
    /// # Panics
    ///
    /// Panics if the index is out of bounds.
    fn point(&self, index: usize) -> Point4<f64>;

    /// Returns the normal at the given index, if available.
    ///
    /// Returns `None` if normals are not available or index is out of bounds.
    fn normal(&self, index: usize) -> Option<Normal4<f64>> {
        let _ = index;
        None
    }

    /// Returns the covariance matrix at the given index, if available.
    ///
    /// Returns `None` if covariances are not available or index is out of bounds.
    fn covariance(&self, index: usize) -> Option<Covariance4<f64>> {
        let _ = index;
        None
    }

    /// Returns the point at the given index without bounds checking.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `index < self.size()`.
    unsafe fn point_unchecked(&self, index: usize) -> Point4<f64> {
        self.point(index)
    }

    /// Returns the normal at the given index without bounds checking.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `index < self.size()` and `self.has_normals()`.
    unsafe fn normal_unchecked(&self, index: usize) -> Normal4<f64> {
        self.normal(index).unwrap_or_else(|| Vector4::zeros())
    }

    /// Returns the covariance at the given index without bounds checking.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `index < self.size()` and `self.has_covariances()`.
    unsafe fn covariance_unchecked(&self, index: usize) -> Covariance4<f64> {
        self.covariance(index).unwrap_or_else(|| Matrix4::zeros())
    }
}

/// Mutable trait for point cloud modification.
///
/// This trait extends `PointCloudTrait` with methods for modifying point cloud data.
/// It enables generic algorithms that need to modify point clouds.
///
/// # Examples
///
/// ```rust
/// use nalgebra::Vector4;
/// use small_gicp::traits::MutablePointCloudTrait;
///
/// fn translate_points<P: MutablePointCloudTrait>(cloud: &mut P, offset: Vector4<f64>) {
///     for i in 0..cloud.size() {
///         let mut point = cloud.point(i);
///         point.x += offset.x;
///         point.y += offset.y;
///         point.z += offset.z;
///         cloud.set_point(i, point);
///     }
/// }
/// ```
pub trait MutablePointCloudTrait: PointCloudTrait {
    /// Resizes the point cloud to contain `size` points.
    ///
    /// If the new size is larger, new points are initialized to zero.
    /// If the new size is smaller, excess points are removed.
    fn resize(&mut self, size: usize);

    /// Sets the point at the given index.
    ///
    /// # Panics
    ///
    /// Panics if the index is out of bounds.
    fn set_point(&mut self, index: usize, point: Point4<f64>);

    /// Sets the normal at the given index.
    ///
    /// This is a no-op if the cloud doesn't support normals.
    ///
    /// # Panics
    ///
    /// Panics if the index is out of bounds.
    fn set_normal(&mut self, index: usize, normal: Normal4<f64>) {
        let _ = (index, normal);
    }

    /// Sets the covariance matrix at the given index.
    ///
    /// This is a no-op if the cloud doesn't support covariances.
    ///
    /// # Panics
    ///
    /// Panics if the index is out of bounds.
    fn set_covariance(&mut self, index: usize, covariance: Covariance4<f64>) {
        let _ = (index, covariance);
    }

    /// Sets the point at the given index without bounds checking.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `index < self.size()`.
    unsafe fn set_point_unchecked(&mut self, index: usize, point: Point4<f64>) {
        self.set_point(index, point);
    }

    /// Sets the normal at the given index without bounds checking.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `index < self.size()` and `self.has_normals()`.
    unsafe fn set_normal_unchecked(&mut self, index: usize, normal: Normal4<f64>) {
        self.set_normal(index, normal);
    }

    /// Sets the covariance at the given index without bounds checking.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `index < self.size()` and `self.has_covariances()`.
    unsafe fn set_covariance_unchecked(&mut self, index: usize, covariance: Covariance4<f64>) {
        self.set_covariance(index, covariance);
    }
}

/// Helper functions for creating 4D vectors with correct homogeneous coordinates.
pub mod helpers {
    use super::{Normal4, Point4};
    use nalgebra::Vector3;

    /// Creates a 4D point vector from 3D coordinates.
    ///
    /// The fourth component is set to 1.0 for homogeneous coordinates.
    pub fn point_from_xyz(x: f64, y: f64, z: f64) -> Point4<f64> {
        Point4::new(x, y, z, 1.0)
    }

    /// Creates a 4D point vector from a 3D vector.
    ///
    /// The fourth component is set to 1.0 for homogeneous coordinates.
    pub fn point_from_vector3(v: Vector3<f64>) -> Point4<f64> {
        Point4::new(v.x, v.y, v.z, 1.0)
    }

    /// Creates a 4D normal vector from 3D coordinates.
    ///
    /// The fourth component is set to 0.0 for direction vectors.
    pub fn normal_from_xyz(nx: f64, ny: f64, nz: f64) -> Normal4<f64> {
        Normal4::new(nx, ny, nz, 0.0)
    }

    /// Creates a 4D normal vector from a 3D vector.
    ///
    /// The fourth component is set to 0.0 for direction vectors.
    pub fn normal_from_vector3(v: Vector3<f64>) -> Normal4<f64> {
        Normal4::new(v.x, v.y, v.z, 0.0)
    }

    /// Extracts 3D coordinates from a 4D point vector.
    pub fn point_to_vector3(p: Point4<f64>) -> Vector3<f64> {
        Vector3::new(p.x, p.y, p.z)
    }

    /// Extracts 3D coordinates from a 4D normal vector.
    pub fn normal_to_vector3(n: Normal4<f64>) -> Vector3<f64> {
        Vector3::new(n.x, n.y, n.z)
    }

    /// Validates that a 4D point has the correct homogeneous coordinate.
    pub fn is_valid_point(p: Point4<f64>) -> bool {
        (p.w - 1.0).abs() < f64::EPSILON
    }

    /// Validates that a 4D normal has the correct homogeneous coordinate.
    pub fn is_valid_normal(n: Normal4<f64>) -> bool {
        n.w.abs() < f64::EPSILON
    }
}

/// Bounds checking helpers for safe trait implementations.
pub mod bounds {
    /// Checks if an index is within bounds and panics with a descriptive message if not.
    pub fn check_bounds(index: usize, size: usize, operation: &str) {
        if index >= size {
            panic!(
                "Index {} out of bounds for {} operation on point cloud of size {}",
                index, operation, size
            );
        }
    }

    /// Checks if an index is within bounds and returns a Result.
    pub fn check_bounds_result(index: usize, size: usize, operation: &str) -> Result<(), String> {
        if index >= size {
            Err(format!(
                "Index {} out of bounds for {} operation on point cloud of size {}",
                index, operation, size
            ))
        } else {
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Matrix4, Vector3, Vector4};

    #[test]
    fn test_helper_functions() {
        // Test point creation
        let p = helpers::point_from_xyz(1.0, 2.0, 3.0);
        assert_eq!(p, Vector4::new(1.0, 2.0, 3.0, 1.0));
        assert!(helpers::is_valid_point(p));

        let v3 = Vector3::new(4.0, 5.0, 6.0);
        let p2 = helpers::point_from_vector3(v3);
        assert_eq!(p2, Vector4::new(4.0, 5.0, 6.0, 1.0));

        // Test normal creation
        let n = helpers::normal_from_xyz(1.0, 0.0, 0.0);
        assert_eq!(n, Vector4::new(1.0, 0.0, 0.0, 0.0));
        assert!(helpers::is_valid_normal(n));

        let v3_n = Vector3::new(0.0, 1.0, 0.0);
        let n2 = helpers::normal_from_vector3(v3_n);
        assert_eq!(n2, Vector4::new(0.0, 1.0, 0.0, 0.0));

        // Test conversions
        let v3_back = helpers::point_to_vector3(p);
        assert_eq!(v3_back, Vector3::new(1.0, 2.0, 3.0));

        let v3_n_back = helpers::normal_to_vector3(n);
        assert_eq!(v3_n_back, Vector3::new(1.0, 0.0, 0.0));
    }

    #[test]
    fn test_bounds_checking() {
        // Should not panic
        bounds::check_bounds(0, 5, "test");
        bounds::check_bounds(4, 5, "test");

        // Should return Ok
        assert!(bounds::check_bounds_result(0, 5, "test").is_ok());
        assert!(bounds::check_bounds_result(4, 5, "test").is_ok());

        // Should return Err
        assert!(bounds::check_bounds_result(5, 5, "test").is_err());
        assert!(bounds::check_bounds_result(10, 5, "test").is_err());
    }

    #[test]
    #[should_panic(expected = "Index 5 out of bounds")]
    fn test_bounds_checking_panic() {
        bounds::check_bounds(5, 5, "test");
    }

    // Mock implementation for testing
    #[derive(Debug)]
    struct MockPointCloud {
        points: Vec<Point4<f64>>,
        normals: Option<Vec<Normal4<f64>>>,
        covariances: Option<Vec<Covariance4<f64>>>,
    }

    impl MockPointCloud {
        fn new(size: usize) -> Self {
            Self {
                points: vec![Vector4::zeros(); size],
                normals: None,
                covariances: None,
            }
        }

        fn with_normals(mut self) -> Self {
            let size = self.points.len();
            self.normals = Some(vec![Vector4::zeros(); size]);
            self
        }

        fn with_covariances(mut self) -> Self {
            let size = self.points.len();
            self.covariances = Some(vec![Matrix4::zeros(); size]);
            self
        }
    }

    impl PointCloudTrait for MockPointCloud {
        fn size(&self) -> usize {
            self.points.len()
        }

        fn has_points(&self) -> bool {
            true
        }

        fn has_normals(&self) -> bool {
            self.normals.is_some()
        }

        fn has_covariances(&self) -> bool {
            self.covariances.is_some()
        }

        fn point(&self, index: usize) -> Point4<f64> {
            bounds::check_bounds(index, self.size(), "point access");
            self.points[index]
        }

        fn normal(&self, index: usize) -> Option<Normal4<f64>> {
            if self.has_normals() {
                bounds::check_bounds(index, self.size(), "normal access");
                Some(self.normals.as_ref().unwrap()[index])
            } else {
                None
            }
        }

        fn covariance(&self, index: usize) -> Option<Covariance4<f64>> {
            if self.has_covariances() {
                bounds::check_bounds(index, self.size(), "covariance access");
                Some(self.covariances.as_ref().unwrap()[index])
            } else {
                None
            }
        }
    }

    impl MutablePointCloudTrait for MockPointCloud {
        fn resize(&mut self, size: usize) {
            self.points.resize(size, Vector4::zeros());
            if let Some(ref mut normals) = self.normals {
                normals.resize(size, Vector4::zeros());
            }
            if let Some(ref mut covariances) = self.covariances {
                covariances.resize(size, Matrix4::zeros());
            }
        }

        fn set_point(&mut self, index: usize, point: Point4<f64>) {
            bounds::check_bounds(index, self.size(), "set point");
            self.points[index] = point;
        }

        fn set_normal(&mut self, index: usize, normal: Normal4<f64>) {
            let size = self.size();
            if let Some(ref mut normals) = self.normals {
                bounds::check_bounds(index, size, "set normal");
                normals[index] = normal;
            }
        }

        fn set_covariance(&mut self, index: usize, covariance: Covariance4<f64>) {
            let size = self.size();
            if let Some(ref mut covariances) = self.covariances {
                bounds::check_bounds(index, size, "set covariance");
                covariances[index] = covariance;
            }
        }
    }

    #[test]
    fn test_trait_basic_functionality() {
        let mut cloud = MockPointCloud::new(3).with_normals().with_covariances();

        assert_eq!(cloud.size(), 3);
        assert!(!cloud.empty());
        assert!(cloud.has_points());
        assert!(cloud.has_normals());
        assert!(cloud.has_covariances());

        // Test point operations
        let point = helpers::point_from_xyz(1.0, 2.0, 3.0);
        cloud.set_point(0, point);
        assert_eq!(cloud.point(0), point);

        // Test normal operations
        let normal = helpers::normal_from_xyz(1.0, 0.0, 0.0);
        cloud.set_normal(0, normal);
        assert_eq!(cloud.normal(0), Some(normal));

        // Test covariance operations
        let cov = Matrix4::identity();
        cloud.set_covariance(0, cov);
        assert_eq!(cloud.covariance(0), Some(cov));

        // Test resize
        cloud.resize(5);
        assert_eq!(cloud.size(), 5);
        assert_eq!(cloud.point(0), point); // Original data preserved
    }

    #[test]
    fn test_optional_features() {
        let cloud = MockPointCloud::new(2); // No normals or covariances

        assert!(!cloud.has_normals());
        assert!(!cloud.has_covariances());
        assert_eq!(cloud.normal(0), None);
        assert_eq!(cloud.covariance(0), None);
    }

    #[test]
    fn test_empty_cloud() {
        let cloud = MockPointCloud::new(0);
        assert!(cloud.empty());
        assert_eq!(cloud.size(), 0);
    }

    #[test]
    #[should_panic(expected = "Index 3 out of bounds")]
    fn test_bounds_checking_point() {
        let cloud = MockPointCloud::new(3);
        cloud.point(3);
    }

    #[test]
    #[should_panic(expected = "Index 2 out of bounds")]
    fn test_bounds_checking_set_point() {
        let mut cloud = MockPointCloud::new(2);
        cloud.set_point(2, Vector4::zeros());
    }
}
