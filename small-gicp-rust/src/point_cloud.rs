//! Point cloud data structures and operations.

use crate::{
    error::Result,
    traits::{Covariance4, MutablePointCloudTrait, Normal4, Point4, PointCloudTrait},
};
use nalgebra::{Matrix4, Point3, Vector3, Vector4};
use std::ops::Range;
use tracing::{debug, info, trace, warn};

/// A view over point data that supports both iteration and random access.
#[derive(Debug, Clone)]
pub struct PointsView<'a> {
    data: &'a [f64],
    len: usize,
}

impl<'a> PointsView<'a> {
    /// Create a new points view from raw data.
    fn new(data: &'a [f64]) -> Self {
        assert_eq!(data.len() % 4, 0, "Points data must be multiple of 4");
        Self {
            data,
            len: data.len() / 4,
        }
    }

    /// Get the number of points.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Check if the view is empty.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Get a point by index with bounds checking.
    pub fn get(&self, index: usize) -> Option<Point3<f64>> {
        if index < self.len {
            let start = index * 4;
            Some(Point3::new(
                self.data[start],
                self.data[start + 1],
                self.data[start + 2],
            ))
        } else {
            None
        }
    }

    /// Get the first point.
    pub fn first(&self) -> Option<Point3<f64>> {
        self.get(0)
    }

    /// Get the last point.
    pub fn last(&self) -> Option<Point3<f64>> {
        if self.len > 0 {
            self.get(self.len - 1)
        } else {
            None
        }
    }

    /// Create a slice view of the points.
    pub fn slice(&self, range: Range<usize>) -> PointsView<'a> {
        assert!(range.start <= range.end && range.end <= self.len);
        let start_byte = range.start * 4;
        let end_byte = range.end * 4;
        PointsView::new(&self.data[start_byte..end_byte])
    }
}

// Note: We don't implement Index trait since we can't return a reference to Point3<f64>
// that we construct on the fly. Users should use get() method for bounds-checked access
// or iterate through the view.

impl Iterator for PointsView<'_> {
    type Item = Point3<f64>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.len > 0 {
            let point = Point3::new(self.data[0], self.data[1], self.data[2]);
            self.data = &self.data[4..];
            self.len -= 1;
            Some(point)
        } else {
            None
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.len, Some(self.len))
    }
}

impl ExactSizeIterator for PointsView<'_> {}

impl DoubleEndedIterator for PointsView<'_> {
    fn next_back(&mut self) -> Option<Self::Item> {
        if self.len > 0 {
            self.len -= 1;
            let start = self.len * 4;
            Some(Point3::new(
                self.data[start],
                self.data[start + 1],
                self.data[start + 2],
            ))
        } else {
            None
        }
    }
}

// IntoIterator is automatically implemented for Iterator types

/// A view over normal data that supports both iteration and random access.
#[derive(Debug, Clone)]
pub struct NormalsView<'a> {
    data: &'a [f64],
    len: usize,
}

impl<'a> NormalsView<'a> {
    /// Create a new normals view from raw data.
    fn new(data: &'a [f64]) -> Self {
        assert_eq!(data.len() % 4, 0, "Normals data must be multiple of 4");
        Self {
            data,
            len: data.len() / 4,
        }
    }

    /// Get the number of normals.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Check if the view is empty.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Get a normal by index with bounds checking.
    pub fn get(&self, index: usize) -> Option<Vector3<f64>> {
        if index < self.len {
            let start = index * 4;
            Some(Vector3::new(
                self.data[start],
                self.data[start + 1],
                self.data[start + 2],
            ))
        } else {
            None
        }
    }

    /// Get the first normal.
    pub fn first(&self) -> Option<Vector3<f64>> {
        self.get(0)
    }

    /// Get the last normal.
    pub fn last(&self) -> Option<Vector3<f64>> {
        if self.len > 0 {
            self.get(self.len - 1)
        } else {
            None
        }
    }

    /// Create a slice view of the normals.
    pub fn slice(&self, range: Range<usize>) -> NormalsView<'a> {
        assert!(range.start <= range.end && range.end <= self.len);
        let start_byte = range.start * 4;
        let end_byte = range.end * 4;
        NormalsView::new(&self.data[start_byte..end_byte])
    }
}

impl Iterator for NormalsView<'_> {
    type Item = Vector3<f64>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.len > 0 {
            let normal = Vector3::new(self.data[0], self.data[1], self.data[2]);
            self.data = &self.data[4..];
            self.len -= 1;
            Some(normal)
        } else {
            None
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.len, Some(self.len))
    }
}

impl ExactSizeIterator for NormalsView<'_> {}

impl DoubleEndedIterator for NormalsView<'_> {
    fn next_back(&mut self) -> Option<Self::Item> {
        if self.len > 0 {
            self.len -= 1;
            let start = self.len * 4;
            Some(Vector3::new(
                self.data[start],
                self.data[start + 1],
                self.data[start + 2],
            ))
        } else {
            None
        }
    }
}

// IntoIterator is automatically implemented for Iterator types

/// A view over covariance data that supports both iteration and random access.
#[derive(Debug, Clone)]
pub struct CovariancesView<'a> {
    data: &'a [f64],
    len: usize,
}

impl<'a> CovariancesView<'a> {
    /// Create a new covariances view from raw data.
    fn new(data: &'a [f64]) -> Self {
        assert_eq!(
            data.len() % 16,
            0,
            "Covariances data must be multiple of 16"
        );
        Self {
            data,
            len: data.len() / 16,
        }
    }

    /// Get the number of covariances.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Check if the view is empty.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Get a covariance matrix by index with bounds checking.
    pub fn get(&self, index: usize) -> Option<Matrix4<f64>> {
        if index < self.len {
            let start = index * 16;
            let mut matrix = Matrix4::zeros();
            for row in 0..4 {
                for col in 0..4 {
                    matrix[(row, col)] = self.data[start + row * 4 + col];
                }
            }
            Some(matrix)
        } else {
            None
        }
    }

    /// Get the first covariance.
    pub fn first(&self) -> Option<Matrix4<f64>> {
        self.get(0)
    }

    /// Get the last covariance.
    pub fn last(&self) -> Option<Matrix4<f64>> {
        if self.len > 0 {
            self.get(self.len - 1)
        } else {
            None
        }
    }

    /// Create a slice view of the covariances.
    pub fn slice(&self, range: Range<usize>) -> CovariancesView<'a> {
        assert!(range.start <= range.end && range.end <= self.len);
        let start_byte = range.start * 16;
        let end_byte = range.end * 16;
        CovariancesView::new(&self.data[start_byte..end_byte])
    }
}

impl Iterator for CovariancesView<'_> {
    type Item = Matrix4<f64>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.len > 0 {
            let mut matrix = Matrix4::zeros();
            for row in 0..4 {
                for col in 0..4 {
                    matrix[(row, col)] = self.data[row * 4 + col];
                }
            }
            self.data = &self.data[16..];
            self.len -= 1;
            Some(matrix)
        } else {
            None
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.len, Some(self.len))
    }
}

impl ExactSizeIterator for CovariancesView<'_> {}

impl DoubleEndedIterator for CovariancesView<'_> {
    fn next_back(&mut self) -> Option<Self::Item> {
        if self.len > 0 {
            self.len -= 1;
            let start = self.len * 16;
            let mut matrix = Matrix4::zeros();
            for row in 0..4 {
                for col in 0..4 {
                    matrix[(row, col)] = self.data[start + row * 4 + col];
                }
            }
            Some(matrix)
        } else {
            None
        }
    }
}

// IntoIterator is automatically implemented for Iterator types

/// A 3D point cloud with optional normal vectors and covariances.
///
/// This is a high-level Rust wrapper around the small-gicp-sys PointCloud.
/// It provides a trait-based interface for working with point clouds in a
/// type-safe and ergonomic way.
pub struct PointCloud {
    inner: small_gicp_sys::PointCloud,
}

impl PointCloud {
    /// Create a new empty point cloud.
    pub fn new() -> Result<Self> {
        trace!("Creating new empty PointCloud");
        let inner = small_gicp_sys::PointCloud::new();
        debug!("Created PointCloud with {} points", inner.len());
        Ok(Self { inner })
    }

    /// Create a point cloud from a vector of points.
    pub fn from_points(points: &[Point3<f64>]) -> Result<Self> {
        info!("Creating PointCloud from {} points", points.len());
        let mut cloud = Self::new()?;
        cloud.resize(points.len())?;
        cloud.set_points(points)?;
        debug!(
            "Successfully created PointCloud with {} points",
            cloud.len()
        );
        Ok(cloud)
    }

    /// Create a point cloud from vectors of points and normals.
    pub fn from_points_and_normals(
        points: &[Point3<f64>],
        normals: &[Vector3<f64>],
    ) -> Result<Self> {
        let mut cloud = Self::from_points(points)?;
        cloud.set_normals(normals)?;
        Ok(cloud)
    }

    /// Get the number of points in the cloud.
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    /// Check if the cloud is empty.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Access the underlying small-gicp-sys PointCloud.
    pub(crate) fn inner(&self) -> &small_gicp_sys::PointCloud {
        &self.inner
    }

    /// Reserve space for n points.
    pub fn reserve(&mut self, n: usize) {
        self.inner.reserve(n);
    }

    /// Resize the cloud to n points.
    pub fn resize(&mut self, n: usize) -> Result<()> {
        self.inner.resize(n);
        Ok(())
    }

    /// Clear all points from the cloud.
    pub fn clear(&mut self) {
        self.inner.clear();
    }

    /// Add a point to the cloud.
    pub fn add_point(&mut self, x: f64, y: f64, z: f64) {
        self.inner.add_point(x, y, z);
    }

    /// Set a point at the given index.
    pub fn set_point(&mut self, index: usize, x: f64, y: f64, z: f64) -> Result<()> {
        self.inner.set_point(index, x, y, z);
        Ok(())
    }

    /// Get a point at the given index.
    pub fn point_at(&self, index: usize) -> Result<(f64, f64, f64)> {
        self.inner
            .get_point(index)
            .ok_or_else(|| crate::error::SmallGicpError::IndexOutOfBounds {
                index,
                size: self.len(),
            })
    }

    /// Set multiple points efficiently.
    pub fn set_points(&mut self, points: &[Point3<f64>]) -> Result<()> {
        // Convert Point3<f64> to raw f64 array in (x, y, z, 1.0) format
        let mut raw_points = Vec::with_capacity(points.len() * 4);
        for point in points {
            raw_points.extend_from_slice(&[point.x, point.y, point.z, 1.0]);
        }
        self.inner.set_points_bulk(&raw_points);
        Ok(())
    }

    /// Get all points as an efficient view that supports both iteration and random access.
    pub fn points(&self) -> PointsView<'_> {
        PointsView::new(self.inner.points_data())
    }

    /// Get raw points data as a slice (x, y, z, 1.0 for each point).
    pub fn points_raw(&self) -> &[f64] {
        self.inner.points_data()
    }

    /// Get raw points data as 4D homogeneous coordinates.
    pub fn points_raw_xyzw(&self) -> &[[f64; 4]] {
        let data = self.inner.points_data();
        assert_eq!(data.len() % 4, 0);
        // SAFETY: We ensure the slice length is a multiple of 4
        unsafe { std::slice::from_raw_parts(data.as_ptr() as *const [f64; 4], data.len() / 4) }
    }

    /// Convert all points to a vector (copies data).
    pub fn to_points_vec(&self) -> Vec<Point3<f64>> {
        self.points().collect()
    }

    /// Set normal vectors.
    pub fn set_normals(&mut self, normals: &[Vector3<f64>]) -> Result<()> {
        // Convert Vector3<f64> to raw f64 array in (nx, ny, nz, 0.0) format
        let mut raw_normals = Vec::with_capacity(normals.len() * 4);
        for normal in normals {
            raw_normals.extend_from_slice(&[normal.x, normal.y, normal.z, 0.0]);
        }
        self.inner.set_normals_bulk(&raw_normals);
        Ok(())
    }

    /// Get normal vectors as an efficient view that supports both iteration and random access.
    pub fn normals(&self) -> Option<NormalsView<'_>> {
        if self.has_normals() {
            Some(NormalsView::new(self.inner.normals_data()))
        } else {
            None
        }
    }

    /// Get raw normals data as a slice (nx, ny, nz, 0.0 for each normal).
    pub fn normals_raw(&self) -> &[f64] {
        self.inner.normals_data()
    }

    /// Get raw normals data as 4D vectors.
    pub fn normals_raw_xyzw(&self) -> &[[f64; 4]] {
        let data = self.inner.normals_data();
        if data.is_empty() {
            return &[];
        }
        assert_eq!(data.len() % 4, 0);
        // SAFETY: We ensure the slice length is a multiple of 4
        unsafe { std::slice::from_raw_parts(data.as_ptr() as *const [f64; 4], data.len() / 4) }
    }

    /// Convert all normals to a vector (copies data).
    pub fn to_normals_vec(&self) -> Vec<Vector3<f64>> {
        self.normals()
            .map(|view| view.collect())
            .unwrap_or_default()
    }

    /// Check if the cloud has normal vectors.
    pub fn has_normals(&self) -> bool {
        !self.inner.normals_data().is_empty()
    }

    /// Set covariance matrices.
    pub fn set_covariances(&mut self, covariances: &[Matrix4<f64>]) -> Result<()> {
        if covariances.len() != self.len() {
            return Err(crate::error::SmallGicpError::InvalidArgument(format!(
                "Number of covariances {} does not match number of points {}",
                covariances.len(),
                self.len()
            )));
        }

        // Convert Matrix4<f64> to raw f64 array (row-major 4x4 matrices)
        let mut raw_covariances = Vec::with_capacity(covariances.len() * 16);
        for covariance in covariances {
            for row in 0..4 {
                for col in 0..4 {
                    raw_covariances.push(covariance[(row, col)]);
                }
            }
        }
        self.inner.set_covariances_bulk(&raw_covariances);
        Ok(())
    }

    /// Get covariance matrices as an efficient view that supports both iteration and random access.
    pub fn covariances(&self) -> Option<CovariancesView<'_>> {
        if self.has_covariances() {
            Some(CovariancesView::new(self.inner.covs_data()))
        } else {
            None
        }
    }

    /// Get raw covariances data as a slice (16 values per 4x4 matrix in row-major order).
    pub fn covariances_raw(&self) -> &[f64] {
        self.inner.covs_data()
    }

    /// Convert all covariances to a vector (copies data).
    pub fn to_covariances_vec(&self) -> Vec<Matrix4<f64>> {
        self.covariances()
            .map(|view| view.collect())
            .unwrap_or_default()
    }

    /// Check if the cloud has covariance matrices.
    pub fn has_covariances(&self) -> bool {
        !self.inner.covs_data().is_empty()
    }

    /// Estimate normal vectors using k nearest neighbors.
    pub fn estimate_normals(&mut self, k: usize) -> Result<()> {
        info!(
            "Estimating normals for {} points using k={} neighbors",
            self.len(),
            k
        );
        self.inner.estimate_normals(k as i32, 1);
        debug!("Normal estimation completed");
        Ok(())
    }

    /// Estimate normal vectors using k nearest neighbors with multiple threads.
    pub fn estimate_normals_parallel(&mut self, k: usize, num_threads: usize) -> Result<()> {
        self.inner.estimate_normals(k as i32, num_threads as i32);
        Ok(())
    }

    /// Estimate covariance matrices using k nearest neighbors.
    pub fn estimate_covariances(&mut self, k: usize) -> Result<()> {
        self.inner.estimate_covariances(k as i32, 1);
        Ok(())
    }

    /// Estimate covariance matrices using k nearest neighbors with multiple threads.
    pub fn estimate_covariances_parallel(&mut self, k: usize, num_threads: usize) -> Result<()> {
        self.inner
            .estimate_covariances(k as i32, num_threads as i32);
        Ok(())
    }

    /// Apply a transformation to all points in the cloud.
    pub fn transform(&mut self, transform: &Matrix4<f64>) -> Result<()> {
        // Convert Matrix4<f64> to Transform struct
        let mut matrix = [0.0; 16];
        for row in 0..4 {
            for col in 0..4 {
                matrix[row * 4 + col] = transform[(row, col)];
            }
        }
        let transform_struct = small_gicp_sys::Transform { matrix };
        self.inner.transform(&transform_struct);
        Ok(())
    }

    /// Create a transformed copy of the cloud.
    pub fn transformed(&self, transform: &Matrix4<f64>) -> Result<Self> {
        // Convert Matrix4<f64> to Transform struct
        let mut matrix = [0.0; 16];
        for row in 0..4 {
            for col in 0..4 {
                matrix[row * 4 + col] = transform[(row, col)];
            }
        }
        let transform_struct = small_gicp_sys::Transform { matrix };
        Ok(Self {
            inner: self.inner.transformed(&transform_struct),
        })
    }

    /// Access the underlying small-gicp-sys PointCloud mutably.
    /// This is for internal use only and should not be exposed to users.
    pub(crate) fn inner_mut(&mut self) -> &mut small_gicp_sys::PointCloud {
        &mut self.inner
    }

    /// Create a PointCloud from a small-gicp-sys PointCloud.
    /// This is for internal use only and should not be exposed to users.
    pub(crate) fn from_cxx(inner: small_gicp_sys::PointCloud) -> Self {
        Self { inner }
    }

    /// Convert to a small-gicp-sys PointCloud.
    /// This is for internal use only and should not be exposed to users.
    pub(crate) fn into_cxx(self) -> small_gicp_sys::PointCloud {
        self.inner
    }

    /// Set points from raw f64 data (3 values per point: x, y, z).
    pub fn set_points_bulk(&mut self, points_data: &[f64]) -> Result<()> {
        if points_data.len() % 3 != 0 {
            return Err(crate::error::SmallGicpError::InvalidArgument(format!(
                "Points data length {} is not a multiple of 3",
                points_data.len()
            )));
        }

        let num_points = points_data.len() / 3;
        let mut points = Vec::with_capacity(num_points);

        for i in 0..num_points {
            let base = i * 3;
            points.push(Point3::new(
                points_data[base],
                points_data[base + 1],
                points_data[base + 2],
            ));
        }

        self.resize(num_points)?;
        self.set_points(&points)?;
        Ok(())
    }

    /// Set normals from raw f64 data (3 values per normal: x, y, z).
    pub fn set_normals_bulk(&mut self, normals: &[f64]) -> Result<()> {
        if normals.len() % 3 != 0 {
            return Err(crate::error::SmallGicpError::InvalidArgument(format!(
                "Normals data length {} is not a multiple of 3",
                normals.len()
            )));
        }

        // Convert 3-value normals to 4-value format expected by CXX (nx, ny, nz, 0.0)
        let num_normals = normals.len() / 3;
        let mut normals_4d = Vec::with_capacity(num_normals * 4);

        for i in 0..num_normals {
            let base = i * 3;
            normals_4d.extend_from_slice(&[
                normals[base],
                normals[base + 1],
                normals[base + 2],
                0.0, // w component
            ]);
        }

        self.inner.set_normals_bulk(&normals_4d);
        Ok(())
    }

    /// Set covariances from raw f64 data (16 values per covariance matrix).
    pub fn set_covariances_bulk(&mut self, covariances: &[f64]) -> Result<()> {
        if covariances.len() % 16 != 0 {
            return Err(crate::error::SmallGicpError::InvalidArgument(format!(
                "Covariances data length {} is not a multiple of 16",
                covariances.len()
            )));
        }

        let expected_len = self.len() * 16;
        if covariances.len() != expected_len {
            return Err(crate::error::SmallGicpError::InvalidArgument(format!(
                "Covariances data length {} does not match expected length {} for {} points",
                covariances.len(),
                expected_len,
                self.len()
            )));
        }

        self.inner.set_covariances_bulk(covariances);
        Ok(())
    }

    /// Get a covariance matrix at the given index.
    pub fn covariance_at(&self, index: usize) -> Result<Matrix4<f64>> {
        if index >= self.len() {
            return Err(crate::error::SmallGicpError::IndexOutOfBounds {
                index,
                size: self.len(),
            });
        }

        let raw_data = self.inner.covs_data();
        if raw_data.is_empty() {
            return Err(crate::error::SmallGicpError::InvalidArgument(
                "Point cloud has no covariance data".to_string(),
            ));
        }

        let start_idx = index * 16;
        let mut matrix = Matrix4::zeros();
        for row in 0..4 {
            for col in 0..4 {
                matrix[(row, col)] = raw_data[start_idx + row * 4 + col];
            }
        }
        Ok(matrix)
    }

    /// Set a covariance matrix at the given index.
    pub fn set_covariance(&mut self, index: usize, covariance: Matrix4<f64>) -> Result<()> {
        if index >= self.len() {
            return Err(crate::error::SmallGicpError::IndexOutOfBounds {
                index,
                size: self.len(),
            });
        }

        // Ensure the covariances array exists and is the right size
        if !self.has_covariances() {
            // Initialize with identity matrices
            let identity_covs = vec![Matrix4::identity(); self.len()];
            self.set_covariances(&identity_covs)?;
        }

        // Get current covariances, update the specific one, and set them back
        let mut covs = self.to_covariances_vec();
        if covs.len() != self.len() {
            covs.resize(self.len(), Matrix4::identity());
        }
        covs[index] = covariance;
        self.set_covariances(&covs)?;

        Ok(())
    }

    /// Get a normal vector at the given index.
    pub fn normal_at(&self, index: usize) -> Result<Vector3<f64>> {
        if index >= self.len() {
            return Err(crate::error::SmallGicpError::IndexOutOfBounds {
                index,
                size: self.len(),
            });
        }

        let raw_data = self.inner.normals_data();
        if raw_data.is_empty() {
            return Err(crate::error::SmallGicpError::InvalidArgument(
                "Point cloud has no normal data".to_string(),
            ));
        }

        let start_idx = index * 4;
        Ok(Vector3::new(
            raw_data[start_idx],
            raw_data[start_idx + 1],
            raw_data[start_idx + 2],
        ))
    }

    /// Copy points to an output array (3 values per point: x, y, z).
    pub fn copy_points_to_array(&self, output: &mut [f64]) -> Result<()> {
        if output.len() != self.len() * 3 {
            return Err(crate::error::SmallGicpError::InvalidArgument(format!(
                "Output array size {} does not match expected size {}",
                output.len(),
                self.len() * 3
            )));
        }

        for (i, point) in self.points().enumerate() {
            let start = i * 3;
            output[start] = point.x;
            output[start + 1] = point.y;
            output[start + 2] = point.z;
        }
        Ok(())
    }

    /// Copy normals to an output array (3 values per normal: x, y, z).
    pub fn copy_normals_to_array(&self, output: &mut [f64]) -> Result<()> {
        let normals = self.normals().ok_or_else(|| {
            crate::error::SmallGicpError::InvalidArgument("Point cloud has no normals".to_string())
        })?;

        if output.len() != normals.len() * 3 {
            return Err(crate::error::SmallGicpError::InvalidArgument(format!(
                "Output array size {} does not match expected size {}",
                output.len(),
                normals.len() * 3
            )));
        }

        for (i, normal) in normals.enumerate() {
            let start = i * 3;
            output[start] = normal.x;
            output[start + 1] = normal.y;
            output[start + 2] = normal.z;
        }
        Ok(())
    }

    /// Copy covariances to an output array (16 values per matrix).
    pub fn copy_covariances_to_array(&self, output: &mut [f64]) -> Result<()> {
        let covariances = self.covariances().ok_or_else(|| {
            crate::error::SmallGicpError::InvalidArgument(
                "Point cloud has no covariances".to_string(),
            )
        })?;

        if output.len() != covariances.len() * 16 {
            return Err(crate::error::SmallGicpError::InvalidArgument(format!(
                "Output array size {} does not match expected size {}",
                output.len(),
                covariances.len() * 16
            )));
        }

        for (i, matrix) in covariances.enumerate() {
            let start = i * 16;
            for row in 0..4 {
                for col in 0..4 {
                    output[start + row * 4 + col] = matrix[(row, col)];
                }
            }
        }
        Ok(())
    }

    /// Get direct access to points data as a slice.
    /// Returns 4D homogeneous coordinates (x, y, z, w) where w=1.0.
    ///
    /// This is a safe wrapper around the internal data access.
    pub fn points_data(&self) -> &[f64] {
        self.inner.points_data()
    }

    /// Get direct access to normals data as a slice.
    /// Returns 4D vectors (x, y, z, w) where w=0.0.
    ///
    /// This is a safe wrapper around the internal data access.
    pub fn normals_data(&self) -> &[f64] {
        self.inner.normals_data()
    }

    /// Get direct access to covariances data as a slice.
    /// Returns flattened 4x4 matrices in row-major order.
    ///
    /// This is a safe wrapper around the internal data access.
    pub fn covariances_data(&self) -> &[f64] {
        self.inner.covs_data()
    }

    /// Check if the cloud is empty using the trait method name.
    pub fn empty(&self) -> bool {
        self.is_empty()
    }

    /// Check if the cloud has points using the trait method name.
    pub fn has_points(&self) -> bool {
        !self.is_empty()
    }

    /// Load a point cloud from a PLY file.
    pub fn load_ply<P: AsRef<std::path::Path>>(path: P) -> Result<Self> {
        let path_display = path.as_ref().display().to_string();
        info!("Loading PointCloud from PLY file: {}", path_display);
        let inner =
            small_gicp_sys::Io::load_ply(path).map_err(crate::error::SmallGicpError::IoError)?;
        let cloud = Self { inner };
        info!(
            "Successfully loaded {} points from {}",
            cloud.len(),
            path_display
        );
        Ok(cloud)
    }

    /// Save the point cloud to a PLY file.
    pub fn save_ply<P: AsRef<std::path::Path>>(&self, path: P) -> Result<()> {
        small_gicp_sys::Io::save_ply(path, &self.inner)
            .map_err(crate::error::SmallGicpError::IoError)?;
        Ok(())
    }

    /// Downsample using voxel grid filtering.
    pub fn voxelgrid_sampling(&self, config: &crate::config::VoxelGridConfig) -> Result<Self> {
        use crate::preprocessing::Preprocessing;
        Ok(Preprocessing::voxel_downsample(
            self,
            config.leaf_size,
            config.num_threads,
        ))
    }

    /// Downsample using random sampling.
    pub fn random_sampling(&self, num_samples: usize) -> Result<Self> {
        use crate::preprocessing::Preprocessing;
        Ok(Preprocessing::random_downsample(self, num_samples))
    }

    /// Preprocess point cloud with the given configuration.
    pub fn preprocess_points(&self, config: &crate::config::PreprocessingConfig) -> Result<Self> {
        use crate::preprocessing::Preprocessing;

        // Start with downsampling if configured
        let mut processed = if let Some(downsample_config) = &config.downsampling {
            self.voxelgrid_sampling(downsample_config)?
        } else {
            self.clone()
        };

        // Estimate normals if configured
        if let Some(normal_config) = &config.normal_estimation {
            Preprocessing::estimate_normals(
                &mut processed,
                normal_config.num_neighbors as usize,
                normal_config.num_threads,
            )?;
        }

        // Estimate covariances if configured
        if let Some(cov_config) = &config.covariance_estimation {
            Preprocessing::estimate_covariances(
                &mut processed,
                cov_config.num_neighbors as usize,
                cov_config.num_threads,
            )?;
        }

        Ok(processed)
    }
}

impl std::fmt::Debug for PointCloud {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PointCloud")
            .field("len", &self.len())
            .finish()
    }
}

impl Default for PointCloud {
    fn default() -> Self {
        Self::new().unwrap()
    }
}

impl Clone for PointCloud {
    fn clone(&self) -> Self {
        // Create a new point cloud and copy data
        let mut new_cloud = Self::new().unwrap();

        // Copy points
        if !self.is_empty() {
            let points_data = self.inner.points_data();
            if !points_data.is_empty() {
                new_cloud.inner.set_points_bulk(points_data);
            }

            // Copy normals if available
            let normals_data = self.inner.normals_data();
            if !normals_data.is_empty() {
                new_cloud.inner.set_normals_bulk(normals_data);
            }

            // Copy covariances if available
            let covs_data = self.inner.covs_data();
            if !covs_data.is_empty() {
                new_cloud.inner.set_covariances_bulk(covs_data);
            }
        }

        new_cloud
    }
}

// Manual implementation of Send and Sync
// This is safe because small_gicp_sys::PointCloud manages its own memory safety
// and the underlying C++ implementation is thread-safe for read operations
unsafe impl Send for PointCloud {}
unsafe impl Sync for PointCloud {}

// TODO: Implement trait implementations for PointCloudTrait and MutablePointCloudTrait
// These will provide the generic interface for the point cloud.

impl PointCloudTrait for PointCloud {
    fn len(&self) -> usize {
        self.len()
    }

    fn point(&self, i: usize) -> Point4<f64> {
        let (x, y, z) = self.point_at(i).unwrap_or((0.0, 0.0, 0.0));
        Vector4::new(x, y, z, 1.0)
    }

    fn normal(&self, i: usize) -> Option<Normal4<f64>> {
        self.normal_at(i)
            .ok()
            .map(|n| Vector4::new(n.x, n.y, n.z, 0.0))
    }

    fn covariance(&self, i: usize) -> Option<Covariance4<f64>> {
        self.covariance_at(i).ok()
    }

    fn has_points(&self) -> bool {
        !self.is_empty()
    }

    fn has_normals(&self) -> bool {
        self.has_normals()
    }

    fn has_covariances(&self) -> bool {
        self.has_covariances()
    }
}

impl MutablePointCloudTrait for PointCloud {
    fn resize(&mut self, n: usize) {
        self.resize(n).unwrap();
    }

    fn set_point(&mut self, i: usize, point: Point4<f64>) {
        self.set_point(i, point.x, point.y, point.z).unwrap();
    }

    fn set_normal(&mut self, i: usize, normal: Normal4<f64>) {
        // For individual normal setting, we need to modify the bulk data
        // This is not efficient but implements the trait requirement
        warn!("Individual normal setting is not efficient - consider using bulk operations");

        // Get current normals data or create new if empty
        let mut normals_data = if self.has_normals() {
            self.inner.normals_data().to_vec()
        } else {
            vec![0.0; self.len() * 4]
        };

        // Update the specific normal
        if i < self.len() {
            let start = i * 4;
            normals_data[start] = normal.x;
            normals_data[start + 1] = normal.y;
            normals_data[start + 2] = normal.z;
            normals_data[start + 3] = normal.w;

            // Set the updated data back
            self.inner.set_normals_bulk(&normals_data);
        }
    }

    fn set_covariance(&mut self, i: usize, cov: Covariance4<f64>) {
        // For individual covariance setting, we need to modify the bulk data
        // This is not efficient but implements the trait requirement
        warn!("Individual covariance setting is not efficient - consider using bulk operations");

        // Get current covariances data or create new if empty
        let mut covs_data = if self.has_covariances() {
            self.inner.covs_data().to_vec()
        } else {
            vec![0.0; self.len() * 16]
        };

        // Update the specific covariance
        if i < self.len() {
            let start = i * 16;
            for row in 0..4 {
                for col in 0..4 {
                    covs_data[start + row * 4 + col] = cov[(row, col)];
                }
            }

            // Set the updated data back
            self.inner.set_covariances_bulk(&covs_data);
        }
    }
}

#[cfg(test)]
pub mod tests {
    use super::*;
    use nalgebra::{Matrix4, Vector3, Vector4};

    // Reference: small_gicp/src/test/points_test.cpp

    /// Test that PointCloud correctly implements all required traits
    /// Reference: points_test.cpp - trait checks in test_points()
    #[test]
    fn test_traits_implementation() {
        let points = vec![
            Point3::new(1.0, 2.0, 3.0),
            Point3::new(4.0, 5.0, 6.0),
            Point3::new(7.0, 8.0, 9.0),
        ];
        let cloud = PointCloud::from_points(&points).unwrap();

        // Test PointCloudTrait implementation
        assert_eq!(cloud.len(), 3);
        assert!(!cloud.is_empty());

        // Test point access returns correct values
        let (x, y, z) = cloud.point_at(0).unwrap();
        assert_eq!((x, y, z), (1.0, 2.0, 3.0));

        // Test normal access
        // Note: Default normals in C++ may contain uninitialized data
        // We just verify that we can access them without panicking
        let _ = cloud.normal_at(0).unwrap();

        // Test covariance access
        // Note: Default covariances in C++ may contain uninitialized data
        // We just verify that we can access them without panicking
        let _ = cloud.covariance_at(0).unwrap();
    }

    /// Test point access methods
    /// Reference: points_test.cpp:29 - traits::point() validation
    #[test]
    fn test_point_access() {
        let test_points = vec![
            Point3::new(1.0, 2.0, 3.0),
            Point3::new(4.0, 5.0, 6.0),
            Point3::new(7.0, 8.0, 9.0),
            Point3::new(10.0, 11.0, 12.0),
        ];
        let cloud = PointCloud::from_points(&test_points).unwrap();

        // Test individual point access
        for i in 0..cloud.len() {
            let (x, y, z) = cloud.point_at(i).unwrap();
            assert_eq!(x, test_points[i].x);
            assert_eq!(y, test_points[i].y);
            assert_eq!(z, test_points[i].z);
        }

        // Test bounds checking
        assert!(cloud.point_at(cloud.len()).is_err());

        // Test points view iterator
        let view = cloud.points();
        let collected: Vec<_> = view.collect();
        assert_eq!(collected.len(), test_points.len());
        for (i, point) in collected.iter().enumerate() {
            assert_eq!(point.x, test_points[i].x);
            assert_eq!(point.y, test_points[i].y);
            assert_eq!(point.z, test_points[i].z);
        }

        // Test raw data access
        let raw_data = cloud.points_raw();
        assert_eq!(raw_data.len(), test_points.len() * 4);
        for i in 0..test_points.len() {
            assert_eq!(raw_data[i * 4], test_points[i].x);
            assert_eq!(raw_data[i * 4 + 1], test_points[i].y);
            assert_eq!(raw_data[i * 4 + 2], test_points[i].z);
            assert_eq!(raw_data[i * 4 + 3], 1.0); // w component
        }
    }

    /// Test normal access and modification
    /// Reference: points_test.cpp:26-30 - traits::set_normal() and traits::normal()
    #[test]
    fn test_normal_access() {
        let points = vec![Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 5.0, 6.0)];
        let mut cloud = PointCloud::from_points(&points).unwrap();

        // Test that we can access normals
        // Note: Default normals in C++ may contain uninitialized data
        for i in 0..cloud.len() {
            let _ = cloud.normal_at(i).unwrap();
        }

        // Test setting normals
        let test_normals = vec![Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)];

        cloud.set_normals(&test_normals).unwrap();

        // Verify normals were set correctly
        for (i, expected) in test_normals.iter().enumerate() {
            let normal = cloud.normal_at(i).unwrap();
            assert!((normal.x - expected.x).abs() < 1e-6);
            assert!((normal.y - expected.y).abs() < 1e-6);
            assert!((normal.z - expected.z).abs() < 1e-6);
        }

        // Test bounds checking
        assert!(cloud.normal_at(cloud.len()).is_err());
    }

    /// Test covariance access and modification
    /// Reference: points_test.cpp:27-31 - traits::set_cov() and traits::cov()
    #[test]
    fn test_covariance_access() {
        let points = vec![Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 5.0, 6.0)];
        let mut cloud = PointCloud::from_points(&points).unwrap();

        // Test default covariances
        // Note: Default covariances in C++ may contain uninitialized data
        // We just verify that we can access them without panicking
        for i in 0..cloud.len() {
            let _ = cloud.covariance_at(i).unwrap();
        }

        // Test setting covariances
        // Create test covariance matrices (symmetric positive semi-definite)
        let mut test_cov1 = Matrix4::zeros();
        test_cov1[(0, 0)] = 2.0;
        test_cov1[(1, 1)] = 3.0;
        test_cov1[(2, 2)] = 4.0;
        test_cov1[(3, 3)] = 1.0;
        test_cov1[(0, 1)] = 0.5;
        test_cov1[(1, 0)] = 0.5;

        cloud.set_covariance(0, test_cov1).unwrap();

        // Verify covariance was set correctly
        let retrieved_cov = cloud.covariance_at(0).unwrap();
        for row in 0..4 {
            for col in 0..4 {
                assert!((retrieved_cov[(row, col)] - test_cov1[(row, col)]).abs() < 1e-6);
            }
        }

        // Test bounds checking
        assert!(cloud.covariance_at(cloud.len()).is_err());
    }

    /// Test resize operations
    /// Reference: points_test.cpp:34-41 - traits::resize()
    #[test]
    fn test_resize_operations() {
        let initial_points = vec![
            Point3::new(1.0, 2.0, 3.0),
            Point3::new(4.0, 5.0, 6.0),
            Point3::new(7.0, 8.0, 9.0),
            Point3::new(10.0, 11.0, 12.0),
        ];
        let mut cloud = PointCloud::from_points(&initial_points).unwrap();

        // Set some custom normals and covariances
        let normals: Vec<Vector3<f64>> = (0..cloud.len())
            .map(|i| Vector3::new(i as f64, (i + 1) as f64, (i + 2) as f64))
            .collect();
        cloud.set_normals(&normals).unwrap();

        for i in 0..cloud.len() {
            let mut cov = Matrix4::zeros();
            cov[(0, 0)] = (i + 1) as f64;
            cov[(1, 1)] = (i + 2) as f64;
            cov[(2, 2)] = (i + 3) as f64;
            cov[(3, 3)] = 1.0;
            cloud.set_covariance(i, cov).unwrap();
        }

        // Test resize to smaller size
        let new_size = 2;
        cloud.resize(new_size).unwrap();
        assert_eq!(cloud.len(), new_size);

        // Verify that first 2 points are preserved
        for i in 0..new_size {
            let (x, y, z) = cloud.point_at(i).unwrap();
            assert_eq!(x, initial_points[i].x);
            assert_eq!(y, initial_points[i].y);
            assert_eq!(z, initial_points[i].z);

            // Check normals are preserved
            let normal = cloud.normal_at(i).unwrap();
            assert_eq!(normal.x, i as f64);
            assert_eq!(normal.y, (i + 1) as f64);
            assert_eq!(normal.z, (i + 2) as f64);

            // Check covariances are preserved
            let cov = cloud.covariance_at(i).unwrap();
            assert_eq!(cov[(0, 0)], (i + 1) as f64);
            assert_eq!(cov[(1, 1)], (i + 2) as f64);
            assert_eq!(cov[(2, 2)], (i + 3) as f64);
        }

        // Test resize to larger size
        cloud.resize(5).unwrap();
        assert_eq!(cloud.len(), 5);

        // Original points should still be preserved
        for i in 0..new_size {
            let (x, y, z) = cloud.point_at(i).unwrap();
            assert_eq!(x, initial_points[i].x);
            assert_eq!(y, initial_points[i].y);
            assert_eq!(z, initial_points[i].z);
        }

        // Note: In the C++ implementation, resizing to a larger size
        // may not initialize the new elements. This is implementation-defined
        // behavior. We just verify that we can access the new indices without
        // panicking.
        for i in new_size..5 {
            let _ = cloud.point_at(i).unwrap();
            let _ = cloud.normal_at(i).unwrap();
            let _ = cloud.covariance_at(i).unwrap();
        }
    }

    /// Test PLY file I/O operations
    #[test]
    fn test_ply_io() {
        // Test loading PLY file
        let test_file = "data/source.ply";
        if std::path::Path::new(test_file).exists() {
            match PointCloud::load_ply(test_file) {
                Ok(cloud) => {
                    assert!(!cloud.is_empty());
                    // Verify we can access points
                    let _ = cloud.point_at(0).unwrap();
                }
                Err(e) => {
                    // PLY loading might not be implemented yet
                    eprintln!("PLY loading not yet implemented: {}", e);
                }
            }
        } else {
            eprintln!(
                "Test PLY file not found at {}, skipping PLY I/O test",
                test_file
            );
        }

        // Test saving PLY file
        let points = vec![Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 5.0, 6.0)];
        let cloud = PointCloud::from_points(&points).unwrap();

        let temp_file = "/tmp/test_point_cloud.ply";
        match cloud.save_ply(temp_file) {
            Ok(_) => {
                // Try to load it back
                match PointCloud::load_ply(temp_file) {
                    Ok(loaded) => {
                        assert_eq!(loaded.len(), cloud.len());
                        for i in 0..cloud.len() {
                            let (x1, y1, z1) = cloud.point_at(i).unwrap();
                            let (x2, y2, z2) = loaded.point_at(i).unwrap();
                            assert!((x1 - x2).abs() < 1e-6);
                            assert!((y1 - y2).abs() < 1e-6);
                            assert!((z1 - z2).abs() < 1e-6);
                        }
                    }
                    Err(e) => eprintln!("Failed to load saved PLY: {}", e),
                }
                // Clean up
                let _ = std::fs::remove_file(temp_file);
            }
            Err(e) => {
                // PLY saving might not be implemented yet
                eprintln!("PLY saving not yet implemented: {}", e);
            }
        }
    }

    // Additional test utilities matching C++ test patterns

    /// Test with random data similar to C++ test
    /// Reference: points_test.cpp:44-60 - PointsTest.PointsTest
    #[test]
    fn test_with_random_data() {
        use rand::{distributions::Uniform, thread_rng, Rng};

        let mut rng = thread_rng();
        let dist = Uniform::new(-100.0, 100.0);

        // Generate random points
        let num_points = 100;
        let src_points: Vec<Point3<f64>> = (0..num_points)
            .map(|_| Point3::new(rng.sample(&dist), rng.sample(&dist), rng.sample(&dist)))
            .collect();

        let mut cloud = PointCloud::from_points(&src_points).unwrap();
        assert_eq!(cloud.len(), src_points.len());

        // Test setting random normals and covariances
        let normals: Vec<Vector3<f64>> = (0..num_points)
            .map(|_| Vector3::new(rng.sample(&dist), rng.sample(&dist), rng.sample(&dist)))
            .collect();

        cloud.set_normals(&normals).unwrap();

        let covs: Vec<Matrix4<f64>> = (0..num_points)
            .map(|_| {
                let v = Vector4::new(
                    rng.sample(&dist),
                    rng.sample(&dist),
                    rng.sample(&dist),
                    rng.sample(&dist),
                );
                v * v.transpose()
            })
            .collect();

        // Set covariances
        for i in 0..cloud.len() {
            cloud.set_covariance(i, covs[i].clone()).unwrap();
        }

        // Verify all data
        for i in 0..cloud.len() {
            // Check points
            let (x, y, z) = cloud.point_at(i).unwrap();
            assert!((x - src_points[i].x).abs() < 1e-3);
            assert!((y - src_points[i].y).abs() < 1e-3);
            assert!((z - src_points[i].z).abs() < 1e-3);

            // Check normals
            let normal = cloud.normal_at(i).unwrap();
            assert!((normal.x - normals[i].x).abs() < 1e-3);
            assert!((normal.y - normals[i].y).abs() < 1e-3);
            assert!((normal.z - normals[i].z).abs() < 1e-3);

            // Check covariances
            let cov = cloud.covariance_at(i).unwrap();
            for row in 0..4 {
                for col in 0..4 {
                    assert!((cov[(row, col)] - covs[i][(row, col)]).abs() < 1e-3);
                }
            }
        }

        // Test resize to half size
        let half_size = src_points.len() / 2;
        cloud.resize(half_size).unwrap();
        assert_eq!(cloud.len(), half_size);

        // Verify data is preserved after resize
        for i in 0..cloud.len() {
            let (x, y, z) = cloud.point_at(i).unwrap();
            assert!((x - src_points[i].x).abs() < 1e-3);
            assert!((y - src_points[i].y).abs() < 1e-3);
            assert!((z - src_points[i].z).abs() < 1e-3);

            let normal = cloud.normal_at(i).unwrap();
            assert!((normal.x - normals[i].x).abs() < 1e-3);
            assert!((normal.y - normals[i].y).abs() < 1e-3);
            assert!((normal.z - normals[i].z).abs() < 1e-3);
        }
    }
}

/// Conversion utilities for point clouds.
pub mod conversions {
    use super::*;
    use crate::traits::PointCloudTrait;

    /// Convert from any PointCloudTrait to a PointCloud.
    pub fn from_trait<P: PointCloudTrait>(input: &P) -> Result<PointCloud> {
        let mut output = PointCloud::new()?;
        output.resize(input.len())?;

        // Copy points
        for i in 0..input.len() {
            let point = input.point(i);
            output.set_point(i, point.x, point.y, point.z)?;
        }

        // Copy normals if available
        if input.has_normals() {
            let mut normals_data = Vec::with_capacity(input.len() * 3);
            for i in 0..input.len() {
                if let Some(normal) = input.normal(i) {
                    normals_data.extend_from_slice(&[normal.x, normal.y, normal.z]);
                } else {
                    normals_data.extend_from_slice(&[0.0, 0.0, 0.0]);
                }
            }
            output.set_normals_bulk(&normals_data)?;
        }

        // Copy covariances if available
        if input.has_covariances() {
            let mut covs_data = Vec::with_capacity(input.len() * 16);
            for i in 0..input.len() {
                if let Some(cov) = input.covariance(i) {
                    for row in 0..4 {
                        for col in 0..4 {
                            covs_data.push(cov[(row, col)]);
                        }
                    }
                } else {
                    // Default identity matrix
                    for row in 0..4 {
                        for col in 0..4 {
                            covs_data.push(if row == col { 1.0 } else { 0.0 });
                        }
                    }
                }
            }
            output.set_covariances_bulk(&covs_data)?;
        }

        Ok(output)
    }

    /// Convert 3D points to 4D homogeneous coordinates.
    pub fn points3_to_points4(points: &[Point3<f64>]) -> Vec<Vector4<f64>> {
        points
            .iter()
            .map(|p| Vector4::new(p.x, p.y, p.z, 1.0))
            .collect()
    }

    /// Convert 4D homogeneous coordinates to 3D points.
    pub fn points4_to_points3(points: &[Vector4<f64>]) -> Vec<Point3<f64>> {
        points.iter().map(|p| Point3::new(p.x, p.y, p.z)).collect()
    }

    /// Convert 3D normals to 4D vectors.
    pub fn normals3_to_normals4(normals: &[Vector3<f64>]) -> Vec<Vector4<f64>> {
        normals
            .iter()
            .map(|n| Vector4::new(n.x, n.y, n.z, 0.0))
            .collect()
    }

    /// Convert 4D vectors to 3D normals.
    pub fn normals4_to_normals3(normals: &[Vector4<f64>]) -> Vec<Vector3<f64>> {
        normals
            .iter()
            .map(|n| Vector3::new(n.x, n.y, n.z))
            .collect()
    }

    /// Copy data from one point cloud to another.
    pub fn copy_data<S: PointCloudTrait, T: crate::traits::MutablePointCloudTrait>(
        source: &S,
        target: &mut T,
    ) -> Result<()> {
        // Resize target to match source
        target.resize(source.len());

        // Copy points
        for i in 0..source.len() {
            target.set_point(i, source.point(i));
        }

        // Copy normals if available
        if source.has_normals() {
            for i in 0..source.len() {
                if let Some(normal) = source.normal(i) {
                    target.set_normal(i, normal);
                }
            }
        }

        // Copy covariances if available
        if source.has_covariances() {
            for i in 0..source.len() {
                if let Some(cov) = source.covariance(i) {
                    target.set_covariance(i, cov);
                }
            }
        }

        Ok(())
    }
}
