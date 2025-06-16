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

impl<'a> Iterator for PointsView<'a> {
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

impl<'a> ExactSizeIterator for PointsView<'a> {}

impl<'a> DoubleEndedIterator for PointsView<'a> {
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

impl<'a> Iterator for NormalsView<'a> {
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

impl<'a> ExactSizeIterator for NormalsView<'a> {}

impl<'a> DoubleEndedIterator for NormalsView<'a> {
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

impl<'a> Iterator for CovariancesView<'a> {
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

impl<'a> ExactSizeIterator for CovariancesView<'a> {}

impl<'a> DoubleEndedIterator for CovariancesView<'a> {
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
/// This is a high-level Rust wrapper around the small-gicp-cxx PointCloud.
/// It provides a trait-based interface for working with point clouds in a
/// type-safe and ergonomic way.
pub struct PointCloud {
    inner: small_gicp_cxx::PointCloud,
}

impl PointCloud {
    /// Create a new empty point cloud.
    pub fn new() -> Result<Self> {
        trace!("Creating new empty PointCloud");
        let inner = small_gicp_cxx::PointCloud::new();
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

    /// Access the underlying small-gicp-cxx PointCloud.
    pub(crate) fn inner(&self) -> &small_gicp_cxx::PointCloud {
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
    pub fn get_point(&self, index: usize) -> Result<(f64, f64, f64)> {
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
        let transform_struct = small_gicp_cxx::Transform { matrix };
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
        let transform_struct = small_gicp_cxx::Transform { matrix };
        Ok(Self {
            inner: self.inner.transformed(&transform_struct),
        })
    }

    /// Access the underlying small-gicp-cxx PointCloud mutably.
    /// This is for internal use only and should not be exposed to users.
    pub(crate) fn inner_mut(&mut self) -> &mut small_gicp_cxx::PointCloud {
        &mut self.inner
    }

    /// Create a PointCloud from a small-gicp-cxx PointCloud.
    /// This is for internal use only and should not be exposed to users.
    pub(crate) fn from_cxx(inner: small_gicp_cxx::PointCloud) -> Self {
        Self { inner }
    }

    /// Convert to a small-gicp-cxx PointCloud.
    /// This is for internal use only and should not be exposed to users.
    pub(crate) fn into_cxx(self) -> small_gicp_cxx::PointCloud {
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
    pub fn get_covariance(&self, index: usize) -> Result<Matrix4<f64>> {
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
    pub fn get_normal(&self, index: usize) -> Result<Vector3<f64>> {
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
        let inner = small_gicp_cxx::Io::load_ply(path)
            .map_err(|e| crate::error::SmallGicpError::IoError(e))?;
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
        small_gicp_cxx::Io::save_ply(path, &self.inner)
            .map_err(|e| crate::error::SmallGicpError::IoError(e))?;
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
// This is safe because small_gicp_cxx::PointCloud manages its own memory safety
// and the underlying C++ implementation is thread-safe for read operations
unsafe impl Send for PointCloud {}
unsafe impl Sync for PointCloud {}

// TODO: Implement trait implementations for PointCloudTrait and MutablePointCloudTrait
// These will provide the generic interface for the point cloud.

impl PointCloudTrait for PointCloud {
    fn size(&self) -> usize {
        self.len()
    }

    fn point(&self, i: usize) -> Point4<f64> {
        let (x, y, z) = self.get_point(i).unwrap_or((0.0, 0.0, 0.0));
        Vector4::new(x, y, z, 1.0)
    }

    fn normal(&self, i: usize) -> Option<Normal4<f64>> {
        self.get_normal(i)
            .ok()
            .map(|n| Vector4::new(n.x, n.y, n.z, 0.0))
    }

    fn covariance(&self, i: usize) -> Option<Covariance4<f64>> {
        self.get_covariance(i).ok()
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
mod tests {
    use super::*;

    #[test]
    fn test_points_view() {
        let points = vec![
            Point3::new(1.0, 2.0, 3.0),
            Point3::new(4.0, 5.0, 6.0),
            Point3::new(7.0, 8.0, 9.0),
        ];
        let cloud = PointCloud::from_points(&points).unwrap();
        let view = cloud.points();

        // Test iterator
        let collected: Vec<_> = view.clone().collect();
        assert_eq!(collected.len(), 3);
        assert_eq!(collected[0], Point3::new(1.0, 2.0, 3.0));

        // Test random access
        assert_eq!(view.get(0), Some(Point3::new(1.0, 2.0, 3.0)));
        assert_eq!(view.get(1), Some(Point3::new(4.0, 5.0, 6.0)));
        assert_eq!(view.get(3), None);

        // Test size methods
        assert_eq!(view.len(), 3);
        assert!(!view.is_empty());

        // Test first/last
        assert_eq!(view.first(), Some(Point3::new(1.0, 2.0, 3.0)));
        assert_eq!(view.last(), Some(Point3::new(7.0, 8.0, 9.0)));
    }

    #[test]
    fn test_raw_data_access() {
        let points = vec![Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 5.0, 6.0)];
        let cloud = PointCloud::from_points(&points).unwrap();

        // Test raw points data
        let raw_data = cloud.points_raw();
        assert_eq!(raw_data.len(), 8); // 2 points * 4 components each
        assert_eq!(raw_data[0], 1.0); // x of first point
        assert_eq!(raw_data[1], 2.0); // y of first point
        assert_eq!(raw_data[2], 3.0); // z of first point
        assert_eq!(raw_data[3], 1.0); // w of first point

        // Test XYZW array access
        let xyzw_data = cloud.points_raw_xyzw();
        assert_eq!(xyzw_data.len(), 2);
        assert_eq!(xyzw_data[0], [1.0, 2.0, 3.0, 1.0]);
        assert_eq!(xyzw_data[1], [4.0, 5.0, 6.0, 1.0]);

        // Test to_points_vec
        let points_vec = cloud.to_points_vec();
        assert_eq!(points_vec, points);
    }

    #[test]
    fn test_view_slicing() {
        let points = vec![
            Point3::new(1.0, 2.0, 3.0),
            Point3::new(4.0, 5.0, 6.0),
            Point3::new(7.0, 8.0, 9.0),
            Point3::new(10.0, 11.0, 12.0),
        ];
        let cloud = PointCloud::from_points(&points).unwrap();
        let view = cloud.points();

        // Test slicing
        let slice = view.slice(1..3);
        assert_eq!(slice.len(), 2);
        assert_eq!(slice.get(0), Some(Point3::new(4.0, 5.0, 6.0)));
        assert_eq!(slice.get(1), Some(Point3::new(7.0, 8.0, 9.0)));

        // Test empty slice
        let empty_slice = view.slice(2..2);
        assert!(empty_slice.is_empty());
    }

    #[test]
    fn test_point_cloud_creation() {
        // TODO: Implement basic tests once the API is functional
        // let cloud = PointCloud::new();
        // assert_eq!(cloud.len(), 0);
        // assert!(cloud.is_empty());
    }

    #[test]
    fn test_point_cloud_from_points() {
        // TODO: Implement tests for creating point clouds from data
        // let points = vec![
        //     Point3::new(1.0, 2.0, 3.0),
        //     Point3::new(4.0, 5.0, 6.0),
        // ];
        // let cloud = PointCloud::from_points(&points).unwrap();
        // assert_eq!(cloud.len(), 2);
    }

    #[test]
    fn test_normal_estimation() {
        // TODO: Implement tests for normal estimation
        // let points = create_test_points();
        // let mut cloud = PointCloud::from_points(&points).unwrap();
        // cloud.estimate_normals(5).unwrap();
        // assert!(cloud.has_normals());
    }

    #[test]
    fn test_transformation() {
        // TODO: Implement tests for transformations
        // let points = create_test_points();
        // let mut cloud = PointCloud::from_points(&points).unwrap();
        // let transform = Matrix4::identity();
        // cloud.transform(&transform).unwrap();
    }
}

/// Conversion utilities for point clouds.
pub mod conversions {
    use super::*;
    use crate::traits::PointCloudTrait;

    /// Convert from any PointCloudTrait to a PointCloud.
    pub fn from_trait<P: PointCloudTrait>(input: &P) -> Result<PointCloud> {
        let mut output = PointCloud::new()?;
        output.resize(input.size())?;

        // Copy points
        for i in 0..input.size() {
            let point = input.point(i);
            output.set_point(i, point.x, point.y, point.z)?;
        }

        // Copy normals if available
        if input.has_normals() {
            let mut normals_data = Vec::with_capacity(input.size() * 4);
            for i in 0..input.size() {
                if let Some(normal) = input.normal(i) {
                    normals_data.extend_from_slice(&[normal.x, normal.y, normal.z, normal.w]);
                } else {
                    normals_data.extend_from_slice(&[0.0, 0.0, 0.0, 0.0]);
                }
            }
            output.set_normals_bulk(&normals_data)?;
        }

        // Copy covariances if available
        if input.has_covariances() {
            let mut covs_data = Vec::with_capacity(input.size() * 16);
            for i in 0..input.size() {
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
        target.resize(source.size());

        // Copy points
        for i in 0..source.size() {
            target.set_point(i, source.point(i));
        }

        // Copy normals if available
        if source.has_normals() {
            for i in 0..source.size() {
                if let Some(normal) = source.normal(i) {
                    target.set_normal(i, normal);
                }
            }
        }

        // Copy covariances if available
        if source.has_covariances() {
            for i in 0..source.size() {
                if let Some(cov) = source.covariance(i) {
                    target.set_covariance(i, cov);
                }
            }
        }

        Ok(())
    }
}
