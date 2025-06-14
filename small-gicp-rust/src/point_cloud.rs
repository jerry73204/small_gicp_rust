//! Point cloud data structures and operations.

use crate::{
    error::{check_error, Result, SmallGicpError},
    traits::{
        bounds, helpers, Covariance4, MutablePointCloudTrait, Normal4, Point4, PointCloudTrait,
    },
};
use nalgebra::{Matrix4, Point3, Vector3};
use std::{ffi::CString, ptr};

/// A 3D point cloud with optional normal vectors.
#[derive(Debug)]
pub struct PointCloud {
    pub(crate) handle: *mut small_gicp_sys::small_gicp_point_cloud_t,
}

impl PointCloud {
    /// Create a new empty point cloud.
    pub fn new() -> Result<Self> {
        let mut handle = ptr::null_mut();
        let error = unsafe { small_gicp_sys::small_gicp_point_cloud_create(&mut handle) };
        check_error(error)?;
        Ok(PointCloud { handle })
    }

    /// Create a point cloud from a vector of points.
    pub fn from_points(points: &[Point3<f64>]) -> Result<Self> {
        let mut cloud = Self::new()?;
        cloud.set_points(points)?;
        Ok(cloud)
    }

    /// Create a point cloud from vectors of points and normals.
    pub fn from_points_and_normals(
        points: &[Point3<f64>],
        normals: &[Vector3<f64>],
    ) -> Result<Self> {
        if points.len() != normals.len() {
            return Err(SmallGicpError::InvalidArgument(
                "Points and normals must have the same length".to_string(),
            ));
        }

        let mut cloud = Self::new()?;
        cloud.set_points(points)?;
        cloud.set_normals(normals)?;
        Ok(cloud)
    }

    /// Create a point cloud from a raw float array.
    /// The array should contain points as [x, y, z, x, y, z, ...].
    pub fn from_float_array(points: &[f32]) -> Result<Self> {
        if points.len() % 3 != 0 {
            return Err(SmallGicpError::InvalidArgument(
                "Points array length must be a multiple of 3".to_string(),
            ));
        }

        let mut handle = ptr::null_mut();
        let error = unsafe {
            small_gicp_sys::small_gicp_load_points_from_array(
                points.as_ptr(),
                points.len() / 3,
                &mut handle,
            )
        };
        check_error(error)?;
        Ok(PointCloud { handle })
    }

    /// Load a point cloud from a PLY file.
    pub fn from_ply(filename: &str) -> Result<Self> {
        let c_filename = CString::new(filename)
            .map_err(|_| SmallGicpError::InvalidArgument("Invalid filename".to_string()))?;

        let mut handle = ptr::null_mut();
        let error =
            unsafe { small_gicp_sys::small_gicp_load_ply(c_filename.as_ptr(), &mut handle) };

        match error {
            small_gicp_sys::small_gicp_error_t::SMALL_GICP_SUCCESS => Ok(PointCloud { handle }),
            small_gicp_sys::small_gicp_error_t::SMALL_GICP_ERROR_FILE_NOT_FOUND => {
                Err(SmallGicpError::FileNotFound(filename.to_string()))
            }
            _ => Err(SmallGicpError::from(error)),
        }
    }

    /// Save the point cloud to a PLY file.
    pub fn save_ply(&self, filename: &str) -> Result<()> {
        let c_filename = CString::new(filename)
            .map_err(|_| SmallGicpError::InvalidArgument("Invalid filename".to_string()))?;

        let error =
            unsafe { small_gicp_sys::small_gicp_save_ply(c_filename.as_ptr(), self.handle) };

        match error {
            small_gicp_sys::small_gicp_error_t::SMALL_GICP_SUCCESS => Ok(()),
            small_gicp_sys::small_gicp_error_t::SMALL_GICP_ERROR_IO_ERROR => Err(
                SmallGicpError::IoError(format!("Failed to save to {}", filename)),
            ),
            _ => Err(SmallGicpError::from(error)),
        }
    }

    /// Get the number of points in the cloud.
    pub fn len(&self) -> usize {
        let mut size = 0;
        let error = unsafe { small_gicp_sys::small_gicp_point_cloud_size(self.handle, &mut size) };

        if error == small_gicp_sys::small_gicp_error_t::SMALL_GICP_SUCCESS {
            size
        } else {
            0
        }
    }

    /// Check if the point cloud is empty.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Resize the point cloud to hold the specified number of points.
    pub fn resize(&mut self, size: usize) -> Result<()> {
        let error = unsafe { small_gicp_sys::small_gicp_point_cloud_resize(self.handle, size) };
        check_error(error)
    }

    /// Set all points in the cloud.
    pub fn set_points(&mut self, points: &[Point3<f64>]) -> Result<()> {
        self.resize(points.len())?;

        for (i, point) in points.iter().enumerate() {
            let error = unsafe {
                small_gicp_sys::small_gicp_point_cloud_set_point(
                    self.handle,
                    i,
                    point.x,
                    point.y,
                    point.z,
                )
            };
            check_error(error)?;
        }

        Ok(())
    }

    /// Set all normals in the cloud.
    pub fn set_normals(&mut self, normals: &[Vector3<f64>]) -> Result<()> {
        let cloud_size = self.len();
        if normals.len() != cloud_size {
            return Err(SmallGicpError::InvalidArgument(format!(
                "Expected {} normals, got {}",
                cloud_size,
                normals.len()
            )));
        }

        for (i, normal) in normals.iter().enumerate() {
            let error = unsafe {
                small_gicp_sys::small_gicp_point_cloud_set_normal(
                    self.handle,
                    i,
                    normal.x,
                    normal.y,
                    normal.z,
                )
            };
            check_error(error)?;
        }

        Ok(())
    }

    /// Get a point by index.
    pub fn get_point(&self, index: usize) -> Result<Point3<f64>> {
        if index >= self.len() {
            return Err(SmallGicpError::IndexOutOfBounds {
                index,
                size: self.len(),
            });
        }

        let mut x = 0.0;
        let mut y = 0.0;
        let mut z = 0.0;
        let error = unsafe {
            small_gicp_sys::small_gicp_point_cloud_get_point(
                self.handle,
                index,
                &mut x,
                &mut y,
                &mut z,
            )
        };
        check_error(error)?;
        Ok(Point3::new(x, y, z))
    }

    /// Get a normal by index.
    pub fn get_normal(&self, index: usize) -> Result<Vector3<f64>> {
        if index >= self.len() {
            return Err(SmallGicpError::IndexOutOfBounds {
                index,
                size: self.len(),
            });
        }

        let mut x = 0.0;
        let mut y = 0.0;
        let mut z = 0.0;
        let error = unsafe {
            small_gicp_sys::small_gicp_point_cloud_get_normal(
                self.handle,
                index,
                &mut x,
                &mut y,
                &mut z,
            )
        };
        check_error(error)?;
        Ok(Vector3::new(x, y, z))
    }

    /// Get all points in the cloud.
    pub fn points(&self) -> Result<Vec<Point3<f64>>> {
        let size = self.len();
        let mut points = Vec::with_capacity(size);

        for i in 0..size {
            points.push(self.get_point(i)?);
        }

        Ok(points)
    }

    /// Get all normals in the cloud.
    pub fn normals(&self) -> Result<Vec<Vector3<f64>>> {
        let size = self.len();
        let mut normals = Vec::with_capacity(size);

        for i in 0..size {
            normals.push(self.get_normal(i)?);
        }

        Ok(normals)
    }

    /// Set a single point by index.
    pub fn set_point(&mut self, index: usize, point: Point3<f64>) -> Result<()> {
        if index >= self.len() {
            return Err(SmallGicpError::IndexOutOfBounds {
                index,
                size: self.len(),
            });
        }

        let error = unsafe {
            small_gicp_sys::small_gicp_point_cloud_set_point(
                self.handle,
                index,
                point.x,
                point.y,
                point.z,
            )
        };
        check_error(error)
    }

    /// Set a single normal by index.
    pub fn set_normal(&mut self, index: usize, normal: Vector3<f64>) -> Result<()> {
        if index >= self.len() {
            return Err(SmallGicpError::IndexOutOfBounds {
                index,
                size: self.len(),
            });
        }

        let error = unsafe {
            small_gicp_sys::small_gicp_point_cloud_set_normal(
                self.handle,
                index,
                normal.x,
                normal.y,
                normal.z,
            )
        };
        check_error(error)
    }

    /// Set a covariance matrix by index.
    pub fn set_covariance(&mut self, index: usize, covariance: Matrix4<f64>) -> Result<()> {
        if index >= self.len() {
            return Err(SmallGicpError::IndexOutOfBounds {
                index,
                size: self.len(),
            });
        }

        // Convert Matrix4 to row-major array
        let cov_array: [f64; 16] = [
            covariance[(0, 0)],
            covariance[(0, 1)],
            covariance[(0, 2)],
            covariance[(0, 3)],
            covariance[(1, 0)],
            covariance[(1, 1)],
            covariance[(1, 2)],
            covariance[(1, 3)],
            covariance[(2, 0)],
            covariance[(2, 1)],
            covariance[(2, 2)],
            covariance[(2, 3)],
            covariance[(3, 0)],
            covariance[(3, 1)],
            covariance[(3, 2)],
            covariance[(3, 3)],
        ];

        let error = unsafe {
            small_gicp_sys::small_gicp_point_cloud_set_covariance(
                self.handle,
                index,
                cov_array.as_ptr(),
            )
        };
        check_error(error)
    }

    /// Get a covariance matrix by index.
    pub fn get_covariance(&self, index: usize) -> Result<Matrix4<f64>> {
        if index >= self.len() {
            return Err(SmallGicpError::IndexOutOfBounds {
                index,
                size: self.len(),
            });
        }

        let mut cov_array = [0.0f64; 16];
        let error = unsafe {
            small_gicp_sys::small_gicp_point_cloud_get_covariance(
                self.handle,
                index,
                cov_array.as_mut_ptr(),
            )
        };
        check_error(error)?;

        // Convert row-major array back to Matrix4
        Ok(Matrix4::new(
            cov_array[0],
            cov_array[1],
            cov_array[2],
            cov_array[3],
            cov_array[4],
            cov_array[5],
            cov_array[6],
            cov_array[7],
            cov_array[8],
            cov_array[9],
            cov_array[10],
            cov_array[11],
            cov_array[12],
            cov_array[13],
            cov_array[14],
            cov_array[15],
        ))
    }

    /// Set all covariances in the cloud.
    pub fn set_covariances(&mut self, covariances: &[Matrix4<f64>]) -> Result<()> {
        let cloud_size = self.len();
        if covariances.len() != cloud_size {
            return Err(SmallGicpError::InvalidArgument(format!(
                "Expected {} covariances, got {}",
                cloud_size,
                covariances.len()
            )));
        }

        for (i, covariance) in covariances.iter().enumerate() {
            self.set_covariance(i, *covariance)?;
        }

        Ok(())
    }

    /// Get all covariances in the cloud.
    pub fn covariances(&self) -> Result<Vec<Matrix4<f64>>> {
        let size = self.len();
        let mut covariances = Vec::with_capacity(size);

        for i in 0..size {
            covariances.push(self.get_covariance(i)?);
        }

        Ok(covariances)
    }

    /// Check if the point cloud has point data.
    pub fn has_points(&self) -> bool {
        let mut has_points = false;
        let error = unsafe {
            small_gicp_sys::small_gicp_point_cloud_has_points(self.handle, &mut has_points)
        };

        if error == small_gicp_sys::small_gicp_error_t::SMALL_GICP_SUCCESS {
            has_points
        } else {
            false
        }
    }

    /// Check if the point cloud has normal data.
    pub fn has_normals(&self) -> bool {
        let mut has_normals = false;
        let error = unsafe {
            small_gicp_sys::small_gicp_point_cloud_has_normals(self.handle, &mut has_normals)
        };

        if error == small_gicp_sys::small_gicp_error_t::SMALL_GICP_SUCCESS {
            has_normals
        } else {
            false
        }
    }

    /// Check if the point cloud has covariance data.
    pub fn has_covariances(&self) -> bool {
        let mut has_covariances = false;
        let error = unsafe {
            small_gicp_sys::small_gicp_point_cloud_has_covariances(
                self.handle,
                &mut has_covariances,
            )
        };

        if error == small_gicp_sys::small_gicp_error_t::SMALL_GICP_SUCCESS {
            has_covariances
        } else {
            false
        }
    }

    /// Check if the point cloud is empty (alternative implementation).
    pub fn empty(&self) -> bool {
        let mut is_empty = true;
        let error =
            unsafe { small_gicp_sys::small_gicp_point_cloud_empty(self.handle, &mut is_empty) };

        if error == small_gicp_sys::small_gicp_error_t::SMALL_GICP_SUCCESS {
            is_empty
        } else {
            true
        }
    }

    /// Set points from a raw slice efficiently.
    pub fn set_points_bulk(&mut self, points: &[f64]) -> Result<()> {
        if points.len() % 3 != 0 {
            return Err(SmallGicpError::InvalidArgument(
                "Points array length must be a multiple of 3".to_string(),
            ));
        }

        let num_points = points.len() / 3;
        let error = unsafe {
            small_gicp_sys::small_gicp_point_cloud_set_points_bulk(
                self.handle,
                points.as_ptr(),
                num_points,
            )
        };
        check_error(error)
    }

    /// Set normals from a raw slice efficiently.
    pub fn set_normals_bulk(&mut self, normals: &[f64]) -> Result<()> {
        if normals.len() % 3 != 0 {
            return Err(SmallGicpError::InvalidArgument(
                "Normals array length must be a multiple of 3".to_string(),
            ));
        }

        let num_points = normals.len() / 3;
        let cloud_size = self.len();
        if num_points != cloud_size {
            return Err(SmallGicpError::InvalidArgument(format!(
                "Expected {} normals, got {}",
                cloud_size, num_points
            )));
        }

        let error = unsafe {
            small_gicp_sys::small_gicp_point_cloud_set_normals_bulk(
                self.handle,
                normals.as_ptr(),
                num_points,
            )
        };
        check_error(error)
    }

    /// Set covariances from a raw slice efficiently.
    pub fn set_covariances_bulk(&mut self, covariances: &[f64]) -> Result<()> {
        if covariances.len() % 16 != 0 {
            return Err(SmallGicpError::InvalidArgument(
                "Covariances array length must be a multiple of 16".to_string(),
            ));
        }

        let num_points = covariances.len() / 16;
        let cloud_size = self.len();
        if num_points != cloud_size {
            return Err(SmallGicpError::InvalidArgument(format!(
                "Expected {} covariances, got {}",
                cloud_size, num_points
            )));
        }

        let error = unsafe {
            small_gicp_sys::small_gicp_point_cloud_set_covariances_bulk(
                self.handle,
                covariances.as_ptr(),
                num_points,
            )
        };
        check_error(error)
    }

    /// Copy points to a pre-allocated array.
    pub fn copy_points_to_array(&self, points_array: &mut [f64]) -> Result<()> {
        let expected_size = self.len() * 3;
        if points_array.len() != expected_size {
            return Err(SmallGicpError::InvalidArgument(format!(
                "Array size must be {} (3 * num_points), got {}",
                expected_size,
                points_array.len()
            )));
        }

        let error = unsafe {
            small_gicp_sys::small_gicp_point_cloud_copy_points_to_array(
                self.handle,
                points_array.as_mut_ptr(),
                points_array.len(),
            )
        };
        check_error(error)
    }

    /// Copy normals to a pre-allocated array.
    pub fn copy_normals_to_array(&self, normals_array: &mut [f64]) -> Result<()> {
        let expected_size = self.len() * 3;
        if normals_array.len() != expected_size {
            return Err(SmallGicpError::InvalidArgument(format!(
                "Array size must be {} (3 * num_points), got {}",
                expected_size,
                normals_array.len()
            )));
        }

        let error = unsafe {
            small_gicp_sys::small_gicp_point_cloud_copy_normals_to_array(
                self.handle,
                normals_array.as_mut_ptr(),
                normals_array.len(),
            )
        };
        check_error(error)
    }

    /// Copy covariances to a pre-allocated array.
    pub fn copy_covariances_to_array(&self, covariances_array: &mut [f64]) -> Result<()> {
        let expected_size = self.len() * 16;
        if covariances_array.len() != expected_size {
            return Err(SmallGicpError::InvalidArgument(format!(
                "Array size must be {} (16 * num_points), got {}",
                expected_size,
                covariances_array.len()
            )));
        }

        let error = unsafe {
            small_gicp_sys::small_gicp_point_cloud_copy_covariances_to_array(
                self.handle,
                covariances_array.as_mut_ptr(),
                covariances_array.len(),
            )
        };
        check_error(error)
    }

    /// Get direct access to the points data.
    ///
    /// # Safety
    /// The returned slice is valid only as long as the point cloud exists
    /// and is not modified. This provides zero-copy access for performance-critical code.
    pub unsafe fn points_data(&self) -> Result<&[f64]> {
        let mut data_ptr = ptr::null_mut();
        let mut data_size = 0;

        let error = small_gicp_sys::small_gicp_point_cloud_get_points_data(
            self.handle,
            &mut data_ptr,
            &mut data_size,
        );
        check_error(error)?;

        if data_ptr.is_null() || data_size == 0 {
            Ok(&[])
        } else {
            Ok(std::slice::from_raw_parts(data_ptr, data_size))
        }
    }

    /// Get direct access to the normals data.
    ///
    /// # Safety
    /// The returned slice is valid only as long as the point cloud exists
    /// and is not modified. This provides zero-copy access for performance-critical code.
    pub unsafe fn normals_data(&self) -> Result<&[f64]> {
        let mut data_ptr = ptr::null_mut();
        let mut data_size = 0;

        let error = small_gicp_sys::small_gicp_point_cloud_get_normals_data(
            self.handle,
            &mut data_ptr,
            &mut data_size,
        );
        check_error(error)?;

        if data_ptr.is_null() || data_size == 0 {
            Ok(&[])
        } else {
            Ok(std::slice::from_raw_parts(data_ptr, data_size))
        }
    }

    /// Get direct access to the covariances data.
    ///
    /// # Safety
    /// The returned slice is valid only as long as the point cloud exists
    /// and is not modified. This provides zero-copy access for performance-critical code.
    pub unsafe fn covariances_data(&self) -> Result<&[f64]> {
        let mut data_ptr = ptr::null_mut();
        let mut data_size = 0;

        let error = small_gicp_sys::small_gicp_point_cloud_get_covariances_data(
            self.handle,
            &mut data_ptr,
            &mut data_size,
        );
        check_error(error)?;

        if data_ptr.is_null() || data_size == 0 {
            Ok(&[])
        } else {
            Ok(std::slice::from_raw_parts(data_ptr, data_size))
        }
    }
}

impl Default for PointCloud {
    fn default() -> Self {
        Self::new().expect("Failed to create default point cloud")
    }
}

impl Drop for PointCloud {
    fn drop(&mut self) {
        if !self.handle.is_null() {
            unsafe {
                small_gicp_sys::small_gicp_point_cloud_destroy(self.handle);
            }
        }
    }
}

// Ensure PointCloud is Send and Sync
unsafe impl Send for PointCloud {}
unsafe impl Sync for PointCloud {}

impl Clone for PointCloud {
    fn clone(&self) -> Self {
        let mut new_cloud = PointCloud::new().expect("Failed to create new point cloud");

        if let Ok(points) = self.points() {
            new_cloud.set_points(&points).expect("Failed to set points");
        }

        if let Ok(normals) = self.normals() {
            new_cloud
                .set_normals(&normals)
                .expect("Failed to set normals");
        }

        if let Ok(covariances) = self.covariances() {
            new_cloud
                .set_covariances(&covariances)
                .expect("Failed to set covariances");
        }

        new_cloud
    }
}

// Implementation of PointCloudTrait for the C wrapper PointCloud
impl PointCloudTrait for PointCloud {
    fn size(&self) -> usize {
        self.len()
    }

    fn has_points(&self) -> bool {
        self.has_points()
    }

    fn has_normals(&self) -> bool {
        self.has_normals()
    }

    fn has_covariances(&self) -> bool {
        self.has_covariances()
    }

    fn point(&self, index: usize) -> Point4<f64> {
        bounds::check_bounds(index, self.size(), "point access");
        match self.get_point(index) {
            Ok(p3) => helpers::point_from_vector3(p3.coords),
            Err(_) => panic!("Failed to get point at index {}", index),
        }
    }

    fn normal(&self, index: usize) -> Option<Normal4<f64>> {
        if !self.has_normals() {
            return None;
        }
        bounds::check_bounds(index, self.size(), "normal access");
        match self.get_normal(index) {
            Ok(n3) => Some(helpers::normal_from_vector3(n3)),
            Err(_) => None,
        }
    }

    fn covariance(&self, index: usize) -> Option<Covariance4<f64>> {
        if !self.has_covariances() {
            return None;
        }
        bounds::check_bounds(index, self.size(), "covariance access");
        match self.get_covariance(index) {
            Ok(cov) => Some(cov),
            Err(_) => None,
        }
    }

    unsafe fn point_unchecked(&self, index: usize) -> Point4<f64> {
        // For C wrapper, we still need bounds checking since C API doesn't guarantee safety
        self.point(index)
    }

    unsafe fn normal_unchecked(&self, index: usize) -> Normal4<f64> {
        self.normal(index)
            .unwrap_or_else(|| helpers::normal_from_xyz(0.0, 0.0, 0.0))
    }

    unsafe fn covariance_unchecked(&self, index: usize) -> Covariance4<f64> {
        self.covariance(index).unwrap_or_else(|| Matrix4::zeros())
    }
}

// Implementation of MutablePointCloudTrait for the C wrapper PointCloud
impl MutablePointCloudTrait for PointCloud {
    fn resize(&mut self, size: usize) {
        PointCloud::resize(self, size).expect("Failed to resize point cloud");
    }

    fn set_point(&mut self, index: usize, point: Point4<f64>) {
        bounds::check_bounds(index, self.size(), "set point");
        if !helpers::is_valid_point(point) {
            panic!(
                "Invalid point: fourth component must be 1.0, got {}",
                point.w
            );
        }
        let p3 = Point3::new(point.x, point.y, point.z);
        PointCloud::set_point(self, index, p3).expect("Failed to set point");
    }

    fn set_normal(&mut self, index: usize, normal: Normal4<f64>) {
        bounds::check_bounds(index, self.size(), "set normal");
        if !helpers::is_valid_normal(normal) {
            panic!(
                "Invalid normal: fourth component must be 0.0, got {}",
                normal.w
            );
        }
        let n3 = Vector3::new(normal.x, normal.y, normal.z);
        PointCloud::set_normal(self, index, n3).expect("Failed to set normal");
    }

    fn set_covariance(&mut self, index: usize, covariance: Covariance4<f64>) {
        bounds::check_bounds(index, self.size(), "set covariance");
        PointCloud::set_covariance(self, index, covariance).expect("Failed to set covariance");
    }

    unsafe fn set_point_unchecked(&mut self, index: usize, point: Point4<f64>) {
        // For C wrapper, we still need bounds checking since C API doesn't guarantee safety
        <Self as MutablePointCloudTrait>::set_point(self, index, point);
    }

    unsafe fn set_normal_unchecked(&mut self, index: usize, normal: Normal4<f64>) {
        <Self as MutablePointCloudTrait>::set_normal(self, index, normal);
    }

    unsafe fn set_covariance_unchecked(&mut self, index: usize, covariance: Covariance4<f64>) {
        <Self as MutablePointCloudTrait>::set_covariance(self, index, covariance);
    }
}

/// Conversion utilities between trait types and C wrapper types.
pub mod conversions {
    use super::*;
    use crate::traits::{helpers, MutablePointCloudTrait, PointCloudTrait};

    /// Convert a slice of Point3 to Point4 vectors.
    pub fn points3_to_points4(points: &[Point3<f64>]) -> Vec<Point4<f64>> {
        points
            .iter()
            .map(|p| helpers::point_from_vector3(p.coords))
            .collect()
    }

    /// Convert a slice of Point4 vectors to Point3.
    pub fn points4_to_points3(points: &[Point4<f64>]) -> Vec<Point3<f64>> {
        points.iter().map(|p| Point3::new(p.x, p.y, p.z)).collect()
    }

    /// Convert a slice of Vector3 normals to Normal4 vectors.
    pub fn normals3_to_normals4(normals: &[Vector3<f64>]) -> Vec<Normal4<f64>> {
        normals
            .iter()
            .map(|n| helpers::normal_from_vector3(*n))
            .collect()
    }

    /// Convert a slice of Normal4 vectors to Vector3.
    pub fn normals4_to_normals3(normals: &[Normal4<f64>]) -> Vec<Vector3<f64>> {
        normals
            .iter()
            .map(|n| Vector3::new(n.x, n.y, n.z))
            .collect()
    }

    /// Create a PointCloud from any type implementing PointCloudTrait.
    pub fn from_trait<P: PointCloudTrait>(source: &P) -> Result<PointCloud> {
        let mut cloud = PointCloud::new()?;
        cloud.resize(source.size())?;

        // Copy points
        for i in 0..source.size() {
            let point4 = source.point(i);
            let point3 = Point3::new(point4.x, point4.y, point4.z);
            cloud.set_point(i, point3)?;
        }

        // Copy normals if available
        if source.has_normals() {
            for i in 0..source.size() {
                if let Some(normal4) = source.normal(i) {
                    let normal3 = Vector3::new(normal4.x, normal4.y, normal4.z);
                    cloud.set_normal(i, normal3)?;
                }
            }
        }

        // Copy covariances if available
        if source.has_covariances() {
            for i in 0..source.size() {
                if let Some(cov) = source.covariance(i) {
                    cloud.set_covariance(i, cov)?;
                }
            }
        }

        Ok(cloud)
    }

    /// Copy data from any PointCloudTrait to a MutablePointCloudTrait.
    pub fn copy_data<S: PointCloudTrait, T: MutablePointCloudTrait>(source: &S, target: &mut T) {
        target.resize(source.size());

        // Copy points
        for i in 0..source.size() {
            target.set_point(i, source.point(i));
        }

        // Copy normals if both support them
        if source.has_normals() {
            for i in 0..source.size() {
                if let Some(normal) = source.normal(i) {
                    target.set_normal(i, normal);
                }
            }
        }

        // Copy covariances if both support them
        if source.has_covariances() {
            for i in 0..source.size() {
                if let Some(cov) = source.covariance(i) {
                    target.set_covariance(i, cov);
                }
            }
        }
    }
}
