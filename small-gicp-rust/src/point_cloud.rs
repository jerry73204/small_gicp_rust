//! Point cloud data structures and operations.

use crate::error::{check_error, Result, SmallGicpError};
use nalgebra::{Point3, Vector3};
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

        new_cloud
    }
}
