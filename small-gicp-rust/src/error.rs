//! Error types for small_gicp operations.

use thiserror::Error;

/// Errors that can occur during small_gicp operations.
#[derive(Error, Debug)]
pub enum SmallGicpError {
    /// Invalid argument provided to a function.
    #[error("Invalid argument: {0}")]
    InvalidArgument(String),

    /// Index out of bounds error.
    #[error("Index {0} out of bounds (length: {1})")]
    IndexOutOfBounds(usize, usize),

    /// Feature not implemented error.
    #[error("Feature not implemented: {0}")]
    NotImplemented(String),

    /// FFI error from C++ library.
    #[error("FFI error: {0}")]
    FfiError(String),

    /// Point cloud is empty.
    #[error("Point cloud is empty")]
    EmptyPointCloud,

    /// Incompatible point cloud sizes.
    #[error("Incompatible point cloud sizes: source={0}, target={1}")]
    IncompatibleSizes(usize, usize),

    /// I/O error.
    #[error("IO error: {0}")]
    Io(String),

    /// Registration failed to converge.
    #[error("Registration failed to converge after {0} iterations")]
    ConvergenceFailure(usize),

    /// Invalid voxel size.
    #[error("Invalid voxel size: {0} (must be positive)")]
    InvalidVoxelSize(f64),

    /// Invalid number of neighbors.
    #[error("Invalid number of neighbors: {0} (must be positive)")]
    InvalidNeighborCount(i32),

    /// Matrix dimension mismatch.
    #[error("Matrix dimension mismatch: expected {0}, got {1}")]
    MatrixDimensionMismatch(String, String),

    /// Thread pool error.
    #[error("Thread pool error: {0}")]
    ThreadPoolError(String),

    /// Out of memory error.
    #[error("Out of memory")]
    OutOfMemory,

    /// File not found error.
    #[error("File not found: {0}")]
    FileNotFound(String),

    /// Internal C++ exception.
    #[error("Internal exception: {0}")]
    InternalException(String),

    /// Invalid parameter error.
    #[error("Invalid parameter '{0}': {1}")]
    InvalidParameter(&'static str, String),

    /// CXX bridge error.
    #[error("CXX bridge error: {0}")]
    CxxError(String),
}

// Conversion traits for common error types
impl From<String> for SmallGicpError {
    fn from(error: String) -> Self {
        Self::CxxError(error)
    }
}

impl From<std::io::Error> for SmallGicpError {
    fn from(error: std::io::Error) -> Self {
        Self::Io(error.to_string())
    }
}

/// Result type for small_gicp operations.
pub type Result<T> = std::result::Result<T, SmallGicpError>;

// FFI Validation Framework

/// Validate that a point cloud is not empty.
/// This function is generic to avoid circular dependencies.
pub fn validate_point_cloud_not_empty<T>(cloud: &T) -> Result<()>
where
    T: crate::traits::PointCloudTrait,
{
    if cloud.len() == 0 {
        Err(SmallGicpError::EmptyPointCloud)
    } else {
        Ok(())
    }
}

/// Validate index bounds.
pub fn validate_index_bounds(index: usize, length: usize) -> Result<()> {
    if index >= length {
        Err(SmallGicpError::IndexOutOfBounds(index, length))
    } else {
        Ok(())
    }
}

/// Validate voxel size is positive and finite.
pub fn validate_voxel_size(size: f64) -> Result<()> {
    if size <= 0.0 || !size.is_finite() {
        Err(SmallGicpError::InvalidVoxelSize(size))
    } else {
        Ok(())
    }
}

/// Validate neighbor count is positive.
pub fn validate_neighbor_count(neighbors: i32) -> Result<()> {
    if neighbors <= 0 {
        Err(SmallGicpError::InvalidNeighborCount(neighbors))
    } else {
        Ok(())
    }
}

/// Validate that two point clouds have compatible sizes.
pub fn validate_compatible_sizes(source_size: usize, target_size: usize) -> Result<()> {
    if source_size != target_size {
        Err(SmallGicpError::IncompatibleSizes(source_size, target_size))
    } else {
        Ok(())
    }
}

/// Helper function to check if a value is valid and return an error if not.
pub fn validate_parameter<T: std::fmt::Debug>(
    param_name: &'static str,
    value: T,
    predicate: impl Fn(&T) -> bool,
) -> Result<T> {
    if predicate(&value) {
        Ok(value)
    } else {
        Err(SmallGicpError::InvalidParameter(
            param_name,
            format!("{:?}", value),
        ))
    }
}

/// Helper function to validate point cloud is not empty (legacy).
pub fn validate_non_empty(size: usize) -> Result<()> {
    if size == 0 {
        Err(SmallGicpError::EmptyPointCloud)
    } else {
        Ok(())
    }
}

/// Helper function to validate index bounds (legacy).
pub fn validate_index(index: usize, size: usize) -> Result<()> {
    if index >= size {
        Err(SmallGicpError::IndexOutOfBounds(index, size))
    } else {
        Ok(())
    }
}

/// Helper function to validate that two collections have the same length.
pub fn validate_same_length(len1: usize, len2: usize, name1: &str, name2: &str) -> Result<()> {
    if len1 != len2 {
        Err(SmallGicpError::InvalidArgument(format!(
            "{} and {} must have the same length ({} vs {})",
            name1, name2, len1, len2
        )))
    } else {
        Ok(())
    }
}

/// Helper function to check C++ error codes and convert them to Rust errors.
/// Assumes error code 0 means success, non-zero means error.
pub fn check_error(error_code: i32) -> Result<()> {
    if error_code == 0 {
        Ok(())
    } else {
        Err(SmallGicpError::InternalException(format!(
            "C++ operation failed with error code: {}",
            error_code
        )))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_types() {
        let error = SmallGicpError::InvalidArgument("test".to_string());
        assert_eq!(error.to_string(), "Invalid argument: test");

        let error = SmallGicpError::IndexOutOfBounds(5, 3);
        assert_eq!(error.to_string(), "Index 5 out of bounds (length: 3)");

        let error = SmallGicpError::InvalidVoxelSize(-1.0);
        assert_eq!(
            error.to_string(),
            "Invalid voxel size: -1 (must be positive)"
        );

        let error = SmallGicpError::IncompatibleSizes(10, 20);
        assert_eq!(
            error.to_string(),
            "Incompatible point cloud sizes: source=10, target=20"
        );
    }

    #[test]
    fn test_validate_voxel_size() {
        assert!(validate_voxel_size(0.1).is_ok());
        assert!(validate_voxel_size(1.0).is_ok());

        assert!(validate_voxel_size(0.0).is_err());
        assert!(validate_voxel_size(-1.0).is_err());
        assert!(validate_voxel_size(f64::NAN).is_err());
        assert!(validate_voxel_size(f64::INFINITY).is_err());
    }

    #[test]
    fn test_validate_neighbor_count() {
        assert!(validate_neighbor_count(1).is_ok());
        assert!(validate_neighbor_count(10).is_ok());

        assert!(validate_neighbor_count(0).is_err());
        assert!(validate_neighbor_count(-1).is_err());
    }

    #[test]
    fn test_validate_index_bounds() {
        assert!(validate_index_bounds(0, 5).is_ok());
        assert!(validate_index_bounds(4, 5).is_ok());
        assert!(validate_index_bounds(5, 5).is_err());
    }

    #[test]
    fn test_validate_compatible_sizes() {
        assert!(validate_compatible_sizes(10, 10).is_ok());
        assert!(validate_compatible_sizes(5, 10).is_err());

        let error = validate_compatible_sizes(5, 10).unwrap_err();
        match error {
            SmallGicpError::IncompatibleSizes(source, target) => {
                assert_eq!(source, 5);
                assert_eq!(target, 10);
            }
            _ => panic!("Expected IncompatibleSizes error"),
        }
    }

    #[test]
    fn test_validate_parameter() {
        let result = validate_parameter("test_param", 5, |&x| x > 0);
        assert_eq!(result.unwrap(), 5);

        let result = validate_parameter("test_param", -1, |&x| x > 0);
        assert!(result.is_err());
    }

    #[test]
    fn test_validate_non_empty() {
        assert!(validate_non_empty(1).is_ok());
        assert!(validate_non_empty(0).is_err());
    }

    #[test]
    fn test_validate_index() {
        assert!(validate_index(0, 5).is_ok());
        assert!(validate_index(4, 5).is_ok());
        assert!(validate_index(5, 5).is_err());
    }

    #[test]
    fn test_validate_same_length() {
        assert!(validate_same_length(3, 3, "points", "normals").is_ok());
        assert!(validate_same_length(3, 4, "points", "normals").is_err());
    }

    #[test]
    fn test_error_conversions() {
        let io_error = std::io::Error::new(std::io::ErrorKind::NotFound, "file not found");
        let gicp_error: SmallGicpError = io_error.into();
        assert!(matches!(gicp_error, SmallGicpError::Io(_)));

        let string_error = "test error".to_string();
        let gicp_error: SmallGicpError = string_error.into();
        assert!(matches!(gicp_error, SmallGicpError::CxxError(_)));
    }
}
