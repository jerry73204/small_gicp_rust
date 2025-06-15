//! Error types for small_gicp operations.

use thiserror::Error;

/// Errors that can occur during small_gicp operations.
#[derive(Error, Debug, Clone, PartialEq)]
pub enum SmallGicpError {
    /// Invalid argument provided to a function.
    #[error("Invalid argument: {0}")]
    InvalidArgument(String),

    /// Out of memory error.
    #[error("Out of memory")]
    OutOfMemory,

    /// File not found error.
    #[error("File not found: {0}")]
    FileNotFound(String),

    /// I/O error.
    #[error("I/O error: {0}")]
    IoError(String),

    /// Internal C++ exception.
    #[error("Internal exception: {0}")]
    InternalException(String),

    /// Point cloud is empty.
    #[error("Point cloud is empty")]
    EmptyPointCloud,

    /// Index out of bounds.
    #[error("Index {index} out of bounds (size: {size})")]
    IndexOutOfBounds { index: usize, size: usize },

    /// Registration failed to converge.
    #[error("Registration failed to converge after {iterations} iterations")]
    RegistrationFailed { iterations: i32 },

    /// Not implemented error.
    #[error("Not implemented: {0}")]
    NotImplemented(String),

    /// Invalid parameter error.
    #[error("Invalid parameter '{param}': {value}")]
    InvalidParameter { param: &'static str, value: String },

    /// CXX bridge error.
    #[error("CXX bridge error: {0}")]
    CxxError(String),
}

// TODO: Implement conversion from small-gicp-cxx error types
// The small-gicp-cxx crate uses String errors, so we need to convert them
// impl From<String> for SmallGicpError {
//     fn from(error: String) -> Self {
//         Self::CxxError(error)
//     }
// }

impl From<std::io::Error> for SmallGicpError {
    fn from(error: std::io::Error) -> Self {
        Self::IoError(error.to_string())
    }
}

/// Result type for small_gicp operations.
pub type Result<T> = std::result::Result<T, SmallGicpError>;

/// Helper function to check if a value is valid and return an error if not.
pub fn validate_parameter<T: std::fmt::Debug>(
    param_name: &'static str,
    value: T,
    predicate: impl Fn(&T) -> bool,
) -> Result<T> {
    if predicate(&value) {
        Ok(value)
    } else {
        Err(SmallGicpError::InvalidParameter {
            param: param_name,
            value: format!("{:?}", value),
        })
    }
}

/// Helper function to validate point cloud is not empty.
pub fn validate_non_empty(size: usize) -> Result<()> {
    if size == 0 {
        Err(SmallGicpError::EmptyPointCloud)
    } else {
        Ok(())
    }
}

/// Helper function to validate index bounds.
pub fn validate_index(index: usize, size: usize) -> Result<()> {
    if index >= size {
        Err(SmallGicpError::IndexOutOfBounds { index, size })
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

        let error = SmallGicpError::IndexOutOfBounds { index: 5, size: 3 };
        assert_eq!(error.to_string(), "Index 5 out of bounds (size: 3)");
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
}
