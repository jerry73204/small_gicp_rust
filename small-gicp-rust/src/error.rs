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
}

impl From<small_gicp_sys::small_gicp_error_t> for SmallGicpError {
    fn from(error: small_gicp_sys::small_gicp_error_t) -> Self {
        match error {
            small_gicp_sys::small_gicp_error_t::SMALL_GICP_SUCCESS => {
                unreachable!("Success is not an error")
            }
            small_gicp_sys::small_gicp_error_t::SMALL_GICP_ERROR_INVALID_ARGUMENT => {
                SmallGicpError::InvalidArgument("Unknown".to_string())
            }
            small_gicp_sys::small_gicp_error_t::SMALL_GICP_ERROR_OUT_OF_MEMORY => {
                SmallGicpError::OutOfMemory
            }
            small_gicp_sys::small_gicp_error_t::SMALL_GICP_ERROR_FILE_NOT_FOUND => {
                SmallGicpError::FileNotFound("Unknown".to_string())
            }
            small_gicp_sys::small_gicp_error_t::SMALL_GICP_ERROR_IO_ERROR => {
                SmallGicpError::IoError("Unknown".to_string())
            }
            small_gicp_sys::small_gicp_error_t::SMALL_GICP_ERROR_EXCEPTION => {
                SmallGicpError::InternalException("Unknown".to_string())
            }
        }
    }
}

/// Result type for small_gicp operations.
pub type Result<T> = std::result::Result<T, SmallGicpError>;

/// Utility function to convert C error codes to Rust Result.
pub(crate) fn check_error(error: small_gicp_sys::small_gicp_error_t) -> Result<()> {
    match error {
        small_gicp_sys::small_gicp_error_t::SMALL_GICP_SUCCESS => Ok(()),
        _ => Err(SmallGicpError::from(error)),
    }
}

/// Get error string from C library.
pub(crate) fn get_error_string(error: small_gicp_sys::small_gicp_error_t) -> String {
    unsafe {
        let c_str = small_gicp_sys::small_gicp_error_string(error);
        if c_str.is_null() {
            "Unknown error".to_string()
        } else {
            std::ffi::CStr::from_ptr(c_str)
                .to_string_lossy()
                .into_owned()
        }
    }
}
