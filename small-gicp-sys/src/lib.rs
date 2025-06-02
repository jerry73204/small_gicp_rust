//! Raw FFI bindings to the small_gicp C API
//!
//! This crate provides low-level bindings to the small_gicp point cloud registration library.
//! It is generated using bindgen and exposes the C API directly.
//!
//! # Safety
//!
//! This crate provides raw FFI bindings and is inherently unsafe. Users should consider
//! using a higher-level safe wrapper crate instead of using these bindings directly.

#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

// Include the generated bindings
include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_codes() {
        // Test that error codes are properly defined
        assert_eq!(small_gicp_error_t::SMALL_GICP_SUCCESS as i32, 0);
        assert_eq!(
            small_gicp_error_t::SMALL_GICP_ERROR_INVALID_ARGUMENT as i32,
            -1
        );
        assert_eq!(
            small_gicp_error_t::SMALL_GICP_ERROR_OUT_OF_MEMORY as i32,
            -2
        );
        assert_eq!(
            small_gicp_error_t::SMALL_GICP_ERROR_FILE_NOT_FOUND as i32,
            -3
        );
        assert_eq!(small_gicp_error_t::SMALL_GICP_ERROR_IO_ERROR as i32, -4);
        assert_eq!(small_gicp_error_t::SMALL_GICP_ERROR_EXCEPTION as i32, -5);
    }

    #[test]
    fn test_registration_types() {
        // Test that registration types are properly defined
        assert_eq!(small_gicp_registration_type_t::SMALL_GICP_ICP as i32, 0);
        assert_eq!(
            small_gicp_registration_type_t::SMALL_GICP_PLANE_ICP as i32,
            1
        );
        assert_eq!(small_gicp_registration_type_t::SMALL_GICP_GICP as i32, 2);
        assert_eq!(small_gicp_registration_type_t::SMALL_GICP_VGICP as i32, 3);
    }
}
