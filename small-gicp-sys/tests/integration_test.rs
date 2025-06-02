use small_gicp_sys::*;
use std::ptr;

#[test]
fn test_point_cloud_creation() {
    unsafe {
        // Create a point cloud
        let mut cloud: *mut small_gicp_point_cloud_t = ptr::null_mut();
        let result = small_gicp_point_cloud_create(&mut cloud);
        assert_eq!(result, small_gicp_error_t::SMALL_GICP_SUCCESS);
        assert!(!cloud.is_null());

        // Resize the point cloud
        let result = small_gicp_point_cloud_resize(cloud, 10);
        assert_eq!(result, small_gicp_error_t::SMALL_GICP_SUCCESS);

        // Get size
        let mut size: usize = 0;
        let result = small_gicp_point_cloud_size(cloud, &mut size);
        assert_eq!(result, small_gicp_error_t::SMALL_GICP_SUCCESS);
        assert_eq!(size, 10);

        // Set a point
        let result = small_gicp_point_cloud_set_point(cloud, 0, 1.0, 2.0, 3.0);
        assert_eq!(result, small_gicp_error_t::SMALL_GICP_SUCCESS);

        // Get the point back
        let mut x: f64 = 0.0;
        let mut y: f64 = 0.0;
        let mut z: f64 = 0.0;
        let result = small_gicp_point_cloud_get_point(cloud, 0, &mut x, &mut y, &mut z);
        assert_eq!(result, small_gicp_error_t::SMALL_GICP_SUCCESS);
        assert_eq!(x, 1.0);
        assert_eq!(y, 2.0);
        assert_eq!(z, 3.0);

        // Destroy the point cloud
        let result = small_gicp_point_cloud_destroy(cloud);
        assert_eq!(result, small_gicp_error_t::SMALL_GICP_SUCCESS);
    }
}

#[test]
fn test_error_string() {
    unsafe {
        let error_str = small_gicp_error_string(small_gicp_error_t::SMALL_GICP_SUCCESS);
        assert!(!error_str.is_null());

        let error_str =
            small_gicp_error_string(small_gicp_error_t::SMALL_GICP_ERROR_INVALID_ARGUMENT);
        assert!(!error_str.is_null());
    }
}

#[test]
fn test_kdtree_creation() {
    unsafe {
        // Create a point cloud
        let mut cloud: *mut small_gicp_point_cloud_t = ptr::null_mut();
        small_gicp_point_cloud_create(&mut cloud);
        small_gicp_point_cloud_resize(cloud, 100);

        // Add some random points
        for i in 0..100 {
            let x = (i as f64) * 0.1;
            let y = (i as f64) * 0.2;
            let z = (i as f64) * 0.3;
            small_gicp_point_cloud_set_point(cloud, i, x, y, z);
        }

        // Create KdTree
        let mut kdtree: *mut small_gicp_kdtree_t = ptr::null_mut();
        let result = small_gicp_kdtree_create(cloud, 1, &mut kdtree);
        assert_eq!(result, small_gicp_error_t::SMALL_GICP_SUCCESS);
        assert!(!kdtree.is_null());

        // Test nearest neighbor search
        let mut index: usize = 0;
        let mut sq_dist: f64 = 0.0;
        let result = small_gicp_kdtree_nearest_neighbor_search(
            kdtree,
            5.0,
            10.0,
            15.0,
            &mut index,
            &mut sq_dist,
        );
        assert_eq!(result, small_gicp_error_t::SMALL_GICP_SUCCESS);

        // Clean up
        small_gicp_kdtree_destroy(kdtree);
        small_gicp_point_cloud_destroy(cloud);
    }
}

#[test]
fn test_version() {
    unsafe {
        let mut buffer = [0u8; 64];
        let result = small_gicp_get_version(buffer.as_mut_ptr() as *mut i8, buffer.len());
        assert_eq!(result, small_gicp_error_t::SMALL_GICP_SUCCESS);
    }
}
