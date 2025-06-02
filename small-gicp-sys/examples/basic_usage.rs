//! Basic usage example of small-gicp-sys
//!
//! This example demonstrates how to create point clouds and perform
//! basic operations using the raw FFI bindings.

use small_gicp_sys::*;
use std::ptr;

fn main() {
    unsafe {
        println!("Small GICP Sys - Basic Usage Example");
        println!("====================================\n");

        // Get version
        let mut version_buffer = [0u8; 64];
        let result =
            small_gicp_get_version(version_buffer.as_mut_ptr() as *mut i8, version_buffer.len());
        if result == small_gicp_error_t::SMALL_GICP_SUCCESS {
            let version =
                std::ffi::CStr::from_ptr(version_buffer.as_ptr() as *const i8).to_string_lossy();
            println!("Library version: {}", version);
        }

        // Create point clouds
        let mut target: *mut small_gicp_point_cloud_t = ptr::null_mut();
        let mut source: *mut small_gicp_point_cloud_t = ptr::null_mut();

        println!("\nCreating point clouds...");
        assert_eq!(
            small_gicp_point_cloud_create(&mut target),
            small_gicp_error_t::SMALL_GICP_SUCCESS
        );
        assert_eq!(
            small_gicp_point_cloud_create(&mut source),
            small_gicp_error_t::SMALL_GICP_SUCCESS
        );

        // Add some synthetic data
        let n_points = 100;
        small_gicp_point_cloud_resize(target, n_points);
        small_gicp_point_cloud_resize(source, n_points);

        println!("Adding {} points to each cloud...", n_points);
        for i in 0..n_points {
            let angle = 2.0 * std::f64::consts::PI * (i as f64) / (n_points as f64);
            let radius = 10.0;

            // Target: circle in XY plane
            let tx = radius * angle.cos();
            let ty = radius * angle.sin();
            let tz = 0.0;
            small_gicp_point_cloud_set_point(target, i, tx, ty, tz);

            // Source: slightly rotated and translated circle
            let sx = radius * angle.cos() + 0.5;
            let sy = radius * angle.sin() - 0.3;
            let sz = 0.1;
            small_gicp_point_cloud_set_point(source, i, sx, sy, sz);
        }

        // Create KdTree for target
        println!("\nBuilding KdTree...");
        let mut kdtree: *mut small_gicp_kdtree_t = ptr::null_mut();
        assert_eq!(
            small_gicp_kdtree_create(target, 1, &mut kdtree),
            small_gicp_error_t::SMALL_GICP_SUCCESS
        );

        // Test nearest neighbor search
        println!("\nTesting nearest neighbor search...");
        let query = (5.0, 5.0, 0.0);
        let mut nearest_idx: usize = 0;
        let mut sq_dist: f64 = 0.0;

        let result = small_gicp_kdtree_nearest_neighbor_search(
            kdtree,
            query.0,
            query.1,
            query.2,
            &mut nearest_idx,
            &mut sq_dist,
        );

        if result == small_gicp_error_t::SMALL_GICP_SUCCESS {
            let mut nx = 0.0;
            let mut ny = 0.0;
            let mut nz = 0.0;
            small_gicp_point_cloud_get_point(target, nearest_idx, &mut nx, &mut ny, &mut nz);

            println!(
                "Query: ({:.2}, {:.2}, {:.2}) -> Nearest: idx={}, ({:.2}, {:.2}, {:.2}), dist={:.3}",
                query.0, query.1, query.2,
                nearest_idx, nx, ny, nz,
                sq_dist.sqrt()
            );
        }

        // Perform simple ICP registration
        println!("\nPerforming ICP registration...");
        let mut result = small_gicp_registration_result_t {
            T_target_source: [0.0; 16],
            converged: false,
            iterations: 0,
            num_inliers: 0,
            error: 0.0,
        };

        let reg_result = small_gicp_align(
            target,
            source,
            small_gicp_registration_type_t::SMALL_GICP_ICP,
            ptr::null(), // identity initial guess
            1,           // single thread
            &mut result,
        );

        if reg_result == small_gicp_error_t::SMALL_GICP_SUCCESS {
            println!("Registration completed!");
            println!("  Converged: {}", result.converged);
            println!("  Iterations: {}", result.iterations);
            println!("  Error: {:.6}", result.error);
            println!("  Transformation matrix:");
            for i in 0..4 {
                print!("    [");
                for j in 0..4 {
                    print!("{:8.4}", result.T_target_source[i * 4 + j]);
                    if j < 3 {
                        print!(", ");
                    }
                }
                println!("]");
            }
        } else {
            println!("Registration failed!");
        }

        // Clean up
        println!("\nCleaning up...");
        small_gicp_kdtree_destroy(kdtree);
        small_gicp_point_cloud_destroy(source);
        small_gicp_point_cloud_destroy(target);

        println!("\nDone!");
    }
}
