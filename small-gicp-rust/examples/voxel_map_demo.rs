//! Demonstrates the voxel map functionality

use nalgebra::{Point3, Vector3};
use small_gicp_rust::{
    error::Result,
    point_cloud::PointCloud,
    voxelmap::{IncrementalVoxelMap, SearchOffsetPattern},
};

fn main() -> Result<()> {
    // Create a voxel map with voxel size of 0.1 meters
    let mut voxelmap = IncrementalVoxelMap::new(0.1);

    // Create a point cloud with some sample points
    let mut cloud = PointCloud::new()?;
    cloud.add_point(0.0, 0.0, 0.0);
    cloud.add_point(0.05, 0.05, 0.05); // Same voxel
    cloud.add_point(0.2, 0.2, 0.2); // Different voxel
    cloud.add_point(0.25, 0.25, 0.25); // Same voxel as previous

    // Insert the point cloud into the voxel map
    voxelmap.insert(&cloud)?;
    println!("Voxel map size after insertion: {}", voxelmap.size());
    println!("Number of voxels: {}", voxelmap.num_voxels()?);
    println!("Voxel size: {}", voxelmap.voxel_size());

    // Test single point insertion
    let point = Point3::new(0.5, 0.5, 0.5);
    voxelmap.insert_point(&point)?;
    println!(
        "Voxel map size after single point insertion: {}",
        voxelmap.size()
    );

    // Test point with normal insertion
    let point_with_normal = Point3::new(1.0, 1.0, 1.0);
    let normal = Vector3::new(0.0, 0.0, 1.0);
    voxelmap.insert_point_with_normal(&point_with_normal, &normal)?;

    // Skip covariance insertion for now (not implemented)
    // let point_with_cov = Point3::new(1.5, 1.5, 1.5);
    // let covariance = Matrix3::identity() * 0.01;
    // voxelmap.insert_point_with_covariance(&point_with_cov, &covariance)?;

    // Test voxel coordinate checking
    println!("\nVoxel existence checks:");
    println!(
        "Has voxel at (0, 0, 0): {}",
        voxelmap.has_voxel_at_coords(0, 0, 0)
    );
    println!(
        "Has voxel at (2, 2, 2): {}",
        voxelmap.has_voxel_at_coords(2, 2, 2)
    );
    println!(
        "Has voxel at (10, 10, 10): {}",
        voxelmap.has_voxel_at_coords(10, 10, 10)
    );

    // Test search offset configuration
    voxelmap.set_search_offsets(SearchOffsetPattern::FaceEdgeCorner)?;
    println!("\nSet search offsets to FaceEdgeCorner (27 neighbors)");

    // Find nearby voxels
    let query_point = [0.1, 0.1, 0.1];
    let max_distance = 0.5;
    let nearby_voxels = voxelmap.find_nearby_voxels(&query_point, max_distance);
    println!(
        "\nNearby voxels to ({}, {}, {}) within {} meters:",
        query_point[0], query_point[1], query_point[2], max_distance
    );
    for voxel in &nearby_voxels {
        println!(
            "  Voxel at ({}, {}, {}), distance: {:.3}",
            voxel.coordinates[0], voxel.coordinates[1], voxel.coordinates[2], voxel.distance
        );
    }

    // Get voxel data at specific coordinates
    if let Some(voxel_data) = voxelmap.get_voxel(&[0, 0, 0]) {
        println!("\nVoxel data at (0, 0, 0):");
        println!("  Number of points: {}", voxel_data.num_points);
        println!(
            "  Mean: ({:.3}, {:.3}, {:.3})",
            voxel_data.mean[0], voxel_data.mean[1], voxel_data.mean[2]
        );
    }

    // Finalize the voxel map
    voxelmap.finalize();
    println!("\nVoxel map finalized");

    Ok(())
}
