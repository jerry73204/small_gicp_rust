//! Demonstrates the voxel map search functionality

use nalgebra::Point3;
use small_gicp_rust::{
    error::Result, point_cloud::PointCloud, traits::PointCloudTrait, voxelmap::IncrementalVoxelMap,
};

fn main() -> Result<()> {
    // Create a voxel map with voxel size of 0.1 meters
    let mut voxelmap = IncrementalVoxelMap::new(0.1);

    // Create a point cloud with a grid of points
    let mut cloud = PointCloud::new()?;

    // Add points in a 3x3x3 grid
    for x in 0..3 {
        for y in 0..3 {
            for z in 0..3 {
                cloud.add_point(x as f64 * 0.2, y as f64 * 0.2, z as f64 * 0.2);
            }
        }
    }

    // Insert the point cloud into the voxel map
    voxelmap.insert(&cloud)?;
    println!("Inserted {} points into voxel map", cloud.size());
    println!("Voxel map contains {} voxels", voxelmap.num_voxels()?);

    // Test nearest neighbor search
    println!("\n=== Nearest Neighbor Search ===");
    let query_point = Point3::new(0.15, 0.15, 0.15);
    match voxelmap.nearest_neighbor_search(&query_point) {
        Ok((index, sq_distance)) => {
            println!(
                "Query point: ({:.2}, {:.2}, {:.2})",
                query_point.x, query_point.y, query_point.z
            );
            println!("Nearest neighbor index: {}", index);
            println!("Squared distance: {:.4}", sq_distance);
            println!("Distance: {:.4}", sq_distance.sqrt());
        }
        Err(e) => println!("Nearest neighbor search failed: {:?}", e),
    }

    // Test k-nearest neighbors search
    println!("\n=== K-Nearest Neighbors Search ===");
    let k = 5;
    match voxelmap.knn_search(&query_point, k) {
        Ok((indices, sq_distances)) => {
            println!("Found {} nearest neighbors:", indices.len());
            for (i, (idx, sq_dist)) in indices.iter().zip(sq_distances.iter()).enumerate() {
                println!("  {}: index={}, distance={:.4}", i + 1, idx, sq_dist.sqrt());
            }
        }
        Err(e) => println!("KNN search failed: {:?}", e),
    }

    // Test voxel coordinate calculation
    println!("\n=== Voxel Coordinate Calculation ===");
    let test_points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.05, 0.05, 0.05),
        Point3::new(0.15, 0.15, 0.15),
        Point3::new(0.25, 0.25, 0.25),
    ];

    for point in &test_points {
        match voxelmap.get_voxel_coords(point) {
            Ok(coords) => {
                println!(
                    "Point ({:.2}, {:.2}, {:.2}) -> Voxel coordinates: [{}, {}, {}]",
                    point.x, point.y, point.z, coords[0], coords[1], coords[2]
                );
            }
            Err(e) => println!("Failed to get voxel coords: {:?}", e),
        }
    }

    // Verify voxel existence at calculated coordinates
    println!("\n=== Voxel Existence Check ===");
    if let Ok(coords) = voxelmap.get_voxel_coords(&Point3::new(0.0, 0.0, 0.0)) {
        let has_voxel = voxelmap.has_voxel_at_coords(coords[0], coords[1], coords[2]);
        println!(
            "Voxel at coordinates [{}, {}, {}] exists: {}",
            coords[0], coords[1], coords[2], has_voxel
        );
    }

    // Test get_voxel_index and get_gaussian_voxel
    println!("\n=== Voxel Index and Data Retrieval ===");
    let test_coords = [[0, 0, 0], [2, 2, 2], [10, 10, 10]];

    for coords in &test_coords {
        match voxelmap.get_voxel_index(*coords) {
            Ok(index) => {
                println!(
                    "Voxel at [{}, {}, {}] has index: {}",
                    coords[0], coords[1], coords[2], index
                );

                // Get the Gaussian voxel data by index
                match voxelmap.get_gaussian_voxel(index) {
                    Ok(voxel) => {
                        println!("  Gaussian voxel data:");
                        println!("    Number of points: {}", voxel.num_points);
                        println!(
                            "    Mean: ({:.3}, {:.3}, {:.3})",
                            voxel.mean[0], voxel.mean[1], voxel.mean[2]
                        );
                    }
                    Err(e) => println!("  Failed to get Gaussian voxel: {:?}", e),
                }
            }
            Err(e) => println!(
                "Voxel at [{}, {}, {}] not found: {:?}",
                coords[0], coords[1], coords[2], e
            ),
        }
    }

    Ok(())
}
