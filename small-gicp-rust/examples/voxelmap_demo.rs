//! Demonstration of the incremental voxel map functionality.
//!
//! This example shows how to:
//! - Create an incremental voxel map
//! - Insert points and point clouds
//! - Perform nearest neighbor searches
//! - Use different container types
//! - Work with Gaussian voxel statistics

use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use small_gicp_rust::prelude::*;

fn main() -> Result<()> {
    println!("=== Incremental Voxel Map Demo ===\n");

    // Demo 1: Basic voxel map with points
    demo_basic_voxelmap()?;

    // Demo 2: Point cloud insertion with transformations
    demo_point_cloud_insertion()?;

    // Demo 3: Nearest neighbor search
    demo_nearest_neighbor_search()?;

    // Demo 4: Different container types
    demo_container_types()?;

    // Demo 5: Gaussian voxel statistics
    demo_gaussian_voxels()?;

    println!("\n=== Demo completed successfully! ===");
    Ok(())
}

/// Demonstrate basic voxel map creation and point insertion
fn demo_basic_voxelmap() -> Result<()> {
    println!("--- Demo 1: Basic Voxel Map ---");

    // Create a voxel map with 0.1 unit voxel size
    let mut voxelmap = IncrementalVoxelMap::with_leaf_size(0.1, VoxelContainerType::FlatPoints)?;

    // Insert individual points
    let points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.15, 0.0, 0.0),   // Different voxel
        Point3::new(0.0, 0.15, 0.0),   // Different voxel
        Point3::new(0.05, 0.05, 0.05), // Same voxel as first point
    ];

    for (i, point) in points.iter().enumerate() {
        voxelmap.insert_point(point)?;
        println!(
            "Inserted point {}: ({:.2}, {:.2}, {:.2})",
            i + 1,
            point.x,
            point.y,
            point.z
        );
    }

    println!("Total points in voxelmap: {}", voxelmap.size()?);
    println!("Number of voxels: {}", voxelmap.num_voxels()?);
    println!();

    Ok(())
}

/// Demonstrate point cloud insertion with transformations
fn demo_point_cloud_insertion() -> Result<()> {
    println!("--- Demo 2: Point Cloud Insertion with Transformations ---");

    let mut voxelmap = IncrementalVoxelMap::with_leaf_size(0.05, VoxelContainerType::FlatPoints)?;

    // Create a small point cloud
    let points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.1, 0.0, 0.0),
        Point3::new(0.0, 0.1, 0.0),
    ];
    let cloud = PointCloud::from_points(&points)?;

    // Insert the original cloud
    voxelmap.insert_point_cloud(&cloud)?;
    println!("Inserted original cloud with {} points", cloud.len());

    // Create a transformation (translate by (0.5, 0, 0))
    let transform =
        Isometry3::from_parts(Translation3::new(0.5, 0.0, 0.0), UnitQuaternion::identity());

    // Insert the transformed cloud
    voxelmap.insert_point_cloud_with_transform(&cloud, &transform)?;
    println!("Inserted transformed cloud (translated by +0.5 in X)");

    println!("Total points in voxelmap: {}", voxelmap.size()?);
    println!("Number of voxels: {}", voxelmap.num_voxels()?);
    println!();

    Ok(())
}

/// Demonstrate nearest neighbor search functionality
fn demo_nearest_neighbor_search() -> Result<()> {
    println!("--- Demo 3: Nearest Neighbor Search ---");

    let mut voxelmap = IncrementalVoxelMap::with_leaf_size(0.1, VoxelContainerType::FlatPoints)?;

    // Create a grid of points
    let mut points = Vec::new();
    for i in 0..5 {
        for j in 0..5 {
            points.push(Point3::new(i as f64 * 0.2, j as f64 * 0.2, 0.0));
        }
    }

    let cloud = PointCloud::from_points(&points)?;
    voxelmap.insert_point_cloud(&cloud)?;

    println!("Created 5x5 grid of points (25 total)");

    // Test different search offset patterns
    let patterns = [
        SearchOffsetPattern::Center,
        SearchOffsetPattern::FaceNeighbors,
        SearchOffsetPattern::FullNeighborhood,
    ];

    for pattern in &patterns {
        voxelmap.set_search_offsets(*pattern)?;

        // Search for nearest neighbor to a query point
        let query = Point3::new(0.35, 0.45, 0.1);
        let (index, distance) = voxelmap.nearest_neighbor_search(&query)?;

        println!("Search pattern: {:?}", pattern);
        println!("  Query: ({:.2}, {:.2}, {:.2})", query.x, query.y, query.z);
        println!(
            "  Nearest neighbor index: {}, distance: {:.4}",
            index, distance
        );

        // K-nearest neighbor search
        let (indices, distances) = voxelmap.knn_search(&query, 3)?;
        println!("  3-NN indices: {:?}", indices);
        println!(
            "  3-NN distances: {:?}",
            distances
                .iter()
                .map(|d| format!("{:.4}", d))
                .collect::<Vec<_>>()
        );
    }

    println!();
    Ok(())
}

/// Demonstrate different container types
fn demo_container_types() -> Result<()> {
    println!("--- Demo 4: Different Container Types ---");

    // Points with normals
    let mut normal_voxelmap =
        IncrementalVoxelMap::with_leaf_size(0.1, VoxelContainerType::FlatNormal)?;

    let point = Point3::new(0.0, 0.0, 0.0);
    let normal = Vector3::new(0.0, 0.0, 1.0); // Z-up normal

    normal_voxelmap.insert_point_with_normal(&point, &normal)?;
    println!("Inserted point with normal in FlatNormal container");

    // Points with covariances
    let mut cov_voxelmap =
        IncrementalVoxelMap::with_leaf_size(0.1, VoxelContainerType::FlatCovariance)?;

    let covariance = nalgebra::Matrix3::identity() * 0.01; // Small covariance
    cov_voxelmap.insert_point_with_covariance(&point, &covariance)?;
    println!("Inserted point with covariance in FlatCovariance container");

    // Combined normals and covariances
    let mut combined_voxelmap =
        IncrementalVoxelMap::with_leaf_size(0.1, VoxelContainerType::FlatNormalCovariance)?;
    combined_voxelmap.insert_point_with_normal(&point, &normal)?;
    println!("Inserted point with normal in FlatNormalCovariance container");

    println!();
    Ok(())
}

/// Demonstrate Gaussian voxel statistics
fn demo_gaussian_voxels() -> Result<()> {
    println!("--- Demo 5: Gaussian Voxel Statistics ---");

    let mut gaussian_voxelmap =
        IncrementalVoxelMap::with_leaf_size(0.2, VoxelContainerType::Gaussian)?;

    // Insert multiple points in the same voxel to create statistics
    let base_points = vec![
        Point3::new(0.05, 0.05, 0.05),
        Point3::new(0.08, 0.06, 0.04),
        Point3::new(0.06, 0.08, 0.06),
        Point3::new(0.04, 0.04, 0.05),
        Point3::new(0.07, 0.07, 0.05),
    ];

    for point in &base_points {
        gaussian_voxelmap.insert_point(point)?;
    }

    println!(
        "Inserted {} points for Gaussian statistics",
        base_points.len()
    );

    // Note: Some operations like finalize() and get_gaussian_voxel() may not be implemented
    // in all C wrapper versions, so we handle this gracefully
    match gaussian_voxelmap.finalize() {
        Ok(_) => {
            println!("Finalized Gaussian voxel statistics");

            // Try to get voxel coordinates and Gaussian statistics
            if let Ok(coords) = gaussian_voxelmap.get_voxel_coords(&base_points[0]) {
                if let Ok(voxel_index) = gaussian_voxelmap.get_voxel_index(coords) {
                    match gaussian_voxelmap.get_gaussian_voxel(voxel_index) {
                        Ok(gaussian) => {
                            println!("Gaussian voxel statistics:");
                            println!("  Number of points: {}", gaussian.num_points);
                            println!(
                                "  Mean: ({:.4}, {:.4}, {:.4})",
                                gaussian.mean.x, gaussian.mean.y, gaussian.mean.z
                            );
                            println!("  Covariance trace: {:.6}", gaussian.covariance.trace());
                        }
                        Err(SmallGicpError::NotImplemented(_)) => {
                            println!("Gaussian voxel access not implemented in this build");
                        }
                        Err(e) => println!("Error accessing Gaussian voxel: {:?}", e),
                    }
                }
            }
        }
        Err(SmallGicpError::NotImplemented(_)) => {
            println!("Gaussian voxel finalization not implemented in this build");
            println!("Basic insertion completed successfully");
        }
        Err(e) => println!("Error finalizing Gaussian voxels: {:?}", e),
    }

    println!();
    Ok(())
}
