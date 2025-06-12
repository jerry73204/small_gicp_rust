//! Point cloud preprocessing demonstration.
//!
//! This example shows various preprocessing operations including
//! downsampling, normal estimation, and KdTree operations.

use nalgebra::Point3;
use small_gicp_rust::{estimate_covariances, estimate_normals, prelude::*};

fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    println!("Small GICP Rust - Preprocessing Demo");
    println!("===================================");

    // Create a dense point cloud
    let original_cloud = create_dense_point_cloud();
    println!("\nOriginal cloud: {} points", original_cloud.len());

    // Example 1: Voxel grid downsampling
    println!("\n1. Voxel Grid Downsampling");
    println!("--------------------------");

    let leaf_sizes = [0.1, 0.05, 0.02];
    for leaf_size in leaf_sizes {
        let voxel_config = VoxelGridConfig {
            leaf_size,
            backend: DownsamplingBackend::Default,
            num_threads: 4,
        };
        let downsampled = original_cloud.voxelgrid_sampling(&voxel_config)?;
        println!(
            "Leaf size {:.2}: {} → {} points ({:.1}% reduction)",
            leaf_size,
            original_cloud.len(),
            downsampled.len(),
            (1.0 - downsampled.len() as f64 / original_cloud.len() as f64) * 100.0
        );
    }

    // Example 2: Random downsampling
    println!("\n2. Random Downsampling");
    println!("----------------------");

    let sample_counts = [1000, 500, 100];
    for num_samples in sample_counts {
        if num_samples < original_cloud.len() {
            let downsampled = original_cloud.random_sampling(num_samples)?;
            println!(
                "Sample count {}: {} → {} points",
                num_samples,
                original_cloud.len(),
                downsampled.len()
            );
        }
    }

    // Example 3: Complete preprocessing pipeline
    println!("\n3. Complete Preprocessing Pipeline");
    println!("----------------------------------");

    let result = original_cloud.preprocess_points(&PreprocessorConfig {
        downsampling_resolution: 0.05,
        num_neighbors: 20,
        num_threads: 4,
    })?;

    println!("Preprocessing complete:");
    println!("  Original: {} points", original_cloud.len());
    println!("  Processed: {} points", result.cloud.len());
    println!("  KdTree created: yes");

    // Verify normals were estimated
    if !result.cloud.is_empty() {
        let normal = result.cloud.get_normal(0)?;
        println!(
            "  First normal: [{:.3}, {:.3}, {:.3}] (magnitude: {:.3})",
            normal.x,
            normal.y,
            normal.z,
            normal.magnitude()
        );
    }

    // Example 4: KdTree nearest neighbor search
    println!("\n4. KdTree Nearest Neighbor Search");
    println!("---------------------------------");

    let kdtree = &result.kdtree;
    let query_point = Point3::new(0.5, 0.5, 0.5);

    // Single nearest neighbor
    let (nearest_idx, sq_dist) = kdtree.nearest_neighbor(query_point)?;
    let nearest_point = result.cloud.get_point(nearest_idx)?;

    println!(
        "Query point: [{:.2}, {:.2}, {:.2}]",
        query_point.x, query_point.y, query_point.z
    );
    println!(
        "Nearest point: [{:.2}, {:.2}, {:.2}] at index {} (distance: {:.4})",
        nearest_point.x,
        nearest_point.y,
        nearest_point.z,
        nearest_idx,
        sq_dist.sqrt()
    );

    // K nearest neighbors
    let k = 5;
    let knn_results = kdtree.knn_search(query_point, k)?;
    println!("\n{} nearest neighbors:", k);
    for (i, (idx, sq_dist)) in knn_results.iter().enumerate() {
        let point = result.cloud.get_point(*idx)?;
        println!(
            "  {}: index={}, distance={:.4}, point=[{:.2}, {:.2}, {:.2}]",
            i + 1,
            idx,
            sq_dist.sqrt(),
            point.x,
            point.y,
            point.z
        );
    }

    // Radius search
    let radius = 0.2;
    let radius_results = kdtree.radius_search(query_point, radius, 10)?;
    println!("\nPoints within radius {:.1}:", radius);
    println!("  Found {} points", radius_results.len());
    for (i, (idx, sq_dist)) in radius_results.iter().take(5).enumerate() {
        let point = result.cloud.get_point(*idx)?;
        println!(
            "  {}: index={}, distance={:.4}, point=[{:.2}, {:.2}, {:.2}]",
            i + 1,
            idx,
            sq_dist.sqrt(),
            point.x,
            point.y,
            point.z
        );
    }
    if radius_results.len() > 5 {
        println!("  ... and {} more", radius_results.len() - 5);
    }

    // Example 5: Manual normal estimation
    println!("\n5. Manual Normal Estimation");
    println!("---------------------------");

    let voxel_config = VoxelGridConfig {
        leaf_size: 0.1,
        backend: DownsamplingBackend::Default,
        num_threads: 1,
    };
    let mut test_cloud = original_cloud.voxelgrid_sampling(&voxel_config)?;
    let kdtree_config = KdTreeConfig::default();
    let test_kdtree = KdTree::new(&test_cloud, &kdtree_config)?;

    println!("Before normal estimation:");
    if !test_cloud.is_empty() {
        match test_cloud.get_normal(0) {
            Ok(normal) => println!("  First normal magnitude: {:.6}", normal.magnitude()),
            Err(_) => println!("  No normals available"),
        }
    }

    estimate_normals(
        &mut test_cloud,
        &test_kdtree,
        &NormalEstimationConfig {
            num_neighbors: 15,
            backend: NormalEstimationBackend::Default,
            num_threads: 1,
        },
    )?;

    println!("After normal estimation:");
    if !test_cloud.is_empty() {
        let normal = test_cloud.get_normal(0)?;
        println!(
            "  First normal: [{:.3}, {:.3}, {:.3}] (magnitude: {:.3})",
            normal.x,
            normal.y,
            normal.z,
            normal.magnitude()
        );
    }

    // Example 6: Covariance estimation for GICP
    println!("\n6. Covariance Estimation for GICP");
    println!("---------------------------------");

    estimate_covariances(
        &mut test_cloud,
        &test_kdtree,
        &CovarianceEstimationConfig {
            num_neighbors: 15,
            backend: NormalEstimationBackend::Default,
            num_threads: 1,
        },
    )?;
    println!("Covariances estimated successfully (required for GICP)");

    println!("\nPreprocessing demo completed!");
    Ok(())
}

fn create_dense_point_cloud() -> PointCloud {
    let mut points = Vec::new();

    // Create a dense grid of points
    let resolution = 0.01;
    let size = 1.0;

    let steps = (size / resolution) as i32;
    for i in 0..steps {
        for j in 0..steps {
            for k in 0..steps {
                let x = i as f64 * resolution;
                let y = j as f64 * resolution;
                let z = k as f64 * resolution;

                // Add some geometric structure (sphere + cube)
                let distance_from_center =
                    ((x - 0.5).powi(2) + (y - 0.5).powi(2) + (z - 0.5).powi(2)).sqrt();

                // Include points that are either in a sphere or form the edges of a cube
                if distance_from_center <= 0.3
                    || (x < 0.1 || x > 0.9 || y < 0.1 || y > 0.9 || z < 0.1 || z > 0.9)
                {
                    points.push(Point3::new(x, y, z));
                }
            }
        }
    }

    PointCloud::from_points(&points).expect("Failed to create dense point cloud")
}
