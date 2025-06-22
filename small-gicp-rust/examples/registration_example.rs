//! Example demonstrating the new registration API.

use nalgebra::{Isometry3, Point3};
use small_gicp::{
    create_gaussian_voxelmap,
    kdtree::KdTree,
    point_cloud::PointCloud,
    preprocess_points,
    registration::{
        align_gicp, align_icp, align_plane_icp, align_vgicp, GicpSettings, IcpSettings,
        PlaneIcpSettings, VgicpSettings,
    },
    Result,
};

fn main() -> Result<()> {
    println!("Small GICP Registration Example");
    println!("==============================\n");

    // Create sample point clouds
    let mut source = PointCloud::new()?;
    let mut target = PointCloud::new()?;

    // Generate a simple 3D pattern for target
    println!("Creating sample point clouds...");
    for i in 0..50 {
        for j in 0..50 {
            let x = i as f64 * 0.1;
            let y = j as f64 * 0.1;
            let z = (x * x + y * y).sin() * 0.5;
            target.add_point(x, y, z);
        }
    }

    // Apply a known transformation to create the source cloud
    let true_transform =
        Isometry3::rotation(nalgebra::Vector3::z() * 0.1) * Isometry3::translation(0.5, 0.3, 0.1);

    for i in 0..target.len() {
        let pt = target.point_at(i)?;
        let transformed = true_transform * pt;
        source.add_point(transformed.x, transformed.y, transformed.z);
    }

    println!("  Target points: {}", target.len());
    println!("  Source points: {}", source.len());
    println!("  True transform: rotation=0.1 rad, translation=[0.5, 0.3, 0.1]");

    // Example 1: Basic ICP registration
    println!("\n1. Basic ICP Registration");
    println!("-------------------------");
    {
        // Build KdTree for target
        let target_tree = KdTree::new(&target)?;

        // Configure ICP
        let settings = IcpSettings {
            max_iterations: 50,
            num_threads: 4,
            ..Default::default()
        };

        // Run registration
        let result = align_icp(&source, &target, &target_tree, None, settings)?;

        println!("  Converged: {}", result.converged);
        println!("  Iterations: {}", result.iterations);
        println!("  Final error: {:.6}", result.error);

        // Extract translation from result
        let translation = result.t_target_source.translation.vector;
        println!(
            "  Estimated translation: [{:.3}, {:.3}, {:.3}]",
            translation.x, translation.y, translation.z
        );
    }

    // Example 2: Plane ICP with normal estimation
    println!("\n2. Point-to-Plane ICP Registration");
    println!("----------------------------------");
    {
        // Preprocess target to estimate normals
        let (mut processed_target, target_tree) = preprocess_points(&target, 0.2, 20, 4)?;
        let (processed_source, _) = preprocess_points(&source, 0.2, 20, 4)?;

        println!("  Target has normals: {}", processed_target.has_normals());

        // Configure Plane ICP
        let settings = PlaneIcpSettings {
            max_iterations: 30,
            ..Default::default()
        };

        // Run registration
        let result = align_plane_icp(
            &processed_source,
            &processed_target,
            &target_tree,
            None,
            settings,
        )?;

        println!("  Converged: {}", result.converged);
        println!("  Iterations: {}", result.iterations);
        println!("  Final error: {:.6}", result.error);
    }

    // Example 3: GICP Registration
    println!("\n3. GICP Registration");
    println!("--------------------");
    {
        // Preprocess both clouds (compute covariances)
        let (processed_target, target_tree) = preprocess_points(&target, 0.2, 20, 4)?;
        let (processed_source, source_tree) = preprocess_points(&source, 0.2, 20, 4)?;

        println!("  Downsampled target: {} points", processed_target.len());
        println!("  Downsampled source: {} points", processed_source.len());
        println!(
            "  Covariances computed: {}",
            processed_target.has_covariances()
        );

        // Configure GICP
        let settings = GicpSettings::default();

        // Run GICP (requires both source and target trees)
        let result = align_gicp(
            &processed_source,
            &processed_target,
            &source_tree,
            &target_tree,
            None,
            settings,
        )?;

        println!("  Converged: {}", result.converged);
        println!("  Iterations: {}", result.iterations);
        println!("  Final error: {:.6}", result.error);
    }

    // Example 4: VGICP Registration with voxel maps
    println!("\n4. VGICP Registration");
    println!("---------------------");
    {
        // Create a voxel map from target
        let voxelmap = create_gaussian_voxelmap(&target, 0.5)?;
        println!("  Created voxel map with resolution: 0.5");

        // Configure VGICP
        let settings = VgicpSettings {
            voxel_resolution: 0.5,
            ..Default::default()
        };

        // Run VGICP registration
        let result = align_vgicp(&source, &voxelmap, None, settings)?;

        println!("  Converged: {}", result.converged);
        println!("  Iterations: {}", result.iterations);
        println!("  Final error: {:.6}", result.error);
    }

    // Example 5: Custom settings
    println!("\n5. Registration with Custom Settings");
    println!("------------------------------------");
    {
        let target_tree = KdTree::new(&target)?;

        // Create custom ICP settings
        let settings = IcpSettings {
            max_correspondence_distance: 2.0,
            rotation_eps: 0.001,
            translation_eps: 0.001,
            num_threads: 8,
            max_iterations: 100,
        };

        let result = align_icp(&source, &target, &target_tree, None, settings)?;

        println!("  Used custom convergence criteria");
        println!("  Converged: {}", result.converged);
        println!("  Iterations: {}", result.iterations);
        println!("  Final error: {:.6}", result.error);
    }

    println!("\n✓ Registration examples completed successfully!");
    Ok(())
}
