//! Example demonstrating the simplified registration API.

use nalgebra::{Isometry3, Point3};
use small_gicp::{
    align, create_gaussian_voxelmap,
    kdtree::KdTree,
    point_cloud::PointCloud,
    preprocess_points,
    registration::{RegistrationSetting, RegistrationType},
    traits::{MutablePointCloudTrait, PointCloudTrait},
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

    // Apply a known transformation to create source cloud
    let true_transform =
        Isometry3::translation(0.5, 0.3, 0.1) * Isometry3::rotation(nalgebra::Vector3::z() * 0.1);

    for i in 0..target.size() {
        let (x, y, z) = target.point_at(i)?;
        let pt = Point3::new(x, y, z);
        let transformed = true_transform * pt;
        source.add_point(transformed.x, transformed.y, transformed.z);
    }

    println!("  Target points: {}", target.size());
    println!("  Source points: {}", source.size());
    println!("  True transform: rotation=0.1 rad, translation=[0.5, 0.3, 0.1]");

    // Example 1: Basic ICP registration
    println!("\n1. Basic ICP Registration");
    println!("-------------------------");
    {
        // Build KdTree for target
        let target_tree = KdTree::new(&target)?;

        // Configure ICP
        let mut settings = RegistrationSetting::default();
        settings.reg_type = RegistrationType::ICP;
        settings.max_iterations = 50;
        settings.num_threads = 4;

        // Run registration
        let result = align(&target, &source, &target_tree, None, Some(settings))?;

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

    // Example 2: Preprocessing pipeline
    println!("\n2. Registration with Preprocessing");
    println!("----------------------------------");
    {
        // Preprocess both clouds (downsample, compute normals and covariances)
        let (processed_target, target_tree) = preprocess_points(&target, 0.2, 20, 4)?;
        let (processed_source, _) = preprocess_points(&source, 0.2, 20, 4)?;

        println!("  Downsampled target: {} points", processed_target.size());
        println!("  Downsampled source: {} points", processed_source.size());
        println!("  Normals computed: {}", processed_target.has_normals());
        println!(
            "  Covariances computed: {}",
            processed_target.has_covariances()
        );

        // Run GICP (requires covariances)
        let mut settings = RegistrationSetting::default();
        settings.reg_type = RegistrationType::GICP;

        let result = align(
            &processed_target,
            &processed_source,
            &target_tree,
            None,
            Some(settings),
        )?;

        println!("\n  GICP Results:");
        println!("  Converged: {}", result.converged);
        println!("  Iterations: {}", result.iterations);
        println!("  Final error: {:.6}", result.error);
    }

    // Example 3: Voxel map creation (for VGICP)
    println!("\n3. Voxel Map Creation");
    println!("---------------------");
    {
        let voxel_resolution = 0.5;
        let voxelmap = create_gaussian_voxelmap(&target, voxel_resolution)?;

        println!("  Voxel resolution: {}", voxel_resolution);
        println!("  Number of voxels: {}", voxelmap.size());

        // Note: VGICP alignment is not yet fully implemented
        println!("  (VGICP alignment will be available in future release)");
    }

    // Example 4: Multi-stage registration
    println!("\n4. Multi-stage Registration");
    println!("---------------------------");
    {
        // Stage 1: Coarse alignment with downsampled clouds
        let (coarse_target, coarse_tree) = preprocess_points(&target, 0.5, 10, 4)?;
        let (coarse_source, _) = preprocess_points(&source, 0.5, 10, 4)?;

        let mut coarse_settings = RegistrationSetting::default();
        coarse_settings.reg_type = RegistrationType::ICP;
        coarse_settings.max_correspondence_distance = 2.0;

        let coarse_result = align(
            &coarse_target,
            &coarse_source,
            &coarse_tree,
            None,
            Some(coarse_settings),
        )?;

        println!("  Stage 1 (coarse): error = {:.6}", coarse_result.error);

        // Stage 2: Fine alignment with full resolution
        let target_tree = KdTree::new(&target)?;
        let mut fine_settings = RegistrationSetting::default();
        fine_settings.reg_type = RegistrationType::ICP;
        fine_settings.max_correspondence_distance = 0.5;

        let fine_result = align(
            &target,
            &source,
            &target_tree,
            Some(coarse_result.t_target_source),
            Some(fine_settings),
        )?;

        println!("  Stage 2 (fine): error = {:.6}", fine_result.error);

        let translation = fine_result.t_target_source.translation.vector;
        println!(
            "  Final translation: [{:.3}, {:.3}, {:.3}]",
            translation.x, translation.y, translation.z
        );
    }

    println!("\nRegistration examples completed successfully!");

    Ok(())
}
