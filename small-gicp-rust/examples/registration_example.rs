//! Example demonstrating the new registration API.

use rand::distributions::{Distribution, Uniform};
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

    // Create a random transformation to apply to source cloud
    // This simulates a realistic scenario where source needs to be aligned to target
    let mut rng = rand::thread_rng();

    // Random rotation between -45 and 45 degrees (more reasonable for ICP convergence)
    let rotation_dist = Uniform::new(-45.0_f64.to_radians(), 45.0_f64.to_radians());
    let rotation_angle = rotation_dist.sample(&mut rng);

    // Random translation between -1.5 and 1.5 for each axis
    let trans_dist = Uniform::new(-1.5, 1.5);
    let translation = nalgebra::Vector3::new(
        trans_dist.sample(&mut rng),
        trans_dist.sample(&mut rng),
        trans_dist.sample(&mut rng) * 0.5, // Smaller Z translation
    );

    // Create transformation: first rotate, then translate
    let true_transform = nalgebra::Isometry3::new(
        translation,
        nalgebra::Vector3::z() * rotation_angle, // Rotation around Z axis
    );

    // Add some noise to make it more realistic
    let noise_dist = Uniform::new(-0.01, 0.01); // Small noise

    // Transform the source cloud (this is what we want registration to recover)
    for i in 0..50 {
        for j in 0..50 {
            let x = i as f64 * 0.1;
            let y = j as f64 * 0.1;
            let z = (x * x + y * y).sin() * 0.5;

            // Apply transformation to source points
            let pt = nalgebra::Point3::new(x, y, z);
            let transformed = true_transform * pt;

            // Add small noise
            source.add_point(
                transformed.x + noise_dist.sample(&mut rng),
                transformed.y + noise_dist.sample(&mut rng),
                transformed.z + noise_dist.sample(&mut rng),
            );
        }
    }

    println!("  Target points: {}", target.len());
    println!("  Source points: {}", source.len());
    println!(
        "  Applied transform to source: rotation={:.3} rad (~{:.1}°), translation=[{:.1}, {:.1}, {:.1}]",
        rotation_angle,
        rotation_angle.to_degrees(),
        translation.x,
        translation.y,
        translation.z
    );

    // Registration will find the inverse transform (from source back to target)
    let expected_transform = true_transform.inverse();
    let expected_trans = expected_transform.translation.vector;
    let expected_rotation = expected_transform.rotation.euler_angles();
    println!(
        "  Expected registration result: rotation={:.3} rad (~{:.1}°), translation=[{:.3}, {:.3}, {:.3}]",
        expected_rotation.2,
        expected_rotation.2.to_degrees(),
        expected_trans.x,
        expected_trans.y,
        expected_trans.z
    );

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

        // Extract transformation from result
        let estimated_transform = &result.t_target_source;
        let translation = estimated_transform.translation.vector;
        let rotation = estimated_transform.rotation.euler_angles();

        println!(
            "  Estimated translation: [{:.3}, {:.3}, {:.3}]",
            translation.x, translation.y, translation.z
        );
        println!(
            "  Estimated rotation: {:.3} rad (~{:.1}°) around Z",
            rotation.2,
            rotation.2.to_degrees()
        );

        // Compare with ground truth (inverse of applied transform)
        let expected_trans = true_transform.inverse().translation.vector;
        let trans_error = (translation - expected_trans).norm();
        println!("  Translation error: {:.6}", trans_error);
    }

    // Example 2: Plane ICP with normal estimation
    println!("\n2. Point-to-Plane ICP Registration");
    println!("----------------------------------");
    {
        // Preprocess target to estimate normals
        let (processed_target, target_tree) = preprocess_points(&target, 0.2, 20, 4)?;
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

        if result.converged {
            // Extract transformation from result
            let estimated_transform = &result.t_target_source;
            let translation = estimated_transform.translation.vector;
            let rotation = estimated_transform.rotation.euler_angles();

            println!(
                "  Estimated translation: [{:.3}, {:.3}, {:.3}]",
                translation.x, translation.y, translation.z
            );
            println!(
                "  Estimated rotation: {:.3} rad (~{:.1}°) around Z",
                rotation.2,
                rotation.2.to_degrees()
            );

            // Compare with ground truth
            let expected_trans = true_transform.inverse().translation.vector;
            let trans_error = (translation - expected_trans).norm();
            println!("  Translation error: {:.6}", trans_error);
        }
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

        if result.converged {
            // Extract transformation from result
            let estimated_transform = &result.t_target_source;
            let translation = estimated_transform.translation.vector;
            let rotation = estimated_transform.rotation.euler_angles();

            println!(
                "  Estimated translation: [{:.3}, {:.3}, {:.3}]",
                translation.x, translation.y, translation.z
            );
            println!(
                "  Estimated rotation: {:.3} rad (~{:.1}°) around Z",
                rotation.2,
                rotation.2.to_degrees()
            );

            // Compare with ground truth
            let expected_trans = true_transform.inverse().translation.vector;
            let trans_error = (translation - expected_trans).norm();
            println!("  Translation error: {:.6}", trans_error);
        }
    }

    // Example 4: VGICP Registration with voxel maps
    println!("\n4. VGICP Registration");
    println!("---------------------");
    {
        // First preprocess the target to downsample it
        let (processed_target, _) = preprocess_points(&target, 0.2, 20, 4)?;
        println!("  Downsampled target: {} points", processed_target.len());

        // Create a voxel map from the downsampled target
        let voxel_resolution = 1.0; // Standard resolution as in C++ tests
        let voxelmap = create_gaussian_voxelmap(&processed_target, voxel_resolution)?;
        println!("  Created voxel map with resolution: {}", voxel_resolution);
        println!("  Voxel map size: {} voxels", voxelmap.len());

        // Configure VGICP
        let settings = VgicpSettings {
            voxel_resolution,
            max_iterations: 50,
            max_correspondence_distance: 2.0,
            ..Default::default()
        };

        // Run VGICP registration
        let result = align_vgicp(&source, &voxelmap, None, settings)?;

        println!("  Converged: {}", result.converged);
        println!("  Iterations: {}", result.iterations);
        println!("  Final error: {:.6}", result.error);

        // Extract transformation from result
        let estimated_transform = &result.t_target_source;
        let translation = estimated_transform.translation.vector;
        let rotation = estimated_transform.rotation.euler_angles();

        println!(
            "  Estimated translation: [{:.3}, {:.3}, {:.3}]",
            translation.x, translation.y, translation.z
        );
        println!(
            "  Estimated rotation: {:.3} rad (~{:.1}°) around Z",
            rotation.2,
            rotation.2.to_degrees()
        );

        // Note: VGICP currently has issues - it converges immediately without iterating
        // This appears to be related to how the voxel map is treated in the FFI layer
        if result.iterations == 0 {
            println!(
                "  ⚠️  VGICP converged without iterations - implementation needs investigation"
            );
        } else {
            // Compare with ground truth
            let expected_trans = true_transform.inverse().translation.vector;
            let trans_error = (translation - expected_trans).norm();
            println!("  Translation error: {:.6}", trans_error);
        }
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

        if result.converged {
            // Extract transformation from result
            let estimated_transform = &result.t_target_source;
            let translation = estimated_transform.translation.vector;
            let rotation = estimated_transform.rotation.euler_angles();

            println!(
                "  Estimated translation: [{:.3}, {:.3}, {:.3}]",
                translation.x, translation.y, translation.z
            );
            println!(
                "  Estimated rotation: {:.3} rad (~{:.1}°) around Z",
                rotation.2,
                rotation.2.to_degrees()
            );

            // Compare with ground truth
            let expected_trans = true_transform.inverse().translation.vector;
            let trans_error = (translation - expected_trans).norm();
            println!("  Translation error: {:.6}", trans_error);
        }
    }

    println!("\n✓ Registration examples completed successfully!");
    Ok(())
}
