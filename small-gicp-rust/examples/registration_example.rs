//! Example demonstrating the new registration API.

use rand::distributions::{Distribution, Uniform};
use small_gicp::{
    create_gaussian_voxelmap,
    kdtree::KdTree,
    point_cloud::PointCloud,
    preprocess_points,
    registration::{
        align_gicp, align_icp, align_plane_icp, align_vgicp, GicpSettings, IcpSettings,
        PlaneIcpSettings, RegistrationResult, VgicpSettings,
    },
    Result,
};

/// Data structure to hold generated point clouds and transformation
struct RegistrationData {
    source: PointCloud,
    target: PointCloud,
    true_transform: nalgebra::Isometry3<f64>,
}

/// Generate sample point clouds for registration testing
fn generate_sample_clouds() -> Result<RegistrationData> {
    println!("Creating sample point clouds...");

    let mut source = PointCloud::new()?;
    let mut target = PointCloud::new()?;

    // Generate a simple 3D pattern for target
    for i in 0..50 {
        for j in 0..50 {
            let x = i as f64 * 0.1;
            let y = j as f64 * 0.1;
            let z = (x * x + y * y).sin() * 0.5;
            target.add_point(x, y, z);
        }
    }

    // Create a random transformation to apply to source cloud
    let mut rng = rand::thread_rng();

    // Random rotation between -30 and 30 degrees (more reasonable for ICP convergence)
    let rotation_dist = Uniform::new(-30.0_f64.to_radians(), 30.0_f64.to_radians());
    let rotation_angle = rotation_dist.sample(&mut rng);

    // Random translation between -1.0 and 1.0 for each axis (reduced for better convergence)
    let trans_dist = Uniform::new(-1.0, 1.0);
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

    Ok(RegistrationData {
        source,
        target,
        true_transform,
    })
}

/// Print registration results and compare with ground truth
fn print_registration_results(
    result: &RegistrationResult,
    true_transform: &nalgebra::Isometry3<f64>,
    method_name: &str,
) {
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

        // Compare with ground truth (inverse of applied transform)
        let expected_trans = true_transform.inverse().translation.vector;
        let trans_error = (translation - expected_trans).norm();
        println!("  Translation error: {:.6}", trans_error);
    } else if method_name == "VGICP" && result.iterations == 0 {
        println!("  ⚠️  VGICP converged without iterations - implementation needs investigation");
    }
}

/// Demonstrate basic ICP registration
fn demo_basic_icp(data: &RegistrationData) -> Result<()> {
    println!("\n1. Basic ICP Registration");
    println!("-------------------------");

    // Build KdTree for target
    let target_tree = KdTree::new(&data.target)?;

    // Configure ICP with more robust settings
    let settings = IcpSettings {
        max_iterations: 100,
        max_correspondence_distance: 2.0, // Increased for better initial matching
        rotation_eps: 0.1 * std::f64::consts::PI / 180.0, // 0.1 degrees
        translation_eps: 1e-3,
        num_threads: 4,
    };

    // Provide a coarse initial guess (identity transform)
    // For better convergence with large transformations, you could compute
    // centroids and provide a translation-only initial guess
    let init_transform = None; // Using identity for now

    // Run registration
    let result = align_icp(
        &data.source,
        &data.target,
        &target_tree,
        init_transform,
        settings,
    )?;

    print_registration_results(&result, &data.true_transform, "ICP");

    Ok(())
}

/// Demonstrate Point-to-Plane ICP with normal estimation
fn demo_plane_icp(data: &RegistrationData) -> Result<()> {
    println!("\n2. Point-to-Plane ICP Registration");
    println!("----------------------------------");

    // Preprocess target to estimate normals
    let (processed_target, target_tree) = preprocess_points(&data.target, 0.2, 20, 4)?;
    let (processed_source, _) = preprocess_points(&data.source, 0.2, 20, 4)?;

    println!("  Target has normals: {}", processed_target.has_normals());

    // Configure Plane ICP with better settings
    let settings = PlaneIcpSettings {
        max_iterations: 100,
        max_correspondence_distance: 2.0,
        rotation_eps: 0.1 * std::f64::consts::PI / 180.0,
        translation_eps: 1e-3,
        num_threads: 4,
    };

    // Run registration
    let result = align_plane_icp(
        &processed_source,
        &processed_target,
        &target_tree,
        None,
        settings,
    )?;

    print_registration_results(&result, &data.true_transform, "PlaneICP");

    Ok(())
}

/// Demonstrate GICP registration
fn demo_gicp(data: &RegistrationData) -> Result<()> {
    println!("\n3. GICP Registration");
    println!("--------------------");

    // Preprocess both clouds (compute covariances)
    let (processed_target, target_tree) = preprocess_points(&data.target, 0.2, 20, 4)?;
    let (processed_source, source_tree) = preprocess_points(&data.source, 0.2, 20, 4)?;

    println!("  Downsampled target: {} points", processed_target.len());
    println!("  Downsampled source: {} points", processed_source.len());
    println!(
        "  Covariances computed: {}",
        processed_target.has_covariances()
    );

    // Configure GICP with better settings
    let settings = GicpSettings {
        max_iterations: 100,
        max_correspondence_distance: 2.0,
        rotation_eps: 0.1 * std::f64::consts::PI / 180.0,
        translation_eps: 1e-3,
        num_threads: 4,
    };

    // Run GICP (requires both source and target trees)
    let result = align_gicp(
        &processed_source,
        &processed_target,
        &source_tree,
        &target_tree,
        None,
        settings,
    )?;

    print_registration_results(&result, &data.true_transform, "GICP");

    Ok(())
}

/// Demonstrate VGICP registration with voxel maps
fn demo_vgicp(data: &RegistrationData) -> Result<()> {
    println!("\n4. VGICP Registration");
    println!("---------------------");

    // First preprocess the target to downsample it
    let (processed_target, _) = preprocess_points(&data.target, 0.2, 20, 4)?;
    println!("  Downsampled target: {} points", processed_target.len());

    // Create a voxel map from the downsampled target
    let voxel_resolution = 0.5; // Better resolution for our 5x5 point cloud
    let voxelmap = create_gaussian_voxelmap(&processed_target, voxel_resolution)?;
    println!("  Created voxel map with resolution: {}", voxel_resolution);
    println!("  Voxel map size: {} voxels", voxelmap.len());

    // Configure VGICP with better settings
    let settings = VgicpSettings {
        voxel_resolution,
        max_iterations: 100,
        max_correspondence_distance: 2.0,
        rotation_eps: 0.1 * std::f64::consts::PI / 180.0,
        translation_eps: 1e-3,
        num_threads: 4,
    };

    // For VGICP, also downsample the source for consistency
    let (processed_source, _) = preprocess_points(&data.source, 0.2, 20, 4)?;
    println!("  Downsampled source: {} points", processed_source.len());

    // Run VGICP registration
    let result = align_vgicp(&processed_source, &voxelmap, None, settings)?;

    // Extract transformation from result
    let estimated_transform = &result.t_target_source;
    let translation = estimated_transform.translation.vector;
    let rotation = estimated_transform.rotation.euler_angles();

    println!("  Converged: {}", result.converged);
    println!("  Iterations: {}", result.iterations);
    println!("  Final error: {:.6}", result.error);
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
    if result.iterations == 0 {
        println!("  ⚠️  VGICP converged without iterations - implementation needs investigation");
    } else {
        // Compare with ground truth
        let expected_trans = data.true_transform.inverse().translation.vector;
        let trans_error = (translation - expected_trans).norm();
        println!("  Translation error: {:.6}", trans_error);
    }

    Ok(())
}

/// Demonstrate registration with custom settings
fn demo_custom_settings(data: &RegistrationData) -> Result<()> {
    println!("\n5. Registration with Custom Settings");
    println!("------------------------------------");

    let target_tree = KdTree::new(&data.target)?;

    // Create custom ICP settings
    let settings = IcpSettings {
        max_correspondence_distance: 2.0,
        rotation_eps: 0.001,
        translation_eps: 0.001,
        num_threads: 8,
        max_iterations: 100,
    };

    let result = align_icp(&data.source, &data.target, &target_tree, None, settings)?;

    println!("  Used custom convergence criteria");
    print_registration_results(&result, &data.true_transform, "CustomICP");

    Ok(())
}

fn main() -> Result<()> {
    println!("Small GICP Registration Example");
    println!("==============================\n");

    // Generate sample point clouds with random transformation
    let data = generate_sample_clouds()?;

    // Run various registration algorithms
    demo_basic_icp(&data)?;
    demo_plane_icp(&data)?;
    demo_gicp(&data)?;
    demo_vgicp(&data)?;
    demo_custom_settings(&data)?;

    println!("\n✓ Registration examples completed successfully!");
    Ok(())
}
