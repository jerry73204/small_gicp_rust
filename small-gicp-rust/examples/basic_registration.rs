//! Basic point cloud registration example.
//!
//! This example demonstrates how to use the high-level API to perform
//! point cloud registration using different algorithms.

use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use small_gicp_rust::prelude::*;

fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    println!("Small GICP Rust - Basic Registration Example");
    println!("============================================");

    // Create synthetic point clouds
    let (target, source) = create_synthetic_data();

    println!("\nCreated point clouds:");
    println!("  Target: {} points", target.len());
    println!("  Source: {} points", source.len());

    // Example 1: Basic registration with automatic preprocessing
    println!("\n1. Basic GICP Registration");
    println!("--------------------------");

    let settings = RegistrationSettings {
        registration_type: RegistrationType::Gicp,
        num_threads: 4,
        initial_guess: None,
    };

    let result = register(&target, &source, &settings)?;
    print_registration_result(&result);

    // Example 2: Registration with manual preprocessing
    println!("\n2. Registration with Manual Preprocessing");
    println!("----------------------------------------");

    let target_processed = target.preprocess_points(&PreprocessorConfig {
        downsampling_resolution: 0.05,
        num_neighbors: 20,
        num_threads: 4,
    })?;
    let source_processed = source.preprocess_points(&PreprocessorConfig {
        downsampling_resolution: 0.05,
        num_neighbors: 20,
        num_threads: 4,
    })?;

    println!("Preprocessed clouds:");
    println!(
        "  Target: {} → {} points",
        target.len(),
        target_processed.cloud.len()
    );
    println!(
        "  Source: {} → {} points",
        source.len(),
        source_processed.cloud.len()
    );

    let target_tree = &target_processed.kdtree;
    let result = register_preprocessed(
        &target_processed.cloud,
        &source_processed.cloud,
        target_tree,
        &settings,
    )?;
    print_registration_result(&result);

    // Example 3: Different registration algorithms
    println!("\n3. Comparison of Registration Algorithms");
    println!("---------------------------------------");

    let algorithms = [
        ("ICP", RegistrationType::Icp),
        ("Plane ICP", RegistrationType::PlaneIcp),
        ("GICP", RegistrationType::Gicp),
    ];

    for (name, reg_type) in algorithms {
        let settings = RegistrationSettings {
            registration_type: reg_type,
            num_threads: 1,
            initial_guess: None,
        };

        match register(&target, &source, &settings) {
            Ok(result) => {
                println!(
                    "{}: error={:.6}, iterations={}, converged={}",
                    name, result.error, result.iterations, result.converged
                );
            }
            Err(e) => {
                println!("{}: failed - {}", name, e);
            }
        }
    }

    // Example 4: VGICP registration
    println!("\n4. VGICP Registration");
    println!("---------------------");

    let voxelmap_config = GaussianVoxelMapConfig {
        voxel_resolution: 0.1,
        num_threads: 4,
    };
    let target_voxelmap = GaussianVoxelMap::new(&target, &voxelmap_config)?;
    let vgicp_settings = RegistrationSettings {
        registration_type: RegistrationType::Vgicp,
        num_threads: 4,
        initial_guess: None,
    };

    let result = register_vgicp(&target_voxelmap, &source, &vgicp_settings)?;
    print_registration_result(&result);

    // Example 5: Registration with initial guess
    println!("\n5. Registration with Initial Guess");
    println!("----------------------------------");

    // Create a small perturbation as initial guess
    let initial_translation = Translation3::new(0.1, 0.1, 0.0);
    let initial_rotation = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.1);
    let initial_guess = Isometry3::from_parts(initial_translation, initial_rotation);

    let settings_with_guess = RegistrationSettings {
        registration_type: RegistrationType::Gicp,
        num_threads: 4,
        initial_guess: Some(initial_guess),
    };

    let result = register(&target, &source, &settings_with_guess)?;
    print_registration_result(&result);

    println!("\nExample completed successfully!");
    Ok(())
}

fn create_synthetic_data() -> (PointCloud, PointCloud) {
    // Create a simple synthetic point cloud (a cube)
    let mut target_points = Vec::new();
    let mut source_points = Vec::new();

    // Generate points in a 10x10x10 grid
    for i in 0..10 {
        for j in 0..10 {
            for k in 0..10 {
                let x = i as f64 * 0.1;
                let y = j as f64 * 0.1;
                let z = k as f64 * 0.1;
                target_points.push(Point3::new(x, y, z));

                // Source is the same cloud with small translation and rotation
                let transformed_point = transform_point(
                    Point3::new(x, y, z),
                    Vector3::new(0.05, 0.03, 0.02), // translation
                    0.1,                            // small rotation around z-axis
                );
                source_points.push(transformed_point);
            }
        }
    }

    // Add some noise to make it more realistic
    for point in &mut source_points {
        let noise_scale = 0.01;
        point.x += (point.x * 12345.0).sin() * noise_scale;
        point.y += (point.y * 23456.0).sin() * noise_scale;
        point.z += (point.z * 34567.0).sin() * noise_scale;
    }

    let target = PointCloud::from_points(&target_points).expect("Failed to create target cloud");
    let source = PointCloud::from_points(&source_points).expect("Failed to create source cloud");

    (target, source)
}

fn transform_point(point: Point3<f64>, translation: Vector3<f64>, rotation_z: f64) -> Point3<f64> {
    let cos_r = rotation_z.cos();
    let sin_r = rotation_z.sin();

    let rotated_x = point.x * cos_r - point.y * sin_r;
    let rotated_y = point.x * sin_r + point.y * cos_r;
    let rotated_z = point.z;

    Point3::new(
        rotated_x + translation.x,
        rotated_y + translation.y,
        rotated_z + translation.z,
    )
}

fn print_registration_result(result: &RegistrationResult) {
    println!("Registration result:");
    println!("  Converged: {}", result.converged);
    println!("  Iterations: {}", result.iterations);
    println!("  Final error: {:.6}", result.error);
    println!("  Inliers: {}", result.num_inliers);

    let translation = result.translation();
    let rotation = result.rotation();

    println!(
        "  Translation: [{:.4}, {:.4}, {:.4}]",
        translation.x, translation.y, translation.z
    );

    let euler = rotation.euler_angles();
    println!(
        "  Rotation (RPY): [{:.4}, {:.4}, {:.4}] rad",
        euler.0, euler.1, euler.2
    );
}
