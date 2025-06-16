use nalgebra::Point3;
use small_gicp::prelude::*;

fn main() -> small_gicp::Result<()> {
    println!("Registration Demo");

    // Create source point cloud
    let mut source = PointCloud::new()?;
    source.add_point(0.0, 0.0, 0.0);
    source.add_point(1.0, 0.0, 0.0);
    source.add_point(0.0, 1.0, 0.0);
    source.add_point(1.0, 1.0, 0.0);

    // Create target point cloud (slightly translated)
    let mut target = PointCloud::new()?;
    target.add_point(0.1, 0.1, 0.0);
    target.add_point(1.1, 0.1, 0.0);
    target.add_point(0.1, 1.1, 0.0);
    target.add_point(1.1, 1.1, 0.0);

    println!("Source cloud has {} points", source.size());
    println!("Target cloud has {} points", target.size());

    // Create registration settings
    let settings = RegistrationSettings::new(RegistrationType::Icp)
        .with_max_iterations(50)
        .with_max_correspondence_distance(0.5);

    println!("Registration settings: {:?}", settings);

    // Perform registration
    match register(&target, &source, &settings) {
        Ok(result) => {
            println!("Registration result:");
            println!("  Converged: {}", result.converged);
            println!("  Iterations: {}", result.iterations);
            println!("  Error: {:.6}", result.error);
            println!("  Transformation matrix:");
            for i in 0..4 {
                println!(
                    "    [{:8.4}, {:8.4}, {:8.4}, {:8.4}]",
                    result.transformation[(i, 0)],
                    result.transformation[(i, 1)],
                    result.transformation[(i, 2)],
                    result.transformation[(i, 3)]
                );
            }

            // Test transformation extraction
            let translation = result.translation();
            let rotation = result.rotation();
            println!(
                "  Translation: [{:.4}, {:.4}, {:.4}]",
                translation.x, translation.y, translation.z
            );
            println!(
                "  Rotation quaternion: [{:.4}, {:.4}, {:.4}, {:.4}]",
                rotation.w, rotation.i, rotation.j, rotation.k
            );

            // Test point transformation
            let test_point = Point3::new(0.5, 0.5, 0.0);
            let transformed_point = result.transform_point(test_point);
            println!(
                "  Transform point {:?} -> {:?}",
                test_point, transformed_point
            );
        }
        Err(e) => {
            println!("Registration failed: {:?}", e);
        }
    }

    // Test VGICP registration
    println!("\nTesting VGICP registration:");

    // Create a voxel map from target
    let target_voxelmap = GaussianVoxelMap::with_voxel_size(0.1);
    let vgicp_settings = RegistrationSettings::new(RegistrationType::Vgicp).with_max_iterations(30);

    match register_vgicp(&target_voxelmap, &source, None, &vgicp_settings) {
        Ok(result) => {
            println!("VGICP Registration result:");
            println!("  Converged: {}", result.converged);
            println!("  Iterations: {}", result.iterations);
            println!("  Error: {:.6}", result.error);
        }
        Err(e) => {
            println!("VGICP Registration failed: {:?}", e);
        }
    }

    Ok(())
}
