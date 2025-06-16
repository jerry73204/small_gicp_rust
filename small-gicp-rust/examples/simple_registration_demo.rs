use small_gicp::{prelude::*, registration::estimate_transformation};

fn main() -> small_gicp::Result<()> {
    println!("Simple Registration Demo");

    // Create source point cloud with more points
    let mut source = PointCloud::new()?;
    for i in 0..10 {
        let x = (i as f64) * 0.1;
        source.add_point(x, 0.0, 0.0);
        source.add_point(x, 0.1, 0.0);
    }

    // Create target point cloud (slightly translated)
    let mut target = PointCloud::new()?;
    for i in 0..10 {
        let x = (i as f64) * 0.1 + 0.05; // Small translation
        target.add_point(x, 0.02, 0.0); // Small translation in Y
        target.add_point(x, 0.12, 0.0);
    }

    println!("Source cloud has {} points", source.size());
    println!("Target cloud has {} points", target.size());

    // Test individual registration functions

    // 1. Test basic registration function
    println!("\n1. Testing basic register function:");
    let settings = RegistrationSettings::new(RegistrationType::Icp)
        .with_max_iterations(10)
        .with_max_correspondence_distance(0.2);

    match register(&target, &source, &settings) {
        Ok(result) => {
            println!("  ✓ Registration successful!");
            println!("  Converged: {}", result.converged);
            println!("  Iterations: {}", result.iterations);
            println!("  Error: {:.6}", result.error);
        }
        Err(e) => {
            println!("  ✗ Registration failed: {:?}", e);
        }
    }

    // 2. Test robust kernel
    println!("\n2. Testing robust kernel:");
    match RobustKernel::huber(1.0) {
        Ok(kernel) => {
            let weight = kernel.compute_weight(0.5)?;
            println!("  ✓ Huber kernel weight for error 0.5: {:.3}", weight);
        }
        Err(e) => {
            println!("  ✗ Robust kernel failed: {:?}", e);
        }
    }

    // 3. Test DOF restriction
    println!("\n3. Testing DOF restriction:");
    let dof = DofRestriction::planar_2d()?;
    println!("  ✓ Planar 2D DOF restriction created");
    println!("  Translation mask: {:?}", dof.translation_mask);
    println!("  Rotation mask: {:?}", dof.rotation_mask);

    // 4. Test transformation estimation
    println!("\n4. Testing transformation estimation:");
    let source_points = vec![(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)];
    let target_points = vec![(0.1, 0.1, 0.0), (1.1, 0.1, 0.0), (0.1, 1.1, 0.0)];

    match estimate_transformation(&source_points, &target_points) {
        Ok(transform) => {
            println!("  ✓ Transformation estimated");
            println!(
                "  Translation: [{:.3}, {:.3}, {:.3}]",
                transform[(0, 3)],
                transform[(1, 3)],
                transform[(2, 3)]
            );
        }
        Err(e) => {
            println!("  ✗ Transformation estimation failed: {:?}", e);
        }
    }

    println!("\n✓ Registration module test completed successfully!");
    Ok(())
}
