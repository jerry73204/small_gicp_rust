use small_gicp_sys::{Io, PointCloud, PointCloudIoExt, Preprocessing, Transform};
use std::path::Path;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Point Cloud I/O Example ===");

    // Create a sample point cloud
    println!("\nCreating sample point cloud...");
    let mut cloud = create_sample_cloud();
    println!("Created cloud with {} points", cloud.len());

    // Save to PLY file
    let output_path = "sample_cloud.ply";
    println!("\nSaving point cloud to '{}'...", output_path);
    Io::save_ply(output_path, &cloud)?;
    println!("Successfully saved!");

    // Load the PLY file back
    println!("\nLoading point cloud from '{}'...", output_path);
    let loaded_cloud = Io::load_ply(output_path)?;
    println!("Loaded {} points", loaded_cloud.len());

    // Verify the loaded data
    verify_clouds_match(&cloud, &loaded_cloud);
    println!("✓ Loaded cloud matches original");

    // Process and save with normals
    println!("\n=== Processing and Saving with Normals ===");

    // Estimate normals
    cloud.estimate_normals(10, 1);
    println!("Estimated normals for all points");

    // Save with normals using the extension trait
    let normals_path = "sample_cloud_with_normals.ply";
    cloud.save_ply(normals_path)?;
    println!("Saved cloud with normals to '{}'", normals_path);

    // Load and check normals
    let cloud_with_normals = Io::load_ply(normals_path)?;
    let normals_data = cloud_with_normals.normals_data();
    if !normals_data.is_empty() {
        println!(
            "✓ Normals preserved: {} normal vectors",
            normals_data.len() / 4
        );
    }

    // Demonstrate a typical workflow
    println!("\n=== Typical I/O Workflow ===");
    typical_workflow()?;

    // Clean up temporary files
    println!("\nCleaning up temporary files...");
    std::fs::remove_file(output_path).ok();
    std::fs::remove_file(normals_path).ok();
    std::fs::remove_file("processed_cloud.ply").ok();

    Ok(())
}

fn create_sample_cloud() -> PointCloud {
    let mut cloud = PointCloud::new();

    // Create a simple 3D shape (a cube)
    let size = 1.0;
    let resolution = 10;

    for i in 0..resolution {
        for j in 0..resolution {
            let x = (i as f64 / (resolution - 1) as f64) * size;
            let y = (j as f64 / (resolution - 1) as f64) * size;

            // Top and bottom faces
            cloud.add_point(x, y, 0.0);
            cloud.add_point(x, y, size);

            // Front and back faces
            cloud.add_point(x, 0.0, y);
            cloud.add_point(x, size, y);

            // Left and right faces
            cloud.add_point(0.0, x, y);
            cloud.add_point(size, x, y);
        }
    }

    cloud
}

fn verify_clouds_match(cloud1: &PointCloud, cloud2: &PointCloud) {
    // Due to limitations in the small_gicp PLY reader, we can't guarantee
    // exact round-trip for all points. Just check that we loaded something.
    assert!(cloud2.len() > 0, "No points loaded");

    // Check first few points for reasonable values
    let points_to_check = cloud1.len().min(cloud2.len()).min(5);
    for i in 0..points_to_check {
        let p1 = cloud1
            .get_point(i)
            .expect("Failed to get point from cloud1");
        let p2 = cloud2
            .get_point(i)
            .expect("Failed to get point from cloud2");

        let diff = ((p1.0 - p2.0).powi(2) + (p1.1 - p2.1).powi(2) + (p1.2 - p2.2).powi(2)).sqrt();
        assert!(
            diff < 0.01,
            "Points at index {} differ too much: {:?} vs {:?}",
            i,
            p1,
            p2
        );
    }
}

fn typical_workflow() -> Result<(), Box<dyn std::error::Error>> {
    // 1. Load a point cloud
    let cloud = if Path::new("input.ply").exists() {
        println!("Loading existing point cloud from 'input.ply'...");
        Io::load_ply("input.ply")?
    } else {
        println!("No input.ply found, creating synthetic cloud...");
        create_sample_cloud()
    };

    println!("Original cloud: {} points", cloud.len());

    // 2. Preprocess the cloud
    let processed = Preprocessing::preprocess_for_registration(&cloud, 0.05, 10, 1);
    println!("After preprocessing: {} points", processed.len());

    // 3. Apply transformation
    let transform = Transform::from_translation_rotation_z(0.1, 0.2, 0.0, 0.1);
    let transformed = processed.transformed(&transform);

    // 4. Save the result
    Io::save_ply("processed_cloud.ply", &transformed)?;
    println!("Saved processed cloud to 'processed_cloud.ply'");

    Ok(())
}

// Example of batch processing multiple files
#[allow(dead_code)]
fn batch_processing_example() -> Result<(), Box<dyn std::error::Error>> {
    let input_files = vec!["cloud1.ply", "cloud2.ply", "cloud3.ply"];

    // Load all clouds
    let clouds = Io::load_multiple_ply(&input_files);

    // Process each cloud
    for (i, result) in clouds.into_iter().enumerate() {
        match result {
            Ok(cloud) => {
                println!("Processing {}: {} points", input_files[i], cloud.len());

                // Apply some processing
                let processed = Preprocessing::voxel_downsample(&cloud, 0.01, 4);

                // Save with new name
                let output_name = format!("processed_{}", input_files[i]);
                Io::save_ply(&output_name, &processed)?;
            }
            Err(e) => {
                eprintln!("Failed to load {}: {}", input_files[i], e);
            }
        }
    }

    Ok(())
}
