use small_gicp_sys::{PointCloud, Preprocessing, Transform};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Point Cloud Operations Example ===");

    // Create a point cloud using bulk operations for better performance
    let mut cloud = PointCloud::new();

    // Create a simple grid of points
    let mut points = Vec::new();
    for i in 0..10 {
        for j in 0..10 {
            points.extend_from_slice(&[
                i as f64 * 0.1,
                j as f64 * 0.1,
                0.0,
                1.0, // homogeneous coordinate
            ]);
        }
    }

    println!(
        "Creating point cloud with {} points using bulk operations",
        points.len() / 4
    );
    cloud.set_points_bulk(&points);
    println!("Point cloud size: {}", cloud.len());

    // Demonstrate point access
    if let Some((x, y, z)) = cloud.get_point(0) {
        println!("First point: ({:.2}, {:.2}, {:.2})", x, y, z);
    }

    // Apply transformation
    println!("\n=== Transformation Operations ===");

    // Create a translation + rotation transformation
    let transform =
        Transform::from_translation_rotation_z(1.0, 2.0, 0.5, std::f64::consts::PI / 4.0);
    println!(
        "Created transformation: translation({:.2}, {:.2}, {:.2}) + rotation_z({:.2} rad)",
        1.0,
        2.0,
        0.5,
        std::f64::consts::PI / 4.0
    );

    // Transform the cloud (creates a new cloud)
    let transformed_cloud = cloud.transformed(&transform);

    if let Some((x, y, z)) = transformed_cloud.get_point(0) {
        println!(
            "First point after transformation: ({:.2}, {:.2}, {:.2})",
            x, y, z
        );
    }

    // Original cloud should be unchanged
    if let Some((x, y, z)) = cloud.get_point(0) {
        println!(
            "Original first point (unchanged): ({:.2}, {:.2}, {:.2})",
            x, y, z
        );
    }

    // In-place transformation
    cloud.transform(&Transform::translation(0.0, 0.0, 1.0));
    if let Some((x, y, z)) = cloud.get_point(0) {
        println!(
            "First point after in-place translation: ({:.2}, {:.2}, {:.2})",
            x, y, z
        );
    }

    // Demonstrate preprocessing
    println!("\n=== Preprocessing Operations ===");

    // Create a denser point cloud for downsampling
    let mut dense_cloud = PointCloud::new();
    let mut dense_points = Vec::new();
    for i in 0..50 {
        for j in 0..50 {
            dense_points.extend_from_slice(&[i as f64 * 0.01, j as f64 * 0.01, 0.0, 1.0]);
        }
    }
    dense_cloud.set_points_bulk(&dense_points);
    println!("Dense cloud size: {}", dense_cloud.len());

    // Voxel grid downsampling
    let downsampled = Preprocessing::voxel_downsample(&dense_cloud, 0.05, 1);
    println!(
        "After voxel downsampling (voxel_size=0.05): {}",
        downsampled.len()
    );

    // Random downsampling
    let random_sampled = Preprocessing::random_downsample(&dense_cloud, 100);
    println!(
        "After random downsampling (100 points): {}",
        random_sampled.len()
    );

    // Estimate normals
    let mut normal_cloud = downsampled;
    Preprocessing::estimate_normals(&mut normal_cloud, 10, 1);
    println!("Estimated normals for {} points", normal_cloud.len());

    // Access raw data for performance-critical operations
    println!("\n=== Raw Data Access ===");
    let points_data = normal_cloud.points_data();
    println!(
        "Raw points data length: {} (4 values per point)",
        points_data.len()
    );

    let normals_data = normal_cloud.normals_data();
    if !normals_data.is_empty() {
        println!(
            "Raw normals data length: {} (4 values per normal)",
            normals_data.len()
        );
    }

    // Demonstrate transform utilities
    println!("\n=== Transform Utilities ===");

    let _identity = Transform::identity(); // TODO: Use this transform for demonstration
    let translation = Transform::translation(1.0, 2.0, 3.0);
    let rotation_z = Transform::rotation_z(std::f64::consts::PI / 2.0);

    let combined = translation.compose(&rotation_z);
    let (tx, ty, tz) = combined.get_translation();
    println!(
        "Combined transform translation: ({:.2}, {:.2}, {:.2})",
        tx, ty, tz
    );

    let inverse = combined.inverse();
    let back_to_identity = combined.compose(&inverse);
    println!("Inverse transformation test - should be close to identity:");
    for i in 0..4 {
        for j in 0..4 {
            print!("{:8.4} ", back_to_identity.matrix[i * 4 + j]);
        }
        println!();
    }

    println!("\n=== Performance-Oriented Workflow ===");

    // Demonstrate a typical high-performance workflow
    let start_time = std::time::Instant::now();

    // 1. Create large point cloud with bulk operations
    let mut large_points = Vec::new();
    for i in 0..1000 {
        large_points.extend_from_slice(&[
            (i as f64 * 0.001) % 1.0,
            ((i * 7) as f64 * 0.001) % 1.0,
            ((i * 13) as f64 * 0.001) % 1.0,
            1.0,
        ]);
    }
    let mut large_cloud = PointCloud::from_points_bulk(&large_points);

    // 2. Preprocess with parallel operations
    let processed = Preprocessing::preprocess_for_registration(&large_cloud, 0.02, 10, 4);

    // 3. Apply transformation
    let transform = Transform::from_translation_rotation_z(0.1, 0.2, 0.3, 0.1);
    large_cloud.transform(&transform);

    let elapsed = start_time.elapsed();
    println!(
        "Processed {} points in {:.2}ms",
        large_points.len() / 4,
        elapsed.as_secs_f64() * 1000.0
    );
    println!("Final cloud size after preprocessing: {}", processed.len());

    Ok(())
}
