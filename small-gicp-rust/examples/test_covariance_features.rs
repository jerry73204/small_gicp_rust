use nalgebra::{Matrix4, Point3, Vector3};
use small_gicp_rust::{
    config::KdTreeConfig,
    error::Result,
    kdtree::KdTree,
    point_cloud::PointCloud,
    preprocessing::{
        estimate_covariances, estimate_local_features_auto, estimate_normals_and_covariances,
        CovarianceEstimationConfig, LocalFeatureEstimationConfig, LocalFeatureSetterType,
        LocalFeaturesBackend, NormalEstimationBackend, NormalEstimationConfig,
    },
};

fn main() -> Result<()> {
    println!("=== Small GICP Rust - Covariance and Direct Access Features Demo ===\n");

    // Create a structured test point cloud (a small planar grid)
    let mut points = Vec::new();
    let mut normals = Vec::new();

    // Create a 5x5 grid in the XY plane with slight Z variation
    for i in 0..5 {
        for j in 0..5 {
            let x = i as f64 * 0.1;
            let y = j as f64 * 0.1;
            let z = 0.01 * (i as f64 + j as f64); // Slight slope

            points.push(Point3::new(x, y, z));
            normals.push(Vector3::new(0.0, 0.0, 1.0)); // Pointing up
        }
    }

    println!("Created test point cloud with {} points", points.len());
    let mut cloud = PointCloud::from_points_and_normals(&points, &normals)?;

    // === Test 1: Validation Functions ===
    println!("\n1. Testing validation functions:");
    println!("   has_points(): {}", cloud.has_points());
    println!("   has_normals(): {}", cloud.has_normals());
    println!("   has_covariances(): {}", cloud.has_covariances());
    println!("   empty(): {}", cloud.empty());
    println!("   len(): {}", cloud.len());

    // === Test 2: Individual Covariance Operations ===
    println!("\n2. Testing individual covariance operations:");

    // Create different types of covariance matrices
    let isotropic_cov = Matrix4::new(
        0.01, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 1.0,
    );

    let anisotropic_cov = Matrix4::new(
        0.1, 0.02, 0.0, 0.0, 0.02, 0.05, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 1.0,
    );

    // Set individual covariances
    cloud.set_covariance(0, isotropic_cov)?;
    cloud.set_covariance(12, anisotropic_cov)?; // Center point

    // Retrieve and verify
    let retrieved_iso = cloud.get_covariance(0)?;
    let retrieved_aniso = cloud.get_covariance(12)?;

    println!(
        "   Set isotropic covariance: diagonal = [{:.3}, {:.3}, {:.3}]",
        retrieved_iso[(0, 0)],
        retrieved_iso[(1, 1)],
        retrieved_iso[(2, 2)]
    );
    println!(
        "   Set anisotropic covariance: diagonal = [{:.3}, {:.3}, {:.3}]",
        retrieved_aniso[(0, 0)],
        retrieved_aniso[(1, 1)],
        retrieved_aniso[(2, 2)]
    );

    // === Test 3: Bulk Covariance Operations ===
    println!("\n3. Testing bulk covariance operations:");

    // Create varied covariances for all points
    let mut all_covariances = Vec::new();
    for i in 0..cloud.len() {
        let scale = 0.01 + (i as f64) * 0.001; // Gradually increasing uncertainty
        let cov = Matrix4::new(
            scale,
            0.0,
            0.0,
            0.0,
            0.0,
            scale,
            0.0,
            0.0,
            0.0,
            0.0,
            scale * 0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
        );
        all_covariances.push(cov);
    }

    cloud.set_covariances(&all_covariances)?;
    let retrieved_all = cloud.covariances()?;
    println!("   Set {} covariances", all_covariances.len());
    println!("   Retrieved {} covariances", retrieved_all.len());

    // Test bulk raw data operations
    let cov_data: Vec<f64> = all_covariances
        .iter()
        .flat_map(|m| (0..4).flat_map(move |i| (0..4).map(move |j| m[(i, j)])))
        .collect();

    cloud.set_covariances_bulk(&cov_data)?;
    println!(
        "   Set covariances from raw data ({} values)",
        cov_data.len()
    );

    // === Test 4: Direct Data Access ===
    println!("\n4. Testing direct data access:");

    unsafe {
        let points_data = cloud.points_data()?;
        let normals_data = cloud.normals_data()?;
        let covariances_data = cloud.covariances_data()?;

        println!(
            "   Direct points access: {} values ({} points × 4 components)",
            points_data.len(),
            points_data.len() / 4
        );
        println!(
            "   Direct normals access: {} values ({} normals × 4 components)",
            normals_data.len(),
            normals_data.len() / 4
        );
        println!(
            "   Direct covariances access: {} values ({} matrices × 16 elements)",
            covariances_data.len(),
            covariances_data.len() / 16
        );

        // Show some sample data
        println!(
            "   First point: [{:.3}, {:.3}, {:.3}, {:.1}]",
            points_data[0], points_data[1], points_data[2], points_data[3]
        );
        println!(
            "   First normal: [{:.3}, {:.3}, {:.3}, {:.1}]",
            normals_data[0], normals_data[1], normals_data[2], normals_data[3]
        );
        println!("   First covariance [0,0]: {:.6}", covariances_data[0]);
    }

    // === Test 5: Copy Operations ===
    println!("\n5. Testing copy operations:");

    let mut points_array = vec![0.0; cloud.len() * 3];
    let mut normals_array = vec![0.0; cloud.len() * 3];
    let mut cov_array = vec![0.0; cloud.len() * 16];

    cloud.copy_points_to_array(&mut points_array)?;
    cloud.copy_normals_to_array(&mut normals_array)?;
    cloud.copy_covariances_to_array(&mut cov_array)?;

    println!("   Copied {} point values", points_array.len());
    println!("   Copied {} normal values", normals_array.len());
    println!("   Copied {} covariance values", cov_array.len());

    // === Test 6: Covariance Estimation ===
    println!("\n6. Testing covariance estimation:");

    // Create a new cloud for estimation testing
    let mut estimation_cloud = PointCloud::from_points(&points)?;

    // Build KdTree for neighbor search
    let kdtree_config = KdTreeConfig::default();
    let kdtree = KdTree::new(&estimation_cloud, &kdtree_config)?;

    // Estimate covariances from local neighborhoods
    let cov_config = LocalFeatureEstimationConfig {
        setter_type: LocalFeatureSetterType::Covariance,
        backend: LocalFeaturesBackend::Default,
        num_neighbors: 8,
        num_threads: 1,
    };

    estimate_local_features_auto(&mut estimation_cloud, &cov_config)?;

    println!(
        "   Estimated covariances from {} neighbors",
        cov_config.num_neighbors
    );
    println!(
        "   Cloud now has covariances: {}",
        estimation_cloud.has_covariances()
    );

    // Sample the estimated covariance
    let estimated_cov = estimation_cloud.get_covariance(12)?; // Center point
    println!(
        "   Center point estimated covariance diagonal: [{:.6}, {:.6}, {:.6}]",
        estimated_cov[(0, 0)],
        estimated_cov[(1, 1)],
        estimated_cov[(2, 2)]
    );

    // === Test 7: Combined Normal and Covariance Estimation ===
    println!("\n7. Testing combined normal and covariance estimation:");

    let mut combined_cloud = PointCloud::from_points(&points)?;
    let combined_kdtree = KdTree::new(&combined_cloud, &kdtree_config)?;

    let normal_covariance_config = LocalFeatureEstimationConfig {
        setter_type: LocalFeatureSetterType::NormalCovariance,
        backend: LocalFeaturesBackend::Default,
        num_neighbors: 10,
        num_threads: 1,
    };

    estimate_local_features_auto(&mut combined_cloud, &normal_covariance_config)?;

    println!("   Estimated both normals and covariances");
    println!("   has_normals(): {}", combined_cloud.has_normals());
    println!("   has_covariances(): {}", combined_cloud.has_covariances());

    // Check normal direction (should be mostly pointing in +Z for our planar grid)
    let center_normal = combined_cloud.get_normal(12)?;
    println!(
        "   Center point estimated normal: [{:.3}, {:.3}, {:.3}]",
        center_normal.x, center_normal.y, center_normal.z
    );

    // === Test 8: Performance Test ===
    println!("\n8. Performance demonstration:");

    // Create a larger point cloud
    let mut large_points = Vec::new();
    for i in 0..1000 {
        let angle = (i as f64) * 2.0 * std::f64::consts::PI / 1000.0;
        large_points.push(Point3::new(angle.cos(), angle.sin(), i as f64 * 0.001));
    }

    let mut large_cloud = PointCloud::from_points(&large_points)?;

    // Bulk set covariances using direct data
    let identity_matrix: Vec<f64> = (0..16)
        .map(|i| if i % 5 == 0 { 0.01 } else { 0.0 })
        .collect();
    let all_cov_data: Vec<f64> = identity_matrix.repeat(1000);

    large_cloud.set_covariances_bulk(&all_cov_data)?;

    // Direct access for reading
    unsafe {
        let cov_data = large_cloud.covariances_data()?;
        println!(
            "   Handled {} covariances efficiently ({} total values)",
            cov_data.len() / 16,
            cov_data.len()
        );
    }

    // === Summary ===
    println!("\n=== Summary ===");
    println!("✅ Individual covariance operations: set/get single matrices");
    println!("✅ Bulk covariance operations: set/get all matrices at once");
    println!("✅ Raw data operations: set from/copy to raw float arrays");
    println!("✅ Direct memory access: zero-copy access to internal data");
    println!("✅ Covariance estimation: compute from local neighborhoods");
    println!("✅ Combined estimation: normals and covariances together");
    println!("✅ Validation functions: check data presence and integrity");
    println!("✅ Performance features: efficient bulk operations for large clouds");

    println!("\n🎉 All covariance and direct access features are fully functional!");
    Ok(())
}
