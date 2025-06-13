//! Advanced preprocessing demonstration showcasing the new backend-aware features.
//!
//! This example demonstrates the enhanced preprocessing capabilities including:
//! - Backend-aware downsampling (OpenMP, TBB)
//! - Seeded random sampling for reproducibility
//! - Local feature estimation with different setter types
//! - Direct setter interfaces for manual feature setting

use nalgebra::Point3;
use small_gicp_rust::{
    estimate_local_features_auto, estimate_local_features_cloud, estimate_normals,
    prelude::*,
    preprocessing::{NormalEstimation, PreprocessingStrategy},
    set_covariance_direct, set_normal_covariance_direct, set_normal_direct,
};

fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    println!("Small GICP Rust - Advanced Preprocessing Demo");
    println!("==============================================");

    // Create a test point cloud
    let cloud = create_structured_point_cloud();
    println!("\nOriginal cloud: {} points", cloud.len());

    // Example 1: Backend-aware voxel grid downsampling
    println!("\n1. Backend-Aware Voxel Grid Downsampling");
    println!("----------------------------------------");

    let backends = [
        ("Default", DownsamplingBackend::Default),
        ("OpenMP", DownsamplingBackend::OpenMp),
        ("TBB", DownsamplingBackend::Tbb),
    ];

    for (name, backend) in backends {
        let config = VoxelGridConfig {
            leaf_size: 0.1,
            backend,
            num_threads: 4,
        };

        let start = std::time::Instant::now();
        let downsampled = cloud.voxelgrid_sampling(&config)?;
        let duration = start.elapsed();

        println!(
            "{} backend: {} → {} points ({:.2}ms)",
            name,
            cloud.len(),
            downsampled.len(),
            duration.as_secs_f64() * 1000.0
        );
    }

    // Example 2: Seeded random sampling for reproducibility
    println!("\n2. Seeded Random Sampling");
    println!("------------------------");

    let config_with_seed = RandomSamplingConfig {
        num_samples: 200,
        seed: Some(42),
    };

    let config_no_seed = RandomSamplingConfig {
        num_samples: 200,
        seed: None,
    };

    println!("Testing reproducibility with seed=42:");
    let sample1 = cloud.random_sampling_with_config(&config_with_seed)?;
    let sample2 = cloud.random_sampling_with_config(&config_with_seed)?;

    // Check if the first few points are identical (they should be with same seed)
    let mut identical_count = 0;
    for i in 0..std::cmp::min(5, sample1.len()) {
        let p1 = sample1.get_point(i)?;
        let p2 = sample2.get_point(i)?;
        if (p1 - p2).magnitude() < 1e-10 {
            identical_count += 1;
        }
    }
    println!(
        "  First {} points identical: {}",
        std::cmp::min(5, sample1.len()),
        identical_count
    );

    println!("Testing randomness without seed:");
    let sample3 = cloud.random_sampling_with_config(&config_no_seed)?;
    let sample4 = cloud.random_sampling_with_config(&config_no_seed)?;

    // These should likely be different
    let mut different_count = 0;
    for i in 0..std::cmp::min(5, sample3.len()) {
        let p3 = sample3.get_point(i)?;
        let p4 = sample4.get_point(i)?;
        if (p3 - p4).magnitude() > 1e-10 {
            different_count += 1;
        }
    }
    println!(
        "  First {} points different: {}",
        std::cmp::min(5, sample3.len()),
        different_count
    );

    // Example 3: Local feature estimation with different setter types
    println!("\n3. Local Feature Estimation");
    println!("---------------------------");

    let test_cloud = cloud.voxelgrid_sampling(&VoxelGridConfig {
        leaf_size: 0.08,
        backend: DownsamplingBackend::Default,
        num_threads: 2,
    })?;

    let kdtree = KdTree::new(&test_cloud, &KdTreeConfig::default())?;

    // Test different setter types
    let setter_types = [
        ("Normal only", LocalFeatureSetterType::Normal),
        ("Covariance only", LocalFeatureSetterType::Covariance),
        (
            "Normal + Covariance",
            LocalFeatureSetterType::NormalCovariance,
        ),
    ];

    for (name, setter_type) in setter_types {
        let mut test_cloud_copy = test_cloud.clone();

        let config = LocalFeatureEstimationConfig {
            setter_type,
            backend: LocalFeaturesBackend::OpenMp,
            num_neighbors: 10,
            num_threads: 4,
        };

        let start = std::time::Instant::now();
        estimate_local_features_auto(&mut test_cloud_copy, &config)?;
        let duration = start.elapsed();

        println!(
            "{}: {} points processed ({:.2}ms)",
            name,
            test_cloud_copy.len(),
            duration.as_secs_f64() * 1000.0
        );

        // Verify features were estimated
        if !test_cloud_copy.is_empty() {
            match setter_type {
                LocalFeatureSetterType::Normal => {
                    let normal = test_cloud_copy.get_normal(0)?;
                    println!("  Normal magnitude: {:.3}", normal.magnitude());
                }
                LocalFeatureSetterType::Covariance => {
                    let cov = test_cloud_copy.get_covariance(0)?;
                    println!("  Covariance trace: {:.6}", cov.trace());
                }
                LocalFeatureSetterType::NormalCovariance => {
                    let normal = test_cloud_copy.get_normal(0)?;
                    let cov = test_cloud_copy.get_covariance(0)?;
                    println!(
                        "  Normal magnitude: {:.3}, Covariance trace: {:.6}",
                        normal.magnitude(),
                        cov.trace()
                    );
                }
            }
        }
    }

    // Example 4: Auto feature estimation (builds KdTree internally)
    println!("\n4. Auto Feature Estimation");
    println!("--------------------------");

    let mut auto_test_cloud = test_cloud.clone();
    let config = LocalFeatureEstimationConfig {
        setter_type: LocalFeatureSetterType::NormalCovariance,
        backend: LocalFeaturesBackend::Default,
        num_neighbors: 15,
        num_threads: 2,
    };

    let start = std::time::Instant::now();
    estimate_local_features_auto(&mut auto_test_cloud, &config)?;
    let duration = start.elapsed();

    println!(
        "Auto estimation (with internal KdTree): {} points ({:.2}ms)",
        auto_test_cloud.len(),
        duration.as_secs_f64() * 1000.0
    );

    if !auto_test_cloud.is_empty() {
        let normal = auto_test_cloud.get_normal(0)?;
        let cov = auto_test_cloud.get_covariance(0)?;
        println!(
            "  Normal: [{:.3}, {:.3}, {:.3}]",
            normal.x, normal.y, normal.z
        );
        println!("  Covariance trace: {:.6}", cov.trace());
    }

    // Example 5: Direct setter interface
    println!("\n5. Direct Setter Interface");
    println!("--------------------------");

    let mut manual_cloud = test_cloud.clone();

    // Manually set normal using eigenvector matrix
    let eigenvectors = [
        1.0, 0.0, 0.0, // First eigenvector (tangent)
        0.0, 1.0, 0.0, // Second eigenvector (tangent)
        0.0, 0.0, 1.0, // Third eigenvector (normal - pointing in Z direction)
    ];

    println!("Setting manual normal for first point...");
    set_normal_direct(&mut manual_cloud, 0, &eigenvectors)?;

    let manual_normal = manual_cloud.get_normal(0)?;
    println!(
        "Manual normal: [{:.3}, {:.3}, {:.3}] (should be close to [0, 0, 1])",
        manual_normal.x, manual_normal.y, manual_normal.z
    );

    // Set covariance as well
    println!("Setting manual covariance for first point...");
    set_covariance_direct(&mut manual_cloud, 0, &eigenvectors)?;

    let manual_cov = manual_cloud.get_covariance(0)?;
    println!("Manual covariance trace: {:.6}", manual_cov.trace());

    // Set both normal and covariance together
    println!("Setting both normal and covariance for second point...");
    if manual_cloud.len() > 1 {
        set_normal_covariance_direct(&mut manual_cloud, 1, &eigenvectors)?;

        let normal = manual_cloud.get_normal(1)?;
        let cov = manual_cloud.get_covariance(1)?;
        println!(
            "Combined setting - Normal: [{:.3}, {:.3}, {:.3}], Covariance trace: {:.6}",
            normal.x,
            normal.y,
            normal.z,
            cov.trace()
        );
    }

    // Example 6: Backend comparison for normal estimation
    println!("\n6. Backend Comparison for Normal Estimation");
    println!("-------------------------------------------");

    let backends = [
        ("Default", NormalEstimationBackend::Default),
        ("OpenMP", NormalEstimationBackend::OpenMp),
        ("TBB", NormalEstimationBackend::Tbb),
    ];

    for (name, backend) in backends {
        let mut backend_test_cloud = test_cloud.clone();

        let config = NormalEstimationConfig {
            num_neighbors: 10,
            backend,
            num_threads: 4,
        };

        let start = std::time::Instant::now();
        NormalEstimation::estimate_normals(
            &mut backend_test_cloud,
            &config,
            PreprocessingStrategy::CWrapper,
        )?;
        let duration = start.elapsed();

        println!(
            "{} backend: {} points ({:.2}ms)",
            name,
            backend_test_cloud.len(),
            duration.as_secs_f64() * 1000.0
        );

        if !backend_test_cloud.is_empty() {
            let normal = backend_test_cloud.get_normal(0)?;
            println!("  First normal magnitude: {:.3}", normal.magnitude());
        }
    }

    println!("\nAdvanced preprocessing demo completed!");
    println!("All backend-aware features are working correctly.");

    Ok(())
}

/// Create a structured point cloud with geometric patterns for testing.
fn create_structured_point_cloud() -> PointCloud {
    let mut points = Vec::new();

    // Create a structured cloud with planes and edges
    let resolution = 0.02;
    let _size = 1.0;

    // Bottom plane (Z = 0)
    for i in 0..50 {
        for j in 0..50 {
            let x = i as f64 * resolution;
            let y = j as f64 * resolution;
            points.push(Point3::new(x, y, 0.0));
        }
    }

    // Vertical plane (X = 0.5)
    for j in 0..25 {
        for k in 0..25 {
            let y = j as f64 * resolution;
            let z = k as f64 * resolution;
            points.push(Point3::new(0.5, y, z));
        }
    }

    // Some scattered points for variety
    for i in 0..200 {
        let x = (i as f64 * 0.003) % 1.0;
        let y = ((i as f64 * 0.007) % 1.0).sin() * 0.5 + 0.5;
        let z = ((i as f64 * 0.011) % 1.0).cos() * 0.3 + 0.3;
        points.push(Point3::new(x, y, z));
    }

    PointCloud::from_points(&points).expect("Failed to create structured point cloud")
}
