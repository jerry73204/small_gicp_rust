//! Advanced Configuration Demo
//!
//! This example demonstrates the advanced configuration parameters available
//! in the small_gicp Rust API, including extended optimizer settings,
//! KdTree projections, parallel processing configuration, and comprehensive
//! registration settings.

use nalgebra::Point3;
use small_gicp_rust::{
    config::*, error::Result, kdtree::KdTree, point_cloud::PointCloud, preprocessing,
    registration::*,
};

fn main() -> Result<()> {
    println!("=== Advanced Configuration Demo ===\n");

    // Create some sample point clouds
    let target_points = create_sample_points(0.0);
    let source_points = create_sample_points(0.1); // Slightly offset

    let mut target = PointCloud::from_points(&target_points)?;
    let mut source = PointCloud::from_points(&source_points)?;

    println!("1. Advanced KdTree Configuration");

    // Create an extended KdTree configuration with normal projection
    let kdtree_config = ExtendedKdTreeConfig {
        builder_type: KdTreeBuilderType::OpenMp,
        num_threads: 4,
        max_leaf_size: 10,
        projection: ExtendedProjectionConfig {
            projection_type: ProjectionType::Normal,
            max_scan_count: 64,
        },
    };

    // For this demo, convert to basic config (since the demo data might not have normals)
    let basic_kdtree_config = KdTreeConfig {
        builder_type: kdtree_config.builder_type,
        num_threads: kdtree_config.num_threads,
        max_leaf_size: kdtree_config.max_leaf_size,
        projection: ProjectionConfig {
            projection_type: ProjectionType::AxisAligned, // Use axis-aligned for demo
            max_scan_count: kdtree_config.projection.max_scan_count,
        },
    };

    println!("  - Builder type: {:?}", basic_kdtree_config.builder_type);
    println!(
        "  - Projection type: {:?}",
        basic_kdtree_config.projection.projection_type
    );
    println!("  - Max leaf size: {}", basic_kdtree_config.max_leaf_size);
    println!(
        "  - Max scan count: {}\n",
        basic_kdtree_config.projection.max_scan_count
    );

    println!("2. Advanced Preprocessing with Backend Selection");

    // Use TBB backend for voxel grid sampling
    let voxel_config = VoxelGridConfig {
        leaf_size: 0.05,
        backend: DownsamplingBackend::Tbb,
        num_threads: 4,
    };

    target = target.voxelgrid_sampling(&voxel_config)?;
    source = source.voxelgrid_sampling(&voxel_config)?;

    println!("  - Downsampling backend: {:?}", voxel_config.backend);
    println!("  - Leaf size: {}", voxel_config.leaf_size);
    println!("  - Target cloud size after downsampling: {}", target.len());
    println!(
        "  - Source cloud size after downsampling: {}\n",
        source.len()
    );

    // Use OpenMP backend for normal estimation
    let normal_config = NormalEstimationConfig {
        num_neighbors: 20,
        backend: NormalEstimationBackend::OpenMp,
        num_threads: 4,
    };

    // Create temporary KdTrees for normal estimation
    let temp_target_tree = KdTree::new(&target, &KdTreeConfig::default())?;
    let temp_source_tree = KdTree::new(&source, &KdTreeConfig::default())?;

    preprocessing::estimate_normals(&mut target, &temp_target_tree, &normal_config)?;
    preprocessing::estimate_normals(&mut source, &temp_source_tree, &normal_config)?;

    println!("  - Normal estimation backend: {:?}", normal_config.backend);
    println!("  - Number of neighbors: {}", normal_config.num_neighbors);
    println!("  - Normals estimated for both clouds\n");

    println!("3. KdTree with Advanced Configuration");
    let target_tree = KdTree::new(&target, &basic_kdtree_config)?;

    // Demonstrate KNN search with custom settings
    let knn_config = KnnConfig {
        epsilon: 0.01, // Allow 1% approximation for faster search
    };

    let query_point = Point3::new(0.0, 0.0, 0.0);
    let neighbors = target_tree.knn_search_with_settings(query_point, 5, &knn_config)?;
    println!("  - KNN search with epsilon: {}", knn_config.epsilon);
    println!("  - Found {} neighbors for query point", neighbors.len());
    println!(
        "  - First neighbor distance: {:.4}\n",
        neighbors.get(0).map(|(_, d)| d.sqrt()).unwrap_or(0.0)
    );

    println!("4. Extended Optimizer Configuration");

    let optimizer_config = ExtendedOptimizerConfig {
        optimizer_type: OptimizerType::LevenbergMarquardt,
        verbose: false,
        max_iterations: 50,
        lambda: 1e-4,
        max_inner_iterations: 15,
        lambda_factor: 2.0,
    };

    println!("  - Optimizer type: {:?}", optimizer_config.optimizer_type);
    println!("  - Max iterations: {}", optimizer_config.max_iterations);
    println!("  - Lambda: {}", optimizer_config.lambda);
    println!(
        "  - Max inner iterations: {}",
        optimizer_config.max_inner_iterations
    );
    println!("  - Lambda factor: {}\n", optimizer_config.lambda_factor);

    println!("5. Complete Registration Configuration");

    let complete_config = CompleteRegistrationConfig {
        registration: AdvancedRegistrationConfig {
            registration_type: RegistrationType::Gicp,
            voxel_resolution: 0.1,
            downsampling_resolution: 0.05,
            max_correspondence_distance: 0.5,
            rotation_eps: 0.1 * std::f64::consts::PI / 180.0,
            translation_eps: 1e-3,
            num_threads: 4,
            max_iterations: 50,
            verbose: false,
        },
        termination: TerminationConfig {
            translation_eps: 1e-3,
            rotation_eps: 0.1 * std::f64::consts::PI / 180.0,
        },
        optimizer: optimizer_config,
        correspondence_rejector: ExtendedCorrespondenceRejectorConfig {
            rejector_type: CorrespondenceRejectorType::Distance,
            max_dist_sq: 0.25,
        },
        robust_kernel: RobustKernelConfig {
            kernel_type: RobustKernelType::Huber,
            scale_parameter: 0.5,
        },
        dof_restriction: None,
        parallel_processing: ParallelProcessingConfig {
            reduction_type: ReductionType::OpenMp,
            num_threads: 4,
        },
    };

    println!(
        "  - Registration type: {:?}",
        complete_config.registration.registration_type
    );
    println!(
        "  - Robust kernel: {:?}",
        complete_config.robust_kernel.kernel_type
    );
    println!(
        "  - Correspondence rejector: {:?}",
        complete_config.correspondence_rejector.rejector_type
    );
    println!(
        "  - Parallel reduction: {:?}",
        complete_config.parallel_processing.reduction_type
    );
    println!(
        "  - Max correspondence distance: {}",
        complete_config.registration.max_correspondence_distance
    );

    // Set global parallel reduction strategy
    set_parallel_reduction_strategy(&complete_config.parallel_processing)?;
    println!("  - Global parallel reduction strategy set\n");

    println!("6. Performing Registration with Complete Configuration");

    // Note: register_with_complete_config is available but may use fallback implementation
    // For this demo, we'll use the standard registration with some of the advanced settings
    let settings = RegistrationSettings {
        registration_type: complete_config.registration.registration_type,
        num_threads: complete_config.registration.num_threads,
        initial_guess: None,
    };

    match register_preprocessed(&target, &source, &target_tree, &settings) {
        Ok(result) => {
            println!("  - Registration completed successfully");
            println!("  - Converged: {}", result.converged);
            println!("  - Iterations: {}", result.iterations);
            println!("  - Final error: {:.6}", result.error);
            println!(
                "  - Translation: [{:.4}, {:.4}, {:.4}]",
                result.translation().x,
                result.translation().y,
                result.translation().z
            );
        }
        Err(e) => {
            println!(
                "  - Registration failed (expected with synthetic data): {:?}",
                e
            );
            println!("  - This demonstrates error handling in the advanced configuration");
        }
    }

    println!("\n=== Advanced Configuration Demo Complete ===");
    println!("This demo showed:");
    println!("✓ Extended KdTree configuration with projection types");
    println!("✓ Backend-aware preprocessing (TBB/OpenMP)");
    println!("✓ KNN search with approximation settings");
    println!("✓ Extended optimizer configuration");
    println!("✓ Complete registration configuration");
    println!("✓ Parallel processing configuration");

    Ok(())
}

fn create_sample_points(offset: f64) -> Vec<Point3<f64>> {
    let mut points = Vec::new();

    // Create a simple 3D grid of points with more significant transformation
    for i in 0..10 {
        for j in 0..10 {
            for k in 0..5 {
                points.push(Point3::new(
                    i as f64 * 0.1 + offset * 10.0, // More significant offset
                    j as f64 * 0.1 + offset * 5.0,
                    k as f64 * 0.1 + offset * 2.0,
                ));
            }
        }
    }

    points
}
