use small_gicp_sys::{
    KdTree, PointCloud, Registration, RegistrationSettingsBuilder, Transform, TransformExt,
};

fn main() {
    println!("Testing small-gicp-sys basic functionality...");

    // Create a simple point cloud
    let mut cloud = PointCloud::new();
    cloud.add_point(0.0, 0.0, 0.0);
    cloud.add_point(1.0, 0.0, 0.0);
    cloud.add_point(0.0, 1.0, 0.0);
    cloud.add_point(0.0, 0.0, 1.0);

    println!("Created point cloud with {} points", cloud.len());

    // Test point access
    if let Some(point) = cloud.get_point(0) {
        println!("First point: ({}, {}, {})", point.0, point.1, point.2);
    }

    // Create KdTree
    let tree = KdTree::build(&cloud, 1);
    println!("Built KdTree");

    // Test nearest neighbor search
    if let Some(nn) = tree.nearest_neighbor(0.1, 0.1, 0.1) {
        println!("Nearest neighbor to (0.1, 0.1, 0.1): index {}", nn);
    }

    // Test knn search
    let knn = tree.knn_search(0.0, 0.0, 0.0, 3);
    println!("3 nearest neighbors to origin: {:?}", knn);

    // Create another cloud for registration
    let target_cloud = PointCloud::from_points(&[
        (0.1, 0.1, 0.1),
        (1.1, 0.1, 0.1),
        (0.1, 1.1, 0.1),
        (0.1, 0.1, 1.1),
    ]);

    let target_tree = KdTree::build(&target_cloud, 1);

    // Test registration
    let settings = RegistrationSettingsBuilder::new()
        .max_iterations(10)
        .num_threads(1)
        .build();

    let result = Registration::icp(&cloud, &target_cloud, &target_tree, None, Some(settings));

    println!("Registration result:");
    println!("  Converged: {}", result.converged);
    println!("  Iterations: {}", result.iterations);
    println!("  Error: {}", result.error);

    // Test transform
    let identity = Transform::identity();
    let (x, y, z) = identity.transform_point(1.0, 2.0, 3.0);
    println!("Identity transform of (1,2,3): ({}, {}, {})", x, y, z);

    println!("All tests completed successfully!");
}
