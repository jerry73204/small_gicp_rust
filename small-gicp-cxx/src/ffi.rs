#[cxx::bridge(namespace = "small_gicp_cxx")]
pub mod ffi {
    // Shared structs between Rust and C++
    /// A 3D point with x, y, z coordinates
    #[derive(Debug, Clone, Copy)]
    pub struct Point3d {
        /// X coordinate
        pub x: f64,
        /// Y coordinate
        pub y: f64,
        /// Z coordinate
        pub z: f64,
    }

    /// A 4x4 transformation matrix stored as row-major array
    #[derive(Debug, Clone, Copy)]
    pub struct Transform {
        /// 4x4 transformation matrix as 16-element array (row-major)
        pub matrix: [f64; 16],
    }

    /// Result of point cloud registration
    #[derive(Debug, Clone)]
    pub struct RegistrationResult {
        /// Final transformation matrix
        pub transformation: Transform,
        /// Whether the algorithm converged
        pub converged: bool,
        /// Number of iterations performed
        pub iterations: i32,
        /// Final registration error
        pub error: f64,
    }

    /// Settings for point cloud registration algorithms
    #[derive(Debug, Clone)]
    pub struct RegistrationSettings {
        /// Maximum number of iterations
        pub max_iterations: i32,
        /// Rotation convergence threshold
        pub rotation_epsilon: f64,
        /// Translation convergence threshold
        pub transformation_epsilon: f64,
        /// Maximum distance for point correspondences
        pub max_correspondence_distance: f64,
        /// Number of threads for parallel processing
        pub num_threads: i32,
    }

    // Opaque C++ types
    unsafe extern "C++" {
        include!("small-gicp-cxx/include/wrapper.h");

        // PointCloud type and methods
        type PointCloud;

        fn size(self: &PointCloud) -> usize;
        fn reserve(self: Pin<&mut PointCloud>, n: usize);
        fn resize(self: Pin<&mut PointCloud>, n: usize);
        fn clear(self: Pin<&mut PointCloud>);

        fn add_point(self: Pin<&mut PointCloud>, point: Point3d);
        fn set_point(self: Pin<&mut PointCloud>, index: usize, point: Point3d);
        fn get_point(self: &PointCloud, index: usize) -> Point3d;

        fn points_data(self: &PointCloud) -> &[f64];
        fn normals_data(self: &PointCloud) -> &[f64];
        fn covs_data(self: &PointCloud) -> &[f64];

        fn estimate_normals(self: Pin<&mut PointCloud>, num_neighbors: i32, num_threads: i32);
        fn estimate_covariances(self: Pin<&mut PointCloud>, num_neighbors: i32, num_threads: i32);

        fn voxel_downsample(
            self: &PointCloud,
            voxel_size: f64,
            num_threads: i32,
        ) -> UniquePtr<PointCloud>;

        // KdTree type and methods
        type KdTree;

        fn nearest_neighbor(self: &KdTree, point: Point3d) -> usize;
        fn knn_search(self: &KdTree, point: Point3d, k: usize) -> Vec<usize>;
        fn radius_search(self: &KdTree, point: Point3d, radius: f64) -> Vec<usize>;

        // UnsafeKdTree type and methods (high-performance variant)
        type UnsafeKdTree;

        fn unsafe_nearest_neighbor(self: &UnsafeKdTree, point: Point3d) -> usize;
        fn unsafe_knn_search(self: &UnsafeKdTree, point: Point3d, k: usize) -> Vec<usize>;
        fn unsafe_radius_search(self: &UnsafeKdTree, point: Point3d, radius: f64) -> Vec<usize>;

        // GaussianVoxelMap type and methods
        type GaussianVoxelMap;

        fn insert(self: Pin<&mut GaussianVoxelMap>, cloud: &PointCloud);
        fn size(self: &GaussianVoxelMap) -> usize;

        // IncrementalVoxelMap type and methods
        type IncrementalVoxelMap;

        fn incremental_insert(self: Pin<&mut IncrementalVoxelMap>, cloud: &PointCloud);
        fn incremental_size(self: &IncrementalVoxelMap) -> usize;
        fn incremental_clear(self: Pin<&mut IncrementalVoxelMap>);
        fn incremental_finalize(self: Pin<&mut IncrementalVoxelMap>);

        // Factory functions
        fn create_point_cloud() -> UniquePtr<PointCloud>;
        fn create_kdtree(cloud: &PointCloud, num_threads: i32) -> UniquePtr<KdTree>;
        fn create_unsafe_kdtree(cloud: &PointCloud, num_threads: i32) -> UniquePtr<UnsafeKdTree>;
        fn create_voxelmap(voxel_size: f64) -> UniquePtr<GaussianVoxelMap>;
        fn create_incremental_voxelmap(voxel_size: f64) -> UniquePtr<IncrementalVoxelMap>;

        // Preprocessing functions
        fn downsample_voxelgrid(
            cloud: &PointCloud,
            voxel_size: f64,
            num_threads: i32,
        ) -> UniquePtr<PointCloud>;
        fn downsample_random(cloud: &PointCloud, num_samples: usize) -> UniquePtr<PointCloud>;
        fn compute_normals(cloud: Pin<&mut PointCloud>, num_neighbors: i32, num_threads: i32);
        fn compute_covariances(cloud: Pin<&mut PointCloud>, num_neighbors: i32, num_threads: i32);

        // Registration functions
        fn align_points_icp(
            source: &PointCloud,
            target: &PointCloud,
            target_tree: &KdTree,
            init_guess: &Transform,
            settings: &RegistrationSettings,
        ) -> RegistrationResult;

        fn align_points_point_to_plane_icp(
            source: &PointCloud,
            target: &PointCloud,
            target_tree: &KdTree,
            init_guess: &Transform,
            settings: &RegistrationSettings,
        ) -> RegistrationResult;

        fn align_points_gicp(
            source: &PointCloud,
            target: &PointCloud,
            source_tree: &KdTree,
            target_tree: &KdTree,
            init_guess: &Transform,
            settings: &RegistrationSettings,
        ) -> RegistrationResult;

        fn align_points_vgicp(
            source: &PointCloud,
            target_voxelmap: &GaussianVoxelMap,
            init_guess: &Transform,
            settings: &RegistrationSettings,
        ) -> RegistrationResult;
    }
}

// Re-export commonly used types
pub use ffi::{Point3d, RegistrationResult, RegistrationSettings, Transform};

// Helper implementations
impl Default for Transform {
    fn default() -> Self {
        let mut matrix = [0.0; 16];
        // Initialize as identity matrix
        matrix[0] = 1.0;
        matrix[5] = 1.0;
        matrix[10] = 1.0;
        matrix[15] = 1.0;
        Transform { matrix }
    }
}

impl Default for RegistrationSettings {
    fn default() -> Self {
        RegistrationSettings {
            max_iterations: 20,
            rotation_epsilon: 1e-5,
            transformation_epsilon: 1e-5,
            max_correspondence_distance: 1.0,
            num_threads: 1,
        }
    }
}
