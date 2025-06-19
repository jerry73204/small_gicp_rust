#[cxx::bridge(namespace = "small_gicp_sys")]
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

    /// Nearest neighbor search result with index and distance
    #[derive(Debug, Clone, Copy)]
    pub struct NearestNeighborResult {
        /// Index of the nearest neighbor
        pub index: usize,
        /// Squared distance to the nearest neighbor
        pub squared_distance: f64,
    }

    /// K-nearest neighbor search result with indices and distances
    #[derive(Debug, Clone)]
    pub struct KnnSearchResult {
        /// Indices of the k nearest neighbors
        pub indices: Vec<usize>,
        /// Squared distances to the k nearest neighbors
        pub squared_distances: Vec<f64>,
    }

    /// Gaussian voxel data structure
    #[derive(Debug, Clone)]
    pub struct GaussianVoxelData {
        /// Number of points in this voxel
        pub num_points: usize,
        /// Mean position of points in this voxel (x, y, z)
        pub mean: [f64; 3],
        /// Covariance matrix of points in this voxel (3x3 row-major)
        pub covariance: [f64; 9],
    }

    /// Voxel information structure
    #[derive(Debug, Clone)]
    pub struct VoxelInfoData {
        /// Voxel index in the map
        pub index: usize,
        /// Voxel coordinates (x, y, z)
        pub coordinates: [i32; 3],
        /// Distance to query point
        pub distance: f64,
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
        /// Number of inlier correspondences
        pub num_inliers: usize,
        /// Information matrix (6x6 as 36-element array, row-major)
        pub information_matrix: [f64; 36],
        /// Information vector (6-element array)
        pub information_vector: [f64; 6],
    }

    /// Settings for KdTree construction
    #[derive(Debug, Clone)]
    pub struct KdTreeSettings {
        /// Number of threads for parallel construction
        pub num_threads: i32,
        /// Maximum leaf size for tree nodes
        pub max_leaf_size: i32,
    }

    /// Registration algorithm type
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[repr(i32)]
    pub enum RegistrationType {
        /// Point-to-point ICP
        ICP = 0,
        /// Point-to-plane ICP
        PointToPlaneICP = 1,
        /// Generalized ICP (distribution-to-distribution)
        GICP = 2,
        /// Voxelized GICP
        VGICP = 3,
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
        include!("small-gicp-sys/include/wrapper.h");

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

        // Bulk operations for performance
        fn set_points_bulk(self: Pin<&mut PointCloud>, points: &[f64]);
        fn set_normals_bulk(self: Pin<&mut PointCloud>, normals: &[f64]);
        fn set_covariances_bulk(self: Pin<&mut PointCloud>, covariances: &[f64]);

        // Transformation operations
        fn transform(self: Pin<&mut PointCloud>, transform: &Transform);
        fn transformed(self: &PointCloud, transform: &Transform) -> UniquePtr<PointCloud>;

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

        // Distance-returning search methods
        fn nearest_neighbor_with_distance(self: &KdTree, point: Point3d) -> NearestNeighborResult;
        fn knn_search_with_distances(self: &KdTree, point: Point3d, k: usize) -> KnnSearchResult;
        fn radius_search_with_distances(
            self: &KdTree,
            point: Point3d,
            radius: f64,
        ) -> KnnSearchResult;

        // Tree information
        fn size(self: &KdTree) -> usize;

        // UnsafeKdTree type and methods (high-performance variant)
        type UnsafeKdTree;

        fn unsafe_nearest_neighbor(self: &UnsafeKdTree, point: Point3d) -> usize;
        fn unsafe_knn_search(self: &UnsafeKdTree, point: Point3d, k: usize) -> Vec<usize>;
        fn unsafe_radius_search(self: &UnsafeKdTree, point: Point3d, radius: f64) -> Vec<usize>;

        // Distance-returning unsafe search methods
        fn unsafe_nearest_neighbor_with_distance(
            self: &UnsafeKdTree,
            point: Point3d,
        ) -> NearestNeighborResult;
        fn unsafe_knn_search_with_distances(
            self: &UnsafeKdTree,
            point: Point3d,
            k: usize,
        ) -> KnnSearchResult;
        fn unsafe_radius_search_with_distances(
            self: &UnsafeKdTree,
            point: Point3d,
            radius: f64,
        ) -> KnnSearchResult;

        // Zero-copy construction methods
        unsafe fn unsafe_validate_data_ptr(self: &UnsafeKdTree, expected_ptr: *const f64) -> bool;

        // GaussianVoxelMap type and methods
        type GaussianVoxelMap;

        fn insert(self: Pin<&mut GaussianVoxelMap>, cloud: &PointCloud);
        fn size(self: &GaussianVoxelMap) -> usize;
        fn get_voxel_size(self: &GaussianVoxelMap) -> f64;
        fn get_num_voxels(self: &GaussianVoxelMap) -> usize;
        fn clear_voxels(self: Pin<&mut GaussianVoxelMap>);
        fn has_voxel_at_coords(self: &GaussianVoxelMap, x: i32, y: i32, z: i32) -> bool;

        // IncrementalVoxelMap type and methods
        type IncrementalVoxelMap;

        fn incremental_insert(self: Pin<&mut IncrementalVoxelMap>, cloud: &PointCloud);
        fn incremental_size(self: &IncrementalVoxelMap) -> usize;
        fn incremental_clear(self: Pin<&mut IncrementalVoxelMap>);
        fn incremental_finalize(self: Pin<&mut IncrementalVoxelMap>);
        fn incremental_get_voxel_size(self: &IncrementalVoxelMap) -> f64;
        fn incremental_get_num_voxels(self: &IncrementalVoxelMap) -> usize;
        fn incremental_insert_point(self: Pin<&mut IncrementalVoxelMap>, x: f64, y: f64, z: f64);
        fn incremental_insert_with_transform(
            self: Pin<&mut IncrementalVoxelMap>,
            cloud: &PointCloud,
            transform: &Transform,
        );
        fn incremental_has_voxel_at_coords(
            self: &IncrementalVoxelMap,
            x: i32,
            y: i32,
            z: i32,
        ) -> bool;
        fn incremental_set_search_offsets(self: Pin<&mut IncrementalVoxelMap>, num_offsets: i32);
        fn incremental_get_voxel_data(
            self: &IncrementalVoxelMap,
            x: i32,
            y: i32,
            z: i32,
        ) -> GaussianVoxelData;
        fn incremental_find_voxels_in_radius(
            self: &IncrementalVoxelMap,
            x: f64,
            y: f64,
            z: f64,
            radius: f64,
        ) -> Vec<VoxelInfoData>;
        fn incremental_nearest_neighbor_search(
            self: &IncrementalVoxelMap,
            x: f64,
            y: f64,
            z: f64,
        ) -> NearestNeighborResult;
        fn incremental_knn_search(
            self: &IncrementalVoxelMap,
            x: f64,
            y: f64,
            z: f64,
            k: usize,
        ) -> KnnSearchResult;
        fn incremental_get_voxel_coords(
            self: &IncrementalVoxelMap,
            x: f64,
            y: f64,
            z: f64,
        ) -> [i32; 3];
        fn incremental_get_voxel_index(self: &IncrementalVoxelMap, x: i32, y: i32, z: i32)
            -> usize;
        fn incremental_get_gaussian_voxel_by_index(
            self: &IncrementalVoxelMap,
            index: usize,
        ) -> GaussianVoxelData;

        // Factory functions
        fn create_point_cloud() -> UniquePtr<PointCloud>;
        fn create_kdtree(cloud: &PointCloud, num_threads: i32) -> UniquePtr<KdTree>;
        fn create_unsafe_kdtree(cloud: &PointCloud, num_threads: i32) -> UniquePtr<UnsafeKdTree>;
        unsafe fn create_unsafe_kdtree_from_points_ptr(
            points_data: *const f64,
            num_points: usize,
            settings: &KdTreeSettings,
        ) -> UniquePtr<UnsafeKdTree>;
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

        // I/O functions
        fn load_ply(filename: &str) -> UniquePtr<PointCloud>;
        fn save_ply(filename: &str, cloud: &PointCloud);

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
pub use ffi::{KdTreeSettings, Point3d, RegistrationResult, RegistrationSettings, Transform};

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

impl Default for KdTreeSettings {
    fn default() -> Self {
        KdTreeSettings {
            num_threads: 1,
            max_leaf_size: 20,
        }
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
