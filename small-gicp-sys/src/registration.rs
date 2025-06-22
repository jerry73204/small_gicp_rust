use crate::{
    ffi::{
        ffi::{
            align_points_gicp, align_points_icp, align_points_point_to_plane_icp,
            align_points_vgicp,
        },
        RegistrationResult, RegistrationSettings, Transform,
    },
    GaussianVoxelMap, KdTree, PointCloud,
};

/// Registration algorithms for point cloud alignment
pub struct Registration;

impl Registration {
    /// Align two point clouds using standard ICP (Iterative Closest Point)
    pub fn icp(
        source: &PointCloud,
        target: &PointCloud,
        target_tree: &KdTree,
        init_guess: Option<Transform>,
        settings: Option<RegistrationSettings>,
    ) -> RegistrationResult {
        let init = init_guess.unwrap_or_default();
        let settings = settings.unwrap_or_default();

        align_points_icp(
            source.as_ffi(),
            target.as_ffi(),
            target_tree.as_ffi(),
            &init,
            &settings,
        )
    }

    /// Align two point clouds using Point-to-Plane ICP
    /// Requires normals to be computed for the target cloud
    pub fn point_to_plane_icp(
        source: &PointCloud,
        target: &PointCloud,
        target_tree: &KdTree,
        init_guess: Option<Transform>,
        settings: Option<RegistrationSettings>,
    ) -> RegistrationResult {
        let init = init_guess.unwrap_or_default();
        let settings = settings.unwrap_or_default();

        align_points_point_to_plane_icp(
            source.as_ffi(),
            target.as_ffi(),
            target_tree.as_ffi(),
            &init,
            &settings,
        )
    }

    /// Align two point clouds using GICP (Generalized ICP)
    /// Requires covariances to be computed for both clouds
    pub fn gicp(
        source: &PointCloud,
        target: &PointCloud,
        source_tree: &KdTree,
        target_tree: &KdTree,
        init_guess: Option<Transform>,
        settings: Option<RegistrationSettings>,
    ) -> RegistrationResult {
        let init = init_guess.unwrap_or_default();
        let settings = settings.unwrap_or_default();

        align_points_gicp(
            source.as_ffi(),
            target.as_ffi(),
            source_tree.as_ffi(),
            target_tree.as_ffi(),
            &init,
            &settings,
        )
    }

    /// Align a point cloud to a voxel map using VGICP (Voxelized GICP)
    pub fn vgicp(
        source: &PointCloud,
        target_voxelmap: &GaussianVoxelMap,
        init_guess: Option<Transform>,
        settings: Option<RegistrationSettings>,
    ) -> RegistrationResult {
        let init = init_guess.unwrap_or_default();
        let settings = settings.unwrap_or_default();

        align_points_vgicp(source.as_ffi(), target_voxelmap.as_ffi(), &init, &settings)
    }
}

/// Builder for registration settings
pub struct RegistrationSettingsBuilder {
    settings: RegistrationSettings,
}

impl RegistrationSettingsBuilder {
    /// Create a new registration settings builder with default values
    pub fn new() -> Self {
        Self {
            settings: RegistrationSettings::default(),
        }
    }

    /// Set maximum number of iterations
    pub fn max_iterations(mut self, iterations: i32) -> Self {
        self.settings.max_iterations = iterations;
        self
    }

    /// Set rotation epsilon for convergence
    pub fn rotation_epsilon(mut self, epsilon: f64) -> Self {
        self.settings.rotation_epsilon = epsilon;
        self
    }

    /// Set transformation epsilon for convergence
    pub fn transformation_epsilon(mut self, epsilon: f64) -> Self {
        self.settings.transformation_epsilon = epsilon;
        self
    }

    /// Set maximum correspondence distance
    pub fn max_correspondence_distance(mut self, distance: f64) -> Self {
        self.settings.max_correspondence_distance = distance;
        self
    }

    /// Set number of threads for parallel processing
    pub fn num_threads(mut self, threads: i32) -> Self {
        self.settings.num_threads = threads;
        self
    }

    /// Build the settings
    pub fn build(self) -> RegistrationSettings {
        self.settings
    }
}

impl Default for RegistrationSettingsBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// Helper trait for transform operations
pub trait TransformExt {
    /// Create identity transform
    fn identity() -> Self;

    /// Create transform from 4x4 matrix
    fn from_matrix(matrix: &[[f64; 4]; 4]) -> Self;

    /// Get as 4x4 matrix
    fn to_matrix(&self) -> [[f64; 4]; 4];

    /// Apply transform to a point
    fn transform_point(&self, x: f64, y: f64, z: f64) -> (f64, f64, f64);
}

impl TransformExt for Transform {
    fn identity() -> Self {
        Transform::default()
    }

    fn from_matrix(matrix: &[[f64; 4]; 4]) -> Self {
        let mut flat = [0.0; 16];
        for i in 0..4 {
            for j in 0..4 {
                flat[i * 4 + j] = matrix[i][j];
            }
        }
        Transform { matrix: flat }
    }

    fn to_matrix(&self) -> [[f64; 4]; 4] {
        let mut matrix = [[0.0; 4]; 4];
        for (i, row) in matrix.iter_mut().enumerate() {
            for (j, elem) in row.iter_mut().enumerate() {
                *elem = self.matrix[i * 4 + j];
            }
        }
        matrix
    }

    fn transform_point(&self, x: f64, y: f64, z: f64) -> (f64, f64, f64) {
        let m = &self.matrix;
        let w = m[12] * x + m[13] * y + m[14] * z + m[15];
        (
            (m[0] * x + m[1] * y + m[2] * z + m[3]) / w,
            (m[4] * x + m[5] * y + m[6] * z + m[7]) / w,
            (m[8] * x + m[9] * y + m[10] * z + m[11]) / w,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_transform() {
        let t = Transform::identity();
        let (x, y, z) = t.transform_point(1.0, 2.0, 3.0);
        assert_eq!((x, y, z), (1.0, 2.0, 3.0));
    }

    #[test]
    fn test_settings_builder() {
        let settings = RegistrationSettingsBuilder::new()
            .max_iterations(50)
            .num_threads(4)
            .max_correspondence_distance(0.5)
            .build();

        assert_eq!(settings.max_iterations, 50);
        assert_eq!(settings.num_threads, 4);
        assert_eq!(settings.max_correspondence_distance, 0.5);
    }
}
