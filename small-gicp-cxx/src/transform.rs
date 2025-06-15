use crate::ffi::Transform;

impl Transform {
    /// Create an identity transformation
    pub fn identity() -> Self {
        Transform {
            matrix: [
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            ],
        }
    }

    /// Create a translation-only transformation
    pub fn translation(x: f64, y: f64, z: f64) -> Self {
        Transform {
            matrix: [
                1.0, 0.0, 0.0, x, 0.0, 1.0, 0.0, y, 0.0, 0.0, 1.0, z, 0.0, 0.0, 0.0, 1.0,
            ],
        }
    }

    /// Create a rotation around Z axis (yaw) transformation
    pub fn rotation_z(angle_rad: f64) -> Self {
        let cos_a = angle_rad.cos();
        let sin_a = angle_rad.sin();
        Transform {
            matrix: [
                cos_a, -sin_a, 0.0, 0.0, sin_a, cos_a, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                1.0,
            ],
        }
    }

    /// Create a rotation around Y axis (pitch) transformation
    pub fn rotation_y(angle_rad: f64) -> Self {
        let cos_a = angle_rad.cos();
        let sin_a = angle_rad.sin();
        Transform {
            matrix: [
                cos_a, 0.0, sin_a, 0.0, 0.0, 1.0, 0.0, 0.0, -sin_a, 0.0, cos_a, 0.0, 0.0, 0.0, 0.0,
                1.0,
            ],
        }
    }

    /// Create a rotation around X axis (roll) transformation
    pub fn rotation_x(angle_rad: f64) -> Self {
        let cos_a = angle_rad.cos();
        let sin_a = angle_rad.sin();
        Transform {
            matrix: [
                1.0, 0.0, 0.0, 0.0, 0.0, cos_a, -sin_a, 0.0, 0.0, sin_a, cos_a, 0.0, 0.0, 0.0, 0.0,
                1.0,
            ],
        }
    }

    /// Create a transformation from translation and rotation around Z axis
    pub fn from_translation_rotation_z(x: f64, y: f64, z: f64, yaw_rad: f64) -> Self {
        let cos_yaw = yaw_rad.cos();
        let sin_yaw = yaw_rad.sin();
        Transform {
            matrix: [
                cos_yaw, -sin_yaw, 0.0, x, sin_yaw, cos_yaw, 0.0, y, 0.0, 0.0, 1.0, z, 0.0, 0.0,
                0.0, 1.0,
            ],
        }
    }

    /// Create a transformation from a 4x4 matrix (row-major)
    pub fn from_matrix(matrix: [f64; 16]) -> Self {
        Transform { matrix }
    }

    /// Get the translation component as (x, y, z)
    pub fn get_translation(&self) -> (f64, f64, f64) {
        (self.matrix[3], self.matrix[7], self.matrix[11])
    }

    /// Get the rotation component as a 3x3 matrix (row-major)
    pub fn rotation_matrix_3x3(&self) -> [f64; 9] {
        [
            self.matrix[0],
            self.matrix[1],
            self.matrix[2],
            self.matrix[4],
            self.matrix[5],
            self.matrix[6],
            self.matrix[8],
            self.matrix[9],
            self.matrix[10],
        ]
    }

    /// Compose two transformations (self * other)
    pub fn compose(&self, other: &Transform) -> Transform {
        let mut result = [0.0; 16];

        // Matrix multiplication: result = self * other
        for i in 0..4 {
            for j in 0..4 {
                for k in 0..4 {
                    result[i * 4 + j] += self.matrix[i * 4 + k] * other.matrix[k * 4 + j];
                }
            }
        }

        Transform { matrix: result }
    }

    /// Get the inverse transformation
    pub fn inverse(&self) -> Transform {
        // For rigid body transformations, inverse is [R^T, -R^T*t; 0, 1]
        let r00 = self.matrix[0];
        let r01 = self.matrix[1];
        let r02 = self.matrix[2];
        let tx = self.matrix[3];
        let r10 = self.matrix[4];
        let r11 = self.matrix[5];
        let r12 = self.matrix[6];
        let ty = self.matrix[7];
        let r20 = self.matrix[8];
        let r21 = self.matrix[9];
        let r22 = self.matrix[10];
        let tz = self.matrix[11];

        // R^T
        let rt00 = r00;
        let rt01 = r10;
        let rt02 = r20;
        let rt10 = r01;
        let rt11 = r11;
        let rt12 = r21;
        let rt20 = r02;
        let rt21 = r12;
        let rt22 = r22;

        // -R^T * t
        let inv_tx = -(rt00 * tx + rt01 * ty + rt02 * tz);
        let inv_ty = -(rt10 * tx + rt11 * ty + rt12 * tz);
        let inv_tz = -(rt20 * tx + rt21 * ty + rt22 * tz);

        Transform {
            matrix: [
                rt00, rt01, rt02, inv_tx, rt10, rt11, rt12, inv_ty, rt20, rt21, rt22, inv_tz, 0.0,
                0.0, 0.0, 1.0,
            ],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_identity_transform() {
        let t = Transform::identity();
        let expected = [
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ];
        assert_eq!(t.matrix, expected);
    }

    #[test]
    fn test_translation() {
        let t = Transform::translation(1.0, 2.0, 3.0);
        assert_eq!(t.get_translation(), (1.0, 2.0, 3.0));
    }

    #[test]
    fn test_compose_identity() {
        let t1 = Transform::translation(1.0, 2.0, 3.0);
        let identity = Transform::identity();
        let result = t1.compose(&identity);

        // Should be approximately equal to t1
        for i in 0..16 {
            assert!((result.matrix[i] - t1.matrix[i]).abs() < 1e-10);
        }
    }

    #[test]
    fn test_inverse() {
        let t = Transform::translation(1.0, 2.0, 3.0);
        let inv = t.inverse();
        let identity = t.compose(&inv);

        let expected_identity = Transform::identity();
        for i in 0..16 {
            assert!((identity.matrix[i] - expected_identity.matrix[i]).abs() < 1e-10);
        }
    }
}
