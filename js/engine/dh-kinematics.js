/**
 * 6-DOF Collaborative Robot Kinematics (UR5e Geometry)
 * Computes Forward Kinematics, Analytical Geometric Jacobian J(q),
 * and Yoshikawa Manipulability Index w(q) = sqrt(det(J*J^T)).
 */
export class DHKinematics {
    constructor() {
        // Standard UR5e Denavit-Hartenberg parameters (meters, radians)
        // [d, a, alpha]
        this.dhParams = [
            { d: 0.1625, a: 0.0,     alpha: Math.PI / 2 },  // Joint 1 (Base)
            { d: 0.0,    a: -0.425,  alpha: 0.0 },          // Joint 2 (Shoulder)
            { d: 0.0,    a: -0.3922, alpha: 0.0 },          // Joint 3 (Elbow)
            { d: 0.1333, a: 0.0,     alpha: Math.PI / 2 },  // Joint 4 (Wrist 1)
            { d: 0.0997, a: 0.0,     alpha: -Math.PI / 2 }, // Joint 5 (Wrist 2)
            { d: 0.0996, a: 0.0,     alpha: 0.0 }           // Joint 6 (Wrist 3)
        ];
    }

    /**
     * Compute 4x4 homogeneous transformation matrix A_i for joint i
     */
    getTransformMatrix(q_i, d_i, a_i, alpha_i) {
        let cq = Math.cos(q_i);
        let sq = Math.sin(q_i);
        let ca = Math.cos(alpha_i);
        let sa = Math.sin(alpha_i);

        return [
            [cq, -sq * ca,  sq * sa, a_i * cq],
            [sq,  cq * ca, -cq * sa, a_i * sq],
            [0,   sa,       ca,      d_i],
            [0,   0,        0,       1]
        ];
    }

    /**
     * Matrix multiplication 4x4
     */
    multiplyMatrices(A, B) {
        let C = [
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0]
        ];
        for (let i = 0; i < 4; i++) {
            for (let j = 0; j < 4; j++) {
                for (let k = 0; k < 4; k++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    /**
     * Compute forward kinematics for joint angles q = [q1, q2, q3, q4, q5, q6]
     * @returns {Array<Array<number>>} Positions of all 7 frames (base + 6 joints) in 3D
     */
    forwardKinematics(q) {
        let T = [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ];

        let positions = [[0, 0, 0]];
        let transforms = [T];

        for (let i = 0; i < 6; i++) {
            let p = this.dhParams[i];
            let A = this.getTransformMatrix(q[i], p.d, p.a, p.alpha);
            T = this.multiplyMatrices(T, A);
            transforms.push(T);
            positions.push([T[0][3], T[1][3], T[2][3]]);
        }

        return { positions, transforms, endEffector: positions[6] };
    }

    /**
     * Compute 3x6 position Jacobian J_v(q)
     */
    computePositionJacobian(q) {
        let { positions, transforms } = this.forwardKinematics(q);
        let pE = positions[6];

        let J = [
            new Float32Array(6),
            new Float32Array(6),
            new Float32Array(6)
        ];

        for (let i = 0; i < 6; i++) {
            let T = transforms[i];
            // z-axis vector of joint i
            let z = [T[0][2], T[1][2], T[2][2]];
            let p_i = positions[i];
            // vector from joint i to end effector: r = pE - p_i
            let rx = pE[0] - p_i[0];
            let ry = pE[1] - p_i[1];
            let rz = pE[2] - p_i[2];

            // cross product J_vi = z x r
            J[0][i] = z[1] * rz - z[2] * ry;
            J[1][i] = z[2] * rx - z[0] * rz;
            J[2][i] = z[0] * ry - z[1] * rx;
        }

        return J;
    }

    /**
     * Compute manipulability measure w = sqrt(det(J * J^T))
     */
    getManipulability(q) {
        let J = this.computePositionJacobian(q);
        // Compute 3x3 matrix JJT = J * J^T
        let JJT = [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]
        ];

        for (let r = 0; r < 3; r++) {
            for (let c = 0; c < 3; c++) {
                let sum = 0;
                for (let k = 0; k < 6; k++) sum += J[r][k] * J[c][k];
                JJT[r][c] = sum;
            }
        }

        // Determinant of 3x3 matrix
        let det = JJT[0][0] * (JJT[1][1] * JJT[2][2] - JJT[1][2] * JJT[2][1]) -
                  JJT[0][1] * (JJT[1][0] * JJT[2][2] - JJT[1][2] * JJT[2][0]) +
                  JJT[0][2] * (JJT[1][0] * JJT[2][1] - JJT[1][1] * JJT[2][0]);

        return Math.sqrt(Math.max(0, det));
    }
}
