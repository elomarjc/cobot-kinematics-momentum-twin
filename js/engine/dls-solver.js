/**
 * Damped Least Squares (DLS) Singularity-Robust Inverse Kinematics Solver
 * Solves: q_dot = J^T * (J * J^T + lambda^2 * I)^-1 * e
 * Guarantees bounded joint velocities near kinematic singularities.
 */
export class DLSSolver {
    constructor(kinematics) {
        this.kin = kinematics;
        this.lambdaMax = 0.08; // Maximum damping near singularity
        this.wThreshold = 0.04; // Singularity boundary threshold
        this.maxJointSpeed = 2.5; // rad/s
    }

    /**
     * Compute joint velocity demand dot_q to move toward target position [xt, yt, zt]
     */
    solve(q, targetPos, kp = 4.0) {
        let { endEffector } = this.kin.forwardKinematics(q);
        let ex = targetPos[0] - endEffector[0];
        let ey = targetPos[1] - endEffector[1];
        let ez = targetPos[2] - endEffector[2];

        let errorDist = Math.hypot(ex, ey, ez);

        // Desired Cartesian velocity v_d = kp * error
        let vx = kp * ex;
        let vy = kp * ey;
        let vz = kp * ez;

        // Position Jacobian J (3x6)
        let J = this.kin.computePositionJacobian(q);

        // Manipulability w
        let w = this.kin.getManipulability(q);

        // Dynamic damping factor lambda^2
        let lambdaSq = 0.0;
        if (w < this.wThreshold) {
            let ratio = w / this.wThreshold;
            lambdaSq = (this.lambdaMax ** 2) * (1.0 - ratio * ratio);
        }

        // Compute 3x3 matrix: A = J * J^T + lambda^2 * I
        let A = [
            [lambdaSq, 0, 0],
            [0, lambdaSq, 0],
            [0, 0, lambdaSq]
        ];

        for (let r = 0; r < 3; r++) {
            for (let c = 0; c < 3; c++) {
                let sum = 0;
                for (let k = 0; k < 6; k++) sum += J[r][k] * J[c][k];
                A[r][c] += sum;
            }
        }

        // Invert 3x3 matrix A
        let invA = this.invert3x3(A);
        if (!invA) return new Float32Array(6);

        // y = invA * [vx, vy, vz]^T
        let y0 = invA[0][0] * vx + invA[0][1] * vy + invA[0][2] * vz;
        let y1 = invA[1][0] * vx + invA[1][1] * vy + invA[1][2] * vz;
        let y2 = invA[2][0] * vx + invA[2][1] * vy + invA[2][2] * vz;

        // dot_q = J^T * y
        let qDot = new Float32Array(6);
        for (let i = 0; i < 6; i++) {
            let vel = J[0][i] * y0 + J[1][i] * y1 + J[2][i] * y2;
            qDot[i] = Math.max(-this.maxJointSpeed, Math.min(this.maxJointSpeed, vel));
        }

        return { qDot, errorDist, manipulability: w, lambdaSq };
    }

    invert3x3(M) {
        let d = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
                M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
                M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);

        if (Math.abs(d) < 1e-9) return null;
        let invD = 1.0 / d;

        return [
            [
                (M[1][1] * M[2][2] - M[1][2] * M[2][1]) * invD,
                (M[0][2] * M[2][1] - M[0][1] * M[2][2]) * invD,
                (M[0][1] * M[1][2] - M[0][2] * M[1][1]) * invD
            ],
            [
                (M[1][2] * M[2][0] - M[1][0] * M[2][2]) * invD,
                (M[0][0] * M[2][2] - M[0][2] * M[2][0]) * invD,
                (M[0][2] * M[1][0] - M[0][0] * M[1][2]) * invD
            ],
            [
                (M[1][0] * M[2][1] - M[1][1] * M[2][0]) * invD,
                (M[0][1] * M[2][0] - M[0][0] * M[2][1]) * invD,
                (M[0][0] * M[1][1] - M[0][1] * M[1][0]) * invD
            ]
        ];
    }
}
