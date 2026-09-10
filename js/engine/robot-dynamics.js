/**
 * 6-DOF Robot Dynamic Equations of Motion
 * Computes generalized mass matrix M(q), Coriolis vector C(q, q_dot)*q_dot,
 * and gravity torques g(q).
 */
export class RobotDynamics {
    constructor() {
        // Link masses (kg) for UR5e
        this.masses = [3.7, 8.39, 2.27, 1.22, 1.22, 0.19];
        // Link lengths (m)
        this.lengths = [0.1625, 0.425, 0.3922, 0.1333, 0.0997, 0.0996];
        this.g = 9.81; // m/s^2

        // External collision disturbance torque (Nm)
        this.tauExt = new Float32Array(6);
    }

    /**
     * Compute gravity torque vector g(q) in Nm
     */
    computeGravityTorques(q) {
        let gTorques = new Float32Array(6);
        let q2 = q[1];
        let q3 = q[2];
        let q4 = q[3];

        // Base joint 1 axis is vertical (gravity has 0 torque on joint 1)
        gTorques[0] = 0.0;

        // Shoulder joint 2 holds upper arm, forearm and wrist
        let mTotal2 = this.masses[1] + this.masses[2] + this.masses[3] + this.masses[4] + this.masses[5];
        let rC2 = 0.21; // center of mass
        gTorques[1] = mTotal2 * this.g * rC2 * Math.cos(q2);

        // Elbow joint 3 holds forearm and wrist
        let mTotal3 = this.masses[2] + this.masses[3] + this.masses[4] + this.masses[5];
        let rC3 = 0.19;
        gTorques[2] = mTotal3 * this.g * rC3 * Math.cos(q2 + q3);

        // Wrist joints
        gTorques[3] = (this.masses[3] + this.masses[4]) * this.g * 0.08 * Math.cos(q2 + q3 + q4);
        gTorques[4] = 0.0;
        gTorques[5] = 0.0;

        return gTorques;
    }

    /**
     * Approximate effective joint inertia M_ii(q)
     */
    computeInertiaVector(q) {
        let M = new Float32Array(6);
        M[0] = 1.8; // Base rotation inertia
        // Shoulder inertia depends on arm reach (q3)
        M[1] = 2.4 + 1.2 * Math.cos(q[2]);
        M[2] = 0.85;
        M[3] = 0.15;
        M[4] = 0.12;
        M[5] = 0.06;
        return M;
    }

    /**
     * Compute motor command torques for trajectory following
     */
    computeTorques(q, qDot, qDotDemand, dt) {
        let g = this.computeGravityTorques(q);
        let M = this.computeInertiaVector(q);
        let tau = new Float32Array(6);

        // PD velocity controller + gravity feedforward
        const kpVel = 25.0;
        for (let i = 0; i < 6; i++) {
            let velErr = qDotDemand[i] - qDot[i];
            let inertialAcc = (velErr / dt) * M[i];
            tau[i] = g[i] + kpVel * velErr + inertialAcc * 0.1;
        }

        return tau;
    }
}
