/**
 * Generalized Momentum Disturbance Observer
 * Sensorless collision detection:
 * p(t) = M(q) * q_dot
 * tau_ext_hat(t) = Ko * [ p(t) - integral( tau - g(q) + tau_ext_hat ) dt - p(0) ]
 * Detects human collision in milliseconds without external sensor skins.
 */
export class MomentumObserver {
    constructor(dynamics) {
        this.dynamics = dynamics;
        this.Ko = 35.0; // Observer diagonal gain matrix (rad/s)
        this.integralTerm = new Float32Array(6);
        this.tauExtHat = new Float32Array(6);
        this.initialized = false;

        // Safety threshold (ISO/TS 15066 quasi-static contact limit ~10-12 Nm)
        this.collisionThresholdNm = 12.0;
        this.isCollisionTriggered = false;
        this.emergencyStopEngaged = false;
        this.impactMagnitude = 0.0;
    }

    reset() {
        this.integralTerm.fill(0);
        this.tauExtHat.fill(0);
        this.initialized = false;
        this.isCollisionTriggered = false;
        this.emergencyStopEngaged = false;
        this.impactMagnitude = 0.0;
    }

    /**
     * Update momentum observer step
     */
    update(dt, q, qDot, tauMotor) {
        let M = this.dynamics.computeInertiaVector(q);
        let g = this.dynamics.computeGravityTorques(q);

        // Auto-initialize integral term to p(0) to eliminate initial impulse
        if (!this.initialized) {
            for (let i = 0; i < 6; i++) {
                this.integralTerm[i] = M[i] * qDot[i];
            }
            this.initialized = true;
        }

        let sumSq = 0.0;

        for (let i = 0; i < 6; i++) {
            // Generalized momentum p_i = M_i * q_dot_i
            let p_i = M[i] * qDot[i];

            // Integrand: tau_motor - g(q) + tau_ext_hat
            let integrand = (tauMotor[i] - g[i] + this.tauExtHat[i]);
            this.integralTerm[i] += integrand * dt;

            // Observer residual estimate: tau_ext_hat = Ko * [ p_i - integral ]
            let residual = this.Ko * (p_i - this.integralTerm[i]);
            this.tauExtHat[i] = residual;

            sumSq += residual * residual;
        }

        this.impactMagnitude = Math.sqrt(sumSq);

        // Safety monitoring (Category 0 Emergency Stop trigger)
        if (this.impactMagnitude > this.collisionThresholdNm) {
            this.isCollisionTriggered = true;
            this.emergencyStopEngaged = true;
        }

        return {
            tauExtHat: this.tauExtHat,
            impactMagnitude: this.impactMagnitude,
            isCollision: this.isCollisionTriggered,
            isEStop: this.emergencyStopEngaged
        };
    }
}
