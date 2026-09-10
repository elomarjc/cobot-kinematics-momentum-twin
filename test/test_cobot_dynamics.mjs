import test from 'node:test';
import assert from 'node:assert/strict';
import { DHKinematics } from '../js/engine/dh-kinematics.js';
import { DLSSolver } from '../js/engine/dls-solver.js';
import { RobotDynamics } from '../js/engine/robot-dynamics.js';
import { MomentumObserver } from '../js/engine/momentum-observer.js';

test('1. UR5e Forward Kinematics and Reach Envelope', () => {
    const kin = new DHKinematics();
    
    // Zero angles configuration: arm horizontally extended along X
    const qZero = new Float32Array([0, 0, 0, 0, 0, 0]);
    const resZero = kin.forwardKinematics(qZero);
    
    assert.equal(resZero.positions.length, 7, 'Should compute 7 3D coordinate frames (base + 6 joints)');
    
    const ee = resZero.endEffector;
    const radialDist = Math.hypot(ee[0], ee[1]);
    
    // UR5e nominal maximum horizontal reach is ~0.85 m (850 mm)
    assert.ok(radialDist > 0.80 && radialDist < 0.90, `End effector horizontal reach should be ~0.85m, got: ${radialDist.toFixed(3)}m`);
});

test('2. Yoshikawa Manipulability Index & Singularity Detection', () => {
    const kin = new DHKinematics();

    // Nominal dexterous pose: elbow bent at 90 deg
    const qDexterous = new Float32Array([0, -Math.PI / 4, Math.PI / 2, -Math.PI / 4, Math.PI / 2, 0]);
    const wDexterous = kin.getManipulability(qDexterous);

    // Singularity pose: elbow straight (q3 = 0) and arm fully extended
    const qSingular = new Float32Array([0, -Math.PI / 2, 0.001, 0, 0, 0]);
    const wSingular = kin.getManipulability(qSingular);

    assert.ok(wDexterous > 0.05, `Dexterous pose manipulability should be high (> 0.05), got: ${wDexterous.toFixed(4)}`);
    assert.ok(wSingular < 0.02, `Singular pose manipulability should drop near zero (< 0.02), got: ${wSingular.toFixed(4)}`);
    assert.ok(wDexterous > wSingular * 3, 'Dexterous manipulability should be significantly greater than near-singularity');
});

test('3. DLS Singularity Robust Inverse Kinematics Velocity Bounding', () => {
    const kin = new DHKinematics();
    const solver = new DLSSolver(kin);

    // Position arm near boundary singularity
    const qNearSingular = new Float32Array([0, -Math.PI / 2, 0.005, 0, 0, 0]);
    
    // Command a Cartesian step that points outwards past workspace boundary
    const targetBeyond = [1.2, 0.0, 0.5]; // Unreachable 1.2m reach
    const sol = solver.solve(qNearSingular, targetBeyond, 5.0);

    assert.ok(sol.lambdaSq > 0, `Damping parameter lambdaSq should activate near singularity, got: ${sol.lambdaSq}`);
    
    // Check all joint velocities are strictly bounded by maxJointSpeed (2.5 rad/s)
    for (let i = 0; i < 6; i++) {
        assert.ok(
            Math.abs(sol.qDot[i]) <= solver.maxJointSpeed + 1e-6,
            `Joint ${i} velocity must not explode; got ${sol.qDot[i]} rad/s <= ${solver.maxJointSpeed}`
        );
    }
});

test('4. Sensorless Generalized Momentum Observer Collision Detection (ISO/TS 15066)', () => {
    const dynamics = new RobotDynamics();
    const observer = new MomentumObserver(dynamics);

    const dt = 0.01; // 10 ms control loop
    let q = new Float32Array([0, -0.7, 1.4, -0.7, 1.57, 0]);
    let qDot = new Float32Array([0.1, -0.15, 0.2, 0.05, 0, 0]);

    // Step 1: Run 50 steps under nominal unperturbed motion
    for (let t = 0; t < 50; t++) {
        let tauMotor = dynamics.computeTorques(q, qDot, qDot, dt);
        observer.update(dt, q, qDot, tauMotor);
    }

    assert.equal(observer.emergencyStopEngaged, false, 'Nominal motion should not trigger E-Stop');
    assert.ok(observer.impactMagnitude < 1.0, `Nominal observer residual should remain near zero, got: ${observer.impactMagnitude.toFixed(2)} Nm`);

    // Step 2: Inject human impact collision disturbance (25 Nm on joint 2)
    let collisionDetected = false;
    for (let t = 0; t < 20; t++) {
        let tauMotor = dynamics.computeTorques(q, qDot, qDot, dt);
        tauMotor[1] += 25.0; // Human contact torque
        let res = observer.update(dt, q, qDot, tauMotor);
        if (res.isEStop) {
            collisionDetected = true;
            break;
        }
    }

    assert.ok(collisionDetected, 'Observer must detect collision disturbance within 200 ms');
    assert.equal(observer.emergencyStopEngaged, true, 'Emergency stop must be engaged upon collision');
    assert.ok(observer.impactMagnitude > observer.collisionThresholdNm, 'Impact magnitude must exceed ISO/TS 15066 safety threshold');
});
