import { DHKinematics } from './engine/dh-kinematics.js';
import { DLSSolver } from './engine/dls-solver.js';
import { RobotDynamics } from './engine/robot-dynamics.js';
import { MomentumObserver } from './engine/momentum-observer.js';
import { Cobot3DView } from './ui/cobot-3d-view.js';
import { ObserverChart } from './ui/observer-chart.js';

class CobotApp {
    constructor() {
        this.kin = new DHKinematics();
        this.solver = new DLSSolver(this.kin);
        this.dynamics = new RobotDynamics();
        this.observer = new MomentumObserver(this.dynamics);

        // Current joint angles (rad) - nominal starting pose
        this.q = new Float32Array([0.0, -0.8, 1.6, -0.8, 1.57, 0.0]);
        this.qDot = new Float32Array(6);

        // Desired Cartesian Target [x, y, z]
        let initialFK = this.kin.forwardKinematics(this.q);
        this.targetPos = [...initialFK.endEffector];

        // Collision disturbance injection
        this.injectedDisturbanceNm = 0.0;
        this.disturbanceTimer = 0.0;

        // UI Components
        this.view3d = new Cobot3DView('webgl-container');
        this.chart = new ObserverChart('chart-canvas');

        this.initControls();
        this.setupFloatingHUD();
        this.lastTime = performance.now();
        this.loop = this.loop.bind(this);
        requestAnimationFrame(this.loop);
    }

    initControls() {
        // Target sliders
        const sliderX = document.getElementById('slider-target-x');
        const sliderY = document.getElementById('slider-target-y');
        const sliderZ = document.getElementById('slider-target-z');
        const valX = document.getElementById('val-target-x');
        const valY = document.getElementById('val-target-y');
        const valZ = document.getElementById('val-target-z');

        if (sliderX && sliderY && sliderZ) {
            sliderX.value = this.targetPos[0].toFixed(2);
            sliderY.value = this.targetPos[1].toFixed(2);
            sliderZ.value = this.targetPos[2].toFixed(2);
            if (valX) valX.textContent = sliderX.value + ' m';
            if (valY) valY.textContent = sliderY.value + ' m';
            if (valZ) valZ.textContent = sliderZ.value + ' m';

            const updateTarget = () => {
                this.targetPos[0] = parseFloat(sliderX.value);
                this.targetPos[1] = parseFloat(sliderY.value);
                this.targetPos[2] = parseFloat(sliderZ.value);
                if (valX) valX.textContent = sliderX.value + ' m';
                if (valY) valY.textContent = sliderY.value + ' m';
                if (valZ) valZ.textContent = sliderZ.value + ' m';
                this.view3d.setTargetPosition(this.targetPos[0], this.targetPos[2] + 0.4, this.targetPos[1]);
            };

            sliderX.addEventListener('input', updateTarget);
            sliderY.addEventListener('input', updateTarget);
            sliderZ.addEventListener('input', updateTarget);
        }

        // DLS damping slider
        const sliderDamp = document.getElementById('slider-lambda');
        const valDamp = document.getElementById('val-lambda');
        if (sliderDamp) {
            sliderDamp.addEventListener('input', (e) => {
                this.solver.lambdaMax = parseFloat(e.target.value);
                if (valDamp) valDamp.textContent = this.solver.lambdaMax.toFixed(2);
            });
        }

        // Collision Injection Button
        const btnCollide = document.getElementById('btn-collide');
        if (btnCollide) {
            btnCollide.addEventListener('click', () => {
                this.injectCollision(22.0, 0.25); // 22 Nm impulse for 250 ms
            });
        }

        // Emergency Stop Reset Button
        const btnResetEstop = document.getElementById('btn-reset-estop');
        if (btnResetEstop) {
            btnResetEstop.addEventListener('click', () => {
                this.observer.reset();
                this.injectedDisturbanceNm = 0.0;
                this.updateEStopBadge(false);
            });
        }

        // Presets
        const btnPresetPick = document.getElementById('preset-pick');
        const btnPresetSingular = document.getElementById('preset-singular');
        const btnPresetHome = document.getElementById('preset-home');

        if (btnPresetPick) {
            btnPresetPick.addEventListener('click', () => {
                this.setTargetCoords(0.40, 0.35, 0.15);
            });
        }
        if (btnPresetSingular) {
            btnPresetSingular.addEventListener('click', () => {
                // High reach extension near boundary singularity
                this.setTargetCoords(0.78, 0.0, 0.50);
            });
        }
        if (btnPresetHome) {
            btnPresetHome.addEventListener('click', () => {
                this.setTargetCoords(0.35, 0.35, 0.45);
            });
        }
    }

    setTargetCoords(x, y, z) {
        this.targetPos[0] = x;
        this.targetPos[1] = y;
        this.targetPos[2] = z;
        const sliderX = document.getElementById('slider-target-x');
        const sliderY = document.getElementById('slider-target-y');
        const sliderZ = document.getElementById('slider-target-z');
        if (sliderX) sliderX.value = x.toFixed(2);
        if (sliderY) sliderY.value = y.toFixed(2);
        if (sliderZ) sliderZ.value = z.toFixed(2);
        const valX = document.getElementById('val-target-x');
        const valY = document.getElementById('val-target-y');
        const valZ = document.getElementById('val-target-z');
        if (valX) valX.textContent = x.toFixed(2) + ' m';
        if (valY) valY.textContent = y.toFixed(2) + ' m';
        if (valZ) valZ.textContent = z.toFixed(2) + ' m';
        this.view3d.setTargetPosition(x, z + 0.4, y);
    }

    injectCollision(torqueNm, durationSec) {
        this.injectedDisturbanceNm = torqueNm;
        this.disturbanceTimer = durationSec;
    }

    updateEStopBadge(isEstop) {
        const badge = document.getElementById('badge-estop');
        if (badge) {
            if (isEstop) {
                badge.textContent = 'EMERGENCY STOP (ISO/TS 15066 VIOLATION)';
                badge.className = 'status-badge badge-danger';
            } else {
                badge.textContent = 'NORMAL OPERATION (CAT 0 READY)';
                badge.className = 'status-badge badge-normal';
            }
        }
    }

    loop(currentTime) {
        let dt = Math.min((currentTime - this.lastTime) / 1000.0, 0.05);
        if (dt <= 0) dt = 0.016;
        this.lastTime = currentTime;

        // Collision disturbance timer decay
        if (this.disturbanceTimer > 0) {
            this.disturbanceTimer -= dt;
            if (this.disturbanceTimer <= 0) {
                this.injectedDisturbanceNm = 0.0;
            }
        }

        // Kinematics solver
        let dlsRes = this.solver.solve(this.q, this.targetPos, 4.5);
        let qDotDemand = dlsRes.qDot;

        // If Emergency Stop engaged, freeze joint motion immediately
        if (this.observer.emergencyStopEngaged) {
            qDotDemand = new Float32Array(6);
            for (let i = 0; i < 6; i++) {
                this.qDot[i] *= 0.85; // Decelerate aggressively
            }
        } else {
            // Normal low-pass filtered acceleration
            for (let i = 0; i < 6; i++) {
                this.qDot[i] += (qDotDemand[i] - this.qDot[i]) * Math.min(1.0, 18.0 * dt);
            }
        }

        // Integrate joint angles
        for (let i = 0; i < 6; i++) {
            this.q[i] += this.qDot[i] * dt;
        }

        // Compute simulated dynamics torques
        let tauMotor = this.dynamics.computeTorques(this.q, this.qDot, qDotDemand, dt);

        // Inject disturbance on shoulder/elbow if triggered
        if (this.injectedDisturbanceNm > 0) {
            tauMotor[1] += this.injectedDisturbanceNm;
            tauMotor[2] += this.injectedDisturbanceNm * 0.7;
        }

        // Update Generalized Momentum Observer
        let obsRes = this.observer.update(dt, this.q, this.qDot, tauMotor);

        if (obsRes.isEStop) {
            this.updateEStopBadge(true);
        }

        // Forward Kinematics for Rendering
        let fkRes = this.kin.forwardKinematics(this.q);

        // Update 3D WebGL
        this.view3d.update(fkRes, obsRes.isEStop);

        // Update Real-Time Strip Chart
        this.chart.pushData(obsRes.impactMagnitude, dlsRes.manipulability);
        this.chart.render(this.observer.collisionThresholdNm);

        // Update Telemetry Displays
        this.updateTelemetry(fkRes, dlsRes, obsRes);

        requestAnimationFrame(this.loop);
    }

    updateTelemetry(fkRes, dlsRes, obsRes) {
        const telEE = document.getElementById('tel-ee');
        const telManip = document.getElementById('tel-manip');
        const telTauExt = document.getElementById('tel-tau-ext');
        const telErr = document.getElementById('tel-err');
        const telDamp = document.getElementById('tel-damp');

        if (telEE) {
            let [x, y, z] = fkRes.endEffector;
            telEE.textContent = `[${x.toFixed(3)}, ${y.toFixed(3)}, ${z.toFixed(3)}] m`;
        }
        if (telManip) {
            telManip.textContent = dlsRes.manipulability.toFixed(4);
            telManip.style.color = dlsRes.manipulability < 0.03 ? '#f59e0b' : '#38bdf8';
        }
        if (telTauExt) {
            telTauExt.textContent = `${obsRes.impactMagnitude.toFixed(2)} Nm`;
            telTauExt.style.color = obsRes.impactMagnitude > 12.0 ? '#ef4444' : '#10b981';
        }
        if (telErr) {
            telErr.textContent = `${(dlsRes.errorDist * 1000).toFixed(1)} mm`;
        }
        if (telDamp) {
            telDamp.textContent = Math.sqrt(dlsRes.lambdaSq).toFixed(4);
        }
    }
}

window.addEventListener('DOMContentLoaded', () => {
    window.cobotApp = new CobotApp();

    setupFloatingHUD() {
        // 1. Drawer open/close
        const drawer = document.getElementById('telemetry-drawer');
        const backdrop = document.getElementById('telemetry-backdrop');
        const openDrawer = () => {
            drawer?.classList.add('open');
            backdrop?.classList.add('active');
        };
        const closeDrawer = () => {
            drawer?.classList.remove('open');
            backdrop?.classList.remove('active');
        };

        document.getElementById('btn-hud-settings')?.addEventListener('click', openDrawer);
        document.getElementById('btn-close-telemetry')?.addEventListener('click', closeDrawer);
        backdrop?.addEventListener('click', closeDrawer);

        // 2. Fullscreen Toggle
        const fsBtn = document.getElementById('btn-hud-fullscreen');
        fsBtn?.addEventListener('click', () => {
            if (!document.fullscreenElement && !document.webkitFullscreenElement) {
                if (document.documentElement.requestFullscreen) {
                    document.documentElement.requestFullscreen().catch(() => {
                        document.body.classList.toggle('immersive-fullscreen');
                    });
                } else if (document.documentElement.webkitRequestFullscreen) {
                    document.documentElement.webkitRequestFullscreen();
                } else {
                    document.body.classList.toggle('immersive-fullscreen');
                }
            } else {
                if (document.exitFullscreen) document.exitFullscreen().catch(() => {});
                else if (document.webkitExitFullscreen) document.webkitExitFullscreen();
                document.body.classList.remove('immersive-fullscreen');
            }
        });

        // 3. Pause Simulation Toggle
        let isPaused = false;
        const transPause = document.getElementById('btn-transport-pause');
        const railPause = document.getElementById('btn-rail-pause');
        const hudPauseIcon = document.getElementById('hud-pause-icon');
        const hudPauseLabel = document.getElementById('hud-pause-label');
        const railPauseIcon = document.getElementById('rail-pause-icon');

        const togglePause = () => {
            isPaused = !isPaused;
            this.isPaused = isPaused;
            const icon = isPaused ? '▶' : '⏸';
            const label = isPaused ? 'RESUME' : 'PAUSE';
            if (hudPauseIcon) hudPauseIcon.textContent = icon;
            if (hudPauseLabel) hudPauseLabel.textContent = label;
            if (railPauseIcon) railPauseIcon.textContent = icon;
            transPause?.classList.toggle('is-paused', isPaused);
        };

        transPause?.addEventListener('click', togglePause);
        railPause?.addEventListener('click', togglePause);

        // 4. Mode Cards
        const cardHome = document.getElementById('hud-mode-home');
        const cardPick = document.getElementById('hud-mode-pick');
        const cardSingular = document.getElementById('hud-mode-singular');
        const cardImpact = document.getElementById('hud-mode-impact');

        const setCardActive = (activeCard) => {
            [cardHome, cardPick, cardSingular].forEach(c => c?.classList.remove('active'));
            activeCard?.classList.add('active');
        };

        cardHome?.addEventListener('click', () => {
            this.setTargetCoords(0.35, 0.35, 0.45);
            setCardActive(cardHome);
        });

        cardPick?.addEventListener('click', () => {
            this.setTargetCoords(0.40, 0.35, 0.15);
            setCardActive(cardPick);
        });

        cardSingular?.addEventListener('click', () => {
            this.setTargetCoords(0.78, 0.0, 0.50);
            setCardActive(cardSingular);
        });

        cardImpact?.addEventListener('click', () => {
            this.injectCollision(22.0, 0.25);
            cardImpact.classList.add('active');
            setTimeout(() => cardImpact.classList.remove('active'), 1500);
        });

        // 5. Reach X Rail (0.2 to 0.85 m)
        const reachContainer = document.getElementById('reach-rail-container');
        const reachInput = document.getElementById('slider-reach-vertical');
        const reachFill = document.getElementById('reach-rail-fill');
        const reachThumb = document.getElementById('reach-rail-thumb');
        const reachPill = document.getElementById('val-reach-pill');

        const updateReach = (val) => {
            const num = Math.max(0.2, Math.min(0.85, parseFloat(val)));
            this.setTargetCoords(num, this.targetPos[1], this.targetPos[2]);
            if (reachInput) reachInput.value = num.toFixed(2);
            if (reachPill) reachPill.textContent = `X ${num.toFixed(2)}m`;
            const pct = ((num - 0.2) / 0.65) * 100;
            if (reachFill) reachFill.style.height = `${pct}%`;
            if (reachThumb) reachThumb.style.bottom = `${pct}%`;
        };

        reachInput?.addEventListener('input', (e) => updateReach(e.target.value));

        let dragReach = false;
        const handleReachPointer = (e) => {
            const rect = reachContainer.getBoundingClientRect();
            const frac = Math.max(0, Math.min(1, (rect.bottom - e.clientY) / rect.height));
            updateReach(0.2 + frac * 0.65);
        };
        reachContainer?.addEventListener('pointerdown', (e) => {
            dragReach = true;
            reachContainer.setPointerCapture?.(e.pointerId);
            handleReachPointer(e);
        });
        reachContainer?.addEventListener('pointermove', (e) => {
            if (dragReach) handleReachPointer(e);
        });
        const stopReachDrag = (e) => {
            if (dragReach) {
                dragReach = false;
                try { reachContainer.releasePointerCapture?.(e.pointerId); } catch (_) {}
            }
        };
        reachContainer?.addEventListener('pointerup', stopReachDrag);
        reachContainer?.addEventListener('pointercancel', stopReachDrag);

        // 6. Height Z Rail (0.1 to 0.8 m)
        const heightContainer = document.getElementById('height-rail-container');
        const heightInput = document.getElementById('slider-height-vertical');
        const heightFill = document.getElementById('height-rail-fill');
        const heightThumb = document.getElementById('height-rail-thumb');
        const heightPill = document.getElementById('val-height-pill');

        const updateHeight = (val) => {
            const num = Math.max(0.1, Math.min(0.8, parseFloat(val)));
            this.setTargetCoords(this.targetPos[0], this.targetPos[1], num);
            if (heightInput) heightInput.value = num.toFixed(2);
            if (heightPill) heightPill.textContent = `Z ${num.toFixed(2)}m`;
            const pct = ((num - 0.1) / 0.7) * 100;
            if (heightFill) heightFill.style.height = `${pct}%`;
            if (heightThumb) heightThumb.style.bottom = `${pct}%`;
        };

        heightInput?.addEventListener('input', (e) => updateHeight(e.target.value));

        let dragHeight = false;
        const handleHeightPointer = (e) => {
            const rect = heightContainer.getBoundingClientRect();
            const frac = Math.max(0, Math.min(1, (rect.bottom - e.clientY) / rect.height));
            updateHeight(0.1 + frac * 0.7);
        };
        heightContainer?.addEventListener('pointerdown', (e) => {
            dragHeight = true;
            heightContainer.setPointerCapture?.(e.pointerId);
            handleHeightPointer(e);
        });
        heightContainer?.addEventListener('pointermove', (e) => {
            if (dragHeight) handleHeightPointer(e);
        });
        const stopHeightDrag = (e) => {
            if (dragHeight) {
                dragHeight = false;
                try { heightContainer.releasePointerCapture?.(e.pointerId); } catch (_) {}
            }
        };
        heightContainer?.addEventListener('pointerup', stopHeightDrag);
        heightContainer?.addEventListener('pointercancel', stopHeightDrag);
    }

});
