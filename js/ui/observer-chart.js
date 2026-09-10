/**
 * Momentum Disturbance Observer & Singularity Strip-Chart
 * Plots estimated external collision torque tau_ext_hat against ISO/TS 15066 limit
 * and Yoshikawa Manipulability Index w(q).
 */
export class ObserverChart {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');

        this.length = 120;
        this.tauResidual = new Float32Array(this.length);
        this.manipulability = new Float32Array(this.length);
    }

    pushData(residualNm, manipW) {
        for (let i = 0; i < this.length - 1; i++) {
            this.tauResidual[i] = this.tauResidual[i + 1];
            this.manipulability[i] = this.manipulability[i + 1];
        }
        this.tauResidual[this.length - 1] = residualNm;
        this.manipulability[this.length - 1] = manipW;
    }

    render(thresholdNm = 12.0) {
        const width = this.canvas.width;
        const height = this.canvas.height;
        const ctx = this.ctx;

        ctx.clearRect(0, 0, width, height);

        // Background
        ctx.fillStyle = '#0a0e17';
        ctx.fillRect(0, 0, width, height);

        const padLeft = 45;
        const padRight = 15;
        const padTop = 28;
        const padBottom = 25;
        const plotW = width - padLeft - padRight;
        const plotH = height - padTop - padBottom;

        const halfH = (plotH - 25) / 2;

        // Top Plot: Residual Torque (0 to 25 Nm)
        const topY = (tau) => padTop + halfH - (tau / 25.0) * halfH;
        ctx.strokeStyle = '#141d2e';
        ctx.lineWidth = 1;
        ctx.font = '10px Inter, monospace';
        ctx.fillStyle = '#4a5b78';
        ctx.textAlign = 'right';

        [0, 12, 24].forEach(val => {
            let py = topY(val);
            ctx.beginPath();
            ctx.moveTo(padLeft, py);
            ctx.lineTo(padLeft + plotW, py);
            ctx.stroke();
            ctx.fillText(`${val}Nm`, padLeft - 6, py + 3);
        });

        // ISO/TS 15066 Threshold Line (Dashed Red)
        let threshY = topY(thresholdNm);
        ctx.beginPath();
        ctx.setLineDash([4, 4]);
        ctx.strokeStyle = '#ef4444';
        ctx.lineWidth = 1.5;
        ctx.moveTo(padLeft, threshY);
        ctx.lineTo(padLeft + plotW, threshY);
        ctx.stroke();
        ctx.setLineDash([]);

        // Bottom Plot: Manipulability w (0 to 0.12)
        const botTop = padTop + halfH + 15;
        const botY = (w) => (botTop + halfH) - (w / 0.12) * halfH;

        [0, 0.05, 0.10].forEach(val => {
            let py = botY(val);
            ctx.beginPath();
            ctx.moveTo(padLeft, py);
            ctx.lineTo(padLeft + plotW, py);
            ctx.stroke();
            ctx.fillText(`${val.toFixed(2)}`, padLeft - 6, py + 3);
        });

        const mapX = (idx) => padLeft + (idx / (this.length - 1)) * plotW;

        // Draw Residual Trace (Purple or Red)
        ctx.beginPath();
        let maxRes = Math.max(...this.tauResidual);
        ctx.strokeStyle = maxRes > thresholdNm ? '#ef4444' : '#a855f7';
        ctx.lineWidth = 2.0;
        for (let i = 0; i < this.length; i++) {
            let px = mapX(i);
            let py = topY(Math.max(0, this.tauResidual[i]));
            if (i === 0) ctx.moveTo(px, py);
            else ctx.lineTo(px, py);
        }
        ctx.stroke();

        // Draw Manipulability Trace (Cyan)
        ctx.beginPath();
        ctx.strokeStyle = '#38bdf8';
        ctx.lineWidth = 1.8;
        for (let i = 0; i < this.length; i++) {
            let px = mapX(i);
            let py = botY(Math.max(0, this.manipulability[i]));
            if (i === 0) ctx.moveTo(px, py);
            else ctx.lineTo(px, py);
        }
        ctx.stroke();

        // Labels
        ctx.textAlign = 'left';
        ctx.font = 'bold 11px Inter, sans-serif';
        ctx.fillStyle = '#f8fafc';
        ctx.fillText('Generalized Momentum Observer Residual ||τ_ext|| (Nm) vs. ISO/TS 15066 Limit', padLeft + 6, padTop - 8);
        ctx.fillText('Yoshikawa Manipulability Index w(q) (Singularity Proximity)', padLeft + 6, botTop - 6);
    }
}
