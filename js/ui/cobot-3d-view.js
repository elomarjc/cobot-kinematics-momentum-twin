/**
 * 3D Collaborative Robot (Cobot) WebGL Visualizer
 * Articulated 6-DOF serial manipulator with interactive target sphere,
 * joint orientation kinematics, and emergency stop impact indicator.
 */
export class Cobot3DView {
    constructor(containerId, onTargetMoved) {
        this.container = document.getElementById(containerId);
        this.width = this.container.clientWidth || 640;
        this.height = this.container.clientHeight || 420;
        this.onTargetMoved = onTargetMoved;

        this.initThree();
    }

    initThree() {
        const THREE = window.THREE;
        if (!THREE) return;

        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x060911);

        this.camera = new THREE.PerspectiveCamera(40, this.width / this.height, 0.1, 100);
        const isMobileAspect = (this.width / this.height) < 1.0;
        const camX = isMobileAspect ? 1.6 : 1.2;
        const camY = isMobileAspect ? 1.8 : 1.4;
        const camZ = isMobileAspect ? 2.6 : 1.8;
        this.camera.position.set(camX, camY, camZ);

        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(this.width, this.height);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.renderer.shadowMap.enabled = true;
        this.container.appendChild(this.renderer.domElement);

        // Lighting
        const ambient = new THREE.AmbientLight(0xffffff, 0.6);
        this.scene.add(ambient);

        const dirLight = new THREE.DirectionalLight(0xffffff, 0.9);
        dirLight.position.set(2, 4, 3);
        this.scene.add(dirLight);

        // Grid floor
        const grid = new THREE.GridHelper(3, 30, 0x38bdf8, 0x142035);
        grid.position.y = 0;
        this.scene.add(grid);

        // Pedestal Table
        const tableGeo = new THREE.CylinderGeometry(0.22, 0.25, 0.4, 32);
        const tableMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.5 });
        const table = new THREE.Mesh(tableGeo, tableMat);
        table.position.y = 0.2;
        this.scene.add(table);

        // Cobot Joints Mesh Hierarchy (UR-metallic silver & sky blue caps)
        this.linkMeshes = [];
        const linkMat = new THREE.MeshStandardMaterial({ color: 0xe2e8f0, roughness: 0.25, metalness: 0.6 });
        const capMat = new THREE.MeshStandardMaterial({ color: 0x0ea5e9, roughness: 0.3 });

        // Build 6 articulated cylinders representing robot links
        for (let i = 0; i < 6; i++) {
            let linkGroup = new THREE.Group();
            let cylGeo = new THREE.CylinderGeometry(0.045, 0.045, 0.1, 24);
            let cyl = new THREE.Mesh(cylGeo, i % 2 === 0 ? capMat : linkMat);
            linkGroup.add(cyl);
            this.scene.add(linkGroup);
            this.linkMeshes.push(linkGroup);
        }

        // Line connecting link joints
        this.armLineGeo = new THREE.BufferGeometry();
        const armLineMat = new THREE.LineBasicMaterial({ color: 0x38bdf8, linewidth: 3 });
        this.armLine = new THREE.Line(this.armLineGeo, armLineMat);
        this.scene.add(this.armLine);

        // Target End-Effector Sphere (Draggable target)
        const targetGeo = new THREE.SphereGeometry(0.035, 24, 24);
        this.targetMat = new THREE.MeshStandardMaterial({ color: 0x10b981, emissive: 0x10b981, emissiveIntensity: 0.4 });
        this.targetMesh = new THREE.Mesh(targetGeo, this.targetMat);
        this.targetMesh.position.set(0.45, 0.55, 0.2);
        this.scene.add(this.targetMesh);

        this.setupControls();
        this.camera.lookAt(0, 0.45, 0);
    }

    setupControls() {
        let isDragging = false;
        let prevX = 0, prevY = 0;

        this.renderer.domElement.addEventListener('mousedown', (e) => {
            isDragging = true;
            prevX = e.clientX;
            prevY = e.clientY;
        });

        window.addEventListener('mousemove', (e) => {
            if (!isDragging) return;
            let dx = e.clientX - prevX;
            let dy = e.clientY - prevY;
            prevX = e.clientX;
            prevY = e.clientY;

            let theta = dx * 0.006;
            let x = this.camera.position.x;
            let z = this.camera.position.z;
            this.camera.position.x = x * Math.cos(theta) - z * Math.sin(theta);
            this.camera.position.z = x * Math.sin(theta) + z * Math.cos(theta);
            this.camera.position.y = Math.max(0.2, Math.min(2.5, this.camera.position.y + dy * 0.005));

            this.camera.lookAt(0, 0.45, 0);
        });

        window.addEventListener('mouseup', () => { isDragging = false; });

        // Mobile touch rotation support
        let prevTouchX = 0, prevTouchY = 0;
        this.renderer.domElement.addEventListener('touchstart', (e) => {
            if (e.touches.length === 1) {
                prevTouchX = e.touches[0].clientX;
                prevTouchY = e.touches[0].clientY;
            }
        }, { passive: true });
        this.renderer.domElement.addEventListener('touchmove', (e) => {
            if (e.touches.length === 1) {
                let dx = e.touches[0].clientX - prevTouchX;
                let dy = e.touches[0].clientY - prevTouchY;
                prevTouchX = e.touches[0].clientX;
                prevTouchY = e.touches[0].clientY;
                let theta = dx * 0.008;
                let x = this.camera.position.x;
                let z = this.camera.position.z;
                this.camera.position.x = x * Math.cos(theta) - z * Math.sin(theta);
                this.camera.position.z = x * Math.sin(theta) + z * Math.cos(theta);
                this.camera.position.y = Math.max(0.2, Math.min(2.8, this.camera.position.y + dy * 0.005));
                this.camera.lookAt(0, 0.45, 0);
            }
        }, { passive: true });

        window.addEventListener('resize', () => {
            this.width = this.container.clientWidth || window.innerWidth;
            this.height = this.container.clientHeight || window.innerHeight;
            this.camera.aspect = this.width / this.height;
            this.camera.updateProjectionMatrix();
            this.renderer.setSize(this.width, this.height);
        });
        
    }

    setTargetPosition(x, y, z) {
        if (this.targetMesh) {
            this.targetMesh.position.set(x, y, z);
        }
    }

    update(kinematicsResult, isEStop) {
        if (!this.linkMeshes || !kinematicsResult) return;

        let points = [];
        let baseHeight = 0.4; // table surface

        // Update joint frame positions
        kinematicsResult.positions.forEach((pos, idx) => {
            let px = pos[0];
            let py = pos[2] + baseHeight; // Z in robot frame is Up in Three.js
            let pz = pos[1];
            points.push(new (window.THREE).Vector3(px, py, pz));

            if (idx > 0 && idx <= 6) {
                this.linkMeshes[idx - 1].position.set(px, py, pz);
            }
        });

        // Update connecting skeleton line
        this.armLineGeo.setFromPoints(points);

        // Flash target red on emergency stop
        if (this.targetMat) {
            this.targetMat.color.setHex(isEStop ? 0xef4444 : 0x10b981);
            this.targetMat.emissive.setHex(isEStop ? 0xef4444 : 0x10b981);
        }

        this.renderer.render(this.scene, this.camera);
    }
}
