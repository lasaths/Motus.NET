let scene, camera, renderer, robot;
let robotLinks = [];

window.initThreeJs = function() {
    const container = document.getElementById('viewport');
    const width = container.clientWidth;
    const height = container.clientHeight;

    // Scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0a0a);

    // Camera
    camera = new THREE.PerspectiveCamera(50, width / height, 0.1, 1000);
    camera.position.set(1.5, 1.5, 1.5);
    camera.lookAt(0, 0, 0.5);

    // Renderer
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(renderer.domElement);

    // Lights
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
    scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(5, 10, 7);
    scene.add(directionalLight);

    // Grid
    const gridHelper = new THREE.GridHelper(2, 20, 0x333333, 0x1a1a1a);
    scene.add(gridHelper);

    // Ground (collision object)
    const groundGeometry = new THREE.BoxGeometry(0.6, 0.6, 0.05);
    const groundMaterial = new THREE.MeshStandardMaterial({ 
        color: 0x4a4a4a, 
        transparent: true, 
        opacity: 0.6 
    });
    const ground = new THREE.Mesh(groundGeometry, groundMaterial);
    ground.position.set(0.3, 0, -0.025);
    scene.add(ground);

    // Initialize robot visualization (5 links + gripper)
    createRobotLinks();

    // Controls
    const controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.target.set(0, 0, 0.5);
    controls.update();

    // Handle window resize
    window.addEventListener('resize', () => {
        const width = container.clientWidth;
        const height = container.clientHeight;
        camera.aspect = width / height;
        camera.updateProjectionMatrix();
        renderer.setSize(width, height);
    });

    // Animation loop
    function animate() {
        requestAnimationFrame(animate);
        controls.update();
        renderer.render(scene, camera);
    }
    animate();
};

function createRobotLinks() {
    const materials = [
        new THREE.MeshStandardMaterial({ color: 0x4a7fff }),
        new THREE.MeshStandardMaterial({ color: 0x4affaa }),
        new THREE.MeshStandardMaterial({ color: 0xffdd4a }),
        new THREE.MeshStandardMaterial({ color: 0xff884a }),
        new THREE.MeshStandardMaterial({ color: 0xff4a4a })
    ];

    const linkLengths = [0.3, 0.25, 0.25, 0.2, 0.05];
    const linkRadii = [0.03, 0.025, 0.025, 0.02, 0.015];

    for (let i = 0; i < 5; i++) {
        const geometry = new THREE.CylinderGeometry(linkRadii[i], linkRadii[i], linkLengths[i], 16);
        const link = new THREE.Mesh(geometry, materials[i]);
        robotLinks.push(link);
        scene.add(link);
    }

    // Gripper
    const gripperGeometry = new THREE.BoxGeometry(0.06, 0.04, 0.02);
    const gripperMaterial = new THREE.MeshStandardMaterial({ color: 0x999999 });
    const gripper = new THREE.Mesh(gripperGeometry, gripperMaterial);
    robotLinks.push(gripper);
    scene.add(gripper);
}

window.updateRobot = function(linkPosesJson) {
    const linkPoses = JSON.parse(linkPosesJson);
    
    for (let i = 0; i < Math.min(linkPoses.length, robotLinks.length); i++) {
        const pose = linkPoses[i];
        const link = robotLinks[i];
        
        link.position.set(pose.position.x, pose.position.y, pose.position.z);
        link.quaternion.set(pose.quaternion.x, pose.quaternion.y, pose.quaternion.z, pose.quaternion.w);
    }
};

window.fetchFile = async function(path) {
    const response = await fetch(path);
    return await response.text();
};
