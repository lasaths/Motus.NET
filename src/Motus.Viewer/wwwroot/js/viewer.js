/**
 * Motus.Viewer Three.js draw layer for the ICD/LIS bamboo arm.
 * Visual/UX ported from the retired standalone HTML preview (bamboo-viewer/next.html).
 * Joint angles, grips, strut hold, task frames, and collision tint names come from C#;
 * this file does not decide kinematics or collision.
 *
 * Frame: native Three.js Y-up (matches HTML). Motus.NET URDF is Z-up; shared θ vector.
 */
(function () {
  'use strict';

  var DEG = Math.PI / 180;
  var SEG = [0.030, 0.080, 0.088, 0.088, 0.088, 0.046];
  var POLE_Y = 0.40;
  var PICK = { x: 0, y: 0.00778, z: 0.09194 };
  var PLACE = { x: 0, y: 0.1506, z: 0.2000 };

  var scene, camera, renderer, view;
  var look, theta, phi, dist;
  var j1, j2, j3, j4, j5, jaw, mount;
  var lNeg, lPos, rNeg, rPos, strut;
  var axisGroups = [];
  var frameRoot;
  var colliderByName = {};
  var dragging = false, lx = 0, ly = 0;
  var ready = false;

  function metal(hex, rough, metalness) {
    return new THREE.MeshStandardMaterial({ color: hex, roughness: rough, metalness: metalness });
  }

  var mats = {
    tube: metal(0x8b939a, 0.62, 0.28),
    elbow: metal(0x646c74, 0.55, 0.38),
    shoulder: metal(0x8e969e, 0.5, 0.42),
    wrist: metal(0x525a62, 0.58, 0.32),
    carriage: metal(0x4a525a, 0.6, 0.3),
    jaw: metal(0x3c444c, 0.5, 0.4),
    rail: metal(0xb7c0c8, 0.35, 0.55),
    pole: new THREE.MeshStandardMaterial({ color: 0xc4a15a, roughness: 0.86, metalness: 0.02 }),
    strut: new THREE.MeshStandardMaterial({ color: 0xd2b06a, roughness: 0.84, metalness: 0.02 }),
    node: new THREE.MeshStandardMaterial({ color: 0x9a7840, roughness: 0.9, metalness: 0.02 })
  };

  function makeLabel(text, css) {
    var c = document.createElement('canvas');
    c.width = 256; c.height = 128;
    var g = c.getContext('2d');
    g.clearRect(0, 0, 256, 128);
    g.font = '600 64px sans-serif';
    g.fillStyle = css;
    g.textAlign = 'center';
    g.textBaseline = 'middle';
    g.fillText(text, 128, 68);
    var tex = new THREE.CanvasTexture(c);
    var spr = new THREE.Sprite(new THREE.SpriteMaterial({
      map: tex, transparent: true, depthTest: false, depthWrite: false
    }));
    spr.scale.set(0.072, 0.036, 1);
    spr.renderOrder = 2;
    return spr;
  }

  function axisX(color, css, text) {
    var g = new THREE.Group();
    var mat = new THREE.MeshBasicMaterial({ color: color });
    var shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.0022, 0.0022, 0.062, 10), mat);
    shaft.rotation.z = -Math.PI / 2;
    g.add(shaft);
    var cone = new THREE.Mesh(new THREE.ConeGeometry(0.0055, 0.014, 10), mat);
    cone.rotation.z = -Math.PI / 2;
    cone.position.x = 0.036;
    g.add(cone);
    var lab = makeLabel(text, css);
    lab.position.set(0.018, 0.026, 0);
    g.add(lab);
    return g;
  }

  function axisY(color, css, text) {
    var g = new THREE.Group();
    var mat = new THREE.MeshBasicMaterial({ color: color });
    var shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.0022, 0.0022, 0.062, 10), mat);
    g.add(shaft);
    var cone = new THREE.Mesh(new THREE.ConeGeometry(0.0055, 0.014, 10), mat);
    cone.position.y = 0.036;
    g.add(cone);
    var lab = makeLabel(text, css);
    lab.position.set(0.032, 0.016, 0);
    g.add(lab);
    return g;
  }

  function track(obj) { axisGroups.push(obj); return obj; }

  function registerCollider(name, meshes) {
    colliderByName[name] = meshes;
  }

  function addTube(parent, span, colName) {
    var L = Math.max(0.008, span - 0.016);
    var mesh = new THREE.Mesh(new THREE.BoxGeometry(0.012, L, 0.016), mats.tube);
    mesh.position.y = -span / 2;
    parent.add(mesh);
    if (colName) {
      if (!colliderByName[colName]) colliderByName[colName] = [];
      colliderByName[colName].push(mesh);
    }
    return mesh;
  }

  function housing(parent, size, mat, colName) {
    var mesh = new THREE.Mesh(new THREE.BoxGeometry(size[0], size[1], size[2]), mat);
    parent.add(mesh);
    if (colName) {
      if (!colliderByName[colName]) colliderByName[colName] = [];
      colliderByName[colName].push(mesh);
    }
    return mesh;
  }

  function jawPlate(w, h, d, mat) {
    return new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat);
  }

  function setOpening(neg, pos, mm, yLift) {
    var half = (mm / 1000) / 2;
    var shift = half + 0.002;
    neg.position.x = -shift;
    pos.position.x = shift;
    if (typeof yLift === 'number') {
      neg.position.y = yLift;
      pos.position.y = yLift;
    }
  }

  function applyJoints(deg) {
    j1.rotation.y = deg[0] * DEG;
    j2.rotation.x = deg[1] * DEG;
    j3.rotation.x = deg[2] * DEG;
    j4.rotation.x = deg[3] * DEG;
    j5.rotation.y = deg[4] * DEG;
  }

  function setStrut(hold, grasp, release) {
    var g = grasp || PICK;
    var r = release || PLACE;
    if (hold === 1) {
      if (strut.parent !== jaw) jaw.attach(strut);
      strut.position.set(0, 0, 0);
      strut.rotation.set(0, 0, 0);
    } else if (hold === 2) {
      if (strut.parent !== scene) scene.attach(strut);
      strut.position.set(r.x, r.y, r.z);
      strut.rotation.set((r.a || -90) * DEG, 0, 0);
    } else {
      if (strut.parent !== scene) scene.attach(strut);
      strut.position.set(g.x, g.y, g.z);
      strut.rotation.set((g.a || 0) * DEG, 0, 0);
    }
  }

  function updateCamera() {
    camera.position.set(
      look.x + dist * Math.sin(phi) * Math.sin(theta),
      look.y + dist * Math.cos(phi),
      look.z + dist * Math.sin(phi) * Math.cos(theta)
    );
    camera.lookAt(look);
  }

  function resize() {
    if (!view || !renderer) return;
    var w = view.clientWidth, h = view.clientHeight;
    if (w < 1 || h < 1) return;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
  }

  function makePlaneLabel(text) {
    var c = document.createElement('canvas');
    c.width = 512; c.height = 128;
    var g = c.getContext('2d');
    g.clearRect(0, 0, 512, 128);
    g.font = '600 54px sans-serif';
    g.fillStyle = '#f2efe6';
    g.textAlign = 'center';
    g.textBaseline = 'middle';
    var tw = g.measureText(text).width;
    var sc = Math.min(1, 480 / Math.max(tw, 1));
    g.save();
    g.translate(256, 64);
    g.scale(sc, 1);
    g.fillText(text, 0, 0);
    g.restore();
    var tex = new THREE.CanvasTexture(c);
    var spr = new THREE.Sprite(new THREE.SpriteMaterial({
      map: tex, transparent: true, depthTest: false, depthWrite: false
    }));
    spr.scale.set(0.11, 0.028, 1);
    spr.renderOrder = 3;
    return spr;
  }

  var HIT_COLOR = 0xff3b30;
  function clearTints() {
    Object.keys(colliderByName).forEach(function (name) {
      var meshes = colliderByName[name];
      for (var m = 0; m < meshes.length; m++) {
        if (meshes[m].userData.baseMat) meshes[m].material = meshes[m].userData.baseMat;
      }
    });
  }

  function showHits(names) {
    clearTints();
    if (!names || !names.length) return;
    var seen = {};
    for (var i = 0; i < names.length; i++) {
      var name = names[i];
      if (seen[name] || !colliderByName[name]) continue;
      seen[name] = true;
      var meshes = colliderByName[name];
      for (var m = 0; m < meshes.length; m++) {
        var mesh = meshes[m];
        if (!mesh.userData.baseMat) mesh.userData.baseMat = mesh.material;
        var mat = mesh.userData.baseMat.clone();
        mat.color.setHex(HIT_COLOR);
        if (mat.emissive) mat.emissive.setHex(0x5a1008);
        mesh.material = mat;
      }
    }
  }

  window.initThreeJs = function () {
    if (typeof THREE === 'undefined') {
      throw new Error('Three.js is not loaded (check lib/three/three.min.js).');
    }
    if (ready) return;

    view = document.getElementById('viewport');
    if (!view) throw new Error('#viewport element not found.');

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setClearColor(0x000000, 1);
    view.appendChild(renderer.domElement);

    scene = new THREE.Scene();
    camera = new THREE.PerspectiveCamera(34, 1, 0.01, 30);
    look = new THREE.Vector3(0, 0.20, 0.09);
    theta = 0.62; phi = 1.12; dist = 1.12;
    updateCamera();

    scene.add(new THREE.AmbientLight(0xffffff, 0.38));
    var key = new THREE.DirectionalLight(0xffffff, 1.05);
    key.position.set(0.6, 1.2, 0.7);
    scene.add(key);
    var fill = new THREE.DirectionalLight(0xc8d0e0, 0.38);
    fill.position.set(-0.7, 0.4, -0.4);
    scene.add(fill);

    var ground = new THREE.Mesh(
      new THREE.PlaneGeometry(2.2, 2.2),
      new THREE.MeshStandardMaterial({ color: 0x070707, roughness: 1, metalness: 0 })
    );
    ground.rotation.x = -Math.PI / 2;
    scene.add(ground);
    registerCollider('ground', [ground]);

    var grid = new THREE.GridHelper(1.4, 14, 0x3a3a3a, 0x242424);
    grid.position.y = 0.0006;
    scene.add(grid);

    var ring = new THREE.Mesh(
      new THREE.RingGeometry(0.020, 0.025, 64),
      new THREE.MeshBasicMaterial({ color: 0xd7c392, side: THREE.DoubleSide })
    );
    ring.rotation.x = -Math.PI / 2;
    ring.position.set(PLACE.x, 0.0016, PLACE.z);
    scene.add(ring);

    var pole = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.66, 24), mats.pole);
    pole.rotation.z = Math.PI / 2;
    pole.position.set(0, POLE_Y, 0);
    scene.add(pole);
    registerCollider('pole', [pole]);
    [-0.22, 0.16, 0.26].forEach(function (x) {
      var n = new THREE.Mesh(new THREE.TorusGeometry(0.0126, 0.0013, 6, 18), mats.node);
      n.rotation.y = Math.PI / 2;
      n.position.set(x, POLE_Y, 0);
      scene.add(n);
    });

    mount = new THREE.Group();
    mount.position.set(0, POLE_Y, 0);
    mount.rotation.y = Math.PI / 2;
    scene.add(mount);

    var lbody = new THREE.Mesh(new THREE.BoxGeometry(0.020, 0.016, 0.026), mats.carriage);
    lbody.position.y = 0.020;
    mount.add(lbody);
    var lrail = new THREE.Mesh(new THREE.BoxGeometry(0.080, 0.004, 0.004), mats.rail);
    lrail.position.y = 0.014;
    mount.add(lrail);
    lNeg = jawPlate(0.0045, 0.030, 0.024, mats.jaw);
    lPos = jawPlate(0.0045, 0.030, 0.024, mats.jaw);
    mount.add(lNeg); mount.add(lPos);
    registerCollider('L-grip', [lbody, lrail, lNeg, lPos]);
    mount.add(track(axisX(0x3ddc6a, '#3ddc6a', 'L-grip')));

    addTube(mount, SEG[0], 'base link');
    j1 = new THREE.Group();
    j1.position.y = -SEG[0];
    mount.add(j1);
    housing(j1, [0.046, 0.029, 0.034], mats.wrist, 'θ1');
    j1.add(track(axisY(0x33e0e0, '#33e0e0', 'θ1')));

    addTube(j1, SEG[1], 'link θ2');
    j2 = new THREE.Group();
    j2.position.y = -SEG[1];
    j1.add(j2);
    housing(j2, [0.041, 0.061, 0.040], mats.elbow, 'θ2');
    j2.add(track(axisX(0xff8800, '#ff8800', 'θ2')));

    addTube(j2, SEG[2], 'link θ3');
    j3 = new THREE.Group();
    j3.position.y = -SEG[2];
    j2.add(j3);
    housing(j3, [0.041, 0.061, 0.040], mats.shoulder, 'θ3');
    j3.add(track(axisX(0xffffff, '#ffffff', 'θ3')));

    addTube(j3, SEG[3], 'link θ4');
    j4 = new THREE.Group();
    j4.position.y = -SEG[3];
    j3.add(j4);
    housing(j4, [0.041, 0.061, 0.040], mats.elbow, 'θ4');
    j4.add(track(axisX(0xff8800, '#ff8800', 'θ4')));

    addTube(j4, SEG[4], 'link θ5');
    j5 = new THREE.Group();
    j5.position.y = -SEG[4];
    j4.add(j5);
    housing(j5, [0.046, 0.029, 0.034], mats.wrist, 'θ5');
    j5.add(track(axisY(0x33e0e0, '#33e0e0', 'θ5')));

    var neck = new THREE.Mesh(new THREE.BoxGeometry(0.012, 0.028, 0.016), mats.tube);
    neck.position.y = -0.024;
    j5.add(neck);
    registerCollider('neck', [neck]);

    jaw = new THREE.Group();
    jaw.position.y = -SEG[5];
    j5.add(jaw);
    var carriage = new THREE.Mesh(new THREE.BoxGeometry(0.018, 0.018, 0.022), mats.carriage);
    carriage.position.y = 0.022;
    jaw.add(carriage);
    var rrail = new THREE.Mesh(new THREE.BoxGeometry(0.080, 0.004, 0.004), mats.rail);
    rrail.position.set(0, 0.012, 0);
    jaw.add(rrail);
    rNeg = jawPlate(0.0045, 0.020, 0.022, mats.jaw);
    rPos = jawPlate(0.0045, 0.020, 0.022, mats.jaw);
    rNeg.position.y = 0.005;
    rPos.position.y = 0.005;
    jaw.add(rNeg); jaw.add(rPos);
    registerCollider('R-grip', [carriage, rrail, rNeg, rPos]);
    jaw.add(track(axisX(0x3ddc6a, '#3ddc6a', 'R-grip')));

    var strutGeo = new THREE.CylinderGeometry(0.007, 0.007, 0.30, 20);
    strutGeo.rotateX(Math.PI / 2);
    strut = new THREE.Mesh(strutGeo, mats.strut);
    [-0.09, 0, 0.09].forEach(function (z) {
      var n = new THREE.Mesh(new THREE.TorusGeometry(0.0075, 0.0012, 6, 16), mats.node);
      n.position.z = z;
      strut.add(n);
    });
    scene.add(strut);
    registerCollider('strut', [strut]);

    frameRoot = new THREE.Group();
    scene.add(frameRoot);

    setOpening(lNeg, lPos, 24, 0);
    setOpening(rNeg, rPos, 40, 0.005);
    applyJoints([-90, -72, -80, -24, 0]);
    setStrut(0, null, null);

    view.addEventListener('mousedown', function (e) {
      dragging = true; lx = e.clientX; ly = e.clientY;
      view.classList.add('drag');
    });
    window.addEventListener('mouseup', function () {
      dragging = false; view.classList.remove('drag');
    });
    window.addEventListener('mousemove', function (e) {
      if (!dragging) return;
      var dx = e.clientX - lx, dy = e.clientY - ly;
      lx = e.clientX; ly = e.clientY;
      theta -= dx * 0.005;
      phi = Math.max(0.25, Math.min(1.45, phi - dy * 0.005));
      updateCamera();
    });
    view.addEventListener('wheel', function (e) {
      e.preventDefault();
      dist = Math.max(0.35, Math.min(2.4, dist * (1 + Math.sign(e.deltaY) * 0.06)));
      updateCamera();
    }, { passive: false });

    window.addEventListener('resize', resize);
    resize();

    function loop() {
      requestAnimationFrame(loop);
      renderer.render(scene, camera);
    }
    requestAnimationFrame(loop);
    ready = true;
  };

  /**
   * Apply pose from Motus.NET / Blazor.
   * payload: { q:[5], gL, gR, hold, axesVisible, grasp?:{x,y,z,a}, release?:{x,y,z,a}, hitParts?:[string] }
   */
  window.setBambooPose = function (payloadJson) {
    if (!ready) return;
    var p = typeof payloadJson === 'string' ? JSON.parse(payloadJson) : payloadJson;
    if (p.q && p.q.length >= 5) applyJoints(p.q);
    if (typeof p.gL === 'number') setOpening(lNeg, lPos, p.gL, 0);
    if (typeof p.gR === 'number') setOpening(rNeg, rPos, p.gR, 0.005);
    setStrut(p.hold || 0, p.grasp, p.release);
    if (typeof p.axesVisible === 'boolean') {
      axisGroups.forEach(function (g) { g.visible = p.axesVisible; });
    }
    if (p.hitParts && p.hitParts.length) showHits(p.hitParts);
    else clearTints();
  };

  /** tasksJson: [{ id, planes:[{name,x,y0,z,pitch,yMm}] }] */
  window.setBambooFrames = function (tasksJson) {
    if (!ready || !frameRoot) return;
    var tasks = typeof tasksJson === 'string' ? JSON.parse(tasksJson) : tasksJson;
    while (frameRoot.children.length) frameRoot.remove(frameRoot.children[0]);
    (tasks || []).forEach(function (task) {
      (task.planes || []).forEach(function (pl) {
        var yMm = Number(pl.yMm);
        if (!isFinite(yMm)) yMm = 0;
        var holder = new THREE.Group();
        holder.position.set(pl.x, pl.y0 + yMm / 1000, pl.z);
        var axes = new THREE.Group();
        axes.rotation.x = (pl.pitch || 0) * DEG;
        var L = 0.03;
        function axis(dx, dy, dz, color) {
          var geo = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(0, 0, 0),
            new THREE.Vector3(dx, dy, dz)
          ]);
          axes.add(new THREE.Line(geo, new THREE.LineBasicMaterial({ color: color })));
        }
        axis(L, 0, 0, 0xff3355);
        axis(0, L, 0, 0x3ddc6a);
        axis(0, 0, L, 0x4aa3ff);
        holder.add(axes);
        var lab = makePlaneLabel(String(task.id) + ':' + pl.name);
        lab.position.set(0.01, 0.026, 0);
        holder.add(lab);
        frameRoot.add(holder);
      });
    });
  };

  window.fetchFile = async function (path) {
    var response = await fetch(path);
    return await response.text();
  };
})();
