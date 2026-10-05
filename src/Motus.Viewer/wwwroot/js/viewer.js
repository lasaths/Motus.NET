/**
 * Motus.Viewer Three.js draw layer for the ICD/LIS bamboo arm.
 * Visual/UX ported from the retired standalone HTML preview (bamboo-viewer/next.html).
 * Joint angles, grips, the strut pose, task frames, and collision tint names come from C#;
 * this file does not decide kinematics or collision.
 *
 * Geometry: every arm box, the pole, strut and ground are built from the cell description C#
 * hands to initThreeJs (read from bamboo_icd.urdf + BambooIcdCell) — the exact shapes Motus.NET
 * collides. Frame: native Three.js Y-up; Motus.NET world is Z-up (Motus = Rx(+90°)·viewer).
 */
(function () {
  'use strict';

  var DEG = Math.PI / 180;
  var SEG = [0.030, 0.080, 0.088, 0.088, 0.088, 0.046];
  var POLE_Y = 0.40;
  var groundMesh = null;
  var PICK = { x: 0, y: 0.00778, z: 0.09194 };
  var PLACE = { x: 0, y: 0.1506, z: 0.2000 };

  var scene, camera, renderer, view;
  var look, theta, phi, dist;
  var j1, j2, j3, j4, j5, jaw, mount;
  var lNeg, lPos, rNeg, rPos, strut;
  var strutPool = [];
  var footprintRoot = null;
  var axisGroups = [];
  var frameRoot;
  var colliderByName = {};
  var dragging = false, lx = 0, ly = 0;
  var ready = false;
  var strutGeoTemplate = null;
  var strutNodeOffsets = [-0.09, 0, 0.09];

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

  // Same formula as BambooIcdCell.PlateShift (C#): plate centre at ±(opening/2 + 2 mm).
  function setOpening(neg, pos, mm) {
    if (!neg || !pos) return;
    var shift = (mm / 1000) / 2 + 0.002;
    neg.position.x = -shift;
    pos.position.x = shift;
  }

  function applyJoints(deg) {
    j1.rotation.y = deg[0] * DEG;
    j2.rotation.x = deg[1] * DEG;
    j3.rotation.x = deg[2] * DEG;
    j4.rotation.x = deg[3] * DEG;
    j5.rotation.y = deg[4] * DEG;
  }

  /** pose = [x, y, z, qx, qy, qz, qw] in viewer world, computed by Motus FK in C#. */
  function setStrutPose(pose) {
    if (!pose || pose.length < 7) return;
    strut.position.set(pose[0], pose[1], pose[2]);
    strut.quaternion.set(pose[3], pose[4], pose[5], pose[6]);
  }

  function partMaterial(name) {
    if (/^L-grip body|^R-grip carriage/.test(name)) return mats.carriage;
    if (/rail$/.test(name)) return mats.rail;
    if (/jaw[-+]$/.test(name)) return mats.jaw;
    if (/^link |^base tube|^neck/.test(name)) return mats.tube;
    if (name === 'θ1' || name === 'θ5') return mats.wrist;
    if (name === 'θ3') return mats.shoulder;
    return mats.elbow;
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

  function makeStrutMesh() {
    var mesh = new THREE.Mesh(strutGeoTemplate, mats.strut.clone());
    var sr = 0.007;
    strutNodeOffsets.forEach(function (z) {
      var n = new THREE.Mesh(new THREE.TorusGeometry(sr + 0.0005, 0.0012, 6, 16), mats.node);
      n.position.z = z;
      mesh.add(n);
    });
    return mesh;
  }

  function drawFootprint(fp) {
    while (footprintRoot.children.length) footprintRoot.remove(footprintRoot.children[0]);
    var edgeMat = new THREE.LineBasicMaterial({ color: 0xd7c392 });
    (fp.edges || []).forEach(function (e) {
      var geo = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(e.ax, 0.002, e.az),
        new THREE.Vector3(e.bx, 0.002, e.bz)
      ]);
      footprintRoot.add(new THREE.Line(geo, edgeMat));
    });
    (fp.nodes || []).forEach(function (n) {
      var dot = new THREE.Mesh(
        new THREE.SphereGeometry(0.006, 12, 10),
        new THREE.MeshBasicMaterial({ color: 0xe8d5a3 })
      );
      dot.position.set(n.x, 0.006, n.z);
      footprintRoot.add(dot);
      var lab = makePlaneLabel(String(n.name));
      lab.position.set(n.x, 0.028, n.z);
      footprintRoot.add(lab);
    });
  }

  window.initThreeJs = function (cellJson) {
    if (typeof THREE === 'undefined') {
      throw new Error('Three.js is not loaded (check lib/three/three.min.js).');
    }
    if (ready) return;
    var cell = typeof cellJson === 'string' ? JSON.parse(cellJson) : cellJson;
    if (!cell || !cell.parts) throw new Error('initThreeJs needs the Motus cell description.');
    SEG = cell.seg || SEG;
    POLE_Y = cell.poleY || POLE_Y;

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
      new THREE.PlaneGeometry(cell.ground.size, cell.ground.size),
      new THREE.MeshStandardMaterial({ color: 0x070707, roughness: 1, metalness: 0 })
    );
    ground.rotation.x = -Math.PI / 2;
    scene.add(ground);
    registerCollider('ground', [ground]);
    groundMesh = ground;

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

    var pole = new THREE.Mesh(new THREE.CylinderGeometry(cell.pole.radius, cell.pole.radius, cell.pole.length, cell.pole.segments), mats.pole);
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
    mount.add(track(axisX(0x3ddc6a, '#3ddc6a', 'L-grip')));

    j1 = new THREE.Group(); j1.position.y = -SEG[0]; mount.add(j1);
    j1.add(track(axisY(0x33e0e0, '#33e0e0', 'θ1')));
    j2 = new THREE.Group(); j2.position.y = -SEG[1]; j1.add(j2);
    j2.add(track(axisX(0xff8800, '#ff8800', 'θ2')));
    j3 = new THREE.Group(); j3.position.y = -SEG[2]; j2.add(j3);
    j3.add(track(axisX(0xffffff, '#ffffff', 'θ3')));
    j4 = new THREE.Group(); j4.position.y = -SEG[3]; j3.add(j4);
    j4.add(track(axisX(0xff8800, '#ff8800', 'θ4')));
    j5 = new THREE.Group(); j5.position.y = -SEG[4]; j4.add(j5);
    j5.add(track(axisY(0x33e0e0, '#33e0e0', 'θ5')));
    jaw = new THREE.Group(); jaw.position.y = -SEG[5]; j5.add(jaw);
    jaw.add(track(axisX(0x3ddc6a, '#3ddc6a', 'R-grip')));

    // Arm boxes straight from the URDF <collision> entries Motus.NET checks (link frame == group frame).
    var linkGroup = { mount: mount, link_1: j1, link_2: j2, link_3: j3, link_4: j4, link_5: j5 };
    var byName = {};
    cell.parts.forEach(function (p) {
      var parent = linkGroup[p.link];
      if (!parent) return;
      var mesh = new THREE.Mesh(new THREE.BoxGeometry(p.size[0], p.size[1], p.size[2]), partMaterial(p.name));
      mesh.position.set(p.pos[0], p.pos[1], p.pos[2]);
      parent.add(mesh);
      registerCollider(p.name, [mesh]);
      byName[p.name] = mesh;
    });
    lNeg = byName['L-grip jaw-']; lPos = byName['L-grip jaw+'];
    rNeg = byName['R-grip jaw-']; rPos = byName['R-grip jaw+'];

    var sr = cell.strut.radius, sl = cell.strut.length;
    strutGeoTemplate = new THREE.CylinderGeometry(sr, sr, sl, cell.strut.segments);
    strutGeoTemplate.rotateX(Math.PI / 2);
    strut = makeStrutMesh();
    strut.position.set(PICK.x, PICK.y, PICK.z);
    scene.add(strut);
    registerCollider('strut', [strut]);
    strutPool = [strut];

    footprintRoot = new THREE.Group();
    scene.add(footprintRoot);
    if (cell.footprint && cell.footprint.edges) {
      drawFootprint(cell.footprint);
    }
    if (cell.store && cell.store.slots) {
      cell.store.slots.forEach(function (slot, idx) {
        if (idx === 0) return; // primary strut mesh covers first / active
        var mesh = makeStrutMesh();
        mesh.position.set(slot.x, slot.y, slot.z);
        mesh.rotation.x = (slot.pitch || 0) * DEG;
        scene.add(mesh);
        strutPool.push(mesh);
        registerCollider('store-' + (slot.id || idx), [mesh]);
      });
    }

    frameRoot = new THREE.Group();
    scene.add(frameRoot);

    setOpening(lNeg, lPos, 24);
    setOpening(rNeg, rPos, 40);
    applyJoints([-90, -72, -80, -24, 0]);

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
   * payload: { q:[5], gL, gR, strut:[...], struts?:[{name,pose:[7]}], axesVisible, hitParts?:[string] }
   * hitParts are Motus contact body names (URDF collision names, 'pole', 'strut', 'ground', mount parts).
   */
  window.setBambooPose = function (payloadJson) {
    if (!ready) return;
    var p = typeof payloadJson === 'string' ? JSON.parse(payloadJson) : payloadJson;
    if (p.q && p.q.length >= 5) applyJoints(p.q);
    if (typeof p.gL === 'number') setOpening(lNeg, lPos, p.gL);
    if (typeof p.gR === 'number') setOpening(rNeg, rPos, p.gR);
    if (p.struts && p.struts.length) {
      ensureStrutPool(p.struts.length);
      for (var i = 0; i < strutPool.length; i++) {
        if (i < p.struts.length) {
          var s = p.struts[i];
          strutPool[i].visible = true;
          applyPose7(strutPool[i], s.pose);
          if (s.name) registerCollider(s.name, [strutPool[i]]);
        } else {
          strutPool[i].visible = false;
        }
      }
    } else {
      setStrutPose(p.strut);
      for (var j = 1; j < strutPool.length; j++) strutPool[j].visible = false;
      if (strutPool[0]) strutPool[0].visible = true;
    }
    if (typeof p.axesVisible === 'boolean') {
      axisGroups.forEach(function (g) { g.visible = p.axesVisible; });
    }
    if (p.hitParts && p.hitParts.length) showHits(p.hitParts);
    else clearTints();
  };

  function applyPose7(mesh, pose) {
    if (!pose || pose.length < 7) return;
    mesh.position.set(pose[0], pose[1], pose[2]);
    mesh.quaternion.set(pose[3], pose[4], pose[5], pose[6]);
  }

  function ensureStrutPool(n) {
    while (strutPool.length < n) {
      var mesh = makeStrutMesh();
      scene.add(mesh);
      strutPool.push(mesh);
    }
  }

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
