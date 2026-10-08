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
  var anchorGroup = null, anchorMarkers = [];
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
      var ay = (e.ay != null) ? e.ay : 0.002;
      var by = (e.by != null) ? e.by : 0.002;
      var geo = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(e.ax, ay, e.az),
        new THREE.Vector3(e.bx, by, e.bz)
      ]);
      footprintRoot.add(new THREE.Line(geo, edgeMat));
    });
    (fp.nodes || []).forEach(function (n) {
      var ny = (n.y != null) ? n.y : 0.006;
      var dot = new THREE.Mesh(
        new THREE.SphereGeometry(0.006, 12, 10),
        new THREE.MeshBasicMaterial({ color: 0xe8d5a3 })
      );
      dot.position.set(n.x, ny, n.z);
      footprintRoot.add(dot);
      var lab = makePlaneLabel(String(n.name));
      lab.position.set(n.x, ny + 0.022, n.z);
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

    if (cell.ground.pads && cell.ground.pads.length) {
      var padGroup = [];
      cell.ground.pads.forEach(function (pad, i) {
        var hx = pad.halfX || pad.half, hz = pad.halfZ || pad.half;
        var g = new THREE.Mesh(
          new THREE.PlaneGeometry(hx * 2, hz * 2),
          new THREE.MeshStandardMaterial({ color: 0x070707, roughness: 1, metalness: 0 })
        );
        g.rotation.x = -Math.PI / 2;
        g.position.set(pad.x, 0, pad.z);
        scene.add(g);
        padGroup.push(g);
        if (pad.name) registerCollider(pad.name, [g]);
      });
      registerCollider('ground', padGroup);
      groundMesh = padGroup[0];
    } else {
      var ground = new THREE.Mesh(
        new THREE.PlaneGeometry(cell.ground.size, cell.ground.size),
        new THREE.MeshStandardMaterial({ color: 0x070707, roughness: 1, metalness: 0 })
      );
      ground.rotation.x = -Math.PI / 2;
      scene.add(ground);
      registerCollider('ground', [ground]);
      groundMesh = ground;
    }

    var grid = new THREE.GridHelper(1.4, 14, 0x3a3a3a, 0x242424);
    grid.position.y = 0.0006;
    scene.add(grid);

    var ring = new THREE.Mesh(
      new THREE.RingGeometry(0.020, 0.025, 64),
      new THREE.MeshBasicMaterial({ color: 0xd7c392, side: THREE.DoubleSide })
    );
    ring.rotation.x = -Math.PI / 2;
    ring.position.set(PLACE.x, 0.0016, PLACE.z);
    // /bamboo place marker only; /bamboo-truss (fixtures present) marks the clamped base instead.
    if (!(cell.posts && cell.posts.length)) scene.add(ring);

    // /bamboo-truss sets includePole=false: no horizontal hang rod; arm starts at the L-grip dock only.
    if (cell.includePole !== false && cell.pole) {
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
    }

    // /bamboo-truss fixtures: anchor posts, stakes (L-grip anchors) and the feed pegs — exactly the Motus objects.
    if ((cell.posts || []).length) {
      // Frame the whole site: the outboard stakes / stock beyond A as well as the A–D span.
      var minX = Math.min.apply(null, cell.posts.map(function (p) { return p.x; }));
      var maxX = Math.max.apply(null, cell.posts.map(function (p) { return p.x; }));
      look.set((minX + maxX) / 2 - 0.12, 0.12, 0.0);
      dist = Math.max(1.12, (maxX - minX) * 1.5);
      updateCamera();
    }
    (cell.posts || []).forEach(function (post) {
      var m = new THREE.Mesh(new THREE.CylinderGeometry(post.radius, post.radius, post.top, 20), mats.node);
      m.position.set(post.x, post.top / 2, post.z);
      scene.add(m);
      registerCollider(post.name, [m]);
    });
    (cell.racks || []).forEach(function (r) {
      var m = new THREE.Mesh(new THREE.BoxGeometry(r.halfX * 2, r.top, r.halfZ * 2), mats.node);
      m.position.set(r.x, r.top / 2, r.z);
      scene.add(m);
      registerCollider(r.name, [m]);
    });
    // Clamp markers: one ring around each clamped gripper's slot axis + a label naming what it is clamped on
    // (pool of two: the base and, when it is also clamped, the other gripper).
    anchorGroup = new THREE.Group();
    scene.add(anchorGroup);
    anchorMarkers = [];
    for (var ai = 0; ai < 2; ai++) {
      var ring = new THREE.Mesh(new THREE.TorusGeometry(0.034, 0.0035, 10, 48),
        new THREE.MeshBasicMaterial({ color: 0xffb020, depthTest: false, transparent: true, opacity: 0.9 }));
      ring.renderOrder = 4;
      ring.visible = false;
      anchorGroup.add(ring);
      anchorMarkers.push({ ring: ring, label: null, text: '' });
    }

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
    // Node rings at ±0.3·length (±0.09 m on the 0.30 m ICD strut) so shorter canes keep them on the cane.
    strutNodeOffsets = [-0.3 * sl, 0, 0.3 * sl];
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
        mesh.rotation.order = 'YXZ';
        mesh.rotation.y = (slot.yaw || 0) * DEG;
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
   * payload: { q:[5], mount?:{p:[3],q:[4]}, anchors?:[{gripper,on,isBase,held,p:[3],axis:[3]}], hang?:{x,y,z}, gL, gR, strut:[...], struts?:[{name,pose:[7]}], axesVisible, hitParts?:[string] }
   * hitParts are Motus contact body names (URDF collision names, 'pole', 'strut', 'ground', mount parts).
   */
  window.setBambooPose = function (payloadJson) {
    if (!ready) return;
    var p = typeof payloadJson === 'string' ? JSON.parse(payloadJson) : payloadJson;
    // Grounded base swapping: the L-grip mount pose is FK from whichever gripper is clamped (full pose).
    if (p.mount && mount && p.mount.p) {
      mount.position.set(p.mount.p[0], p.mount.p[1], p.mount.p[2]);
      mount.quaternion.set(p.mount.q[0], p.mount.q[1], p.mount.q[2], p.mount.q[3]);
    } else if (p.hang && mount && typeof p.hang.x === 'number') mount.position.set(p.hang.x, p.hang.y, p.hang.z);
    if (anchorGroup) {
      var list = p.anchors || (p.anchor ? [p.anchor] : []);
      anchorMarkers.forEach(function (m, idx) {
        var a = list[idx];
        if (!a || !a.p) {
          m.ring.visible = false;
          if (m.label) m.label.visible = false;
          return;
        }
        m.ring.visible = true;
        m.ring.position.set(a.p[0], a.p[1], a.p[2]);
        // Torus lies in its XY plane: turn its normal (+Z) onto the clamped slot axis.
        var ax = new THREE.Vector3(a.axis[0], a.axis[1], a.axis[2]).normalize();
        m.ring.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), ax);
        m.ring.material.color.setHex(a.held ? 0x40c0ff : 0xffb020);
        var text = a.held ? a.gripper + ' holds cane' : a.gripper + ' clamped on ' + a.on + (a.isBase ? ' (base)' : '');
        if (text !== m.text) {
          if (m.label) scene.remove(m.label);
          m.label = makePlaneLabel(text);
          m.label.scale.set(0.2, 0.04, 1);
          scene.add(m.label);
          m.text = text;
        }
        // Labels sit above their ring. A later label whose on-screen box would overlap an earlier one (rings close
        // together, e.g. the cane just lifted next to the base clamp) is pushed up until the boxes are apart.
        var lpos = new THREE.Vector3(a.p[0], a.p[1] + 0.06, a.p[2]);
        camera.updateMatrixWorld();
        var hw = m.label.scale.x / 2, hh = m.label.scale.y / 2;
        function toCam(v) { return v.clone().applyMatrix4(camera.matrixWorldInverse); }
        function overlaps(pa, pb) {
          var ca = toCam(pa), cb = toCam(pb);
          var za = Math.max(1e-3, -ca.z), zb = Math.max(1e-3, -cb.z);
          return Math.abs(ca.x / za - cb.x / zb) < hw / za + hw / zb && Math.abs(ca.y / za - cb.y / zb) < hh / za + hh / zb + 0.004;
        }
        for (var guard = 0; guard < 40; guard++) {
          var hit = false;
          for (var pi = 0; pi < idx; pi++) {
            var o = anchorMarkers[pi];
            if (o.label && o.label.visible && overlaps(o.label.position, lpos)) { hit = true; break; }
          }
          if (!hit) break;
          lpos.y += 0.01;
        }
        m.label.position.copy(lpos);
        m.label.visible = true;
      });
    }
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

  /** tasksJson: [{ id, planes:[{name,x,y0,z,pitch,yaw?,yMm}] }] */
  // Pick/place frame labels are shown only for the member being built (null: none, e.g. during the walk).
  var activeFrameMember = null;
  window.setBambooActiveMember = function (id) {
    activeFrameMember = id == null || id === '' ? null : String(id);
    if (!frameRoot) return;
    frameRoot.traverse(function (o) {
      if (o.isSprite && o.userData && 'member' in o.userData)
        o.visible = activeFrameMember != null && o.userData.member === activeFrameMember;
    });
  };

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
        holder.rotation.y = (pl.yaw || 0) * DEG;
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
        lab.position.set(0.01, 0.02, 0);
        lab.scale.set(0.066, 0.017, 1); // small: clamp / hold labels must stay readable over them
        lab.userData.member = task.obj == null ? null : String(task.obj);
        lab.visible = activeFrameMember != null && lab.userData.member === activeFrameMember;
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
