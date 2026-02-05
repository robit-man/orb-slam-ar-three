import * as THREE from 'three';
import { AlvaAR } from './alva_ar.js';
import { AlvaARConnectorTHREE } from './alva_ar_three.js';

const DEFAULT_FOV = 60;
const TARGET_FPS = 30;
const MAX_POINT_CAP = 12000;
const BACKPROJECT_DEPTH = 3.0;
const SPHERE_R = 6.0;

const VIDEO_CONSTRAINTS = {
  video: {
    facingMode: 'environment',
    aspectRatio: 16 / 9,
    width: { ideal: 1280 }
  },
  audio: false
};

const dom = {
  startBtn: document.getElementById('startBtn'),
  stopBtn: document.getElementById('stopBtn'),
  resetBtn: document.getElementById('resetBtn'),
  viewBtn: document.getElementById('viewBtn'),
  imuToggle: document.getElementById('imuToggle'),
  sphereBtn: document.getElementById('sphereBtn'),
  pickBtn: document.getElementById('pickBtn'),
  gpsBtn: document.getElementById('gpsBtn'),
  calBtn: document.getElementById('calBtn'),
  calResetBtn: document.getElementById('calResetBtn'),
  status: document.getElementById('status'),
  imuStatus: document.getElementById('imuStatus'),
  fps: document.getElementById('fps'),
  pts2d: document.getElementById('pts2d'),
  pts3d: document.getElementById('pts3d'),
  threeCanvas: document.getElementById('three-canvas'),
  cameraCanvas: document.getElementById('camera-canvas')
};

const applyPose = AlvaARConnectorTHREE.Initialize(THREE);

const state = {
  started: false,
  running: false,
  resizing: false,
  tracking: false,
  useIMU: true,
  alva: null,
  imu: null,
  media: null,
  slamCanvas: null,
  slamCtx: null,
  cameraCtx: null,
  processWidth: 0,
  processHeight: 0,
  view: null,
  sphereMode: true,
  pickMode: false,
  anchorUp: new THREE.Vector3(0, 1, 0),
  anchorFrame: null,
  slamCenter: new THREE.Vector3(),
  slamCenterSmooth: new THREE.Vector3(),
  slamCenterInitialized: false,
  points2d: 0,
  fps: 0,
  frames: 0,
  lastFpsSample: performance.now(),
  lastUiUpdate: 0
};

state.anchorFrame = computeTangentFrameFromUp(state.anchorUp);

// Scene Map:
// - mainCamera: third-person overview
// - trackedCamera: SLAM pose (first-person)
// - anchorGroup: tangent frame on globe
// - slamWorldGroup: SLAM local space, recentered to point cloud

class Camera
{
  static async Initialize(constraints = null)
  {
    if ('facingMode' in constraints && 'deviceId' in constraints)
    {
      throw new Error("Camera settings 'deviceId' and 'facingMode' are mutually exclusive.");
    }

    if ('facingMode' in constraints && ['environment', 'user'].indexOf(constraints.facingMode) === -1)
    {
      throw new Error("Camera settings 'facingMode' can only be 'environment' or 'user'.");
    }

    const setupUserMediaStream = (permission) =>
    {
      return new Promise((resolve, reject) =>
      {
        const onSuccess = (stream) =>
        {
          const track = stream.getVideoTracks()[0];

          if (typeof track === 'undefined')
          {
            reject(new Error('Failed to access camera: Permission denied (No track).'));
          }
          else
          {
            const video = document.createElement('video');

            video.setAttribute('autoplay', 'autoplay');
            video.setAttribute('playsinline', 'playsinline');
            video.setAttribute('webkit-playsinline', 'webkit-playsinline');
            video.srcObject = stream;

            video.onloadedmetadata = () =>
            {
              const settings = track.getSettings();
              const tw = settings.width;
              const th = settings.height;
              const vw = video.videoWidth;
              const vh = video.videoHeight;

              if (vw !== tw || vh !== th)
              {
                console.warn(`Video dimensions mismatch: width: ${tw}/${vw}, height: ${th}/${vh}`);
              }

              video.style.width = vw + 'px';
              video.style.height = vh + 'px';
              video.width = vw;
              video.height = vh;
              video.play();

              resolve(new Camera(video, stream));
            };
          }
        };

        const onFailure = (error) =>
        {
          switch (error.name)
          {
            case 'NotFoundError':
            case 'DevicesNotFoundError':
              reject(new Error('Failed to access camera: Camera not found.'));
              return;
            case 'SourceUnavailableError':
              reject(new Error('Failed to access camera: Camera busy.'));
              return;
            case 'PermissionDeniedError':
            case 'SecurityError':
              reject(new Error('Failed to access camera: Permission denied.'));
              return;
            default:
              reject(new Error('Failed to access camera: Rejected.'));
              return;
          }
        };

        if (permission && permission.state === 'denied')
        {
          reject(new Error('Failed to access camera: Permission denied.'));
          return;
        }

        navigator.mediaDevices.getUserMedia(constraints).then(onSuccess).catch(onFailure);
      });
    };

    if (navigator.permissions && navigator.permissions.query)
    {
      return navigator.permissions
        .query({ name: 'camera' })
        .then(permission => setupUserMediaStream(permission))
        .catch(() => setupUserMediaStream());
    }

    return setupUserMediaStream();
  }

  constructor(videoElement, stream)
  {
    this.el = videoElement;
    this.stream = stream;
    this.width = videoElement.videoWidth;
    this.height = videoElement.videoHeight;
  }

  stop()
  {
    if (this.stream)
    {
      const tracks = this.stream.getTracks();
      tracks.forEach(track => track.stop());
    }
  }
}

const deg2rad = Math.PI / 180;

// ---------- IMU orientation helpers (Phone Twins style) ----------
const zee = new THREE.Vector3(0, 0, 1);
const euler = new THREE.Euler();
const q0 = new THREE.Quaternion();
const q1 = new THREE.Quaternion(-Math.sqrt(0.5), 0, 0, Math.sqrt(0.5));

function screenOrientationRad()
{
  const a = (screen.orientation && typeof screen.orientation.angle === 'number')
    ? screen.orientation.angle
    : (typeof window.orientation === 'number' ? window.orientation : 0);
  return a * Math.PI / 180;
}

function deviceEulerToQuaternion(alpha, beta, gamma, orientRad)
{
  euler.set(beta * deg2rad, alpha * deg2rad, -gamma * deg2rad, 'YXZ');
  const q = new THREE.Quaternion().setFromEuler(euler);
  q.multiply(q1);
  q.multiply(q0.setFromAxisAngle(zee, -orientRad));
  return q;
}

function compassHeadingFromEuler(alpha, beta, gamma)
{
  const a = alpha * deg2rad;
  const b = beta * deg2rad;
  const g = gamma * deg2rad;
  const cA = Math.cos(a), sA = Math.sin(a);
  const sB = Math.sin(b);
  const cG = Math.cos(g), sG = Math.sin(g);
  const rA = -cA * sG - sA * sB * cG;
  const rB = -sA * sG + cA * sB * cG;
  let heading = Math.atan2(rA, rB);
  if (heading < 0) heading += 2 * Math.PI;
  return heading * 180 / Math.PI;
}

function headingFromQuaternion(q)
{
  const top = new THREE.Vector3(0, 1, 0).applyQuaternion(q);
  top.y = 0;
  const m = top.length();
  if (m < 1e-4) return null;
  top.multiplyScalar(1 / m);
  let deg = Math.atan2(top.x, -top.z) * 180 / Math.PI;
  deg = (deg + 360) % 360;
  return deg;
}

function wrapRad(a)
{
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

function computeLinearAccelFallbackFromAccG(accG, qRaw)
{
  if (!accG) return null;
  if (typeof accG.x !== 'number' || typeof accG.y !== 'number' || typeof accG.z !== 'number') return null;
  const worldG = new THREE.Vector3(0, -9.81, 0);
  const inv = qRaw.clone().invert();
  const gDevice = worldG.clone().applyQuaternion(inv);
  return { x: accG.x + gDevice.x, y: accG.y + gDevice.y, z: accG.z + gDevice.z };
}

class IMU
{
  static async Initialize()
  {
    if (window.isSecureContext === false)
    {
      throw new Error('DeviceOrientation is only available in secure contexts (https).');
    }

    if (window.DeviceOrientationEvent === undefined)
    {
      throw new Error('DeviceOrientation not supported.');
    }

    if (window.DeviceMotionEvent === undefined)
    {
      throw new Error('DeviceMotion not supported.');
    }

    const requestPermission = async (EventClass, name) =>
    {
      if (EventClass && typeof EventClass.requestPermission === 'function')
      {
        const state = await EventClass.requestPermission();
        if (state !== 'granted')
        {
          throw new Error(`${name} permission denied.`);
        }
      }
    };

    await requestPermission(window.DeviceOrientationEvent, 'DeviceOrientation');
    await requestPermission(window.DeviceMotionEvent, 'DeviceMotion');

    return new IMU();
  }

  constructor()
  {
    this.motion = [];
    this.orientation = { x: 0, y: 0, z: 0, w: 1 };
    this.qRaw = new THREE.Quaternion();
    this.qOffset = new THREE.Quaternion();
    this.qDisp = new THREE.Quaternion();
    this.heading = null;
    this.headingAcc = null;
    this.acc = null;
    this.accG = null;

    this._onOrientation = this.handleDeviceOrientation.bind(this);
    this._onMotion = this.handleDeviceMotion.bind(this);
    this.start();
  }

  start()
  {
    window.addEventListener('deviceorientation', this._onOrientation, true);
    window.addEventListener('deviceorientationabsolute', this._onOrientation, true);
    window.addEventListener('devicemotion', this._onMotion, true);
  }

  stop()
  {
    window.removeEventListener('deviceorientation', this._onOrientation, true);
    window.removeEventListener('deviceorientationabsolute', this._onOrientation, true);
    window.removeEventListener('devicemotion', this._onMotion, true);
  }

  clear()
  {
    this.motion.length = 0;
  }

  syncOrientation()
  {
    this.orientation.x = this.qDisp.x;
    this.orientation.y = this.qDisp.y;
    this.orientation.z = this.qDisp.z;
    this.orientation.w = this.qDisp.w;
  }

  handleDeviceOrientation(ev)
  {
    if (ev.alpha == null || ev.beta == null || ev.gamma == null) return;

    this.qRaw.copy(deviceEulerToQuaternion(ev.alpha, ev.beta, ev.gamma, screenOrientationRad()));
    this.qDisp.copy(this.qOffset).multiply(this.qRaw);
    this.syncOrientation();

    let heading = null;
    let headingAcc = null;
    if (typeof ev.webkitCompassHeading === 'number' && ev.webkitCompassHeading >= 0)
    {
      heading = ev.webkitCompassHeading;
      if (typeof ev.webkitCompassAccuracy === 'number') headingAcc = ev.webkitCompassAccuracy;
    }
    else if (ev.absolute === true)
    {
      heading = compassHeadingFromEuler(ev.alpha, ev.beta, ev.gamma);
    }
    this.heading = (typeof heading === 'number' && Number.isFinite(heading)) ? ((heading % 360) + 360) % 360 : null;
    this.headingAcc = (typeof headingAcc === 'number' && Number.isFinite(headingAcc)) ? headingAcc : null;
  }

  handleDeviceMotion(ev)
  {
    const rate = ev.rotationRate || {};
    const accel = ev.acceleration || {};
    const accelG = ev.accelerationIncludingGravity || {};

    const gx = (rate.beta || 0) * deg2rad;
    const gy = (rate.gamma || 0) * deg2rad;
    const gz = (rate.alpha || 0) * deg2rad;

    const ax = accel.x || 0;
    const ay = accel.y || 0;
    const az = accel.z || 0;

    if (accelG && typeof accelG.x === 'number' && typeof accelG.y === 'number' && typeof accelG.z === 'number')
    {
      this.accG = { x: accelG.x, y: accelG.y, z: accelG.z };
    }

    if (accel && typeof accel.x === 'number' && typeof accel.y === 'number' && typeof accel.z === 'number')
    {
      this.acc = { x: ax, y: ay, z: az };
    }
    else if (this.accG)
    {
      const lin = computeLinearAccelFallbackFromAccG(this.accG, this.qRaw);
      if (lin) this.acc = lin;
    }

    const timestamp = Date.now();
    this.motion.push({ timestamp, gx, gy, gz, ax, ay, az });

    if (this.motion.length > 120)
    {
      this.motion.shift();
    }
  }

  calibrateYawOnce()
  {
    if (this.heading == null)
    {
      setStatus('Calibrate: no compass heading yet. Ensure HTTPS and allow motion.');
      return;
    }

    const currentH = headingFromQuaternion(this.qDisp);
    if (currentH == null)
    {
      setStatus('Calibrate: heading undefined (device nearly vertical).');
      return;
    }

    const targetH = ((this.heading % 360) + 360) % 360;
    const deltaDeg = ((targetH - currentH + 540) % 360) - 180;
    const deltaRad = wrapRad(deltaDeg * Math.PI / 180);
    const qDelta = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), -deltaRad);
    this.qOffset.premultiply(qDelta);
    this.qDisp.copy(this.qOffset).multiply(this.qRaw);
    this.syncOrientation();
    setStatus(`Calibrated yaw. Compass: ${targetH.toFixed(1)} deg`);
  }

  resetYawCalibration()
  {
    this.qOffset.identity();
    this.qDisp.copy(this.qOffset).multiply(this.qRaw);
    this.syncOrientation();
    setStatus('Yaw calibration reset.');
  }
}

function setStatus(text)
{
  dom.status.textContent = text;
}

function onFrame(frameTickFn, fps = 30)
{
  const fpsInterval = ~~(1000 / fps);
  let t1 = performance.now();

  const onAnimationFrame = async () =>
  {
    const t2 = performance.now();
    const td = t2 - t1;

    if (td > fpsInterval)
    {
      t1 = t2 - (td % fpsInterval);

      if ((await frameTickFn(t2)) === false)
      {
        return;
      }
    }

    requestAnimationFrame(onAnimationFrame);
  };

  requestAnimationFrame(onAnimationFrame);
}

// ---------- Three helpers ----------
const ORIGIN = new THREE.Vector3(0, 0, 0);
const phoneGeom = new THREE.BoxGeometry(0.6, 1.2, 0.08);

function makePhoneRig(accent)
{
  const group = new THREE.Group();
  const mat = new THREE.MeshStandardMaterial({
    color: accent ? 0xffae00 : 0xffffff,
    roughness: accent ? 0.25 : 0.65,
    metalness: 0.05,
    emissive: 0x000000
  });
  const mesh = new THREE.Mesh(phoneGeom, mat);
  group.add(mesh);

  const fwd = new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), ORIGIN, 1.0, accent ? 0xffae00 : 0xffffff);
  group.add(fwd);

  const ax = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), ORIGIN, 0.001, accent ? 0xffae00 : 0xffffff);
  const ay = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), ORIGIN, 0.001, accent ? 0xffae00 : 0xffffff);
  const az = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), ORIGIN, 0.001, accent ? 0xffae00 : 0xffffff);
  group.add(ax, ay, az);

  return { group, mesh, fwd, accelArrows: { ax, ay, az } };
}

function updateAccelArrows(accelArrows, acc)
{
  const SCALE = 0.08;
  const MAX = 1.1;
  const MIN_VIS = 0.12;
  const comps = [
    ['ax', acc && acc.x, new THREE.Vector3(1, 0, 0)],
    ['ay', acc && acc.y, new THREE.Vector3(0, 1, 0)],
    ['az', acc && acc.z, new THREE.Vector3(0, 0, 1)],
  ];
  for (let i = 0; i < comps.length; i++)
  {
    const k = comps[i][0];
    const v = comps[i][1];
    const axis = comps[i][2];
    const ar = accelArrows[k];
    if (!ar) continue;
    if (typeof v !== 'number' || !Number.isFinite(v) || Math.abs(v) < MIN_VIS)
    {
      ar.setLength(0.001);
      ar.visible = false;
      continue;
    }
    ar.visible = true;
    const dirv = axis.clone().multiplyScalar(v >= 0 ? 1 : -1);
    const len = Math.min(MAX, Math.abs(v) * SCALE);
    ar.setDirection(dirv.normalize());
    ar.setLength(Math.max(0.02, len));
  }
}

function latLonToUp(latDeg, lonDeg)
{
  const d = Math.PI / 180;
  const phi = (90 - latDeg) * d;
  const theta = lonDeg * d;
  const sinPhi = Math.sin(phi);

  const x = sinPhi * Math.sin(theta);
  const y = Math.cos(phi);
  const z = -sinPhi * Math.cos(theta);

  const v = new THREE.Vector3(x, y, z);
  if (v.lengthSq() < 1e-12) return new THREE.Vector3(0, 1, 0);
  return v.normalize();
}

const NORTH_POLE = new THREE.Vector3(0, 1, 0);
const FALLBACK_REF = new THREE.Vector3(0, 0, -1);

function computeTangentFrameFromUp(upVec)
{
  const up = upVec.clone().normalize();

  const east = new THREE.Vector3().crossVectors(NORTH_POLE, up);
  if (east.lengthSq() < 1e-10)
  {
    east.crossVectors(FALLBACK_REF, up);
  }
  east.normalize();

  const north = new THREE.Vector3().crossVectors(up, east).normalize();
  const zAxis = north.clone().multiplyScalar(-1);

  const m = new THREE.Matrix4().makeBasis(east, up, zAxis);
  const q = new THREE.Quaternion().setFromRotationMatrix(m);

  return { up, east, north, q };
}

function createPointCloud(maxPoints)
{
  const geom = new THREE.BufferGeometry();
  const positions = new Float32Array(maxPoints * 3);
  geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geom.setDrawRange(0, 0);

  const mat = new THREE.PointsMaterial({
    size: 0.01,
    sizeAttenuation: true,
    transparent: true,
    opacity: 0.9
  });

  const points = new THREE.Points(geom, mat);

  return {
    points,
    geom,
    positions,
    maxPoints,
    cursor: 0,
    count: 0,
    sum: new THREE.Vector3(),
    vCamPos: new THREE.Vector3(),
    vRay: new THREE.Vector3(),
    vTarget: new THREE.Vector3()
  };
}

function clearPointCloud(pc)
{
  pc.geom.setDrawRange(0, 0);
  pc.geom.attributes.position.needsUpdate = true;
  pc.cursor = 0;
  pc.count = 0;
  pc.sum.set(0, 0, 0);
}

function appendPointCloudFrom2D(pc, camera, points2d, width, height, depth, outCenter)
{
  if (!points2d || !points2d.length)
  {
    return 0;
  }

  camera.updateMatrixWorld(true);
  camera.getWorldPosition(pc.vCamPos);

  let n = 0;

  for (let i = 0; i < points2d.length && n < pc.maxPoints; i++)
  {
    const p = points2d[i];
    const ndcX = (p.x / width) * 2 - 1;
    const ndcY = -((p.y / height) * 2 - 1);

    pc.vTarget.set(ndcX, ndcY, 0.5).unproject(camera);
    pc.vRay.copy(pc.vTarget).sub(pc.vCamPos).normalize();
    pc.vTarget.copy(pc.vCamPos).addScaledVector(pc.vRay, depth);

    const index = pc.cursor % pc.maxPoints;
    const j = index * 3;

    if (pc.count === pc.maxPoints)
    {
      pc.sum.x -= pc.positions[j + 0];
      pc.sum.y -= pc.positions[j + 1];
      pc.sum.z -= pc.positions[j + 2];
    }
    else
    {
      pc.count += 1;
    }

    pc.positions[j + 0] = pc.vTarget.x;
    pc.positions[j + 1] = pc.vTarget.y;
    pc.positions[j + 2] = pc.vTarget.z;
    pc.sum.x += pc.vTarget.x;
    pc.sum.y += pc.vTarget.y;
    pc.sum.z += pc.vTarget.z;
    pc.cursor += 1;
    n++;
  }

  pc.geom.setDrawRange(0, pc.count);
  pc.geom.attributes.position.needsUpdate = true;

  if (outCenter && pc.count > 0)
  {
    outCenter.set(pc.sum.x / pc.count, pc.sum.y / pc.count, pc.sum.z / pc.count);
  }

  return n;
}

function createSceneView()
{
  const scene = new THREE.Scene();
  scene.fog = new THREE.Fog(0x000000, 12, 35);

  const renderer = new THREE.WebGLRenderer({
    canvas: dom.threeCanvas,
    antialias: true,
    alpha: false
  });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setSize(window.innerWidth, window.innerHeight, false);

  const mainCamera = new THREE.PerspectiveCamera(65, window.innerWidth / window.innerHeight, 0.05, 100);
  mainCamera.position.set(0, 15, 15);
  mainCamera.lookAt(0, 1.2, 0);

  const trackedCamera = new THREE.PerspectiveCamera(DEFAULT_FOV, window.innerWidth / window.innerHeight, 0.01, 100);

  scene.add(new THREE.AmbientLight(0xffffff, 0.45));
  const dir = new THREE.DirectionalLight(0xffffff, 0.8);
  dir.position.set(5, 8, 5);
  scene.add(dir);

  const ground = new THREE.Mesh(
    new THREE.CircleGeometry(20, 64),
    new THREE.MeshStandardMaterial({ color: 0x050505, roughness: 1, metalness: 0 })
  );
  ground.rotation.x = -Math.PI / 2;
  ground.position.y = 0;
  scene.add(ground);

  const grid = new THREE.GridHelper(20, 20, 0x222222, 0x111111);
  grid.position.y = 0.01;
  scene.add(grid);

  const globe = new THREE.Mesh(
    new THREE.SphereGeometry(SPHERE_R, 64, 32),
    new THREE.MeshStandardMaterial({
      color: 0xaaaaaa,
      roughness: 1,
      metalness: 0,
      wireframe: true,
      transparent: true,
      opacity: 0.35
    })
  );
  globe.visible = false;
  scene.add(globe);

  const anchorMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.12, 16, 12),
    new THREE.MeshStandardMaterial({ color: 0xffae00, roughness: 0.3, metalness: 0.1 })
  );
  anchorMarker.visible = false;
  scene.add(anchorMarker);

  const anchorGroup = new THREE.Group();
  const slamWorldGroup = new THREE.Group();
  anchorGroup.add(slamWorldGroup);
  scene.add(anchorGroup);

  const pointCloud = createPointCloud(MAX_POINT_CAP);
  slamWorldGroup.add(pointCloud.points);

  const cameraHelper = new THREE.CameraHelper(trackedCamera);
  slamWorldGroup.add(cameraHelper);

  const cameraMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.05, 12, 12),
    new THREE.MeshStandardMaterial({ color: 0xffd166, roughness: 0.4, metalness: 0.1 })
  );
  slamWorldGroup.add(cameraMarker);

  const phoneRig = makePhoneRig(true);
  phoneRig.group.position.set(0, 0.4, 0);
  slamWorldGroup.add(phoneRig.group);

  return {
    renderer,
    scene,
    mainCamera,
    trackedCamera,
    activeCamera: mainCamera,
    ground,
    grid,
    globe,
    anchorMarker,
    anchorGroup,
    slamWorldGroup,
    pointCloud,
    cameraHelper,
    cameraMarker,
    phoneRig,
    raycaster: new THREE.Raycaster(),
    pointerNdc: new THREE.Vector2()
  };
}

function setAnchorFromUp(upVec)
{
  state.anchorUp.copy(upVec).normalize();
  state.anchorFrame = computeTangentFrameFromUp(state.anchorUp);

  if (state.view)
  {
    state.view.anchorMarker.position.copy(state.anchorFrame.up).multiplyScalar(SPHERE_R);
    state.view.anchorMarker.visible = state.sphereMode;
  }

  applyAnchorTransform();
}

function applyAnchorTransform()
{
  if (!state.view) return;

  if (state.sphereMode)
  {
    state.view.anchorGroup.position.copy(state.anchorFrame.up).multiplyScalar(SPHERE_R);
    state.view.anchorGroup.quaternion.copy(state.anchorFrame.q);
    state.view.globe.visible = true;
    state.view.anchorMarker.visible = true;
    state.view.ground.visible = false;
    state.view.grid.visible = false;
  }
  else
  {
    state.view.anchorGroup.position.set(0, 0, 0);
    state.view.anchorGroup.quaternion.identity();
    state.view.globe.visible = false;
    state.view.anchorMarker.visible = false;
    state.view.ground.visible = true;
    state.view.grid.visible = true;
  }
}

function updateSlamCenter()
{
  if (!state.view || state.view.pointCloud.count === 0) return;

  if (!state.slamCenterInitialized)
  {
    state.slamCenterSmooth.copy(state.slamCenter);
    state.slamCenterInitialized = true;
  }
  else
  {
    state.slamCenterSmooth.lerp(state.slamCenter, 0.2);
  }

  state.view.slamWorldGroup.position.copy(state.slamCenterSmooth).multiplyScalar(-1);
}

function pickAnchorFromClientXY(cx, cy)
{
  if (!state.view) return;

  const rect = dom.threeCanvas.getBoundingClientRect();
  const nx = ((cx - rect.left) / rect.width) * 2 - 1;
  const ny = -(((cy - rect.top) / rect.height) * 2 - 1);

  state.view.pointerNdc.set(nx, ny);
  state.view.raycaster.setFromCamera(state.view.pointerNdc, state.view.activeCamera);

  const hits = state.view.raycaster.intersectObject(state.view.globe, false);
  if (!hits || !hits.length)
  {
    setStatus('Pick: no hit. Tap the sphere wireframe.');
    return;
  }

  const p = hits[0].point.clone();
  if (p.lengthSq() < 1e-12) return;

  setAnchorFromUp(p.normalize());
  state.pickMode = false;
  setStatus('Anchor set by pick. Sphere mode: ON.');
}

function updateLayout()
{
  if (!state.view) return;

  const w = window.innerWidth;
  const h = window.innerHeight;

  state.view.renderer.setSize(w, h, false);
  state.view.mainCamera.aspect = w / h;
  state.view.mainCamera.updateProjectionMatrix();
  state.view.trackedCamera.aspect = w / h;
  state.view.trackedCamera.updateProjectionMatrix();
}

function updateFps(now)
{
  state.frames += 1;
  const delta = now - state.lastFpsSample;
  if (delta >= 1000)
  {
    state.fps = (state.frames * 1000) / delta;
    state.frames = 0;
    state.lastFpsSample = now;
  }
}

function updateUI(now)
{
  if (now - state.lastUiUpdate < 80) return;
  state.lastUiUpdate = now;

  dom.status.textContent = state.running
    ? (state.tracking ? 'Tracking' : 'Searching')
    : 'Idle';

  const imuOn = state.useIMU && state.imu;
  let imuText = imuOn ? 'IMU: on' : 'IMU: off';
  if (imuOn && typeof state.imu.heading === 'number')
  {
    imuText += ` | ${state.imu.heading.toFixed(1)}deg`;
  }
  dom.imuStatus.textContent = imuText;

  dom.fps.textContent = `FPS: ${state.fps ? state.fps.toFixed(1) : '-'}`;
  dom.pts2d.textContent = `2D: ${state.points2d || 0}`;
  const pts3d = state.view ? state.view.pointCloud.count : 0;
  dom.pts3d.textContent = `3D: ${pts3d || 0}`;
}

function renderCameraFrame(points)
{
  if (!state.cameraCtx || !state.media) return;

  const w = state.processWidth;
  const h = state.processHeight;

  state.cameraCtx.clearRect(0, 0, w, h);
  state.cameraCtx.drawImage(state.media.el, 0, 0, w, h);

  if (points && points.length)
  {
    state.cameraCtx.fillStyle = 'rgba(255, 255, 255, 0.9)';
    for (const p of points)
    {
      state.cameraCtx.fillRect(p.x - 1, p.y - 1, 2, 2);
    }
  }
}

function updatePhoneRig()
{
  if (!state.view || !state.imu) return;

  state.view.phoneRig.group.quaternion.copy(state.imu.qDisp);
  updateAccelArrows(state.view.phoneRig.accelArrows, state.imu.acc);
}

async function ensureIMU(enabled)
{
  if (!enabled)
  {
    state.useIMU = false;
    if (state.imu)
    {
      state.imu.stop();
      state.imu = null;
    }
    return;
  }

  try
  {
    state.imu = await IMU.Initialize();
    state.useIMU = true;
    setStatus('IMU fusion enabled');
  }
  catch (error)
  {
    state.useIMU = false;
    state.imu = null;
    dom.imuToggle.checked = false;
    setStatus(`IMU unavailable: ${error.message || error}`);
  }
}

async function setupPipeline()
{
  const video = state.media.el;
  const width = Math.max(1, video.videoWidth | 0);
  const height = Math.max(1, video.videoHeight | 0);

  state.processWidth = width;
  state.processHeight = height;

  if (!state.slamCanvas)
  {
    state.slamCanvas = document.createElement('canvas');
  }

  state.slamCanvas.width = width;
  state.slamCanvas.height = height;
  state.slamCtx = state.slamCanvas.getContext('2d', {
    alpha: false,
    desynchronized: true,
    willReadFrequently: true
  });

  dom.cameraCanvas.width = width;
  dom.cameraCanvas.height = height;
  state.cameraCtx = dom.cameraCanvas.getContext('2d', { alpha: false, desynchronized: true });

  state.alva = await AlvaAR.Initialize(width, height, DEFAULT_FOV);

  if (!state.view)
  {
    state.view = createSceneView();
  }

  applyAnchorTransform();
  updateLayout();
}

function resetTracking()
{
  if (!state.alva) return;

  state.alva.reset();
  state.tracking = false;

  if (state.view)
  {
    clearPointCloud(state.view.pointCloud);
    state.slamCenterInitialized = false;
    state.view.slamWorldGroup.position.set(0, 0, 0);
  }

  setStatus('Tracking reset');
}

async function startExperience()
{
  if (state.running) return;

  dom.startBtn.disabled = true;
  setStatus('Starting...');

  try
  {
    state.media = await Camera.Initialize(VIDEO_CONSTRAINTS);

    await setupPipeline();
    await ensureIMU(dom.imuToggle.checked);

    state.running = true;
    state.started = true;
    dom.stopBtn.disabled = false;
    dom.resetBtn.disabled = false;

    if (state.sphereMode)
    {
      tryAutoGpsAnchor();
    }

    onFrame(renderFrame, TARGET_FPS);
  }
  catch (error)
  {
    console.error(error);
    setStatus(`Start failed: ${error.message || error}`);
    dom.startBtn.disabled = false;
  }
}

function stopExperience()
{
  state.running = false;
  state.started = false;
  state.tracking = false;

  if (state.media)
  {
    state.media.stop();
    state.media = null;
  }

  if (state.imu)
  {
    state.imu.stop();
    state.imu = null;
  }

  dom.startBtn.disabled = false;
  dom.stopBtn.disabled = true;
  dom.resetBtn.disabled = true;
  setStatus('Stopped');
}

function toggleView()
{
  if (!state.view) return;

  if (state.view.activeCamera === state.view.mainCamera)
  {
    state.view.activeCamera = state.view.trackedCamera;
    dom.viewBtn.textContent = 'Main view';
  }
  else
  {
    state.view.activeCamera = state.view.mainCamera;
    dom.viewBtn.textContent = 'First-person';
  }
}

function toggleSphereMode()
{
  state.sphereMode = !state.sphereMode;
  dom.sphereBtn.textContent = state.sphereMode ? 'Back to plane' : 'Project to sphere';
  if (state.sphereMode)
  {
    setStatus('Sphere mode: ON. Up is radial.');
    if (state.anchorUp) setAnchorFromUp(state.anchorUp);
  }
  else
  {
    setStatus('Sphere mode: OFF (plane).');
  }
  applyAnchorTransform();
}

function tryAutoGpsAnchor()
{
  if (!('geolocation' in navigator)) return;

  navigator.geolocation.getCurrentPosition(
    (pos) =>
    {
      const c = pos.coords;
      setAnchorFromUp(latLonToUp(c.latitude, c.longitude));
      setStatus(`Anchor set from GPS: ${c.latitude.toFixed(4)}, ${c.longitude.toFixed(4)}`);
    },
    () => {},
    { enableHighAccuracy: true, maximumAge: 250, timeout: 15000 }
  );
}

function useGpsAnchor()
{
  if (!('geolocation' in navigator))
  {
    setStatus('GPS not available.');
    return;
  }

  navigator.geolocation.getCurrentPosition(
    (pos) =>
    {
      const c = pos.coords;
      setAnchorFromUp(latLonToUp(c.latitude, c.longitude));
      setStatus(`Anchor set from GPS: ${c.latitude.toFixed(4)}, ${c.longitude.toFixed(4)}`);
    },
    (err) =>
    {
      setStatus(`GPS error: ${err.message}`);
    },
    { enableHighAccuracy: true, maximumAge: 250, timeout: 15000 }
  );
}

function renderFrame(now)
{
  if (!state.running || !state.alva || !state.slamCtx)
  {
    return false;
  }

  if (state.resizing)
  {
    return true;
  }

  updateFps(now);

  if (!document.hidden)
  {
    state.slamCtx.clearRect(0, 0, state.processWidth, state.processHeight);
    state.slamCtx.drawImage(state.media.el, 0, 0, state.processWidth, state.processHeight);
    const frame = state.slamCtx.getImageData(0, 0, state.processWidth, state.processHeight);

    let pose = null;

    if (state.useIMU && state.imu)
    {
      pose = state.alva.findCameraPoseWithIMU(frame, state.imu.orientation, state.imu.motion);
    }
    else
    {
      pose = state.alva.findCameraPose(frame);
      if (state.imu)
      {
        state.imu.clear();
      }
    }

    const points2d = state.alva.getFramePoints();
    state.points2d = points2d ? points2d.length : 0;
    renderCameraFrame(points2d);

    if (pose && state.view)
    {
      state.tracking = true;
      applyPose(pose, state.view.trackedCamera.quaternion, state.view.trackedCamera.position);
      state.view.cameraHelper.update();
      state.view.cameraMarker.position.copy(state.view.trackedCamera.position);
      state.view.cameraMarker.quaternion.copy(state.view.trackedCamera.quaternion);

      appendPointCloudFrom2D(
        state.view.pointCloud,
        state.view.trackedCamera,
        points2d,
        state.processWidth,
        state.processHeight,
        BACKPROJECT_DEPTH,
        state.slamCenter
      );
      updateSlamCenter();
    }
    else
    {
      state.tracking = false;
    }

    if (state.view)
    {
      updatePhoneRig();
      state.view.renderer.render(state.view.scene, state.view.activeCamera);
    }
  }

  updateUI(now);
  return true;
}

window.addEventListener('resize', () =>
{
  clearTimeout(window.__slamResizeTimeout);
  window.__slamResizeTimeout = setTimeout(() =>
  {
    updateLayout();
  }, 250);
});

// ---------- UI wiring ----------

dom.startBtn.addEventListener('click', () => startExperience());
dom.stopBtn.addEventListener('click', () => stopExperience());
dom.resetBtn.addEventListener('click', () => resetTracking());
dom.viewBtn.addEventListener('click', () => toggleView());

dom.imuToggle.addEventListener('change', async (event) =>
{
  if (event.target.checked)
  {
    await ensureIMU(true);
  }
  else
  {
    await ensureIMU(false);
    setStatus('IMU fusion disabled');
  }
});

dom.sphereBtn.addEventListener('click', () => toggleSphereMode());

dom.pickBtn.addEventListener('click', () =>
{
  if (!state.sphereMode)
  {
    state.sphereMode = true;
    dom.sphereBtn.textContent = 'Back to plane';
  }
  state.pickMode = true;
  setStatus('Pick mode: tap the globe (or hold Shift and tap).');
  applyAnchorTransform();
});

dom.gpsBtn.addEventListener('click', () => useGpsAnchor());

dom.calBtn.addEventListener('click', () =>
{
  if (state.imu) state.imu.calibrateYawOnce();
});

dom.calResetBtn.addEventListener('click', () =>
{
  if (state.imu) state.imu.resetYawCalibration();
});

// Canvas pointer for picking

dom.threeCanvas.addEventListener('pointerdown', (e) =>
{
  if (state.sphereMode && (state.pickMode || e.shiftKey))
  {
    pickAnchorFromClientXY(e.clientX, e.clientY);
  }
});

// Initial UI text

dom.viewBtn.textContent = 'First-person';
dom.sphereBtn.textContent = state.sphereMode ? 'Back to plane' : 'Project to sphere';
setStatus('Idle');
