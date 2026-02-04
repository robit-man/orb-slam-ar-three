import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { AlvaAR } from './alva_ar.js';
import { AlvaARConnectorTHREE } from './alva_ar_three.js';

const DEFAULT_FOV = 60;
const TARGET_FPS = 30;
const MAX_POINT_CAP = 12000;
const BACKPROJECT_DEPTH = 2.0;

const VIDEO_CONSTRAINTS = {
  video: {
    facingMode: 'environment',
    aspectRatio: 16 / 9,
    width: { ideal: 1280 }
  },
  audio: false
};

const dom = {
  start: document.getElementById('start'),
  reset: document.getElementById('reset'),
  imuToggle: document.getElementById('imu-toggle'),
  status: document.getElementById('status'),
  stats: document.getElementById('stats'),
  cameraCanvas: document.getElementById('camera-canvas'),
  mapPanel: document.getElementById('map-panel'),
  mapThree: document.getElementById('map-three')
};

const applyPose = AlvaARConnectorTHREE.Initialize(THREE);

const state = {
  started: false,
  resizing: false,
  tracking: false,
  useIMU: true,
  alva: null,
  imu: null,
  media: null,
  slamCanvas: null,
  slamCtx: null,
  processWidth: 0,
  processHeight: 0,
  views: null
};

class StatsTimer
{
  constructor(bufferSize = 30)
  {
    this.t = 0;
    this.delta = 0;
    this.avg = new Array(bufferSize).fill(0);
    this.idx = 0;
  }

  start()
  {
    this.t = performance.now();
  }

  stop()
  {
    this.delta = ~~((performance.now() - this.t) + 0.5);
    this.avg[this.idx] = this.delta;
    this.idx = (this.idx + 1) % this.avg.length;
  }

  reset()
  {
    this.delta = 0;
  }

  getElapsedAverageTime()
  {
    return ~~(this.avg.reduce((acc, v) => acc + (v / this.avg.length), 0) + 0.5);
  }
}

class Stats
{
  constructor()
  {
    this.fps = 0.0;
    this.frame = 0;
    this.timers = [];
    this.timer = new StatsTimer();
    this.buffer = [];
    this.bufferSize = 16;

    this.mbInfoAvailable = (performance && performance.memory && performance.memory.totalJSHeapSize);
    this.mbSize = Math.pow(1000, 2);
    this.mb = 0;

    this.el = document.createElement('div');
    this.el.className = 'stats-box';
  }

  add(taskName)
  {
    this.timers.push([taskName, new StatsTimer()]);
  }

  next()
  {
    ++this.frame;

    this.timers.forEach(o => o[1].reset());

    if (this.frame > 0)
    {
      this.timer.stop();
      this.buffer.push(this.timer.delta);

      if (this.buffer.length > this.bufferSize)
      {
        this.buffer.shift();
      }

      const sum = this.buffer.reduce((acc, v) => acc + v, 0);
      this.fps = this.buffer.length / sum * 1000;
      this.timer.start();
    }

    if (this.mbInfoAvailable)
    {
      this.mb = performance.memory.usedJSHeapSize;
    }
  }

  start(taskName)
  {
    const entry = this.timers.find(pair => (pair[0] === taskName));
    if (entry) entry[1].start();
  }

  stop(taskName)
  {
    const entry = this.timers.find(pair => (pair[0] === taskName));
    if (entry) entry[1].stop();
  }

  render(info = '')
  {
    let str = `FPS: ${~~this.fps}`;

    this.timers.forEach(o => str += `\n${o[0]} : ${o[1].getElapsedAverageTime()}ms`);

    if (this.mbInfoAvailable)
    {
      str += `\nMemory: ${(this.mb / this.mbSize).toFixed(1)}MB`;
    }

    if (info)
    {
      str += `\n${info}`;
    }

    this.el.textContent = str;
  }
}

const stats = new Stats();
dom.stats.appendChild(stats.el);
stats.add('total');
stats.add('video');
stats.add('slam');

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

              resolve(new Camera(video));
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

  constructor(videoElement)
  {
    this.el = videoElement;
    this.width = videoElement.videoWidth;
    this.height = videoElement.videoHeight;
  }
}

const deg2rad = Math.PI / 180;

class Quaternion
{
  static fromAxisAngle(axisX = 0, axisY = 0, axisZ = 0, angle = 0)
  {
    const angle2 = angle / 2;
    const s = Math.sin(angle2);

    return {
      x: axisX * s,
      y: axisY * s,
      z: axisZ * s,
      w: Math.cos(angle2)
    };
  }

  static fromEuler(x = 0, y = 0, z = 0, order = 'XYZ')
  {
    const cos = Math.cos;
    const sin = Math.sin;

    const c1 = cos(x / 2);
    const c2 = cos(y / 2);
    const c3 = cos(z / 2);

    const s1 = sin(x / 2);
    const s2 = sin(y / 2);
    const s3 = sin(z / 2);

    const q = { x: 0, y: 0, z: 0, w: 1 };

    switch (order)
    {
      case 'XYZ':
        q.x = s1 * c2 * c3 + c1 * s2 * s3;
        q.y = c1 * s2 * c3 - s1 * c2 * s3;
        q.z = c1 * c2 * s3 + s1 * s2 * c3;
        q.w = c1 * c2 * c3 - s1 * s2 * s3;
        break;
      case 'YXZ':
        q.x = s1 * c2 * c3 + c1 * s2 * s3;
        q.y = c1 * s2 * c3 - s1 * c2 * s3;
        q.z = c1 * c2 * s3 - s1 * s2 * c3;
        q.w = c1 * c2 * c3 + s1 * s2 * s3;
        break;
      case 'ZXY':
        q.x = s1 * c2 * c3 - c1 * s2 * s3;
        q.y = c1 * s2 * c3 + s1 * c2 * s3;
        q.z = c1 * c2 * s3 + s1 * s2 * c3;
        q.w = c1 * c2 * c3 - s1 * s2 * s3;
        break;
      case 'ZYX':
        q.x = s1 * c2 * c3 - c1 * s2 * s3;
        q.y = c1 * s2 * c3 + s1 * c2 * s3;
        q.z = c1 * c2 * s3 - s1 * s2 * c3;
        q.w = c1 * c2 * c3 + s1 * s2 * s3;
        break;
      case 'YZX':
        q.x = s1 * c2 * c3 + c1 * s2 * s3;
        q.y = c1 * s2 * c3 + s1 * c2 * s3;
        q.z = c1 * c2 * s3 - s1 * s2 * c3;
        q.w = c1 * c2 * c3 + s1 * s2 * s3;
        break;
      case 'XZY':
        q.x = s1 * c2 * c3 - c1 * s2 * s3;
        q.y = c1 * s2 * c3 - s1 * c2 * s3;
        q.z = c1 * c2 * s3 + s1 * s2 * c3;
        q.w = c1 * c2 * c3 + s1 * s2 * s3;
        break;
      default:
        console.warn('CreateFromEuler() encountered an unknown order: ' + order);
    }

    return q;
  }

  static multiply(a, b)
  {
    const qax = a.x, qay = a.y, qaz = a.z, qaw = a.w;
    const qbx = b.x, qby = b.y, qbz = b.z, qbw = b.w;

    return {
      x: qax * qbw + qaw * qbx + qay * qbz - qaz * qby,
      y: qay * qbw + qaw * qby + qaz * qbx - qax * qbz,
      z: qaz * qbw + qaw * qbz + qax * qby - qay * qbx,
      w: qaw * qbw - qax * qbx - qay * qby - qaz * qbz
    };
  }

  static dot(a, b)
  {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
  }
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
    this.EPS = 0.000001;
    this.motion = [];

    this.orientation = { x: 1, y: 0, z: 0, w: 1 };
    this.worldTransform = /iPad|iPhone|iPod/.test(navigator.platform)
      ? Quaternion.fromAxisAngle(1, 0, 0, -Math.PI / 2)
      : Quaternion.fromAxisAngle(0, 1, 0, Math.PI / 2);

    const handleDeviceOrientation = (event) =>
    {
      if (event.beta === null || event.gamma === null || event.alpha === null)
      {
        return;
      }

      const x = event.beta * deg2rad;
      const y = event.gamma * deg2rad;
      const z = event.alpha * deg2rad;

      const orientation = Quaternion.multiply(this.worldTransform, Quaternion.fromEuler(x, y, z, 'ZXY'));

      if (8 * (1 - Quaternion.dot(this.orientation, orientation)) > this.EPS)
      {
        this.orientation = orientation;
      }
    };

    const handleDeviceMotion = (event) =>
    {
      const rate = event.rotationRate || {};
      const accel = event.acceleration || event.accelerationIncludingGravity || {};

      const gx = (rate.beta || 0) * deg2rad;
      const gy = (rate.gamma || 0) * deg2rad;
      const gz = (rate.alpha || 0) * deg2rad;

      const ax = accel.x || 0;
      const ay = accel.y || 0;
      const az = accel.z || 0;

      const timestamp = Date.now();

      this.motion.push({ timestamp, gx, gy, gz, ax, ay, az });

      if (this.motion.length > 120)
      {
        this.motion.shift();
      }
    };

    window.addEventListener('devicemotion', handleDeviceMotion.bind(this), false);
    window.addEventListener('deviceorientation', handleDeviceOrientation.bind(this), false);
  }

  clear()
  {
    this.motion.length = 0;
  }
}

function setStatus(text)
{
  dom.status.textContent = text;
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
    vCamPos: new THREE.Vector3(),
    vRay: new THREE.Vector3(),
    vTarget: new THREE.Vector3()
  };
}

function clearPointCloud(pc)
{
  pc.geom.setDrawRange(0, 0);
  pc.geom.attributes.position.needsUpdate = true;
}

function updatePointCloudFrom2D(pc, camera, points2d, width, height, depth)
{
  if (!points2d || !points2d.length)
  {
    clearPointCloud(pc);
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

    const j = n * 3;
    pc.positions[j + 0] = pc.vTarget.x;
    pc.positions[j + 1] = pc.vTarget.y;
    pc.positions[j + 2] = pc.vTarget.z;
    n++;
  }

  pc.geom.setDrawRange(0, n);
  pc.geom.attributes.position.needsUpdate = true;
  return n;
}

class ThirdPersonView
{
  constructor(container, width, height, fov)
  {
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setClearColor(new THREE.Color('rgb(12, 16, 24)'));
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setSize(width, height, false);

    this.camera = new THREE.PerspectiveCamera(55, width / height, 0.01, 2000);
    this.camera.position.set(-2, 2, 2);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.minDistance = 0.2;
    this.controls.maxDistance = 50;

    this.trackedCamera = new THREE.PerspectiveCamera(fov, width / height, 0.01, 1000);
    this.cameraHelper = new THREE.CameraHelper(this.trackedCamera);

    this.scene = new THREE.Scene();
    this.scene.add(new THREE.AmbientLight(0xb0b0b0));
    this.scene.add(new THREE.HemisphereLight(0x404040, 0xf0f0f0, 1));
    this.scene.add(new THREE.GridHelper(10, 20));
    this.scene.add(new THREE.AxesHelper(0.5));
    this.scene.add(this.cameraHelper);

    this.pointCloud = createPointCloud(MAX_POINT_CAP);
    this.scene.add(this.pointCloud.points);
    this.container.appendChild(this.renderer.domElement);
  }

  setSize(width, height)
  {
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  }

  setFov(fov)
  {
    this.trackedCamera.fov = fov;
    this.trackedCamera.updateProjectionMatrix();
  }

  updatePose(pose)
  {
    applyPose(pose, this.trackedCamera.quaternion, this.trackedCamera.position);
    this.cameraHelper.update();
  }

  setPointCloudVisible(visible)
  {
    this.pointCloud.points.visible = visible;
  }

  updatePointCloud(points2d, width, height, depth)
  {
    return updatePointCloudFrom2D(this.pointCloud, this.trackedCamera, points2d, width, height, depth);
  }

  clearPointCloud()
  {
    clearPointCloud(this.pointCloud);
  }

  render()
  {
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}

class CameraFrameView
{
  constructor(canvas)
  {
    this.canvas = canvas;
    this.ctx = this.canvas.getContext('2d', { alpha: false, desynchronized: true });
    this.width = 0;
    this.height = 0;
  }

  setSize(width, height)
  {
    this.width = width;
    this.height = height;
    this.canvas.width = width;
    this.canvas.height = height;
  }

  render(source, points)
  {
    if (!source || !this.width || !this.height)
    {
      return;
    }

    this.ctx.clearRect(0, 0, this.width, this.height);
    this.ctx.drawImage(source, 0, 0, this.width, this.height);

    if (points && points.length)
    {
      this.ctx.fillStyle = 'rgba(255, 255, 255, 0.9)';
      for (const p of points)
      {
        this.ctx.fillRect(p.x - 1, p.y - 1, 2, 2);
      }
    }
  }
}

async function ensureIMU(enabled)
{
  if (!enabled)
  {
    state.useIMU = false;
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

function updateLayout()
{
  if (!state.views)
  {
    return;
  }

  const mapRect = dom.mapPanel.getBoundingClientRect();
  state.views.map.setSize(Math.max(1, Math.round(mapRect.width)), Math.max(1, Math.round(mapRect.height)));
  state.views.map.setFov(DEFAULT_FOV);
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

  state.alva = await AlvaAR.Initialize(width, height, DEFAULT_FOV);

  if (!state.views)
  {
    state.views = {
      map: new ThirdPersonView(dom.mapThree, 1, 1, DEFAULT_FOV),
      camera: new CameraFrameView(dom.cameraCanvas)
    };
  }

  state.views.camera.setSize(width, height);
  updateLayout();
}

function resetTracking()
{
  if (!state.alva)
  {
    return;
  }

  state.alva.reset();
  state.tracking = false;

  if (state.views)
  {
    state.views.map.clearPointCloud();
  }

  setStatus('Tracking reset');
}

async function startExperience()
{
  if (state.started)
  {
    return;
  }

  dom.start.disabled = true;
  setStatus('Starting...');

  try
  {
    state.media = await Camera.Initialize(VIDEO_CONSTRAINTS);

    await ensureIMU(dom.imuToggle.checked);
    await setupPipeline();

    dom.reset.disabled = false;
    state.started = true;

    onFrame(renderFrame, TARGET_FPS);
  }
  catch (error)
  {
    console.error(error);
    setStatus(`Start failed: ${error.message || error}`);
    dom.start.disabled = false;
  }
}

async function handleImuToggle(event)
{
  if (event.target.checked)
  {
    await ensureIMU(true);
  }
  else
  {
    state.useIMU = false;
    if (state.imu)
    {
      state.imu.clear();
    }
    setStatus('IMU fusion disabled');
  }
}

async function handleResize()
{
  if (!state.started || state.resizing)
  {
    return;
  }

  state.resizing = true;
  updateLayout();

  state.resizing = false;
}

function renderFrame()
{
  if (!state.started || !state.alva || !state.slamCtx)
  {
    return false;
  }

  if (state.resizing)
  {
    return true;
  }

  stats.next();
  stats.start('total');

  if (!document.hidden)
  {
    stats.start('video');
    state.slamCtx.clearRect(0, 0, state.processWidth, state.processHeight);
    state.slamCtx.drawImage(state.media.el, 0, 0, state.processWidth, state.processHeight);
    const frame = state.slamCtx.getImageData(0, 0, state.processWidth, state.processHeight);
    stats.stop('video');

    stats.start('slam');
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

    stats.stop('slam');

    const points2d = state.alva.getFramePoints();
    state.views.camera.render(state.slamCanvas, points2d);

    if (pose)
    {
      state.tracking = true;
      state.views.map.updatePose(pose);
      state.views.map.updatePointCloud(points2d, state.processWidth, state.processHeight, BACKPROJECT_DEPTH);
    }
    else
    {
      state.tracking = false;
      state.views.map.clearPointCloud();
    }
    state.views.map.render();
  }

  stats.stop('total');
  stats.render(`${state.tracking ? 'Tracking' : 'Searching'} | ${state.useIMU ? 'IMU' : 'Vision'}`);
  setStatus(state.tracking ? 'Tracking' : 'Tracking lost');

  return true;
}

window.addEventListener('resize', () =>
{
  clearTimeout(window.__slamResizeTimeout);
  window.__slamResizeTimeout = setTimeout(() =>
  {
    handleResize().catch(console.error);
  }, 250);
});

dom.start.addEventListener('click', () => startExperience());
dom.reset.addEventListener('click', () => resetTracking());
dom.imuToggle.addEventListener('change', handleImuToggle);
