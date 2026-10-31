import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import type {
  DebugLayers,
  ScanMessage,
  SlamMessage,
  SlamPose,
  StateMessage,
  TrackInfo,
  Vec2,
} from '../types';

type CameraMode = 'orbit' | 'top';

/**
 * Three.js scene for the M0 demo. The scene is a pure renderer: it receives
 * track geometry, car state and lidar scans from the backend and never runs
 * physics or control logic.
 */
export class TrackScene {
  private readonly container: HTMLElement;
  private readonly scene = new THREE.Scene();
  private readonly camera: THREE.PerspectiveCamera;
  private readonly renderer: THREE.WebGLRenderer;
  private readonly controls: OrbitControls;
  private readonly clock = new THREE.Clock();

  private carGroup = new THREE.Group();
  private lidarPoints: THREE.Points | null = null;
  private lidarGeometry: THREE.BufferGeometry | null = null;
  private lidarPositions: Float32Array = new Float32Array(0);
  private lidarColors: Float32Array = new Float32Array(0);
  private trail: THREE.Line | null = null;
  private trailPositions: Float32Array = new Float32Array(300 * 3);
  private trailCount = 0;
  private slamGhost = new THREE.Group();
  private slamTrail: THREE.Line | null = null;
  private slamTrailPositions = new Float32Array(600 * 3);
  private slamTrailCount = 0;
  private slamMap: THREE.Mesh | null = null;
  private slamTexture: THREE.CanvasTexture | null = null;
  private slamCanvas: HTMLCanvasElement | null = null;
  private actualGhost = new THREE.Group();
  private latestState: StateMessage | null = null;
  private latestSlam: SlamMessage | null = null;
  private renderPose: 'actual' | 'controller' | 'slam' = 'actual';
  private odomGhost = new THREE.Group();
  private odomTrail: THREE.Line | null = null;
  private odomTrailPositions = new Float32Array(600 * 3);
  private odomTrailCount = 0;
  private mclGhost = new THREE.Group();
  private mclTrail: THREE.Line | null = null;
  private mclTrailPositions = new Float32Array(600 * 3);
  private mclTrailCount = 0;
  private mclParticles: THREE.Points | null = null;
  private mclParticleGeometry: THREE.BufferGeometry | null = null;
  private mclParticlePositions: Float32Array = new Float32Array(400 * 3);
  private gapMarker = new THREE.Group();
  private gapLine: THREE.Line | null = null;
  private gapDot: THREE.Mesh | null = null;
  private gapPositions: Float32Array = new Float32Array(6);
  private layers: DebugLayers = {
    map: true,
    slam: true,
    mcl: true,
    particles: true,
    odom: false,
    gtTrail: true,
    gap: true,
  };
  private walls: THREE.InstancedMesh | null = null;
  private startLine: THREE.Mesh | null = null;
  private centerline: THREE.Line | null = null;
  private surface: THREE.Mesh | null = null;
  private obstaclesGroup = new THREE.Group();

  private animationFrame = 0;
  private resizeObserver: ResizeObserver;
  private cameraMode: CameraMode = 'orbit';

  constructor(container: HTMLElement) {
    this.container = container;

    this.scene.background = new THREE.Color(0x080c11);
    this.scene.fog = new THREE.Fog(0x080c11, 18, 34);

    this.camera = new THREE.PerspectiveCamera(52, 1, 0.1, 100);
    this.camera.position.set(7.5, 7.5, 12.5);

    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(7.5, 0, 4);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.maxPolarAngle = Math.PI / 2.05;
    this.controls.minDistance = 3;
    this.controls.maxDistance = 28;

    const hemi = new THREE.HemisphereLight(0xdff1ff, 0x18222c, 1.4);
    this.scene.add(hemi);
    const key = new THREE.DirectionalLight(0xffffff, 2.2);
    key.position.set(4, 12, 8);
    this.scene.add(key);
    const rim = new THREE.DirectionalLight(0x6fd3ff, 0.8);
    rim.position.set(-6, 6, -4);
    this.scene.add(rim);

    this.buildFloor();
    this.buildCar();
    this.buildLidar();
    this.buildTrail();
    this.buildSlamGhost();
    this.buildSlamTrail();
    this.buildActualGhost();
    this.buildOdomGhost();
    this.buildMclGhost();
    this.buildMclTrail();
    this.buildMclParticles();
    this.buildGapMarker();

    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(container);
    this.resize();
    this.loop();
  }

  private buildFloor(): void {
    const geometry = new THREE.PlaneGeometry(15, 8);
    const material = new THREE.MeshStandardMaterial({
      color: 0x0d141b,
      roughness: 0.95,
      metalness: 0.0,
    });
    const floor = new THREE.Mesh(geometry, material);
    floor.rotation.x = -Math.PI / 2;
    floor.position.set(7.5, 0, 4);
    this.scene.add(floor);

    const grid = new THREE.GridHelper(16, 32, 0x1d3142, 0x101c26);
    grid.position.set(7.5, 0.002, 4);
    (grid.material as THREE.Material).opacity = 0.55;
    (grid.material as THREE.Material).transparent = true;
    this.scene.add(grid);
  }

  private buildCar(): void {
    const bodyMaterial = new THREE.MeshStandardMaterial({
      color: 0xff5d5d,
      roughness: 0.35,
      metalness: 0.25,
    });
    const body = new THREE.Mesh(new THREE.BoxGeometry(0.40, 0.07, 0.19), bodyMaterial);
    body.position.y = 0.10;
    this.carGroup.add(body);

    const cabin = new THREE.Mesh(
      new THREE.BoxGeometry(0.16, 0.06, 0.15),
      new THREE.MeshStandardMaterial({ color: 0x1a2430, roughness: 0.3, metalness: 0.4 }),
    );
    cabin.position.set(-0.03, 0.16, 0);
    this.carGroup.add(cabin);

    const nose = new THREE.Mesh(
      new THREE.BoxGeometry(0.03, 0.02, 0.13),
      new THREE.MeshStandardMaterial({ color: 0xf5f7fa, emissive: 0x223344 }),
    );
    nose.position.set(0.205, 0.11, 0);
    this.carGroup.add(nose);

    const wheelMaterial = new THREE.MeshStandardMaterial({
      color: 0x0a0d10,
      roughness: 0.9,
      metalness: 0.0,
    });
    const wheelGeometry = new THREE.CylinderGeometry(0.035, 0.035, 0.028, 14);
    for (const x of [-0.13, 0.13]) {
      for (const z of [-0.096, 0.096]) {
        const wheel = new THREE.Mesh(wheelGeometry, wheelMaterial);
        wheel.rotation.z = Math.PI / 2;
        wheel.position.set(x, 0.035, z);
        this.carGroup.add(wheel);
      }
    }

    const lidar = new THREE.Mesh(
      new THREE.CylinderGeometry(0.032, 0.032, 0.03, 16),
      new THREE.MeshStandardMaterial({
        color: 0x56f09f,
        emissive: 0x1b6b47,
        emissiveIntensity: 1.6,
        roughness: 0.4,
      }),
    );
    lidar.position.set(0.10, 0.185, 0);
    this.carGroup.add(lidar);

    this.scene.add(this.carGroup);
  }

  private buildLidar(): void {
    this.lidarGeometry = new THREE.BufferGeometry();
    this.lidarGeometry.setAttribute(
      'position',
      new THREE.BufferAttribute(this.lidarPositions, 3),
    );
    this.lidarGeometry.setAttribute('color', new THREE.BufferAttribute(this.lidarColors, 3));
    const material = new THREE.PointsMaterial({
      size: 0.036,
      vertexColors: true,
      sizeAttenuation: true,
      transparent: true,
      opacity: 0.95,
    });
    this.lidarPoints = new THREE.Points(this.lidarGeometry, material);
    this.lidarPoints.frustumCulled = false;
    this.scene.add(this.lidarPoints);
  }

  private buildSlamGhost(): void {
    const body = new THREE.Mesh(
      new THREE.BoxGeometry(0.42, 0.10, 0.21),
      new THREE.MeshBasicMaterial({
        color: 0x58a6ff,
        wireframe: true,
        transparent: true,
        opacity: 0.75,
      }),
    );
    body.position.y = 0.12;
    this.slamGhost.add(body);
    const ring = new THREE.Mesh(
      new THREE.RingGeometry(0.22, 0.25, 24),
      new THREE.MeshBasicMaterial({
        color: 0x58a6ff,
        transparent: true,
        opacity: 0.45,
        side: THREE.DoubleSide,
      }),
    );
    ring.rotation.x = -Math.PI / 2;
    ring.position.y = 0.02;
    this.slamGhost.add(ring);
    this.slamGhost.visible = false;
    this.scene.add(this.slamGhost);
  }

  private buildSlamTrail(): void {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(this.slamTrailPositions, 3));
    geometry.setDrawRange(0, 0);
    const material = new THREE.LineBasicMaterial({
      color: 0x58a6ff,
      transparent: true,
      opacity: 0.75,
    });
    this.slamTrail = new THREE.Line(geometry, material);
    this.slamTrail.frustumCulled = false;
    this.scene.add(this.slamTrail);
  }

  private buildActualGhost(): void {
    const body = new THREE.Mesh(
      new THREE.BoxGeometry(0.42, 0.10, 0.21),
      new THREE.MeshBasicMaterial({
        color: 0xffb454,
        wireframe: true,
        transparent: true,
        opacity: 0.65,
      }),
    );
    body.position.y = 0.12;
    this.actualGhost.add(body);
    this.actualGhost.visible = false;
    this.scene.add(this.actualGhost);
  }

  private buildOdomGhost(): void {
    const body = new THREE.Mesh(
      new THREE.BoxGeometry(0.42, 0.10, 0.21),
      new THREE.MeshBasicMaterial({
        color: 0x9aa7b4,
        wireframe: true,
        transparent: true,
        opacity: 0.5,
      }),
    );
    body.position.y = 0.12;
    this.odomGhost.add(body);
    this.odomGhost.visible = false;
    this.scene.add(this.odomGhost);

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(this.odomTrailPositions, 3));
    geometry.setDrawRange(0, 0);
    this.odomTrail = new THREE.Line(
      geometry,
      new THREE.LineBasicMaterial({ color: 0x9aa7b4, transparent: true, opacity: 0.5 }),
    );
    this.odomTrail.frustumCulled = false;
    this.odomTrail.visible = false;
    this.scene.add(this.odomTrail);
  }

  private buildMclGhost(): void {
    const body = new THREE.Mesh(
      new THREE.BoxGeometry(0.42, 0.10, 0.21),
      new THREE.MeshBasicMaterial({
        color: 0x56f09f,
        wireframe: true,
        transparent: true,
        opacity: 0.8,
      }),
    );
    body.position.y = 0.12;
    this.mclGhost.add(body);
    this.mclGhost.visible = false;
    this.scene.add(this.mclGhost);
  }

  private buildMclTrail(): void {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(this.mclTrailPositions, 3));
    geometry.setDrawRange(0, 0);
    this.mclTrail = new THREE.Line(
      geometry,
      new THREE.LineBasicMaterial({ color: 0x56f09f, transparent: true, opacity: 0.7 }),
    );
    this.mclTrail.frustumCulled = false;
    this.scene.add(this.mclTrail);
  }

  private buildMclParticles(): void {
    this.mclParticleGeometry = new THREE.BufferGeometry();
    this.mclParticleGeometry.setAttribute(
      'position',
      new THREE.BufferAttribute(this.mclParticlePositions, 3),
    );
    this.mclParticles = new THREE.Points(
      this.mclParticleGeometry,
      new THREE.PointsMaterial({
        color: 0x56f09f,
        size: 0.035,
        transparent: true,
        opacity: 0.55,
        sizeAttenuation: true,
      }),
    );
    this.mclParticles.frustumCulled = false;
    this.scene.add(this.mclParticles);
  }

  private buildGapMarker(): void {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(this.gapPositions, 3));
    geometry.setDrawRange(0, 2);
    this.gapLine = new THREE.Line(
      geometry,
      new THREE.LineBasicMaterial({
        color: 0xff5dd6,
        transparent: true,
        opacity: 0.95,
        depthTest: false,
      }),
    );
    this.gapLine.frustumCulled = false;
    this.gapLine.renderOrder = 12;
    this.gapMarker.add(this.gapLine);

    this.gapDot = new THREE.Mesh(
      new THREE.SphereGeometry(0.035, 12, 8),
      new THREE.MeshBasicMaterial({ color: 0xff5dd6, depthTest: false }),
    );
    this.gapDot.renderOrder = 13;
    this.gapMarker.add(this.gapDot);
    this.gapMarker.visible = false;
    this.scene.add(this.gapMarker);
  }

  private updateGapMarker(state: StateMessage): void {
    const gap = state.gap;
    const visible =
      state.auto_controller === 'gap_follow' &&
      this.layers.gap &&
      Boolean(gap?.valid) &&
      (gap?.range_m ?? 0) > 0;
    this.gapMarker.visible = visible;
    if (!visible || !gap) return;
    const range = Math.max(0.1, Math.min(gap.range_m, 8));
    const angle = state.car.theta + gap.angle_rad;
    const x = state.car.x + Math.cos(angle) * range;
    const z = state.car.y + Math.sin(angle) * range;
    this.gapPositions[0] = state.car.x;
    this.gapPositions[1] = 0.14;
    this.gapPositions[2] = state.car.y;
    this.gapPositions[3] = x;
    this.gapPositions[4] = 0.14;
    this.gapPositions[5] = z;
    const geometry = this.gapLine?.geometry;
    if (geometry) {
      geometry.setDrawRange(0, 2);
      geometry.attributes.position.needsUpdate = true;
    }
    this.gapDot?.position.set(x, 0.14, z);
  }

  private buildTrail(): void {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(this.trailPositions, 3));
    geometry.setDrawRange(0, 0);
    const material = new THREE.LineBasicMaterial({ color: 0xffb454, transparent: true, opacity: 0.8 });
    this.trail = new THREE.Line(geometry, material);
    this.trail.frustumCulled = false;
    this.scene.add(this.trail);
  }

  setTrack(track: TrackInfo): void {
    if (this.walls) {
      this.scene.remove(this.walls);
      this.walls.geometry.dispose();
      (this.walls.material as THREE.Material).dispose();
      this.walls = null;
    }
    if (this.centerline) {
      this.scene.remove(this.centerline);
      this.centerline.geometry.dispose();
      (this.centerline.material as THREE.Material).dispose();
      this.centerline = null;
    }
    if (this.startLine) {
      this.scene.remove(this.startLine);
      this.startLine.geometry.dispose();
      (this.startLine.material as THREE.Material).dispose();
      this.startLine = null;
    }
    if (this.surface) {
      this.scene.remove(this.surface);
      this.surface.geometry.dispose();
      (this.surface.material as THREE.Material).dispose();
      this.surface = null;
    }

    this.buildSurface(track);
    this.buildObstacles(track);
    const segments: Array<[Vec2, Vec2]> = [];
    for (const loop of [track.inner, track.outer]) {
      for (let i = 0; i < loop.length; i += 1) {
        segments.push([loop[i], loop[(i + 1) % loop.length]]);
      }
    }

    const geometry = new THREE.CylinderGeometry(track.wall_radius_m, track.wall_radius_m, 1, 8);
    const material = new THREE.MeshStandardMaterial({
      color: 0x73818d,
      roughness: 0.82,
      metalness: 0.08,
    });
    const walls = new THREE.InstancedMesh(geometry, material, segments.length);
    const matrix = new THREE.Matrix4();
    const quaternion = new THREE.Quaternion();
    const up = new THREE.Vector3(0, 1, 0);
    const position = new THREE.Vector3();
    const direction = new THREE.Vector3();
    const scale = new THREE.Vector3();

    segments.forEach(([a, b], index) => {
      const dx = b[0] - a[0];
      const dy = b[1] - a[1];
      const length = Math.hypot(dx, dy);
      position.set((a[0] + b[0]) / 2, track.wall_height_m / 2, (a[1] + b[1]) / 2);
      direction.set(dx, 0, dy).normalize();
      quaternion.setFromUnitVectors(up, direction);
      scale.set(1, Math.max(length, 1e-4), 1);
      matrix.compose(position, quaternion, scale);
      walls.setMatrixAt(index, matrix);
    });
    walls.instanceMatrix.needsUpdate = true;
    this.scene.add(walls);
    this.walls = walls;

    const centerPoints = track.centerline.map(
      ([x, y]) => new THREE.Vector3(x, 0.012, y),
    );
    const centerGeometry = new THREE.BufferGeometry().setFromPoints(centerPoints);
    const centerMaterial = new THREE.LineDashedMaterial({
      color: 0x2f81f7,
      dashSize: 0.18,
      gapSize: 0.12,
      transparent: true,
      opacity: 0.75,
    });
    this.centerline = new THREE.LineLoop(centerGeometry, centerMaterial);
    this.centerline.computeLineDistances();
    this.scene.add(this.centerline);

    this.buildStartLine(track);
  }

  private buildSurface(track: TrackInfo): void {
    const points = track.centerline;
    const count = points.length;
    const positions = new Float32Array(count * 2 * 3);
    const indices: number[] = [];
    const halfWidth = track.width_m / 2;
    for (let i = 0; i < count; i += 1) {
      const current = points[i];
      const next = points[(i + 1) % count];
      const dx = next[0] - current[0];
      const dy = next[1] - current[1];
      const length = Math.hypot(dx, dy) || 1;
      const nx = -dy / length;
      const ny = dx / length;
      const offset = i * 6;
      positions[offset] = current[0] + nx * halfWidth;
      positions[offset + 1] = 0.004;
      positions[offset + 2] = current[1] + ny * halfWidth;
      positions[offset + 3] = current[0] - nx * halfWidth;
      positions[offset + 4] = 0.004;
      positions[offset + 5] = current[1] - ny * halfWidth;
      const a = i * 2;
      const b = i * 2 + 1;
      const c = ((i + 1) % count) * 2;
      const d = c + 1;
      indices.push(a, b, c, b, d, c);
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.setIndex(indices);
    geometry.computeVertexNormals();
    const material = new THREE.MeshStandardMaterial({
      color: 0x10212d,
      roughness: 0.92,
      metalness: 0.03,
      side: THREE.DoubleSide,
    });
    this.surface = new THREE.Mesh(geometry, material);
    this.scene.add(this.surface);
  }

  private buildObstacles(track: TrackInfo): void {
    this.scene.remove(this.obstaclesGroup);
    this.obstaclesGroup.traverse((object) => {
      const mesh = object as THREE.Mesh;
      if (mesh.geometry) mesh.geometry.dispose();
      const material = (mesh as THREE.Mesh).material;
      if (Array.isArray(material)) material.forEach((item) => item.dispose());
      else if (material) material.dispose();
    });
    this.obstaclesGroup = new THREE.Group();
    for (const obstacle of track.obstacles ?? []) {
      const isBox = obstacle.kind === 'box';
      const geometry = isBox
        ? new THREE.BoxGeometry(obstacle.length, obstacle.height, obstacle.width)
        : new THREE.CylinderGeometry(obstacle.radius, obstacle.radius, obstacle.height, 12);
      const material = new THREE.MeshStandardMaterial({
        color: isBox ? 0xffb454 : 0x58a6ff,
        roughness: 0.55,
        metalness: 0.15,
        emissive: isBox ? 0x3a2408 : 0x0b2738,
        emissiveIntensity: 0.4,
      });
      const mesh = new THREE.Mesh(geometry, material);
      mesh.position.set(obstacle.x, obstacle.height / 2, obstacle.y);
      mesh.rotation.y = -obstacle.yaw;
      this.obstaclesGroup.add(mesh);
    }
    this.scene.add(this.obstaclesGroup);
  }

  private buildStartLine(track: TrackInfo): void {
    const points = sampleLoop(track.centerline, track.start_distance_m);
    const [x, y, theta] = points;
    const geometry = new THREE.BoxGeometry(track.width_m, 0.006, 0.08);
    const material = new THREE.MeshStandardMaterial({
      color: 0xf5f7fa,
      emissive: 0x3a4652,
      emissiveIntensity: 0.4,
    });
    this.startLine = new THREE.Mesh(geometry, material);
    this.startLine.position.set(x, 0.004, y);
    this.startLine.rotation.y = -theta;
    this.scene.add(this.startLine);
  }

  updateState(state: StateMessage): void {
    this.latestState = state;
    this.actualGhost.position.set(state.car.x, 0, state.car.y);
    this.actualGhost.rotation.y = -state.car.theta;
    this.applyRenderPose();

    const x = state.car.x;
    const y = state.car.y;
    const last = this.trailCount > 0 ? this.trailCount - 1 : -1;
    const lx = last >= 0 ? this.trailPositions[last * 3] : NaN;
    const lz = last >= 0 ? this.trailPositions[last * 3 + 2] : NaN;
    if (!Number.isFinite(lx) || Math.hypot(x - lx, y - lz) > 0.03) {
      if (this.trailCount >= this.trailPositions.length / 3) {
        this.trailPositions.copyWithin(0, 3);
        this.trailCount -= 1;
      }
      const index = this.trailCount * 3;
      this.trailPositions[index] = x;
      this.trailPositions[index + 1] = 0.02;
      this.trailPositions[index + 2] = y;
      this.trailCount += 1;
      const geometry = this.trail?.geometry;
      if (geometry) {
        geometry.setDrawRange(0, this.trailCount);
        geometry.attributes.position.needsUpdate = true;
      }
    }
    this.updateGapMarker(state);
  }

  updateScan(scan: ScanMessage, state: StateMessage): void {
    const count = scan.angles.length;
    if (!this.lidarGeometry || !this.lidarPoints) return;
    if (this.lidarPositions.length < count * 3) {
      this.lidarPositions = new Float32Array(count * 3);
      this.lidarColors = new Float32Array(count * 3);
      this.lidarGeometry.setAttribute(
        'position',
        new THREE.BufferAttribute(this.lidarPositions, 3),
      );
      this.lidarGeometry.setAttribute(
        'color',
        new THREE.BufferAttribute(this.lidarColors, 3),
      );
    }

    const near = new THREE.Color(0x56f09f);
    const far = new THREE.Color(0x2f81f7);
    const color = new THREE.Color();
    let visible = 0;
    for (let i = 0; i < count; i += 1) {
      const range = scan.ranges[i];
      if (range === null || !Number.isFinite(range) || range <= 0) continue;
      const angle = state.car.theta + scan.angles[i];
      const index = visible * 3;
      this.lidarPositions[index] = state.car.x + Math.cos(angle) * range;
      this.lidarPositions[index + 1] = 0.16;
      this.lidarPositions[index + 2] = state.car.y + Math.sin(angle) * range;
      const t = Math.min(1, Math.max(0, range / 8));
      color.copy(near).lerp(far, t);
      this.lidarColors[index] = color.r;
      this.lidarColors[index + 1] = color.g;
      this.lidarColors[index + 2] = color.b;
      visible += 1;
    }
    this.lidarGeometry.setDrawRange(0, visible);
    this.lidarGeometry.attributes.position.needsUpdate = true;
    this.lidarGeometry.attributes.color.needsUpdate = true;
  }

  updateSlam(slam: SlamMessage): void {
    this.latestSlam = slam;
    this.applyRenderPose();
    if (slam.pose) {
      this.slamGhost.visible = this.layers.slam;
      this.slamGhost.position.set(slam.pose.x, 0, slam.pose.y);
      this.slamGhost.rotation.y = -slam.pose.theta;
      this.appendTrail(
        this.slamTrailPositions,
        this.slamTrail,
        'slamTrailCount',
        slam.pose,
        this.layers.slam,
      );
    }
    if (slam.odometry) {
      this.odomGhost.visible = this.layers.odom;
      this.odomGhost.position.set(slam.odometry.x, 0, slam.odometry.y);
      this.odomGhost.rotation.y = -slam.odometry.theta;
      this.appendTrail(
        this.odomTrailPositions,
        this.odomTrail,
        'odomTrailCount',
        slam.odometry,
        this.layers.odom,
      );
    }
    if (slam.mcl?.pose) {
      this.mclGhost.visible = this.layers.mcl;
      this.mclGhost.position.set(slam.mcl.pose.x, 0, slam.mcl.pose.y);
      this.mclGhost.rotation.y = -slam.mcl.pose.theta;
      this.appendTrail(
        this.mclTrailPositions,
        this.mclTrail,
        'mclTrailCount',
        slam.mcl.pose,
        this.layers.mcl,
      );
      this.updateMclParticles(slam);
    } else {
      this.mclGhost.visible = false;
      if (this.mclTrail) this.mclTrail.visible = false;
      if (this.mclParticles) this.mclParticles.visible = false;
    }
    if (slam.map) {
      this.updateSlamMap(slam);
    }
  }

  private appendTrail(
    positions: Float32Array,
    line: THREE.Line | null,
    countField: 'slamTrailCount' | 'odomTrailCount' | 'mclTrailCount',
    pose: SlamPose,
    visible: boolean,
  ): void {
    if (line) line.visible = visible;
    if (!visible) return;
    const count = this[countField];
    const last = count > 0 ? count - 1 : -1;
    const lx = last >= 0 ? positions[last * 3] : NaN;
    const lz = last >= 0 ? positions[last * 3 + 2] : NaN;
    if (Number.isFinite(lx) && Math.hypot(pose.x - lx, pose.y - lz) <= 0.04) return;
    if (count >= positions.length / 3) {
      positions.copyWithin(0, 3);
      this[countField] = count - 1;
    }
    const index = this[countField] * 3;
    positions[index] = pose.x;
    positions[index + 1] = 0.025;
    positions[index + 2] = pose.y;
    this[countField] += 1;
    const geometry = line?.geometry;
    if (geometry) {
      geometry.setDrawRange(0, this[countField]);
      geometry.attributes.position.needsUpdate = true;
    }
  }

  private updateMclParticles(slam: SlamMessage): void {
    if (!this.mclParticleGeometry || !this.mclParticles) return;
    const particles = slam.mcl?.particles ?? [];
    const max = this.mclParticlePositions.length / 3;
    const count = Math.min(particles.length, max);
    for (let i = 0; i < count; i += 1) {
      this.mclParticlePositions[i * 3] = particles[i][0];
      this.mclParticlePositions[i * 3 + 1] = 0.04;
      this.mclParticlePositions[i * 3 + 2] = particles[i][1];
    }
    this.mclParticleGeometry.setDrawRange(0, count);
    this.mclParticleGeometry.attributes.position.needsUpdate = true;
    this.mclParticles.visible = this.layers.mcl && this.layers.particles && count > 0;
  }

  setLayerVisibility(layers: DebugLayers): void {
    this.layers = { ...layers };
    this.slamGhost.visible = layers.slam;
    if (this.slamTrail) this.slamTrail.visible = layers.slam;
    this.odomGhost.visible = layers.odom;
    if (this.odomTrail) this.odomTrail.visible = layers.odom;
    this.mclGhost.visible = layers.mcl;
    if (this.mclTrail) this.mclTrail.visible = layers.mcl;
    if (this.mclParticles) {
      this.mclParticles.visible =
        layers.mcl && layers.particles && this.mclParticleGeometry?.drawRange.count !== 0;
    }
    if (this.slamMap) this.slamMap.visible = layers.map;
    if (this.trail) this.trail.visible = layers.gtTrail;
    if (this.latestState) this.updateGapMarker(this.latestState);
  }

  private updateSlamMap(slam: SlamMessage): void {
    const map = slam.map;
    if (!map) return;
    const canvas = this.slamCanvas ?? document.createElement('canvas');
    canvas.width = map.width;
    canvas.height = map.height;
    const context = canvas.getContext('2d');
    if (!context) return;
    const image = context.createImageData(map.width, map.height);
    const bytes = Uint8Array.from(atob(map.data), (char) => char.charCodeAt(0));
    for (let i = 0; i < bytes.length; i += 1) {
      const value = bytes[i];
      const offset = i * 4;
      if (value === 100) {
        image.data[offset] = 255;
        image.data[offset + 1] = 180;
        image.data[offset + 2] = 84;
        image.data[offset + 3] = 220;
      } else if (value === 0) {
        image.data[offset] = 40;
        image.data[offset + 1] = 110;
        image.data[offset + 2] = 170;
        image.data[offset + 3] = 38;
      } else {
        image.data[offset + 3] = 0;
      }
    }
    context.putImageData(image, 0, 0);
    this.slamCanvas = canvas;

    if (!this.slamTexture) {
      this.slamTexture = new THREE.CanvasTexture(canvas);
      this.slamTexture.flipY = false;
      this.slamTexture.colorSpace = THREE.SRGBColorSpace;
    } else {
      this.slamTexture.needsUpdate = true;
    }

    const width = map.width * map.resolution;
    const height = map.height * map.resolution;
    if (this.slamMap) {
      this.scene.remove(this.slamMap);
      this.slamMap.geometry.dispose();
      (this.slamMap.material as THREE.Material).dispose();
    }
    const geometry = new THREE.PlaneGeometry(width, height);
    const material = new THREE.MeshBasicMaterial({
      map: this.slamTexture,
      transparent: true,
      opacity: 0.9,
      side: THREE.DoubleSide,
      depthWrite: false,
    });
    this.slamMap = new THREE.Mesh(geometry, material);
    this.slamMap.rotation.x = Math.PI / 2;
    this.slamMap.position.set(
      map.origin_x + width / 2,
      0.007,
      map.origin_y + height / 2,
    );
    this.slamMap.visible = this.layers.map;
    this.scene.add(this.slamMap);
  }

  setRenderPose(mode: 'actual' | 'controller' | 'slam'): void {
    this.renderPose = mode;
    this.applyRenderPose();
  }

  private applyRenderPose(): void {
    const state = this.latestState;
    if (!state) return;
    let x = state.car.x;
    let y = state.car.y;
    let theta = state.car.theta;
    if (this.renderPose === 'slam' && this.latestSlam?.pose) {
      x = this.latestSlam.pose.x;
      y = this.latestSlam.pose.y;
      theta = this.latestSlam.pose.theta;
    } else if (this.renderPose === 'controller') {
      const mode = state.localization_mode ?? 'gt';
      if (mode === 'mcl' && this.latestSlam?.mcl?.pose) {
        x = this.latestSlam.mcl.pose.x;
        y = this.latestSlam.mcl.pose.y;
        theta = this.latestSlam.mcl.pose.theta;
      } else if (mode === 'odom' && this.latestSlam?.odometry) {
        x = this.latestSlam.odometry.x;
        y = this.latestSlam.odometry.y;
        theta = this.latestSlam.odometry.theta;
      }
    }
    this.carGroup.position.set(x, 0, y);
    this.carGroup.rotation.y = -theta;
    this.actualGhost.visible = this.renderPose !== 'actual';
  }

  setCamera(mode: CameraMode): void {
    this.cameraMode = mode;
    if (mode === 'top') {
      this.camera.position.set(7.5, 15, 4.02);
      this.controls.target.set(7.5, 0, 4);
    } else {
      this.camera.position.set(7.5, 7.5, 12.5);
      this.controls.target.set(7.5, 0, 4);
    }
    this.controls.update();
  }

  getCameraMode(): CameraMode {
    return this.cameraMode;
  }

  private resize(): void {
    const width = this.container.clientWidth;
    const height = this.container.clientHeight;
    if (width === 0 || height === 0) return;
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }

  private loop = (): void => {
    this.animationFrame = requestAnimationFrame(this.loop);
    const delta = this.clock.getDelta();
    this.controls.update();
    if (delta > 0) {
      // Reserved for animated sensor effects; the scene is static otherwise.
    }
    this.renderer.render(this.scene, this.camera);
  };

  dispose(): void {
    cancelAnimationFrame(this.animationFrame);
    this.resizeObserver.disconnect();
    this.controls.dispose();
    this.renderer.dispose();
    this.renderer.domElement.remove();
    this.scene.traverse((object) => {
      const mesh = object as THREE.Mesh;
      if (mesh.geometry) mesh.geometry.dispose();
      const material = (mesh as THREE.Mesh).material;
      if (Array.isArray(material)) {
        material.forEach((item) => item.dispose());
      } else if (material) {
        material.dispose();
      }
    });
  }
}

function sampleLoop(points: Vec2[], distance: number): [number, number, number] {
  if (points.length === 0) return [0, 0, 0];
  const lengths: number[] = [];
  let total = 0;
  for (let i = 0; i < points.length; i += 1) {
    const a = points[i];
    const b = points[(i + 1) % points.length];
    const length = Math.hypot(b[0] - a[0], b[1] - a[1]);
    lengths.push(length);
    total += length;
  }
  let target = ((distance % total) + total) % total;
  for (let i = 0; i < points.length; i += 1) {
    if (target <= lengths[i]) {
      const a = points[i];
      const b = points[(i + 1) % points.length];
      const t = target / Math.max(lengths[i], 1e-9);
      const x = a[0] + (b[0] - a[0]) * t;
      const y = a[1] + (b[1] - a[1]) * t;
      const theta = Math.atan2(b[1] - a[1], b[0] - a[0]);
      return [x, y, theta];
    }
    target -= lengths[i];
  }
  return [points[0][0], points[0][1], 0];
}
