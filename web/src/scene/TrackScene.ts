import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import type { ScanMessage, StateMessage, TrackInfo, Vec2 } from '../types';

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
  private walls: THREE.InstancedMesh | null = null;
  private startLine: THREE.Mesh | null = null;
  private centerline: THREE.Line | null = null;
  private surface: THREE.Mesh | null = null;

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
    this.carGroup.position.set(state.car.x, 0, state.car.y);
    this.carGroup.rotation.y = -state.car.theta;

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
