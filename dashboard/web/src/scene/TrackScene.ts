import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import type {
  BodyPart,
  CameraMode,
  DebugLayers,
  HelloMessage,
  ScanMessage,
  SensorsSpec,
  StateMessage,
  TrackInfo,
  Vec2,
  VehicleSpec,
  ViewMode,
} from '../types';

interface CarModel {
  group: THREE.Group;
  frontPivots: THREE.Group[];
  sensorGroup: THREE.Group;
  coverageGroup: THREE.Group;
  labelGroup: THREE.Group;
}

const TRACK_BACKGROUND = 0x191b1f;
const STUDIO_BACKGROUND = 0x202329;

function physicalPosition(values: [number, number, number]): THREE.Vector3 {
  return new THREE.Vector3(values[0], values[2], values[1]);
}

function deg(value: number): number {
  return (value * Math.PI) / 180;
}

function makeLabel(text: string, color = '#aab1bb'): THREE.Sprite {
  const canvas = document.createElement('canvas');
  canvas.width = 256;
  canvas.height = 64;
  const context = canvas.getContext('2d');
  if (context) {
    context.clearRect(0, 0, canvas.width, canvas.height);
    context.font = '600 22px Consolas, monospace';
    context.textAlign = 'center';
    context.textBaseline = 'middle';
    context.fillStyle = 'rgba(18, 20, 24, 0.78)';
    context.fillRect(20, 12, 216, 40);
    context.fillStyle = color;
    context.fillText(text, 128, 33);
  }
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const material = new THREE.SpriteMaterial({
    map: texture,
    transparent: true,
    depthTest: false,
    depthWrite: false,
  });
  const sprite = new THREE.Sprite(material);
  sprite.scale.set(0.12, 0.03, 1);
  sprite.renderOrder = 20;
  return sprite;
}

export class TrackScene {
  private readonly container: HTMLElement;
  private readonly scene = new THREE.Scene();
  private readonly camera: THREE.PerspectiveCamera;
  private readonly renderer: THREE.WebGLRenderer;
  private readonly controls: OrbitControls;
  private readonly clock = new THREE.Clock();

  private readonly trackGroup = new THREE.Group();
  private readonly studioGroup = new THREE.Group();
  private trackCar: CarModel | null = null;
  private studioCar: CarModel | null = null;
  private vehicle: VehicleSpec | null = null;
  private sensors: SensorsSpec | null = null;

  private lidarPoints: THREE.Points | null = null;
  private lidarGeometry: THREE.BufferGeometry | null = null;
  private lidarPositions = new Float32Array(0);
  private lidarColors = new Float32Array(0);
  private trail: THREE.Line | null = null;
  private trailPositions = new Float32Array(1800 * 3);
  private trailCount = 0;
  private odomTrail: THREE.Line | null = null;
  private odomTrailPositions = new Float32Array(1800 * 3);
  private odomTrailCount = 0;

  private centerline: THREE.Line | null = null;
  private obstaclesGroup = new THREE.Group();
  private studioGrid: THREE.GridHelper | null = null;
  private studioLidarRing: THREE.Mesh | null = null;

  private gapGroup = new THREE.Group();
  private gapLine: THREE.Line | null = null;
  private gapFan: THREE.Mesh | null = null;
  private gapDot: THREE.Mesh | null = null;
  private readonly gapFanPositions = new Float32Array((33 + 1) * 3);

  private latestState: StateMessage | null = null;
  private lidarHeight = 0.14;
  private viewMode: ViewMode = 'track';
  private cameraMode: CameraMode = 'orbit';
  private layers: DebugLayers = {
    scan: true,
    trail: true,
    odometry: false,
    sensors: false,
    coverage: false,
    labels: true,
    gap: true,
  };

  private animationFrame = 0;
  private readonly resizeObserver: ResizeObserver;
  private turntable = false;
  private wheelSpin = 0;

  constructor(container: HTMLElement) {
    this.container = container;
    this.scene.background = new THREE.Color(TRACK_BACKGROUND);
    this.scene.fog = new THREE.Fog(TRACK_BACKGROUND, 20, 38);

    this.camera = new THREE.PerspectiveCamera(50, 1, 0.03, 100);
    this.camera.position.set(7.5, 7.5, 13.5);

    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(7.5, 0, 4);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.maxPolarAngle = Math.PI / 2.03;
    this.controls.minDistance = 2.5;
    this.controls.maxDistance = 30;
    this.controls.autoRotateSpeed = 0.75;

    const hemisphere = new THREE.HemisphereLight(0xdde6ef, 0x1a1d22, 1.15);
    this.scene.add(hemisphere);
    const key = new THREE.DirectionalLight(0xffffff, 2.1);
    key.position.set(3, 10, 5);
    this.scene.add(key);
    const fill = new THREE.DirectionalLight(0x9db7cf, 0.65);
    fill.position.set(-5, 4, -3);
    this.scene.add(fill);
    const studioKey = new THREE.DirectionalLight(0xfff4e2, 2.6);
    studioKey.position.set(1.0, 1.4, 0.8);
    this.scene.add(studioKey);
    const rim = new THREE.DirectionalLight(0x79a7c9, 1.1);
    rim.position.set(-0.8, 0.8, -1.0);
    this.scene.add(rim);

    this.scene.add(this.trackGroup);
    this.scene.add(this.studioGroup);
    this.buildStudio();
    this.buildLidar();
    this.buildTrail();
    this.buildOdomTrail();
    this.buildGapMarker();

    this.studioGroup.visible = false;
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(container);
    this.resize();
    this.loop();
  }

  // --------------------------------------------------------------- materials
  private materialForPart(part: BodyPart): THREE.MeshStandardMaterial {
    return new THREE.MeshStandardMaterial({
      color: new THREE.Color(part.color),
      roughness: 0.62,
      metalness: part.shape === 'box' ? 0.16 : 0.3,
      transparent: part.opacity < 1,
      opacity: part.opacity,
    });
  }

  private addPart(group: THREE.Group, part: BodyPart): void {
    if (!part.visible) return;
    let geometry: THREE.BufferGeometry;
    if (part.shape === 'cylinder') {
      geometry = new THREE.CylinderGeometry(
        part.size_m[0] / 2,
        part.size_m[0] / 2,
        part.size_m[2],
        24,
      );
    } else if (part.shape === 'sphere') {
      geometry = new THREE.SphereGeometry(part.size_m[0] / 2, 24, 14);
    } else {
      geometry = new THREE.BoxGeometry(part.size_m[0], part.size_m[2], part.size_m[1]);
    }
    const mesh = new THREE.Mesh(geometry, this.materialForPart(part));
    mesh.position.copy(physicalPosition(part.position_m));
    mesh.rotation.set(deg(part.rpy_deg[0]), -deg(part.rpy_deg[2]), -deg(part.rpy_deg[1]));
    group.add(mesh);
  }

  // ------------------------------------------------------------------- car
  private buildCarModel(): CarModel {
    const group = new THREE.Group();
    const sensorGroup = new THREE.Group();
    const coverageGroup = new THREE.Group();
    const labelGroup = new THREE.Group();
    const frontPivots: THREE.Group[] = [];

    const vehicle = this.vehicle;
    const sensors = this.sensors;
    if (!vehicle || !sensors) {
      return { group, frontPivots, sensorGroup, coverageGroup, labelGroup };
    }

    group.position.y = vehicle.ride_height_m;
    for (const part of vehicle.structure) this.addPart(group, part);

    const tyreMaterial = new THREE.MeshStandardMaterial({
      color: 0x14161a,
      roughness: 0.88,
      metalness: 0.02,
    });
    const hubMaterial = new THREE.MeshStandardMaterial({
      color: 0x8b929b,
      roughness: 0.35,
      metalness: 0.75,
    });
    for (const wheel of vehicle.wheels) {
      const pivot = new THREE.Group();
      pivot.position.copy(physicalPosition(wheel.position_m));
      const wheelGroup = new THREE.Group();
      const tyre = new THREE.Mesh(
        new THREE.CylinderGeometry(wheel.radius_m, wheel.radius_m, wheel.width_m, 22),
        tyreMaterial,
      );
      tyre.rotation.x = Math.PI / 2;
      wheelGroup.add(tyre);
      if (wheel.steered) {
        const hub = new THREE.Mesh(
          new THREE.CylinderGeometry(
            wheel.radius_m * 0.52,
            wheel.radius_m * 0.52,
            wheel.width_m + 0.004,
            12,
          ),
          hubMaterial,
        );
        hub.rotation.x = Math.PI / 2;
        wheelGroup.add(hub);
      }
      pivot.add(wheelGroup);
      group.add(pivot);
      if (wheel.steered) frontPivots.push(pivot);
    }

    for (const mount of sensors.mounts) {
      const marker = this.buildSensorMarker(mount.kind, mount.position_m, mount.rpy_deg);
      if (marker) sensorGroup.add(marker);
      const coverage = this.buildSensorCoverage(
        mount.kind,
        mount.position_m,
        mount.rpy_deg,
        mount.range_m,
        mount.fov_deg,
      );
      if (coverage) coverageGroup.add(coverage);
      const label = makeLabel(mount.id === 'lidar' ? 'LIDAR' : mount.id, mount.color);
      label.position.copy(physicalPosition(mount.position_m));
      label.position.y += 0.055;
      labelGroup.add(label);
    }

    group.add(sensorGroup, coverageGroup, labelGroup);
    return { group, frontPivots, sensorGroup, coverageGroup, labelGroup };
  }

  private buildSensorMarker(
    kind: 'lidar' | 'ultrasonic' | 'imu',
    position: [number, number, number],
    rpy: [number, number, number],
  ): THREE.Group | null {
    const group = new THREE.Group();
    group.position.copy(physicalPosition(position));
    group.rotation.y = -deg(rpy[2]);
    if (kind === 'lidar') {
      const body = new THREE.Mesh(
        new THREE.CylinderGeometry(0.023, 0.024, 0.024, 20),
        new THREE.MeshStandardMaterial({
          color: 0x2e343c,
          roughness: 0.4,
          metalness: 0.55,
        }),
      );
      const band = new THREE.Mesh(
        new THREE.CylinderGeometry(0.0245, 0.0245, 0.007, 20),
        new THREE.MeshStandardMaterial({
          color: 0xd7a84b,
          emissive: 0x35240a,
          emissiveIntensity: 1.2,
          roughness: 0.35,
        }),
      );
      band.position.y = 0.004;
      group.add(body, band);
      const ring = new THREE.Mesh(
        new THREE.RingGeometry(0.285, 0.288, 72),
        new THREE.MeshBasicMaterial({
          color: 0xd7a84b,
          transparent: true,
          opacity: 0.32,
          side: THREE.DoubleSide,
          depthWrite: false,
        }),
      );
      ring.rotation.x = -Math.PI / 2;
      ring.position.y = -0.045;
      group.add(ring);
    } else if (kind === 'ultrasonic') {
      const caseMesh = new THREE.Mesh(
        new THREE.BoxGeometry(0.022, 0.014, 0.03),
        new THREE.MeshStandardMaterial({
          color: 0x2f353d,
          roughness: 0.5,
          metalness: 0.3,
        }),
      );
      const face = new THREE.Mesh(
        new THREE.CircleGeometry(0.008, 16),
        new THREE.MeshStandardMaterial({
          color: 0x5f9fc7,
          emissive: 0x112a3a,
          emissiveIntensity: 1.0,
          roughness: 0.4,
        }),
      );
      face.rotation.y = Math.PI / 2;
      face.position.set(0.012, 0, 0);
      group.add(caseMesh, face);
    } else {
      const imu = new THREE.Mesh(
        new THREE.BoxGeometry(0.024, 0.008, 0.016),
        new THREE.MeshStandardMaterial({
          color: 0x262a31,
          roughness: 0.55,
          metalness: 0.25,
        }),
      );
      const stripe = new THREE.Mesh(
        new THREE.BoxGeometry(0.004, 0.0085, 0.0165),
        new THREE.MeshStandardMaterial({ color: 0x9a86c7, roughness: 0.4 }),
      );
      stripe.position.x = 0.007;
      group.add(imu, stripe);
    }
    return group;
  }

  private buildSensorCoverage(
    kind: 'lidar' | 'ultrasonic' | 'imu',
    position: [number, number, number],
    rpy: [number, number, number],
    range: number,
    fovDeg: number,
  ): THREE.Object3D | null {
    if (kind === 'lidar') {
      const group = new THREE.Group();
      for (const radius of [0.32, 0.62]) {
        const ring = new THREE.Mesh(
          new THREE.RingGeometry(radius, radius + 0.003, 96),
          new THREE.MeshBasicMaterial({
            color: 0xd7a84b,
            transparent: true,
            opacity: 0.20,
            side: THREE.DoubleSide,
            depthWrite: false,
          }),
        );
        ring.rotation.x = -Math.PI / 2;
        group.add(ring);
      }
      group.position.set(position[0], 0.006, position[1]);
      return group;
    }
    if (kind === 'ultrasonic') {
      const radius = Math.min(0.55, Math.max(0.18, range * 0.1));
      const half = deg(fovDeg / 2);
      const length = radius;
      const points: number[] = [0, 0, 0, length, 0, Math.tan(half) * length, length, 0, -Math.tan(half) * length];
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.Float32BufferAttribute(points, 3));
      geometry.setIndex([0, 1, 2]);
      const mesh = new THREE.Mesh(
        geometry,
        new THREE.MeshBasicMaterial({
          color: 0x5f9fc7,
          transparent: true,
          opacity: 0.16,
          side: THREE.DoubleSide,
          depthWrite: false,
        }),
      );
      mesh.position.set(position[0], 0.007, position[1]);
      mesh.rotation.y = -deg(rpy[2]);
      return mesh;
    }
    return null;
  }

  private buildStudio(): void {
    const floor = new THREE.Mesh(
      new THREE.CircleGeometry(1.9, 64),
      new THREE.MeshStandardMaterial({
        color: 0x272a30,
        roughness: 0.9,
        metalness: 0.02,
      }),
    );
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = -0.006;
    this.studioGroup.add(floor);

    const platform = new THREE.Mesh(
      new THREE.CylinderGeometry(0.52, 0.55, 0.014, 64),
      new THREE.MeshStandardMaterial({
        color: 0x30343b,
        roughness: 0.6,
        metalness: 0.2,
      }),
    );
    platform.position.y = 0.002;
    this.studioGroup.add(platform);

    this.studioGrid = new THREE.GridHelper(3.6, 36, 0x3a4048, 0x2c3036);
    this.studioGrid.position.y = 0.004;
    const gridMaterial = this.studioGrid.material as THREE.Material;
    gridMaterial.transparent = true;
    gridMaterial.opacity = 0.45;
    this.studioGroup.add(this.studioGrid);

    this.studioLidarRing = new THREE.Mesh(
      new THREE.RingGeometry(0.77, 0.775, 96),
      new THREE.MeshBasicMaterial({
        color: 0xd7a84b,
        transparent: true,
        opacity: 0.22,
        side: THREE.DoubleSide,
        depthWrite: false,
      }),
    );
    this.studioLidarRing.rotation.x = -Math.PI / 2;
    this.studioLidarRing.position.y = 0.006;
    this.studioGroup.add(this.studioLidarRing);
  }

  setCar(hello: HelloMessage): void {
    this.vehicle = hello.vehicle;
    this.sensors = hello.sensors;
    const lidar = hello.sensors.mounts.find((mount) => mount.kind === 'lidar');
    this.lidarHeight = lidar ? lidar.ground_height_m : hello.vehicle.ride_height_m + 0.05;

    if (this.trackCar) {
      this.scene.remove(this.trackCar.group);
      this.disposeObject(this.trackCar.group);
    }
    if (this.studioCar) {
      this.studioGroup.remove(this.studioCar.group);
      this.disposeObject(this.studioCar.group);
    }
    this.trackCar = this.buildCarModel();
    this.studioCar = this.buildCarModel();
    this.trackCar.group.visible = this.viewMode === 'track';
    this.studioCar.group.visible = this.viewMode === 'car';
    this.scene.add(this.trackCar.group);
    this.studioGroup.add(this.studioCar.group);
    this.applyLayers();
  }

  setTrack(track: TrackInfo): void {
    for (const child of [...this.trackGroup.children]) {
      this.trackGroup.remove(child);
      this.disposeObject(child);
    }
    this.centerline = null;
    this.obstaclesGroup = new THREE.Group();

    this.buildFloor();
    this.buildSurface(track);
    this.buildWalls(track);
    this.buildCenterline(track);
    this.buildStartLine(track);
    this.trackGroup.add(this.obstaclesGroup);
    this.buildObstacles(track);
  }

  private buildFloor(): void {
    const floor = new THREE.Mesh(
      new THREE.PlaneGeometry(15, 8),
      new THREE.MeshStandardMaterial({
        color: 0x1c1f24,
        roughness: 0.96,
        metalness: 0.0,
      }),
    );
    floor.rotation.x = -Math.PI / 2;
    floor.position.set(7.5, 0, 4);
    this.trackGroup.add(floor);

    const grid = new THREE.GridHelper(16, 32, 0x30363e, 0x252a31);
    grid.position.set(7.5, 0.002, 4);
    const material = grid.material as THREE.Material;
    material.opacity = 0.55;
    material.transparent = true;
    this.trackGroup.add(grid);
  }

  private buildSurface(track: TrackInfo): void {
    const points = track.centerline;
    const count = points.length;
    const positions = new Float32Array(count * 2 * 3);
    const indices: number[] = [];
    const halfWidth = track.width_m / 2;
    for (let index = 0; index < count; index += 1) {
      const current = points[index];
      const next = points[(index + 1) % count];
      const dx = next[0] - current[0];
      const dy = next[1] - current[1];
      const length = Math.hypot(dx, dy) || 1;
      const nx = -dy / length;
      const ny = dx / length;
      const offset = index * 6;
      positions[offset] = current[0] + nx * halfWidth;
      positions[offset + 1] = 0.004;
      positions[offset + 2] = current[1] + ny * halfWidth;
      positions[offset + 3] = current[0] - nx * halfWidth;
      positions[offset + 4] = 0.004;
      positions[offset + 5] = current[1] - ny * halfWidth;
      const a = index * 2;
      const b = index * 2 + 1;
      const c = ((index + 1) % count) * 2;
      const d = c + 1;
      indices.push(a, b, c, b, d, c);
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.setIndex(indices);
    geometry.computeVertexNormals();
    const material = new THREE.MeshStandardMaterial({
      color: 0x252a31,
      roughness: 0.95,
      metalness: 0.02,
      side: THREE.DoubleSide,
    });
    const surface = new THREE.Mesh(geometry, material);
    this.trackGroup.add(surface);
  }
  private buildWalls(track: TrackInfo): void {
    const segments: Array<[Vec2, Vec2]> = [];
    for (const loop of [track.inner, track.outer]) {
      for (let index = 0; index < loop.length; index += 1) {
        segments.push([loop[index], loop[(index + 1) % loop.length]]);
      }
    }
    const geometry = new THREE.CylinderGeometry(track.wall_radius_m, track.wall_radius_m, 1, 8);
    const material = new THREE.MeshStandardMaterial({
      color: 0x5a646e,
      roughness: 0.72,
      metalness: 0.25,
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
    this.trackGroup.add(walls);
  }

  private buildCenterline(track: TrackInfo): void {
    const points = track.centerline.map(([x, y]) => new THREE.Vector3(x, 0.012, y));
    const geometry = new THREE.BufferGeometry().setFromPoints(points);
    const material = new THREE.LineDashedMaterial({
      color: 0x546a7e,
      dashSize: 0.2,
      gapSize: 0.16,
      transparent: true,
      opacity: 0.75,
    });
    this.centerline = new THREE.LineLoop(geometry, material);
    this.centerline.computeLineDistances();
    this.trackGroup.add(this.centerline);
  }

  private buildStartLine(track: TrackInfo): void {
    const [x, y, theta] = sampleLoop(track.centerline, track.start_distance_m);
    const geometry = new THREE.BoxGeometry(track.width_m, 0.006, 0.08);
    const material = new THREE.MeshStandardMaterial({
      color: 0xd9dde2,
      roughness: 0.65,
      metalness: 0.1,
    });
    const startLine = new THREE.Mesh(geometry, material);
    startLine.position.set(x, 0.005, y);
    startLine.rotation.y = -theta;
    this.trackGroup.add(startLine);
  }

  private buildObstacles(track: TrackInfo): void {
    for (const obstacle of track.obstacles) {
      const isBox = obstacle.kind === 'box';
      const geometry = isBox
        ? new THREE.BoxGeometry(obstacle.length, obstacle.height, obstacle.width)
        : new THREE.CylinderGeometry(obstacle.radius, obstacle.radius, obstacle.height, 16);
      const material = new THREE.MeshStandardMaterial({
        color: isBox ? 0x8c6d43 : 0x4b6170,
        roughness: 0.75,
        metalness: 0.12,
      });
      const mesh = new THREE.Mesh(geometry, material);
      mesh.position.set(obstacle.x, obstacle.height / 2, obstacle.y);
      mesh.rotation.y = -obstacle.yaw;
      this.obstaclesGroup.add(mesh);
    }
  }

  private buildLidar(): void {
    this.lidarGeometry = new THREE.BufferGeometry();
    this.lidarGeometry.setAttribute(
      'position',
      new THREE.BufferAttribute(this.lidarPositions, 3),
    );
    this.lidarGeometry.setAttribute('color', new THREE.BufferAttribute(this.lidarColors, 3));
    const material = new THREE.PointsMaterial({
      size: 0.028,
      vertexColors: true,
      sizeAttenuation: true,
      transparent: true,
      opacity: 0.92,
    });
    this.lidarPoints = new THREE.Points(this.lidarGeometry, material);
    this.lidarPoints.frustumCulled = false;
    this.scene.add(this.lidarPoints);
  }

  private buildTrail(): void {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(this.trailPositions, 3));
    geometry.setDrawRange(0, 0);
    this.trail = new THREE.Line(
      geometry,
      new THREE.LineBasicMaterial({
        color: 0xd7a84b,
        transparent: true,
        opacity: 0.7,
      }),
    );
    this.trail.frustumCulled = false;
    this.scene.add(this.trail);
  }

  private buildOdomTrail(): void {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(this.odomTrailPositions, 3));
    geometry.setDrawRange(0, 0);
    this.odomTrail = new THREE.Line(
      geometry,
      new THREE.LineBasicMaterial({
        color: 0x7b8794,
        transparent: true,
        opacity: 0.55,
      }),
    );
    this.odomTrail.frustumCulled = false;
    this.odomTrail.visible = false;
    this.scene.add(this.odomTrail);
  }

  private buildGapMarker(): void {
    const fanGeometry = new THREE.BufferGeometry();
    fanGeometry.setAttribute(
      'position',
      new THREE.BufferAttribute(this.gapFanPositions, 3),
    );
    const indices: number[] = [];
    for (let index = 0; index < 33; index += 1) {
      indices.push(0, index + 1, index + 2);
    }
    fanGeometry.setIndex(indices);
    this.gapFan = new THREE.Mesh(
      fanGeometry,
      new THREE.MeshBasicMaterial({
        color: 0xd7a84b,
        transparent: true,
        opacity: 0.13,
        side: THREE.DoubleSide,
        depthWrite: false,
      }),
    );
    this.gapFan.renderOrder = 4;
    this.gapGroup.add(this.gapFan);

    const lineGeometry = new THREE.BufferGeometry();
    lineGeometry.setAttribute(
      'position',
      new THREE.BufferAttribute(new Float32Array(6), 3),
    );
    this.gapLine = new THREE.Line(
      lineGeometry,
      new THREE.LineBasicMaterial({
        color: 0xe7a85b,
        transparent: true,
        opacity: 0.95,
        depthTest: false,
      }),
    );
    this.gapLine.renderOrder = 5;
    this.gapGroup.add(this.gapLine);

    this.gapDot = new THREE.Mesh(
      new THREE.SphereGeometry(0.032, 12, 8),
      new THREE.MeshBasicMaterial({ color: 0xe7a85b, depthTest: false }),
    );
    this.gapDot.renderOrder = 6;
    this.gapGroup.add(this.gapDot);
    this.gapGroup.visible = false;
    this.scene.add(this.gapGroup);
  }

  private updateGapMarker(state: StateMessage): void {
    const gap = state.gap;
    const visible =
      this.viewMode === 'track' &&
      state.auto_controller === 'gap_follow' &&
      gap.valid &&
      this.layers.gap;
    this.gapGroup.visible = visible;
    if (!visible) return;
    const range = Math.max(0.35, Math.min(gap.range_m, 3.0));
    const y = 0.035;
    const startX = state.car.x;
    const startZ = state.car.y;
    const arcCount = 33;
    this.gapFanPositions[0] = startX;
    this.gapFanPositions[1] = y;
    this.gapFanPositions[2] = startZ;
    for (let index = 0; index <= arcCount; index += 1) {
      const fraction = index / arcCount;
      const angle =
        state.car.theta +
        gap.gap_left_rad +
        (gap.gap_right_rad - gap.gap_left_rad) * fraction;
      const offset = (index + 1) * 3;
      this.gapFanPositions[offset] = startX + Math.cos(angle) * range;
      this.gapFanPositions[offset + 1] = y;
      this.gapFanPositions[offset + 2] = startZ + Math.sin(angle) * range;
    }
    if (this.gapFan) {
      this.gapFan.geometry.attributes.position.needsUpdate = true;
      this.gapFan.geometry.computeBoundingSphere();
    }
    const chosen = state.car.theta + gap.angle_rad;
    const targetX = startX + Math.cos(chosen) * range;
    const targetZ = startZ + Math.sin(chosen) * range;
    if (this.gapLine) {
      const positions = this.gapLine.geometry.attributes.position as THREE.BufferAttribute;
      positions.setXYZ(0, startX, y + 0.002, startZ);
      positions.setXYZ(1, targetX, y + 0.002, targetZ);
      positions.needsUpdate = true;
      this.gapLine.geometry.computeBoundingSphere();
    }
    this.gapDot?.position.set(targetX, y + 0.002, targetZ);
  }

  updateState(state: StateMessage): void {
    this.latestState = state;
    this.updateGapMarker(state);
    const heading = state.car.theta;
    const wheelSpin = state.car.speed * 0.5;

    if (this.trackCar) {
      this.trackCar.group.position.set(state.car.x, this.vehicle?.ride_height_m ?? 0.09, state.car.y);
      this.trackCar.group.rotation.y = -heading;
      for (const pivot of this.trackCar.frontPivots) {
        pivot.rotation.y = -state.car.steering;
      }
      this.trackCar.group.traverse((object) => {
        if (object.userData.wheel) {
          this.wheelSpin += wheelSpin * 0.02;
          object.rotation.z = this.wheelSpin;
        }
      });
    }
    if (this.studioCar) {
      for (const pivot of this.studioCar.frontPivots) {
        pivot.rotation.y = -state.car.steering;
      }
    }

    this.appendTrail(
      this.trailPositions,
      this.trail,
      'trailCount',
      state.car.x,
      state.car.y,
      0.012,
      this.layers.trail,
    );
    this.appendTrail(
      this.odomTrailPositions,
      this.odomTrail,
      'odomTrailCount',
      state.odometry.x,
      state.odometry.y,
      0.014,
      this.layers.odometry,
    );
    if (this.cameraMode === 'chase' && this.viewMode === 'track') {
      this.applyChaseCamera(state);
    }
  }

  updateScan(scan: ScanMessage, state: StateMessage): void {
    if (!this.lidarGeometry || !this.lidarPoints) return;
    const count = scan.angles.length;
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
    const near = new THREE.Color(0xd7a84b);
    const far = new THREE.Color(0x6f8499);
    const color = new THREE.Color();
    let visible = 0;
    for (let index = 0; index < count; index += 1) {
      const range = scan.ranges[index];
      if (range === null || !Number.isFinite(range) || range <= 0) continue;
      const angle = state.car.theta + scan.angles[index];
      const offset = visible * 3;
      this.lidarPositions[offset] = state.car.x + Math.cos(angle) * range;
      this.lidarPositions[offset + 1] = this.lidarHeight;
      this.lidarPositions[offset + 2] = state.car.y + Math.sin(angle) * range;
      const t = Math.min(1, Math.max(0, range / 8));
      color.copy(near).lerp(far, t);
      this.lidarColors[offset] = color.r;
      this.lidarColors[offset + 1] = color.g;
      this.lidarColors[offset + 2] = color.b;
      visible += 1;
    }
    this.lidarGeometry.setDrawRange(0, visible);
    this.lidarGeometry.attributes.position.needsUpdate = true;
    this.lidarGeometry.attributes.color.needsUpdate = true;
    this.applyLayers();
  }

  private appendTrail(
    positions: Float32Array,
    line: THREE.Line | null,
    countField: 'trailCount' | 'odomTrailCount',
    x: number,
    y: number,
    height: number,
    visible: boolean,
  ): void {
    if (!line) return;
    line.visible = visible && this.viewMode === 'track';
    if (!visible) return;
    const count = this[countField];
    const last = count > 0 ? count - 1 : -1;
    const lastX = last >= 0 ? positions[last * 3] : NaN;
    const lastY = last >= 0 ? positions[last * 3 + 2] : NaN;
    if (Number.isFinite(lastX) && Math.hypot(x - lastX, y - lastY) <= 0.02) return;
    if (count >= positions.length / 3) {
      positions.copyWithin(0, 3);
      this[countField] = count - 1;
    }
    const index = this[countField] * 3;
    positions[index] = x;
    positions[index + 1] = height;
    positions[index + 2] = y;
    this[countField] += 1;
    const geometry = line.geometry;
    geometry.setDrawRange(0, this[countField]);
    geometry.attributes.position.needsUpdate = true;
  }

  setViewMode(mode: ViewMode): void {
    this.viewMode = mode;
    const carMode = mode === 'car';
    this.trackGroup.visible = !carMode;
    this.studioGroup.visible = carMode;
    if (this.trackCar) this.trackCar.group.visible = !carMode;
    if (this.studioCar) this.studioCar.group.visible = carMode;
    if (this.lidarPoints) this.lidarPoints.visible = !carMode && this.layers.scan;
    if (this.trail) this.trail.visible = !carMode && this.layers.trail;
    if (this.odomTrail) this.odomTrail.visible = !carMode && this.layers.odometry;
    if (this.centerline) this.centerline.visible = !carMode;
    if (carMode) this.gapGroup.visible = false;
    this.scene.background = new THREE.Color(carMode ? STUDIO_BACKGROUND : TRACK_BACKGROUND);
    this.scene.fog = carMode ? null : new THREE.Fog(TRACK_BACKGROUND, 20, 38);
    this.controls.autoRotate = carMode && this.turntable;
    this.applyCameraForView();
    this.applyLayers();
  }

  setCamera(mode: CameraMode): void {
    this.cameraMode = mode;
    this.applyCameraForView();
    if (this.latestState) this.applyChaseCamera(this.latestState);
  }

  getCameraMode(): CameraMode {
    return this.cameraMode;
  }

  setTurntable(enabled: boolean): void {
    this.turntable = enabled;
    this.controls.autoRotate = enabled && this.viewMode === 'car';
  }

  setLayers(layers: DebugLayers): void {
    this.layers = { ...layers };
    this.applyLayers();
  }

  private applyLayers(): void {
    const carMode = this.viewMode === 'car';
    if (this.trackCar) {
      this.trackCar.sensorGroup.visible = this.layers.sensors;
      this.trackCar.coverageGroup.visible = this.layers.coverage;
      this.trackCar.labelGroup.visible = this.layers.labels && this.layers.sensors;
    }
    if (this.studioCar) {
      this.studioCar.sensorGroup.visible = this.layers.sensors;
      this.studioCar.coverageGroup.visible = this.layers.coverage;
      this.studioCar.labelGroup.visible = this.layers.labels && this.layers.sensors;
    }
    if (this.lidarPoints) this.lidarPoints.visible = this.layers.scan && !carMode;
    if (this.trail) this.trail.visible = this.layers.trail && !carMode;
    if (this.odomTrail) this.odomTrail.visible = this.layers.odometry && !carMode;
    if (this.studioLidarRing) this.studioLidarRing.visible = this.layers.coverage;
    if (this.latestState) this.updateGapMarker(this.latestState);
    else this.gapGroup.visible = false;
  }

  private applyCameraForView(): void {
    const carMode = this.viewMode === 'car';
    if (carMode) {
      this.controls.minDistance = 0.28;
      this.controls.maxDistance = 2.2;
      if (this.cameraMode === 'top') {
        this.camera.position.set(0, 1.25, 0.001);
        this.controls.target.set(0, 0.08, 0);
      } else {
        this.camera.position.set(0.52, 0.39, 0.68);
        this.controls.target.set(0, 0.09, 0);
      }
      this.controls.autoRotate = this.turntable;
    } else {
      this.controls.autoRotate = false;
      this.controls.minDistance = 2.5;
      this.controls.maxDistance = 30;
      if (this.cameraMode === 'top') {
        this.camera.position.set(7.5, 15, 4.02);
        this.controls.target.set(7.5, 0, 4);
      } else if (this.cameraMode === 'chase') {
        if (this.latestState) this.applyChaseCamera(this.latestState);
      } else {
        this.camera.position.set(7.5, 7.5, 13.5);
        this.controls.target.set(7.5, 0, 4);
      }
    }
    this.controls.update();
  }

  private applyChaseCamera(state: StateMessage): void {
    const carY = this.vehicle?.ride_height_m ?? 0.09;
    const behind = 0.95;
    const x = state.car.x - Math.cos(state.car.theta) * behind;
    const z = state.car.y - Math.sin(state.car.theta) * behind;
    this.camera.position.set(x, carY + 0.52, z);
    this.controls.target.set(state.car.x, carY, state.car.y);
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
    this.clock.getDelta();
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  };

  private disposeObject(root: THREE.Object3D): void {
    root.traverse((object) => {
      const mesh = object as THREE.Mesh;
      if (mesh.geometry) mesh.geometry.dispose();
      const material = mesh.material as THREE.Material | THREE.Material[] | undefined;
      if (Array.isArray(material)) {
        for (const item of material) {
          if ((item as THREE.MeshBasicMaterial).map) {
            (item as THREE.MeshBasicMaterial).map?.dispose();
          }
          item.dispose();
        }
      } else if (material) {
        const textured = material as THREE.MeshBasicMaterial;
        textured.map?.dispose();
        material.dispose();
      }
    });
  }

  dispose(): void {
    cancelAnimationFrame(this.animationFrame);
    this.resizeObserver.disconnect();
    this.controls.dispose();
    this.renderer.dispose();
    this.renderer.domElement.remove();
    this.disposeObject(this.scene);
  }
}

function sampleLoop(points: Vec2[], distance: number): [number, number, number] {
  if (points.length === 0) return [0, 0, 0];
  const lengths: number[] = [];
  let total = 0;
  for (let index = 0; index < points.length; index += 1) {
    const a = points[index];
    const b = points[(index + 1) % points.length];
    const length = Math.hypot(b[0] - a[0], b[1] - a[1]);
    lengths.push(length);
    total += length;
  }
  let target = ((distance % total) + total) % total;
  for (let index = 0; index < points.length; index += 1) {
    if (target <= lengths[index]) {
      const a = points[index];
      const b = points[(index + 1) % points.length];
      const t = target / Math.max(lengths[index], 1e-9);
      const x = a[0] + (b[0] - a[0]) * t;
      const y = a[1] + (b[1] - a[1]) * t;
      const theta = Math.atan2(b[1] - a[1], b[0] - a[0]);
      return [x, y, theta];
    }
    target -= lengths[index];
  }
  return [points[0][0], points[0][1], 0];
}
