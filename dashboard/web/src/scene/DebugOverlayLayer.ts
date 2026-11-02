import * as THREE from 'three';
import type { DebugFrame, DebugOverlay, DebugRole, StateMessage } from '../types';

const MAX_LINE_VERTICES = 4096;
const MAX_POINTS = 256;
const MAX_SECTORS = 4;
const SECTOR_SEGMENTS = 24;
const OVERLAY_HEIGHT = 0.035;

const ROLE_COLORS: Record<DebugRole, number> = {
  primary: 0xd7a84b,
  secondary: 0x6ea8d0,
  muted: 0x6b7480,
  accent: 0xff6fae,
  warning: 0xc96b5a,
  candidate: 0x8ec7ef,
};

type CarPose = Pick<StateMessage['car'], 'x' | 'y' | 'theta'>;

function colorFor(role: DebugRole): number {
  return ROLE_COLORS[role] ?? ROLE_COLORS.secondary;
}

function makeLabel(text: string, color: string): THREE.Sprite {
  const canvas = document.createElement('canvas');
  canvas.width = 128;
  canvas.height = 48;
  const context = canvas.getContext('2d');
  if (context) {
    context.clearRect(0, 0, canvas.width, canvas.height);
    context.font = '600 24px Consolas, monospace';
    context.textAlign = 'center';
    context.textBaseline = 'middle';
    context.fillStyle = 'rgba(18, 20, 24, 0.82)';
    context.fillRect(18, 8, 92, 32);
    context.fillStyle = color;
    context.fillText(text, 64, 25);
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
  sprite.scale.set(0.14, 0.052, 1);
  sprite.renderOrder = 21;
  return sprite;
}

export class DebugOverlayLayer {
  private readonly group = new THREE.Group();
  private readonly lineGeometry = new THREE.BufferGeometry();
  private readonly linePositions = new Float32Array(MAX_LINE_VERTICES * 3);
  private readonly lineColors = new Float32Array(MAX_LINE_VERTICES * 3);
  private lineCount = 0;

  private readonly pointGeometry = new THREE.BufferGeometry();
  private readonly pointPositions = new Float32Array(MAX_POINTS * 3);
  private readonly pointColors = new Float32Array(MAX_POINTS * 3);
  private pointCount = 0;

  private readonly sectors: THREE.Mesh[] = [];
  private readonly sectorPositions: Float32Array[] = [];
  private readonly labelSprites = new Map<string, THREE.Sprite>();
  private activeLabels = new Set<string>();
  private readonly lineObject: THREE.LineSegments;
  private readonly pointObject: THREE.Points;

  constructor(scene: THREE.Scene) {
    this.lineGeometry.setAttribute(
      'position',
      new THREE.BufferAttribute(this.linePositions, 3).setUsage(THREE.DynamicDrawUsage),
    );
    this.lineGeometry.setAttribute(
      'color',
      new THREE.BufferAttribute(this.lineColors, 3).setUsage(THREE.DynamicDrawUsage),
    );
    this.lineGeometry.setDrawRange(0, 0);
    this.lineObject = new THREE.LineSegments(
      this.lineGeometry,
      new THREE.LineBasicMaterial({
        vertexColors: true,
        transparent: true,
        opacity: 0.92,
        depthTest: false,
      }),
    );
    this.lineObject.frustumCulled = false;
    this.lineObject.renderOrder = 10;
    this.group.add(this.lineObject);

    this.pointGeometry.setAttribute(
      'position',
      new THREE.BufferAttribute(this.pointPositions, 3).setUsage(THREE.DynamicDrawUsage),
    );
    this.pointGeometry.setAttribute(
      'color',
      new THREE.BufferAttribute(this.pointColors, 3).setUsage(THREE.DynamicDrawUsage),
    );
    this.pointGeometry.setDrawRange(0, 0);
    this.pointObject = new THREE.Points(
      this.pointGeometry,
      new THREE.PointsMaterial({
        size: 0.05,
        vertexColors: true,
        transparent: true,
        opacity: 0.95,
        depthTest: false,
      }),
    );
    this.pointObject.frustumCulled = false;
    this.pointObject.renderOrder = 12;
    this.group.add(this.pointObject);

    for (let index = 0; index < MAX_SECTORS; index += 1) {
      const positions = new Float32Array(SECTOR_SEGMENTS * 9);
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute(
        'position',
        new THREE.BufferAttribute(positions, 3).setUsage(THREE.DynamicDrawUsage),
      );
      geometry.setDrawRange(0, 0);
      const mesh = new THREE.Mesh(
        geometry,
        new THREE.MeshBasicMaterial({
          transparent: true,
          opacity: 0.08,
          side: THREE.DoubleSide,
          depthTest: false,
          depthWrite: false,
        }),
      );
      mesh.frustumCulled = false;
      mesh.renderOrder = 9;
      mesh.visible = false;
      this.sectors.push(mesh);
      this.sectorPositions.push(positions);
      this.group.add(mesh);
    }

    this.group.visible = false;
    scene.add(this.group);
  }

  update(frame: DebugFrame | null, car: CarPose, visible: boolean): void {
    if (!frame || !visible || frame.overlays.length === 0) {
      this.group.visible = false;
      return;
    }
    this.group.visible = true;
    this.lineCount = 0;
    this.pointCount = 0;
    this.activeLabels.clear();
    let sectorCount = 0;
    let pointSize = 0.04;

    for (const overlay of frame.overlays) {
      if (overlay.kind === 'sector' && sectorCount < MAX_SECTORS) {
        this.writeSector(this.sectors[sectorCount], this.sectorPositions[sectorCount], overlay, car);
        sectorCount += 1;
      }
    }
    for (let index = sectorCount; index < MAX_SECTORS; index += 1) {
      this.sectors[index].visible = false;
    }

    for (const overlay of frame.overlays) {
      if (overlay.kind === 'points') {
        pointSize = Math.max(pointSize, overlay.size);
      }
      this.writeOverlay(overlay, car);
    }

    (this.pointObject.material as THREE.PointsMaterial).size = pointSize;
    this.lineGeometry.setDrawRange(0, this.lineCount);
    this.lineGeometry.attributes.position.needsUpdate = true;
    this.lineGeometry.attributes.color.needsUpdate = true;
    this.pointGeometry.setDrawRange(0, this.pointCount);
    this.pointGeometry.attributes.position.needsUpdate = true;
    this.pointGeometry.attributes.color.needsUpdate = true;

    for (const [text, sprite] of this.labelSprites) {
      sprite.visible = this.activeLabels.has(text);
    }
  }

  private writeSector(
    mesh: THREE.Mesh,
    positions: Float32Array,
    overlay: Extract<DebugOverlay, { kind: 'sector' }>,
    car: CarPose,
  ): void {
    const center = this.worldPoint(overlay.center, overlay.frame, car);
    const start = this.worldAngle(overlay.start_angle_rad, overlay.frame, car);
    const end = this.worldAngle(overlay.end_angle_rad, overlay.frame, car);
    let vertex = 0;
    for (let index = 0; index < SECTOR_SEGMENTS; index += 1) {
      const a0 = start + ((end - start) * index) / SECTOR_SEGMENTS;
      const a1 = start + ((end - start) * (index + 1)) / SECTOR_SEGMENTS;
      positions[vertex] = center[0];
      positions[vertex + 1] = OVERLAY_HEIGHT;
      positions[vertex + 2] = center[1];
      positions[vertex + 3] = center[0] + Math.cos(a0) * overlay.range_m;
      positions[vertex + 4] = OVERLAY_HEIGHT;
      positions[vertex + 5] = center[1] + Math.sin(a0) * overlay.range_m;
      positions[vertex + 6] = center[0] + Math.cos(a1) * overlay.range_m;
      positions[vertex + 7] = OVERLAY_HEIGHT;
      positions[vertex + 8] = center[1] + Math.sin(a1) * overlay.range_m;
      vertex += 9;
    }
    const geometry = mesh.geometry as THREE.BufferGeometry;
    geometry.setDrawRange(0, SECTOR_SEGMENTS * 3);
    geometry.attributes.position.needsUpdate = true;
    const material = mesh.material as THREE.MeshBasicMaterial;
    material.color.setHex(colorFor(overlay.role));
    material.opacity = overlay.opacity;
    mesh.visible = true;
  }

  private writeOverlay(overlay: DebugOverlay, car: CarPose): void {
    if (overlay.kind === 'polyline') {
      const points = overlay.points.map((point) =>
        this.worldPoint(point, overlay.frame, car),
      );
      for (let index = 0; index < points.length - 1; index += 1) {
        this.addLine(points[index], points[index + 1], overlay.role);
      }
      if (overlay.closed && points.length > 2) {
        this.addLine(points[points.length - 1], points[0], overlay.role);
      }
      return;
    }
    if (overlay.kind === 'ray') {
      const origin = this.worldPoint(overlay.origin, overlay.frame, car);
      const angle =
        overlay.frame === 'car' ? car.theta + overlay.angle_rad : overlay.angle_rad;
      const end: [number, number] = [
        origin[0] + Math.cos(angle) * overlay.range_m,
        origin[1] + Math.sin(angle) * overlay.range_m,
      ];
      this.addLine(origin, end, overlay.role);
      return;
    }
    if (overlay.kind === 'points') {
      for (const point of overlay.points) {
        const world = this.worldPoint([point.x, point.y], overlay.frame, car);
        this.addPoint(world, point.role);
        if (point.label) this.placeLabel(point.label, world, point.role);
      }
      return;
    }
    if (overlay.kind === 'label') {
      const world = this.worldPoint(overlay.position, overlay.frame, car);
      this.placeLabel(overlay.text, world, overlay.role);
    }
  }

  private worldPoint(
    point: [number, number],
    frame: DebugOverlay['frame'],
    car: CarPose,
  ): [number, number] {
    if (frame === 'world') return [point[0], point[1]];
    const cosine = Math.cos(car.theta);
    const sine = Math.sin(car.theta);
    return [
      car.x + point[0] * cosine - point[1] * sine,
      car.y + point[0] * sine + point[1] * cosine,
    ];
  }

  private worldAngle(
    angle: number,
    frame: DebugOverlay['frame'],
    car: CarPose,
  ): number {
    return frame === 'car' ? car.theta + angle : angle;
  }

  private addLine(
    start: [number, number],
    end: [number, number],
    role: DebugRole,
  ): void {
    if (this.lineCount + 2 > MAX_LINE_VERTICES) return;
    const color = new THREE.Color(colorFor(role));
    for (const point of [start, end]) {
      const offset = this.lineCount * 3;
      this.linePositions[offset] = point[0];
      this.linePositions[offset + 1] = OVERLAY_HEIGHT;
      this.linePositions[offset + 2] = point[1];
      this.lineColors[offset] = color.r;
      this.lineColors[offset + 1] = color.g;
      this.lineColors[offset + 2] = color.b;
      this.lineCount += 1;
    }
  }

  private addPoint(point: [number, number], role: DebugRole): void {
    if (this.pointCount >= MAX_POINTS) return;
    const color = new THREE.Color(colorFor(role));
    const offset = this.pointCount * 3;
    this.pointPositions[offset] = point[0];
    this.pointPositions[offset + 1] = OVERLAY_HEIGHT + 0.003;
    this.pointPositions[offset + 2] = point[1];
    this.pointColors[offset] = color.r;
    this.pointColors[offset + 1] = color.g;
    this.pointColors[offset + 2] = color.b;
    this.pointCount += 1;
  }

  private placeLabel(text: string, point: [number, number], role: DebugRole): void {
    let sprite = this.labelSprites.get(text);
    if (!sprite) {
      sprite = makeLabel(text, `#${new THREE.Color(colorFor(role)).getHexString()}`);
      this.labelSprites.set(text, sprite);
      this.group.add(sprite);
    }
    sprite.position.set(point[0], OVERLAY_HEIGHT + 0.075, point[1]);
    sprite.visible = true;
    this.activeLabels.add(text);
  }

  dispose(): void {
    this.lineGeometry.dispose();
    (this.lineObject.material as THREE.Material).dispose();
    this.pointGeometry.dispose();
    (this.pointObject.material as THREE.Material).dispose();
    for (const sector of this.sectors) {
      sector.geometry.dispose();
      (sector.material as THREE.Material).dispose();
    }
    for (const sprite of this.labelSprites.values()) {
      const material = sprite.material as THREE.SpriteMaterial;
      material.map?.dispose();
      material.dispose();
    }
    this.labelSprites.clear();
    this.group.clear();
    this.group.removeFromParent();
  }
}
