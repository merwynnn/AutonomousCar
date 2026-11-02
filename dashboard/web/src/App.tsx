import { useEffect, useRef, useState, type ReactNode } from 'react';
import { TrackScene } from './scene/TrackScene';
import { TuningPanel } from './TuningPanel';
import { GapDebugPanel } from './GapDebugPanel';
import type {
  BodyPart,
  CameraMode,
  DebugLayers,
  HelloMessage,
  ScanMessage,
  SensorMount,
  StateMessage,
  Vec3,
  ViewMode,
  WheelSpec,
} from './types';
import { useTelemetry, type ConnectionStatus } from './useTelemetry';

function formatTime(seconds: number | null | undefined): string {
  if (seconds === null || seconds === undefined || !Number.isFinite(seconds)) return '—';
  const minutes = Math.floor(seconds / 60);
  const rest = seconds - minutes * 60;
  return minutes > 0
    ? `${minutes}:${rest.toFixed(2).padStart(5, '0')}`
    : `${rest.toFixed(2)}`;
}

function num(value: number | null | undefined, digits = 3): string {
  if (value === null || value === undefined || !Number.isFinite(value)) return '—';
  return value.toFixed(digits);
}

function vector(values: Vec3 | number[], digits = 3): string {
  return values.map((value) => value.toFixed(digits)).join('  ');
}

function statusLabel(status: ConnectionStatus): string {
  switch (status) {
    case 'open':
      return 'Connected';
    case 'connecting':
      return 'Connecting';
    case 'error':
      return 'Error';
    default:
      return 'Offline';
  }
}

function Icon({ name }: { name: string }) {
  const paths: Record<string, ReactNode> = {
    play: <path d="M8 5v14l11-7z" />,
    pause: <path d="M7 5h4v14H7zM13 5h4v14h-4z" />,
    step: <path d="M6 5v14l9-7zM17 5h2v14h-2z" />,
    reset: <path d="M5 5v6h6M5 11a7 7 0 1 1 2 5" fill="none" stroke="currentColor" strokeWidth="1.7" />,
    reload: <path d="M20 11a8 8 0 1 0-2.3 5.7M20 4v7h-7" fill="none" stroke="currentColor" strokeWidth="1.7" />,
    car: <path d="M5 16l1.2-5.2A2 2 0 0 1 8.1 9h7.8a2 2 0 0 1 1.9 1.8L19 16M5 16h14v3h-2v-2H7v2H5zM8 19h1M15 19h1" fill="none" stroke="currentColor" strokeWidth="1.6" />,
    track: <path d="M4 14c0-6 4-10 8-10s8 4 8 10-4 6-8 6-8 0-8-6zM8 14c0-4 2-6 4-6s4 2 4 6-2 2-4 2-4 2-4-2z" fill="none" stroke="currentColor" strokeWidth="1.5" />,
    camera: <path d="M4 8h3l2-3h6l2 3h3v10H4zM12 16a3.5 3.5 0 1 0 0-7 3.5 3.5 0 0 0 0 7z" fill="none" stroke="currentColor" strokeWidth="1.5" />,
    orbit: <path d="M4 12a8 8 0 0 1 16 0M12 4c4 2 6 4 6 8M12 4C8 6 6 8 6 12M8 18l-4-1 1-4M16 18l4-1-1-4" fill="none" stroke="currentColor" strokeWidth="1.4" />,
    top: <path d="M4 5h16v14H4zM4 9h16M9 5v14" fill="none" stroke="currentColor" strokeWidth="1.4" />,
    chase: <path d="M4 8l6-4 10 4v9l-10 4-6-4zM10 4v9M20 8l-10 5" fill="none" stroke="currentColor" strokeWidth="1.4" />,
    settings: <path d="M12 8.5A3.5 3.5 0 1 0 12 15.5 3.5 3.5 0 0 0 12 8.5zM4 12l2-1 .5-2-1.2-1.8 1.5-1.5L8.6 7 10.6 6l1-2h2l1 2 2 1 1.8-1.2 1.5 1.5L18.5 9l.5 2 2 1v2l-2 1-.5 2 1.2 1.8-1.5 1.5-1.8-1.2-2 1-1 2h-2l-1-2-2-1-1.8 1.2-1.5-1.5L6.5 17l-.5-2-2-1z" fill="none" stroke="currentColor" strokeWidth="1.2" />,
    folder: <path d="M3 6h6l2 2h10v11H3z" fill="none" stroke="currentColor" strokeWidth="1.4" />,
    box: <path d="M4 8l8-4 8 4v8l-8 4-8-4zM4 8l8 4 8-4M12 12v8" fill="none" stroke="currentColor" strokeWidth="1.3" />,
    sensor: <path d="M12 4a8 8 0 1 0 0 16M12 8a4 4 0 1 0 0 8M12 12h8" fill="none" stroke="currentColor" strokeWidth="1.3" />,
    wheel: <circle cx="12" cy="12" r="7" fill="none" stroke="currentColor" strokeWidth="1.4" />,
    grid: <path d="M4 4h16v16H4zM4 9h16M4 15h16M9 4v16M15 4v16" fill="none" stroke="currentColor" strokeWidth="1.2" />,
    left: <path d="M14.5 5.5 8 12l6.5 6.5" fill="none" stroke="currentColor" strokeWidth="1.7" />,
    right: <path d="M9.5 5.5 16 12l-6.5 6.5" fill="none" stroke="currentColor" strokeWidth="1.7" />,
  };
  return (
    <svg className="icon" viewBox="0 0 24 24" aria-hidden="true">
      {paths[name] ?? paths.box}
    </svg>
  );
}

function ToolButton({
  icon,
  label,
  active = false,
  disabled = false,
  title,
  onClick,
}: {
  icon: string;
  label: string;
  active?: boolean;
  disabled?: boolean;
  title: string;
  onClick: () => void;
}) {
  return (
    <button
      type="button"
      className={`tool-button ${active ? 'active' : ''}`}
      title={title}
      disabled={disabled}
      onClick={onClick}
    >
      <Icon name={icon} />
      <span>{label}</span>
    </button>
  );
}

function SceneCanvas({
  hello,
  state,
  scan,
  viewMode,
  cameraMode,
  layers,
  turntable,
}: {
  hello: HelloMessage | null;
  state: StateMessage | null;
  scan: ScanMessage | null;
  viewMode: ViewMode;
  cameraMode: CameraMode;
  layers: DebugLayers;
  turntable: boolean;
}) {
  const containerRef = useRef<HTMLDivElement>(null);
  const sceneRef = useRef<TrackScene | null>(null);

  useEffect(() => {
    if (!containerRef.current) return;
    const scene = new TrackScene(containerRef.current);
    sceneRef.current = scene;
    return () => {
      scene.dispose();
      sceneRef.current = null;
    };
  }, []);

  useEffect(() => {
    if (!hello) return;
    sceneRef.current?.setCar(hello);
    sceneRef.current?.setTrack(hello.track);
  }, [hello]);

  useEffect(() => {
    if (state) sceneRef.current?.updateState(state);
  }, [state]);

  useEffect(() => {
    if (state && scan) sceneRef.current?.updateScan(scan, state);
  }, [scan, state]);

  useEffect(() => {
    sceneRef.current?.setViewMode(viewMode);
  }, [viewMode]);

  useEffect(() => {
    sceneRef.current?.setCamera(cameraMode);
  }, [cameraMode]);

  useEffect(() => {
    sceneRef.current?.setLayers(layers);
  }, [layers]);

  useEffect(() => {
    sceneRef.current?.setTurntable(turntable);
  }, [turntable]);

  return <div ref={containerRef} className="canvas-host" />;
}

function PropertyRow({
  label,
  value,
  unit,
  tone,
}: {
  label: string;
  value: string;
  unit?: string;
  tone?: 'normal' | 'accent' | 'muted';
}) {
  return (
    <div className="property-row">
      <span>{label}</span>
      <strong className={tone ? `prop-${tone}` : ''}>
        {value}
        {unit ? <em>{unit}</em> : null}
      </strong>
    </div>
  );
}

function PropertyGroup({ title, children }: { title: string; children: ReactNode }) {
  return (
    <section className="property-group">
      <div className="property-group-title">{title}</div>
      {children}
    </section>
  );
}

type Selection =
  | 'vehicle'
  | 'structure'
  | 'wheels'
  | 'sensors'
  | 'lidar'
  | 'ultrasonic'
  | 'imu'
  | 'encoders'
  | 'track'
  | 'scenario'
  | 'race'
  | `part:${string}`
  | `sonar:${string}`;

function TreeRow({
  icon,
  label,
  selected,
  depth = 0,
  caret,
  trailing,
  onClick,
}: {
  icon: string;
  label: string;
  selected: boolean;
  depth?: number;
  caret?: string;
  trailing?: string;
  onClick: () => void;
}) {
  return (
    <button
      type="button"
      className={`tree-row ${selected ? 'selected' : ''}`}
      style={{ paddingLeft: 8 + depth * 16 }}
      onClick={onClick}
    >
      <span className="tree-caret">{caret ?? ''}</span>
      <span className="tree-icon">
        <Icon name={icon} />
      </span>
      <span className="tree-label">{label}</span>
      {trailing ? <span className="tree-trailing">{trailing}</span> : null}
    </button>
  );
}

function PanelRail({
  side,
  label,
  onExpand,
}: {
  side: 'left' | 'right';
  label: string;
  onExpand: () => void;
}) {
  return (
    <button
      type="button"
      className={`panel-rail panel-rail-${side}`}
      title={`Show ${label.toLowerCase()}`}
      onClick={onExpand}
    >
      <Icon name={side === 'left' ? 'right' : 'left'} />
      <span>{label}</span>
    </button>
  );
}

function Outliner({
  hello,
  selection,
  onSelect,
  onCollapse,
}: {
  hello: HelloMessage | null;
  selection: Selection;
  onSelect: (selection: Selection) => void;
  onCollapse: () => void;
}) {
  if (!hello) {
    return (
      <aside className="outliner">
        <div className="panel-heading">
          <span>World outliner</span>
          <button
            type="button"
            className="panel-collapse"
            title="Collapse outliner"
            onClick={onCollapse}
          >
            <Icon name="left" />
          </button>
        </div>
        <div className="empty-panel">Waiting for simulator…</div>
      </aside>
    );
  }
  const { vehicle, sensors } = hello;
  return (
    <aside className="outliner">
      <div className="panel-heading">
        <span>World outliner</span>
        <div className="panel-heading-actions">
          <span className="panel-heading-count">{vehicle.structure.length + sensors.mounts.length + 3}</span>
          <button
            type="button"
            className="panel-collapse"
            title="Collapse outliner"
            onClick={onCollapse}
          >
            <Icon name="left" />
          </button>
        </div>
      </div>
      <div className="tree">
        <div className="tree-section">Project</div>
        <TreeRow
          icon="folder"
          label={hello.scenario.name}
          caret="▾"
          selected={selection === 'scenario'}
          onClick={() => onSelect('scenario')}
        />
        <TreeRow
          icon="car"
          label={vehicle.name}
          caret="▾"
          selected={selection === 'vehicle'}
          onClick={() => onSelect('vehicle')}
        />
        <TreeRow
          icon="box"
          label="Structure"
          depth={1}
          caret="▾"
          selected={selection === 'structure'}
          trailing={`${vehicle.structure.length}`}
          onClick={() => onSelect('structure')}
        />
        {vehicle.structure.map((part) => (
          <TreeRow
            key={part.name}
            icon="box"
            label={part.name}
            depth={2}
            selected={selection === `part:${part.name}`}
            onClick={() => onSelect(`part:${part.name}`)}
          />
        ))}
        <TreeRow
          icon="wheel"
          label="Wheels"
          depth={1}
          selected={selection === 'wheels'}
          trailing={`${vehicle.wheels.length}`}
          onClick={() => onSelect('wheels')}
        />
        <TreeRow
          icon="sensor"
          label="Sensors"
          depth={1}
          caret="▾"
          selected={selection === 'sensors'}
          trailing={`${sensors.mounts.length}`}
          onClick={() => onSelect('sensors')}
        />
        <TreeRow
          icon="sensor"
          label="Lidar"
          depth={2}
          selected={selection === 'lidar'}
          onClick={() => onSelect('lidar')}
        />
        <TreeRow
          icon="sensor"
          label="Ultrasonic"
          depth={2}
          caret="▾"
          selected={selection === 'ultrasonic'}
          trailing={`${sensors.ultrasonic.sensors.length}`}
          onClick={() => onSelect('ultrasonic')}
        />
        {sensors.ultrasonic.sensors.map((sensor) => (
          <TreeRow
            key={sensor.name}
            icon="sensor"
            label={sensor.name}
            depth={3}
            selected={selection === `sonar:${sensor.name}`}
            onClick={() => onSelect(`sonar:${sensor.name}`)}
          />
        ))}
        <TreeRow
          icon="sensor"
          label="IMU"
          depth={2}
          selected={selection === 'imu'}
          onClick={() => onSelect('imu')}
        />
        <TreeRow
          icon="sensor"
          label="Encoders"
          depth={2}
          selected={selection === 'encoders'}
          onClick={() => onSelect('encoders')}
        />
        <div className="tree-section">Environment</div>
        <TreeRow
          icon="track"
          label={hello.track.name === 'random_track' ? 'Random track' : 'Test track'}
          selected={selection === 'track'}
          onClick={() => onSelect('track')}
        />
        <TreeRow
          icon="grid"
          label="Race session"
          selected={selection === 'race'}
          onClick={() => onSelect('race')}
        />
      </div>

      <div className="outliner-section config-paths">
        <div className="panel-heading compact">Configuration</div>
        <div className="path-row" title={hello.paths.vehicle}>
          <span>Vehicle</span>
          <code>{shortPath(hello.paths.vehicle)}</code>
        </div>
        <div className="path-row" title={hello.paths.sensors}>
          <span>Sensors</span>
          <code>{shortPath(hello.paths.sensors)}</code>
        </div>
      </div>
    </aside>
  );
}

function shortPath(path: string): string {
  const parts = path.replaceAll('\\', '/').split('/');
  return parts.slice(-3).join('/');
}

function SensorDetails({
  mount,
  hello,
}: {
  mount: SensorMount;
  hello: HelloMessage;
}) {
  const sensors = hello.sensors;
  const isLidar = mount.kind === 'lidar';
  const isSonar = mount.kind === 'ultrasonic';
  const isImu = mount.kind === 'imu';
  return (
    <>
      <div className="details-identity">
        <div className={`details-glyph glyph-${mount.kind}`}>
          <Icon name="sensor" />
        </div>
        <div>
          <h2>{mount.name}</h2>
          <p>
            {mount.kind} · frame <code>{mount.frame_id}</code>
          </p>
        </div>
      </div>
      <PropertyGroup title="Mount">
        <PropertyRow label="Position" value={vector(mount.position_m)} unit="m" />
        <PropertyRow label="Rotation" value={vector(mount.rpy_deg)} unit="deg" />
        <PropertyRow
          label="Height above floor"
          value={num(mount.ground_height_m)}
          unit="m"
          tone="accent"
        />
        <PropertyRow label="Body frame" value="x fwd · y left · z up" tone="muted" />
      </PropertyGroup>
      <PropertyGroup title="Characteristics">
        {isLidar ? (
          <>
            <PropertyRow label="Beams" value={`${sensors.lidar.beams}`} />
            <PropertyRow label="Rate" value={num(sensors.lidar.rate_hz, 1)} unit="Hz" />
            <PropertyRow
              label="Range"
              value={`${num(sensors.lidar.min_range_m, 2)} – ${num(sensors.lidar.max_range_m, 1)}`}
              unit="m"
            />
            <PropertyRow label="Scan direction" value={sensors.lidar.scan_direction} />
            <PropertyRow
              label="Range noise σ"
              value={num(sensors.lidar.noise.range_sigma_m, 4)}
              unit="m"
            />
            <PropertyRow
              label="Angle offset"
              value={num(sensors.lidar.noise.angle_offset_deg, 2)}
              unit="deg"
            />
            <PropertyRow label="Latency" value={num(sensors.lidar.latency_ms, 1)} unit="ms" />
          </>
        ) : null}
        {isSonar ? (
          <>
            <PropertyRow label="Cone angle" value={num(sensors.ultrasonic.cone_angle_deg, 1)} unit="deg" />
            <PropertyRow label="Range" value={`${num(sensors.ultrasonic.min_range_m, 2)} – ${num(sensors.ultrasonic.max_range_m, 1)}`} unit="m" />
            <PropertyRow label="Rate" value={num(sensors.ultrasonic.rate_hz, 1)} unit="Hz" />
            <PropertyRow label="Rays / sample" value={`${sensors.ultrasonic.rays}`} />
            <PropertyRow label="Range noise σ" value={num(sensors.ultrasonic.range_sigma_m, 3)} unit="m" />
            <PropertyRow label="Latency" value={num(sensors.ultrasonic.latency_ms, 1)} unit="ms" />
            <PropertyRow label="Cross-talk" value={`${(sensors.ultrasonic.cross_talk_probability * 100).toFixed(1)} %`} />
          </>
        ) : null}
        {isImu ? (
          <>
            <PropertyRow label="Rate" value={num(sensors.imu.rate_hz, 1)} unit="Hz" />
            <PropertyRow label="Latency" value={num(sensors.imu.latency_ms, 1)} unit="ms" />
            <PropertyRow label="Gyro bias" value={num(sensors.imu.gyro.bias_deg_s, 2)} unit="deg/s" />
            <PropertyRow label="Accel noise" value={num(sensors.imu.accel.noise_m_s2, 3)} unit="m/s²" />
            <PropertyRow label="Axis misalignment" value={num(sensors.imu.mounting_misalignment_deg, 2)} unit="deg" />
          </>
        ) : null}
      </PropertyGroup>
      <div className="details-hint">
        Edit the mount in <code>{shortPath(hello.paths.sensors)}</code> then reload the
        configuration to rebuild both the physics and this visual.
      </div>
    </>
  );
}

function DetailsPanel({
  hello,
  state,
  selection,
  speed,
  onSpeed,
  onOpenTune,
  onGenerateTrack,
  onDefaultTrack,
  onCollapse,
}: {
  hello: HelloMessage | null;
  state: StateMessage | null;
  selection: Selection;
  speed: number;
  onSpeed: (speed: number) => void;
  onOpenTune: () => void;
  onGenerateTrack: (seed: number, density: number, width: number) => void;
  onDefaultTrack: () => void;
  onCollapse: () => void;
}) {
  if (!hello) {
    return (
      <aside className="details-panel">
        <div className="panel-heading">
          <span>Details</span>
          <button
            type="button"
            className="panel-collapse"
            title="Collapse details"
            onClick={onCollapse}
          >
            <Icon name="right" />
          </button>
        </div>
        <div className="empty-panel">No selection</div>
      </aside>
    );
  }
  const vehicle = hello.vehicle;
  const sensors = hello.sensors;
  let content: ReactNode;

  if (selection.startsWith('part:')) {
    const name = selection.slice(5);
    const part = vehicle.structure.find((item) => item.name === name);
    content = part ? <PartDetails part={part} /> : null;
  } else if (selection.startsWith('sonar:')) {
    const name = selection.slice(6);
    const mount = sensors.mounts.find((item) => item.id === name);
    content = mount ? <SensorDetails mount={mount} hello={hello} /> : null;
  } else if (selection === 'lidar') {
    const mount = sensors.mounts.find((item) => item.kind === 'lidar');
    content = mount ? <SensorDetails mount={mount} hello={hello} /> : null;
  } else if (selection === 'imu') {
    const mount = sensors.mounts.find((item) => item.kind === 'imu');
    content = mount ? <SensorDetails mount={mount} hello={hello} /> : null;
  } else if (selection === 'ultrasonic' || selection === 'sensors') {
    content = <SensorOverview hello={hello} />;
  } else if (selection === 'wheels') {
    content = <WheelDetails wheels={vehicle.wheels} vehicle={vehicle} />;
  } else if (selection === 'structure') {
    content = <StructureDetails hello={hello} />;
  } else if (selection === 'track') {
    content = (
      <TrackDetails
        hello={hello}
        onGenerateTrack={onGenerateTrack}
        onDefaultTrack={onDefaultTrack}
      />
    );
  } else if (selection === 'scenario') {
    content = <ScenarioDetails hello={hello} />;
  } else if (selection === 'race') {
    content = (
      <RaceDetails
        hello={hello}
        state={state}
        speed={speed}
        onSpeed={onSpeed}
      />
    );
  } else if (selection === 'encoders') {
    content = <EncoderDetails hello={hello} />;
  } else {
    content = <VehicleDetails hello={hello} />;
  }

  return (
    <aside className="details-panel">
      <div className="panel-heading">
        <span>Details</span>
        <div className="panel-heading-actions">
          <button type="button" className="panel-action" onClick={onOpenTune} title="Edit vehicle and sensor parameters">
            Tune
          </button>
          <span className="panel-heading-count">{selectionLabel(selection)}</span>
          <button
            type="button"
            className="panel-collapse"
            title="Collapse details"
            onClick={onCollapse}
          >
            <Icon name="right" />
          </button>
        </div>
      </div>
      <div className="details-scroll">{content}</div>
    </aside>
  );
}

function selectionLabel(selection: Selection): string {
  if (selection.startsWith('part:')) return 'Body part';
  if (selection.startsWith('sonar:')) return 'Ultrasonic';
  return selection.charAt(0).toUpperCase() + selection.slice(1);
}

function VehicleDetails({ hello }: { hello: HelloMessage }) {
  const vehicle = hello.vehicle;
  return (
    <>
      <div className="details-identity">
        <div className="details-glyph glyph-car">
          <Icon name="car" />
        </div>
        <div>
          <h2>{vehicle.name}</h2>
          <p>{vehicle.description || 'Vehicle definition'}</p>
        </div>
      </div>
      <PropertyGroup title="Mass & inertia">
        <PropertyRow label="Total mass" value={num(vehicle.mass_kg, 3)} unit="kg" tone="accent" />
        <PropertyRow label="Chassis mass" value={num(vehicle.chassis_mass_kg, 3)} unit="kg" />
        <PropertyRow label="Wheel mass" value={num(vehicle.wheel_mass_kg, 3)} unit="kg" />
        <PropertyRow label="Centre of mass" value={vector(vehicle.center_of_mass_m)} unit="m" />
      </PropertyGroup>
      <PropertyGroup title="Geometry">
        <PropertyRow label="Overall size" value={`${num(vehicle.dimensions.length_m, 3)} × ${num(vehicle.dimensions.width_m, 3)} × ${num(vehicle.dimensions.height_m, 3)}`} unit="m" />
        <PropertyRow label="Wheelbase" value={num(vehicle.wheelbase_m, 3)} unit="m" tone="accent" />
        <PropertyRow label="Track width" value={num(vehicle.track_width_m, 3)} unit="m" />
        <PropertyRow label="Wheel radius" value={num(vehicle.wheel_radius_m, 3)} unit="m" />
        <PropertyRow label="Wheel width" value={num(vehicle.wheel_width_m, 3)} unit="m" />
        <PropertyRow label="Ride height" value={num(vehicle.ride_height_m, 3)} unit="m" />
        <PropertyRow label="Body parts" value={`${vehicle.structure.length}`} />
      </PropertyGroup>
      <PropertyGroup title="Drivetrain">
        <PropertyRow label="Drive" value={vehicle.drive.toUpperCase()} tone="accent" />
        <PropertyRow label="Motor torque" value={num(vehicle.motor.max_torque_nm, 2)} unit="N·m" />
        <PropertyRow label="Motor speed" value={num(vehicle.motor.max_rpm, 0)} unit="rpm" />
        <PropertyRow label="Gear ratio" value={num(vehicle.motor.gear_ratio, 2)} unit=":1" />
        <PropertyRow label="Steering limit" value={num(vehicle.steering.max_angle_deg, 1)} unit="deg" />
        <PropertyRow label="Steering rate" value={num(vehicle.steering.rate_deg_s, 0)} unit="deg/s" />
        <PropertyRow label="Tire friction" value={`${num(vehicle.tires.friction_front, 2)} / ${num(vehicle.tires.friction_rear, 2)}`} unit="F/R" />
      </PropertyGroup>
      <PropertyGroup title="Electrical & aero">
        <PropertyRow label="Battery" value={`${vehicle.battery.cells}S · ${num(vehicle.battery.capacity_mah, 0)}`} unit="mAh" />
        <PropertyRow label="Nominal voltage" value={num(vehicle.battery.nominal_voltage_v, 1)} unit="V" />
        <PropertyRow label="Drag coefficient" value={num(vehicle.aero.drag_coefficient, 2)} />
        <PropertyRow label="Frontal area" value={num(vehicle.aero.frontal_area_m2, 3)} unit="m²" />
        <PropertyRow label="Suspension" value={`${num(vehicle.suspension.stiffness, 1)} N/m · ${num(vehicle.suspension.damping, 1)} Ns/m`} />
      </PropertyGroup>
      <div className="details-hint">
        This panel is generated from <code>{shortPath(hello.paths.vehicle)}</code>.
        Change it in YAML or construct a <code>VehicleConfig</code> in Python.
      </div>
    </>
  );
}

function PartDetails({ part }: { part: BodyPart }) {
  return (
    <>
      <div className="details-identity">
        <div className="details-glyph glyph-part">
          <Icon name="box" />
        </div>
        <div>
          <h2>{part.name}</h2>
          <p>{part.shape} body part</p>
        </div>
      </div>
      <PropertyGroup title="Transform">
        <PropertyRow label="Position" value={vector(part.position_m)} unit="m" />
        <PropertyRow label="Rotation" value={vector(part.rpy_deg)} unit="deg" />
        <PropertyRow label="Dimensions" value={vector(part.size_m)} unit="m" />
      </PropertyGroup>
      <PropertyGroup title="Rendering">
        <PropertyRow label="Colour" value={part.color} />
        <PropertyRow label="Opacity" value={num(part.opacity, 2)} />
        <PropertyRow label="Visible" value={part.visible ? 'yes' : 'no'} />
        <PropertyRow label="Collision" value={part.collision ? 'enabled' : 'visual only'} tone={part.collision ? 'accent' : 'muted'} />
      </PropertyGroup>
      <div className="details-hint">
        Add, remove or move parts in the <code>structure</code> list. The MJCF
        collision/visual geometry and this view are generated from the same data.
      </div>
    </>
  );
}

function WheelDetails({ wheels, vehicle }: { wheels: WheelSpec[]; vehicle: HelloMessage['vehicle'] }) {
  return (
    <>
      <div className="details-identity">
        <div className="details-glyph glyph-wheel">
          <Icon name="wheel" />
        </div>
        <div>
          <h2>Wheels</h2>
          <p>{vehicle.drive.toUpperCase()} · radius {num(vehicle.wheel_radius_m, 3)} m</p>
        </div>
      </div>
      {wheels.map((wheel) => (
        <PropertyGroup key={wheel.name} title={wheel.name.replaceAll('_', ' ')}>
          <PropertyRow label="Position" value={vector(wheel.position_m)} unit="m" />
          <PropertyRow label="Radius / width" value={`${num(wheel.radius_m, 3)} / ${num(wheel.width_m, 3)}`} unit="m" />
          <PropertyRow label="Mass" value={num(wheel.mass_kg, 3)} unit="kg" />
          <PropertyRow label="Steered" value={wheel.steered ? 'yes' : 'no'} />
          <PropertyRow label="Driven" value={wheel.driven ? 'yes' : 'no'} />
        </PropertyGroup>
      ))}
    </>
  );
}

function StructureDetails({ hello }: { hello: HelloMessage }) {
  return (
    <>
      <div className="details-identity">
        <div className="details-glyph glyph-part">
          <Icon name="box" />
        </div>
        <div>
          <h2>Structure</h2>
          <p>{hello.vehicle.structure.length} authored body parts</p>
        </div>
      </div>
      {hello.vehicle.structure.map((part) => (
        <PropertyGroup key={part.name} title={part.name}>
          <PropertyRow label="Shape" value={part.shape} />
          <PropertyRow label="Position" value={vector(part.position_m)} unit="m" />
          <PropertyRow label="Size" value={vector(part.size_m)} unit="m" />
          <PropertyRow label="Collision" value={part.collision ? 'yes' : 'no'} />
        </PropertyGroup>
      ))}
    </>
  );
}

function SensorOverview({ hello }: { hello: HelloMessage }) {
  return (
    <>
      <div className="details-identity">
        <div className="details-glyph glyph-sensor">
          <Icon name="sensor" />
        </div>
        <div>
          <h2>Sensors</h2>
          <p>{hello.sensors.mounts.length} mounted devices</p>
        </div>
      </div>
      {hello.sensors.mounts.map((mount) => (
        <div className="sensor-summary" key={mount.id}>
          <span className={`sensor-dot dot-${mount.kind}`} />
          <div>
            <strong>{mount.name}</strong>
            <span>{mount.summary}</span>
            <code>{vector(mount.position_m)} m</code>
          </div>
        </div>
      ))}
      <div className="details-hint">
        Sensor mounts use the body frame. Rotations are roll, pitch, yaw and are
        applied to the lidar ray directions.
      </div>
    </>
  );
}

function EncoderDetails({ hello }: { hello: HelloMessage }) {
  const encoders = hello.sensors.encoders;
  return (
    <>
      <div className="details-identity">
        <div className="details-glyph glyph-sensor">
          <Icon name="sensor" />
        </div>
        <div>
          <h2>Encoders</h2>
          <p>Wheel odometry source</p>
        </div>
      </div>
      <PropertyGroup title="Encoder model">
        <PropertyRow label="Rate" value={num(encoders.rate_hz, 1)} unit="Hz" />
        <PropertyRow label="Ticks / rev" value={`${encoders.ticks_per_rev}`} />
        <PropertyRow label="Wheel radius error" value={num(encoders.wheel_radius_error * 100, 2)} unit="%" />
        <PropertyRow label="Track width error" value={num(encoders.track_width_error * 100, 2)} unit="%" />
        <PropertyRow label="Timestamp jitter" value={num(encoders.timestamp_jitter_ms, 2)} unit="ms" />
        <PropertyRow label="Dropout" value={`${(encoders.dropout_probability * 100).toFixed(2)} %`} />
      </PropertyGroup>
    </>
  );
}

function TrackDetails({
  hello,
  onGenerateTrack,
  onDefaultTrack,
}: {
  hello: HelloMessage;
  onGenerateTrack: (seed: number, density: number, width: number) => void;
  onDefaultTrack: () => void;
}) {
  const track = hello.track;
  const meta = hello.track_meta;
  const [seed, setSeed] = useState(meta.seed);
  const [density, setDensity] = useState(meta.object_density);
  const [width, setWidth] = useState(meta.width_m);

  useEffect(() => {
    setSeed(meta.seed);
    setDensity(meta.object_density);
    setWidth(meta.width_m);
  }, [meta.seed, meta.object_density, meta.width_m]);

  return (
    <>
      <div className="details-identity">
        <div className="details-glyph glyph-track">
          <Icon name="track" />
        </div>
        <div>
          <h2>{meta.mode === 'random' ? 'Random track' : 'Test track'}</h2>
          <p>
            {meta.mode === 'random'
              ? `seed ${meta.seed} · ${track.obstacles.length} obstacles`
              : 'Default PDF-like peanut circuit'}
          </p>
        </div>
      </div>
      <PropertyGroup title="Generator">
        <label className="details-field">
          <span>Seed</span>
          <input type="number" step="1" value={seed} onChange={(event) => setSeed(Number(event.target.value))} />
        </label>
        <label className="details-field">
          <span>Object density</span>
          <div className="details-slider">
            <input
              type="range"
              min="0"
              max="1"
              step="0.05"
              value={density}
              onChange={(event) => setDensity(Number(event.target.value))}
            />
            <strong>{density.toFixed(2)}</strong>
          </div>
        </label>
        <label className="details-field">
          <span>Lane width</span>
          <div className="details-slider">
            <input
              type="range"
              min="0.8"
              max="1.0"
              step="0.01"
              value={width}
              onChange={(event) => setWidth(Number(event.target.value))}
            />
            <strong>{width.toFixed(2)} m</strong>
          </div>
        </label>
        <div className="button-row details-actions">
          <button
            type="button"
            className="primary"
            onClick={() => onGenerateTrack(seed, density, width)}
            title="Generate a random closed track with collidable obstacles"
          >
            Generate
          </button>
          <button
            type="button"
            className="ghost"
            onClick={onDefaultTrack}
            title="Restore the default test track"
          >
            Default
          </button>
        </div>
        <p className="small-note">
          Object density 0 = empty circuit, 1 = maximum crates/poles. Generated
          tracks stay inside the 15 × 8 m arena. <strong>Default</strong> restores
          the reference test track.
        </p>
      </PropertyGroup>
      <PropertyGroup title="Geometry">
        <PropertyRow label="Track length" value={num(track.track_length_m, 2)} unit="m" tone="accent" />
        <PropertyRow label="Lane width" value={num(track.width_m, 2)} unit="m" />
        <PropertyRow label="Bounds" value={`${track.bounds[0]} × ${track.bounds[1]} → ${track.bounds[2]} × ${track.bounds[3]}`} unit="m" />
        <PropertyRow label="Wall height" value={num(track.wall_height_m, 2)} unit="m" />
        <PropertyRow label="Wall radius" value={num(track.wall_radius_m, 3)} unit="m" />
        <PropertyRow label="Start distance" value={num(track.start_distance_m, 2)} unit="m" />
        <PropertyRow label="Obstacles" value={`${track.obstacles.length}`} />
      </PropertyGroup>
      <div className="details-hint">
        The default track is always available. Random tracks rebuild the physics
        world and reset the car to the new start line.
      </div>
    </>
  );
}

function ScenarioDetails({ hello }: { hello: HelloMessage }) {
  const scenario = hello.scenario;
  return (
    <>
      <div className="details-identity">
        <div className="details-glyph glyph-scenario">
          <Icon name="folder" />
        </div>
        <div>
          <h2>{scenario.name}</h2>
          <p>Scenario definition</p>
        </div>
      </div>
      <PropertyGroup title="Run">
        <PropertyRow label="Realism" value={scenario.realism} tone="accent" />
        <PropertyRow label="Seed" value={`${scenario.seed}`} />
        <PropertyRow label="Race laps" value={`${scenario.laps}`} />
        <PropertyRow label="Physics" value="240" unit="Hz" />
        <PropertyRow label="Control" value="60" unit="Hz" />
      </PropertyGroup>
      <PropertyGroup title="Source">
        <PropertyRow label="Vehicle file" value={shortPath(hello.paths.vehicle)} />
        <PropertyRow label="Sensors file" value={shortPath(hello.paths.sensors)} />
      </PropertyGroup>
      <div className="details-hint">{scenario.description}</div>
    </>
  );
}

function RaceDetails({
  hello,
  state,
  speed,
  onSpeed,
}: {
  hello: HelloMessage;
  state: StateMessage | null;
  speed: number;
  onSpeed: (speed: number) => void;
}) {
  return (
    <>
      <div className="details-identity">
        <div className="details-glyph glyph-race">
          <Icon name="grid" />
        </div>
        <div>
          <h2>Race session</h2>
          <p>{state?.paused ? 'Paused' : state?.finished ? 'Finished' : 'Running'}</p>
        </div>
      </div>
      <PropertyGroup title="Timing">
        <PropertyRow label="Lap" value={state ? `${state.race.lap} / ${state.race.laps}` : '—'} />
        <PropertyRow label="Lap time" value={formatTime(state?.race.lap_time)} unit="s" />
        <PropertyRow label="Last lap" value={formatTime(state?.race.last_lap)} unit="s" />
        <PropertyRow label="Best lap" value={formatTime(state?.race.best_lap)} unit="s" tone="accent" />
        <PropertyRow label="Total" value={formatTime(state?.race.total_time)} unit="s" />
      </PropertyGroup>
      <PropertyGroup title="Control">
        <PropertyRow label="Mode" value={state?.control_mode ?? 'auto'} />
        <PropertyRow label="Controller" value={state?.auto_controller ?? 'gap_follow'} />
        <PropertyRow label="Speed limit" value={num(speed, 1)} unit="m/s" />
        <div className="slider-block">
          <input
            type="range"
            min="0.5"
            max="6"
            step="0.1"
            value={speed}
            onChange={(event) => onSpeed(Number(event.target.value))}
          />
          <span>{num(speed, 1)} m/s</span>
        </div>
      </PropertyGroup>
      <PropertyGroup title="Scenario">
        <PropertyRow label="Name" value={hello.scenario.name} />
        <PropertyRow label="Realism" value={hello.scenario.realism} />
      </PropertyGroup>
    </>
  );
}

function ViewportTabs({
  viewMode,
  onChange,
}: {
  viewMode: ViewMode;
  onChange: (mode: ViewMode) => void;
}) {
  return (
    <div className="viewport-tabs">
      <button
        type="button"
        className={viewMode === 'track' ? 'active' : ''}
        onClick={() => onChange('track')}
      >
        Track
      </button>
      <button
        type="button"
        className={viewMode === 'car' ? 'active' : ''}
        onClick={() => onChange('car')}
      >
        Car only
      </button>
      <span className="viewport-tab-note">
        {viewMode === 'car' ? 'isolated vehicle · no track' : 'track overview'}
      </span>
    </div>
  );
}

function ViewportHud({
  hello,
  state,
  viewMode,
  scan,
}: {
  hello: HelloMessage | null;
  state: StateMessage | null;
  viewMode: ViewMode;
  scan: ScanMessage | null;
}) {
  const valid = scan?.ranges.filter((range) => range !== null).length ?? 0;
  const total = scan?.ranges.length ?? 0;
  const running = state ? !state.paused && !state.finished : false;
  return (
    <>
      <div className="viewport-status">
        <span className={`record-dot ${running ? 'live' : ''}`} />
        <span>{running ? 'SIM RUNNING' : state?.finished ? 'FINISHED' : 'SIM PAUSED'}</span>
        <i />
        <span>{viewMode === 'car' ? 'STUDIO' : 'WORLD'}</span>
        <i />
        <span>{state?.control_mode === 'manual' ? 'MANUAL' : (state?.auto_controller ?? 'AUTO').replace('_', '-').toUpperCase()}</span>
      </div>
      <div className="viewport-stats">
        {viewMode === 'car' ? (
          <>
            <span><b>{num(hello?.vehicle.mass_kg, 2)}</b> kg</span>
            <span><b>{num(hello?.vehicle.wheelbase_m, 3)}</b> m wheelbase</span>
            <span><b>{hello?.sensors.lidar.beams ?? '—'}</b> lidar beams</span>
            <span><b>{hello?.sensors.mounts.length ?? '—'}</b> sensors</span>
          </>
        ) : (
          <>
            <span><b>{num(state?.car.speed, 2)}</b> m/s</span>
            <span><b>{((state?.car.steering ?? 0) * 180 / Math.PI).toFixed(1)}</b>° steer</span>
            <span><b>{state ? `${state.race.lap}/${state.race.laps}` : '—'}</b> lap</span>
            <span><b>{formatTime(state?.race.lap_time)}</b> s</span>
          </>
        )}
      </div>
      <div className="viewport-readout">
        <span>valid returns</span>
        <strong>{valid}<em> / {total || '—'}</em></strong>
        <span>sim time</span>
        <strong>{num(state?.t, 1)}<em> s</em></strong>
        <span>yaw</span>
        <strong>{num(state ? state.car.theta * 180 / Math.PI : undefined, 1)}<em> °</em></strong>
      </div>
      <div className="viewport-hints">
        <span><kbd>Z</kbd><kbd>Q</kbd><kbd>S</kbd><kbd>D</kbd> drive</span>
        <span><kbd>Space</kbd> run / pause</span>
        <span><kbd>R</kbd> reset</span>
        <span><kbd>C</kbd> view</span>
      </div>
    </>
  );
}

type DriveKey = 'forward' | 'back' | 'left' | 'right';

const DRIVE_KEYS: Record<string, DriveKey> = {
  KeyW: 'forward',
  KeyZ: 'forward',
  ArrowUp: 'forward',
  KeyS: 'back',
  ArrowDown: 'back',
  KeyA: 'left',
  KeyQ: 'left',
  ArrowLeft: 'left',
  KeyD: 'right',
  ArrowRight: 'right',
};

export default function App() {
  const { status, hello, state, scan, send, configResult, trackResult } = useTelemetry();
  const [viewMode, setViewMode] = useState<ViewMode>('track');
  const [cameraMode, setCameraMode] = useState<CameraMode>('orbit');
  const [turntable, setTurntable] = useState(true);
  const [controlMode, setControlMode] = useState<'auto' | 'manual'>('auto');
  const [autoController, setAutoController] = useState<'gap_follow' | 'wall_follow'>('gap_follow');
  const [speed, setSpeed] = useState(1.5);
  const [layers, setLayers] = useState<DebugLayers>({
    scan: true,
    trail: true,
    odometry: false,
    sensors: true,
    coverage: false,
    labels: false,
    gap: true,
  });
  const [selection, setSelection] = useState<Selection>('vehicle');
  const [outlinerOpen, setOutlinerOpen] = useState(true);
  const [detailsOpen, setDetailsOpen] = useState(true);
  const [tuningOpen, setTuningOpen] = useState(false);
  const [tuningTab, setTuningTab] = useState<'vehicle' | 'sensors'>('vehicle');
  const [toast, setToast] = useState<string | null>(null);
  const keysRef = useRef<Record<DriveKey, boolean>>({
    forward: false,
    back: false,
    left: false,
    right: false,
  });
  const running = state ? !state.paused && !state.finished : false;

  useEffect(() => {
    if (!configResult) return;
    setToast(configResult.message);
    const timer = window.setTimeout(() => setToast(null), 4500);
    return () => window.clearTimeout(timer);
  }, [configResult]);

  useEffect(() => {
    if (!trackResult) return;
    setToast(trackResult.message);
    const timer = window.setTimeout(() => setToast(null), 4500);
    return () => window.clearTimeout(timer);
  }, [trackResult]);

  useEffect(() => {
    send({ type: 'control.mode', mode: controlMode });
  }, [controlMode, send]);

  useEffect(() => {
    send({ type: 'control.controller', controller: autoController });
  }, [autoController, send]);

  useEffect(() => {
    send({ type: 'run.set_speed', speed });
  }, [speed, send]);

  useEffect(() => {
    if (controlMode !== 'manual' || tuningOpen) return;
    const timer = window.setInterval(() => {
      const keys = keysRef.current;
      const throttle = (keys.forward ? 1 : 0) - (keys.back ? 0.7 : 0);
      // Mirror the manual steering command relative to the simulator's
      // positive-left convention, matching the dashboard's on-screen Q/D feel.
      const steering = (keys.right ? 0.45 : 0) - (keys.left ? 0.45 : 0);
      send({ type: 'drive.command', throttle, steering });
    }, 50);
    return () => window.clearInterval(timer);
  }, [controlMode, send, tuningOpen]);

  useEffect(() => {
    const onKeyDown = (event: KeyboardEvent) => {
      if (tuningOpen) return;
      const target = event.target as HTMLElement | null;
      if (target && ['INPUT', 'TEXTAREA', 'SELECT'].includes(target.tagName)) return;
      const driveKey = DRIVE_KEYS[event.code];
      if (driveKey) {
        event.preventDefault();
        keysRef.current[driveKey] = true;
        setControlMode('manual');
        if (state?.paused) send({ type: 'run.start' });
        return;
      }
      const key = event.key.toLowerCase();
      if (event.code === 'Space') {
        event.preventDefault();
        send({ type: running ? 'run.pause' : 'run.start' });
      } else if (key === 'r') {
        send({ type: 'run.reset' });
      } else if (key === 'c') {
        setViewMode((mode) => (mode === 'track' ? 'car' : 'track'));
      } else if (key === '1') {
        setCameraMode('orbit');
      } else if (key === '2') {
        setCameraMode('top');
      } else if (key === '3') {
        setCameraMode('chase');
      } else if (key === 't') {
        setTurntable((value) => !value);
      }
    };
    const onKeyUp = (event: KeyboardEvent) => {
      const driveKey = DRIVE_KEYS[event.code];
      if (driveKey) {
        event.preventDefault();
        keysRef.current[driveKey] = false;
      }
    };
    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('keyup', onKeyUp);
    return () => {
      window.removeEventListener('keydown', onKeyDown);
      window.removeEventListener('keyup', onKeyUp);
    };
  }, [running, send, state?.paused, tuningOpen]);

  const onReset = () => send({ type: 'run.reset' });
  const onReload = () => send({ type: 'config.reload' });
  const openTune = (tab: 'vehicle' | 'sensors') => {
    setTuningTab(tab);
    setTuningOpen(true);
  };
  const selectionTab: 'vehicle' | 'sensors' =
    selection === 'sensors' ||
    selection === 'lidar' ||
    selection === 'ultrasonic' ||
    selection === 'imu' ||
    selection === 'encoders' ||
    selection.startsWith('sonar:')
      ? 'sensors'
      : 'vehicle';
  const onSpeed = (value: number) => {
    setSpeed(value);
    send({ type: 'run.set_speed', speed: value });
  };
  const onGenerateTrack = (seed: number, density: number, width: number) => {
    send({
      type: 'track.generate',
      seed: Math.round(seed),
      object_density: density,
      width_m: width,
    });
  };
  const onDefaultTrack = () => send({ type: 'track.default' });

  return (
    <div className="app-shell">
      <header className="menubar">
        <div className="wordmark">
          <span className="wordmark-mark">A</span>
          <span>AutoBench</span>
          <em>CAR LAB</em>
        </div>
        <nav className="menus" aria-label="Application menu">
          <button type="button">File</button>
          <button type="button">Edit</button>
          <button type="button">View</button>
          <button type="button">Simulation</button>
          <button type="button">Telemetry</button>
          <button type="button">Help</button>
        </nav>
        <div className="menubar-right">
          <span className={`connection connection-${status}`}>
            <i />
            {statusLabel(status)}
          </span>
          <span className="menubar-meta">
            MUJOCO · 240 Hz
          </span>
          {hello?.warnings.length ? (
            <span className="menubar-warning" title={hello.warnings.join('\n')}>
              {hello.warnings.length} warning
            </span>
          ) : null}
        </div>
      </header>

      <div className="toolbar">
        <div className="tool-group">
          <ToolButton
            icon="play"
            label="Run"
            active={running}
            title="Start or resume (Space)"
            onClick={() => send({ type: 'run.start' })}
          />
          <ToolButton
            icon="pause"
            label="Pause"
            active={state?.paused ?? false}
            title="Pause (Space)"
            onClick={() => send({ type: 'run.pause' })}
          />
          <ToolButton
            icon="step"
            label="Step"
            title="Advance 100 ms while paused"
            onClick={() => send({ type: 'run.step' })}
          />
          <ToolButton icon="reset" label="Reset" title="Reset run (R)" onClick={onReset} />
        </div>
        <div className="tool-separator" />
        <div className="tool-group camera-tools">
          <span className="tool-caption">Camera</span>
          <ToolButton
            icon="orbit"
            label="Orbit"
            active={cameraMode === 'orbit'}
            title="Orbit camera (1)"
            onClick={() => setCameraMode('orbit')}
          />
          <ToolButton
            icon="top"
            label="Top"
            active={cameraMode === 'top'}
            title="Top camera (2)"
            onClick={() => setCameraMode('top')}
          />
          <ToolButton
            icon="chase"
            label="Chase"
            active={cameraMode === 'chase'}
            disabled={viewMode === 'car'}
            title="Chase camera (3)"
            onClick={() => setCameraMode('chase')}
          />
          {viewMode === 'car' ? (
            <ToolButton
              icon="orbit"
              label="Turntable"
              active={turntable}
              title="Auto-rotate the car studio (T)"
              onClick={() => setTurntable((value) => !value)}
            />
          ) : null}
        </div>
        <div className="tool-separator" />
        <div className="tool-group drive-tools">
          <span className="tool-caption">Drive</span>
          <div className="segmented slim">
            <button
              type="button"
              className={controlMode === 'auto' ? 'active' : ''}
              onClick={() => setControlMode('auto')}
            >
              Auto
            </button>
            <button
              type="button"
              className={controlMode === 'manual' ? 'active' : ''}
              onClick={() => setControlMode('manual')}
            >
              Manual
            </button>
          </div>
          <select
            className="controller-select"
            value={autoController}
            disabled={controlMode === 'manual'}
            onChange={(event) => setAutoController(event.target.value as 'gap_follow' | 'wall_follow')}
            title="Lidar-only demonstration controller"
          >
            <option value="gap_follow">Furthest gap</option>
            <option value="wall_follow">Wall follow</option>
          </select>
          <label className="speed-control" title="Maximum target speed">
            <span>Speed</span>
            <input
              type="range"
              min="0.5"
              max="6"
              step="0.1"
              value={speed}
              onChange={(event) => onSpeed(Number(event.target.value))}
            />
            <strong>{num(speed, 1)}</strong>
            <em>m/s</em>
          </label>
        </div>
        <div className="toolbar-spacer" />
        <div className="tool-group">
          <ToolButton
            icon="settings"
            label="Tune"
            title="Edit all vehicle and sensor parameters (Ctrl+Z / Ctrl+Y)"
            onClick={() => openTune(selectionTab)}
          />
          <ToolButton
            icon="reload"
            label="Reload config"
            title="Reload vehicle.yaml, sensors.yaml and scenario.yaml"
            onClick={onReload}
          />
        </div>
      </div>

      <div
        className={`workspace${outlinerOpen ? '' : ' outliner-collapsed'}${detailsOpen ? '' : ' details-collapsed'}`}
      >
        {outlinerOpen ? (
          <Outliner
            hello={hello}
            selection={selection}
            onSelect={setSelection}
            onCollapse={() => setOutlinerOpen(false)}
          />
        ) : (
          <PanelRail side="left" label="Outliner" onExpand={() => setOutlinerOpen(true)} />
        )}

        <main className="viewport-pane">
          <div className="viewport-header">
            <ViewportTabs viewMode={viewMode} onChange={setViewMode} />
            <div className="viewport-header-right">
              <span>{cameraMode} camera</span>
              <span className="separator" />
              <span>{hello?.scenario.realism ?? '—'} sensors</span>
            </div>
          </div>
          <div className="viewport-body">
            <SceneCanvas
              hello={hello}
              state={state}
              scan={scan}
              viewMode={viewMode}
              cameraMode={cameraMode}
              layers={layers}
              turntable={turntable}
            />
            <ViewportHud hello={hello} state={state} viewMode={viewMode} scan={scan} />
            <GapDebugPanel state={state} layers={layers} />
            {hello?.warnings.length ? (
              <div className="warning-strip">{hello.warnings.join(' · ')}</div>
            ) : null}
            <div className="viewport-axis" aria-hidden="true">
              <span className="axis-x">X</span>
              <span className="axis-y">Y</span>
              <span className="axis-z">Z</span>
            </div>
          </div>
          <div className="viewport-footer">
            <div className="layer-toggles">
              {(
                [
                  ['scan', 'Lidar'],
                  ['trail', 'Trail'],
                  ['odometry', 'Odom'],
                  ['sensors', 'Sensors'],
                  ['coverage', 'Coverage'],
                  ['labels', 'Labels'],
                  ['gap', 'Gap solver'],
                ] as Array<[keyof DebugLayers, string]>
              ).map(([key, label]) => (
                <label key={key} className={`layer-toggle ${viewMode === 'car' && (key === 'scan' || key === 'trail' || key === 'odometry' || key === 'gap') ? 'disabled' : ''}`}>
                  <input
                    type="checkbox"
                    checked={layers[key]}
                    disabled={viewMode === 'car' && (key === 'scan' || key === 'trail' || key === 'odometry' || key === 'gap')}
                    onChange={(event) =>
                      setLayers((current) => ({ ...current, [key]: event.target.checked }))
                    }
                  />
                  {label}
                </label>
              ))}
            </div>
            <div className="viewport-footer-hint">
              {viewMode === 'car'
                ? 'Vehicle isolated from track geometry · sensor mounts shown in body frame'
                : 'Drag to orbit · scroll to zoom · right-drag to pan'}
            </div>
          </div>
        </main>

        {detailsOpen ? (
          <DetailsPanel
            hello={hello}
            state={state}
            selection={selection}
            speed={speed}
            onSpeed={onSpeed}
            onOpenTune={() => openTune(selectionTab)}
            onGenerateTrack={onGenerateTrack}
            onDefaultTrack={onDefaultTrack}
            onCollapse={() => setDetailsOpen(false)}
          />
        ) : (
          <PanelRail side="right" label="Details" onExpand={() => setDetailsOpen(true)} />
        )}
      </div>

      <footer className="statusbar">
        <span className={`status-ready ${status === 'open' ? 'ok' : ''}`}>
          <i /> {status === 'open' ? 'READY' : statusLabel(status).toUpperCase()}
        </span>
        <span className="status-divider" />
        <span>{hello?.vehicle.name ?? 'no vehicle'}</span>
        <span className="status-divider" />
        <span>{hello ? `${hello.sensors.lidar.beams} beams · ${hello.sensors.lidar.rate_hz} Hz lidar` : 'lidar offline'}</span>
        <span className="status-divider" />
        <span>{hello ? `${hello.sensors.mounts.length} sensors` : '—'}</span>
        <span className="status-spacer" />
        <span>t = {num(state?.t, 1)} s</span>
        <span className="status-divider" />
        <span>{state?.paused ? 'paused' : state?.finished ? 'finished' : 'running'}</span>
        <span className="status-divider" />
        <span>F3 · preferences</span>
      </footer>

      <TuningPanel
        open={tuningOpen}
        initialTab={tuningTab}
        hello={hello}
        configResult={configResult}
        onClose={() => setTuningOpen(false)}
        onSend={send}
      />
      {toast ? <div className="toast">{toast}</div> : null}
    </div>
  );
}
