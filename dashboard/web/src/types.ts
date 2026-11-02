export type Vec2 = [number, number];
export type Vec3 = [number, number, number];

export type Realism = 'ideal' | 'realistic' | 'harsh';
export type ViewMode = 'track' | 'car';
export type CameraMode = 'orbit' | 'top' | 'chase';
export type JsonObject = Record<string, unknown>;

export interface ConfigPayload {
  vehicle: JsonObject;
  sensors: JsonObject;
}

export interface Dimensions {
  length_m: number;
  width_m: number;
  height_m: number;
  ground_clearance_m: number;
}

export interface BodyPart {
  name: string;
  shape: 'box' | 'cylinder' | 'sphere';
  position_m: Vec3;
  rpy_deg: Vec3;
  size_m: Vec3;
  color: string;
  opacity: number;
  visible: boolean;
  collision: boolean;
}

export interface WheelSpec {
  name: string;
  position_m: Vec3;
  radius_m: number;
  width_m: number;
  mass_kg: number;
  steered: boolean;
  driven: boolean;
}

export interface VehicleSpec {
  name: string;
  description: string;
  scale: number;
  dimensions: Dimensions;
  mass_kg: number;
  chassis_mass_kg: number;
  center_of_mass_m: Vec3;
  wheelbase_m: number;
  track_width_m: number;
  wheel_radius_m: number;
  wheel_width_m: number;
  wheel_mass_kg: number;
  wheel_offset_z_m: number;
  ride_height_m: number;
  drive: string;
  limits: {
    max_length_m: number;
    max_width_m: number;
    max_height_m: number;
    min_ground_clearance_m: number;
  };
  steering: {
    max_angle_deg: number;
    rate_deg_s: number;
    deadband_deg: number;
    backlash_deg: number;
    latency_ms: number;
  };
  motor: {
    max_torque_nm: number;
    max_rpm: number;
    gear_ratio: number;
    current_limit_a: number;
    torque_lag_ms: number;
  };
  battery: {
    cells: number;
    nominal_voltage_v: number;
    capacity_mah: number;
    internal_resistance_ohm: number;
  };
  tires: {
    friction_front: number;
    friction_rear: number;
    rolling_resistance: number;
  };
  suspension: {
    stiffness: number;
    damping: number;
    max_travel_m: number;
  };
  aero: {
    drag_coefficient: number;
    frontal_area_m2: number;
  };
  structure: BodyPart[];
  wheels: WheelSpec[];
}

export interface SensorMount {
  id: string;
  kind: 'lidar' | 'ultrasonic' | 'imu';
  name: string;
  frame_id: string;
  position_m: Vec3;
  rpy_deg: Vec3;
  ground_height_m: number;
  range_m: number;
  fov_deg: number;
  color: string;
  summary: string;
  characteristics: Record<string, unknown>;
}

export interface LidarSpec {
  frame_id: string;
  beams: number;
  rate_hz: number;
  min_range_m: number;
  max_range_m: number;
  angular_resolution_deg: number;
  scan_direction: string;
  mount: { position_m: Vec3; rpy_deg: Vec3 };
  noise: {
    range_sigma_m: number;
    range_bias_m: number;
    range_sigma_per_m: number;
    angle_sigma_deg: number;
    angle_offset_deg: number;
    range_scale_error: number;
  };
  latency_ms: number;
  latency_jitter_ms: number;
  dropped_scan_probability: number;
  wall_height_warning_m: number;
}

export interface SonarSpec {
  name: string;
  frame_id: string;
  mount: { position_m: Vec3; rpy_deg: Vec3 };
}

export interface UltrasonicSpec {
  rate_hz: number;
  cone_angle_deg: number;
  rays: number;
  min_range_m: number;
  max_range_m: number;
  range_sigma_m: number;
  range_sigma_ratio: number;
  latency_ms: number;
  cross_talk_probability: number;
  specular_dropout_probability: number;
  sensors: SonarSpec[];
}

export interface ImuSpec {
  frame_id: string;
  rate_hz: number;
  latency_ms: number;
  mount: { position_m: Vec3; rpy_deg: Vec3 };
  gyro: Record<string, number>;
  accel: Record<string, number>;
  mounting_misalignment_deg: number;
}

export interface EncoderSpec {
  rate_hz: number;
  ticks_per_rev: number;
  wheel_radius_error: number;
  track_width_error: number;
  timestamp_jitter_ms: number;
  dropout_probability: number;
}

export interface SensorsSpec {
  name: string;
  description: string;
  lidar: LidarSpec;
  ultrasonic: UltrasonicSpec;
  imu: ImuSpec;
  encoders: EncoderSpec;
  mounts: SensorMount[];
}

export interface TrackObstacle {
  name: string;
  kind: 'box' | 'cylinder';
  x: number;
  y: number;
  yaw: number;
  length: number;
  width: number;
  height: number;
  radius: number;
}

export interface TrackInfo {
  name: string;
  width_m: number;
  bounds: [number, number, number, number];
  centerline: Vec2[];
  inner: Vec2[];
  outer: Vec2[];
  wall_height_m: number;
  wall_radius_m: number;
  start_distance_m: number;
  track_length_m: number;
  obstacles: TrackObstacle[];
}

export interface HelloMessage {
  type: 'hello';
  protocol: number;
  scenario: {
    name: string;
    description: string;
    realism: Realism;
    laps: number;
    seed: number;
  };
  vehicle: VehicleSpec;
  sensors: SensorsSpec;
  config: ConfigPayload;
  track: TrackInfo;
  track_meta: {
    mode: 'default' | 'random';
    seed: number;
    object_density: number;
    width_m: number;
  };
  race: { laps: number };
  warnings: string[];
  paths: { scenario: string; vehicle: string; sensors: string };
  capabilities: {
    manual_driving: boolean;
    live_reload: boolean;
    car_studio: boolean;
  };
}

export interface StateMessage {
  type: 'state';
  t: number;
  paused: boolean;
  finished: boolean;
  car: {
    x: number;
    y: number;
    theta: number;
    speed: number;
    steering: number;
    yaw_rate: number;
    wheel_speeds: number[];
  };
  odometry: { x: number; y: number; theta: number };
  race: {
    lap: number;
    laps: number;
    lap_time: number;
    last_lap: number | null;
    best_lap: number | null;
    total_time: number;
    lap_times: number[];
    progress_m: number;
    lateral_m: number;
  };
  sensors: {
    lidar_rate_hz: number;
    lidar_beams: number;
    realism: Realism;
    sonars: Array<{ name: string; range_m: number | null; valid: boolean }>;
  };
  control_mode: 'auto' | 'manual';
  auto_controller: 'gap_follow' | 'wall_follow';
  speed_limit: number;
  gap: {
    valid: boolean;
    angle_rad: number;
    steering_rad: number;
    range_m: number;
    clearance_m: number;
    width_rad: number;
    gap_left_rad: number;
    gap_right_rad: number;
    front_sector_rad: number;
    throttle: number;
    min_clearance_m: number;
    profile: Array<[number, number]>;
  };
}

export interface ScanMessage {
  type: 'scan';
  t: number;
  t_available?: number;
  frame_id: string;
  angles: number[];
  ranges: (number | null)[];
  intensities: number[];
}

export interface ConfigResultMessage {
  type: 'config.result';
  ok: boolean;
  saved: boolean;
  message: string;
  detail?: unknown;
}

export interface TrackResultMessage {
  type: 'track.result';
  ok: boolean;
  message: string;
}

export interface DebugLayers {
  scan: boolean;
  trail: boolean;
  odometry: boolean;
  sensors: boolean;
  coverage: boolean;
  labels: boolean;
  gap: boolean;
}

export type ServerMessage =
  | HelloMessage
  | StateMessage
  | ScanMessage
  | ConfigResultMessage
  | TrackResultMessage;
