export type Vec2 = [number, number];

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
  track_length_m?: number;
}

export interface HelloMessage {
  type: 'hello';
  protocol: number;
  mode: string;
  scenario: {
    name: string;
    realism: 'ideal' | 'realistic' | 'harsh';
    controller: string;
    race: { laps: number };
  };
  vehicle: {
    name: string;
    length_m: number;
    width_m: number;
    wheelbase_m: number;
    drive: string;
  };
  sensors: {
    lidar: {
      beams: number;
      rate_hz: number;
      max_range_m: number;
      mount: { position_m?: [number, number, number] };
    };
    ultrasonic: string[];
    imu_rate_hz: number;
  };
  track: TrackInfo;
  race: { laps: number };
  slam_cpp: boolean;
  warnings: string[];
  capabilities: {
    manual_driving: boolean;
    slam: boolean;
    replay: boolean;
    batch: boolean;
  };
}

export interface StateMessage {
  type: 'state';
  mode: string;
  t: number;
  paused: boolean;
  finished: boolean;
  car: {
    x: number;
    y: number;
    theta: number;
    speed: number;
    steering?: number;
    yaw_rate?: number;
  };
  race: {
    lap: number;
    laps: number;
    lap_time: number;
    last_lap: number | null;
    best_lap: number | null;
    total_time: number;
    penalties: number;
    progress_m?: number;
    lateral_m?: number;
  };
  sensors: {
    lidar_rate_hz: number;
    lidar_beams: number;
    realism: 'ideal' | 'realistic' | 'harsh';
    scan_seq?: number;
    sonars?: Array<{ name: string; range_m: number | null; valid: boolean }>;
  };
  control_mode?: 'auto' | 'manual';
  auto_controller?: 'wall_follow' | 'pure_pursuit';
}

export interface ScanMessage {
  type: 'scan';
  t: number;
  seq: number;
  frame_id: string;
  angles: number[];
  ranges: (number | null)[];
  realism: 'ideal' | 'realistic' | 'harsh';
}

export type ServerMessage = HelloMessage | StateMessage | ScanMessage;
