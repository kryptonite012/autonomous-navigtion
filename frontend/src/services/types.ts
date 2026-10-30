// Shapes of the data sent by the backend (see backend/api and robot/runtime.py).

export type Severity = "debug" | "info" | "warning" | "error" | "critical";

export interface Pose {
  x: number;
  y: number;
  theta: number;
  valid: boolean;
  source: string;
  timestamp: number;
}

export interface SafetyStatus {
  estop_hardware: boolean;
  estop_software: boolean;
  motion_allowed: boolean;
  faults: Record<string, string>;
  warnings: Record<string, string>;
  obstacle_front_m: number | null;
  obstacle_left_m: number | null;
  obstacle_right_m: number | null;
  zone: "clear" | "slow" | "stop";
  stale_sensors: string[];
  last_command_source: string;
  commanded: [number, number];
  applied: [number, number];
  limited_reason: string;
}

export interface Telemetry {
  timestamp: number;
  linear_velocity: number;
  angular_velocity: number;
  commanded: [number, number];
  battery_voltage: number | null;
  battery_percent: number | null;
  ranges: Record<string, number | null>;
  ranges_valid: Record<string, boolean>;
  imu: { accel: number[]; gyro: number[]; temperature: number | null } | null;
  cpu_temperature: number | null;
  cpu_percent: number | null;
  ram_percent: number | null;
  loop_hz: number;
  pose: Pose;
  motor_status: "ok" | "fault" | "disabled";
  latency_ref?: number;
}

export interface NavStatus {
  state: string;
  goal: { x: number; y: number; theta: number | null; name: string | null } | null;
  path: [number, number][];
  distance_remaining: number | null;
  message: string;
  recoveries: number;
  exploring: boolean;
  planner: string;
  named_locations: Record<string, number[]>;
}

export interface MappingStatus {
  width: number;
  height: number;
  resolution: number;
  origin: [number, number];
  free_ratio: number;
  occupied_ratio: number;
  unknown_ratio: number;
  updates: number;
  active: boolean;
  available: boolean;
  experimental: boolean;
  loaded_map: string | null;
  trajectory_points: number;
  source: string | null;
}

export interface MapSnapshot {
  width: number;
  height: number;
  resolution: number;
  origin: [number, number];
  data_rle: [number, number][];
  trajectory: [number, number][];
}

export interface Detection {
  color: string;
  bbox: [number, number, number, number];
  centroid: [number, number];
  area: number;
  confidence: number;
  timestamp: number;
  object_id: number | null;
  distance_m: number | null;
  bearing_rad: number | null;
  solidity: number;
  circularity: number;
}

export interface VisionStatus {
  available: boolean;
  running: boolean;
  fps: number;
  process_ms: number;
  frames: number;
  rejected_frames: number;
  active_tracks: number;
  unique_objects: Record<string, number>;
  last_error: string;
  distance_estimation: boolean;
  simulated: boolean;
}

export interface SortStatus {
  available: boolean;
  running: boolean;
  state: string;
  message: string;
  target: Detection | null;
  bins: Record<string, number>;
  unclassified_bin: number;
  bin_capacity: number;
  bin_counts: Record<string, number>;
  statistics: Record<string, number>;
  success_rate: number | null;
  verification: string;
}

export interface DeviceHealth {
  name: string;
  kind: string;
  state: "ok" | "degraded" | "fault" | "missing" | "disabled";
  message: string;
  simulated: boolean;
  last_update: number;
}

export interface RobotState {
  name: string;
  hardware_mode: "simulation" | "hardware";
  simulated: boolean;
  mode: string;
  uptime_s: number;
  pose: Pose;
  safety: SafetyStatus;
  navigation: NavStatus;
  mapping: MappingStatus;
  vision: VisionStatus;
  sorting: SortStatus;
  capabilities: Record<string, boolean>;
  ground_truth?: { x: number; y: number; theta: number } | null;
}

export interface RobotEvent {
  type: string;
  severity: Severity;
  description: string;
  timestamp: number;
}
