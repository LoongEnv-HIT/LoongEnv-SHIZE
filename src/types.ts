import React from 'react';

export interface PluginMetadata {
  id: string;
  name: string;
  version: string;
  author: string;
  description: string;
}

export interface PluginComponentProps {
  data: any;
  onAction: (action: string, payload?: any) => void;
}

export interface Plugin {
  metadata: PluginMetadata;
  icon: React.ReactNode;
  stepTitle: string;
  component: React.FC<PluginComponentProps>;
  techStack: string[];
  inputSchema: any;
  outputSchema: any;
}

export type RobotRuntime = 'mujoco' | 'urdf' | 'auto';
export type RobotSourceType = 'MJCF' | 'URDF' | 'AUTO';
export type RobotMaterialProfile = 'industrial_orange_dark' | 'neutral_metallic';

export interface RobotJointLimitRow {
  joint: string;
  position: string;
  velocity?: string;
  torque?: string;
}

export interface RobotAsset {
  id: string;
  name: string;
  runtime: RobotRuntime;
  sourceType: RobotSourceType;
  sourcePath: string;
  sourceCandidates?: string[];
  publicRoot: string;
  description: string;
  jointCount: number;
  jointLabels: string[];
  defaultJointValues: number[];
  engineLabel: string;
  techStack: string[];
  displayScale?: number | [number, number, number];
  displayOffset?: [number, number, number];
  cameraTarget?: [number, number, number];
  cameraDistanceMultiplier?: number;
  materialProfile?: RobotMaterialProfile;
  jointLimits?: RobotJointLimitRow[];
  mujocoAssetFiles?: string[];
}

export type PlanningSsMode = 'arc' | 'time';

export interface PlanningLimits {
  velocity: number[];
  acceleration: number[];
}

export interface PlanningConfig {
  resample_points: number;
  smooth_passes: number;
  smooth_blend: number;
  dt: number;
  v_max: number;
  a_max: number;
  j_max: number;
  minima_count: number | null;
  minima_gap: number;
  minima_threshold_mode: 'mean' | 'fixed' | 'none';
  minima_max_value: number | null;
  junction_velocity_scale: number;
}

export interface PlanningRequest {
  waypointText: string;
  fileName: string;
  limits: PlanningLimits;
  config: PlanningConfig;
  ssMode: PlanningSsMode;
  playbackDtMs: number;
}

export interface PlanningSummary {
  inputFileName: string;
  duration: number;
  samples: number;
  uniformExportDtS: number;
  uniformExportSamples: number;
  ssMode: PlanningSsMode;
  minimaThresholdMode: 'mean' | 'fixed' | 'none';
  minimaIdx: number[];
  minimaV: number[];
  segmentPoints: number[];
  segmentTargets: number[];
  segmentTargetVelocities: number[];
  config: PlanningConfig;
}

export interface PlanningTrajectory {
  duration: number;
  ts: number[];
  qs: number[][];
  qds: number[][];
  qdds: number[][];
  tUniform: number[];
  qUniform: number[][];
}

export interface PlanningMvc {
  gridpoints: number[];
  mvcUpper: number[];
  minimaIdx: number[];
  minimaV: number[];
  chart: Array<{ x: number; y: number }>;
}

export interface PlanningPathProfile {
  t: number[];
  s: number[];
  sd: number[];
  sdd: number[];
  chart: Array<{ t: number; s: number; sd: number; sdd: number }>;
}

export interface PlanningWaypoints {
  raw: number[][];
  smooth: number[][];
}

export interface PlanningResult {
  summary: PlanningSummary;
  trajectory: PlanningTrajectory;
  mvc: PlanningMvc;
  pathProfile: PlanningPathProfile;
  waypoints: PlanningWaypoints;
  warnings: string[];
  logs: string[];
  stderr?: string;
}

export interface PlanningHealth {
  ok: boolean;
  python?: {
    executable: string;
    version: string;
  };
  dependencies?: Record<string, boolean>;
  er15Factory?: {
    ok: boolean;
    error: string | null;
  };
  sidecar?: {
    pythonBin: string;
    bridgeScript: string;
  };
  error?: string;
}

export interface PlaybackState {
  isPlaying: boolean;
  currentTimeSec: number;
  speed: number;
  loop: boolean;
}

export interface WorkspacePluginData {
  planningResult: PlanningResult | null;
  setPlanningResult: React.Dispatch<React.SetStateAction<PlanningResult | null>>;
  playback: PlaybackState;
  setPlayback: React.Dispatch<React.SetStateAction<PlaybackState>>;
  robotAssets: RobotAsset[];
  selectedRobotId: string;
  setSelectedRobotId: React.Dispatch<React.SetStateAction<string>>;
  apiBase?: string;
}
