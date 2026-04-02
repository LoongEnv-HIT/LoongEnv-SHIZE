import React, { useEffect, useMemo, useState } from 'react';
import { Activity, BrainCircuit, Play, Pause, RotateCcw, Upload, AlertTriangle, CheckCircle2 } from 'lucide-react';
import { motion } from 'motion/react';
import {
  CartesianGrid,
  Line,
  LineChart,
  ResponsiveContainer,
  Tooltip,
  XAxis,
  YAxis,
} from 'recharts';
import {
  PlaybackState,
  PlanningConfig,
  PlanningHealth,
  PlanningRequest,
  PlanningResult,
  Plugin,
} from '../types';
import { CollapsibleSection } from '../components/CollapsibleSection';

const POSITION_LIMITS = [
  { joint: 'J1', range: '-2.967 ~ 2.967 rad' },
  { joint: 'J2', range: '-2.7925 ~ 1.5708 rad' },
  { joint: 'J3', range: '-1.4835 ~ 3.0543 rad' },
  { joint: 'J4', range: '-3.316 ~ 3.316 rad' },
  { joint: 'J5', range: '-2.2689 ~ 2.2689 rad' },
  { joint: 'J6', range: '-6.2832 ~ 6.2832 rad' },
] as const;

const DEFAULT_CONFIG: PlanningConfig = {
  resample_points: 1000,
  smooth_passes: 60,
  smooth_blend: 0.49,
  dt: 0.001,
  v_max: 0.02,
  a_max: 0.02,
  j_max: 0.04,
  minima_count: null,
  minima_gap: 12,
  minima_threshold_mode: 'mean',
  minima_max_value: null,
  junction_velocity_scale: 0.8,
};

const DEFAULT_PLAYBACK_STATE: PlaybackState = {
  isPlaying: false,
  currentTimeSec: 0,
  speed: 1,
  loop: true,
};

const REQUIRED_HEADERS = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'];

type DesignPluginData = {
  planningResult: PlanningResult | null;
  setPlanningResult: React.Dispatch<React.SetStateAction<PlanningResult | null>>;
  playback: PlaybackState;
  setPlayback: React.Dispatch<React.SetStateAction<PlaybackState>>;
  apiBase?: string;
};

function parseNumericTokens(line: string) {
  return line
    .trim()
    .split(/[\s,]+/)
    .filter(Boolean)
    .map((token) => Number(token));
}

function validateWaypointText(fileName: string, text: string): string | null {
  const trimmed = text.trim();
  if (!trimmed) {
    return 'Waypoint file is empty.';
  }

  const lines = trimmed.split(/\r?\n/).map((line) => line.trim()).filter(Boolean);
  if (lines.length < 2) {
    return 'Waypoint file must include at least two rows.';
  }

  const lowerName = fileName.toLowerCase();
  const looksLikeCsv = lowerName.endsWith('.csv') || lines[0].includes(',');

  if (looksLikeCsv) {
    const header = lines[0]
      .replace(/^\uFEFF/, '')
      .split(',')
      .map((item) => item.trim());
    const missing = REQUIRED_HEADERS.filter((column) => !header.includes(column));
    if (missing.length > 0) {
      return `CSV is missing required headers: ${missing.join(', ')}`;
    }

    const indexByHeader = new Map<string, number>();
    header.forEach((column, index) => indexByHeader.set(column, index));

    for (let i = 1; i < Math.min(lines.length, 40); i += 1) {
      const row = lines[i].split(',').map((item) => item.trim());
      for (const column of REQUIRED_HEADERS) {
        const columnIndex = indexByHeader.get(column);
        if (columnIndex == null || columnIndex >= row.length) {
          return `Row ${i + 1} does not contain ${column}.`;
        }
        const value = Number(row[columnIndex]);
        if (!Number.isFinite(value)) {
          return `Row ${i + 1} contains non-numeric ${column}.`;
        }
      }
    }

    return null;
  }

  for (let i = 0; i < Math.min(lines.length, 40); i += 1) {
    const values = parseNumericTokens(lines[i]);
    if (values.length !== 6) {
      return `Line ${i + 1} must contain exactly 6 joint values.`;
    }
    if (values.some((value) => !Number.isFinite(value))) {
      return `Line ${i + 1} contains non-numeric values.`;
    }
  }

  return null;
}

export const DesignPlugin: Plugin = {
  metadata: {
    id: 'design',
    name: 'Trajectory Planning',
    description: 'Fixed TOPPRA + MVC + segmented Ruckig planning for ER15 with live 3D playback.',
    version: '2.0.0',
    author: 'LoongEnv Planning',
  },
  icon: <BrainCircuit className="w-5 h-5" />,
  stepTitle: 'Trajectory Planning',
  techStack: ['Python', 'TOPPRA', 'Ruckig', 'Express', 'MuJoCo WASM'],
  inputSchema: {
    file: 'CSV(Titled J1..J6) or TXT(6 columns)',
    limits: 'velocity[6], acceleration[6]',
    ssMode: 'arc | time',
  },
  outputSchema: {
    summary: 'duration, minima, segments',
    trajectory: 'tUniform + qUniform + qs/qds/qdds',
    profiles: 'mvcUpper + pathProfile',
  },
  component: ({ data, onAction }) => {
    const pluginData = (data ?? {}) as Partial<DesignPluginData>;
    const planningResult = pluginData.planningResult ?? null;
    const setPlanningResult = pluginData.setPlanningResult;
    const playback = pluginData.playback ?? DEFAULT_PLAYBACK_STATE;
    const setPlayback = pluginData.setPlayback;
    const apiBase = pluginData.apiBase ?? '/api';

    const [fileName, setFileName] = useState('');
    const [waypointText, setWaypointText] = useState('');
    const [validationError, setValidationError] = useState<string | null>(null);
    const [velocityLimits, setVelocityLimits] = useState<number[]>(Array(6).fill(1));
    const [accelerationLimits, setAccelerationLimits] = useState<number[]>(Array(6).fill(10));
    const [config, setConfig] = useState<PlanningConfig>(DEFAULT_CONFIG);
    const [ssMode, setSsMode] = useState<'arc' | 'time'>('arc');
    const [playbackDtMs, setPlaybackDtMs] = useState(4);
    const [health, setHealth] = useState<PlanningHealth | null>(null);
    const [isRunning, setIsRunning] = useState(false);
    const [runtimeError, setRuntimeError] = useState<string | null>(null);
    const [runtimeLogs, setRuntimeLogs] = useState<string[]>([]);

    const canRun = !isRunning && waypointText.length > 0 && !validationError;
    const duration = planningResult?.trajectory.duration ?? 0;

    const mvcChartData = useMemo(
      () => (planningResult?.mvc.chart ?? []).map((item) => ({ s: item.x, mvc: item.y })),
      [planningResult],
    );

    const profileChartData = useMemo(
      () =>
        (planningResult?.pathProfile.chart ?? []).map((item) => ({
          t: item.t,
          s: item.s,
          sd: item.sd,
        })),
      [planningResult],
    );

    useEffect(() => {
      let disposed = false;

      async function loadHealth() {
        try {
          const response = await fetch(`${apiBase}/design/planning/health`);
          const payload = (await response.json()) as PlanningHealth;
          if (!disposed) {
            setHealth(payload);
          }
        } catch (error) {
          if (!disposed) {
            setHealth({
              ok: false,
              error: error instanceof Error ? error.message : String(error),
            });
          }
        }
      }

      void loadHealth();
      return () => {
        disposed = true;
      };
    }, [apiBase]);

    const updatePlayback = (updater: PlaybackState | ((previous: PlaybackState) => PlaybackState)) => {
      if (!setPlayback) return;
      if (typeof updater === 'function') {
        setPlayback((previous) => (updater as (previous: PlaybackState) => PlaybackState)(previous));
      } else {
        setPlayback(updater);
      }
    };

    const updateConfig = <K extends keyof PlanningConfig>(key: K, value: PlanningConfig[K]) => {
      setConfig((previous) => ({ ...previous, [key]: value }));
    };

    const handleFileChange = async (event: React.ChangeEvent<HTMLInputElement>) => {
      const file = event.target.files?.[0];
      if (!file) {
        return;
      }
      const text = await file.text();
      const error = validateWaypointText(file.name, text);
      setFileName(file.name);
      setWaypointText(text);
      setValidationError(error);
    };

    const handleRunPlanning = async () => {
      const error = validateWaypointText(fileName || 'waypoints.csv', waypointText);
      setValidationError(error);
      if (error) {
        return;
      }
      if (health && !health.ok) {
        setRuntimeError(`Planning sidecar is unavailable: ${health.error ?? 'health check failed'}`);
        return;
      }

      const request: PlanningRequest = {
        waypointText,
        fileName: fileName || 'waypoints.csv',
        limits: {
          velocity: velocityLimits,
          acceleration: accelerationLimits,
        },
        config,
        ssMode,
        playbackDtMs,
      };

      setRuntimeError(null);
      setRuntimeLogs([]);
      setIsRunning(true);

      try {
        const response = await fetch(`${apiBase}/design/planning/run`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify(request),
        });
        const responseText = await response.text();
        let payload: any = null;
        try {
          payload = responseText ? JSON.parse(responseText) : null;
        } catch {
          if (!response.ok) {
            throw new Error(
              `Planning request failed with status ${response.status}. Sidecar may be unreachable or returned invalid output.`,
            );
          }
          throw new Error('Planning response is not valid JSON.');
        }
        if (!response.ok || !payload?.ok) {
          throw new Error(payload?.error || `Planning failed with status ${response.status}`);
        }

        if (setPlanningResult) {
          setPlanningResult(payload as PlanningResult);
        }

        updatePlayback((previous) => ({
          ...previous,
          isPlaying: false,
          currentTimeSec: 0,
        }));

        setRuntimeLogs(payload.logs ?? []);
        onAction('COMPLETE', 'DESIGN_FINISHED');
      } catch (error) {
        if (setPlanningResult) {
          setPlanningResult(null);
        }
        updatePlayback((previous) => ({
          ...previous,
          isPlaying: false,
          currentTimeSec: 0,
        }));
        setRuntimeError(error instanceof Error ? error.message : String(error));
      } finally {
        setIsRunning(false);
      }
    };

    return (
      <div className="flex h-full flex-col bg-white text-[#333333]">
        <div className="flex-1 space-y-4 overflow-y-auto p-4 custom-scrollbar">
          <motion.div initial={{ opacity: 0, y: 6 }} animate={{ opacity: 1, y: 0 }} className="space-y-4">
            <CollapsibleSection title="1) Waypoint File" defaultOpen>
              <div className="space-y-3">
                <label className="flex cursor-pointer items-center justify-center gap-2 border border-dashed border-[#9ab7d7] bg-[#f5faff] px-4 py-3 text-[11px] font-bold text-[#005fa5] hover:bg-[#edf6ff]">
                  <Upload className="h-4 w-4" />
                  Upload CSV/TXT Waypoints
                  <input type="file" accept=".csv,.txt" className="hidden" onChange={handleFileChange} />
                </label>
                <div className="rounded-sm border border-[#e5e5e5] bg-[#fafafa] p-2 text-[11px] font-mono text-[#4b5563]">
                  {fileName ? `Loaded: ${fileName}` : 'No file selected'}
                </div>
                <p className="text-[11px] text-[#6b7280]">
                  CSV must include headers: <span className="font-mono">J1,J2,J3,J4,J5,J6</span>. TXT must contain exactly 6 numeric columns per row.
                </p>
                {validationError && (
                  <div className="flex items-start gap-2 rounded-sm border border-red-200 bg-red-50 px-3 py-2 text-[11px] text-red-700">
                    <AlertTriangle className="mt-0.5 h-4 w-4" />
                    <span>{validationError}</span>
                  </div>
                )}
              </div>
            </CollapsibleSection>

            <CollapsibleSection title="2) Limits & Config" defaultOpen>
              <div className="space-y-4">
                <div className="overflow-hidden rounded-sm border border-[#e5e5e5]">
                  <div className="grid grid-cols-[52px_1fr_1fr_1fr] gap-2 bg-[#f8fafc] px-3 py-2 text-[10px] font-bold uppercase text-[#6b7280]">
                    <span>Joint</span>
                    <span>Position</span>
                    <span>Velocity</span>
                    <span>Acceleration</span>
                  </div>
                  {POSITION_LIMITS.map((joint, index) => (
                    <div key={joint.joint} className="grid grid-cols-[52px_1fr_1fr_1fr] items-center gap-2 border-t border-[#f1f5f9] px-3 py-2 text-[11px]">
                      <span className="font-mono font-bold">{joint.joint}</span>
                      <span className="font-mono text-[#6b7280]">{joint.range}</span>
                      <input
                        type="number"
                        min={0.001}
                        step={0.01}
                        value={velocityLimits[index]}
                        onChange={(event) => {
                          const value = Number(event.target.value);
                          setVelocityLimits((previous) => {
                            const next = [...previous];
                            next[index] = Number.isFinite(value) ? value : previous[index];
                            return next;
                          });
                        }}
                        className="w-full border border-[#d1d5db] bg-white px-2 py-1 font-mono"
                      />
                      <input
                        type="number"
                        min={0.001}
                        step={0.1}
                        value={accelerationLimits[index]}
                        onChange={(event) => {
                          const value = Number(event.target.value);
                          setAccelerationLimits((previous) => {
                            const next = [...previous];
                            next[index] = Number.isFinite(value) ? value : previous[index];
                            return next;
                          });
                        }}
                        className="w-full border border-[#d1d5db] bg-white px-2 py-1 font-mono"
                      />
                    </div>
                  ))}
                </div>

                <div className="grid grid-cols-2 gap-2 text-[11px]">
                  <label className="space-y-1">
                    <span className="text-[#6b7280]">ssMode</span>
                    <select value={ssMode} onChange={(event) => setSsMode(event.target.value as 'arc' | 'time')} className="w-full border border-[#d1d5db] px-2 py-1">
                      <option value="arc">arc</option>
                      <option value="time">time</option>
                    </select>
                  </label>
                  <label className="space-y-1">
                    <span className="text-[#6b7280]">Playback dt (ms)</span>
                    <input type="number" value={playbackDtMs} step={0.5} min={0.5} onChange={(event) => setPlaybackDtMs(Number(event.target.value) || 4)} className="w-full border border-[#d1d5db] px-2 py-1 font-mono" />
                  </label>
                  <label className="space-y-1">
                    <span className="text-[#6b7280]">resample_points</span>
                    <input type="number" value={config.resample_points} onChange={(event) => updateConfig('resample_points', Number(event.target.value) || 1000)} className="w-full border border-[#d1d5db] px-2 py-1 font-mono" />
                  </label>
                  <label className="space-y-1">
                    <span className="text-[#6b7280]">smooth_passes</span>
                    <input type="number" value={config.smooth_passes} onChange={(event) => updateConfig('smooth_passes', Number(event.target.value) || 60)} className="w-full border border-[#d1d5db] px-2 py-1 font-mono" />
                  </label>
                  <label className="space-y-1">
                    <span className="text-[#6b7280]">smooth_blend</span>
                    <input type="number" value={config.smooth_blend} step={0.01} onChange={(event) => updateConfig('smooth_blend', Number(event.target.value) || 0.49)} className="w-full border border-[#d1d5db] px-2 py-1 font-mono" />
                  </label>
                  <label className="space-y-1">
                    <span className="text-[#6b7280]">dt (s)</span>
                    <input type="number" value={config.dt} step={0.0005} onChange={(event) => updateConfig('dt', Number(event.target.value) || 0.001)} className="w-full border border-[#d1d5db] px-2 py-1 font-mono" />
                  </label>
                  <label className="space-y-1">
                    <span className="text-[#6b7280]">v_max</span>
                    <input type="number" value={config.v_max} step={0.001} onChange={(event) => updateConfig('v_max', Number(event.target.value) || 0.02)} className="w-full border border-[#d1d5db] px-2 py-1 font-mono" />
                  </label>
                  <label className="space-y-1">
                    <span className="text-[#6b7280]">a_max</span>
                    <input type="number" value={config.a_max} step={0.001} onChange={(event) => updateConfig('a_max', Number(event.target.value) || 0.02)} className="w-full border border-[#d1d5db] px-2 py-1 font-mono" />
                  </label>
                  <label className="space-y-1">
                    <span className="text-[#6b7280]">j_max</span>
                    <input type="number" value={config.j_max} step={0.001} onChange={(event) => updateConfig('j_max', Number(event.target.value) || 0.04)} className="w-full border border-[#d1d5db] px-2 py-1 font-mono" />
                  </label>
                  <label className="space-y-1">
                    <span className="text-[#6b7280]">minima_gap</span>
                    <input type="number" value={config.minima_gap} onChange={(event) => updateConfig('minima_gap', Number(event.target.value) || 12)} className="w-full border border-[#d1d5db] px-2 py-1 font-mono" />
                  </label>
                  <label className="space-y-1">
                    <span className="text-[#6b7280]">minima_threshold_mode</span>
                    <select value={config.minima_threshold_mode} onChange={(event) => updateConfig('minima_threshold_mode', event.target.value as PlanningConfig['minima_threshold_mode'])} className="w-full border border-[#d1d5db] px-2 py-1">
                      <option value="mean">mean</option>
                      <option value="fixed">fixed</option>
                      <option value="none">none</option>
                    </select>
                  </label>
                  <label className="space-y-1">
                    <span className="text-[#6b7280]">junction_velocity_scale</span>
                    <input type="number" value={config.junction_velocity_scale} step={0.05} onChange={(event) => updateConfig('junction_velocity_scale', Number(event.target.value) || 0.8)} className="w-full border border-[#d1d5db] px-2 py-1 font-mono" />
                  </label>
                </div>
              </div>
            </CollapsibleSection>

            <CollapsibleSection title="3) Runtime Status" defaultOpen>
              <div className="space-y-3 text-[11px]">
                <div className="rounded-sm border border-[#e5e7eb] bg-[#f8fafc] p-3">
                  <div className="flex items-center gap-2 font-bold text-[#374151]">
                    <Activity className="h-4 w-4 text-[#007acc]" />
                    Sidecar Health
                  </div>
                  <div className="mt-2 space-y-1 text-[#4b5563]">
                    <div>
                      Python: {health?.python?.executable ?? 'unknown'}
                    </div>
                    <div>
                      ER15 Factory: {health?.er15Factory?.ok ? 'OK' : 'Not Ready'}
                    </div>
                    {health?.error && <div className="text-red-600">Error: {health.error}</div>}
                  </div>
                </div>

                {runtimeError && (
                  <div className="rounded-sm border border-red-200 bg-red-50 p-3 text-red-700">
                    {runtimeError}
                  </div>
                )}

                {runtimeLogs.length > 0 && (
                  <div className="rounded-sm border border-[#e5e7eb] bg-white p-3 font-mono text-[10px] text-[#374151]">
                    {runtimeLogs.map((line, index) => (
                      <div key={`log-${index}`}>{line}</div>
                    ))}
                  </div>
                )}
              </div>
            </CollapsibleSection>

            <CollapsibleSection title="4) Result & Playback" defaultOpen>
              {planningResult ? (
                <div className="space-y-4 text-[11px]">
                  <div className="grid grid-cols-2 gap-2">
                    <div className="rounded-sm border border-[#e5e7eb] bg-[#f8fafc] p-2">
                      <div className="text-[#6b7280]">Duration</div>
                      <div className="font-mono text-[#111827]">{planningResult.summary.duration.toFixed(4)} s</div>
                    </div>
                    <div className="rounded-sm border border-[#e5e7eb] bg-[#f8fafc] p-2">
                      <div className="text-[#6b7280]">Minima Count</div>
                      <div className="font-mono text-[#111827]">{planningResult.summary.minimaIdx.length}</div>
                    </div>
                  </div>

                  <div className="rounded-sm border border-[#e5e7eb] bg-white p-2">
                    <div className="mb-2 flex items-center justify-between">
                      <span className="font-bold text-[#374151]">Playback</span>
                      <span className="font-mono text-[#4b5563]">
                        {playback.currentTimeSec.toFixed(3)} / {duration.toFixed(3)} s
                      </span>
                    </div>
                    <div className="mb-2 flex items-center gap-2">
                      <button
                        onClick={() => updatePlayback((previous) => ({ ...previous, isPlaying: !previous.isPlaying }))}
                        className="inline-flex items-center gap-1 border border-[#d1d5db] bg-white px-2 py-1 hover:bg-[#f3f4f6]"
                      >
                        {playback.isPlaying ? <Pause className="h-3.5 w-3.5" /> : <Play className="h-3.5 w-3.5" />}
                        {playback.isPlaying ? 'Pause' : 'Play'}
                      </button>
                      <button
                        onClick={() => updatePlayback((previous) => ({ ...previous, isPlaying: false, currentTimeSec: 0 }))}
                        className="inline-flex items-center gap-1 border border-[#d1d5db] bg-white px-2 py-1 hover:bg-[#f3f4f6]"
                      >
                        <RotateCcw className="h-3.5 w-3.5" />
                        Reset
                      </button>
                      <select
                        value={playback.speed}
                        onChange={(event) => updatePlayback((previous) => ({ ...previous, speed: Number(event.target.value) || 1 }))}
                        className="border border-[#d1d5db] bg-white px-2 py-1"
                      >
                        <option value={0.25}>0.25x</option>
                        <option value={0.5}>0.5x</option>
                        <option value={1}>1x</option>
                        <option value={1.5}>1.5x</option>
                        <option value={2}>2x</option>
                      </select>
                      <label className="inline-flex items-center gap-1">
                        <input
                          type="checkbox"
                          checked={playback.loop}
                          onChange={(event) => updatePlayback((previous) => ({ ...previous, loop: event.target.checked }))}
                        />
                        Loop
                      </label>
                    </div>
                    <input
                      type="range"
                      min={0}
                      max={Math.max(duration, 0.001)}
                      step={0.001}
                      value={Math.min(playback.currentTimeSec, Math.max(duration, 0.001))}
                      onChange={(event) => {
                        const nextTime = Number(event.target.value) || 0;
                        updatePlayback((previous) => ({ ...previous, currentTimeSec: nextTime, isPlaying: false }));
                      }}
                      className="w-full"
                    />
                  </div>

                  <div className="h-44 rounded-sm border border-[#e5e7eb] bg-white p-2">
                    <div className="mb-1 font-bold text-[#374151]">MVC Upper Envelope</div>
                    <ResponsiveContainer width="100%" height="100%">
                      <LineChart data={mvcChartData} margin={{ top: 5, right: 8, bottom: 5, left: 0 }}>
                        <CartesianGrid strokeDasharray="3 3" stroke="#eef2f7" />
                        <XAxis dataKey="s" tick={{ fontSize: 10 }} />
                        <YAxis tick={{ fontSize: 10 }} />
                        <Tooltip />
                        <Line type="monotone" dataKey="mvc" stroke="#007acc" dot={false} strokeWidth={1.6} />
                      </LineChart>
                    </ResponsiveContainer>
                  </div>

                  <div className="h-44 rounded-sm border border-[#e5e7eb] bg-white p-2">
                    <div className="mb-1 font-bold text-[#374151]">Path Profile</div>
                    <ResponsiveContainer width="100%" height="100%">
                      <LineChart data={profileChartData} margin={{ top: 5, right: 8, bottom: 5, left: 0 }}>
                        <CartesianGrid strokeDasharray="3 3" stroke="#eef2f7" />
                        <XAxis dataKey="t" tick={{ fontSize: 10 }} />
                        <YAxis tick={{ fontSize: 10 }} />
                        <Tooltip />
                        <Line type="monotone" dataKey="s" stroke="#059669" dot={false} strokeWidth={1.4} />
                        <Line type="monotone" dataKey="sd" stroke="#dc2626" dot={false} strokeWidth={1.2} />
                      </LineChart>
                    </ResponsiveContainer>
                  </div>
                </div>
              ) : (
                <div className="rounded-sm border border-dashed border-[#cbd5e1] bg-[#f8fafc] p-5 text-center text-[11px] text-[#64748b]">
                  Run planning to load summary, curves, and 3D playback trajectory.
                </div>
              )}
            </CollapsibleSection>
          </motion.div>
        </div>

        <div className="border-t border-[#e5e5e5] bg-[#f3f3f3] p-4">
          <button
            onClick={handleRunPlanning}
            disabled={!canRun}
            className="flex w-full items-center justify-center gap-2 rounded-sm bg-[#007acc] py-2 text-[11px] font-bold uppercase tracking-wider text-white transition-all enabled:hover:bg-[#0062a3] disabled:cursor-not-allowed disabled:bg-[#9ca3af]"
          >
            {isRunning ? <Activity className="h-3.5 w-3.5 animate-spin" /> : <CheckCircle2 className="h-3.5 w-3.5" />}
            {isRunning ? 'Planning...' : 'Run Fixed TOPPRA MVC Ruckig'}
          </button>
        </div>
      </div>
    );
  },
};
