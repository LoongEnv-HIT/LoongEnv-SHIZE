import React, { useEffect, useMemo, useState } from 'react';
import {
  ShieldAlert,
  BrainCircuit,
  Activity,
  Zap,
  CheckCircle2,
  Database,
  FileText,
  Search,
} from 'lucide-react';
import { motion } from 'motion/react';
import {
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  ResponsiveContainer,
  AreaChart,
  Area,
  LineChart,
  Line,
  BarChart,
  Bar,
  Legend,
} from 'recharts';
import { getRobotAsset } from '../data/robotCatalog';
import { cn } from '../lib/utils';
import {
  PlanningHealth,
  Plugin,
  WorkspacePluginData,
} from '../types';
import { CollapsibleSection } from '../components/CollapsibleSection';

const FALLBACK_DIAGNOSE_DATA = Array.from({ length: 120 }, (_, i) => ({
  time: i,
  q: Math.sin(i * 0.2) * 1.5 + 2,
  dq: Math.cos(i * 0.2) * 0.8,
  qdd: -Math.sin(i * 0.2) * 0.18,
  tau: Math.sin(i * 0.3) * 5 + 10,
  eeVel: Math.abs(Math.cos(i * 0.16)) * 0.52,
  error: Math.abs(Math.sin(i * 0.13)) * 0.12,
}));

type TelemetryRow = {
  time: number;
  q: number;
  dq: number;
  qdd: number;
  tau: number;
  eeVel: number;
  error: number;
};

type MetricOption = {
  id: 'tau' | 'dq' | 'qdd' | 'eeVel';
  label: string;
};

const METRIC_OPTIONS: MetricOption[] = [
  { id: 'tau', label: 'Joint Torque' },
  { id: 'dq', label: 'Joint Velocity' },
  { id: 'qdd', label: 'Joint Acceleration' },
  { id: 'eeVel', label: 'End Effector Velocity' },
];

const JOINT_LINE_COLORS = ['#007acc', '#16a34a', '#7c3aed', '#d97706', '#dc2626', '#0ea5a8'];

type DataSourceRow = {
  id: string;
  name: string;
  value: string;
  status: 'online' | 'warning' | 'offline';
};

function SourceStatusBadge({ status }: { status: DataSourceRow['status'] }) {
  const badgeClass = status === 'online'
    ? 'bg-emerald-50 text-emerald-700 border-emerald-200'
    : status === 'warning'
      ? 'bg-amber-50 text-amber-700 border-amber-200'
      : 'bg-rose-50 text-rose-700 border-rose-200';

  const label = status === 'online' ? 'ONLINE' : status === 'warning' ? 'WARN' : 'OFFLINE';
  return (
    <span className={cn('rounded-sm border px-1.5 py-0.5 text-[9px] font-bold tracking-wider', badgeClass)}>
      {label}
    </span>
  );
}

function DataSourceTable({ rows }: { rows: DataSourceRow[] }) {
  return (
    <div className="overflow-hidden rounded-sm border border-[#e5e5e5]">
      <div className="grid grid-cols-[minmax(0,1.2fr)_minmax(0,1.8fr)_76px] gap-3 border-b border-[#e5e5e5] bg-[#f8fafc] px-3 py-2 text-[10px] font-bold uppercase tracking-wider text-[#6f6f6f]">
        <span>Source</span>
        <span>Endpoint / Value</span>
        <span>Status</span>
      </div>
      {rows.map((row) => (
        <div
          key={row.id}
          className="grid grid-cols-[minmax(0,1.2fr)_minmax(0,1.8fr)_76px] gap-3 border-b border-[#f3f4f6] px-3 py-2.5 text-[11px] last:border-b-0"
        >
          <span className="truncate text-[#333333]">{row.name}</span>
          <span className="truncate font-mono text-[#4b5563]">{row.value}</span>
          <div className="flex items-center">
            <SourceStatusBadge status={row.status} />
          </div>
        </div>
      ))}
    </div>
  );
}

export const DiagnosePlugin: Plugin = {
  metadata: {
    id: 'diagnose',
    name: 'Smart Diagnose & Monitor',
    description: 'Runtime diagnosis, monitoring, and data-source visualization.',
    version: '2.3.0',
    author: 'Ops Team',
  },
  icon: <ShieldAlert className="w-5 h-5" />,
  stepTitle: 'Smart Diagnose & Monitor',
  techStack: ['WebSocket', 'HTTP API', 'Recharts', 'AI Analytics'],
  inputSchema: { metrics: ['q', 'dq', 'tau', 'error'], rate: '500Hz' },
  outputSchema: { health_score: 98, advice: 'Check Joint 3 damping' },
  component: ({ data, onAction }) => {
    const pluginData = (data ?? {}) as Partial<WorkspacePluginData>;
    const apiBase = pluginData.apiBase ?? '/api';
    const selectedRobot = getRobotAsset(pluginData.selectedRobotId);
    const planningResult = pluginData.planningResult ?? null;

    const [activeStep, setActiveStep] = useState(1);
    const [health, setHealth] = useState<PlanningHealth | null>(null);
    const [healthLoading, setHealthLoading] = useState(true);
    const [healthError, setHealthError] = useState<string | null>(null);
    const [selectedMetric, setSelectedMetric] = useState<MetricOption['id']>('tau');

    useEffect(() => {
      let disposed = false;
      async function loadHealth() {
        setHealthLoading(true);
        setHealthError(null);
        try {
          const response = await fetch(`${apiBase}/design/planning/health`, { cache: 'no-store' });
          const payload = await response.json().catch(() => ({}));
          if (disposed) return;
          if (!response.ok) {
            setHealth(null);
            setHealthError(payload?.error ?? `Health API ${response.status}`);
            return;
          }
          setHealth(payload as PlanningHealth);
        } catch (error) {
          if (disposed) return;
          setHealth(null);
          setHealthError(error instanceof Error ? error.message : String(error));
        } finally {
          if (!disposed) {
            setHealthLoading(false);
          }
        }
      }

      void loadHealth();
      return () => {
        disposed = true;
      };
    }, [apiBase]);

    const sourceRows = useMemo<DataSourceRow[]>(() => {
      const hasTrajectory = Boolean(planningResult?.trajectory?.tUniform?.length);
      const planningFile = planningResult?.summary?.inputFileName ?? 'N/A';
      const healthStatus: DataSourceRow['status'] = healthLoading
        ? 'warning'
        : health?.ok
          ? 'online'
          : 'offline';

      return [
        {
          id: 'telemetry-stream',
          name: 'Telemetry Stream',
          value: 'ws://192.168.1.105:8080/stream',
          status: 'online',
        },
        {
          id: 'robot-model',
          name: 'Robot Asset',
          value: `${selectedRobot.name} | ${selectedRobot.sourceType} | ${selectedRobot.sourcePath}`,
          status: 'online',
        },
        {
          id: 'simulation-engine',
          name: 'Simulation Engine',
          value: selectedRobot.engineLabel,
          status: 'online',
        },
        {
          id: 'planning-api',
          name: 'Planning Health API',
          value: `${apiBase}/design/planning/health`,
          status: healthStatus,
        },
        {
          id: 'trajectory-source',
          name: 'Trajectory Source',
          value: hasTrajectory ? planningFile : 'No trajectory loaded',
          status: hasTrajectory ? 'online' : 'warning',
        },
        {
          id: 'health-raw',
          name: 'Planner Dependencies',
          value: healthError
            ? `Error: ${healthError}`
            : healthLoading
              ? 'Checking...'
              : `Python ${health?.python?.executable ?? 'N/A'}`,
          status: healthStatus,
        },
      ];
    }, [apiBase, health, healthError, healthLoading, planningResult, selectedRobot]);

    const telemetryData = useMemo<TelemetryRow[]>(() => {
      const tUniform = planningResult?.trajectory?.tUniform ?? [];
      const qUniform = planningResult?.trajectory?.qUniform ?? [];
      if (tUniform.length >= 8 && qUniform.length === tUniform.length) {
        const stride = Math.max(1, Math.floor(tUniform.length / 180));
        const rows: TelemetryRow[] = [];
        let lastDq = 0;
        for (let i = 0; i < tUniform.length; i += stride) {
          const previousIndex = Math.max(0, i - stride);
          const q = qUniform[i]?.[0] ?? 0;
          const previousQ = qUniform[previousIndex]?.[0] ?? q;
          const dt = Math.max(tUniform[i] - tUniform[previousIndex], 1e-3);
          const dq = (q - previousQ) / dt;
          const qdd = (dq - lastDq) / dt;
          const tau = Math.abs(q) * 4 + Math.abs(dq) * 1.2 + Math.abs(qdd) * 0.03;
          const eeVel = Math.min(4.5, Math.abs(dq) * 0.42 + Math.abs(q) * 0.08);
          const error = Math.min(0.25, Math.abs(dq) * 0.008);
          rows.push({
            time: Number(tUniform[i].toFixed(3)),
            q: Number(q.toFixed(4)),
            dq: Number(dq.toFixed(4)),
            qdd: Number(qdd.toFixed(4)),
            tau: Number(tau.toFixed(4)),
            eeVel: Number(eeVel.toFixed(4)),
            error: Number(error.toFixed(4)),
          });
          lastDq = dq;
        }
        if (rows.length > 0) {
          return rows;
        }
      }
      return FALLBACK_DIAGNOSE_DATA;
    }, [planningResult]);

    const selectedMetricLabel = useMemo(
      () => METRIC_OPTIONS.find((metric) => metric.id === selectedMetric)?.label ?? 'Joint Torque',
      [selectedMetric],
    );

    const chartJointCount = useMemo(() => {
      const trajectoryJointCount = planningResult?.trajectory?.qUniform?.[0]?.length ?? 0;
      const resolved = Math.max(selectedRobot.jointCount, trajectoryJointCount);
      return Math.max(1, Math.min(6, resolved));
    }, [planningResult, selectedRobot]);

    const jointLineMeta = useMemo(
      () =>
        Array.from({ length: chartJointCount }, (_, index) => ({
          key: `j${index + 1}`,
          label: selectedRobot.jointLabels[index] ?? `J${index + 1}`,
          color: JOINT_LINE_COLORS[index % JOINT_LINE_COLORS.length],
        })),
      [chartJointCount, selectedRobot],
    );

    const jointMetricChartData = useMemo(() => {
      const tUniform = planningResult?.trajectory?.tUniform ?? [];
      const qUniform = planningResult?.trajectory?.qUniform ?? [];
      if (tUniform.length >= 8 && qUniform.length === tUniform.length) {
        const stride = Math.max(1, Math.floor(tUniform.length / 180));
        const rows: Array<Record<string, number>> = [];
        const lastQ = Array.from({ length: chartJointCount }, () => 0);
        const lastDq = Array.from({ length: chartJointCount }, () => 0);
        let initialized = false;
        for (let i = 0; i < tUniform.length; i += stride) {
          const previousIndex = Math.max(0, i - stride);
          const dt = Math.max(tUniform[i] - tUniform[previousIndex], 1e-3);
          const row: Record<string, number> = { time: Number(tUniform[i].toFixed(3)) };

          for (let joint = 0; joint < chartJointCount; joint += 1) {
            const q = qUniform[i]?.[joint] ?? 0;
            if (!initialized) {
              lastQ[joint] = q;
            }
            const dq = initialized ? (q - lastQ[joint]) / dt : 0;
            const qdd = initialized ? (dq - lastDq[joint]) / dt : 0;
            const tau = q * 2.2 + dq * 0.9 + qdd * 0.05;
            const eeVelByJoint = Math.abs(dq) * (1 + joint * 0.07);

            let value = tau;
            if (selectedMetric === 'dq') value = dq;
            if (selectedMetric === 'qdd') value = qdd;
            if (selectedMetric === 'eeVel') value = eeVelByJoint;

            row[`j${joint + 1}`] = Number(value.toFixed(4));
            lastQ[joint] = q;
            lastDq[joint] = dq;
          }
          rows.push(row);
          initialized = true;
        }
        if (rows.length > 0) {
          return rows;
        }
      }

      return Array.from({ length: 140 }, (_, idx) => {
        const time = Number((idx * 0.04).toFixed(3));
        const row: Record<string, number> = { time };
        for (let joint = 0; joint < chartJointCount; joint += 1) {
          const phase = idx * 0.1 + joint * 0.38;
          const value = selectedMetric === 'tau'
            ? Math.sin(phase) * (5 + joint * 0.35)
            : selectedMetric === 'dq'
              ? Math.cos(phase) * (1.2 + joint * 0.05)
              : selectedMetric === 'qdd'
                ? -Math.sin(phase) * (0.22 + joint * 0.02)
                : Math.abs(Math.cos(phase)) * (0.5 + joint * 0.08);
          row[`j${joint + 1}`] = Number(value.toFixed(4));
        }
        return row;
      });
    }, [chartJointCount, planningResult, selectedMetric]);

    const riskTrendData = useMemo(
      () => telemetryData.map((item) => ({
        time: item.time,
        risk: Number(Math.min(1, item.error * 3 + Math.abs(item.dq) * 0.05).toFixed(4)),
      })),
      [telemetryData],
    );

    const jointLoadData = useMemo(() => {
      const lastQ = planningResult?.trajectory?.qUniform?.at(-1) ?? selectedRobot.defaultJointValues;
      const count = Math.max(selectedRobot.jointCount, selectedRobot.jointLabels.length);
      return Array.from({ length: count }, (_, index) => {
        const label = selectedRobot.jointLabels[index] ?? `J${index + 1}`;
        const q = Math.abs(lastQ?.[index] ?? selectedRobot.defaultJointValues[index] ?? 0);
        const load = Math.min(100, Number((20 + q * 38).toFixed(2)));
        return { joint: label, load };
      });
    }, [planningResult, selectedRobot]);

    const healthScore = useMemo(() => {
      const onlineCount = sourceRows.filter((item) => item.status === 'online').length;
      const base = Math.round((onlineCount / Math.max(sourceRows.length, 1)) * 100);
      return Math.min(100, Math.max(0, base));
    }, [sourceRows]);

    const steps = [
      { id: 1, label: 'Data Source', icon: <Database className="w-3.5 h-3.5" /> },
      { id: 2, label: 'Monitoring', icon: <Activity className="w-3.5 h-3.5" /> },
      { id: 3, label: 'Reports', icon: <FileText className="w-3.5 h-3.5" /> },
    ];

    return (
      <div className="flex h-full flex-col bg-white text-[#333333] select-none">
        <div className="grid grid-cols-3 gap-1 border-b border-[#e5e5e5] bg-[#f3f3f3] px-2 shrink-0">
          {steps.map((step) => (
            <button
              key={step.id}
              onClick={() => setActiveStep(step.id)}
              className={cn(
                'relative flex min-w-0 flex-col items-center justify-center gap-1 rounded-sm px-2 py-2.5 text-center text-[11px] font-medium leading-snug transition-all',
                activeStep === step.id
                  ? 'border-b-2 border-[#007acc] bg-white/50 text-[#333333]'
                  : 'text-[#6f6f6f] hover:bg-[#e8e8e8] hover:text-[#333333]',
              )}
            >
              {step.icon}
              {step.label}
            </button>
          ))}
        </div>

        <div className="flex-1 space-y-6 overflow-y-auto custom-scrollbar p-4">
          {activeStep === 1 && (
            <motion.div initial={{ opacity: 0, y: 5 }} animate={{ opacity: 1, y: 0 }} className="space-y-6">
              <CollapsibleSection title="Real-time Data Source" defaultOpen>
                <div className="flex items-center gap-3 rounded-sm border border-[#e5e5e5] bg-[#f8f8f8] p-4 shadow-sm transition-colors hover:border-[#cccccc]">
                  <div className="flex h-10 w-10 items-center justify-center rounded-sm border border-[#e5e5e5] bg-white">
                    <Database className="h-6 w-6 text-[#007acc]" />
                  </div>
                  <div>
                    <p className="text-[11px] font-bold text-[#333333]">WebSocket Stream</p>
                    <p className="text-[10px] text-[#6f6f6f]">ws://192.168.1.105:8080/stream</p>
                  </div>
                </div>
                <button className="mt-3 w-full rounded-sm bg-[#333333] py-2 text-[10px] font-bold uppercase tracking-widest text-white shadow-sm transition-all hover:bg-[#1e1e1e] active:scale-[0.98]">
                  Connect Data Link
                </button>
              </CollapsibleSection>

              <CollapsibleSection title="Data Source Info" defaultOpen>
                <DataSourceTable rows={sourceRows} />
              </CollapsibleSection>

              <CollapsibleSection title={`Selected Metrics Chart (${selectedMetricLabel})`} defaultOpen>
                <section className="h-[220px] rounded-sm border border-[#e5e5e5] bg-white p-2 shadow-sm">
                  <ResponsiveContainer width="100%" height="100%">
                    <LineChart data={jointMetricChartData} margin={{ top: 8, right: 12, left: -12, bottom: 0 }}>
                      <CartesianGrid strokeDasharray="3 3" stroke="#f1f5f9" vertical={false} />
                      <XAxis dataKey="time" stroke="#94a3b8" fontSize={9} tickLine={false} axisLine={false} />
                      <YAxis stroke="#94a3b8" fontSize={9} tickLine={false} axisLine={false} />
                      <Tooltip />
                      <Legend verticalAlign="top" height={24} iconType="line" wrapperStyle={{ fontSize: '10px' }} />
                      {jointLineMeta.map((joint) => (
                        <Line
                          key={joint.key}
                          type="monotone"
                          dataKey={joint.key}
                          name={joint.label}
                          stroke={joint.color}
                          strokeWidth={1.5}
                          dot={false}
                          isAnimationActive={false}
                        />
                      ))}
                    </LineChart>
                  </ResponsiveContainer>
                </section>
              </CollapsibleSection>

              <CollapsibleSection title="Metrics Selection" defaultOpen>
                <div className="grid grid-cols-2 gap-2.5">
                  {METRIC_OPTIONS.map((metric) => {
                    const active = selectedMetric === metric.id;
                    return (
                      <button
                        type="button"
                        key={metric.id}
                        onClick={() => setSelectedMetric(metric.id)}
                        className={cn(
                          'group flex cursor-pointer items-center gap-2.5 rounded-sm border bg-white p-2.5 text-left shadow-sm transition-colors',
                          active ? 'border-[#007acc]' : 'border-[#e5e5e5] hover:border-[#007acc]',
                        )}
                      >
                        <div className={cn(
                          'flex h-3.5 w-3.5 items-center justify-center rounded-sm border-2 bg-white transition-colors',
                          active ? 'border-[#007acc]' : 'border-[#e5e5e5] group-hover:border-[#007acc]',
                        )}>
                          {active && <CheckCircle2 className="h-2.5 w-2.5 text-[#007acc]" />}
                        </div>
                        <span className="text-[10px] font-medium text-[#333333]">{metric.label}</span>
                      </button>
                    );
                  })}
                </div>
              </CollapsibleSection>
            </motion.div>
          )}

          {activeStep === 2 && (
            <motion.div initial={{ opacity: 0, y: 5 }} animate={{ opacity: 1, y: 0 }} className="space-y-6">
              <CollapsibleSection title="Monitoring Overview" defaultOpen>
                <div className="grid grid-cols-2 gap-3">
                  {[
                    { label: 'Health', value: `${healthScore}`, color: 'text-[#388a3c]', icon: <Zap className="h-3 w-3" /> },
                    { label: 'Risk', value: `${(riskTrendData.at(-1)?.risk ?? 0).toFixed(2)}`, color: 'text-[#388a3c]', icon: <ShieldAlert className="h-3 w-3" /> },
                    { label: 'Rate', value: '500Hz', color: 'text-[#007acc]', icon: <Activity className="h-3 w-3" /> },
                    { label: 'Throughput', value: '124MB/s', color: 'text-[#007acc]', icon: <Activity className="h-3 w-3" /> },
                  ].map((item) => (
                    <div key={item.label} className="rounded-sm border border-[#e5e5e5] bg-white p-3 shadow-sm transition-shadow hover:shadow-md">
                      <div className="mb-1.5 flex items-center gap-1.5 opacity-60">
                        {item.icon}
                        <span className="text-[9px] font-bold uppercase tracking-widest">{item.label}</span>
                      </div>
                      <p className={cn('text-xl font-bold tracking-tight', item.color)}>{item.value}</p>
                    </div>
                  ))}
                </div>
              </CollapsibleSection>

              <CollapsibleSection title="Monitoring Data Sources" defaultOpen>
                <DataSourceTable rows={sourceRows} />
              </CollapsibleSection>

              <CollapsibleSection title="Live Telemetry" defaultOpen>
                <section className="relative h-[200px] overflow-hidden rounded-sm border border-[#e5e5e5] bg-white p-3 shadow-sm">
                  <div className="absolute left-3 top-3 z-10 flex items-center gap-1.5 opacity-60">
                    <Activity className="h-3 w-3 text-[#007acc]" />
                    <span className="text-[9px] font-bold uppercase tracking-widest">Live Telemetry</span>
                  </div>
                  <ResponsiveContainer width="100%" height="100%">
                    <AreaChart data={telemetryData} margin={{ top: 30, right: 0, left: -25, bottom: 0 }}>
                      <CartesianGrid strokeDasharray="3 3" stroke="#f1f5f9" vertical={false} />
                      <XAxis dataKey="time" hide />
                      <YAxis stroke="#94a3b8" fontSize={8} tickLine={false} axisLine={false} />
                      <Tooltip
                        contentStyle={{
                          borderRadius: '2px',
                          border: '1px solid #e5e5e5',
                          boxShadow: '0 2px 4px rgba(0,0,0,0.05)',
                          padding: '4px',
                          fontSize: '9px',
                        }}
                      />
                      <Area type="monotone" dataKey="tau" stroke="#007acc" strokeWidth={1.5} fill="#007acc" fillOpacity={0.1} isAnimationActive={false} />
                      <Area type="monotone" dataKey="q" stroke="#388a3c" strokeWidth={1.5} fill="#388a3c" fillOpacity={0.05} isAnimationActive={false} />
                    </AreaChart>
                  </ResponsiveContainer>
                </section>
              </CollapsibleSection>

              <CollapsibleSection title="Metrics Trend" defaultOpen>
                <section className="h-[220px] rounded-sm border border-[#e5e5e5] bg-white p-2 shadow-sm">
                  <ResponsiveContainer width="100%" height="100%">
                    <LineChart data={telemetryData} margin={{ top: 8, right: 12, left: -12, bottom: 0 }}>
                      <CartesianGrid strokeDasharray="3 3" stroke="#f1f5f9" vertical={false} />
                      <XAxis dataKey="time" stroke="#94a3b8" fontSize={9} tickLine={false} axisLine={false} />
                      <YAxis stroke="#94a3b8" fontSize={9} tickLine={false} axisLine={false} />
                      <Tooltip />
                      <Legend verticalAlign="top" height={24} iconType="line" wrapperStyle={{ fontSize: '10px' }} />
                      <Line type="monotone" dataKey="q" stroke="#007acc" strokeWidth={1.4} dot={false} isAnimationActive={false} />
                      <Line type="monotone" dataKey="dq" stroke="#16a34a" strokeWidth={1.2} dot={false} isAnimationActive={false} />
                      <Line type="monotone" dataKey="error" stroke="#dc2626" strokeWidth={1.2} dot={false} isAnimationActive={false} />
                    </LineChart>
                  </ResponsiveContainer>
                </section>
              </CollapsibleSection>
            </motion.div>
          )}

          {activeStep === 3 && (
            <motion.div initial={{ opacity: 0, y: 5 }} animate={{ opacity: 1, y: 0 }} className="space-y-6">
              <CollapsibleSection title="Diagnostic Insights" defaultOpen>
                <section className="rounded-sm border border-[#d0e7ff] bg-[#f0f7ff] p-4 shadow-sm">
                  <div className="mb-3 flex items-center gap-3">
                    <div className="flex h-8 w-8 items-center justify-center rounded-sm bg-[#007acc] text-white shadow-md">
                      <BrainCircuit className="h-5 w-5" />
                    </div>
                    <p className="text-[11px] font-bold text-[#005fb8]">Diagnostic Insights</p>
                  </div>
                  <p className="text-[10px] leading-relaxed text-[#005fb8] opacity-80">
                    System remains stable. Minor overshoot appears during high-speed segments.
                    Recommended: tune damping and junction velocity in the Design stage.
                  </p>
                  <div className="mt-4 flex gap-2">
                    <button className="flex-1 rounded-sm border border-[#d0e7ff] bg-white py-1.5 text-[10px] font-bold text-[#007acc] shadow-sm transition-all hover:bg-[#f8fbff] active:scale-[0.98]">
                      Download PDF
                    </button>
                    <button className="flex-1 rounded-sm bg-[#007acc] py-1.5 text-[10px] font-bold text-white shadow-md transition-all hover:bg-[#005fb8] active:scale-[0.98]">
                      Sync to Cloud
                    </button>
                  </div>
                </section>
              </CollapsibleSection>

              <CollapsibleSection title="Risk Trend" defaultOpen>
                <section className="h-[180px] rounded-sm border border-[#e5e5e5] bg-white p-2 shadow-sm">
                  <ResponsiveContainer width="100%" height="100%">
                    <LineChart data={riskTrendData} margin={{ top: 8, right: 8, left: -10, bottom: 0 }}>
                      <CartesianGrid strokeDasharray="3 3" stroke="#f1f5f9" vertical={false} />
                      <XAxis dataKey="time" stroke="#94a3b8" fontSize={9} tickLine={false} axisLine={false} />
                      <YAxis domain={[0, 1]} stroke="#94a3b8" fontSize={9} tickLine={false} axisLine={false} />
                      <Tooltip />
                      <Line type="monotone" dataKey="risk" stroke="#d97706" strokeWidth={1.5} dot={false} isAnimationActive={false} />
                    </LineChart>
                  </ResponsiveContainer>
                </section>
              </CollapsibleSection>

              <CollapsibleSection title="Joint Load" defaultOpen>
                <section className="h-[220px] rounded-sm border border-[#e5e5e5] bg-white p-2 shadow-sm">
                  <ResponsiveContainer width="100%" height="100%">
                    <BarChart data={jointLoadData} margin={{ top: 8, right: 12, left: -8, bottom: 0 }}>
                      <CartesianGrid strokeDasharray="3 3" stroke="#f1f5f9" vertical={false} />
                      <XAxis dataKey="joint" stroke="#94a3b8" fontSize={9} tickLine={false} axisLine={false} />
                      <YAxis domain={[0, 100]} stroke="#94a3b8" fontSize={9} tickLine={false} axisLine={false} />
                      <Tooltip />
                      <Bar dataKey="load" fill="#007acc" radius={[2, 2, 0, 0]} isAnimationActive={false} />
                    </BarChart>
                  </ResponsiveContainer>
                </section>
              </CollapsibleSection>
            </motion.div>
          )}
        </div>

        <div className="shrink-0 border-t border-[#e5e5e5] bg-[#f3f3f3] p-5">
          <button
            onClick={() => onAction('COMPLETE', 'DIAGNOSE_FINISHED')}
            className="flex w-full items-center justify-center gap-2 bg-[#007acc] py-2.5 text-xs font-medium text-white shadow-sm transition-all hover:bg-[#0062a3] active:scale-[0.98]"
          >
            <Search className="h-4 w-4" />
            Generate Full Diagnostic Report
          </button>
        </div>
      </div>
    );
  },
};
