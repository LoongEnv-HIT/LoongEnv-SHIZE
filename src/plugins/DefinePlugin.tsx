import React from 'react';
import { Box, Cpu, Play } from 'lucide-react';
import { motion } from 'motion/react';
import { CollapsibleSection } from '../components/CollapsibleSection';
import { getRobotAsset } from '../data/robotCatalog';
import { cn } from '../lib/utils';
import { Plugin, RobotAsset, WorkspacePluginData } from '../types';

type DefinePluginData = Pick<
  WorkspacePluginData,
  'robotAssets' | 'selectedRobotId' | 'setSelectedRobotId'
>;

function DetailRow({
  label,
  value,
  mono = false,
}: {
  label: string;
  value: string;
  mono?: boolean;
}) {
  return (
    <div className="grid grid-cols-[112px_minmax(0,1fr)] gap-3 border-b border-[#f3f4f6] px-3 py-2.5 last:border-b-0">
      <span className="text-[11px] text-[#6f6f6f]">{label}</span>
      <span className={cn('text-[11px] text-[#333333]', mono && 'font-mono')}>{value}</span>
    </div>
  );
}

function JointLimitTable({ robot }: { robot: RobotAsset }) {
  if (!robot.jointLimits?.length) {
    return (
      <div className="rounded-sm border border-dashed border-[#d4dbe3] bg-[#f8fafc] px-3 py-2 text-[11px] leading-5 text-[#526070]">
        当前资产未提供结构化关节限位。仿真会按模型默认姿态加载。
      </div>
    );
  }

  return (
    <div className="overflow-hidden rounded-sm border border-[#e5e5e5]">
      <div className="grid grid-cols-[68px_minmax(0,1.6fr)_minmax(0,1fr)_minmax(0,1fr)] gap-3 border-b border-[#e5e5e5] bg-[#f8fafc] px-3 py-2 text-[10px] font-bold uppercase tracking-wider text-[#6f6f6f]">
        <span>Joint</span>
        <span>Position</span>
        <span>Velocity</span>
        <span>Torque</span>
      </div>
      {robot.jointLimits.map((limit) => (
        <div
          key={limit.joint}
          className="grid grid-cols-[68px_minmax(0,1.6fr)_minmax(0,1fr)_minmax(0,1fr)] gap-3 border-b border-[#f3f4f6] px-3 py-2.5 text-[11px] last:border-b-0"
        >
          <span className="font-mono font-semibold text-[#333333]">{limit.joint}</span>
          <span className="font-mono text-[#333333]">{limit.position}</span>
          <span className="font-mono text-[#6f6f6f]">{limit.velocity ?? 'N/A'}</span>
          <span className="font-mono text-[#6f6f6f]">{limit.torque ?? 'N/A'}</span>
        </div>
      ))}
    </div>
  );
}

export const DefinePlugin: Plugin = {
  metadata: {
    id: 'define',
    name: '机器人定义模块',
    description: '从内置 robots 资产中选择机器人，并让中间仿真视图始终跟随当前选择。',
    version: '2.0.0',
    author: 'LoongEnv Core',
  },
  icon: <Box className="w-5 h-5" />,
  stepTitle: '机器人资产与仿真绑定',
  techStack: ['MuJoCo WASM', 'URDF Loader', 'MJCF', 'URDF', 'STL/DAE'],
  inputSchema: { assets: 'public/robots/*', selection: 'robot id' },
  outputSchema: { selectedRobot: 'runtime-ready robot asset', simulation: 'active viewport model' },
  component: ({ data, onAction }) => {
    const {
      robotAssets = [],
      selectedRobotId,
      setSelectedRobotId,
    } = (data ?? {}) as Partial<DefinePluginData>;

    const selectedRobot = getRobotAsset(selectedRobotId);

    const handleSelectRobot = (robotId: string) => {
      setSelectedRobotId?.(robotId);
      onAction('SELECT_ROBOT', robotId);
    };

    return (
      <div className="flex h-full flex-col bg-white text-[#333333] select-none">
        <div className="flex-1 overflow-y-auto custom-scrollbar p-4">
          <motion.div initial={{ opacity: 0, y: 5 }} animate={{ opacity: 1, y: 0 }} className="space-y-6">
            <CollapsibleSection title="资产管理 (Asset Management)" defaultOpen>
              <div className="overflow-hidden rounded-sm border border-[#e5e5e5]">
                {(robotAssets.length ? robotAssets : [selectedRobot]).map((robot) => {
                  const isActive = robot.id === selectedRobot.id;
                  return (
                    <button
                      key={robot.id}
                      type="button"
                      onClick={() => handleSelectRobot(robot.id)}
                      className={cn(
                        'block w-full border-b border-[#f3f4f6] px-3 py-2.5 text-left text-[12px] transition-colors last:border-b-0',
                        isActive ? 'bg-[#eef6ff] text-[#0f4c81] font-semibold' : 'bg-white text-[#333333] hover:bg-[#f8fafc]',
                      )}
                    >
                      {robot.name}
                    </button>
                  );
                })}
              </div>
            </CollapsibleSection>

            <CollapsibleSection title="当前机器人 (Selected Robot)" defaultOpen>
              <div className="overflow-hidden rounded-sm border border-[#e5e5e5] bg-white">
                <DetailRow label="名称" value={selectedRobot.name} />
                <DetailRow label="引擎" value={selectedRobot.engineLabel} />
                <DetailRow label="格式" value={selectedRobot.sourceType} />
                <DetailRow label="关节数" value={String(selectedRobot.jointCount)} mono />
                <DetailRow label="资源路径" value={selectedRobot.sourcePath} mono />
                <DetailRow label="说明" value={selectedRobot.description} />
              </div>
            </CollapsibleSection>

            <CollapsibleSection title="关节约束 (Joint Limits)" defaultOpen>
              <JointLimitTable robot={selectedRobot} />
            </CollapsibleSection>

            <CollapsibleSection title="仿真绑定 (Simulation Binding)" defaultOpen>
              <div className="space-y-3 text-[11px] leading-5 text-[#526070]">
                <div className="flex items-start gap-2 rounded-sm border border-[#e5e5e5] bg-[#f8fafc] px-3 py-2.5">
                  <Cpu className="mt-0.5 h-3.5 w-3.5 text-[#007acc]" />
                  <p>中间 3D 视图会即时切换到当前选中的机器人，不再固定使用 ER15。</p>
                </div>
                <div className="flex items-start gap-2 rounded-sm border border-[#e5e5e5] bg-[#f8fafc] px-3 py-2.5">
                  <Play className="mt-0.5 h-3.5 w-3.5 text-[#007acc]" />
                  <p>ER15 继续走 MuJoCo 链路，其它机器人走 URDF 链路，统一在同一仿真窗口显示。</p>
                </div>
              </div>
            </CollapsibleSection>
          </motion.div>
        </div>

        <div className="border-t border-[#e5e5e5] bg-[#f3f3f3] px-4 py-3 shrink-0">
          <div className="flex items-center justify-between gap-3 rounded-sm border border-[#dbe4ee] bg-white px-3 py-2">
            <div className="min-w-0">
              <div className="text-[10px] font-bold uppercase tracking-wider text-[#6f6f6f]">Simulation Target</div>
              <div className="truncate text-[12px] font-semibold text-[#333333]">{selectedRobot.name}</div>
            </div>
            <button
              type="button"
              onClick={() => handleSelectRobot(selectedRobot.id)}
              className="shrink-0 rounded-sm bg-[#007acc] px-3 py-2 text-[11px] font-bold uppercase tracking-wider text-white transition-colors hover:bg-[#0062a3]"
            >
              已应用到仿真
            </button>
          </div>
        </div>
      </div>
    );
  },
};
