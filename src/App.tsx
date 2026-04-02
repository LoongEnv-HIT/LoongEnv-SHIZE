/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
 */

import React, { useEffect, useState } from 'react';
import { 
  Activity, 
  Cpu, 
  Database, 
  LayoutDashboard, 
  Play, 
  Settings, 
  ShieldAlert, 
  Zap, 
  ArrowRight, 
  Code, 
  Terminal, 
  Layers,
  Upload,
  BrainCircuit,
  ChevronRight,
  ChevronLeft,
  Monitor,
  Box,
  CheckCircle2,
  Puzzle,
  Menu,
  X,
  Plus,
  ExternalLink,
  Info,
  Files,
  Search,
  GitBranch,
  PlayCircle,
  Package,
  UserCircle,
  Bell,
  Split,
  MoreHorizontal
} from 'lucide-react';
import { motion, AnimatePresence } from 'motion/react';
import { cn } from './lib/utils';
import { DEFAULT_ROBOT_ID, getRobotAsset, ROBOT_ASSETS } from './data/robotCatalog';

// --- Types ---
import { PlaybackState, PlanningResult, Plugin } from './types';

// --- Plugins ---
import { DefinePlugin } from './plugins/DefinePlugin';
import { DesignPlugin } from './plugins/DesignPlugin';
import { DeployPlugin } from './plugins/DeployPlugin';
import { DiagnosePlugin } from './plugins/DiagnosePlugin';

// --- New Components ---
import { SimulationView } from './components/SimulationView';
import { SystemMonitor } from './components/SystemMonitor';

// --- Utility ---

// 插件注册表
const PLUGIN_REGISTRY: Plugin[] = [
  DefinePlugin,
  DesignPlugin,
  DeployPlugin,
  DiagnosePlugin
];

// --- Components ---

interface TechBadgeProps {
  name: string;
}

const TechBadge: React.FC<TechBadgeProps> = ({ name }) => (
  <span className="px-1 py-0.5 bg-[#f3f3f3] text-[#6f6f6f] border border-[#e5e5e5] rounded-sm text-[10px] font-mono">
    {name}
  </span>
);

interface SchemaCardProps {
  title: string;
  data: any;
}

const SchemaCard: React.FC<SchemaCardProps> = ({ title, data }) => {
  const [isOpen, setIsOpen] = useState(false);
  return (
    <div className="mb-1">
      <div 
        onClick={() => setIsOpen(!isOpen)}
        className="flex items-center gap-1 px-2 py-0.5 cursor-pointer hover:bg-[#e8e8e8] transition-colors"
      >
        <ChevronRight className={cn("w-3 h-3 text-[#6f6f6f] transition-transform", isOpen && "rotate-90")} />
        <span className="text-[10px] font-bold text-[#6f6f6f] uppercase tracking-wider">{title}</span>
      </div>
      {isOpen && (
        <div className="pl-6 pr-2 py-1 bg-[#f9f9f9] border-l border-[#e5e5e5] ml-3 mt-0.5">
          <pre className="text-[10px] font-mono text-[#6f6f6f] overflow-x-auto custom-scrollbar leading-relaxed">
            {JSON.stringify(data, null, 2)}
          </pre>
        </div>
      )}
    </div>
  );
};

// --- Main App ---

const SIDEBAR_DEFAULT_WIDTH = 560;
const SIDEBAR_MIN_WIDTH = 380;
const SIDEBAR_MAX_WIDTH = 560;
const BOTTOM_PANEL_DEFAULT_HEIGHT = 220;
const BOTTOM_PANEL_MIN_HEIGHT = 160;
const BOTTOM_PANEL_MAX_HEIGHT = 360;

type DragState =
  | { type: 'sidebar'; startPos: number; startValue: number }
  | { type: 'bottom'; startPos: number; startValue: number }
  | null;

function clamp(value: number, min: number, max: number) {
  return Math.min(Math.max(value, min), max);
}

export default function App() {
  const [activeModuleId, setActiveModuleId] = useState('define');
  const [showCore, setShowCore] = useState(false);
  const [showPluginManager, setShowPluginManager] = useState(false);
  const [isSideBarVisible, setIsSideBarVisible] = useState(true);
  const [sidebarWidth, setSidebarWidth] = useState(SIDEBAR_DEFAULT_WIDTH);
  const [bottomPanelHeight, setBottomPanelHeight] = useState(BOTTOM_PANEL_DEFAULT_HEIGHT);
  const [dragState, setDragState] = useState<DragState>(null);
  const [planningResult, setPlanningResult] = useState<PlanningResult | null>(null);
  const [selectedRobotId, setSelectedRobotId] = useState(DEFAULT_ROBOT_ID);
  const [playback, setPlayback] = useState<PlaybackState>({
    isPlaying: false,
    currentTimeSec: 0,
    speed: 1,
    loop: true,
  });

  const activePlugin = PLUGIN_REGISTRY.find(p => p.metadata.id === activeModuleId) || PLUGIN_REGISTRY[0];
  const selectedRobot = getRobotAsset(selectedRobotId);
  const trajectoryDuration = planningResult?.trajectory.duration ?? 0;

  useEffect(() => {
    if (!playback.isPlaying || !planningResult || trajectoryDuration <= 0) {
      return;
    }

    let frameId = 0;
    let previousTs = performance.now();

    const tick = (ts: number) => {
      const elapsed = Math.max(0, (ts - previousTs) / 1000);
      previousTs = ts;

      setPlayback((previous) => {
        if (!previous.isPlaying) {
          return previous;
        }
        const speed = Number.isFinite(previous.speed) ? Math.max(previous.speed, 0.01) : 1;
        let nextTime = previous.currentTimeSec + elapsed * speed;

        if (nextTime >= trajectoryDuration) {
          if (previous.loop) {
            nextTime = nextTime % trajectoryDuration;
          } else {
            return {
              ...previous,
              currentTimeSec: trajectoryDuration,
              isPlaying: false,
            };
          }
        }

        return {
          ...previous,
          currentTimeSec: nextTime,
        };
      });

      frameId = window.requestAnimationFrame(tick);
    };

    frameId = window.requestAnimationFrame(tick);
    return () => {
      window.cancelAnimationFrame(frameId);
    };
  }, [playback.isPlaying, planningResult, trajectoryDuration]);

  useEffect(() => {
    if (!planningResult) {
      setPlayback((previous) => ({
        ...previous,
        isPlaying: false,
        currentTimeSec: 0,
      }));
    }
  }, [planningResult]);

  useEffect(() => {
    setPlayback((previous) => ({
      ...previous,
      isPlaying: false,
      currentTimeSec: 0,
    }));
  }, [selectedRobotId]);

  useEffect(() => {
    if (!dragState) {
      return;
    }

    const handleMouseMove = (event: MouseEvent) => {
      if (dragState.type === 'sidebar') {
        const maxWidth = Math.min(SIDEBAR_MAX_WIDTH, Math.max(SIDEBAR_MIN_WIDTH, window.innerWidth - 320));
        setSidebarWidth(clamp(dragState.startValue + event.clientX - dragState.startPos, SIDEBAR_MIN_WIDTH, maxWidth));
        return;
      }

      const maxHeight = Math.min(BOTTOM_PANEL_MAX_HEIGHT, Math.max(BOTTOM_PANEL_MIN_HEIGHT, Math.floor(window.innerHeight * 0.45)));
      setBottomPanelHeight(clamp(dragState.startValue - (event.clientY - dragState.startPos), BOTTOM_PANEL_MIN_HEIGHT, maxHeight));
    };

    const handleMouseUp = () => setDragState(null);
    const previousCursor = document.body.style.cursor;
    const previousUserSelect = document.body.style.userSelect;

    document.body.style.cursor = dragState.type === 'sidebar' ? 'ew-resize' : 'ns-resize';
    document.body.style.userSelect = 'none';

    window.addEventListener('mousemove', handleMouseMove);
    window.addEventListener('mouseup', handleMouseUp);

    return () => {
      document.body.style.cursor = previousCursor;
      document.body.style.userSelect = previousUserSelect;
      window.removeEventListener('mousemove', handleMouseMove);
      window.removeEventListener('mouseup', handleMouseUp);
    };
  }, [dragState]);

  return (
    <div className="h-screen bg-[#ffffff] text-[#333333] font-sans selection:bg-[#add6ff] flex flex-col overflow-hidden select-none">
      {/* Main Container */}
      <div className="flex-1 flex overflow-hidden">
        
        {/* 1. Activity Bar (Far Left) */}
        <aside className="w-12 bg-[#2c2c2c] flex flex-col items-center py-2 shrink-0 z-50">
          <div className="flex flex-col gap-1 w-full items-center">
            {PLUGIN_REGISTRY.map((p, idx) => {
              const icons = [Files, Search, GitBranch, PlayCircle];
              const IconComponent = icons[idx];
              const iconElement = IconComponent ? <IconComponent className="w-6 h-6" /> : p.icon;
              return (
                <button
                  key={p.metadata.id}
                  onClick={() => {
                    setActiveModuleId(p.metadata.id);
                    setIsSideBarVisible(true);
                    setShowCore(false);
                  }}
                  className={cn(
                    "w-12 h-12 flex items-center justify-center transition-all relative group",
                    activeModuleId === p.metadata.id && isSideBarVisible && !showCore
                      ? "text-white border-l-2 border-white" 
                      : "text-slate-400 hover:text-white"
                  )}
                  title={p.metadata.name}
                >
                  {iconElement}
                </button>
              );
            })}
            <button
              onClick={() => {
                setShowCore(!showCore);
                setIsSideBarVisible(true);
              }}
              className={cn(
                "w-12 h-12 flex items-center justify-center transition-all relative group",
                showCore && isSideBarVisible ? "text-white border-l-2 border-white" : "text-slate-400 hover:text-white"
              )}
              title="核心底座"
            >
              <Database className="w-6 h-6" />
            </button>
          </div>

          <div className="mt-auto flex flex-col gap-1 w-full items-center mb-2">
            <button 
              onClick={() => setShowPluginManager(true)}
              className={cn(
                "w-12 h-12 flex items-center justify-center transition-all",
                showPluginManager ? "text-white" : "text-slate-400 hover:text-white"
              )}
              title="插件管理"
            >
              <Puzzle className="w-6 h-6" />
            </button>
            <button className="w-12 h-12 flex items-center justify-center text-slate-400 hover:text-white transition-all" title="账户">
              <UserCircle className="w-6 h-6" />
            </button>
            <button className="w-12 h-12 flex items-center justify-center text-slate-400 hover:text-white transition-all" title="设置">
              <Settings className="w-6 h-6" />
            </button>
          </div>
        </aside>

        {/* 2. Side Bar (Collapsible) */}
        {isSideBarVisible && (
          <>
          <aside
            style={{ width: sidebarWidth }}
            className="bg-[#f3f3f3] border-r border-[#e5e5e5] flex flex-col shrink-0 z-40 shadow-sm min-w-0"
          >
            <div className="h-9 px-4 flex items-center justify-between shrink-0 bg-[#f3f3f3]">
              <span className="text-[11px] text-[#6f6f6f] font-bold tracking-wider uppercase">
                {showCore ? "资源管理器: 核心" : activePlugin.metadata.name}
              </span>
              <div className="flex items-center gap-1">
                <button className="p-1 hover:bg-[#e8e8e8] rounded text-[#6f6f6f] transition-colors" title="新建文件">
                  <Plus className="w-3.5 h-3.5" />
                </button>
                <button className="p-1 hover:bg-[#e8e8e8] rounded text-[#6f6f6f] transition-colors" title="全部折叠">
                  <ChevronLeft className="w-3.5 h-3.5" />
                </button>
                <button className="p-1 hover:bg-[#e8e8e8] rounded text-[#6f6f6f] transition-colors">
                  <MoreHorizontal className="w-3.5 h-3.5" />
                </button>
              </div>
            </div>

            <div className="flex-1 overflow-y-auto custom-scrollbar flex flex-col bg-[#f3f3f3]">
              {showCore ? (
                <div className="flex flex-col">
                  {/* Accordion Section */}
                  <div className="border-b border-[#e5e5e5]">
                    <div className="h-6 px-1 flex items-center gap-1 bg-[#e8e8e8] cursor-pointer hover:bg-[#d8d8d8] transition-colors">
                      <ChevronRight className="w-4 h-4 rotate-90" />
                      <span className="text-[11px] font-bold text-[#333333] uppercase tracking-tighter">通用架构 (Universal Schema)</span>
                    </div>
                    <div className="py-2 bg-white/50">
                      {['状态 (State)', '动作 (Action)', '日志 (Log)', '指标 (Metrics)'].map(item => (
                        <div key={item} className="flex items-center gap-2 px-6 py-1.5 hover:bg-[#e8e8e8] cursor-pointer text-[12px] text-[#6f6f6f] group">
                          <Database className="w-3.5 h-3.5 text-blue-500 opacity-70 group-hover:opacity-100" /> 
                          <span className="group-hover:text-[#333333] transition-colors">{item}</span>
                        </div>
                      ))}
                    </div>
                  </div>
                </div>
              ) : (
                <div className="flex flex-col flex-1">
                  {/* Plugin Info Section - Collapsible, Closed by default */}
                  <details className="border-b border-[#e5e5e5] group/details">
                    <summary className="h-6 px-1 flex items-center gap-1 bg-[#e8e8e8] cursor-pointer list-none hover:bg-[#d8d8d8] transition-colors">
                      <ChevronRight className="w-4 h-4 transition-transform group-open/details:rotate-90" />
                      <span className="text-[11px] font-bold text-[#333333] uppercase tracking-tighter">插件详情</span>
                    </summary>
                    <div className="p-5 bg-white space-y-5 shadow-inner">
                      <div>
                        <h2 className="text-sm font-bold text-[#333333] mb-2">{activePlugin.stepTitle}</h2>
                        <p className="text-[11px] text-[#6f6f6f] leading-relaxed">
                          {activePlugin.metadata.description}
                        </p>
                      </div>
                      
                      <div>
                        <span className="text-[10px] font-bold text-[#6f6f6f] uppercase block mb-3 tracking-widest">技术栈 (Tech Stack)</span>
                        <div className="flex flex-wrap gap-1.5">
                          {activePlugin.techStack.map(t => <TechBadge key={t} name={t} />)}
                        </div>
                      </div>

                      <div className="space-y-1 pt-2 border-t border-[#f3f3f3]">
                        <SchemaCard title="输入架构 (Input Schema)" data={activePlugin.inputSchema} />
                        <SchemaCard title="输出架构 (Output Schema)" data={activePlugin.outputSchema} />
                      </div>
                    </div>
                  </details>

                  {/* Plugin Custom UI Section - Primary focus */}
                  <div className="flex-1 flex flex-col min-h-0">
                    <div className="h-9 px-4 flex items-center justify-between bg-[#f3f3f3] border-b border-[#e5e5e5] shrink-0">
                      <div className="flex items-center gap-2">
                        <ChevronRight className="w-4 h-4 rotate-90 text-[#6f6f6f]" />
                        <span className="text-[11px] font-bold text-[#333333] uppercase tracking-wider">操作面板</span>
                      </div>
                      <div className="flex items-center gap-2">
                        <button className="p-1 hover:bg-[#e8e8e8] rounded-sm transition-colors text-[#6f6f6f] hover:text-[#333333]">
                          <Plus className="w-3.5 h-3.5" />
                        </button>
                        <button className="p-1 hover:bg-[#e8e8e8] rounded-sm transition-colors text-[#6f6f6f] hover:text-[#333333]">
                          <X className="w-3.5 h-3.5" />
                        </button>
                      </div>
                    </div>
                    <div className="flex-1 overflow-y-auto bg-white custom-scrollbar">
                      <activePlugin.component 
                        data={{
                          planningResult,
                          setPlanningResult,
                          playback,
                          setPlayback,
                          robotAssets: ROBOT_ASSETS,
                          selectedRobotId,
                          setSelectedRobotId,
                          apiBase: '/api',
                        }} 
                        onAction={(a, p) => console.log('Plugin Action:', a, p)} 
                      />
                    </div>
                  </div>
                </div>
              )}
            </div>
          </aside>
          <div
            role="separator"
            aria-orientation="vertical"
            aria-label="调整操作面板宽度"
            onMouseDown={(event) => setDragState({ type: 'sidebar', startPos: event.clientX, startValue: sidebarWidth })}
            className="group w-2 shrink-0 cursor-ew-resize bg-white hover:bg-[#f5f9ff] transition-colors flex items-center justify-center"
            title="拖动以调整操作面板宽度"
          >
            <div className={cn(
              "h-full w-[3px] rounded-full transition-colors",
              dragState?.type === 'sidebar' ? "bg-[#007acc]/60" : "bg-transparent group-hover:bg-[#007acc]/35"
            )} />
          </div>
          </>
        )}

        {/* 3. Main Editor Area */}
        <main className="flex-1 flex flex-col overflow-hidden bg-white relative">
          {/* Tabs Bar */}
          <div className="h-9 bg-[#f3f3f3] flex items-center overflow-x-auto shrink-0 border-b border-[#e5e5e5]">
            <div className="flex h-full">
              <div className="px-4 h-full flex items-center gap-2 bg-white border-r border-[#e5e5e5] border-t-2 border-t-[#007acc] cursor-default">
                <Box className="w-4 h-4 text-blue-500" />
                <span className="text-xs text-[#333333]">SimulationView.mu</span>
                <X className="w-3 h-3 text-[#6f6f6f] hover:bg-[#e8e8e8] rounded" />
              </div>
              <div className="px-4 h-full flex items-center gap-2 hover:bg-[#e8e8e8] border-r border-[#e5e5e5] cursor-pointer group">
                <Terminal className="w-4 h-4 text-emerald-500" />
                <span className="text-xs text-[#6f6f6f]">system.log</span>
                <X className="w-3 h-3 text-transparent group-hover:text-[#6f6f6f] hover:bg-[#d8d8d8] rounded" />
              </div>
            </div>
            <div className="ml-auto px-4 flex items-center gap-3">
              <button className="text-[#6f6f6f] hover:text-[#333333]"><Split className="w-4 h-4" /></button>
              <button className="text-[#6f6f6f] hover:text-[#333333]"><MoreHorizontal className="w-4 h-4" /></button>
            </div>
          </div>

            {/* Breadcrumbs */}
            <div className="h-6 bg-[#ffffff] border-b border-[#f3f3f3] flex items-center px-4 shrink-0 z-10">
              <div className="flex items-center gap-1 text-[11px] text-[#6f6f6f]">
                <span>LoongEnv</span>
                <ChevronRight className="w-3 h-3" />
                <span>src</span>
                <ChevronRight className="w-3 h-3" />
                <span>components</span>
                <ChevronRight className="w-3 h-3" />
                <span className="text-[#333333] font-medium">SimulationView.mu</span>
              </div>
            </div>

            {/* Editor Content (Simulation View) */}
            <div className="flex-1 relative overflow-hidden">
              <SimulationView
                robot={selectedRobot}
                trajectory={planningResult?.trajectory ?? null}
                currentTimeSec={playback.currentTimeSec}
              />
            </div>

          <div
            role="separator"
            aria-orientation="horizontal"
            aria-label="调整底部输出面板高度"
            onMouseDown={(event) => setDragState({ type: 'bottom', startPos: event.clientY, startValue: bottomPanelHeight })}
            className="group h-2 shrink-0 cursor-ns-resize bg-white hover:bg-[#f5f9ff] transition-colors flex items-center justify-center"
            title="拖动以调整输出面板高度"
          >
            <div className={cn(
              "h-[3px] w-full rounded-full transition-colors",
              dragState?.type === 'bottom' ? "bg-[#007acc]/60" : "bg-transparent group-hover:bg-[#007acc]/35"
            )} />
          </div>

          {/* 4. Bottom Panel (Output/Terminal) */}
          <div
            style={{ height: bottomPanelHeight }}
            className="border-t border-[#e5e5e5] flex flex-col shrink-0 min-h-0"
          >
            <SystemMonitor />
          </div>
        </main>
      </div>

      {/* 5. Status Bar (Very Bottom) */}
      <footer className="h-6 bg-[#007acc] text-white flex items-center justify-between px-3 text-[11px] shrink-0 z-[60]">
        <div className="flex items-center h-full">
          <div className="flex items-center gap-1.5 px-2 hover:bg-white/10 h-full cursor-pointer transition-colors">
            <GitBranch className="w-3.5 h-3.5" />
            <span>main*</span>
          </div>
          <div className="flex items-center gap-1.5 px-2 hover:bg-white/10 h-full cursor-pointer transition-colors">
            <X className="w-3.5 h-3.5" />
            <span>0</span>
            <ShieldAlert className="w-3.5 h-3.5" />
            <span>0</span>
          </div>
          <div className="flex items-center gap-1.5 px-2 hover:bg-white/10 h-full cursor-pointer transition-colors">
            <Activity className="w-3.5 h-3.5" />
            <span>{`仿真引擎: ${selectedRobot.engineLabel}`}</span>
          </div>
        </div>
        <div className="flex items-center h-full">
          <div className="px-2 hover:bg-white/10 h-full cursor-pointer transition-colors">第 1 行, 第 1 列</div>
          <div className="px-2 hover:bg-white/10 h-full cursor-pointer transition-colors">空格: 2</div>
          <div className="px-2 hover:bg-white/10 h-full cursor-pointer transition-colors">UTF-8</div>
          <div className="px-2 hover:bg-white/10 h-full cursor-pointer transition-colors flex items-center gap-1">
            <CheckCircle2 className="w-3.5 h-3.5" />
            <span>Prettier</span>
          </div>
          <div className="px-2 hover:bg-white/10 h-full cursor-pointer transition-colors">
            <Bell className="w-3.5 h-3.5" />
          </div>
        </div>
      </footer>

      {/* 插件管理器弹窗 - 保持原有逻辑但调整样式 */}
      <AnimatePresence>
        {showPluginManager && (
          <div className="fixed inset-0 z-[100] flex items-center justify-center p-8">
            <motion.div 
              initial={{ opacity: 0 }}
              animate={{ opacity: 1 }}
              exit={{ opacity: 0 }}
              onClick={() => setShowPluginManager(false)}
              className="absolute inset-0 bg-black/20"
            />
            <motion.div 
              initial={{ opacity: 0, scale: 0.98 }}
              animate={{ opacity: 1, scale: 1 }}
              exit={{ opacity: 0, scale: 0.98 }}
              className="relative w-full max-w-4xl bg-white shadow-2xl overflow-hidden flex flex-col max-h-[80vh] border border-[#e5e5e5]"
            >
              <div className="h-10 px-4 border-b border-[#e5e5e5] flex items-center justify-between bg-[#f3f3f3]">
                <div className="flex items-center gap-2">
                  <Puzzle className="w-4 h-4 text-[#6f6f6f]" />
                  <span className="text-[11px] font-bold text-[#333333] uppercase tracking-wider">插件市场 (Extensions Marketplace)</span>
                </div>
                <button 
                  onClick={() => setShowPluginManager(false)}
                  className="p-1 hover:bg-[#e8e8e8] rounded transition-colors"
                >
                  <X className="w-4 h-4 text-[#6f6f6f]" />
                </button>
              </div>
              
              <div className="flex-1 flex overflow-hidden">
                {/* Sidebar of modal */}
                <div className="w-64 border-r border-[#e5e5e5] bg-[#f3f3f3] overflow-y-auto">
                  <div className="p-2">
                    <div className="px-2 py-1 bg-[#007acc] text-white text-xs rounded-sm mb-1 cursor-pointer">已安装</div>
                    <div className="px-2 py-1 hover:bg-[#e8e8e8] text-[#6f6f6f] text-xs rounded-sm cursor-pointer">推荐插件</div>
                  </div>
                </div>

                {/* Content of modal */}
                <div className="flex-1 overflow-y-auto p-4 grid grid-cols-1 gap-2">
                  {PLUGIN_REGISTRY.map((p) => (
                    <div key={p.metadata.id} className="p-3 bg-white border border-[#e5e5e5] hover:bg-[#f8f8f8] transition-all flex gap-4 cursor-pointer group">
                      <div className="w-10 h-10 bg-[#f3f3f3] flex items-center justify-center text-[#007acc] shrink-0">
                        {p.icon}
                      </div>
                      <div className="flex-1 min-w-0">
                        <div className="flex items-center gap-2 mb-0.5">
                          <h3 className="text-[13px] font-bold text-[#333333]">{p.metadata.name}</h3>
                          <span className="text-[10px] text-[#6f6f6f]">v{p.metadata.version}</span>
                        </div>
                        <p className="text-[11px] text-[#6f6f6f] line-clamp-1 mb-1">
                          {p.metadata.description}
                        </p>
                        <div className="flex items-center gap-3">
                          <span className="text-[10px] font-bold text-[#007acc]">{p.metadata.author}</span>
                          <div className="flex items-center gap-1">
                            <button className="text-[10px] px-2 py-0.5 bg-[#007acc] text-white rounded-sm hover:bg-[#005fb8]">安装</button>
                            <button className="text-[10px] px-2 py-0.5 bg-[#e8e8e8] text-[#333333] rounded-sm hover:bg-[#d8d8d8]"><Settings className="w-3 h-3" /></button>
                          </div>
                        </div>
                      </div>
                    </div>
                  ))}
                </div>
              </div>
            </motion.div>
          </div>
        )}
      </AnimatePresence>
    </div>
  );
}
