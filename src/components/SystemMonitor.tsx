import React from 'react';
import { Activity, Cpu, Database, Zap, ShieldAlert, Terminal, Info, Monitor, Layers, Network } from 'lucide-react';
import { cn } from '../lib/utils';

export function SystemMonitor() {
  return (
    <div className="h-full bg-white flex flex-col overflow-hidden select-text">
      {/* Panel Headers */}
      <div className="h-9 px-4 flex items-center gap-6 border-b border-[#e5e5e5] shrink-0">
        <div className="h-full flex items-center border-b border-[#333333] cursor-pointer">
          <span className="text-[11px] font-bold uppercase tracking-wider text-[#333333]">输出 (Output)</span>
        </div>
        <div className="h-full flex items-center hover:text-[#333333] cursor-pointer text-[#6f6f6f]">
          <span className="text-[11px] font-bold uppercase tracking-wider">终端 (Terminal)</span>
        </div>
        <div className="h-full flex items-center hover:text-[#333333] cursor-pointer text-[#6f6f6f]">
          <span className="text-[11px] font-bold uppercase tracking-wider">调试控制台 (Debug Console)</span>
        </div>
        <div className="h-full flex items-center hover:text-[#333333] cursor-pointer text-[#6f6f6f]">
          <span className="text-[11px] font-bold uppercase tracking-wider">问题 (Problems)</span>
        </div>
      </div>

      {/* Log Content */}
      <div className="flex-1 p-2 overflow-y-auto custom-scrollbar font-mono text-[11px] leading-tight space-y-0.5">
        <div className="flex gap-2">
          <span className="text-[#6f6f6f] shrink-0 select-none w-14">[09:42:15]</span>
          <p className="text-[#333333]"><span className="text-emerald-600">信息 (INFO):</span> MuJoCo WASM 引擎初始化成功。</p>
        </div>
        <div className="flex gap-2">
          <span className="text-[#6f6f6f] shrink-0 select-none w-14">[09:42:16]</span>
          <p className="text-[#333333]"><span className="text-emerald-600">信息 (INFO):</span> WebSocket 已连接至 ws://192.168.1.105:8080/stream</p>
        </div>
        <div className="flex gap-2">
          <span className="text-[#6f6f6f] shrink-0 select-none w-14">[09:42:17]</span>
          <p className="text-[#333333]"><span className="text-blue-600">调试 (DEBUG):</span> 收到状态包: J1=0.452, J2=-1.204, J3=0.881</p>
        </div>
        <div className="flex gap-2">
          <span className="text-[#6f6f6f] shrink-0 select-none w-14">[09:42:18]</span>
          <p className="text-[#333333]"><span className="text-amber-600">警告 (WARN):</span> 关节 J4 扭矩接近阈值 (85%)</p>
        </div>
        <div className="flex gap-2">
          <span className="text-[#6f6f6f] shrink-0 select-none w-14">[09:42:19]</span>
          <p className="text-[#333333]"><span className="text-emerald-600">信息 (INFO):</span> 任务 Task_ID_882 的路径规划已完成。</p>
        </div>
        <div className="flex gap-2">
          <span className="text-[#6f6f6f] shrink-0 select-none w-14">[09:42:20]</span>
          <p className="text-[#333333]"><span className="text-emerald-600">信息 (INFO):</span> 任务执行开始。</p>
        </div>
      </div>
    </div>
  );
}
