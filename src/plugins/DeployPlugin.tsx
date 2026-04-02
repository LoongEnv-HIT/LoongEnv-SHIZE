import React, { useState } from 'react';
import { Play, CheckCircle2, Cpu, Link, Download, Radio } from 'lucide-react';
import { motion } from 'motion/react';
import { Plugin } from '../types';
import { cn } from '../lib/utils';
import { CollapsibleSection } from '../components/CollapsibleSection';

export const DeployPlugin: Plugin = {
  metadata: {
    id: 'deploy',
    name: '实机部署与控制',
    description: '将训练好的策略部署至实时控制内核。',
    version: '1.0.5',
    author: 'Control Team'
  },
  icon: <Play className="w-5 h-5" />,
  stepTitle: '实机部署与控制',
  techStack: ['C++ RT', 'ONNX Runtime', 'pybind11'],
  inputSchema: { target_hw: 'Jetson Orin', freq: '1kHz' },
  outputSchema: { jitter: '15us', status: 'RUNNING' },
  component: ({ data, onAction }) => {
    const [activeStep, setActiveStep] = useState(1);
    const steps = [
      { id: 1, label: '硬件连接 (Hardware)', icon: <Link className="w-3.5 h-3.5" /> },
      { id: 2, label: '部署配置 (Deployment)', icon: <Download className="w-3.5 h-3.5" /> },
      { id: 3, label: '实时控制 (Control)', icon: <Radio className="w-3.5 h-3.5" /> },
    ];

    return (
      <div className="flex flex-col h-full bg-white text-[#333333] select-none">
        {/* VSCode-style Step Navigation */}
        <div className="grid grid-cols-3 px-2 bg-[#f3f3f3] border-b border-[#e5e5e5] shrink-0 gap-1">
          {steps.map((step) => (
            <button
              key={step.id}
              onClick={() => setActiveStep(step.id)}
              className={cn(
                "min-w-0 px-2 py-2.5 text-[11px] font-medium transition-all relative flex flex-col items-center justify-center gap-1 text-center leading-snug rounded-sm",
                activeStep === step.id 
                  ? "text-[#333333] border-b-2 border-[#007acc] bg-white/50" 
                  : "text-[#6f6f6f] hover:text-[#333333] hover:bg-[#e8e8e8]"
              )}
            >
              {step.icon}
              {step.label}
            </button>
          ))}
        </div>

        {/* Content Area */}
        <div className="flex-1 overflow-y-auto custom-scrollbar p-4 space-y-6">
          {activeStep === 1 && (
            <motion.div initial={{ opacity: 0, y: 5 }} animate={{ opacity: 1, y: 0 }} className="space-y-6">
              <CollapsibleSection title="硬件终端 (Hardware Terminal)" defaultOpen>
                <div className="border border-[#e5e5e5] rounded-sm p-4 bg-[#f8f8f8] flex items-center gap-3 shadow-sm hover:border-[#cccccc] transition-colors">
                  <div className="w-10 h-10 bg-white rounded-sm flex items-center justify-center border border-[#e5e5e5]">
                    <Cpu className="w-6 h-6 text-[#007acc]" />
                  </div>
                  <div>
                    <p className="text-[11px] font-bold text-[#333333]">Jetson Orin AGX</p>
                    <p className="text-[10px] text-[#6f6f6f]">IP: 192.168.1.105 | SSH 已就绪</p>
                  </div>
                </div>
                <button className="w-full mt-3 py-2 bg-[#333333] text-white rounded-sm text-[10px] font-bold hover:bg-[#1e1e1e] transition-all uppercase tracking-widest shadow-sm active:scale-[0.98]">
                  建立 SSH 连接 (Establish SSH Connection)
                </button>
              </CollapsibleSection>
              
              <CollapsibleSection title="通信链路 (Communication Links)" defaultOpen={false}>
                <div className="space-y-1 border border-[#e5e5e5] rounded-sm overflow-hidden shadow-sm divide-y divide-[#f3f3f3]">
                  {['EtherCAT 总线', 'CAN-FD 接口', 'ROS2 桥接'].map((label) => (
                    <div key={label} className="flex items-center justify-between px-3 py-2.5 bg-white hover:bg-[#f9f9f9] transition-colors group">
                      <div className="flex items-center gap-2">
                        <div className="w-1.5 h-1.5 bg-[#388a3c] rounded-full shadow-sm animate-pulse" />
                        <span className="text-[11px] text-[#333333] font-medium">{label}</span>
                      </div>
                      <span className="text-[9px] font-bold text-[#388a3c] uppercase tracking-wider opacity-60 group-hover:opacity-100 transition-opacity">已连接 (Connected)</span>
                    </div>
                  ))}
                </div>
              </CollapsibleSection>
            </motion.div>
          )}

          {activeStep === 2 && (
            <motion.div initial={{ opacity: 0, y: 5 }} animate={{ opacity: 1, y: 0 }} className="space-y-6">
              <CollapsibleSection title="部署流程 (Deployment Progress)" defaultOpen>
                <div className="flex flex-col items-center justify-center py-8 text-center space-y-6">
                  <div className="relative">
                    <Download className="w-10 h-10 text-[#007acc] animate-bounce" />
                    <div className="absolute -bottom-1 -right-1 w-4 h-4 bg-emerald-500 rounded-full border-2 border-white flex items-center justify-center">
                      <CheckCircle2 className="w-2.5 h-2.5 text-white" />
                    </div>
                  </div>
                  <div className="space-y-2">
                    <h4 className="text-sm font-bold text-[#333333]">正在部署控制内核 (Deploying Control Kernel)</h4>
                    <p className="text-[11px] text-[#6f6f6f] max-w-[260px] mx-auto leading-relaxed">
                      正在将策略转换为 TensorRT 并同步到实时框架。
                    </p>
                  </div>
                  <div className="w-full max-w-[260px] space-y-2">
                    <div className="h-1 bg-[#e5e5e5] rounded-full overflow-hidden shadow-inner">
                      <motion.div 
                        initial={{ width: 0 }} 
                        animate={{ width: '68%' }} 
                        className="h-full bg-[#007acc]" 
                      />
                    </div>
                    <div className="flex justify-between text-[9px] font-bold text-[#6f6f6f] uppercase tracking-wider">
                      <span>编译 (Compile): 68%</span>
                      <span>同步 (Sync): 100%</span>
                    </div>
                  </div>
                </div>
              </CollapsibleSection>
            </motion.div>
          )}

          {activeStep === 3 && (
            <motion.div initial={{ opacity: 0, y: 5 }} animate={{ opacity: 1, y: 0 }} className="space-y-6">
              <CollapsibleSection title="控制状态 (Control Status)" defaultOpen>
                <div className="p-4 bg-[#f0fdf4] border border-[#dcfce7] rounded-sm shadow-sm">
                  <div className="flex items-center gap-3 mb-4">
                    <div className="w-8 h-8 bg-[#388a3c] rounded-sm flex items-center justify-center text-white shadow-md">
                      <CheckCircle2 className="w-5 h-5" />
                    </div>
                    <div>
                      <p className="text-[11px] font-bold text-[#166534]">实时控制已激活 (Active)</p>
                      <p className="text-[9px] text-[#166534] opacity-70 mt-0.5">抖动 (Jitter): 15us | 1000Hz</p>
                    </div>
                  </div>
                  <button className="w-full py-2.5 bg-[#d32f2f] text-white rounded-sm text-[10px] font-bold hover:bg-[#b71c1c] transition-all uppercase tracking-widest shadow-md active:scale-[0.98]">
                    紧急停止 (E-STOP)
                  </button>
                </div>
              </CollapsibleSection>
              
              <CollapsibleSection title="控制流 (Control Stream)" defaultOpen>
                <div className="bg-white border border-[#e5e5e5] rounded-sm overflow-hidden shadow-sm divide-y divide-[#f3f3f3]">
                  {[1, 2, 3].map(i => (
                    <div key={i} className="flex justify-between items-center text-[10px] font-mono py-2.5 hover:bg-[#f9f9f9] px-3 transition-colors">
                      <span className="text-[#6f6f6f]">J{i}_目标 (TARGET)</span>
                      <span className="text-[#007acc] font-bold">{(Math.random() * 3.14).toFixed(4)}</span>
                    </div>
                  ))}
                </div>
              </CollapsibleSection>
              
              <CollapsibleSection title="传感器反馈 (Sensor Feedback)" defaultOpen={false}>
                <div className="bg-white border border-[#e5e5e5] rounded-sm overflow-hidden shadow-sm divide-y divide-[#f3f3f3]">
                  {[{ name: 'IMU_X', val: '0.02' }, { name: '温度 (TEMP)', val: '42.5°C' }].map(s => (
                    <div key={s.name} className="flex justify-between items-center text-[10px] font-mono py-2.5 hover:bg-[#f9f9f9] px-3 transition-colors">
                      <span className="text-[#6f6f6f]">{s.name}</span>
                      <span className="text-[#388a3c] font-bold">{s.val}</span>
                    </div>
                  ))}
                </div>
              </CollapsibleSection>
            </motion.div>
          )}
        </div>

        {/* Action Bar */}
        <div className="p-4 border-t border-[#e5e5e5] bg-[#f3f3f3] shrink-0">
          <button 
            onClick={() => onAction('COMPLETE', 'DEPLOY_FINISHED')}
            className="w-full bg-[#007acc] hover:bg-[#0062a3] text-white py-2 text-[11px] font-bold uppercase tracking-widest shadow-sm transition-all flex items-center justify-center gap-2 active:scale-[0.98] rounded-sm"
          >
            <Play className="w-3.5 h-3.5 fill-current" /> 开启控制会话 (Start Control Session)
          </button>
        </div>
      </div>
    );
  }
};
