import { useState } from 'react';
import { useTheme } from '../contexts/ThemeContext';
import { useWebSocket } from '../contexts/WebSocketContext';

interface PIDValues { kp: number; ki: number; kd: number; }
interface PIDTuningProps { onUpdate: (values: PIDValues) => void; }

export default function PIDTuning({ onUpdate }: PIDTuningProps) {
  const { isDark } = useTheme();
  const { sendCommand, status } = useWebSocket();
  const [pid, setPid] = useState<PIDValues>({ kp: 1.0, ki: 0.1, kd: 0.05 });

  const isConnected = status === 'connected';

  const updatePID = (key: keyof PIDValues, value: number) => {
    const n = { ...pid, [key]: value };
    setPid(n);
    onUpdate(n);
  };

  const sendPIDToArduino = () => {
    // Send PID_SET command matching motor_imu_controller.ino format: PID_SET,kp,ki,kd
    sendCommand(`PID_SET,${pid.kp},${pid.ki},${pid.kd}`);
  };

  const applyPreset = (values: PIDValues) => {
    setPid(values);
    onUpdate(values);
    sendCommand(`PID_SET,${values.kp},${values.ki},${values.kd}`);
  };

  const cardCls = `rounded-2xl p-4 transition-theme ${isDark ? 'bg-[#12121c] border border-[#1e1e32]' : 'bg-white border border-[#e8e5dd]'}`;
  const labelCls = `text-[10px] font-semibold tracking-[1.5px] uppercase font-['Space_Grotesk',sans-serif] ${isDark ? 'text-zinc-500' : 'text-zinc-400'}`;

  const params = [
    { key: 'kp' as const, label: 'Kp (Proportional)', desc: 'Current error response', max: 5, step: 0.1 },
    { key: 'ki' as const, label: 'Ki (Integral)', desc: 'Accumulated error', max: 2, step: 0.01 },
    { key: 'kd' as const, label: 'Kd (Derivative)', desc: 'Rate of change', max: 1, step: 0.01 },
  ];

  const presets = [
    { name: 'Default', values: { kp: 1.0, ki: 0.1, kd: 0.05 } },
    { name: 'Smooth', values: { kp: 1.5, ki: 0.2, kd: 0.1 } },
    { name: 'Aggressive', values: { kp: 2.0, ki: 0.05, kd: 0.15 } },
  ];

  return (
    <div className={cardCls}>
      <div className="flex items-center justify-between mb-4">
        <div className={labelCls}>PID CONTROLLER</div>
        <button
          onClick={sendPIDToArduino}
          disabled={!isConnected}
          className={`px-3 py-1.5 rounded-full text-[9px] font-bold tracking-wider uppercase transition-all disabled:opacity-40 ${
            isDark ? 'bg-cyan-500/20 text-cyan-400 border border-cyan-500/30 hover:bg-cyan-500/30' : 'bg-emerald-50 text-emerald-600 border border-emerald-200 hover:bg-emerald-100'
          }`}
        >
          SEND TO ARDUINO
        </button>
      </div>

      {!isConnected && (
        <div className={`rounded-xl px-3 py-2 mb-3 border text-[10px] font-semibold tracking-wider uppercase text-center ${
          isDark ? 'bg-red-500/10 border-red-500/20 text-red-400' : 'bg-red-50 border-red-200 text-red-500'
        }`}>
          ⚠️ Connect to send PID values
        </div>
      )}

      <div className="space-y-4">
        {params.map((p) => (
          <div key={p.key}>
            <div className="flex items-end justify-between mb-1.5">
              <div>
                <div className={`text-[9px] font-semibold tracking-wider uppercase ${isDark ? 'text-zinc-400' : 'text-zinc-500'}`}>{p.label}</div>
                <div className={`text-[8px] ${isDark ? 'text-zinc-600' : 'text-zinc-400'}`}>{p.desc}</div>
              </div>
              <input type="number" value={pid[p.key]} step={p.step}
                onChange={(e) => updatePID(p.key, Number(e.target.value))}
                className={`w-[56px] rounded-lg px-2 py-1 text-xs text-right font-mono font-bold outline-none ${
                  isDark ? 'bg-[#0a0a14] border border-[#1a1a2e] text-emerald-400 focus:border-cyan-500/40' : 'bg-zinc-50 border border-zinc-200 text-emerald-600 focus:border-emerald-400'
                }`} />
            </div>
            <input type="range" min="0" max={p.max} step={p.step} value={pid[p.key]}
              onChange={(e) => updatePID(p.key, Number(e.target.value))}
              className={`w-full h-1.5 rounded-full appearance-none cursor-pointer ${isDark ? 'bg-[#1a1a2e]' : 'bg-zinc-100'}`} />
          </div>
        ))}
      </div>

      <div className="grid grid-cols-3 gap-2 mt-4">
        {presets.map((p) => (
          <button key={p.name} onClick={() => applyPreset(p.values)}
            disabled={!isConnected}
            className={`rounded-xl py-2.5 text-[9px] font-bold tracking-wider uppercase transition-all hover:scale-[1.02] active:scale-[0.98] disabled:opacity-40 ${
              isDark ? 'bg-[#1a1a2e] text-zinc-400 hover:bg-[#252540] border border-[#2a2a3e]' : 'bg-zinc-50 text-zinc-500 hover:bg-zinc-100 border border-zinc-200'
            }`}>
            {p.name}
          </button>
        ))}
      </div>
    </div>
  );
}
