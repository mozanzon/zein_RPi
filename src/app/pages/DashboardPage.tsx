import { useState, useEffect } from 'react';
import { useTheme } from '../contexts/ThemeContext';
import { useControlMode } from '../contexts/ControlModeContext';
import { useWebSocket } from '../contexts/WebSocketContext';

// Wheel parameters matching Arduino
const WHEEL_DIAMETER_M = 0.32;
const WHEEL_CIRC = Math.PI * WHEEL_DIAMETER_M;
const TICKS_PER_REV = 600;

function StatCard({ label, value, unit, icon, color, isDark, hasError }: {
  label: string; value: string | number; unit?: string; icon: string;
  color: string; isDark: boolean; hasError?: boolean;
}) {
  return (
    <div className={`rounded-2xl p-4 transition-theme ${
      isDark
        ? 'bg-[#12121c] border border-[#1e1e32]'
        : 'bg-white border border-[#e8e5dd]'
    }`}>
      <div className="flex items-start justify-between mb-2">
        <span className="text-lg">{icon}</span>
        <span className={`text-[9px] font-semibold tracking-[1.5px] uppercase font-['Space_Grotesk',sans-serif] ${
          isDark ? 'text-zinc-600' : 'text-zinc-400'
        }`}>
          {label}
        </span>
      </div>
      <div className="flex items-baseline gap-1">
        {hasError ? (
          <span className={`text-lg font-bold font-['JetBrains_Mono',monospace] ${isDark ? 'text-red-400' : 'text-red-500'}`}>
            NO DATA
          </span>
        ) : (
          <>
            <span className={`text-2xl font-bold font-['JetBrains_Mono',monospace] ${color}`}>
              {value}
            </span>
            {unit && (
              <span className={`text-xs ${isDark ? 'text-zinc-600' : 'text-zinc-400'}`}>
                {unit}
              </span>
            )}
          </>
        )}
      </div>
    </div>
  );
}

function ProgressBar({ value, max, color, isDark }: {
  value: number; max: number; color: string; isDark: boolean;
}) {
  const pct = Math.min(100, (value / max) * 100);
  return (
    <div className={`h-2 rounded-full overflow-hidden ${isDark ? 'bg-[#1a1a2e]' : 'bg-zinc-100'}`}>
      <div
        className={`h-full rounded-full transition-all duration-700 ${color}`}
        style={{ width: `${pct}%` }}
      />
    </div>
  );
}

export default function DashboardPage() {
  const { isDark } = useTheme();
  const { mode } = useControlMode();
  const { telemetry, status, rpiStats } = useWebSocket();
  const [uptime, setUptime] = useState(0);

  // Uptime counter (counts seconds since component mounted while connected)
  useEffect(() => {
    if (status !== 'connected') return;
    const interval = setInterval(() => {
      setUptime(p => p + 1);
    }, 1000);
    return () => clearInterval(interval);
  }, [status]);

  const isConnected = status === 'connected';
  const hasData = telemetry !== null;
  const noData = isConnected && !hasData;

  // Derive values from Arduino telemetry
  const battery = telemetry?.battery ?? 0;
  const encL = telemetry?.encL ?? 0;
  const encR = telemetry?.encR ?? 0;

  // Compute distance from encoder ticks
  const leftDist = (Math.abs(encL) / TICKS_PER_REV) * WHEEL_CIRC;
  const rightDist = (Math.abs(encR) / TICKS_PER_REV) * WHEEL_CIRC;
  const distanceTraveled = (leftDist + rightDist) / 2;

  // RPi stats (from bridge server or defaults)
  const cpu = rpiStats?.cpu ?? 0;
  const memory = rpiStats?.memory ?? 0;
  const temperature = rpiStats?.temperature ?? 0;
  const signal = isConnected ? 100 : 0;
  const cracksTotal = telemetry?.cracksDetected ?? 0;
  const potholesTotal = telemetry?.potholesDetected ?? 0;

  const formatUptime = (seconds: number) => {
    const h = Math.floor(seconds / 3600);
    const m = Math.floor((seconds % 3600) / 60);
    const s = seconds % 60;
    return `${h.toString().padStart(2, '0')}:${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
  };

  // Status banner
  const StatusBanner = () => {
    if (status === 'disconnected' || status === 'error') {
      return (
        <div className={`rounded-2xl p-4 mb-4 border text-center ${
          isDark ? 'bg-red-500/10 border-red-500/20' : 'bg-red-50 border-red-200'
        }`}>
          <div className={`text-sm font-bold ${isDark ? 'text-red-400' : 'text-red-500'}`}>
            ⚠️ Not Connected
          </div>
          <div className={`text-xs mt-1 ${isDark ? 'text-red-400/60' : 'text-red-400'}`}>
            Go to Link tab to connect to the Raspberry Pi
          </div>
        </div>
      );
    }
    if (noData) {
      return (
        <div className={`rounded-2xl p-4 mb-4 border text-center ${
          isDark ? 'bg-amber-500/10 border-amber-500/20' : 'bg-amber-50 border-amber-200'
        }`}>
          <div className={`text-sm font-bold ${isDark ? 'text-amber-400' : 'text-amber-600'}`}>
            ⏳ Waiting for Arduino data...
          </div>
          <div className={`text-xs mt-1 ${isDark ? 'text-amber-400/60' : 'text-amber-500'}`}>
            Connected to RPi but no telemetry received from Arduino yet
          </div>
        </div>
      );
    }
    return null;
  };

  return (
    <div className={`min-h-full p-4 space-y-4 pb-4 ${isDark ? '' : ''}`}>
      {/* Status Banner */}
      <StatusBanner />

      {/* System Overview Banner */}
      <div className={`rounded-2xl p-5 ${
        isDark
          ? 'bg-gradient-to-r from-cyan-500/10 via-emerald-500/10 to-teal-500/10 border border-cyan-500/20'
          : 'bg-gradient-to-r from-emerald-50 via-teal-50 to-cyan-50 border border-emerald-200'
      }`}>
        <div className="flex items-center justify-between">
          <div>
            <div className={`text-[10px] font-semibold tracking-[2px] uppercase font-['Space_Grotesk',sans-serif] mb-1 ${
              isDark ? 'text-cyan-400/80' : 'text-emerald-600'
            }`}>
              SYSTEM OVERVIEW
            </div>
            <div className={`text-lg font-bold font-['Space_Grotesk',sans-serif] ${
              isDark ? 'text-white' : 'text-zinc-800'
            }`}>
              {isConnected && hasData ? 'All Systems Operational' : isConnected ? 'Waiting for Arduino' : 'System Offline'}
            </div>
          </div>
          <div className={`text-right font-['JetBrains_Mono',monospace] ${
            isDark ? 'text-cyan-400' : 'text-emerald-600'
          }`}>
            <div className="text-[10px] tracking-wider uppercase opacity-60">Uptime</div>
            <div className="text-lg font-bold">{formatUptime(uptime)}</div>
          </div>
        </div>
      </div>

      {/* Key Metrics Grid */}
      <div className="grid grid-cols-2 gap-3">
        <StatCard
          label="Battery"
          value={hasData ? battery.toFixed(0) : '--'}
          unit="%"
          icon="🔋"
          color={battery > 50 ? (isDark ? 'text-emerald-400' : 'text-emerald-600') : (isDark ? 'text-amber-400' : 'text-amber-600')}
          isDark={isDark}
          hasError={noData}
        />
        <StatCard
          label="Signal"
          value={isConnected ? '100' : '0'}
          unit="%"
          icon="📶"
          color={isDark ? 'text-cyan-400' : 'text-cyan-600'}
          isDark={isDark}
          hasError={!isConnected}
        />
        <StatCard
          label="CPU Load"
          value={rpiStats ? cpu.toFixed(0) : '--'}
          unit="%"
          icon="⚡"
          color={cpu > 70 ? (isDark ? 'text-red-400' : 'text-red-500') : (isDark ? 'text-emerald-400' : 'text-emerald-600')}
          isDark={isDark}
          hasError={!rpiStats && isConnected}
        />
        <StatCard
          label="Temp"
          value={rpiStats ? temperature.toFixed(1) : '--'}
          unit="°C"
          icon="🌡️"
          color={temperature > 45 ? (isDark ? 'text-red-400' : 'text-red-500') : (isDark ? 'text-sky-400' : 'text-sky-600')}
          isDark={isDark}
          hasError={!rpiStats && isConnected}
        />
      </div>

      {/* Motor Status — derived from encoder data */}
      <div className={`rounded-2xl p-5 transition-theme ${
        isDark ? 'bg-[#12121c] border border-[#1e1e32]' : 'bg-white border border-[#e8e5dd]'
      }`}>
        <div className={`text-[10px] font-semibold tracking-[1.5px] uppercase font-['Space_Grotesk',sans-serif] mb-4 ${
          isDark ? 'text-zinc-500' : 'text-zinc-400'
        }`}>
          MOTOR STATUS
        </div>
        {noData ? (
          <div className={`text-center py-4 text-sm font-bold ${isDark ? 'text-red-400' : 'text-red-500'}`}>
            ⚠️ No encoder data from Arduino
          </div>
        ) : (
          <div className="grid grid-cols-2 gap-4">
            <div>
              <div className="flex items-center justify-between mb-2">
                <span className={`text-xs font-medium ${isDark ? 'text-zinc-400' : 'text-zinc-500'}`}>Left Encoder</span>
                <span className={`text-sm font-bold font-['JetBrains_Mono',monospace] ${isDark ? 'text-emerald-400' : 'text-emerald-600'}`}>
                  {encL}
                </span>
              </div>
              <ProgressBar value={Math.abs(encL) % 10000} max={10000} color="bg-emerald-500" isDark={isDark} />
            </div>
            <div>
              <div className="flex items-center justify-between mb-2">
                <span className={`text-xs font-medium ${isDark ? 'text-zinc-400' : 'text-zinc-500'}`}>Right Encoder</span>
                <span className={`text-sm font-bold font-['JetBrains_Mono',monospace] ${isDark ? 'text-emerald-400' : 'text-emerald-600'}`}>
                  {encR}
                </span>
              </div>
              <ProgressBar value={Math.abs(encR) % 10000} max={10000} color="bg-emerald-500" isDark={isDark} />
            </div>
          </div>
        )}
      </div>

      {/* Paint & Distance */}
      <div className="grid grid-cols-2 gap-3">
        <div className={`rounded-2xl p-4 transition-theme ${
          isDark ? 'bg-[#12121c] border border-[#1e1e32]' : 'bg-white border border-[#e8e5dd]'
        }`}>
          <div className="flex items-center gap-2 mb-3">
            <span className="text-lg">🎨</span>
            <span className={`text-[9px] font-semibold tracking-[1.5px] uppercase font-['Space_Grotesk',sans-serif] ${
              isDark ? 'text-zinc-500' : 'text-zinc-400'
            }`}>
              Paint Level
            </span>
          </div>
          <div className={`text-2xl font-bold font-['JetBrains_Mono',monospace] mb-2 ${
            isDark ? 'text-zinc-500' : 'text-zinc-400'
          }`}>
            N/A
          </div>
          <div className={`text-[9px] ${isDark ? 'text-zinc-600' : 'text-zinc-400'}`}>
            No paint sensor on Arduino
          </div>
        </div>

        <div className={`rounded-2xl p-4 transition-theme ${
          isDark ? 'bg-[#12121c] border border-[#1e1e32]' : 'bg-white border border-[#e8e5dd]'
        }`}>
          <div className="flex items-center gap-2 mb-3">
            <span className="text-lg">📏</span>
            <span className={`text-[9px] font-semibold tracking-[1.5px] uppercase font-['Space_Grotesk',sans-serif] ${
              isDark ? 'text-zinc-500' : 'text-zinc-400'
            }`}>
              Distance
            </span>
          </div>
          <div className={`text-2xl font-bold font-['JetBrains_Mono',monospace] ${
            hasData ? (isDark ? 'text-cyan-400' : 'text-cyan-600') : (isDark ? 'text-red-400' : 'text-red-500')
          }`}>
            {hasData ? distanceTraveled.toFixed(1) : '--'}
          </div>
          <span className={`text-xs ${isDark ? 'text-zinc-600' : 'text-zinc-400'}`}>meters</span>
        </div>
      </div>

      {/* Detection Summary */}
      <div className={`rounded-2xl p-5 transition-theme ${
        isDark ? 'bg-[#12121c] border border-[#1e1e32]' : 'bg-white border border-[#e8e5dd]'
      }`}>
        <div className={`text-[10px] font-semibold tracking-[1.5px] uppercase font-['Space_Grotesk',sans-serif] mb-4 ${
          isDark ? 'text-zinc-500' : 'text-zinc-400'
        }`}>
          AI DETECTION SUMMARY
        </div>
        <div className="grid grid-cols-3 gap-3">
          <div className={`rounded-xl p-3 text-center ${
            isDark ? 'bg-red-500/10 border border-red-500/20' : 'bg-red-50 border border-red-200'
          }`}>
            <div className={`text-xl font-bold font-['JetBrains_Mono',monospace] ${
              isDark ? 'text-red-400' : 'text-red-500'
            }`}>
              {cracksTotal}
            </div>
            <div className={`text-[9px] font-semibold tracking-wider uppercase mt-1 ${
              isDark ? 'text-red-400/60' : 'text-red-400'
            }`}>
              Cracks
            </div>
          </div>
          <div className={`rounded-xl p-3 text-center ${
            isDark ? 'bg-amber-500/10 border border-amber-500/20' : 'bg-amber-50 border border-amber-200'
          }`}>
            <div className={`text-xl font-bold font-['JetBrains_Mono',monospace] ${
              isDark ? 'text-amber-400' : 'text-amber-600'
            }`}>
              {potholesTotal}
            </div>
            <div className={`text-[9px] font-semibold tracking-wider uppercase mt-1 ${
              isDark ? 'text-amber-400/60' : 'text-amber-400'
            }`}>
              Potholes
            </div>
          </div>
          <div className={`rounded-xl p-3 text-center ${
            isDark ? 'bg-emerald-500/10 border border-emerald-500/20' : 'bg-emerald-50 border border-emerald-200'
          }`}>
            <div className={`text-xl font-bold font-['JetBrains_Mono',monospace] ${
              isDark ? 'text-emerald-400' : 'text-emerald-600'
            }`}>
              {cracksTotal + potholesTotal}
            </div>
            <div className={`text-[9px] font-semibold tracking-wider uppercase mt-1 ${
              isDark ? 'text-emerald-400/60' : 'text-emerald-400'
            }`}>
              Total
            </div>
          </div>
        </div>
      </div>

      {/* Resource Usage */}
      <div className={`rounded-2xl p-5 transition-theme ${
        isDark ? 'bg-[#12121c] border border-[#1e1e32]' : 'bg-white border border-[#e8e5dd]'
      }`}>
        <div className={`text-[10px] font-semibold tracking-[1.5px] uppercase font-['Space_Grotesk',sans-serif] mb-4 ${
          isDark ? 'text-zinc-500' : 'text-zinc-400'
        }`}>
          RESOURCE USAGE
        </div>
        <div className="space-y-4">
          <div>
            <div className="flex items-center justify-between mb-1.5">
              <span className={`text-xs font-medium ${isDark ? 'text-zinc-400' : 'text-zinc-500'}`}>CPU</span>
              <span className={`text-xs font-mono font-bold ${rpiStats ? (isDark ? 'text-cyan-400' : 'text-cyan-600') : (isDark ? 'text-zinc-600' : 'text-zinc-400')}`}>
                {rpiStats ? `${cpu.toFixed(0)}%` : 'N/A'}
              </span>
            </div>
            <ProgressBar value={cpu} max={100} color={cpu > 70 ? "bg-red-500" : "bg-cyan-500"} isDark={isDark} />
          </div>
          <div>
            <div className="flex items-center justify-between mb-1.5">
              <span className={`text-xs font-medium ${isDark ? 'text-zinc-400' : 'text-zinc-500'}`}>Memory</span>
              <span className={`text-xs font-mono font-bold ${rpiStats ? (isDark ? 'text-violet-400' : 'text-violet-600') : (isDark ? 'text-zinc-600' : 'text-zinc-400')}`}>
                {rpiStats ? `${memory.toFixed(0)}%` : 'N/A'}
              </span>
            </div>
            <ProgressBar value={memory} max={100} color="bg-violet-500" isDark={isDark} />
          </div>
          <div>
            <div className="flex items-center justify-between mb-1.5">
              <span className={`text-xs font-medium ${isDark ? 'text-zinc-400' : 'text-zinc-500'}`}>Battery</span>
              <span className={`text-xs font-mono font-bold ${hasData ? (battery > 50 ? (isDark ? 'text-emerald-400' : 'text-emerald-600') : (isDark ? 'text-amber-400' : 'text-amber-600')) : (isDark ? 'text-zinc-600' : 'text-zinc-400')}`}>
                {hasData ? `${battery.toFixed(0)}%` : 'N/A'}
              </span>
            </div>
            <ProgressBar value={hasData ? battery : 0} max={100} color={battery > 50 ? "bg-emerald-500" : "bg-amber-500"} isDark={isDark} />
          </div>
        </div>
      </div>
    </div>
  );
}
