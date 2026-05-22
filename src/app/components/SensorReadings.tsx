import { useState, useEffect, useRef } from 'react';
import { useTheme } from '../contexts/ThemeContext';
import { useWebSocket } from '../contexts/WebSocketContext';

// Wheel parameters matching Arduino bridge_controller.ino
const WHEEL_DIAMETER_M = 0.32;  // 32 cm wheel diameter (from motor_imu_controller.ino)
const WHEEL_CIRC = Math.PI * WHEEL_DIAMETER_M;
const TICKS_PER_REV = 600;      // encoder ticks per revolution

interface ComputedEncoder {
  ticks: number;
  rpm: number;
  speed: number; // m/s
  distance: number; // meters
}

export default function SensorReadings() {
  const { isDark } = useTheme();
  const { telemetry, status } = useWebSocket();

  // Track previous values for RPM/speed calculation
  const prevRef = useRef<{ encL: number; encR: number; time: number } | null>(null);
  const [computed, setComputed] = useState<{ left: ComputedEncoder; right: ComputedEncoder }>({
    left: { ticks: 0, rpm: 0, speed: 0, distance: 0 },
    right: { ticks: 0, rpm: 0, speed: 0, distance: 0 },
  });

  // Compute derived values from raw encoder ticks
  useEffect(() => {
    if (!telemetry) return;

    const now = Date.now();
    const prev = prevRef.current;

    let leftRPM = 0;
    let rightRPM = 0;
    let leftSpeed = 0;
    let rightSpeed = 0;

    if (prev) {
      const dtSec = (now - prev.time) / 1000;
      if (dtSec > 0 && dtSec < 5) {
        const deltaL = telemetry.encL - prev.encL;
        const deltaR = telemetry.encR - prev.encR;
        const revsL = Math.abs(deltaL) / TICKS_PER_REV;
        const revsR = Math.abs(deltaR) / TICKS_PER_REV;
        leftRPM = (revsL / dtSec) * 60;
        rightRPM = (revsR / dtSec) * 60;
        leftSpeed = (revsL * WHEEL_CIRC) / dtSec;
        rightSpeed = (revsR * WHEEL_CIRC) / dtSec;
      }
    }

    prevRef.current = { encL: telemetry.encL, encR: telemetry.encR, time: now };

    const leftDist = (Math.abs(telemetry.encL) / TICKS_PER_REV) * WHEEL_CIRC;
    const rightDist = (Math.abs(telemetry.encR) / TICKS_PER_REV) * WHEEL_CIRC;

    setComputed({
      left: { ticks: telemetry.encL, rpm: leftRPM, speed: leftSpeed, distance: leftDist },
      right: { ticks: telemetry.encR, rpm: rightRPM, speed: rightSpeed, distance: rightDist },
    });
  }, [telemetry]);

  const isConnected = status === 'connected';
  const hasData = telemetry !== null;
  const showError = isConnected && !hasData;
  const showOffline = !isConnected;

  const cardCls = `rounded-2xl p-4 transition-theme ${isDark ? 'bg-[#12121c] border border-[#1e1e32]' : 'bg-white border border-[#e8e5dd]'}`;
  const subCardCls = `rounded-xl p-3 ${isDark ? 'bg-[#0a0a14] border border-[#1a1a2e]' : 'bg-zinc-50 border border-zinc-200'}`;
  const labelCls = `text-[10px] font-semibold tracking-[1.5px] uppercase font-['Space_Grotesk',sans-serif] ${isDark ? 'text-zinc-500' : 'text-zinc-400'}`;
  const valCls = `font-bold font-['JetBrains_Mono',monospace] ${isDark ? 'text-emerald-400' : 'text-emerald-600'}`;
  const unitCls = `text-[10px] ${isDark ? 'text-zinc-600' : 'text-zinc-400'}`;
  const errorCls = `text-[10px] font-bold font-['JetBrains_Mono',monospace] ${isDark ? 'text-red-400' : 'text-red-500'}`;

  const getDirection = (deg: number) => {
    if (deg >= 337.5 || deg < 22.5) return 'N';
    if (deg < 67.5) return 'NE';
    if (deg < 112.5) return 'E';
    if (deg < 157.5) return 'SE';
    if (deg < 202.5) return 'S';
    if (deg < 247.5) return 'SW';
    if (deg < 292.5) return 'W';
    return 'NW';
  };

  const renderValue = (value: string | number, unit?: string) => {
    if (showOffline) return <span className={errorCls}>OFFLINE</span>;
    if (showError) return <span className={errorCls}>NO DATA</span>;
    return <>{value}{unit && <span className={unitCls}> {unit}</span>}</>;
  };

  // Status banner
  const StatusBanner = () => {
    if (isConnected && hasData) return null;
    const msg = showOffline ? 'Not connected to hardware' : 'Waiting for sensor data from Arduino...';
    const color = isDark ? 'bg-red-500/10 border-red-500/20 text-red-400' : 'bg-red-50 border-red-200 text-red-500';
    return (
      <div className={`rounded-xl px-3 py-2 mb-3 border text-[10px] font-semibold tracking-wider uppercase text-center ${color}`}>
        ⚠️ {msg}
      </div>
    );
  };

  return (
    <div className={cardCls}>
      <div className={`${labelCls} mb-4`}>SENSOR READINGS</div>

      <StatusBanner />

      {/* Full Encoder Readings */}
      <div className="space-y-3 mb-4">
        {(['left', 'right'] as const).map((side) => {
          const enc = side === 'left' ? computed.left : computed.right;
          return (
            <div key={side} className={subCardCls}>
              <div className={`text-[9px] font-semibold tracking-wider uppercase mb-2 ${isDark ? 'text-zinc-500' : 'text-zinc-400'}`}>
                {side === 'left' ? '⬅️' : '➡️'} {side.toUpperCase()} ENCODER
              </div>
              <div className="grid grid-cols-4 gap-2">
                <div>
                  <div className={`text-[8px] uppercase ${isDark ? 'text-zinc-600' : 'text-zinc-400'}`}>Ticks</div>
                  <div className={`text-sm ${hasData ? valCls : errorCls}`}>
                    {renderValue(enc.ticks.toLocaleString())}
                  </div>
                </div>
                <div>
                  <div className={`text-[8px] uppercase ${isDark ? 'text-zinc-600' : 'text-zinc-400'}`}>RPM</div>
                  <div className={`text-sm ${hasData ? valCls : errorCls}`}>
                    {renderValue(enc.rpm.toFixed(0))}
                  </div>
                </div>
                <div>
                  <div className={`text-[8px] uppercase ${isDark ? 'text-zinc-600' : 'text-zinc-400'}`}>Speed</div>
                  <div className={`text-sm ${hasData ? valCls : errorCls}`}>
                    {renderValue(enc.speed.toFixed(2), 'm/s')}
                  </div>
                </div>
                <div>
                  <div className={`text-[8px] uppercase ${isDark ? 'text-zinc-600' : 'text-zinc-400'}`}>Dist</div>
                  <div className={`text-sm ${hasData ? valCls : errorCls}`}>
                    {renderValue(enc.distance.toFixed(1), 'm')}
                  </div>
                </div>
              </div>
            </div>
          );
        })}
      </div>

      {/* IMU / Orientation */}
      <div className={subCardCls + ' mb-3'}>
        <div className={`text-[9px] font-semibold tracking-wider uppercase mb-2 ${isDark ? 'text-zinc-500' : 'text-zinc-400'}`}>
          🧭 IMU ORIENTATION
        </div>
        <div className="grid grid-cols-3 gap-2">
          {[
            { label: 'Pitch', value: telemetry?.pitch ?? 0 },
            { label: 'Roll', value: telemetry?.roll ?? 0 },
            { label: 'Yaw', value: telemetry?.yaw ?? 0 },
          ].map((item) => (
            <div key={item.label} className="text-center">
              <div className={`text-[8px] uppercase ${isDark ? 'text-zinc-600' : 'text-zinc-400'}`}>{item.label}</div>
              <div className={`text-sm ${hasData ? valCls : errorCls}`}>
                {renderValue(item.value.toFixed(1), '°')}
              </div>
            </div>
          ))}
        </div>
      </div>

      {/* Compass */}
      <div className={subCardCls}>
        <div className="flex items-center justify-between mb-3">
          <div className={`text-[9px] font-semibold tracking-wider uppercase ${isDark ? 'text-zinc-500' : 'text-zinc-400'}`}>
            🧭 COMPASS
          </div>
          <div className="flex items-baseline gap-1">
            <span className={`text-xl ${hasData ? valCls : errorCls}`}>
              {hasData ? (telemetry?.compass ?? 0).toFixed(0) : '---'}
            </span>
            {hasData && <span className={unitCls}>° {getDirection(telemetry?.compass ?? 0)}</span>}
          </div>
        </div>
        <div className="relative w-full aspect-square max-w-[180px] mx-auto">
          <svg viewBox="0 0 200 200" className="w-full h-full">
            <circle cx="100" cy="100" r="90" fill="none" stroke={isDark ? '#1e1e32' : '#e5e7eb'} strokeWidth="2" />
            {['N', 'E', 'S', 'W'].map((d, i) => {
              const positions = [[100, 18], [184, 104], [100, 192], [16, 104]];
              return (
                <text key={d} x={positions[i][0]} y={positions[i][1]} textAnchor="middle"
                  fill={isDark ? '#6b7280' : '#9ca3af'} fontSize="13" fontWeight="bold" fontFamily="Space Grotesk">
                  {d}
                </text>
              );
            })}
            {Array.from({ length: 36 }, (_, i) => {
              const a = (i * 10 - 90) * (Math.PI / 180);
              const r1 = 85, r2 = i % 3 === 0 ? 75 : 80;
              return (
                <line key={i} x1={100 + r1 * Math.cos(a)} y1={100 + r1 * Math.sin(a)}
                  x2={100 + r2 * Math.cos(a)} y2={100 + r2 * Math.sin(a)}
                  stroke={isDark ? '#2a2a3e' : '#d1d5db'} strokeWidth={i % 3 === 0 ? 2 : 1} />
              );
            })}
            <g transform={`rotate(${telemetry?.compass ?? 0} 100 100)`} style={{ transition: 'transform 0.3s ease' }}>
              <polygon points="100,28 106,100 100,112 94,100" fill={isDark ? '#ef4444' : '#dc2626'} />
              <polygon points="100,112 106,100 100,172 94,100" fill={isDark ? '#374151' : '#d1d5db'} />
              <circle cx="100" cy="100" r="7" fill={isDark ? '#10b981' : '#059669'} stroke="white" strokeWidth="2" />
            </g>
          </svg>
        </div>
      </div>
    </div>
  );
}
