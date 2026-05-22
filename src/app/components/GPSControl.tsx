import { useState } from 'react';
import { useTheme } from '../contexts/ThemeContext';
import { useWebSocket } from '../contexts/WebSocketContext';

interface GPSControlProps { onLocationChange: (location: [number, number]) => void; }

export default function GPSControl({ onLocationChange }: GPSControlProps) {
  const { isDark } = useTheme();
  const { status } = useWebSocket();
  // GPS module (NEO-6M) is connected to the Raspberry Pi
  // The bridge server does not currently stream GPS data
  // This component is a placeholder for when GPS is integrated into the bridge

  const isConnected = status === 'connected';

  const cardCls = `rounded-2xl p-4 transition-theme ${isDark ? 'bg-[#12121c] border border-[#1e1e32]' : 'bg-white border border-[#e8e5dd]'}`;
  const labelCls = `text-[10px] font-semibold tracking-[1.5px] uppercase font-['Space_Grotesk',sans-serif] ${isDark ? 'text-zinc-500' : 'text-zinc-400'}`;

  return (
    <div className={cardCls}>
      <div className="flex items-center justify-between mb-3">
        <div className={labelCls}>GPS NAVIGATION</div>
        <div className={`px-3 py-1 rounded-full text-[9px] font-bold tracking-wider uppercase ${
          isDark ? 'bg-amber-500/10 text-amber-400 border border-amber-500/20' : 'bg-amber-50 text-amber-600 border border-amber-200'
        }`}>
          NOT AVAILABLE
        </div>
      </div>

      <div className={`rounded-xl p-4 border text-center ${
        isDark ? 'bg-amber-500/5 border-amber-500/20' : 'bg-amber-50 border-amber-200'
      }`}>
        <div className={`text-2xl mb-2`}>📡</div>
        <div className={`text-sm font-bold ${isDark ? 'text-amber-400' : 'text-amber-600'}`}>
          GPS Not Integrated
        </div>
        <div className={`text-[11px] mt-2 leading-relaxed ${isDark ? 'text-zinc-500' : 'text-zinc-400'}`}>
          The NEO-6M GPS module is connected to the Raspberry Pi but the bridge server
          (<code className="font-mono text-[10px]">server.py</code>) does not currently
          stream GPS coordinates. To enable GPS:
        </div>
        <div className={`text-[10px] mt-3 text-left space-y-1 ${isDark ? 'text-zinc-600' : 'text-zinc-400'}`}>
          <div>1. Add <code className="font-mono">gpsd</code> + <code className="font-mono">gps3</code> to bridge server</div>
          <div>2. Send <code className="font-mono">{`{"type":"gps","lat":..,"lng":..}`}</code> via WebSocket</div>
          <div>3. This component will auto-display the data</div>
        </div>
      </div>

      {!isConnected && (
        <div className={`mt-3 rounded-xl px-3 py-2 border text-[10px] font-semibold tracking-wider uppercase text-center ${
          isDark ? 'bg-red-500/10 border-red-500/20 text-red-400' : 'bg-red-50 border-red-200 text-red-500'
        }`}>
          ⚠️ Not connected to Raspberry Pi
        </div>
      )}
    </div>
  );
}
