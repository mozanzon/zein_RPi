import { useState, useEffect, useRef, useCallback } from 'react';
import { useTheme } from '../contexts/ThemeContext';
import { useWebSocket } from '../contexts/WebSocketContext';

type LatLngTuple = [number, number];

const DEFAULT_CENTER: LatLngTuple = [30.0444, 31.2357]; // Cairo, Egypt

export default function MapPage() {
  const { isDark } = useTheme();
  const { telemetry, status } = useWebSocket();
  const containerRef = useRef<HTMLDivElement>(null);
  const mapRef = useRef<any>(null);
  const leafletRef = useRef<any>(null);
  const markerRef = useRef<any>(null);
  const tileLayerRef = useRef<any>(null);
  const isInitializedRef = useRef(false);

  const [mapReady, setMapReady] = useState(false);
  const [mapError, setMapError] = useState<string | null>(null);

  const isConnected = status === 'connected';

  // ── Initialize Leaflet map ──
  useEffect(() => {
    if (!containerRef.current || isInitializedRef.current) return;
    isInitializedRef.current = true;

    let observer: ResizeObserver | null = null;

    const initMap = async () => {
      try {
        const L = await import('leaflet');
        leafletRef.current = L;

        // Fix default marker icons
        delete (L.Icon.Default.prototype as any)._getIconUrl;
        L.Icon.Default.mergeOptions({
          iconRetinaUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/images/marker-icon-2x.png',
          iconUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/images/marker-icon.png',
          shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/images/marker-shadow.png',
        });

        if (!containerRef.current) {
          setMapError('Map container not found');
          return;
        }

        const map = L.map(containerRef.current, {
          center: DEFAULT_CENTER,
          zoom: 15,
          zoomControl: false,
        });

        // Add tile layer
        const tileUrl = isDark
          ? 'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png'
          : 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png';

        tileLayerRef.current = L.tileLayer(tileUrl, {
          attribution: '© OpenStreetMap contributors',
          maxZoom: 19,
        }).addTo(map);

        // Zoom control
        L.control.zoom({ position: 'bottomright' }).addTo(map);

        // Robot marker
        const robotIcon = L.divIcon({
          html: `<div style="width:16px;height:16px;background:${isDark ? '#00d1ff' : '#10b981'};border:3px solid white;border-radius:50%;box-shadow:0 0 12px ${isDark ? 'rgba(0,209,255,0.6)' : 'rgba(16,185,129,0.6)'}"></div>`,
          className: 'custom-icon',
          iconSize: [16, 16],
          iconAnchor: [8, 8],
        });
        markerRef.current = L.marker(DEFAULT_CENTER, { icon: robotIcon }).addTo(map);

        mapRef.current = map;
        setMapReady(true);
        setMapError(null);

        // Force tile loading
        const resizeMap = () => map.invalidateSize();
        setTimeout(resizeMap, 200);
        setTimeout(resizeMap, 600);
        setTimeout(resizeMap, 1200);

        observer = new ResizeObserver(resizeMap);
        if (containerRef.current) observer.observe(containerRef.current);
      } catch (err) {
        console.error('Map init error:', err);
        setMapError(err instanceof Error ? err.message : 'Failed to initialize map');
      }
    };

    initMap();

    return () => {
      if (observer) observer.disconnect();
      if (mapRef.current) {
        mapRef.current.remove();
        mapRef.current = null;
        isInitializedRef.current = false;
      }
    };
  }, []);

  // ── Keep tile layer synced with theme ──
  useEffect(() => {
    if (!mapReady || !mapRef.current || !leafletRef.current) return;
    const L = leafletRef.current;

    if (tileLayerRef.current) {
      mapRef.current.removeLayer(tileLayerRef.current);
    }

    const tileUrl = isDark
      ? 'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png'
      : 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png';

    tileLayerRef.current = L.tileLayer(tileUrl, {
      attribution: '© OpenStreetMap contributors',
      maxZoom: 19,
    }).addTo(mapRef.current);
  }, [isDark, mapReady]);

  // ── Compass heading from telemetry ──
  const heading = telemetry?.compass ?? 0;

  return (
    <div className="relative h-full flex flex-col overflow-hidden">
      {/* Map fills available space */}
      <div className="flex-1 relative min-h-0 bg-slate-900">
        <div
          ref={containerRef}
          style={{ position: 'absolute', top: 0, left: 0, right: 0, bottom: 0, backgroundColor: isDark ? '#0f172a' : '#e2e8f0' }}
        />

        {/* Loading spinner */}
        {!mapReady && !mapError && (
          <div className="absolute inset-0 flex items-center justify-center bg-black/20 backdrop-blur-sm z-50">
            <div className="flex flex-col items-center gap-3">
              <div className="w-8 h-8 border-3 border-cyan-400 border-t-transparent rounded-full animate-spin" />
              <div className={`text-xs font-semibold ${isDark ? 'text-cyan-400' : 'text-white'}`}>Loading Map...</div>
            </div>
          </div>
        )}

        {/* Error message */}
        {mapError && (
          <div className="absolute inset-0 flex items-center justify-center bg-black/40 backdrop-blur-sm z-50">
            <div className={`p-4 rounded-lg max-w-xs ${isDark ? 'bg-red-900/80 border border-red-500' : 'bg-red-100 border border-red-300'}`}>
              <div className={`text-sm font-bold ${isDark ? 'text-red-200' : 'text-red-800'}`}>Map Error</div>
              <div className={`text-xs mt-1 ${isDark ? 'text-red-300' : 'text-red-700'}`}>{mapError}</div>
              <button
                onClick={() => window.location.reload()}
                className="mt-2 px-3 py-1 bg-red-500 text-white text-xs font-bold rounded hover:bg-red-600"
              >
                Retry
              </button>
            </div>
          </div>
        )}

        {/* Map info overlay */}
        <div className={`absolute top-3 left-3 z-[1000] rounded-lg px-2.5 py-1.5 ${isDark ? 'glass' : 'glass-light'}`}>
          <div className={`text-[9px] font-semibold tracking-wider uppercase ${isDark ? 'text-zinc-400' : 'text-zinc-500'}`}>
            {isDark ? 'CartoDB Dark' : 'OpenStreetMap'}
          </div>
        </div>

        {/* Connection + heading status */}
        <div className={`absolute top-3 right-3 z-[1000] rounded-xl px-3 py-2 ${isDark ? 'glass' : 'glass-light'}`}>
          <div className={`text-[8px] font-bold tracking-wider uppercase ${isDark ? 'text-zinc-400' : 'text-zinc-500'}`}>Status</div>
          <div className="flex items-center gap-1.5 mt-0.5">
            <div className={`w-2 h-2 rounded-full ${isConnected ? 'bg-emerald-400 animate-pulse' : 'bg-red-400'}`} />
            <span className={`text-xs font-bold ${isDark ? 'text-white' : 'text-zinc-700'}`}>
              {isConnected ? 'CONNECTED' : 'OFFLINE'}
            </span>
          </div>
        </div>

        {/* Heading display */}
        <div className={`absolute top-3 left-1/2 -translate-x-1/2 z-[1000] rounded-2xl px-5 py-2 ${
          isDark ? 'bg-emerald-600/90 backdrop-blur-md' : 'bg-emerald-500 shadow-lg'
        }`}>
          <div className="text-[8px] text-white/80 font-bold tracking-wider uppercase text-center">Heading</div>
          <div className="text-lg font-black text-white font-['JetBrains_Mono',monospace] text-center">
            {telemetry ? `${heading.toFixed(0)}` : '--'}<span className="text-sm text-white/70 ml-0.5">°</span>
          </div>
        </div>
      </div>

      {/* Bottom panel — Telemetry from Arduino */}
      <div className={`shrink-0 border-t p-3 transition-theme ${
        isDark ? 'bg-[#0d0d14] border-[#1a1a2e]' : 'bg-white border-[#e0ddd5]'
      }`}>
        {/* Connection status banner */}
        {!isConnected && (
          <div className={`rounded-lg p-2 mb-2 border text-center ${
            isDark ? 'bg-red-500/10 border-red-500/20' : 'bg-red-50 border-red-200'
          }`}>
            <div className={`text-[10px] font-bold ${isDark ? 'text-red-400' : 'text-red-500'}`}>
              ⚠️ Not connected — No live telemetry
            </div>
          </div>
        )}

        {/* GPS notice */}
        <div className={`rounded-lg p-2 mb-2 ${isDark ? 'bg-[#12121c] border border-[#1e1e32]' : 'bg-zinc-50 border border-zinc-200'}`}>
          <div className="flex items-center justify-between">
            <span className={`text-[9px] font-semibold tracking-[1.2px] uppercase ${isDark ? 'text-zinc-500' : 'text-zinc-500'}`}>
              GPS Position
            </span>
            <span className={`text-[9px] font-bold ${isDark ? 'text-amber-400' : 'text-amber-600'}`}>
              NOT AVAILABLE
            </span>
          </div>
          <div className={`text-[8px] mt-1 ${isDark ? 'text-zinc-600' : 'text-zinc-400'}`}>
            GPS module not integrated in bridge server. Map shows default position.
          </div>
        </div>

        {/* Live IMU telemetry */}
        <div className={`text-[9px] font-semibold tracking-[1.5px] uppercase font-['Space_Grotesk',sans-serif] mb-2 ${
          isDark ? 'text-zinc-600' : 'text-zinc-400'
        }`}>
          ARDUINO TELEMETRY
        </div>
        <div className="grid grid-cols-4 gap-2">
          {[
            { label: 'Pitch', value: telemetry ? `${telemetry.pitch.toFixed(1)}°` : '--', color: isDark ? 'text-emerald-400' : 'text-emerald-600' },
            { label: 'Roll', value: telemetry ? `${telemetry.roll.toFixed(1)}°` : '--', color: isDark ? 'text-cyan-400' : 'text-cyan-600' },
            { label: 'Yaw', value: telemetry ? `${telemetry.yaw.toFixed(1)}°` : '--', color: isDark ? 'text-violet-400' : 'text-violet-600' },
            { label: 'Battery', value: telemetry ? `${telemetry.battery}%` : '--', color: isDark ? 'text-amber-400' : 'text-amber-600' },
          ].map((s) => (
            <div key={s.label} className={`rounded-lg p-2 text-center ${
              isDark ? 'bg-[#12121c] border border-[#1e1e32]' : 'bg-zinc-50 border border-zinc-200'
            }`}>
              <div className={`text-[7px] font-bold tracking-wider uppercase mb-0.5 ${isDark ? 'text-zinc-600' : 'text-zinc-400'}`}>{s.label}</div>
              <span className={`text-sm font-bold font-['JetBrains_Mono',monospace] ${
                telemetry ? s.color : (isDark ? 'text-red-400' : 'text-red-500')
              }`}>{s.value}</span>
            </div>
          ))}
        </div>

        {/* Encoder data */}
        <div className="grid grid-cols-2 gap-2 mt-2">
          <div className={`rounded-lg p-2 text-center ${isDark ? 'bg-[#12121c] border border-[#1e1e32]' : 'bg-zinc-50 border border-zinc-200'}`}>
            <div className={`text-[7px] font-bold tracking-wider uppercase mb-0.5 ${isDark ? 'text-zinc-600' : 'text-zinc-400'}`}>Left Encoder</div>
            <span className={`text-sm font-bold font-['JetBrains_Mono',monospace] ${
              telemetry ? (isDark ? 'text-emerald-400' : 'text-emerald-600') : (isDark ? 'text-red-400' : 'text-red-500')
            }`}>{telemetry ? telemetry.encL : '--'}</span>
          </div>
          <div className={`rounded-lg p-2 text-center ${isDark ? 'bg-[#12121c] border border-[#1e1e32]' : 'bg-zinc-50 border border-zinc-200'}`}>
            <div className={`text-[7px] font-bold tracking-wider uppercase mb-0.5 ${isDark ? 'text-zinc-600' : 'text-zinc-400'}`}>Right Encoder</div>
            <span className={`text-sm font-bold font-['JetBrains_Mono',monospace] ${
              telemetry ? (isDark ? 'text-emerald-400' : 'text-emerald-600') : (isDark ? 'text-red-400' : 'text-red-500')
            }`}>{telemetry ? telemetry.encR : '--'}</span>
          </div>
        </div>
      </div>
    </div>
  );
}
