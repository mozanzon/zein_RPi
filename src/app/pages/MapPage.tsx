import { useState, useEffect, useRef } from 'react';
import { useTheme } from '../contexts/ThemeContext';
import { useControlMode } from '../contexts/ControlModeContext';

interface Detection {
  type: 'crack' | 'pothole';
  position: [number, number];
  timestamp: Date;
}

type LatLngTuple = [number, number];

const START_POSITION: LatLngTuple = [30.0444, 31.2357];
const FOLLOW_STEP_INTERVAL_MS = 250;
const FOLLOW_SPEED_MPS = 1.2;
const WAYPOINT_REACH_THRESHOLD_M = 2.5;

function haversineMeters(from: LatLngTuple, to: LatLngTuple) {
  const earthRadius = 6371e3;
  const phi1 = (from[0] * Math.PI) / 180;
  const phi2 = (to[0] * Math.PI) / 180;
  const deltaPhi = ((to[0] - from[0]) * Math.PI) / 180;
  const deltaLambda = ((to[1] - from[1]) * Math.PI) / 180;
  const a = Math.sin(deltaPhi / 2) ** 2 + Math.cos(phi1) * Math.cos(phi2) * Math.sin(deltaLambda / 2) ** 2;
  return earthRadius * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}

function moveTowardPoint(current: LatLngTuple, target: LatLngTuple, maxMeters: number): LatLngTuple {
  const remaining = haversineMeters(current, target);
  if (remaining <= maxMeters || remaining === 0) return target;
  const ratio = maxMeters / remaining;
  return [
    current[0] + (target[0] - current[0]) * ratio,
    current[1] + (target[1] - current[1]) * ratio,
  ];
}

export default function MapPage() {
  const { isDark } = useTheme();
  const { mode } = useControlMode();
  const containerRef = useRef<HTMLDivElement>(null);
  const mapRef = useRef<any>(null);
  const leafletRef = useRef<any>(null);
  const markerRef = useRef<any>(null);
  const polylineRef = useRef<any>(null);
  const waypointLineRef = useRef<any>(null);
  const waypointMarkersRef = useRef<any[]>([]);

  const [robotPosition, setRobotPosition] = useState<LatLngTuple>(START_POSITION);
  const [path, setPath] = useState<LatLngTuple[]>([START_POSITION]);
  const [waypoints, setWaypoints] = useState<LatLngTuple[]>([]);
  const [currentWaypointIndex, setCurrentWaypointIndex] = useState(0);
  const [followState, setFollowState] = useState<'idle' | 'following' | 'paused' | 'complete'>('idle');
  const [detections, setDetections] = useState<Detection[]>([]);
  const [stats, setStats] = useState({ areaMapped: 452, scanRate: 12.5, confidence: 98 });
  const [totalDistance, setTotalDistance] = useState(0);
  const [mapReady, setMapReady] = useState(false);
  const activeWaypoint = currentWaypointIndex < waypoints.length ? waypoints[currentWaypointIndex] : null;
  const remainingWaypoints = Math.max(0, waypoints.length - currentWaypointIndex);
  const followStatusLabel = followState === 'following'
    ? 'FOLLOWING'
    : followState === 'paused'
      ? 'PAUSED'
      : followState === 'complete'
        ? 'COMPLETE'
        : 'IDLE';
  const followStatusColor = followState === 'following'
    ? (isDark ? 'text-cyan-400' : 'text-cyan-600')
    : followState === 'paused'
      ? (isDark ? 'text-amber-400' : 'text-amber-600')
      : followState === 'complete'
        ? (isDark ? 'text-emerald-400' : 'text-emerald-600')
        : (isDark ? 'text-zinc-500' : 'text-zinc-500');

  // Initialize map
  useEffect(() => {
    if (!containerRef.current || mapRef.current) return;

    let observer: ResizeObserver | null = null;
    let clickHandler: ((evt: any) => void) | null = null;

    const initMap = async () => {
      const L = await import('leaflet');
      leafletRef.current = L;

      delete (L.Icon.Default.prototype as any)._getIconUrl;
      L.Icon.Default.mergeOptions({
        iconRetinaUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/images/marker-icon-2x.png',
        iconUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/images/marker-icon.png',
        shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/images/marker-shadow.png',
      });

      if (!containerRef.current) return;

      const map = L.map(containerRef.current, {
        center: START_POSITION,
        zoom: 15,
        zoomControl: false,
      });

      L.tileLayer(
        isDark
          ? 'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png'
          : 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
        { attribution: '© OpenStreetMap', maxZoom: 19 }
      ).addTo(map);

      L.control.zoom({ position: 'bottomright' }).addTo(map);

      const robotIcon = L.divIcon({
        html: `<div style="width:16px;height:16px;background:${isDark ? '#00d1ff' : '#10b981'};border:3px solid white;border-radius:50%;box-shadow:0 0 12px ${isDark ? 'rgba(0,209,255,0.6)' : 'rgba(16,185,129,0.6)'}"></div>`,
        className: 'custom-icon',
        iconSize: [16, 16],
        iconAnchor: [8, 8],
      });

      markerRef.current = L.marker(START_POSITION, { icon: robotIcon }).addTo(map);
      polylineRef.current = L.polyline([START_POSITION], {
        color: isDark ? '#00d1ff' : '#10b981',
        weight: 3, opacity: 0.7, dashArray: '8, 4',
      }).addTo(map);
      waypointLineRef.current = L.polyline([], {
        color: isDark ? '#f59e0b' : '#ea580c',
        weight: 2,
        opacity: 0.9,
        dashArray: '6, 6',
      }).addTo(map);

      mapRef.current = map;
      setMapReady(true);

      clickHandler = (evt: any) => {
        const point: LatLngTuple = [evt.latlng.lat, evt.latlng.lng];
        setWaypoints((prev) => [...prev, point]);
      };
      map.on('click', clickHandler);

      // Force tile loading with repeated invalidateSize
      const resizeMap = () => map.invalidateSize();
      setTimeout(resizeMap, 200);
      setTimeout(resizeMap, 600);
      setTimeout(resizeMap, 1200);

      // Also listen for resize events
      observer = new ResizeObserver(resizeMap);
      if (containerRef.current) observer.observe(containerRef.current);
    };

    initMap();

    return () => {
      if (clickHandler && mapRef.current) {
        mapRef.current.off('click', clickHandler);
      }
      if (observer) {
        observer.disconnect();
      }
      if (mapRef.current) {
        mapRef.current.remove();
        mapRef.current = null;
      }
    };
  }, []);

  // Draw waypoint markers and planned route
  useEffect(() => {
    if (!mapReady || !mapRef.current || !leafletRef.current) return;
    const L = leafletRef.current;

    waypointMarkersRef.current.forEach((marker) => marker.remove());
    waypointMarkersRef.current = [];

    waypoints.forEach((point, idx) => {
      const isActive = idx === currentWaypointIndex && followState === 'following';
      const marker = L.circleMarker(point, {
        radius: isActive ? 7 : 5,
        color: isActive ? '#f59e0b' : (isDark ? '#22d3ee' : '#059669'),
        weight: 2,
        fillColor: isActive ? '#fbbf24' : (isDark ? '#06b6d4' : '#10b981'),
        fillOpacity: 0.95,
      }).addTo(mapRef.current);

      marker.bindTooltip(`${idx + 1}`, {
        permanent: true,
        direction: 'top',
        offset: [0, -8],
      });

      waypointMarkersRef.current.push(marker);
    });

    if (waypointLineRef.current) {
      const startIdx = Math.min(currentWaypointIndex, waypoints.length);
      const remainingPath = waypoints.slice(startIdx);
      const planned = remainingPath.length ? [robotPosition, ...remainingPath] : [];
      waypointLineRef.current.setLatLngs(planned);
    }
  }, [waypoints, currentWaypointIndex, followState, mapReady, isDark, robotPosition]);

  // Keep robot marker and traveled path synced
  useEffect(() => {
    if (markerRef.current) markerRef.current.setLatLng(robotPosition);
    if (polylineRef.current) polylineRef.current.setLatLngs(path);
  }, [robotPosition, path]);

  // Pause following if user leaves AUTO mode
  useEffect(() => {
    if (mode !== 'auto' && followState === 'following') {
      setFollowState('paused');
    }
  }, [mode, followState]);

  // Waypoint following simulation
  useEffect(() => {
    if (mode !== 'auto' || followState !== 'following' || !activeWaypoint) return;

    const interval = setInterval(() => {
      setRobotPosition((prev) => {
        const target = activeWaypoint;
        if (!target) return prev;

        const stepMeters = FOLLOW_SPEED_MPS * (FOLLOW_STEP_INTERVAL_MS / 1000);
        const distanceToTarget = haversineMeters(prev, target);
        const reached = distanceToTarget <= Math.max(stepMeters, WAYPOINT_REACH_THRESHOLD_M);
        const nextPosition = reached ? target : moveTowardPoint(prev, target, stepMeters);

        setPath((p) => [...p, nextPosition]);
        setStats((p) => ({
          ...p,
          areaMapped: p.areaMapped + Math.floor(Math.random() * 2),
          scanRate: 10 + Math.random() * 4,
        }));

        if (Math.random() > 0.95) {
          setDetections((p) => [
            ...p,
            {
              type: Math.random() > 0.5 ? 'crack' : 'pothole',
              position: nextPosition,
              timestamp: new Date(),
            },
          ]);
        }

        if (reached) {
          setCurrentWaypointIndex((idx) => {
            const next = idx + 1;
            if (next >= waypoints.length) {
              setFollowState('complete');
              return waypoints.length;
            }
            return next;
          });
        }

        return nextPosition;
      });
    }, FOLLOW_STEP_INTERVAL_MS);

    return () => clearInterval(interval);
  }, [mode, followState, activeWaypoint, waypoints.length]);

  // Distance calculation
  useEffect(() => {
    if (path.length < 2) return;
    let distanceMeters = 0;
    for (let i = 1; i < path.length; i++) {
      distanceMeters += haversineMeters(path[i - 1], path[i]);
    }
    setTotalDistance(distanceMeters);
  }, [path]);

  const startFollowing = () => {
    if (mode !== 'auto' || waypoints.length === 0) return;
    if (followState === 'complete' || currentWaypointIndex >= waypoints.length) {
      setCurrentWaypointIndex(0);
    }
    setFollowState('following');
  };

  const pauseFollowing = () => {
    if (followState === 'following') setFollowState('paused');
  };

  const stopFollowing = () => {
    setFollowState('idle');
    setCurrentWaypointIndex(0);
  };

  const clearWaypoints = () => {
    setWaypoints([]);
    setCurrentWaypointIndex(0);
    setFollowState('idle');
  };

  const resetTrail = () => {
    setPath([robotPosition]);
    setTotalDistance(0);
  };

  return (
    <div className="relative h-full flex flex-col overflow-hidden">
      {/* Map fills available space */}
      <div className="flex-1 relative min-h-0">
        <div
          ref={containerRef}
          style={{ position: 'absolute', top: 0, left: 0, right: 0, bottom: 0 }}
        />

        {/* Waypoint hint */}
        <div className={`absolute top-3 left-3 z-[1000] rounded-lg px-2.5 py-1.5 ${isDark ? 'glass' : 'glass-light'}`}>
          <div className={`text-[9px] font-semibold tracking-wider uppercase ${isDark ? 'text-zinc-400' : 'text-zinc-500'}`}>
            Tap map to add waypoints
          </div>
        </div>

        {/* Distance badge */}
        <div className={`absolute top-3 left-1/2 -translate-x-1/2 z-[1000] rounded-2xl px-5 py-2 ${
          isDark ? 'bg-emerald-600/90 backdrop-blur-md' : 'bg-emerald-500 shadow-lg'
        }`}>
          <div className="text-[8px] text-white/80 font-bold tracking-wider uppercase text-center">Distance</div>
          <div className="text-lg font-black text-white font-['JetBrains_Mono',monospace] text-center">
            {totalDistance.toFixed(1)}<span className="text-sm text-white/70 ml-0.5">m</span>
          </div>
        </div>

        {/* Status badge */}
        <div className={`absolute top-3 right-3 z-[1000] rounded-xl px-3 py-2 ${
          isDark ? 'glass' : 'glass-light'
        }`}>
          <div className={`text-[8px] font-bold tracking-wider uppercase ${isDark ? 'text-zinc-400' : 'text-zinc-500'}`}>Status</div>
          <div className="flex items-center gap-1.5 mt-0.5">
            <div className={`w-2 h-2 rounded-full animate-pulse ${followState === 'following' ? 'bg-cyan-400' : mode === 'auto' ? 'bg-emerald-400' : 'bg-zinc-400'}`} />
            <span className={`text-xs font-bold ${isDark ? 'text-white' : 'text-zinc-700'}`}>
              {mode === 'auto' ? 'AUTO' : 'MANUAL'} / {followStatusLabel}
            </span>
          </div>
        </div>

        {/* Coordinates */}
        <div className={`absolute bottom-3 left-3 z-[1000] rounded-lg px-2.5 py-1.5 ${isDark ? 'glass' : 'glass-light'}`}>
          <div className={`text-[10px] font-mono ${isDark ? 'text-cyan-400' : 'text-emerald-600'}`}>
            {robotPosition[0].toFixed(6)}°, {robotPosition[1].toFixed(6)}°
          </div>
        </div>
      </div>

      {/* Stats panel at bottom */}
      <div className={`shrink-0 border-t p-3 transition-theme ${
        isDark ? 'bg-[#0d0d14] border-[#1a1a2e]' : 'bg-white border-[#e0ddd5]'
      }`}>
        <div className={`rounded-lg p-2 mb-2 ${isDark ? 'bg-[#12121c] border border-[#1e1e32]' : 'bg-zinc-50 border border-zinc-200'}`}>
          <div className="flex items-center justify-between">
            <span className={`text-[9px] font-semibold tracking-[1.2px] uppercase ${isDark ? 'text-zinc-500' : 'text-zinc-500'}`}>Waypoint Mission</span>
            <span className={`text-[10px] font-bold ${followStatusColor}`}>{followStatusLabel}</span>
          </div>
          <div className="grid grid-cols-3 gap-2 mt-2">
            <div className={`rounded-md p-1.5 text-center ${isDark ? 'bg-[#0a0a14] border border-[#1a1a2e]' : 'bg-white border border-zinc-200'}`}>
              <div className={`text-[7px] font-bold tracking-wider uppercase ${isDark ? 'text-zinc-600' : 'text-zinc-400'}`}>Total</div>
              <div className={`text-xs font-bold font-['JetBrains_Mono',monospace] ${isDark ? 'text-cyan-400' : 'text-cyan-600'}`}>{waypoints.length}</div>
            </div>
            <div className={`rounded-md p-1.5 text-center ${isDark ? 'bg-[#0a0a14] border border-[#1a1a2e]' : 'bg-white border border-zinc-200'}`}>
              <div className={`text-[7px] font-bold tracking-wider uppercase ${isDark ? 'text-zinc-600' : 'text-zinc-400'}`}>Remaining</div>
              <div className={`text-xs font-bold font-['JetBrains_Mono',monospace] ${isDark ? 'text-amber-400' : 'text-amber-600'}`}>{remainingWaypoints}</div>
            </div>
            <div className={`rounded-md p-1.5 text-center ${isDark ? 'bg-[#0a0a14] border border-[#1a1a2e]' : 'bg-white border border-zinc-200'}`}>
              <div className={`text-[7px] font-bold tracking-wider uppercase ${isDark ? 'text-zinc-600' : 'text-zinc-400'}`}>Active</div>
              <div className={`text-xs font-bold font-['JetBrains_Mono',monospace] ${isDark ? 'text-emerald-400' : 'text-emerald-600'}`}>
                {activeWaypoint ? `#${Math.min(currentWaypointIndex + 1, waypoints.length)}` : '--'}
              </div>
            </div>
          </div>
          <div className="grid grid-cols-3 gap-2 mt-2">
            <button
              onClick={startFollowing}
              disabled={mode !== 'auto' || waypoints.length === 0 || followState === 'following'}
              className={`py-1.5 rounded-md text-[8px] font-bold tracking-wider uppercase transition-all disabled:opacity-40 ${
                isDark ? 'bg-cyan-500/20 text-cyan-400 border border-cyan-500/30' : 'bg-cyan-50 text-cyan-600 border border-cyan-200'
              }`}
            >
              {followState === 'paused' ? 'Resume' : 'Start'}
            </button>
            <button
              onClick={pauseFollowing}
              disabled={followState !== 'following'}
              className={`py-1.5 rounded-md text-[8px] font-bold tracking-wider uppercase transition-all disabled:opacity-40 ${
                isDark ? 'bg-amber-500/20 text-amber-400 border border-amber-500/30' : 'bg-amber-50 text-amber-600 border border-amber-200'
              }`}
            >
              Pause
            </button>
            <button
              onClick={stopFollowing}
              disabled={followState === 'idle' && currentWaypointIndex === 0}
              className={`py-1.5 rounded-md text-[8px] font-bold tracking-wider uppercase transition-all disabled:opacity-40 ${
                isDark ? 'bg-red-500/20 text-red-400 border border-red-500/30' : 'bg-red-50 text-red-500 border border-red-200'
              }`}
            >
              Stop
            </button>
          </div>
          <div className="grid grid-cols-2 gap-2 mt-2">
            <button
              onClick={clearWaypoints}
              disabled={waypoints.length === 0}
              className={`py-1.5 rounded-md text-[8px] font-bold tracking-wider uppercase transition-all disabled:opacity-40 ${
                isDark ? 'bg-[#1a1a2e] text-zinc-400 border border-[#2a2a3e]' : 'bg-zinc-100 text-zinc-600 border border-zinc-200'
              }`}
            >
              Clear WPs
            </button>
            <button
              onClick={resetTrail}
              className={`py-1.5 rounded-md text-[8px] font-bold tracking-wider uppercase transition-all ${
                isDark ? 'bg-[#1a1a2e] text-zinc-400 border border-[#2a2a3e]' : 'bg-zinc-100 text-zinc-600 border border-zinc-200'
              }`}
            >
              Reset Trail
            </button>
          </div>
          {mode !== 'auto' && (
            <div className={`text-[8px] mt-2 ${isDark ? 'text-amber-400' : 'text-amber-700'}`}>
              Switch to AUTO mode in Controls tab to run waypoint following.
            </div>
          )}
        </div>

        <div className={`text-[9px] font-semibold tracking-[1.5px] uppercase font-['Space_Grotesk',sans-serif] mb-2 ${
          isDark ? 'text-zinc-600' : 'text-zinc-400'
        }`}>
          TELEMETRY
        </div>
        <div className="grid grid-cols-4 gap-2">
          {[
            { label: 'Area', value: stats.areaMapped, unit: 'm²', color: isDark ? 'text-emerald-400' : 'text-emerald-600' },
            { label: 'Rate', value: stats.scanRate.toFixed(1), unit: 'Hz', color: isDark ? 'text-cyan-400' : 'text-cyan-600' },
            { label: 'Detect', value: detections.length, unit: '', color: isDark ? 'text-red-400' : 'text-red-500' },
            { label: 'Conf', value: stats.confidence, unit: '%', color: isDark ? 'text-violet-400' : 'text-violet-600' },
          ].map((s) => (
            <div key={s.label} className={`rounded-lg p-2 text-center ${
              isDark ? 'bg-[#12121c] border border-[#1e1e32]' : 'bg-zinc-50 border border-zinc-200'
            }`}>
              <div className={`text-[7px] font-bold tracking-wider uppercase mb-0.5 ${isDark ? 'text-zinc-600' : 'text-zinc-400'}`}>{s.label}</div>
              <span className={`text-sm font-bold font-['JetBrains_Mono',monospace] ${s.color}`}>{s.value}</span>
              {s.unit && <span className={`text-[8px] ml-0.5 ${isDark ? 'text-zinc-700' : 'text-zinc-400'}`}>{s.unit}</span>}
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}
