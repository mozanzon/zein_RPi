import { createContext, useContext, useState, useRef, useCallback, useEffect, ReactNode } from 'react';

// ── Types matching Arduino bridge_controller.ino output ──
// Arduino sends: {"type":"telemetry","encL":int,"encR":int,"pitch":float,"roll":float,"yaw":float,"compass":float,"battery":int}
// Bridge server forwards as: {"type":"telemetry_update","data":{...}}

export interface ArduinoTelemetry {
  type?: string;
  encL: number;
  encR: number;
  pitch: number;
  roll: number;
  yaw: number;
  compass: number;
  battery: number;
  cracksDetected?: number;
  potholesDetected?: number;
}

export interface VideoFrame {
  frame: string; // base64 JPEG
  detections: Detection[];
}

export interface Detection {
  type: 'crack' | 'pothole';
  x: number;
  y: number;
  width: number;
  height: number;
  confidence: number;
}

export interface LogEntry {
  id: number;
  timestamp: Date;
  level: 'info' | 'warn' | 'error' | 'debug';
  source: string;
  message: string;
}

export type ConnectionStatus = 'disconnected' | 'connecting' | 'connected' | 'error';

interface WebSocketContextType {
  // Connection
  status: ConnectionStatus;
  connect: (ip: string, port: string) => void;
  disconnect: () => void;
  connectionError: string | null;

  // Telemetry from Arduino
  telemetry: ArduinoTelemetry | null;
  lastTelemetryTime: number | null;

  // Video
  videoFrame: VideoFrame | null;

  // Commands
  sendCommand: (cmd: string) => void;
  toggleAI: (enabled: boolean) => void;

  // System logs (real from WebSocket)
  logs: LogEntry[];
  clearLogs: () => void;

  // Computed: RPi system stats (from bridge server)
  rpiStats: RpiStats | null;
}

export interface RpiStats {
  cpu: number;
  memory: number;
  temperature: number;
  signal: number;
  uptime: number;
}

const WebSocketContext = createContext<WebSocketContextType | undefined>(undefined);

let logIdCounter = 0;

export function WebSocketProvider({ children }: { children: ReactNode }) {
  const wsRef = useRef<WebSocket | null>(null);
  const [status, setStatus] = useState<ConnectionStatus>('disconnected');
  const [connectionError, setConnectionError] = useState<string | null>(null);
  const [telemetry, setTelemetry] = useState<ArduinoTelemetry | null>(null);
  const [lastTelemetryTime, setLastTelemetryTime] = useState<number | null>(null);
  const [videoFrame, setVideoFrame] = useState<VideoFrame | null>(null);
  const [logs, setLogs] = useState<LogEntry[]>([]);
  const [rpiStats, setRpiStats] = useState<RpiStats | null>(null);
  const connectTimeRef = useRef<number>(0);
  const reconnectTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const ipRef = useRef('');
  const portRef = useRef('');

  const addLog = useCallback((level: LogEntry['level'], source: string, message: string) => {
    setLogs(prev => [...prev.slice(-500), {
      id: ++logIdCounter,
      timestamp: new Date(),
      level,
      source,
      message,
    }]);
  }, []);

  const clearLogs = useCallback(() => setLogs([]), []);

  const disconnect = useCallback(() => {
    if (reconnectTimerRef.current) {
      clearTimeout(reconnectTimerRef.current);
      reconnectTimerRef.current = null;
    }
    if (wsRef.current) {
      wsRef.current.onclose = null; // prevent reconnect on intentional close
      wsRef.current.close();
      wsRef.current = null;
    }
    setStatus('disconnected');
    setTelemetry(null);
    setVideoFrame(null);
    setRpiStats(null);
    addLog('info', 'SYSTEM', 'Disconnected from server');
  }, [addLog]);

  const connect = useCallback((ip: string, port: string) => {
    // Clean up any existing connection
    if (wsRef.current) {
      wsRef.current.onclose = null;
      wsRef.current.close();
    }
    if (reconnectTimerRef.current) {
      clearTimeout(reconnectTimerRef.current);
      reconnectTimerRef.current = null;
    }

    ipRef.current = ip;
    portRef.current = port;

    setStatus('connecting');
    setConnectionError(null);
    connectTimeRef.current = Date.now();
    addLog('info', 'SYSTEM', `Connecting to ws://${ip}:${port}...`);

    try {
      const ws = new WebSocket(`ws://${ip}:${port}`);
      wsRef.current = ws;

      // Connection timeout
      const timeout = setTimeout(() => {
        if (ws.readyState !== WebSocket.OPEN) {
          ws.close();
          setStatus('error');
          setConnectionError('Connection timed out after 10s');
          addLog('error', 'SYSTEM', 'Connection timed out');
        }
      }, 10000);

      ws.onopen = () => {
        clearTimeout(timeout);
        setStatus('connected');
        setConnectionError(null);
        addLog('info', 'SYSTEM', `Connected to ws://${ip}:${port}`);
      };

      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);

          switch (data.type) {
            case 'telemetry_update': {
              // Bridge forwards Arduino JSON inside data field
              const t = data.data as ArduinoTelemetry;
              setTelemetry(t);
              setLastTelemetryTime(Date.now());
              break;
            }
            case 'video_frame': {
              setVideoFrame({
                frame: data.frame,
                detections: data.detections || [],
              });
              break;
            }
            case 'rpi_stats': {
              // If the bridge sends RPi system stats
              setRpiStats(data.data as RpiStats);
              break;
            }
            case 'log': {
              addLog(data.level || 'info', data.source || 'BRIDGE', data.message || '');
              break;
            }
            default: {
              // Raw Arduino telemetry (no wrapper)
              if (data.encL !== undefined || data.encR !== undefined) {
                setTelemetry(data as ArduinoTelemetry);
                setLastTelemetryTime(Date.now());
              }
              break;
            }
          }
        } catch (err) {
          // Non-JSON message — log it
          addLog('debug', 'SERIAL', String(event.data));
        }
      };

      ws.onerror = () => {
        clearTimeout(timeout);
        setConnectionError('WebSocket connection error');
        addLog('error', 'SYSTEM', 'WebSocket error');
      };

      ws.onclose = (event) => {
        clearTimeout(timeout);
        if (status === 'connected' || status === 'connecting') {
          setStatus('error');
          setConnectionError(`Connection closed: ${event.reason || 'Unknown reason'}`);
          addLog('warn', 'SYSTEM', 'Connection lost. Reconnecting in 3s...');

          // Auto-reconnect
          reconnectTimerRef.current = setTimeout(() => {
            if (ipRef.current && portRef.current) {
              connect(ipRef.current, portRef.current);
            }
          }, 3000);
        }
      };
    } catch (err) {
      setStatus('error');
      setConnectionError(err instanceof Error ? err.message : 'Failed to connect');
      addLog('error', 'SYSTEM', `Connection failed: ${err}`);
    }
  }, [addLog, status]);

  const sendCommand = useCallback((cmd: string) => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({ type: 'command', cmd }));
      addLog('debug', 'CMD', `Sent: ${cmd}`);
    } else {
      addLog('warn', 'CMD', `Cannot send "${cmd}" — not connected`);
    }
  }, [addLog]);

  const toggleAI = useCallback((enabled: boolean) => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({ type: 'toggle_ai', enabled }));
      addLog('info', 'AI', `AI detection ${enabled ? 'enabled' : 'disabled'}`);
    }
  }, [addLog]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (reconnectTimerRef.current) clearTimeout(reconnectTimerRef.current);
      if (wsRef.current) {
        wsRef.current.onclose = null;
        wsRef.current.close();
      }
    };
  }, []);

  // Uptime counter for RPi stats
  useEffect(() => {
    if (status !== 'connected') return;
    const interval = setInterval(() => {
      setRpiStats(prev => prev ? { ...prev, uptime: prev.uptime + 1 } : null);
    }, 1000);
    return () => clearInterval(interval);
  }, [status]);

  return (
    <WebSocketContext.Provider value={{
      status,
      connect,
      disconnect,
      connectionError,
      telemetry,
      lastTelemetryTime,
      videoFrame,
      sendCommand,
      toggleAI,
      logs,
      clearLogs,
      rpiStats,
    }}>
      {children}
    </WebSocketContext.Provider>
  );
}

export function useWebSocket() {
  const context = useContext(WebSocketContext);
  if (!context) {
    throw new Error('useWebSocket must be used within WebSocketProvider');
  }
  return context;
}
