// Live robot data over a single WebSocket, shared through context.

import { createContext, ReactNode, useContext, useEffect, useRef, useState } from "react";
import { getToken } from "../services/api";
import type {
  Detection,
  MapSnapshot,
  NavStatus,
  Pose,
  RobotEvent,
  RobotState,
  SortStatus,
  Telemetry,
  VisionStatus,
} from "../services/types";

export interface TelemetryPoint {
  t: number;
  v: number;
  w: number;
  battery: number | null;
  cpuTemp: number | null;
  cpu: number | null;
  [sensor: string]: number | null;
}

interface RobotCtx {
  connected: boolean;
  latencyMs: number | null;
  state: RobotState | null;
  telemetry: Telemetry | null;
  history: TelemetryPoint[];
  pose: Pose | null;
  detections: { items: Detection[]; stats: VisionStatus | null };
  navigation: NavStatus | null;
  sorting: SortStatus | null;
  map: MapSnapshot | null;
  events: RobotEvent[];
  alerts: RobotEvent[];
  dismissAlert: (ts: number) => void;
}

const Ctx = createContext<RobotCtx | null>(null);
const HISTORY = 300; // ~30 s at 10 Hz

export function RobotProvider({ children }: { children: ReactNode }) {
  const [connected, setConnected] = useState(false);
  const [latencyMs, setLatency] = useState<number | null>(null);
  const [state, setState] = useState<RobotState | null>(null);
  const [telemetry, setTelemetry] = useState<Telemetry | null>(null);
  const [history, setHistory] = useState<TelemetryPoint[]>([]);
  const [pose, setPose] = useState<Pose | null>(null);
  const [detections, setDetections] = useState<{ items: Detection[]; stats: VisionStatus | null }>({
    items: [],
    stats: null,
  });
  const [navigation, setNavigation] = useState<NavStatus | null>(null);
  const [sorting, setSorting] = useState<SortStatus | null>(null);
  const [map, setMap] = useState<MapSnapshot | null>(null);
  const [events, setEvents] = useState<RobotEvent[]>([]);
  const [alerts, setAlerts] = useState<RobotEvent[]>([]);
  const retry = useRef(0);

  useEffect(() => {
    let ws: WebSocket | null = null;
    let closed = false;
    let timer: number | undefined;

    const connect = () => {
      const token = getToken();
      if (!token) return;
      const proto = location.protocol === "https:" ? "wss" : "ws";
      ws = new WebSocket(`${proto}://${location.host}/ws?token=${encodeURIComponent(token)}`);
      ws.onopen = () => {
        setConnected(true);
        retry.current = 0;
      };
      ws.onclose = () => {
        setConnected(false);
        if (!closed) {
          const delay = Math.min(10000, 500 * 2 ** retry.current++);
          timer = window.setTimeout(connect, delay);
        }
      };
      ws.onmessage = (msg) => {
        const { topic, data, ts } = JSON.parse(msg.data);
        switch (topic) {
          case "hello":
            setState(data.state);
            break;
          case "state":
            setState(data);
            break;
          case "telemetry": {
            const tel = data as Telemetry;
            setTelemetry(tel);
            if (ts) setLatency(Math.max(0, Date.now() - ts * 1000));
            const p: TelemetryPoint = {
              t: tel.timestamp,
              v: tel.linear_velocity,
              w: tel.angular_velocity,
              battery: tel.battery_voltage,
              cpuTemp: tel.cpu_temperature,
              cpu: tel.cpu_percent,
            };
            for (const [k, d] of Object.entries(tel.ranges)) p[k] = d;
            setHistory((h) => (h.length >= HISTORY ? [...h.slice(h.length - HISTORY + 1), p] : [...h, p]));
            break;
          }
          case "pose":
            setPose(data);
            break;
          case "detections":
            setDetections({ items: data.items, stats: data.stats });
            break;
          case "navigation":
            setNavigation(data);
            break;
          case "sorting":
            setSorting(data.status);
            break;
          case "map":
            setMap(data);
            break;
          case "events":
            setEvents((e) => [data, ...e].slice(0, 200));
            break;
          case "alerts":
            setAlerts((a) => [data, ...a].slice(0, 5));
            break;
        }
      };
    };
    connect();
    return () => {
      closed = true;
      window.clearTimeout(timer);
      ws?.close();
    };
  }, []);

  const dismissAlert = (ts: number) => setAlerts((a) => a.filter((x) => x.timestamp !== ts));

  return (
    <Ctx.Provider
      value={{
        connected,
        latencyMs,
        state,
        telemetry,
        history,
        pose,
        detections,
        navigation,
        sorting,
        map,
        events,
        alerts,
        dismissAlert,
      }}
    >
      {children}
    </Ctx.Provider>
  );
}

export function useRobot(): RobotCtx {
  const c = useContext(Ctx);
  if (!c) throw new Error("useRobot outside RobotProvider");
  return c;
}
