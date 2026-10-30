import { NavLink, Outlet } from "react-router-dom";
import { useAuth } from "../hooks/useAuth";
import { useApiAction } from "../hooks/useApiAction";
import { useRobot } from "../hooks/useRobot";
import { api } from "../services/api";
import { Dot, Pill, fmt } from "./ui";

const NAV = [
  ["/", "Overview"],
  ["/camera", "Live Camera"],
  ["/mapping", "Mapping"],
  ["/navigation", "Navigation"],
  ["/missions", "Missions"],
  ["/detection", "Object Detection"],
  ["/sorting", "Color Sorting"],
  ["/telemetry", "Telemetry"],
  ["/hardware", "Hardware"],
  ["/sensors", "Sensors"],
  ["/maps", "Map Management"],
  ["/logs", "Event Logs"],
  ["/settings", "Settings"],
  ["/simulation", "Simulation"],
] as const;

function EstopButton() {
  const { state } = useRobot();
  const { run } = useApiAction();
  const latched = state?.safety.estop_software || state?.safety.estop_hardware;
  return (
    <div className="flex items-center gap-2">
      <button
        className="btn-danger px-5 py-2 text-base font-bold tracking-wide shadow-[0_0_0_2px_rgba(239,68,68,0.35)]"
        onClick={() => run(() => api.post("/api/robot/emergency-stop"), "Emergency stop engaged")}
        title="Stops all motors immediately. The physical e-stop button works without this dashboard."
      >
        E-STOP
      </button>
      {latched && (
        <button
          className="btn-ghost"
          onClick={() => run(() => api.post("/api/robot/emergency-stop/reset"), "Emergency stop reset")}
        >
          Reset
        </button>
      )}
    </div>
  );
}

function Alerts() {
  const { alerts, dismissAlert } = useRobot();
  if (!alerts.length) return null;
  return (
    <div className="space-y-1 px-6 pt-3">
      {alerts.map((a) => (
        <div
          key={a.timestamp + a.type}
          className={`flex items-center justify-between rounded-md border px-3 py-1.5 text-sm ${
            a.severity === "warning" ? "border-warn/40 bg-warn/10 text-warn" : "border-bad/40 bg-bad/10 text-bad"
          }`}
        >
          <span>
            <b className="uppercase text-xs mr-2">{a.severity}</b>
            {a.description}
          </span>
          <button className="text-xs opacity-70 hover:opacity-100" onClick={() => dismissAlert(a.timestamp)}>
            dismiss
          </button>
        </div>
      ))}
    </div>
  );
}

export default function Layout() {
  const { user, logout } = useAuth();
  const { connected, state, telemetry, latencyMs } = useRobot();
  const s = state?.safety;
  return (
    <div className="flex h-full">
      <aside className="w-56 shrink-0 border-r border-ink-700 bg-ink-900 flex flex-col">
        <div className="px-5 py-4 border-b border-ink-700">
          <div className="text-lg font-bold tracking-widest text-accent">NEXUS</div>
          <div className="text-[11px] text-slate-500">Autonomous Robot Control</div>
        </div>
        <nav className="flex-1 overflow-y-auto py-2">
          {NAV.filter(([to]) => to !== "/simulation" || state?.simulated).map(([to, label]) => (
            <NavLink
              key={to}
              to={to}
              end={to === "/"}
              className={({ isActive }) =>
                `block px-5 py-2 text-sm border-l-2 ${
                  isActive ? "border-accent bg-ink-800 text-white" : "border-transparent text-slate-400 hover:text-slate-200 hover:bg-ink-850"
                }`
              }
            >
              {label}
            </NavLink>
          ))}
        </nav>
        <div className="px-5 py-3 border-t border-ink-700 text-xs text-slate-500">
          <div>{user?.username} · {user?.role}</div>
          <button className="text-accent hover:underline mt-1" onClick={logout}>Sign out</button>
        </div>
      </aside>
      <div className="flex-1 flex flex-col min-w-0">
        <header className="h-14 shrink-0 border-b border-ink-700 bg-ink-900/80 flex items-center gap-4 px-6">
          <div className="flex items-center gap-2 text-sm">
            <Dot tone={connected ? "ok" : "bad"} pulse={!connected} />
            {connected ? "Connected" : "Disconnected"}
          </div>
          {state && (
            <>
              <Pill tone={state.simulated ? "info" : "ok"}>{state.simulated ? "SIMULATION" : "HARDWARE"}</Pill>
              <Pill tone="muted">mode: {state.mode}</Pill>
              {s && !s.motion_allowed && <Pill tone="bad">MOTION DISABLED</Pill>}
              {s && s.zone !== "clear" && <Pill tone={s.zone === "stop" ? "bad" : "warn"}>obstacle {s.zone}</Pill>}
            </>
          )}
          <div className="ml-auto flex items-center gap-4 text-xs text-slate-400 font-mono">
            {telemetry?.battery_percent != null && <span>BAT {telemetry.battery_percent}%</span>}
            {latencyMs != null && <span title="age of the last telemetry message">lag {fmt(latencyMs, 0)} ms</span>}
            <EstopButton />
          </div>
        </header>
        <Alerts />
        <main className="flex-1 overflow-y-auto p-6">
          <Outlet />
        </main>
      </div>
    </div>
  );
}
