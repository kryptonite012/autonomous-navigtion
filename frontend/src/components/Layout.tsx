import { useEffect, useState } from "react";
import { NavLink, Outlet, useLocation } from "react-router-dom";
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
  const [menuOpen, setMenuOpen] = useState(false);
  const location = useLocation();
  useEffect(() => setMenuOpen(false), [location.pathname]);
  return (
    <div className="flex h-full">
      {menuOpen && <div className="fixed inset-0 z-30 bg-black/60 lg:hidden" onClick={() => setMenuOpen(false)} />}
      <aside className={`fixed inset-y-0 left-0 z-40 w-56 shrink-0 border-r border-ink-700 bg-ink-900 flex flex-col
                         transition-transform lg:static lg:translate-x-0 ${menuOpen ? "translate-x-0" : "-translate-x-full"}`}>
        <div className="px-5 py-4 border-b border-ink-700 flex items-center justify-between">
          <div>
            <div className="text-lg font-bold tracking-widest text-accent">NEXUS</div>
            <div className="text-[11px] text-slate-500">Autonomous Robot Control</div>
          </div>
          <button className="lg:hidden text-slate-400 text-xl" onClick={() => setMenuOpen(false)} aria-label="close menu">×</button>
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
        <header className="min-h-14 shrink-0 border-b border-ink-700 bg-ink-900/80 flex flex-wrap items-center gap-x-4 gap-y-2 px-3 py-2 sm:px-6">
          <button className="lg:hidden btn-ghost px-2" onClick={() => setMenuOpen(true)} aria-label="open menu">☰</button>
          <div className="flex items-center gap-2 text-sm">
            <Dot tone={connected ? "ok" : "bad"} pulse={!connected} />
            <span className="hidden sm:inline">{connected ? "Connected" : "Disconnected"}</span>
          </div>
          {state && (
            <>
              <Pill tone={state.simulated ? "info" : "ok"}>
                <span className="md:hidden">{state.simulated ? "SIM" : "HW"}</span>
                <span className="hidden md:inline">{state.simulated ? "SIMULATION" : "HARDWARE"}</span>
              </Pill>
              <span className="hidden sm:inline-flex"><Pill tone="muted">mode: {state.mode}</Pill></span>
              {s && !s.motion_allowed && <Pill tone="bad">MOTION DISABLED</Pill>}
              {s && s.zone !== "clear" && <Pill tone={s.zone === "stop" ? "bad" : "warn"}>obstacle {s.zone}</Pill>}
              {state.recording?.active && <Pill tone="bad">● REC</Pill>}
            </>
          )}
          <div className="ml-auto flex items-center gap-4 text-xs text-slate-400 font-mono">
            {telemetry?.battery_percent != null && <span>BAT {telemetry.battery_percent}%</span>}
            {latencyMs != null && <span className="hidden md:inline" title="age of the last telemetry message">lag {fmt(latencyMs, 0)} ms</span>}
            <EstopButton />
          </div>
        </header>
        <Alerts />
        <main className="flex-1 overflow-y-auto p-3 sm:p-6">
          <Outlet />
        </main>
      </div>
    </div>
  );
}
