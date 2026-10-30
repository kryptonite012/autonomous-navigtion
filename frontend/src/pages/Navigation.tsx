import { useState } from "react";
import Teleop from "../components/Teleop";
import { Card, Pill, fmt } from "../components/ui";
import LiveMap from "../dashboards/LiveMap";
import { useApiAction } from "../hooks/useApiAction";
import { useAuth } from "../hooks/useAuth";
import { useRobot } from "../hooks/useRobot";
import { api } from "../services/api";

const STATE_TONE: Record<string, "ok" | "warn" | "bad" | "info" | "muted"> = {
  navigating: "info", exploring: "info", reached: "ok", failed: "bad", paused: "warn",
  recovering: "warn", waiting_localization: "bad", idle: "muted",
};

export default function Navigation() {
  const { state, navigation, pose } = useRobot();
  const { can } = useAuth();
  const { run, busy } = useApiAction();
  const [gx, setGx] = useState("");
  const [gy, setGy] = useState("");
  const [gth, setGth] = useState("");
  const [locName, setLocName] = useState("");
  const nav = navigation ?? state?.navigation;
  const op = can("operator");
  const motionOk = state?.safety.motion_allowed ?? false;

  const sendGoal = (x: number, y: number, theta?: number) =>
    run(() => api.post("/api/navigation/goal", { x, y, theta }), `Goal (${x.toFixed(2)}, ${y.toFixed(2)}) sent`);

  return (
    <div className="grid grid-cols-1 xl:grid-cols-3 gap-6">
      <Card title="Map — click to set a destination" className="xl:col-span-2"
            actions={nav && <Pill tone={STATE_TONE[nav.state] ?? "muted"}>{nav.state}</Pill>}>
        <LiveMap height={520} onPick={op ? (x, y) => sendGoal(+x.toFixed(2), +y.toFixed(2)) : undefined} />
        {nav?.message && <div className="text-sm text-slate-400 mt-2">{nav.message}</div>}
      </Card>
      <div className="space-y-6">
        <Card title="Navigation">
          <div className="kv"><span>Current</span><span>({fmt(pose?.x)}, {fmt(pose?.y)}) {pose ? fmt((pose.theta * 180) / Math.PI, 0) : "—"}°</span></div>
          <div className="kv"><span>Target</span><span>{nav?.goal ? `(${fmt(nav.goal.x)}, ${fmt(nav.goal.y)})${nav.goal.name ? " " + nav.goal.name : ""}` : "—"}</span></div>
          <div className="kv"><span>Distance</span><span>{fmt(nav?.distance_remaining)} m</span></div>
          <div className="kv"><span>Planner</span><span>{nav?.planner ?? "—"}</span></div>
          <div className="kv"><span>Path points</span><span>{nav?.path.length ?? 0}</span></div>
          <div className="kv"><span>Recoveries</span><span>{nav?.recoveries ?? 0}</span></div>
          <div className="grid grid-cols-2 gap-2 mt-3">
            <button className="btn-primary" disabled={!op || busy || !motionOk}
                    onClick={() => run(() => api.post("/api/navigation/start"), "Exploration started")}>Explore</button>
            <button className="btn-ghost" disabled={!op || busy || nav?.state === "idle"}
                    onClick={() => run(() => api.post("/api/navigation/cancel"), "Navigation stopped")}>Stop</button>
            <button className="btn-ghost" disabled={!op || busy}
                    onClick={() => run(() => api.post("/api/navigation/pause"), "Paused")}>Pause</button>
            <button className="btn-ghost" disabled={!op || busy || nav?.state !== "paused"}
                    onClick={() => run(() => api.post("/api/navigation/resume"), "Resumed")}>Resume</button>
          </div>
        </Card>
        <Card title="Destination">
          <div className="grid grid-cols-3 gap-2">
            <input className="input" placeholder="x (m)" value={gx} onChange={(e) => setGx(e.target.value)} />
            <input className="input" placeholder="y (m)" value={gy} onChange={(e) => setGy(e.target.value)} />
            <input className="input" placeholder="θ (deg)" value={gth} onChange={(e) => setGth(e.target.value)} />
          </div>
          <button className="btn-primary w-full mt-2" disabled={!op || busy || gx === "" || gy === "" || isNaN(+gx) || isNaN(+gy)}
                  onClick={() => sendGoal(+gx, +gy, gth === "" ? undefined : (+gth * Math.PI) / 180)}>Go</button>
          <div className="label mt-4">Named locations</div>
          <div className="flex flex-wrap gap-2">
            {Object.keys(nav?.named_locations ?? {}).map((n) => (
              <button key={n} className="btn-ghost" disabled={!op || busy}
                      onClick={() => run(() => api.post("/api/navigation/goal", { location: n }), `Going to ${n}`)}>{n}</button>
            ))}
          </div>
          <div className="flex gap-2 mt-3">
            <input className="input" placeholder="save current pose as…" value={locName} onChange={(e) => setLocName(e.target.value)} />
            <button className="btn-ghost" disabled={!op || !locName || !pose}
                    onClick={() => pose && run(() => api.post("/api/navigation/locations", { name: locName, x: pose.x, y: pose.y, theta: pose.theta }), "Location saved")}>Save</button>
          </div>
        </Card>
        <Card title="Manual control">
          {state?.capabilities.drive ? <Teleop disabled={!op || !motionOk} /> : <div className="text-sm text-slate-500">Drive hardware unavailable.</div>}
          {!motionOk && <div className="text-xs text-bad mt-2">Motion disabled: {state?.safety.estop_software || state?.safety.estop_hardware ? "emergency stop active" : Object.values(state?.safety.faults ?? {}).join("; ")}</div>}
        </Card>
      </div>
    </div>
  );
}
