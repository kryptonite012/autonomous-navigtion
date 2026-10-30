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
  const [clickMode, setClickMode] = useState<"goal" | "pose">("goal");
  const [poseHeading, setPoseHeading] = useState("0");
  const loc = state?.localization;
  const nav = navigation ?? state?.navigation;
  const op = can("operator");
  const motionOk = state?.safety.motion_allowed ?? false;

  const sendGoal = (x: number, y: number, theta?: number) =>
    run(() => api.post("/api/navigation/goal", { x, y, theta }), `Goal (${x.toFixed(2)}, ${y.toFixed(2)}) sent`);

  const pick = (x: number, y: number) => {
    x = +x.toFixed(2);
    y = +y.toFixed(2);
    if (clickMode === "pose") {
      const th = ((+poseHeading || 0) * Math.PI) / 180;
      run(() => api.post("/api/localization/initial-pose", { x, y, theta: th }), `Initial pose set to (${x}, ${y})`);
      setClickMode("goal");
    } else {
      sendGoal(x, y);
    }
  };

  return (
    <div className="grid grid-cols-1 xl:grid-cols-3 gap-6">
      <Card title={clickMode === "pose" ? "Map — click where the robot is" : "Map — click to set a destination"}
            className="xl:col-span-2"
            actions={<>
              {clickMode === "pose" && <Pill tone="warn">setting initial pose</Pill>}
              {nav && <Pill tone={STATE_TONE[nav.state] ?? "muted"}>{nav.state}</Pill>}
            </>}>
        <LiveMap height={520} onPick={op ? pick : undefined} />
        <div className="flex gap-4 text-xs text-slate-500 mt-2">
          <span className="text-accent">● estimated pose (ring = 2σ)</span>
          <span className="text-violet-400">— particles</span>
          <span>■ objects seen by the camera</span>
        </div>
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
        <Card title="Localisation" actions={loc && (
          <Pill tone={!loc.active ? "muted" : loc.searching ? "warn" : loc.converged ? "ok" : "bad"}>
            {!loc.active ? "odometry only" : loc.searching ? "searching" : loc.converged ? "localised" : "uncertain"}
          </Pill>)}>
          <div className="kv"><span>Method</span><span>{pose?.source ?? "—"}</span></div>
          <div className="kv"><span>Position σ</span><span>{fmt(pose?.std_xy, 3)} m</span></div>
          <div className="kv"><span>Heading σ</span><span>{pose?.std_theta != null ? fmt((pose.std_theta * 180) / Math.PI, 1) : "—"}°</span></div>
          {loc?.active && (
            <>
              <div className="kv"><span>Map</span><span>{loc.map}</span></div>
              <div className="kv"><span>Particles / Neff</span><span>{loc.particles} / {fmt(loc.neff, 0)}</span></div>
              <div className="kv"><span>Scan match</span><span>{fmt(loc.likelihood * 100, 0)} %</span></div>
            </>
          )}
          {!loc?.enabled && <p className="text-xs text-slate-500 mt-2">Map localisation needs a LiDAR (or localization.mode: on, experimental). Without it the pose is odometry only and drifts; set the pose again when it does.</p>}
          {loc?.experimental && <p className="text-xs text-warn mt-2">Experimental: localising with sparse range sensors.</p>}
          <div className="flex gap-2 mt-3 items-end">
            <label className="w-24"><span className="label">Heading °</span>
              <input className="input" value={poseHeading} onChange={(e) => setPoseHeading(e.target.value)} /></label>
            <button className={clickMode === "pose" ? "btn-warn" : "btn-ghost"} disabled={!op}
                    onClick={() => setClickMode(clickMode === "pose" ? "goal" : "pose")}>
              {clickMode === "pose" ? "Cancel" : "Set pose on map"}
            </button>
            <button className="btn-ghost" disabled={!op || busy || !loc?.active}
                    onClick={() => run(() => api.post("/api/localization/global"), "Global localisation started")}>Global</button>
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
