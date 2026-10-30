import { useState } from "react";
import type { Circle } from "../components/MapCanvas";
import { Card, COLOR_HEX, Pill, Stat, Unavailable, fmt } from "../components/ui";
import LiveMap from "../dashboards/LiveMap";
import { useApiAction, usePoll } from "../hooks/useApiAction";
import { useAuth } from "../hooks/useAuth";
import { useRobot } from "../hooks/useRobot";
import { api } from "../services/api";
import type { SimState } from "../services/types";

type ClickMode = "none" | "obstacle" | "moving" | "teleport";

/** Simulation-only control room: inject faults and set up test scenarios. */
export default function Simulation() {
  const { state } = useRobot();
  const { can } = useAuth();
  const { run, busy } = useApiAction();
  const { data: sim, refresh } = usePoll<SimState>(state?.simulated ? "/api/simulation/state" : null, 1000);
  const [mode, setMode] = useState<ClickMode>("none");
  const [heading, setHeading] = useState("0");
  const [count, setCount] = useState(6);
  const [mapName, setMapName] = useState("sim_reference");
  const op = can("operator");

  if (state && !state.simulated) {
    return <Unavailable what="Simulation control" reason="The robot is running in hardware mode. Fault injection is only possible in simulation." />;
  }

  const fault = (body: Record<string, unknown>, msg: string) => run(() => api.post("/api/simulation/faults", body), msg).then(refresh);

  const pick = (x: number, y: number) => {
    x = +x.toFixed(2);
    y = +y.toFixed(2);
    if (mode === "obstacle") run(() => api.post("/api/simulation/obstacles", { x, y, radius: 0.1 }), "Obstacle placed").then(refresh);
    if (mode === "moving") run(() => api.post("/api/simulation/obstacles", { x, y, radius: 0.1, vx: 0.15, vy: 0.08 }), "Moving obstacle placed").then(refresh);
    if (mode === "teleport") {
      run(() => api.post("/api/simulation/teleport", { x, y, theta: ((+heading || 0) * Math.PI) / 180 }), "Robot kidnapped").then(refresh);
      setMode("none");
    }
  };

  const circles: Circle[] = [
    ...(sim?.dynamic ?? []).map(([x, y, r]) => ({ x, y, r, color: "rgba(249,115,22,0.85)" })),
    ...(sim?.objects ?? []).filter((o) => !o.held && o.sorted_bin == null)
      .map((o) => ({ x: o.x, y: o.y, r: 0.03, color: COLOR_HEX[o.color] ?? "#fff" })),
  ];
  const disc = new Set(sim?.faults.disconnected_sensors ?? []);
  const toggleSensor = (name: string) => {
    const next = new Set(disc);
    if (next.has(name)) next.delete(name);
    else next.add(name);
    fault({ disconnected_sensors: [...next] }, `${name} ${next.has(name) ? "disconnected" : "reconnected"}`);
  };
  const toggles: [keyof SimState["faults"], string][] = [
    ["motor_fault", "Motor driver failure"],
    ["encoder_fault", "Encoder signal lost"],
    ["servo_fault", "Servo controller failure"],
    ["camera_covered", "Camera covered"],
    ["estop_pressed", "Physical e-stop pressed"],
  ];

  return (
    <div className="space-y-6">
      <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
        <Stat label="Pose error (estimate vs truth)" value={sim ? fmt(sim.pose_error_m, 3) : null} unit="m"
              tone={sim ? (sim.pose_error_m > 0.3 ? "bad" : sim.pose_error_m > 0.1 ? "warn" : "ok") : undefined}
              sub={sim?.estimate.source} />
        <Stat label="Collisions (sim steps)" value={sim?.collisions ?? null} tone={sim && sim.collisions > 0 ? "warn" : "ok"} />
        <Stat label="Battery (simulated)" value={sim ? fmt(sim.battery_v, 2) : null} unit="V" />
        <Stat label="Objects on floor" value={sim ? sim.objects.filter((o) => o.sorted_bin == null && !o.held).length : null}
              sub={sim ? `${sim.objects.filter((o) => o.sorted_bin != null).length} sorted` : undefined} />
      </div>

      <div className="grid grid-cols-1 xl:grid-cols-3 gap-6">
        <Card title="World" className="xl:col-span-2" actions={<>
          {mode !== "none" && <Pill tone="warn">click map: {mode}</Pill>}
          <Pill tone="info">green dashed = true pose</Pill>
        </>}>
          <LiveMap height={520} onPick={op && mode !== "none" ? pick : undefined} circles={circles} layers={{ groundTruth: true }} />
          <div className="flex flex-wrap gap-2 mt-3">
            <button className={mode === "obstacle" ? "btn-warn" : "btn-ghost"} disabled={!op} onClick={() => setMode(mode === "obstacle" ? "none" : "obstacle")}>Place obstacle</button>
            <button className={mode === "moving" ? "btn-warn" : "btn-ghost"} disabled={!op} onClick={() => setMode(mode === "moving" ? "none" : "moving")}>Place moving obstacle</button>
            <button className="btn-ghost" disabled={!op || busy} onClick={() => run(() => api.del("/api/simulation/obstacles"), "Obstacles removed").then(refresh)}>Clear obstacles</button>
            <span className="mx-2 border-l border-ink-600" />
            <input className="input w-20" value={heading} onChange={(e) => setHeading(e.target.value)} title="heading after kidnap (deg)" />
            <button className={mode === "teleport" ? "btn-warn" : "btn-ghost"} disabled={!op} onClick={() => setMode(mode === "teleport" ? "none" : "teleport")}>Kidnap robot</button>
          </div>
          <p className="text-xs text-slate-500 mt-2">
            Kidnapping moves the robot without telling its odometry. With a LiDAR, use <i>Navigation → Global</i> to re-localise;
            without one, set the pose on the map.
          </p>
        </Card>

        <div className="space-y-6">
          <Card title="Fault injection">
            {toggles.map(([k, labelText]) => (
              <label key={k} className="flex items-center justify-between py-1.5 border-b border-ink-800 text-sm">
                <span>{labelText}</span>
                <input type="checkbox" disabled={!op || busy} checked={!!sim?.faults[k]}
                       onChange={(e) => fault({ [k]: e.target.checked }, `${labelText}: ${e.target.checked ? "on" : "off"}`)} />
              </label>
            ))}
            <div className="label mt-3">Disconnect range sensors</div>
            <div className="flex flex-wrap gap-2">
              {sim?.sensors.map((n) => (
                <button key={n} disabled={!op || busy} onClick={() => toggleSensor(n)}
                        className={`btn border ${disc.has(n) ? "border-bad text-bad" : "border-ink-600"}`}>{n}</button>
              ))}
            </div>
            <label className="block mt-3">
              <span className="label">Battery voltage: {fmt(sim?.battery_v, 2)} V (return home &lt; 7.0, cut-off &lt; 6.6)</span>
              <input type="range" min={6.0} max={8.4} step={0.05} defaultValue={8.2} disabled={!op} className="w-full accent-cyan-400"
                     onPointerUp={(e) => fault({ battery_voltage: +(e.target as HTMLInputElement).value }, "Battery voltage set")} />
            </label>
          </Card>

          <Card title="Scenario">
            <div className="flex gap-2 items-end">
              <label className="w-24"><span className="label">Objects</span>
                <input type="number" min={1} max={30} className="input" value={count} onChange={(e) => setCount(+e.target.value)} /></label>
              <button className="btn-ghost flex-1" disabled={!op || busy} onClick={() => run(() => api.post("/api/simulation/objects/scatter", { count }), "Objects scattered").then(refresh)}>Scatter objects</button>
            </div>
            <div className="flex gap-2 items-end mt-3">
              <label className="flex-1"><span className="label">Reference map name</span>
                <input className="input" value={mapName} onChange={(e) => setMapName(e.target.value)} /></label>
              <button className="btn-ghost" disabled={!op || busy || !mapName}
                      onClick={() => run(() => api.post("/api/simulation/reference-map", { name: mapName }), "Ground-truth map saved — load it on the Mapping screen")}>Save</button>
            </div>
            <p className="text-xs text-slate-500 mt-1">Ground-truth map of the simulated room, for testing navigation and missions without exploring first.</p>
            <button className="btn-warn w-full mt-3" disabled={!op || busy}
                    onClick={() => confirm("Reset the simulated world, clear faults and stop all tasks?") && run(() => api.post("/api/simulation/reset"), "Simulation reset").then(refresh)}>Reset world & faults</button>
          </Card>
        </div>
      </div>
    </div>
  );
}
