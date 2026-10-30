import { Link } from "react-router-dom";
import CameraView from "../components/CameraView";
import { TimeSeries } from "../components/Charts";
import { Card, Dot, Pill, Stat, fmt, healthTone } from "../components/ui";
import { usePoll } from "../hooks/useApiAction";
import { useRobot } from "../hooks/useRobot";
import type { DeviceHealth } from "../services/types";

export default function Overview() {
  const { state, telemetry, history, connected, detections, sorting } = useRobot();
  const { data: health } = usePoll<{ overall: string; devices: DeviceHealth[] }>("/api/system/health", 3000);
  const safety = state?.safety;
  const pose = telemetry?.pose ?? state?.pose;
  const sort = sorting ?? state?.sorting;
  const batTone = telemetry?.battery_percent == null ? undefined : telemetry.battery_percent < 15 ? "bad" : telemetry.battery_percent < 35 ? "warn" : "ok";
  const tempTone = telemetry?.cpu_temperature == null ? undefined : telemetry.cpu_temperature > 80 ? "bad" : telemetry.cpu_temperature > 70 ? "warn" : "ok";

  return (
    <div className="space-y-6">
      <div className="grid grid-cols-2 md:grid-cols-4 xl:grid-cols-6 gap-4">
        <Stat label="Connection" value={connected ? "Online" : "Offline"} tone={connected ? "ok" : "bad"}
              sub={state ? `${state.hardware_mode} · up ${Math.round(state.uptime_s / 60)} min` : undefined} />
        <Stat label="Mode" value={state?.mode ?? null} sub={state?.navigation.state} />
        <Stat label="Battery" value={telemetry?.battery_percent ?? null} unit="%" tone={batTone}
              sub={telemetry?.battery_voltage != null ? `${fmt(telemetry.battery_voltage)} V` : "no battery monitor"} />
        <Stat label="CPU temp" value={telemetry?.cpu_temperature != null ? fmt(telemetry.cpu_temperature, 1) : null}
              unit="°C" tone={tempTone} sub={telemetry?.cpu_temperature == null ? "sensor not readable" : undefined} />
        <Stat label="CPU / RAM" value={telemetry?.cpu_percent != null ? `${fmt(telemetry.cpu_percent, 0)}/${fmt(telemetry.ram_percent, 0)}` : null} unit="%" />
        <Stat label="E-stop" value={safety?.estop_hardware || safety?.estop_software ? "ACTIVE" : "Released"}
              tone={safety?.estop_hardware || safety?.estop_software ? "bad" : "ok"}
              sub={safety?.estop_hardware ? "physical button pressed" : safety?.estop_software ? "software latch" : undefined} />
      </div>

      <div className="grid grid-cols-1 xl:grid-cols-3 gap-6">
        <Card title="Live camera" actions={<Link className="text-xs text-accent" to="/camera">open →</Link>} className="xl:col-span-1">
          {state?.capabilities.vision ? <CameraView /> : <div className="text-sm text-slate-500">No camera detected</div>}
          <div className="mt-3 flex flex-wrap gap-2">
            {detections.items.map((d) => (
              <Pill key={d.object_id ?? d.centroid.join()} tone="info">#{d.object_id} {d.color}</Pill>
            ))}
            {!detections.items.length && <span className="text-xs text-slate-500">no objects in view</span>}
          </div>
        </Card>

        <Card title="Position & navigation">
          <div className="kv"><span>X</span><span>{fmt(pose?.x)} m</span></div>
          <div className="kv"><span>Y</span><span>{fmt(pose?.y)} m</span></div>
          <div className="kv"><span>Heading</span><span>{pose ? fmt((pose.theta * 180) / Math.PI, 1) : "—"}°</span></div>
          <div className="kv"><span>Pose source</span><span>{pose?.source ?? "—"}</span></div>
          <div className="kv"><span>Pose valid / σ</span><span>{pose ? (pose.valid ? "yes" : "NO") : "—"} / {fmt(pose?.std_xy, 3)} m</span></div>
          <div className="kv"><span>Mission</span><span>{state?.mission.mission ? `${state.mission.mission} (${state.mission.state})` : "—"}</span></div>
          <div className="kv"><span>Navigation</span><span>{state?.navigation.state ?? "—"}</span></div>
          <div className="kv"><span>Distance to goal</span><span>{fmt(state?.navigation.distance_remaining)} m</span></div>
          <div className="kv"><span>Front obstacle</span><span>{fmt(safety?.obstacle_front_m)} m</span></div>
        </Card>

        <Card title="Vision & sorting">
          <div className="kv"><span>Objects in view</span><span>{detections.items.length}</span></div>
          <div className="kv"><span>Objects seen (session)</span><span>{Object.values(state?.vision.unique_objects ?? {}).reduce((a, b) => a + b, 0)}</span></div>
          <div className="kv"><span>Detection rate</span><span>{fmt(detections.stats?.fps, 1)} fps</span></div>
          <div className="kv"><span>Sorting state</span><span>{sort?.state ?? "—"}</span></div>
          <div className="kv"><span>Sorted (verified)</span><span>{sort?.statistics.success ?? 0}</span></div>
          <div className="kv"><span>Unverified / failed</span><span>{(sort?.statistics.unverified ?? 0)} / {(sort?.statistics.failed_pickup ?? 0) + (sort?.statistics.failed_verification ?? 0)}</span></div>
          <div className="kv"><span>Verification method</span><span>{sort?.verification ?? "—"}</span></div>
          <div className="kv"><span>Objects on the map</span><span>{state?.objects_mapped ?? 0}</span></div>
        </Card>
      </div>

      <div className="grid grid-cols-1 xl:grid-cols-3 gap-6">
        <Card title="Velocity (last 30 s)" className="xl:col-span-2">
          <TimeSeries data={history} series={[{ key: "v", label: "linear m/s" }, { key: "w", label: "angular rad/s" }]} />
        </Card>
        <Card title="Sensor health" actions={health && <Pill tone={health.overall === "ok" ? "ok" : health.overall === "degraded" ? "warn" : "bad"}>{health.overall}</Pill>}>
          <ul className="space-y-1.5">
            {health?.devices.map((d) => (
              <li key={d.name} className="flex items-center justify-between text-sm">
                <span className="flex items-center gap-2"><Dot tone={healthTone(d.state)} />{d.name}</span>
                <span className="text-xs text-slate-500">{d.state}{d.simulated ? " (sim)" : ""}</span>
              </li>
            ))}
          </ul>
          {safety && Object.keys(safety.faults).length > 0 && (
            <div className="mt-3 text-sm text-bad">
              {Object.entries(safety.faults).map(([k, v]) => <div key={k}>⚠ {v}</div>)}
            </div>
          )}
        </Card>
      </div>
    </div>
  );
}
