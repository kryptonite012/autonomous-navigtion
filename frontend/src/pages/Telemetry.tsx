import { TimeSeries } from "../components/Charts";
import { Card, Pill, Stat, fmt } from "../components/ui";
import { useRobot } from "../hooks/useRobot";

export default function Telemetry() {
  const { telemetry: t, history, latencyMs, state } = useRobot();
  const sensors = Object.keys(t?.ranges ?? {});
  const motorTone = t?.motor_status === "ok" ? "ok" : t?.motor_status === "fault" ? "bad" : "warn";
  return (
    <div className="space-y-6">
      <div className="grid grid-cols-2 md:grid-cols-4 xl:grid-cols-7 gap-4">
        <Stat label="Linear vel." value={fmt(t?.linear_velocity, 3)} unit="m/s" />
        <Stat label="Angular vel." value={fmt(t?.angular_velocity, 3)} unit="rad/s" />
        <Stat label="Battery" value={t?.battery_voltage != null ? fmt(t.battery_voltage) : null} unit="V" />
        <Stat label="Motors" value={t?.motor_status ?? null} tone={motorTone} />
        <Stat label="CPU temp" value={t?.cpu_temperature != null ? fmt(t.cpu_temperature, 1) : null} unit="°C" />
        <Stat label="Control loop" value={fmt(t?.loop_hz, 1)} unit="Hz" sub={`target ${state ? "" : ""}20 Hz`} />
        <Stat label="Msg latency" value={latencyMs != null ? fmt(latencyMs, 0) : null} unit="ms"
              sub="includes clock offset between robot and browser" />
      </div>

      {state && Object.keys(state.safety.faults).length > 0 && (
        <div className="card border-bad/50 p-4 text-bad text-sm">
          {Object.entries(state.safety.faults).map(([k, v]) => <div key={k}><b>{k}</b>: {v}</div>)}
        </div>
      )}

      <div className="grid grid-cols-1 xl:grid-cols-2 gap-6">
        <Card title="Velocity">
          <TimeSeries data={history} series={[{ key: "v", label: "linear (m/s)" }, { key: "w", label: "angular (rad/s)" }]} />
        </Card>
        <Card title="Distance sensors (m)">
          <TimeSeries data={history} series={sensors.map((s) => ({ key: s }))} domain={[0, 2.5]} />
        </Card>
        <Card title="Battery voltage (V)">
          <TimeSeries data={history} series={[{ key: "battery", label: "battery" }]} />
        </Card>
        <Card title="CPU temperature (°C) / load (%)">
          <TimeSeries data={history} series={[{ key: "cpuTemp", label: "temp" }, { key: "cpu", label: "load" }]} />
        </Card>
      </div>

      <div className="grid grid-cols-1 xl:grid-cols-3 gap-6">
        <Card title="IMU">
          {t?.imu ? (
            <>
              <div className="kv"><span>Accel (m/s²)</span><span>{t.imu.accel.map((a) => fmt(a, 2)).join("  ")}</span></div>
              <div className="kv"><span>Gyro (rad/s)</span><span>{t.imu.gyro.map((g) => fmt(g, 3)).join("  ")}</span></div>
              <div className="kv"><span>Temperature</span><span>{fmt(t.imu.temperature, 1)} °C</span></div>
            </>
          ) : <div className="text-sm text-slate-500">No IMU data</div>}
        </Card>
        <Card title="Commanded vs applied">
          <div className="kv"><span>Commanded (v, ω)</span><span>{state?.safety.commanded.map((x) => fmt(x, 3)).join(", ")}</span></div>
          <div className="kv"><span>Applied (v, ω)</span><span>{state?.safety.applied.map((x) => fmt(x, 3)).join(", ")}</span></div>
          <div className="kv"><span>Source</span><span>{state?.safety.last_command_source || "—"}</span></div>
          <div className="kv"><span>Limited by</span><span>{state?.safety.limited_reason || "—"}</span></div>
        </Card>
        <Card title="Distance readings">
          {sensors.map((s) => (
            <div className="kv" key={s}>
              <span>{s}</span>
              <span>
                {t?.ranges_valid[s] === false ? <Pill tone="bad">invalid</Pill> : t?.ranges[s] == null ? "out of range" : `${fmt(t.ranges[s])} m`}
              </span>
            </div>
          ))}
        </Card>
      </div>
    </div>
  );
}
