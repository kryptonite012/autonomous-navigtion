import { Card, Pill, fmt } from "../components/ui";
import { usePoll } from "../hooks/useApiAction";
import { useRobot } from "../hooks/useRobot";

interface Readings {
  timestamp: number;
  ranges: { sensor: string; distance_m: number | null; angle_rad: number; valid: boolean; max_range_m: number; age_s: number }[];
  imu: { accel: number[]; gyro: number[]; temperature: number | null } | null;
  battery_voltage: number | null;
  battery_percent: number | null;
  encoders: [number, number] | null;
  simulated: boolean;
}

/** Top-down view of the range sensor beams around the robot. */
function RangeRadar({ ranges, stop, slow }: { ranges: Readings["ranges"]; stop: number; slow: number }) {
  const S = 300, C = S / 2, scale = C / 2.0; // 2 m to the edge
  return (
    <svg viewBox={`0 0 ${S} ${S}`} className="w-full max-w-sm mx-auto">
      <circle cx={C} cy={C} r={slow * scale} fill="rgba(245,158,11,0.08)" stroke="#f59e0b" strokeDasharray="4 3" />
      <circle cx={C} cy={C} r={stop * scale} fill="rgba(239,68,68,0.12)" stroke="#ef4444" strokeDasharray="4 3" />
      {[0.5, 1, 1.5, 2].map((r) => <circle key={r} cx={C} cy={C} r={r * scale} fill="none" stroke="#1e2632" />)}
      {ranges.map((r) => {
        const d = r.distance_m ?? Math.min(r.max_range_m, 2);
        const a = -r.angle_rad - Math.PI / 2; // forward is up
        const half = r.sensor.startsWith("us_") ? 0.13 : 0.035;
        const p = (ang: number) => `${C + Math.cos(ang) * d * scale},${C + Math.sin(ang) * d * scale}`;
        const color = !r.valid ? "#64748b" : r.distance_m == null ? "#22d3ee" : r.distance_m < stop ? "#ef4444" : r.distance_m < slow ? "#f59e0b" : "#22d3ee";
        return (
          <g key={r.sensor}>
            <polygon points={`${C},${C} ${p(a - half)} ${p(a + half)}`} fill={color} fillOpacity={0.25} stroke={color} />
            <text x={C + Math.cos(a) * (d * scale + 12)} y={C + Math.sin(a) * (d * scale + 12)} fill="#94a3b8" fontSize="9" textAnchor="middle">
              {r.sensor}
            </text>
          </g>
        );
      })}
      <circle cx={C} cy={C} r={0.12 * scale} fill="#10151d" stroke="#22d3ee" strokeWidth={2} />
      <line x1={C} y1={C} x2={C} y2={C - 0.2 * scale} stroke="#22d3ee" strokeWidth={2} />
    </svg>
  );
}

export default function Sensors() {
  const { data } = usePoll<Readings>("/api/sensors/readings", 500);
  const { state } = useRobot();
  const safety = state?.safety;
  const cfgStop = 0.15, cfgSlow = 0.35;
  return (
    <div className="grid grid-cols-1 xl:grid-cols-3 gap-6">
      <Card title="Obstacle view" actions={data?.simulated && <Pill tone="info">simulated sensors</Pill>}>
        {data ? <RangeRadar ranges={data.ranges} stop={cfgStop} slow={cfgSlow} /> : null}
        <div className="flex justify-center gap-4 text-xs text-slate-500 mt-2">
          <span className="text-bad">— stop zone</span><span className="text-warn">— slow zone</span>
        </div>
        {safety && <div className="text-center mt-2"><Pill tone={safety.zone === "clear" ? "ok" : safety.zone === "slow" ? "warn" : "bad"}>zone: {safety.zone}</Pill></div>}
      </Card>
      <Card title="Distance sensors" className="xl:col-span-2">
        <table className="w-full text-sm">
          <thead><tr className="table-head"><th>Sensor</th><th>Angle</th><th>Distance</th><th>Max range</th><th>Age</th><th>Status</th></tr></thead>
          <tbody>
            {data?.ranges.map((r) => (
              <tr key={r.sensor} className="border-t border-ink-800 font-mono">
                <td className="font-sans py-1.5">{r.sensor}</td>
                <td>{fmt((r.angle_rad * 180) / Math.PI, 0)}°</td>
                <td>{r.distance_m == null ? "clear" : `${fmt(r.distance_m)} m`}</td>
                <td>{fmt(r.max_range_m, 1)} m</td>
                <td>{fmt(r.age_s * 1000, 0)} ms</td>
                <td className="font-sans">
                  {!r.valid ? <Pill tone="bad">no data</Pill> : safety?.stale_sensors.includes(r.sensor) ? <Pill tone="warn">stale</Pill> : <Pill tone="ok">ok</Pill>}
                </td>
              </tr>
            ))}
          </tbody>
        </table>
        <div className="grid grid-cols-1 md:grid-cols-3 gap-4 mt-6">
          <div>
            <div className="card-title mb-2">IMU</div>
            {data?.imu ? (
              <>
                <div className="kv"><span>ax ay az</span><span>{data.imu.accel.map((v) => fmt(v, 2)).join(" ")}</span></div>
                <div className="kv"><span>gx gy gz</span><span>{data.imu.gyro.map((v) => fmt(v, 3)).join(" ")}</span></div>
              </>
            ) : <div className="text-sm text-slate-500">not available</div>}
          </div>
          <div>
            <div className="card-title mb-2">Encoders</div>
            {data?.encoders ? (
              <>
                <div className="kv"><span>Left ticks</span><span>{data.encoders[0]}</span></div>
                <div className="kv"><span>Right ticks</span><span>{data.encoders[1]}</span></div>
              </>
            ) : <div className="text-sm text-slate-500">not available</div>}
          </div>
          <div>
            <div className="card-title mb-2">Battery</div>
            <div className="kv"><span>Voltage</span><span>{fmt(data?.battery_voltage)} V</span></div>
            <div className="kv"><span>Charge (est.)</span><span>{data?.battery_percent ?? "—"} %</span></div>
            {safety?.warnings.battery && <div className="text-xs text-warn mt-1">{safety.warnings.battery}</div>}
          </div>
        </div>
      </Card>
    </div>
  );
}
