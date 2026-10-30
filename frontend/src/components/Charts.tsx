import { CartesianGrid, Legend, Line, LineChart, ResponsiveContainer, Tooltip, XAxis, YAxis } from "recharts";
import type { TelemetryPoint } from "../hooks/useRobot";

const PALETTE = ["#22d3ee", "#f59e0b", "#a78bfa", "#22c55e", "#f472b6", "#94a3b8"];

export function TimeSeries({ data, series, unit, height = 200, domain }: {
  data: TelemetryPoint[];
  series: { key: string; label?: string }[];
  unit?: string;
  height?: number;
  domain?: [number | "auto", number | "auto"];
}) {
  const t0 = data.length ? data[data.length - 1].t : 0;
  const rows = data.map((p) => ({ ...p, rel: +(p.t - t0).toFixed(1) }));
  return (
    <ResponsiveContainer width="100%" height={height}>
      <LineChart data={rows} margin={{ top: 4, right: 8, bottom: 0, left: -12 }}>
        <CartesianGrid stroke="#1e2632" strokeDasharray="3 3" />
        <XAxis dataKey="rel" type="number" domain={["dataMin", 0]} tick={{ fill: "#64748b", fontSize: 11 }}
               tickFormatter={(v) => `${v}s`} />
        <YAxis tick={{ fill: "#64748b", fontSize: 11 }} domain={domain ?? ["auto", "auto"]} unit={unit} />
        <Tooltip contentStyle={{ background: "#10151d", border: "1px solid #2a3442", fontSize: 12 }}
                 labelFormatter={(v) => `${v}s`} formatter={(v: number) => (v == null ? "—" : v.toFixed(3))} />
        {series.length > 1 && <Legend wrapperStyle={{ fontSize: 12 }} />}
        {series.map((s, i) => (
          <Line key={s.key} dataKey={s.key} name={s.label ?? s.key} stroke={PALETTE[i % PALETTE.length]}
                dot={false} strokeWidth={1.6} isAnimationActive={false} connectNulls={false} />
        ))}
      </LineChart>
    </ResponsiveContainer>
  );
}
