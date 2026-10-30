import { useEffect, useState } from "react";
import { Bar, BarChart, CartesianGrid, ResponsiveContainer, Tooltip, XAxis, YAxis } from "recharts";
import CameraView from "../components/CameraView";
import { Card, COLOR_HEX, Pill, Unavailable, fmtTime } from "../components/ui";
import { useApiAction, usePoll } from "../hooks/useApiAction";
import { useAuth } from "../hooks/useAuth";
import { useRobot } from "../hooks/useRobot";
import { api } from "../services/api";

interface HistoryRow {
  id: number;
  object_id: number;
  detected_color: string;
  target_bin: number;
  sorting_status: string;
  attempts: number;
  message: string;
  timestamp: string;
}

const STATUS_TONE: Record<string, "ok" | "warn" | "bad"> = {
  success: "ok", unverified: "warn", failed_pickup: "bad", failed_verification: "bad",
};

export default function Sorting() {
  const { state, sorting } = useRobot();
  const { can } = useAuth();
  const { run, busy } = useApiAction();
  const s = sorting ?? state?.sorting;
  const { data: history, refresh } = usePoll<HistoryRow[]>("/api/sorting/history?limit=50", 3000);
  const [bins, setBins] = useState<Record<string, number>>({});
  const [uncl, setUncl] = useState(5);
  const [cap, setCap] = useState(10);
  const [newColor, setNewColor] = useState("");
  const op = can("operator");

  useEffect(() => {
    if (s && !Object.keys(bins).length) {
      setBins(s.bins);
      setUncl(s.unclassified_bin);
      setCap(s.bin_capacity);
    }
  }, [s, bins]);

  if (state && !state.capabilities.sorting) {
    return <Unavailable what="Sorting" reason="Sorting needs the servo controller (PCA9685) and a camera. One of them was not detected." />;
  }
  const binIds = Array.from(new Set([...Object.values(s?.bins ?? {}), s?.unclassified_bin ?? 5])).sort();
  const stats = s?.statistics ?? {};
  const chart = ["success", "unverified", "failed_pickup", "failed_verification"].map((k) => ({ name: k.replace("_", " "), n: stats[k] ?? 0 }));

  return (
    <div className="space-y-6">
      <div className="grid grid-cols-1 xl:grid-cols-3 gap-6">
        <Card title="Sorting control" actions={s && <Pill tone={s.running ? "info" : "muted"}>{s.state}</Pill>}>
          <div className="grid grid-cols-3 gap-2">
            <button className="btn-primary" disabled={!op || busy || (s?.running && s.state !== "paused")}
                    onClick={() => run(() => s?.state === "paused" ? api.post("/api/sorting/resume") : api.post("/api/sorting/start"), "Sorting started")}>
              {s?.state === "paused" ? "Resume" : "Start"}
            </button>
            <button className="btn-ghost" disabled={!op || busy || !s?.running}
                    onClick={() => run(() => api.post("/api/sorting/pause"), "Sorting paused")}>Pause</button>
            <button className="btn-ghost" disabled={!op || busy || !s?.running}
                    onClick={() => run(() => api.post("/api/sorting/stop"), "Sorting stopped")}>Stop</button>
          </div>
          <div className="text-sm text-slate-400 mt-3 min-h-[1.25rem]">{s?.message}</div>
          {s?.target && <div className="text-sm mt-1">Target: #{s.target.object_id} {s.target.color}</div>}
          <div className="kv mt-3"><span>Verification</span><span>{s?.verification === "bin_sensor" ? "bin break-beam sensor" : "NONE — results are unverified"}</span></div>
          <div className="kv"><span>Success rate (verified)</span><span>{s?.success_rate != null ? `${(s.success_rate * 100).toFixed(0)} %` : "—"}</span></div>
          <div className="kv"><span>Pickup attempts</span><span>{stats.attempts ?? 0}</span></div>
        </Card>
        <Card title="Camera"><CameraView /></Card>
        <Card title="Results this session">
          <ResponsiveContainer width="100%" height={200}>
            <BarChart data={chart} margin={{ left: -20 }}>
              <CartesianGrid stroke="#1e2632" strokeDasharray="3 3" />
              <XAxis dataKey="name" tick={{ fill: "#64748b", fontSize: 10 }} />
              <YAxis allowDecimals={false} tick={{ fill: "#64748b", fontSize: 11 }} />
              <Tooltip contentStyle={{ background: "#10151d", border: "1px solid #2a3442" }} />
              <Bar dataKey="n" fill="#22d3ee" isAnimationActive={false} />
            </BarChart>
          </ResponsiveContainer>
        </Card>
      </div>

      <div className="grid grid-cols-1 xl:grid-cols-3 gap-6">
        <Card title="Bin capacity" actions={op && <button className="btn-ghost" onClick={() => run(() => api.post("/api/sorting/bins/empty"), "Bin counters reset")}>Bins emptied</button>}>
          <div className="space-y-3">
            {binIds.map((b) => {
              const n = s?.bin_counts[String(b)] ?? 0;
              const pct = Math.min(100, (n / (s?.bin_capacity || 1)) * 100);
              const colors = Object.entries(s?.bins ?? {}).filter(([, v]) => v === b).map(([c]) => c);
              return (
                <div key={b}>
                  <div className="flex justify-between text-sm">
                    <span>Bin {b} <span className="text-slate-500">{b === s?.unclassified_bin ? "unclassified" : colors.join(", ")}</span></span>
                    <span className="font-mono">{n}/{s?.bin_capacity}</span>
                  </div>
                  <div className="h-2 bg-ink-700 rounded mt-1">
                    <div className={`h-2 rounded ${pct >= 100 ? "bg-bad" : pct > 75 ? "bg-warn" : "bg-accent"}`} style={{ width: `${pct}%` }} />
                  </div>
                </div>
              );
            })}
          </div>
        </Card>

        <Card title="Colour → bin mapping">
          {Object.entries(bins).map(([c, b]) => (
            <div key={c} className="flex items-center gap-2 mb-2">
              <span className="h-3 w-3 rounded-full" style={{ background: COLOR_HEX[c] ?? "#888" }} />
              <span className="w-20 text-sm">{c}</span>
              <input type="number" min={1} max={16} className="input w-20" value={b}
                     onChange={(e) => setBins({ ...bins, [c]: +e.target.value })} />
              <button className="text-xs text-slate-500 hover:text-bad" onClick={() => {
                const { [c]: _, ...rest } = bins;
                void _;
                setBins(rest);
              }}>remove</button>
            </div>
          ))}
          <div className="flex gap-2 mb-3">
            <input className="input" placeholder="add colour" value={newColor} onChange={(e) => setNewColor(e.target.value.toLowerCase())} />
            <button className="btn-ghost" disabled={!newColor} onClick={() => { setBins({ ...bins, [newColor]: uncl }); setNewColor(""); }}>Add</button>
          </div>
          <div className="grid grid-cols-2 gap-2">
            <label><span className="label">Unclassified bin</span>
              <input type="number" className="input" value={uncl} onChange={(e) => setUncl(+e.target.value)} /></label>
            <label><span className="label">Bin capacity</span>
              <input type="number" className="input" value={cap} onChange={(e) => setCap(+e.target.value)} /></label>
          </div>
          <div className="flex gap-2 mt-3">
            <button className="btn-primary" disabled={!op || busy || s?.running}
                    onClick={() => run(() => api.put("/api/sorting/config", { bins, unclassified_bin: uncl, bin_capacity: cap }), "Mapping applied")}>Apply</button>
            <button className="btn-ghost" disabled={!op || busy || s?.running}
                    onClick={() => run(() => api.put("/api/sorting/config", { bins, unclassified_bin: uncl, bin_capacity: cap, persist: true }), "Mapping saved")}>Apply & save</button>
          </div>
          {s?.running && <div className="text-xs text-slate-500 mt-2">Stop sorting to change the mapping.</div>}
        </Card>

        <ServoCalibration disabled={!op || !!s?.running} />
      </div>

      <Card title="Sorting history" actions={<button className="btn-ghost" onClick={refresh}>Refresh</button>}>
        <table className="w-full text-sm">
          <thead><tr className="table-head"><th>Time</th><th>Object</th><th>Colour</th><th>Bin</th><th>Status</th><th>Attempts</th><th>Detail</th></tr></thead>
          <tbody>
            {history?.map((h) => (
              <tr key={h.id} className="border-t border-ink-800">
                <td className="font-mono">{fmtTime(h.timestamp)}</td>
                <td className="font-mono">#{h.object_id}</td>
                <td>{h.detected_color}</td>
                <td>{h.target_bin}</td>
                <td><Pill tone={STATUS_TONE[h.sorting_status] ?? "muted"}>{h.sorting_status}</Pill></td>
                <td>{h.attempts}</td>
                <td className="text-slate-400">{h.message}</td>
              </tr>
            ))}
          </tbody>
        </table>
        {!history?.length && <div className="text-sm text-slate-500 py-4">No sorting operations recorded yet.</div>}
      </Card>
    </div>
  );
}

function ServoCalibration({ disabled }: { disabled: boolean }) {
  const { run } = useApiAction();
  const [angles, setAngles] = useState<Record<string, number>>({ base: 90, shoulder: 150, gripper: 30, sorter: 90 });
  return (
    <Card title="Servo calibration">
      {Object.entries(angles).map(([ch, a]) => (
        <label key={ch} className="block mb-3">
          <span className="label">{ch}: {a}°</span>
          <input type="range" min={0} max={180} value={a} disabled={disabled} className="w-full accent-cyan-400"
                 onChange={(e) => setAngles({ ...angles, [ch]: +e.target.value })}
                 onPointerUp={() => run(() => api.post("/api/sorting/servo", { channel: ch, angle: angles[ch] }))} />
        </label>
      ))}
      <p className="text-xs text-slate-500">Move each joint and note the angles for the poses in config/nexus.yaml (sorting.poses, bin_angles).</p>
    </Card>
  );
}
