import { useState } from "react";
import { Card, Pill, Unavailable, fmt } from "../components/ui";
import LiveMap from "../dashboards/LiveMap";
import { useApiAction, usePoll } from "../hooks/useApiAction";
import { useAuth } from "../hooks/useAuth";
import { useRobot } from "../hooks/useRobot";
import { api } from "../services/api";
import type { MappingStatus } from "../services/types";

export default function Mapping() {
  const { state } = useRobot();
  const { can } = useAuth();
  const { run, busy } = useApiAction();
  const [name, setName] = useState("");
  const { data: maps, refresh } = usePoll<{ name: string }[]>("/api/mapping/maps", 0);
  const [loadName, setLoadName] = useState("");
  const st: MappingStatus | undefined = state?.mapping;
  const caps = state?.capabilities;

  if (caps && !caps.mapping && !caps.mapping_experimental) {
    return <Unavailable what="Mapping" reason="Mapping needs wheel encoders plus a LiDAR or range sensors. None were detected." />;
  }
  const op = can("operator");

  return (
    <div className="grid grid-cols-1 xl:grid-cols-4 gap-6">
      <Card title="Occupancy grid" className="xl:col-span-3"
            actions={st?.experimental ? <Pill tone="warn">EXPERIMENTAL: no LiDAR</Pill> : <Pill tone="ok">LiDAR mapping</Pill>}>
        <LiveMap height={560} />
        <div className="flex gap-4 text-xs text-slate-500 mt-2">
          <span><span className="inline-block w-3 h-3 bg-[#dee4ec] mr-1 align-middle" />free</span>
          <span><span className="inline-block w-3 h-3 bg-black border border-ink-600 mr-1 align-middle" />occupied</span>
          <span><span className="inline-block w-3 h-3 bg-[#262e3a] mr-1 align-middle" />unknown</span>
          <span className="text-accent">— planned path</span>
          <span>— trajectory</span>
          <span>1 m grid</span>
        </div>
      </Card>
      <div className="space-y-6">
        <Card title="Mapping control">
          <div className="space-y-2">
            <button className="btn-primary w-full" disabled={!op || busy || st?.active}
                    onClick={() => run(() => api.post("/api/mapping/start", { explore: true, reset: false }), "Exploration started")}>
              Start autonomous exploration
            </button>
            <button className="btn-ghost w-full" disabled={!op || busy || st?.active}
                    onClick={() => run(() => api.post("/api/mapping/start", { explore: false, reset: false }), "Mapping started (drive manually)")}>
              Map while driving manually
            </button>
            <button className="btn-ghost w-full" disabled={!op || busy || !st?.active}
                    onClick={() => run(() => api.post("/api/mapping/stop"), "Mapping stopped")}>
              Stop mapping
            </button>
            <button className="btn-ghost w-full" disabled={!op || busy}
                    onClick={() => confirm("Clear the current map?") && run(() => api.post("/api/mapping/reset"), "Map cleared")}>
              Reset map
            </button>
          </div>
          {st?.experimental && (
            <p className="text-xs text-warn mt-3">
              Without LiDAR the map is built from a few ultrasonic/ToF beams and wheel odometry. There is no loop
              closure, so drift accumulates; use it as a rough outline only.
            </p>
          )}
        </Card>
        <Card title="Save / load">
          <label className="label">Save current map as</label>
          <div className="flex gap-2">
            <input className="input" placeholder="living_room" value={name} onChange={(e) => setName(e.target.value)} />
            <button className="btn-primary" disabled={!op || !name || busy}
                    onClick={() => run(() => api.post("/api/mapping/save", { name }), "Map saved").then(refresh)}>Save</button>
          </div>
          <label className="label mt-3">Load saved map</label>
          <div className="flex gap-2">
            <select className="input" value={loadName} onChange={(e) => setLoadName(e.target.value)}>
              <option value="">select…</option>
              {maps?.map((m) => <option key={m.name}>{m.name}</option>)}
            </select>
            <button className="btn-ghost" disabled={!op || !loadName || busy}
                    onClick={() => run(() => api.post("/api/mapping/load", { name: loadName }), "Map loaded")}>Load</button>
          </div>
        </Card>
        <Card title="Statistics">
          <div className="kv"><span>Source</span><span>{st?.source ?? "—"}</span></div>
          <div className="kv"><span>Resolution</span><span>{fmt(st?.resolution, 2)} m</span></div>
          <div className="kv"><span>Size</span><span>{st ? `${st.width}×${st.height}` : "—"}</span></div>
          <div className="kv"><span>Free</span><span>{fmt((st?.free_ratio ?? 0) * 100, 1)} %</span></div>
          <div className="kv"><span>Occupied</span><span>{fmt((st?.occupied_ratio ?? 0) * 100, 1)} %</span></div>
          <div className="kv"><span>Unknown</span><span>{fmt((st?.unknown_ratio ?? 0) * 100, 1)} %</span></div>
          <div className="kv"><span>Sensor updates</span><span>{st?.updates ?? 0}</span></div>
          <div className="kv"><span>Loaded map</span><span>{st?.loaded_map ?? "—"}</span></div>
          {st?.scan_matching && (
            <>
              <div className="kv"><span>Scan matches (accepted)</span><span>{st.scan_matching.matches} ({st.scan_matching.accepted})</span></div>
              <div className="kv"><span>Last match score / shift</span><span>{st.scan_matching.last_score ?? "—"} / {st.scan_matching.last_shift_m ?? "—"} m</span></div>
              <div className="kv"><span>Match time</span><span>{st.scan_matching.last_ms} ms</span></div>
            </>
          )}
        </Card>
      </div>
    </div>
  );
}
