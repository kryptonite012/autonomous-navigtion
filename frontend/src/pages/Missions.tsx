import { useState } from "react";
import { Card, Pill, fmt } from "../components/ui";
import LiveMap from "../dashboards/LiveMap";
import { useApiAction, usePoll } from "../hooks/useApiAction";
import { useAuth } from "../hooks/useAuth";
import { useRobot } from "../hooks/useRobot";
import { api } from "../services/api";
import type { MissionDef, Waypoint } from "../services/types";

const STATE_TONE: Record<string, "ok" | "warn" | "bad" | "info" | "muted"> = {
  running: "info", dwelling: "info", returning: "info", completed: "ok", failed: "bad", aborted: "warn", idle: "muted",
};

const blank = (): MissionDef => ({ name: "", waypoints: [], loop: false, max_loops: 0, return_home: false, on_failure: "skip" });

function label(w: Waypoint) {
  return w.location ?? `(${fmt(w.x)}, ${fmt(w.y)})`;
}

export default function Missions() {
  const { state } = useRobot();
  const { can } = useAuth();
  const { run, busy } = useApiAction();
  const { data: missions, refresh } = usePoll<MissionDef[]>("/api/missions", 0);
  const [draft, setDraft] = useState<MissionDef>(blank());
  const [addLoc, setAddLoc] = useState("");
  const status = state?.mission;
  const named = Object.keys(state?.navigation.named_locations ?? {});
  const op = can("operator");
  const running = status && ["running", "dwelling", "returning"].includes(status.state);

  const addWaypoint = (w: Waypoint) => setDraft((d) => ({ ...d, waypoints: [...d.waypoints, w] }));
  const move = (i: number, delta: number) =>
    setDraft((d) => {
      const wps = [...d.waypoints];
      const j = i + delta;
      if (j < 0 || j >= wps.length) return d;
      [wps[i], wps[j]] = [wps[j], wps[i]];
      return { ...d, waypoints: wps };
    });

  const save = () =>
    run(() => api.put(`/api/missions/${encodeURIComponent(draft.name)}`, {
      waypoints: draft.waypoints, loop: draft.loop, max_loops: draft.max_loops,
      return_home: draft.return_home, on_failure: draft.on_failure,
    }), `Mission '${draft.name}' saved`).then(refresh);

  return (
    <div className="grid grid-cols-1 xl:grid-cols-3 gap-6">
      <Card title="Map — click to add a waypoint" className="xl:col-span-2"
            actions={status && <Pill tone={STATE_TONE[status.state] ?? "muted"}>{status.state}</Pill>}>
        <LiveMap height={500} onPick={op ? (x, y) => addWaypoint({ x: +x.toFixed(2), y: +y.toFixed(2), dwell_s: 0 }) : undefined} />
        {status?.mission && (
          <div className="mt-3 text-sm">
            <div><b>{status.mission}</b>: {status.message}</div>
            <div className="text-slate-400 text-xs mt-1">
              reached {status.reached} · loops {status.loops_done}
              {status.skipped.length > 0 && <> · skipped: {status.skipped.join(", ")}</>}
            </div>
            <div className="h-2 bg-ink-700 rounded mt-2">
              <div className="h-2 bg-accent rounded" style={{ width: `${status.waypoints ? (Math.min(status.waypoint_index, status.waypoints) / status.waypoints) * 100 : 0}%` }} />
            </div>
          </div>
        )}
      </Card>

      <div className="space-y-6">
        <Card title="Saved missions" actions={running && op && (
          <button className="btn-danger" onClick={() => run(() => api.post("/api/missions/stop"), "Mission stopped")}>Stop mission</button>
        )}>
          {!missions?.length && <div className="text-sm text-slate-500">No missions yet. Build one below.</div>}
          {missions?.map((m) => (
            <div key={m.name} className="flex items-center justify-between py-2 border-b border-ink-800 last:border-0">
              <div>
                <div className="text-sm font-medium">{m.name}</div>
                <div className="text-xs text-slate-500">
                  {m.waypoints.length} waypoints{m.loop ? ` · loop${m.max_loops ? ` ×${m.max_loops}` : ""}` : ""}
                  {m.return_home ? " · return home" : ""} · on failure: {m.on_failure}
                </div>
              </div>
              <div className="flex gap-1">
                <button className="btn-primary" disabled={!op || busy || !!running}
                        onClick={() => run(() => api.post(`/api/missions/${encodeURIComponent(m.name)}/start`), `Mission '${m.name}' started`)}>Start</button>
                <button className="btn-ghost" disabled={!op} onClick={() => setDraft(JSON.parse(JSON.stringify(m)))}>Edit</button>
                <button className="btn-ghost text-bad" disabled={!op || busy}
                        onClick={() => confirm(`Delete mission ${m.name}?`) &&
                          run(() => api.del(`/api/missions/${encodeURIComponent(m.name)}`), "Deleted").then(refresh)}>✕</button>
              </div>
            </div>
          ))}
        </Card>

        <Card title={draft.name ? `Edit: ${draft.name}` : "New mission"} actions={<button className="btn-ghost" onClick={() => setDraft(blank())}>Clear</button>}>
          <label className="block mb-2"><span className="label">Name</span>
            <input className="input" value={draft.name} onChange={(e) => setDraft({ ...draft, name: e.target.value })} placeholder="evening patrol" /></label>
          <div className="label">Waypoints (click the map, or add a named location)</div>
          <div className="flex gap-2 mb-2">
            <select className="input" value={addLoc} onChange={(e) => setAddLoc(e.target.value)}>
              <option value="">named location…</option>
              {named.map((n) => <option key={n}>{n}</option>)}
            </select>
            <button className="btn-ghost" disabled={!addLoc} onClick={() => { addWaypoint({ location: addLoc, dwell_s: 0 }); setAddLoc(""); }}>Add</button>
          </div>
          <ol className="space-y-1 mb-3">
            {draft.waypoints.map((w, i) => (
              <li key={i} className="flex items-center gap-2 text-sm bg-ink-850 rounded px-2 py-1">
                <span className="w-5 text-slate-500">{i + 1}.</span>
                <span className="flex-1 font-mono">{label(w)}</span>
                <input type="number" min={0} max={3600} title="dwell (s)" className="input w-16 py-0.5" value={w.dwell_s}
                       onChange={(e) => setDraft((d) => ({ ...d, waypoints: d.waypoints.map((x, j) => (j === i ? { ...x, dwell_s: +e.target.value } : x)) }))} />
                <button className="text-slate-500 hover:text-white" onClick={() => move(i, -1)}>↑</button>
                <button className="text-slate-500 hover:text-white" onClick={() => move(i, 1)}>↓</button>
                <button className="text-slate-500 hover:text-bad" onClick={() => setDraft((d) => ({ ...d, waypoints: d.waypoints.filter((_, j) => j !== i) }))}>✕</button>
              </li>
            ))}
          </ol>
          <div className="grid grid-cols-2 gap-2 text-sm">
            <label className="flex items-center gap-2"><input type="checkbox" checked={draft.loop} onChange={(e) => setDraft({ ...draft, loop: e.target.checked })} />Loop (patrol)</label>
            <label className="flex items-center gap-2"><input type="checkbox" checked={draft.return_home} onChange={(e) => setDraft({ ...draft, return_home: e.target.checked })} />Return home at end</label>
            <label><span className="label">Max loops (0 = ∞)</span>
              <input type="number" min={0} className="input" value={draft.max_loops} disabled={!draft.loop} onChange={(e) => setDraft({ ...draft, max_loops: +e.target.value })} /></label>
            <label><span className="label">If unreachable</span>
              <select className="input" value={draft.on_failure} onChange={(e) => setDraft({ ...draft, on_failure: e.target.value as "skip" | "abort" })}>
                <option value="skip">skip waypoint</option><option value="abort">abort mission</option>
              </select></label>
          </div>
          <button className="btn-primary w-full mt-3" disabled={!op || busy || !draft.name || !draft.waypoints.length} onClick={save}>Save mission</button>
          <p className="text-xs text-slate-500 mt-2">
            With a low battery (below navigation.return_voltage) any running mission is aborted and the robot drives to the dock.
          </p>
        </Card>
      </div>
    </div>
  );
}
