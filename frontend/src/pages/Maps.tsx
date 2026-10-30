import { useState } from "react";
import { Card, Pill } from "../components/ui";
import { useApiAction, usePoll } from "../hooks/useApiAction";
import { useAuth } from "../hooks/useAuth";
import { authedUrl, api } from "../services/api";

interface MapRow {
  name: string;
  modified: number;
  resolution: number | null;
  experimental: boolean | null;
  loaded: boolean;
}

export default function Maps() {
  const { data, refresh } = usePoll<MapRow[]>("/api/mapping/maps", 5000);
  const { can } = useAuth();
  const { run, busy } = useApiAction();
  const [renaming, setRenaming] = useState<string | null>(null);
  const [newName, setNewName] = useState("");
  const op = can("operator");

  return (
    <Card title="Saved maps" actions={<button className="btn-ghost" onClick={refresh}>Refresh</button>}>
      <table className="w-full text-sm">
        <thead><tr className="table-head"><th>Name</th><th>Saved</th><th>Resolution</th><th>Source</th><th>Files</th><th /></tr></thead>
        <tbody>
          {data?.map((m) => (
            <tr key={m.name} className="border-t border-ink-800">
              <td className="py-2">
                {renaming === m.name ? (
                  <div className="flex gap-2">
                    <input className="input w-48" value={newName} onChange={(e) => setNewName(e.target.value)} />
                    <button className="btn-primary" disabled={!newName || busy} onClick={() =>
                      run(() => api.post("/api/mapping/rename", { old_name: m.name, new_name: newName }), "Renamed").then(() => { setRenaming(null); refresh(); })}>OK</button>
                    <button className="btn-ghost" onClick={() => setRenaming(null)}>Cancel</button>
                  </div>
                ) : (
                  <span className="font-medium">{m.name} {m.loaded && <Pill tone="ok">loaded</Pill>}</span>
                )}
              </td>
              <td className="text-slate-400">{new Date(m.modified * 1000).toLocaleString()}</td>
              <td className="font-mono">{m.resolution ? `${m.resolution} m` : "—"}</td>
              <td>{m.experimental == null ? "—" : m.experimental ? <Pill tone="warn">experimental</Pill> : <Pill tone="ok">lidar</Pill>}</td>
              <td className="space-x-2 text-xs">
                <a className="text-accent hover:underline" href={authedUrl(`/api/mapping/maps/${m.name}/download?fmt=yaml`)}>yaml</a>
                <a className="text-accent hover:underline" href={authedUrl(`/api/mapping/maps/${m.name}/download?fmt=pgm`)}>pgm</a>
              </td>
              <td className="text-right space-x-2">
                <button className="btn-ghost" disabled={!op || busy}
                        onClick={() => run(() => api.post("/api/mapping/load", { name: m.name }), `Loaded ${m.name}`).then(refresh)}>Load</button>
                <button className="btn-ghost" disabled={!op} onClick={() => { setRenaming(m.name); setNewName(m.name); }}>Rename</button>
                <button className="btn-ghost text-bad" disabled={!op || busy}
                        onClick={() => confirm(`Delete map ${m.name}? This cannot be undone.`) &&
                          run(() => api.del(`/api/mapping/maps/${m.name}`), "Deleted").then(refresh)}>Delete</button>
              </td>
            </tr>
          ))}
        </tbody>
      </table>
      {!data?.length && <div className="text-sm text-slate-500 py-6">No maps saved yet. Create one on the Mapping screen.</div>}
      <p className="text-xs text-slate-500 mt-4">Maps are stored in ROS map_server format (PGM + YAML) and can be used directly with Nav2's map_server.</p>
    </Card>
  );
}
