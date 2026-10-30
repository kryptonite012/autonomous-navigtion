import { useState } from "react";
import { Pill } from "../components/ui";
import { useApiAction, usePoll } from "../hooks/useApiAction";
import { authedUrl, api } from "../services/api";
import type { RecordingStatus } from "../services/types";

interface Listing {
  status: RecordingStatus;
  recordings: { name: string; size_bytes: number; modified: number; recording: boolean }[];
}

/** Flight recorder: raw sensor logs for offline replay (scripts/replay.py). */
export default function Recordings({ canOperate }: { canOperate: boolean }) {
  const { data, refresh } = usePoll<Listing>("/api/recordings", 2000);
  const { run, busy } = useApiAction();
  const stamp = new Date().toISOString().slice(0, 16).replace(/\D/g, "");
  const [name, setName] = useState(`run_${stamp}`);
  const st = data?.status;
  return (
    <div>
      <div className="flex flex-wrap items-end gap-2 mb-4">
        {st?.active ? (
          <>
            <Pill tone="bad">● recording {st.name}</Pill>
            <span className="text-sm text-slate-400 font-mono">{st.duration_s}s · {st.records} records · {st.scans} scans</span>
            <button className="btn-danger" disabled={!canOperate || busy}
                    onClick={() => run(() => api.post("/api/recordings/stop"), "Recording stopped").then(refresh)}>Stop</button>
          </>
        ) : (
          <>
            <label><span className="label">Name</span><input className="input w-56" value={name} onChange={(e) => setName(e.target.value)} /></label>
            <button className="btn-primary" disabled={!canOperate || busy || !name}
                    onClick={() => run(() => api.post("/api/recordings/start", { name }), "Recording started").then(refresh)}>Start recording</button>
          </>
        )}
      </div>
      <table className="w-full text-sm">
        <thead><tr className="table-head"><th>Name</th><th>Size</th><th>Saved</th><th /></tr></thead>
        <tbody>
          {data?.recordings.map((r) => (
            <tr key={r.name} className="border-t border-ink-800">
              <td className="py-1.5 font-mono">{r.name} {r.recording && <Pill tone="bad">live</Pill>}</td>
              <td className="font-mono">{(r.size_bytes / 1e6).toFixed(2)} MB</td>
              <td className="text-slate-400">{new Date(r.modified * 1000).toLocaleString()}</td>
              <td className="text-right space-x-2">
                {!r.recording && <a className="text-accent text-xs hover:underline" href={authedUrl(`/api/recordings/${r.name}/download`)}>download</a>}
                <button className="btn-ghost text-bad" disabled={!canOperate || busy || r.recording}
                        onClick={() => confirm(`Delete recording ${r.name}?`) &&
                          run(() => api.del(`/api/recordings/${r.name}`), "Deleted").then(refresh)}>Delete</button>
              </td>
            </tr>
          ))}
        </tbody>
      </table>
      {!data?.recordings.length && <div className="text-sm text-slate-500 py-4">No recordings yet.</div>}
      <p className="text-xs text-slate-500 mt-3">
        Records encoder ticks, gyro, range sensors, LiDAR scans, commands and pose estimates every control cycle
        (simulation adds ground truth). Replay offline with different parameters:
        <code className="ml-1">python3 scripts/replay.py run.jsonl.gz</code>. Recordings stop after {Math.round((st?.max_seconds ?? 1800) / 60)} min.
      </p>
    </div>
  );
}
