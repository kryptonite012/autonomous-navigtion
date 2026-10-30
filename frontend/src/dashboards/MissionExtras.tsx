import { useState } from "react";
import { Card, Pill } from "../components/ui";
import { useApiAction, usePoll } from "../hooks/useApiAction";
import { api } from "../services/api";
import type { MissionDef, ScheduleDef } from "../services/types";

const DAYS = ["mon", "tue", "wed", "thu", "fri", "sat", "sun"];

/** Generate a lawnmower sweep over the free space of the loaded map. */
export function CoverageCard({ disabled, onCreated }: { disabled: boolean; onCreated: () => void }) {
  const { run, busy } = useApiAction();
  const [name, setName] = useState("sweep");
  const [spacing, setSpacing] = useState(0.5);
  const [loop, setLoop] = useState(false);
  return (
    <Card title="Coverage mission">
      <p className="text-xs text-slate-500 mb-2">
        Sweeps the whole known floor in rows, e.g. to find every object for sorting or for a patrol.
      </p>
      <div className="grid grid-cols-2 gap-2">
        <label><span className="label">Name</span><input className="input" value={name} onChange={(e) => setName(e.target.value)} /></label>
        <label><span className="label">Row spacing: {spacing.toFixed(2)} m</span>
          <input type="range" min={0.3} max={1.5} step={0.05} value={spacing} className="w-full accent-cyan-400"
                 onChange={(e) => setSpacing(+e.target.value)} /></label>
      </div>
      <label className="flex items-center gap-2 text-sm mt-2"><input type="checkbox" checked={loop} onChange={(e) => setLoop(e.target.checked)} />Repeat (patrol)</label>
      <button className="btn-primary w-full mt-3" disabled={disabled || busy || !name}
              onClick={() => run(() => api.post<{ mission: MissionDef }>("/api/missions/coverage", { name, spacing, loop, return_home: !loop }),
                `Coverage mission '${name}' created`).then((r) => r && onCreated())}>Generate from map</button>
    </Card>
  );
}

export function SchedulesCard({ missions, disabled }: { missions: string[]; disabled: boolean }) {
  const { run, busy } = useApiAction();
  const { data, refresh } = usePoll<ScheduleDef[]>("/api/schedules", 15000);
  const [name, setName] = useState("nightly patrol");
  const [mission, setMission] = useState("");
  const [time, setTime] = useState("21:00");
  const [days, setDays] = useState<string[]>([...DAYS]);

  const save = (body: Partial<ScheduleDef> & { name: string }) =>
    run(() => api.put(`/api/schedules/${encodeURIComponent(body.name)}`, {
      mission: body.mission, time: body.time, days: body.days, enabled: body.enabled ?? true,
    }), "Schedule saved").then(refresh);

  return (
    <Card title="Schedules">
      {!data?.length && <div className="text-sm text-slate-500 mb-2">No schedules.</div>}
      {data?.map((s) => (
        <div key={s.name} className="flex items-center justify-between py-1.5 border-b border-ink-800 text-sm">
          <div>
            <div className="font-medium">{s.name} <span className="text-slate-500 font-normal">→ {s.mission}</span></div>
            <div className="text-xs text-slate-500">
              {s.time} · {s.days.length === 7 ? "every day" : s.days.join(", ")}
              {s.last_run && <> · last {s.last_run}: {s.last_result}</>}
            </div>
          </div>
          <div className="flex gap-1 items-center">
            <button className="btn-ghost" disabled={disabled || busy} onClick={() => save({ ...s, enabled: !s.enabled })}>
              {s.enabled ? <Pill tone="ok">on</Pill> : <Pill>off</Pill>}
            </button>
            <button className="btn-ghost text-bad" disabled={disabled || busy}
                    onClick={() => run(() => api.del(`/api/schedules/${encodeURIComponent(s.name)}`), "Schedule deleted").then(refresh)}>✕</button>
          </div>
        </div>
      ))}
      <div className="grid grid-cols-2 gap-2 mt-3">
        <label><span className="label">Name</span><input className="input" value={name} onChange={(e) => setName(e.target.value)} /></label>
        <label><span className="label">Mission</span>
          <select className="input" value={mission} onChange={(e) => setMission(e.target.value)}>
            <option value="">select…</option>
            {missions.map((m) => <option key={m}>{m}</option>)}
          </select></label>
        <label><span className="label">Time (robot clock)</span><input type="time" className="input" value={time} onChange={(e) => setTime(e.target.value)} /></label>
      </div>
      <div className="flex flex-wrap gap-1 mt-2">
        {DAYS.map((d) => (
          <button key={d} className={`btn border text-xs ${days.includes(d) ? "border-accent text-white" : "border-ink-600 text-slate-500"}`}
                  onClick={() => setDays(days.includes(d) ? days.filter((x) => x !== d) : [...days, d])}>{d}</button>
        ))}
      </div>
      <button className="btn-primary w-full mt-3" disabled={disabled || busy || !name || !mission || !days.length}
              onClick={() => save({ name, mission, time, days, enabled: true })}>Add schedule</button>
      <p className="text-xs text-slate-500 mt-2">A run is skipped (and logged) if the robot is busy, e-stopped or not localised at that time.</p>
    </Card>
  );
}
