import { useState } from "react";
import { Card, Pill } from "../components/ui";
import { usePoll } from "../hooks/useApiAction";
import { useAuth } from "../hooks/useAuth";

interface EventRow {
  id: number;
  type: string;
  severity: string;
  description: string;
  timestamp: string;
}

interface AuditRow {
  id: number;
  username: string | null;
  action: string;
  detail: string | null;
  client_ip: string;
  success: boolean;
  timestamp: string;
}

const TONE: Record<string, "ok" | "warn" | "bad" | "info" | "muted"> = {
  info: "info", warning: "warn", error: "bad", critical: "bad", debug: "muted",
};

export default function Logs() {
  const [severity, setSeverity] = useState("");
  const [filter, setFilter] = useState("");
  const [tab, setTab] = useState<"events" | "audit">("events");
  const { can } = useAuth();
  const { data: events } = usePoll<EventRow[]>(`/api/logs/events?limit=500${severity ? `&severity=${severity}` : ""}`, 3000);
  const { data: audit } = usePoll<AuditRow[]>(tab === "audit" && can("admin") ? "/api/users/audit?limit=300" : null, 5000);
  const rows = (events ?? []).filter((e) => !filter || `${e.type} ${e.description}`.toLowerCase().includes(filter.toLowerCase()));

  const exportCsv = () => {
    const lines = ["timestamp,severity,type,description", ...rows.map((e) =>
      [e.timestamp, e.severity, e.type, `"${e.description.replace(/"/g, '""')}"`].join(","))];
    const a = document.createElement("a");
    a.href = URL.createObjectURL(new Blob([lines.join("\n")], { type: "text/csv" }));
    a.download = "nexus-events.csv";
    a.click();
  };

  return (
    <Card title={
      <div className="flex gap-4 normal-case tracking-normal">
        <button className={tab === "events" ? "text-white" : "text-slate-500"} onClick={() => setTab("events")}>System events</button>
        {can("admin") && <button className={tab === "audit" ? "text-white" : "text-slate-500"} onClick={() => setTab("audit")}>Audit log</button>}
      </div>
    } actions={tab === "events" && (
      <>
        <input className="input w-56" placeholder="search…" value={filter} onChange={(e) => setFilter(e.target.value)} />
        <select className="input w-36" value={severity} onChange={(e) => setSeverity(e.target.value)}>
          <option value="">all severities</option>
          {["info", "warning", "error", "critical"].map((s) => <option key={s}>{s}</option>)}
        </select>
        <button className="btn-ghost" onClick={exportCsv}>Export CSV</button>
      </>
    )}>
      {tab === "events" ? (
        <table className="w-full text-sm">
          <thead><tr className="table-head"><th className="w-44">Time</th><th className="w-24">Severity</th><th className="w-48">Type</th><th>Description</th></tr></thead>
          <tbody>
            {rows.map((e) => (
              <tr key={e.id} className="border-t border-ink-800 align-top">
                <td className="font-mono text-xs py-1.5 text-slate-400">{new Date(e.timestamp).toLocaleString()}</td>
                <td><Pill tone={TONE[e.severity] ?? "muted"}>{e.severity}</Pill></td>
                <td className="font-mono text-xs">{e.type}</td>
                <td>{e.description}</td>
              </tr>
            ))}
          </tbody>
        </table>
      ) : (
        <table className="w-full text-sm">
          <thead><tr className="table-head"><th>Time</th><th>User</th><th>Action</th><th>Detail</th><th>IP</th><th>Result</th></tr></thead>
          <tbody>
            {audit?.map((a) => (
              <tr key={a.id} className="border-t border-ink-800">
                <td className="font-mono text-xs py-1.5 text-slate-400">{new Date(a.timestamp).toLocaleString()}</td>
                <td>{a.username ?? "—"}</td>
                <td className="font-mono text-xs">{a.action}</td>
                <td className="text-slate-400 text-xs">{a.detail}</td>
                <td className="font-mono text-xs">{a.client_ip}</td>
                <td>{a.success ? <Pill tone="ok">ok</Pill> : <Pill tone="bad">denied</Pill>}</td>
              </tr>
            ))}
          </tbody>
        </table>
      )}
      {tab === "events" && !rows.length && <div className="text-sm text-slate-500 py-6">No events.</div>}
    </Card>
  );
}
