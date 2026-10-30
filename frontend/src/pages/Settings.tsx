import { useEffect, useState } from "react";
import { Card, Pill, fmt } from "../components/ui";
import { useApiAction, usePoll } from "../hooks/useApiAction";
import { useAuth } from "../hooks/useAuth";
import { api } from "../services/api";

interface UserRow {
  id: number;
  username: string;
  role: string;
  is_active: boolean;
  created_at: string;
}

interface SystemStatus {
  robot: string;
  hardware_mode: string;
  uptime_s: number;
  metrics: { cpu_temperature: number | null; cpu_percent: number | null; ram_percent: number | null; ram_used_mb: number | null; disk_percent: number | null };
  throttled: number | null;
  control_loop_hz: number;
  control_loop_overruns: number;
  websocket_clients: number;
  database_version: number;
}

const SAFETY_FIELDS = [
  ["emergency_stop_distance_m", "Emergency stop distance (m)", 0.05, 1.0, 0.01],
  ["slow_down_distance_m", "Slow-down distance (m)", 0.1, 2.0, 0.01],
  ["slow_down_factor", "Slow-down speed factor", 0.1, 1.0, 0.05],
  ["max_linear_mps", "Max linear speed (m/s)", 0.05, 0.5, 0.01],
  ["max_angular_rps", "Max turn rate (rad/s)", 0.2, 3.0, 0.1],
] as const;

export default function Settings() {
  const { can, user } = useAuth();
  const { run, busy } = useApiAction();
  const isAdmin = can("admin");
  const { data: sys } = usePoll<SystemStatus>("/api/system/status", 5000);
  const { data: users, refresh: refreshUsers } = usePoll<UserRow[]>(isAdmin ? "/api/users" : null, 0);
  const { data: cfg } = usePoll<{ safety: Record<string, number>; chassis: Record<string, number> }>("/api/system/config", 0);
  const [safety, setSafety] = useState<Record<string, number>>({});
  const [nu, setNu] = useState({ username: "", password: "", role: "operator" });
  const [pw, setPw] = useState({ current_password: "", new_password: "" });

  useEffect(() => {
    if (cfg) setSafety({ ...cfg.safety, max_linear_mps: cfg.chassis.max_linear_mps, max_angular_rps: cfg.chassis.max_angular_rps });
  }, [cfg]);

  return (
    <div className="grid grid-cols-1 xl:grid-cols-2 gap-6">
      <Card title="System">
        <div className="kv"><span>Robot</span><span>{sys?.robot}</span></div>
        <div className="kv"><span>Mode</span><span>{sys?.hardware_mode}</span></div>
        <div className="kv"><span>Uptime</span><span>{sys ? `${(sys.uptime_s / 3600).toFixed(2)} h` : "—"}</span></div>
        <div className="kv"><span>CPU / RAM / disk</span><span>{fmt(sys?.metrics.cpu_percent, 0)}% / {fmt(sys?.metrics.ram_percent, 0)}% / {fmt(sys?.metrics.disk_percent, 0)}%</span></div>
        <div className="kv"><span>Control loop</span><span>{fmt(sys?.control_loop_hz, 1)} Hz ({sys?.control_loop_overruns ?? 0} overruns)</span></div>
        <div className="kv"><span>Pi throttling flags</span><span>{sys?.throttled == null ? "n/a" : `0x${sys.throttled.toString(16)}`}</span></div>
        <div className="kv"><span>Dashboard clients</span><span>{sys?.websocket_clients}</span></div>
        <div className="kv"><span>Database schema</span><span>v{sys?.database_version}</span></div>
        {isAdmin && (
          <div className="flex gap-2 mt-4">
            <button className="btn-ghost" disabled={busy} onClick={() => run(() => api.post<{ file: string }>("/api/system/backup"), "Database backup created")}>Back up database</button>
            <button className="btn-warn" disabled={busy} onClick={() => confirm("Stop the robot and restart the NEXUS service?") &&
              run(() => api.post("/api/system/shutdown"), "Service shutting down")}>Restart service</button>
            <button className="btn-danger" disabled={busy} onClick={() => confirm("Power off the Raspberry Pi?") &&
              run(() => api.post("/api/system/shutdown?poweroff=true"), "Powering off")}>Power off</button>
          </div>
        )}
      </Card>

      <Card title="Safety limits" actions={!isAdmin && <Pill>admin only</Pill>}>
        {SAFETY_FIELDS.map(([k, label, min, max, step]) => (
          <label key={k} className="block mb-3">
            <span className="label">{label}: {fmt(safety[k], 2)}</span>
            <input type="range" min={min} max={max} step={step} value={safety[k] ?? min} disabled={!isAdmin}
                   className="w-full accent-cyan-400" onChange={(e) => setSafety({ ...safety, [k]: +e.target.value })} />
          </label>
        ))}
        <button className="btn-primary" disabled={!isAdmin || busy} onClick={() => run(() => api.put("/api/system/safety",
          Object.fromEntries(SAFETY_FIELDS.map(([k]) => [k, safety[k]]))), "Safety limits updated (until restart)")}>Apply</button>
        <p className="text-xs text-slate-500 mt-2">Changes apply immediately and last until restart. Set permanent values in config/nexus.yaml.</p>
      </Card>

      {isAdmin && (
        <Card title="Users">
          <table className="w-full text-sm mb-4">
            <thead><tr className="table-head"><th>User</th><th>Role</th><th>Status</th><th /></tr></thead>
            <tbody>
              {users?.map((u) => (
                <tr key={u.id} className="border-t border-ink-800">
                  <td className="py-1.5">{u.username}</td>
                  <td>
                    <select className="input w-32" value={u.role} disabled={u.username === user?.username}
                            onChange={(e) => run(() => api.patch(`/api/users/${u.id}`, { role: e.target.value }), "Role updated").then(refreshUsers)}>
                      <option>viewer</option><option>operator</option><option>admin</option>
                    </select>
                  </td>
                  <td>{u.is_active ? <Pill tone="ok">active</Pill> : <Pill>disabled</Pill>}</td>
                  <td className="text-right space-x-2">
                    {u.username !== user?.username && (
                      <>
                        <button className="btn-ghost" onClick={() => run(() => api.patch(`/api/users/${u.id}`, { is_active: !u.is_active }), "Updated").then(refreshUsers)}>
                          {u.is_active ? "Disable" : "Enable"}
                        </button>
                        <button className="btn-ghost text-bad" onClick={() => confirm(`Delete ${u.username}?`) &&
                          run(() => api.del(`/api/users/${u.id}`), "User deleted").then(refreshUsers)}>Delete</button>
                      </>
                    )}
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
          <div className="grid grid-cols-4 gap-2">
            <input className="input" placeholder="username" value={nu.username} onChange={(e) => setNu({ ...nu, username: e.target.value })} />
            <input className="input" type="password" placeholder="password" value={nu.password} onChange={(e) => setNu({ ...nu, password: e.target.value })} />
            <select className="input" value={nu.role} onChange={(e) => setNu({ ...nu, role: e.target.value })}>
              <option>viewer</option><option>operator</option><option>admin</option>
            </select>
            <button className="btn-primary" disabled={busy || !nu.username || !nu.password}
                    onClick={() => run(() => api.post("/api/users", nu), "User created").then(() => { setNu({ username: "", password: "", role: "operator" }); refreshUsers(); })}>Add user</button>
          </div>
          <p className="text-xs text-slate-500 mt-2">Passwords: at least 10 characters with 3 of lowercase, uppercase, digit, symbol.</p>
        </Card>
      )}

      <Card title="Change my password">
        <div className="space-y-2 max-w-sm">
          <input className="input" type="password" placeholder="current password" value={pw.current_password}
                 onChange={(e) => setPw({ ...pw, current_password: e.target.value })} />
          <input className="input" type="password" placeholder="new password" value={pw.new_password}
                 onChange={(e) => setPw({ ...pw, new_password: e.target.value })} />
          <button className="btn-primary" disabled={busy || !pw.current_password || !pw.new_password}
                  onClick={() => run(() => api.post("/api/auth/password", pw), "Password changed").then(() => setPw({ current_password: "", new_password: "" }))}>Change password</button>
        </div>
      </Card>
    </div>
  );
}
