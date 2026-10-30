import { FormEvent, useState } from "react";
import { useAuth } from "../hooks/useAuth";

export default function Login() {
  const { login } = useAuth();
  const [username, setUsername] = useState("");
  const [password, setPassword] = useState("");
  const [error, setError] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);

  const submit = async (e: FormEvent) => {
    e.preventDefault();
    setBusy(true);
    setError(null);
    try {
      await login(username, password);
    } catch (err) {
      setError((err as Error).message);
    } finally {
      setBusy(false);
    }
  };

  return (
    <div className="min-h-full flex items-center justify-center p-6">
      <form onSubmit={submit} className="card w-full max-w-sm p-8 space-y-5">
        <div>
          <div className="text-2xl font-bold tracking-[0.3em] text-accent">NEXUS</div>
          <div className="text-sm text-slate-400 mt-1">Robot control console — sign in</div>
        </div>
        <label className="block">
          <span className="label">Username</span>
          <input className="input" autoFocus autoComplete="username" value={username}
                 onChange={(e) => setUsername(e.target.value)} />
        </label>
        <label className="block">
          <span className="label">Password</span>
          <input className="input" type="password" autoComplete="current-password" value={password}
                 onChange={(e) => setPassword(e.target.value)} />
        </label>
        {error && <div className="text-sm text-bad">{error}</div>}
        <button className="btn-primary w-full py-2" disabled={busy || !username || !password}>
          {busy ? "Signing in…" : "Sign in"}
        </button>
        <p className="text-xs text-slate-500">
          First start: the admin password is in <code>data/initial_admin_password</code> on the robot unless
          NEXUS_ADMIN_PASSWORD was set.
        </p>
      </form>
    </div>
  );
}
