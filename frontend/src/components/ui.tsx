import { ReactNode } from "react";

export function Card({ title, actions, children, className = "" }: {
  title?: ReactNode;
  actions?: ReactNode;
  children: ReactNode;
  className?: string;
}) {
  return (
    <section className={`card p-3 sm:p-4 overflow-x-auto ${className}`}>
      {(title || actions) && (
        <header className="flex items-center justify-between mb-3 gap-2">
          {title && <h2 className="card-title">{title}</h2>}
          {actions && <div className="flex gap-2 flex-wrap">{actions}</div>}
        </header>
      )}
      {children}
    </section>
  );
}

const TONE: Record<string, string> = {
  ok: "bg-ok/15 text-ok border-ok/30",
  warn: "bg-warn/15 text-warn border-warn/30",
  bad: "bg-bad/15 text-bad border-bad/30",
  info: "bg-accent/10 text-accent border-accent/30",
  muted: "bg-ink-700 text-slate-400 border-ink-600",
};

export type Tone = keyof typeof TONE;

export function Pill({ tone = "muted", children }: { tone?: Tone; children: ReactNode }) {
  return (
    <span className={`inline-flex items-center gap-1.5 rounded-full border px-2 py-0.5 text-xs font-medium ${TONE[tone]}`}>
      {children}
    </span>
  );
}

export function Dot({ tone = "muted", pulse = false }: { tone?: Tone; pulse?: boolean }) {
  const c = { ok: "bg-ok", warn: "bg-warn", bad: "bg-bad", info: "bg-accent", muted: "bg-slate-500" }[tone];
  return <span className={`inline-block h-2 w-2 rounded-full ${c} ${pulse ? "animate-pulse" : ""}`} />;
}

export function Stat({ label, value, unit, tone, sub }: {
  label: string;
  value: ReactNode;
  unit?: string;
  tone?: Tone;
  sub?: ReactNode;
}) {
  const color = tone === "bad" ? "text-bad" : tone === "warn" ? "text-warn" : tone === "ok" ? "text-ok" : "text-slate-100";
  return (
    <div className="card px-4 py-3">
      <div className="card-title">{label}</div>
      <div className={`mt-1 text-2xl font-semibold font-mono ${color}`}>
        {value ?? "—"}
        {unit && value !== null && value !== undefined && <span className="ml-1 text-sm text-slate-400">{unit}</span>}
      </div>
      {sub && <div className="text-xs text-slate-500 mt-0.5">{sub}</div>}
    </div>
  );
}

export function Empty({ children }: { children: ReactNode }) {
  return <div className="text-sm text-slate-500 py-6 text-center">{children}</div>;
}

export function Unavailable({ what, reason }: { what: string; reason: string }) {
  return (
    <div className="card p-6 border-warn/40">
      <div className="text-warn font-medium">{what} unavailable</div>
      <div className="text-sm text-slate-400 mt-1">{reason}</div>
    </div>
  );
}

export function healthTone(state: string): Tone {
  return state === "ok" ? "ok" : state === "degraded" ? "warn" : state === "disabled" ? "muted" : "bad";
}

export function fmt(n: number | null | undefined, digits = 2): string {
  return n === null || n === undefined || Number.isNaN(n) ? "—" : n.toFixed(digits);
}

export function fmtTime(ts: number | string): string {
  const d = typeof ts === "number" ? new Date(ts * 1000) : new Date(ts);
  return d.toLocaleTimeString();
}

export const COLOR_HEX: Record<string, string> = {
  red: "#ef4444",
  green: "#22c55e",
  blue: "#3b82f6",
  yellow: "#eab308",
  orange: "#f97316",
  white: "#e5e7eb",
  black: "#4b5563",
};
