import { useEffect, useRef, useState } from "react";
import { api } from "../services/api";
import { useToast } from "./Toast";

type Dir = "fwd" | "back" | "left" | "right" | "fl" | "fr";
const TWIST: Record<Dir, [number, number]> = {
  fwd: [1, 0],
  back: [-1, 0],
  left: [0, 1],
  right: [0, -1],
  fl: [1, 0.6],
  fr: [1, -0.6],
};
const KEYS: Record<string, Dir> = { w: "fwd", s: "back", a: "left", d: "right", q: "fl", e: "fr" };

/**
 * Hold-to-drive controls. Commands are repeated at 5 Hz while held; when
 * released (or the tab loses focus) commands stop and the robot's command
 * watchdog brings it to a halt within ~0.6 s even if the stop is lost.
 */
export default function Teleop({ disabled }: { disabled?: boolean }) {
  const [speed, setSpeed] = useState(0.15);
  const [turn, setTurn] = useState(1.0);
  const [active, setActive] = useState<Dir | null>(null);
  const toast = useToast();
  const lastErr = useRef(0);

  useEffect(() => {
    if (!active || disabled) return;
    const send = () => {
      const [v, w] = TWIST[active];
      api.post("/api/robot/teleop", { linear: v * speed, angular: w * turn }).catch((e) => {
        if (Date.now() - lastErr.current > 2000) {
          lastErr.current = Date.now();
          toast.push("error", (e as Error).message);
        }
        setActive(null);
      });
    };
    send();
    const id = window.setInterval(send, 200);
    return () => {
      window.clearInterval(id);
      api.post("/api/robot/teleop", { linear: 0, angular: 0 }).catch(() => {});
    };
  }, [active, speed, turn, disabled, toast]);

  useEffect(() => {
    const down = (e: KeyboardEvent) => {
      if ((e.target as HTMLElement).tagName === "INPUT") return;
      const d = KEYS[e.key.toLowerCase()];
      if (d && !e.repeat) setActive(d);
      if (e.key === " ") setActive(null);
    };
    const up = (e: KeyboardEvent) => {
      if (KEYS[e.key.toLowerCase()]) setActive(null);
    };
    const blur = () => setActive(null);
    window.addEventListener("keydown", down);
    window.addEventListener("keyup", up);
    window.addEventListener("blur", blur);
    return () => {
      window.removeEventListener("keydown", down);
      window.removeEventListener("keyup", up);
      window.removeEventListener("blur", blur);
    };
  }, []);

  const pad = (d: Dir, label: string) => (
    <button
      disabled={disabled}
      onPointerDown={() => setActive(d)}
      onPointerUp={() => setActive(null)}
      onPointerLeave={() => setActive((a) => (a === d ? null : a))}
      className={`h-14 rounded-md border text-lg select-none ${
        active === d ? "bg-accent text-ink-950 border-accent" : "bg-ink-800 border-ink-600 hover:bg-ink-700"
      } disabled:opacity-30`}
    >
      {label}
    </button>
  );

  return (
    <div>
      <div className="grid grid-cols-3 gap-2 w-56">
        {pad("fl", "↖")}
        {pad("fwd", "▲")}
        {pad("fr", "↗")}
        {pad("left", "◀")}
        <button
          className="h-14 rounded-md border border-ink-600 bg-ink-800 text-xs hover:bg-ink-700"
          onClick={() => {
            setActive(null);
            api.post("/api/robot/stop").catch(() => {});
          }}
        >
          STOP
        </button>
        {pad("right", "▶")}
        <div />
        {pad("back", "▼")}
        <div />
      </div>
      <div className="mt-4 space-y-3 w-56">
        <label className="block">
          <span className="label">Linear speed: {speed.toFixed(2)} m/s</span>
          <input type="range" min={0.05} max={0.3} step={0.01} value={speed}
                 onChange={(e) => setSpeed(+e.target.value)} className="w-full accent-cyan-400" />
        </label>
        <label className="block">
          <span className="label">Turn rate: {turn.toFixed(1)} rad/s</span>
          <input type="range" min={0.3} max={2} step={0.1} value={turn}
                 onChange={(e) => setTurn(+e.target.value)} className="w-full accent-cyan-400" />
        </label>
        <p className="text-xs text-slate-500">Keyboard: W/A/S/D, Q/E diagonals, space to stop. Reverse is limited — there are no rear sensors.</p>
      </div>
    </div>
  );
}
