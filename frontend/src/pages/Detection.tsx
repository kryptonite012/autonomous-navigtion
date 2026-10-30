import { useEffect, useState } from "react";
import CameraView from "../components/CameraView";
import { Card, COLOR_HEX, fmt, fmtTime } from "../components/ui";
import { useApiAction, usePoll } from "../hooks/useApiAction";
import { useAuth } from "../hooks/useAuth";
import { useRobot } from "../hooks/useRobot";
import { api } from "../services/api";
import type { MappedObject } from "../services/types";

interface VisionFull {
  colors: Record<string, { lower: number[]; upper: number[] }[]>;
  enabled_colors: string[];
  settings: { min_area_px: number; min_confidence: number; blur_kernel: number; morph_kernel: number };
  unique_objects: Record<string, number>;
}

const CH = ["H", "S", "V"];
const MAX = [179, 255, 255];

export default function Detection() {
  const { detections } = useRobot();
  const { can } = useAuth();
  const { run, busy } = useApiAction();
  const { data, refresh } = usePoll<VisionFull>("/api/vision/status", 0);
  const { data: omap, refresh: refreshObjects } = usePoll<{ calibrated: boolean; objects: MappedObject[] }>("/api/vision/objects", 2000);
  const [color, setColor] = useState("red");
  const [ranges, setRanges] = useState<{ lower: number[]; upper: number[] }[]>([]);
  const [roi, setRoi] = useState("270,190,100,100");
  const [settings, setSettings] = useState<VisionFull["settings"] | null>(null);
  const op = can("operator");

  useEffect(() => {
    if (data) {
      setRanges(JSON.parse(JSON.stringify(data.colors[color] ?? [])));
      setSettings(data.settings);
    }
  }, [data, color]);

  const setVal = (ri: number, bound: "lower" | "upper", ch: number, v: number) =>
    setRanges((rs) => rs.map((r, i) => (i === ri ? { ...r, [bound]: r[bound].map((x, j) => (j === ch ? v : x)) } : r)));

  const toggleColor = (c: string) => {
    if (!data) return;
    const en = data.enabled_colors.includes(c) ? data.enabled_colors.filter((x) => x !== c) : [...data.enabled_colors, c];
    run(() => api.put("/api/vision/settings", { enabled_colors: en }), `${c} ${en.includes(c) ? "enabled" : "disabled"}`).then(refresh);
  };

  return (
    <div className="space-y-6">
      <div className="grid grid-cols-1 xl:grid-cols-3 gap-6">
        <Card title="Detections" className="xl:col-span-2">
          <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
            <CameraView kind="annotated" />
            <CameraView kind="mask" />
          </div>
          <table className="w-full text-sm mt-4">
            <thead>
              <tr className="table-head"><th>ID</th><th>Colour</th><th>Confidence</th><th>Centroid</th><th>BBox</th><th>Area</th><th>Solidity</th><th>Distance</th><th>Time</th></tr>
            </thead>
            <tbody>
              {detections.items.map((d) => (
                <tr key={d.object_id ?? d.centroid.join()} className="border-t border-ink-800 font-mono">
                  <td>#{d.object_id}</td>
                  <td className="font-sans"><span className="inline-block h-2 w-2 rounded-full mr-1.5" style={{ background: COLOR_HEX[d.color] }} />{d.color}</td>
                  <td>{fmt(d.confidence)}</td>
                  <td>{d.centroid.map((c) => Math.round(c)).join(", ")}</td>
                  <td>{d.bbox.join(", ")}</td>
                  <td>{Math.round(d.area)}</td>
                  <td>{fmt(d.solidity)}</td>
                  <td title="only shown after camera calibration">{d.distance_m != null ? `${fmt(d.distance_m)} m` : "n/a"}</td>
                  <td>{fmtTime(d.timestamp)}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </Card>
        <div className="space-y-6">
          <Card title="Statistics">
            {Object.entries(data?.unique_objects ?? {}).map(([c, n]) => (
              <div className="kv" key={c}><span>{c}</span><span>{n} objects</span></div>
            ))}
            <div className="kv"><span>Processing</span><span>{fmt(detections.stats?.fps, 1)} fps / {fmt(detections.stats?.process_ms, 1)} ms</span></div>
          </Card>
          <Card title="Enabled colours">
            <div className="flex flex-wrap gap-2">
              {Object.keys(data?.colors ?? {}).map((c) => (
                <button key={c} disabled={!op || busy} onClick={() => toggleColor(c)}
                        className={`btn border ${data?.enabled_colors.includes(c) ? "border-accent text-white" : "border-ink-600 text-slate-500"}`}>
                  <span className="h-2 w-2 rounded-full" style={{ background: COLOR_HEX[c] }} />{c}
                </button>
              ))}
            </div>
          </Card>
          {settings && (
            <Card title="Filters">
              {(["min_area_px", "min_confidence", "blur_kernel", "morph_kernel"] as const).map((k) => (
                <label key={k} className="block mb-2">
                  <span className="label">{k.replace(/_/g, " ")}: {settings[k]}</span>
                  <input type="range" className="w-full accent-cyan-400"
                         min={k === "min_confidence" ? 0 : k === "min_area_px" ? 20 : 1}
                         max={k === "min_confidence" ? 1 : k === "min_area_px" ? 5000 : 21}
                         step={k === "min_confidence" ? 0.05 : k === "min_area_px" ? 10 : 2}
                         value={settings[k]} onChange={(e) => setSettings({ ...settings, [k]: +e.target.value })} />
                </label>
              ))}
              <button className="btn-primary w-full" disabled={!op || busy}
                      onClick={() => run(() => api.put("/api/vision/settings", settings), "Filters applied")}>Apply</button>
            </Card>
          )}
        </div>
      </div>

      <Card title="Objects on the map" actions={<button className="btn-ghost" disabled={!op || busy}
            onClick={() => run(() => api.del("/api/vision/objects"), "Object map cleared").then(refreshObjects)}>Clear</button>}>
        {!omap?.calibrated ? (
          <div className="text-sm text-slate-500">Objects are only placed on the map once the camera is calibrated (scripts/calibrate_camera.py), because a single uncalibrated camera gives no distance.</div>
        ) : (
          <table className="w-full text-sm">
            <thead><tr className="table-head"><th>ID</th><th>Colour</th><th>Position (m)</th><th>Sightings</th><th>Last seen</th><th>Status</th></tr></thead>
            <tbody>
              {omap.objects.map((o) => (
                <tr key={o.id} className="border-t border-ink-800">
                  <td className="font-mono">{o.id}</td>
                  <td><span className="inline-block h-2 w-2 rounded-full mr-1.5" style={{ background: COLOR_HEX[o.color] }} />{o.color}</td>
                  <td className="font-mono">({fmt(o.x)}, {fmt(o.y)})</td>
                  <td className="font-mono">{o.observations}</td>
                  <td className="font-mono">{fmtTime(o.last_seen)}</td>
                  <td>{o.sorted ? "picked" : "on floor"}</td>
                </tr>
              ))}
            </tbody>
          </table>
        )}
      </Card>

      <Card title="HSV calibration">
        <div className="flex flex-wrap items-end gap-4 mb-4">
          <label>
            <span className="label">Colour</span>
            <select className="input w-40" value={color} onChange={(e) => setColor(e.target.value)}>
              {Object.keys(data?.colors ?? {}).map((c) => <option key={c}>{c}</option>)}
            </select>
          </label>
          <label>
            <span className="label">Sample region x,y,w,h (place the object in it)</span>
            <input className="input w-48 font-mono" value={roi} onChange={(e) => setRoi(e.target.value)} />
          </label>
          <button className="btn-ghost" disabled={!op || busy} onClick={() =>
            run(() => api.post<{ ranges: { lower: number[]; upper: number[] }[] }>("/api/vision/calibrate", { color, roi: roi.split(",").map(Number) }), "Thresholds sampled from camera")
              .then((r) => r && setRanges(r.ranges))}>Sample from camera</button>
        </div>
        {ranges.map((r, ri) => (
          <div key={ri} className="grid grid-cols-1 md:grid-cols-3 gap-4 mb-3">
            {CH.map((ch, ci) => (
              <div key={ch} className="bg-ink-850 rounded-md p-3">
                <div className="label">Range {ri + 1} · {ch}: {r.lower[ci]} – {r.upper[ci]}</div>
                <input type="range" min={0} max={MAX[ci]} value={r.lower[ci]} className="w-full accent-cyan-400"
                       onChange={(e) => setVal(ri, "lower", ci, Math.min(+e.target.value, r.upper[ci]))} />
                <input type="range" min={0} max={MAX[ci]} value={r.upper[ci]} className="w-full accent-cyan-400"
                       onChange={(e) => setVal(ri, "upper", ci, Math.max(+e.target.value, r.lower[ci]))} />
              </div>
            ))}
          </div>
        ))}
        <div className="flex gap-2">
          <button className="btn-primary" disabled={!op || busy || !ranges.length}
                  onClick={() => run(() => api.post("/api/vision/calibrate", { color, ranges }), "Applied (not saved)").then(refresh)}>Apply</button>
          <button className="btn-ghost" disabled={!op || busy || !ranges.length}
                  onClick={() => run(() => api.post("/api/vision/calibrate", { color, ranges, persist: true }), "Applied and saved to config").then(refresh)}>Apply & save</button>
          <button className="btn-ghost" onClick={() => refresh()}>Revert</button>
        </div>
      </Card>
    </div>
  );
}
