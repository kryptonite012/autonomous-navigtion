import CameraView from "../components/CameraView";
import { Card, COLOR_HEX, Pill, Unavailable, fmt } from "../components/ui";
import { useApiAction } from "../hooks/useApiAction";
import { useAuth } from "../hooks/useAuth";
import { useRobot } from "../hooks/useRobot";
import { api } from "../services/api";

export default function Camera() {
  const { state, detections } = useRobot();
  const { can } = useAuth();
  const { run, busy } = useApiAction();
  const stats = detections.stats ?? state?.vision;
  if (state && !state.capabilities.vision) {
    return <Unavailable what="Camera" reason="No camera was detected at startup. Check the ribbon cable or USB connection and restart the service." />;
  }
  const counts: Record<string, number> = {};
  detections.items.forEach((d) => (counts[d.color] = (counts[d.color] ?? 0) + 1));

  return (
    <div className="space-y-6">
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        <Card title="Original feed"><CameraView kind="raw" /></Card>
        <Card title="Processed feed (detections)"><CameraView kind="annotated" /></Card>
      </div>
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        <Card title="Pipeline" actions={can("operator") && (
          stats?.running
            ? <button className="btn-ghost" disabled={busy} onClick={() => run(() => api.post("/api/vision/stop"), "Vision stopped")}>Stop</button>
            : <button className="btn-primary" disabled={busy} onClick={() => run(() => api.post("/api/vision/start"), "Vision started")}>Start</button>
        )}>
          <div className="kv"><span>Status</span><span>{stats?.running ? "running" : "stopped"}</span></div>
          <div className="kv"><span>Processing rate</span><span>{fmt(stats?.fps, 1)} fps</span></div>
          <div className="kv"><span>Processing time</span><span>{fmt(stats?.process_ms, 1)} ms</span></div>
          <div className="kv"><span>Frames / rejected</span><span>{stats?.frames ?? 0} / {stats?.rejected_frames ?? 0}</span></div>
          <div className="kv"><span>Distance estimates</span><span>{stats?.distance_estimation ? "calibrated" : "off (uncalibrated)"}</span></div>
          <div className="kv"><span>Source</span><span>{stats?.simulated ? "simulated camera" : "physical camera"}</span></div>
          {stats?.last_error && <div className="text-sm text-warn mt-2">{stats.last_error}</div>}
        </Card>
        <Card title={`Objects in view (${detections.items.length})`}>
          <div className="flex flex-wrap gap-2 mb-3">
            {Object.entries(counts).map(([c, n]) => (
              <Pill key={c}><span className="h-2 w-2 rounded-full" style={{ background: COLOR_HEX[c] }} />{c}: {n}</Pill>
            ))}
          </div>
          <table className="w-full text-sm">
            <thead><tr className="table-head"><th>ID</th><th>Colour</th><th>Conf.</th><th>Area</th></tr></thead>
            <tbody>
              {detections.items.map((d) => (
                <tr key={d.object_id ?? d.centroid.join()} className="border-t border-ink-800">
                  <td className="font-mono">#{d.object_id}</td><td>{d.color}</td>
                  <td className="font-mono">{fmt(d.confidence)}</td><td className="font-mono">{Math.round(d.area)}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </Card>
        <Card title="Mask view (all enabled colours)"><CameraView kind="mask" /></Card>
      </div>
    </div>
  );
}
