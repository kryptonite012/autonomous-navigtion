import { Card, Dot, Pill, healthTone } from "../components/ui";
import { usePoll } from "../hooks/useApiAction";
import type { DeviceHealth } from "../services/types";

interface Health {
  overall: string;
  devices: DeviceHealth[];
}

type Cfg = Record<string, Record<string, unknown>>;

const CAP_TEXT: Record<string, string> = {
  drive: "Motor driver + emergency stop input",
  odometry: "Wheel encoders",
  obstacle_detection: "Ultrasonic / ToF / LiDAR",
  vision: "Camera",
  mapping: "LiDAR + encoders (metric mapping)",
  mapping_experimental: "Range sensors + encoders (experimental mapping)",
  navigation: "Encoders + drive + range sensing",
  sorting: "Servo controller + camera",
  sort_verification: "Bin break-beam sensor",
  battery_monitoring: "ADS1115 battery ADC",
};

export default function Hardware() {
  const { data: health } = usePoll<Health>("/api/system/health", 3000);
  const { data: status } = usePoll<{ capabilities: Record<string, boolean>; hardware_mode: string }>("/api/system/status", 5000);
  const { data: cfg } = usePoll<Cfg>("/api/system/config", 0);
  const hw = (cfg?.hardware ?? {}) as Record<string, Record<string, unknown>>;

  return (
    <div className="space-y-6">
      <div className="grid grid-cols-1 xl:grid-cols-2 gap-6">
        <Card title="Devices" actions={status && <Pill tone={status.hardware_mode === "hardware" ? "ok" : "info"}>{status.hardware_mode}</Pill>}>
          <table className="w-full text-sm">
            <thead><tr className="table-head"><th>Device</th><th>Type</th><th>State</th><th>Detail</th></tr></thead>
            <tbody>
              {health?.devices.map((d) => (
                <tr key={d.name} className="border-t border-ink-800">
                  <td className="py-1.5"><Dot tone={healthTone(d.state)} /> <span className="ml-1">{d.name}</span></td>
                  <td className="text-slate-400">{d.kind}</td>
                  <td><Pill tone={healthTone(d.state)}>{d.state}{d.simulated ? " · sim" : ""}</Pill></td>
                  <td className="text-xs text-slate-500">{d.message}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </Card>
        <Card title="Capabilities (derived from detected hardware)">
          {Object.entries(status?.capabilities ?? {}).map(([k, v]) => (
            <div key={k} className="flex items-center justify-between py-1.5 border-b border-ink-800 last:border-0 text-sm">
              <span><Dot tone={v ? "ok" : "muted"} /> <span className="ml-1">{k.replace(/_/g, " ")}</span></span>
              <span className="text-xs text-slate-500">{CAP_TEXT[k]}</span>
            </div>
          ))}
        </Card>
      </div>
      <Card title="Configured hardware (config/nexus.yaml)">
        <div className="grid grid-cols-1 md:grid-cols-2 xl:grid-cols-3 gap-4">
          {Object.entries(hw).map(([name, c]) => (
            <div key={name} className="bg-ink-850 rounded-md p-3">
              <div className="flex justify-between mb-1">
                <span className="font-medium text-sm">{name}</span>
                <Pill tone={c.enabled ? "ok" : "muted"}>{c.enabled ? "enabled" : "disabled"}</Pill>
              </div>
              <pre className="text-[11px] text-slate-400 whitespace-pre-wrap font-mono">{JSON.stringify(c, null, 1).replace(/[{}"]/g, "").trim()}</pre>
            </div>
          ))}
        </div>
        <p className="text-xs text-slate-500 mt-3">
          Pin assignments are edited in config/nexus.yaml on the robot and take effect after restarting the service.
          See docs/hardware/wiring.md for the wiring and GPIO table.
        </p>
      </Card>
    </div>
  );
}
