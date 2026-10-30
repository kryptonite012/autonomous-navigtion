import MapCanvas from "../components/MapCanvas";
import { usePoll } from "../hooks/useApiAction";
import { useRobot } from "../hooks/useRobot";
import type { MappingStatus, MapSnapshot, Pose } from "../services/types";

interface Current extends MapSnapshot {
  status: MappingStatus;
  pose: Pose;
  path: [number, number][];
}

/** Map + live pose/path. Polls the full grid, takes pose and path from the WebSocket. */
export default function LiveMap({ onPick, height = 480, goal }: {
  onPick?: (x: number, y: number) => void;
  height?: number;
  goal?: { x: number; y: number } | null;
}) {
  const { pose, navigation } = useRobot();
  const { data } = usePoll<Current>("/api/mapping/current", 1500);
  const nav = navigation;
  return (
    <MapCanvas
      map={data}
      pose={pose ?? data?.pose}
      path={nav?.path ?? data?.path ?? []}
      goal={goal ?? (nav?.goal ? { x: nav.goal.x, y: nav.goal.y } : null)}
      onPick={onPick}
      height={height}
    />
  );
}

export function useMappingStatus() {
  return usePoll<Current>("/api/mapping/current", 1500).data?.status ?? null;
}
