import { useEffect, useRef } from "react";
import type { MapSnapshot, Pose } from "../services/types";

interface Props {
  map: MapSnapshot | null;
  pose?: Pose | null;
  path?: [number, number][];
  goal?: { x: number; y: number } | null;
  obstacles?: [number, number][];
  onPick?: (x: number, y: number) => void;
  height?: number;
}

const COLORS = { unknown: [38, 46, 58], free: [222, 228, 236], occ: [12, 14, 18] };

/** Renders the occupancy grid with robot, trajectory, path and goal. Click to pick a goal. */
export default function MapCanvas({ map, pose, path = [], goal, obstacles = [], onPick, height = 480 }: Props) {
  const ref = useRef<HTMLCanvasElement>(null);
  const gridImg = useRef<ImageData | null>(null);

  // decode the run-length grid into an image once per map update
  useEffect(() => {
    if (!map) return;
    const img = new ImageData(map.width, map.height);
    let i = 0;
    for (const [v, n] of map.data_rle) {
      const c = v === -1 ? COLORS.unknown : v === 0 ? COLORS.free : COLORS.occ;
      for (let k = 0; k < n; k++, i++) {
        // grid row 0 is the lowest y: flip vertically for the screen
        const row = map.height - 1 - Math.floor(i / map.width);
        const col = i % map.width;
        const o = (row * map.width + col) * 4;
        img.data[o] = c[0];
        img.data[o + 1] = c[1];
        img.data[o + 2] = c[2];
        img.data[o + 3] = 255;
      }
    }
    gridImg.current = img;
  }, [map]);

  useEffect(() => {
    const cv = ref.current;
    if (!cv) return;
    const ctx = cv.getContext("2d")!;
    const W = cv.clientWidth;
    const H = height;
    cv.width = W * devicePixelRatio;
    cv.height = H * devicePixelRatio;
    ctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
    ctx.fillStyle = "#0c1016";
    ctx.fillRect(0, 0, W, H);
    if (!map || !gridImg.current) {
      ctx.fillStyle = "#64748b";
      ctx.font = "13px sans-serif";
      ctx.fillText("No map data yet", 16, 24);
      return;
    }
    const scale = Math.min(W / map.width, H / map.height);
    const ox = (W - map.width * scale) / 2;
    const oy = (H - map.height * scale) / 2;
    const tmp = document.createElement("canvas");
    tmp.width = map.width;
    tmp.height = map.height;
    tmp.getContext("2d")!.putImageData(gridImg.current, 0, 0);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(tmp, ox, oy, map.width * scale, map.height * scale);

    const toPx = (x: number, y: number): [number, number] => [
      ox + ((x - map.origin[0]) / map.resolution) * scale,
      oy + (map.height - (y - map.origin[1]) / map.resolution) * scale,
    ];

    // 1 m grid lines
    ctx.strokeStyle = "rgba(34,211,238,0.07)";
    ctx.lineWidth = 1;
    const cells = 1 / map.resolution;
    for (let c = 0; c <= map.width; c += cells) {
      ctx.beginPath();
      ctx.moveTo(ox + c * scale, oy);
      ctx.lineTo(ox + c * scale, oy + map.height * scale);
      ctx.stroke();
    }
    for (let r = 0; r <= map.height; r += cells) {
      ctx.beginPath();
      ctx.moveTo(ox, oy + r * scale);
      ctx.lineTo(ox + map.width * scale, oy + r * scale);
      ctx.stroke();
    }

    if (map.trajectory.length > 1) {
      ctx.strokeStyle = "rgba(148,163,184,0.7)";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      map.trajectory.forEach(([x, y], i) => {
        const [px, py] = toPx(x, y);
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke();
    }
    if (path.length > 1) {
      ctx.strokeStyle = "#22d3ee";
      ctx.lineWidth = 2;
      ctx.setLineDash([6, 4]);
      ctx.beginPath();
      path.forEach(([x, y], i) => {
        const [px, py] = toPx(x, y);
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke();
      ctx.setLineDash([]);
    }
    ctx.fillStyle = "#f97316";
    for (const [x, y] of obstacles) {
      const [px, py] = toPx(x, y);
      ctx.fillRect(px - 2, py - 2, 4, 4);
    }
    if (goal) {
      const [gx, gy] = toPx(goal.x, goal.y);
      ctx.strokeStyle = "#22c55e";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(gx, gy, 8, 0, Math.PI * 2);
      ctx.moveTo(gx - 12, gy);
      ctx.lineTo(gx + 12, gy);
      ctx.moveTo(gx, gy - 12);
      ctx.lineTo(gx, gy + 12);
      ctx.stroke();
    }
    if (pose) {
      const [px, py] = toPx(pose.x, pose.y);
      const r = Math.max(6, (0.12 / map.resolution) * scale);
      ctx.fillStyle = pose.valid ? "rgba(34,211,238,0.25)" : "rgba(239,68,68,0.3)";
      ctx.strokeStyle = pose.valid ? "#22d3ee" : "#ef4444";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(px, py, r, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(px, py);
      ctx.lineTo(px + Math.cos(pose.theta) * r * 1.8, py - Math.sin(pose.theta) * r * 1.8);
      ctx.stroke();
    }
  }, [map, pose, path, goal, obstacles, height]);

  const click = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!onPick || !map) return;
    const cv = ref.current!;
    const rect = cv.getBoundingClientRect();
    const W = rect.width;
    const H = height;
    const scale = Math.min(W / map.width, H / map.height);
    const ox = (W - map.width * scale) / 2;
    const oy = (H - map.height * scale) / 2;
    const cx = (e.clientX - rect.left - ox) / scale;
    const cy = map.height - (e.clientY - rect.top - oy) / scale;
    onPick(map.origin[0] + cx * map.resolution, map.origin[1] + cy * map.resolution);
  };

  return (
    <canvas
      ref={ref}
      onClick={click}
      style={{ width: "100%", height }}
      className={`rounded-md border border-ink-700 ${onPick ? "cursor-crosshair" : ""}`}
    />
  );
}
