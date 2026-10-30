import { useEffect, useState } from "react";
import { authedUrl } from "../services/api";

/** MJPEG stream from the backend; reconnects if the stream drops. */
export default function CameraView({ kind = "annotated", className = "" }: { kind?: "raw" | "annotated" | "mask"; className?: string }) {
  const [nonce, setNonce] = useState(0);
  const [failed, setFailed] = useState(false);
  useEffect(() => setFailed(false), [kind]);
  useEffect(() => {
    if (!failed) return;
    const id = window.setTimeout(() => {
      setFailed(false);
      setNonce((n) => n + 1);
    }, 3000);
    return () => window.clearTimeout(id);
  }, [failed]);
  return (
    <div className={`relative bg-black rounded-md overflow-hidden border border-ink-700 aspect-[4/3] ${className}`}>
      {failed ? (
        <div className="absolute inset-0 flex items-center justify-center text-sm text-slate-500">
          camera stream unavailable — retrying…
        </div>
      ) : (
        <img
          key={`${kind}-${nonce}`}
          src={authedUrl(`/api/vision/stream.mjpg?kind=${kind}&fps=10&n=${nonce}`)}
          onError={() => setFailed(true)}
          className="w-full h-full object-contain"
          alt={`${kind} camera feed`}
        />
      )}
      <span className="absolute top-2 left-2 text-[10px] uppercase tracking-wider bg-black/60 px-1.5 py-0.5 rounded">{kind}</span>
    </div>
  );
}
