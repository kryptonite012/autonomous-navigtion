import { useCallback, useEffect, useState } from "react";
import { api } from "../services/api";
import { useToast } from "../components/Toast";

/** Run an API call, report success/failure through a toast, track busy state. */
export function useApiAction() {
  const toast = useToast();
  const [busy, setBusy] = useState(false);
  const run = useCallback(
    async <T,>(fn: () => Promise<T>, success?: string): Promise<T | undefined> => {
      setBusy(true);
      try {
        const r = await fn();
        if (success) toast.push("ok", success);
        return r;
      } catch (e) {
        toast.push("error", (e as Error).message);
        return undefined;
      } finally {
        setBusy(false);
      }
    },
    [toast],
  );
  return { run, busy };
}

/** Poll a GET endpoint at a fixed interval. */
export function usePoll<T>(path: string | null, intervalMs = 2000) {
  const [data, setData] = useState<T | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [tick, setTick] = useState(0);
  const refresh = useCallback(() => setTick((t) => t + 1), []);
  useEffect(() => {
    if (!path) return;
    let alive = true;
    const load = () =>
      api
        .get<T>(path)
        .then((d) => {
          if (alive) {
            setData(d);
            setError(null);
          }
        })
        .catch((e) => alive && setError((e as Error).message));
    load();
    const id = intervalMs > 0 ? window.setInterval(load, intervalMs) : undefined;
    return () => {
      alive = false;
      if (id) window.clearInterval(id);
    };
  }, [path, intervalMs, tick]);
  return { data, error, refresh };
}
