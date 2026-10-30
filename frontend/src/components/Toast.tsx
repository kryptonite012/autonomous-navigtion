import { createContext, ReactNode, useCallback, useContext, useMemo, useState } from "react";

type Kind = "ok" | "error" | "info";
interface Toast {
  id: number;
  kind: Kind;
  text: string;
}

const Ctx = createContext<{ push: (kind: Kind, text: string) => void } | null>(null);
let seq = 0;

export function ToastProvider({ children }: { children: ReactNode }) {
  const [items, setItems] = useState<Toast[]>([]);
  const push = useCallback((kind: Kind, text: string) => {
    const id = ++seq;
    setItems((t) => [...t, { id, kind, text }]);
    window.setTimeout(() => setItems((t) => t.filter((x) => x.id !== id)), kind === "error" ? 6000 : 3000);
  }, []);
  const value = useMemo(() => ({ push }), [push]);
  return (
    <Ctx.Provider value={value}>
      {children}
      <div className="fixed bottom-4 right-4 z-50 flex flex-col gap-2 w-80">
        {items.map((t) => (
          <div
            key={t.id}
            className={`card px-3 py-2 text-sm shadow-lg border-l-4 ${
              t.kind === "error" ? "border-l-bad" : t.kind === "ok" ? "border-l-ok" : "border-l-accent"
            }`}
          >
            {t.text}
          </div>
        ))}
      </div>
    </Ctx.Provider>
  );
}

export function useToast() {
  const c = useContext(Ctx);
  if (!c) throw new Error("useToast outside ToastProvider");
  return c;
}
