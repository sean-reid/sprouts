import type { Envelope, Reply, RequestOf, RequestType, Results } from "./protocol";

interface Pending {
  resolve: (value: unknown) => void;
  reject: (error: Error) => void;
  timer: ReturnType<typeof setTimeout>;
}

const CALL_TIMEOUT_MS = 30_000;

export class Engine {
  private readonly worker: Worker;
  private readonly pending = new Map<number, Pending>();
  private nextId = 0;

  constructor(onFailure: () => void) {
    this.worker = new Worker(new URL("./worker.ts", import.meta.url), { type: "module" });
    this.worker.onmessage = (event: MessageEvent<Reply>) => this.settle(event.data);
    this.worker.onerror = onFailure;
  }

  call<K extends RequestType>(request: RequestOf<K>): Promise<Results[K]> {
    return new Promise<Results[K]>((resolve, reject) => {
      const id = ++this.nextId;
      const timer = setTimeout(() => {
        this.pending.delete(id);
        reject(new Error(`Worker call '${request.type}' timed out`));
      }, CALL_TIMEOUT_MS);
      this.pending.set(id, { resolve: resolve as (value: unknown) => void, reject, timer });
      const envelope: Envelope = { id, request };
      this.worker.postMessage(envelope);
    });
  }

  private settle(reply: Reply): void {
    const entry = this.pending.get(reply.id);
    if (!entry) return;
    clearTimeout(entry.timer);
    this.pending.delete(reply.id);
    if ("error" in reply) entry.reject(new Error(reply.error));
    else entry.resolve(reply.result);
  }
}
