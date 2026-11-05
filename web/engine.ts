import type { Args, Envelope, Method, Reply, Result } from "./protocol";

export class StaleRequest extends Error {
  constructor(method: Method) {
    super(`Worker call '${method}' superseded`);
    this.name = "StaleRequest";
  }
}

export class CallTimeout extends Error {
  constructor(method: Method) {
    super(`Worker call '${method}' timed out`);
    this.name = "CallTimeout";
  }
}

interface Pending {
  method: Method;
  lane: string | null;
  resolve: (value: unknown) => void;
  reject: (error: Error) => void;
  timer: ReturnType<typeof setTimeout>;
}

const CALL_TIMEOUT_MS = 30_000;
// A share replays the whole game in the engine, routing every move, up
// to three times; on a slow machine a 30-move game needs about a minute
// per replay.
const SHARE_CALL_TIMEOUT_MS = 180_000;
const SHARE_METHODS: ReadonlySet<Method> = new Set<Method>(["indexMoves", "classify", "rehearse"]);

export class Engine {
  private readonly worker: Worker;
  private readonly pending = new Map<number, Pending>();
  private readonly lanes = new Map<string, number>();
  private nextId = 0;

  constructor(onFailure: () => void) {
    this.worker = new Worker(new URL("./worker.ts", import.meta.url), { type: "module" });
    this.worker.onmessage = (event: MessageEvent<Reply>) => this.settle(event.data);
    this.worker.onerror = onFailure;
  }

  call<M extends Method>(method: M, ...args: Args<M>): Promise<Result<M>> {
    return this.send(null, method, args);
  }

  // Supersedes the pending request in the lane, whose caller gets StaleRequest.
  latest<M extends Method>(lane: string, method: M, ...args: Args<M>): Promise<Result<M>> {
    this.cancel(lane);
    return this.send(lane, method, args);
  }

  // Ends the worker; pending requests never settle.
  dispose(): void {
    for (const entry of this.pending.values()) clearTimeout(entry.timer);
    this.pending.clear();
    this.worker.terminate();
  }

  cancel(lane: string): void {
    const id = this.lanes.get(lane);
    if (id === undefined) return;
    const entry = this.take(id);
    entry?.reject(new StaleRequest(entry.method));
  }

  private send<M extends Method>(
    lane: string | null,
    method: M,
    args: Args<M>,
  ): Promise<Result<M>> {
    return new Promise<Result<M>>((resolve, reject) => {
      const id = ++this.nextId;
      const timer = setTimeout(
        () => {
          this.take(id)?.reject(new CallTimeout(method));
        },
        SHARE_METHODS.has(method) ? SHARE_CALL_TIMEOUT_MS : CALL_TIMEOUT_MS,
      );
      this.pending.set(id, {
        method,
        lane,
        resolve: resolve as (value: unknown) => void,
        reject,
        timer,
      });
      if (lane !== null) this.lanes.set(lane, id);
      this.worker.postMessage({ id, method, args } as Envelope);
    });
  }

  private take(id: number): Pending | undefined {
    const entry = this.pending.get(id);
    if (!entry) return undefined;
    this.pending.delete(id);
    clearTimeout(entry.timer);
    if (entry.lane !== null && this.lanes.get(entry.lane) === id) this.lanes.delete(entry.lane);
    return entry;
  }

  private settle(reply: Reply): void {
    const entry = this.take(reply.id);
    if (!entry) return;
    if (reply.ok) entry.resolve(reply.result);
    else entry.reject(new Error(reply.error));
  }
}
