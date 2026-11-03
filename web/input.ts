import type { Point } from "./state";
import { DOUBLE_TAP_ZOOM, type View, wheelZoomFactor } from "./view";

export type PointerKind = "mouse" | "pen" | "touch";

export interface StrokeHandlers {
  // True when the press began a stroke. One that did not still taps or ends
  // on release, unless a second finger lands first.
  start(world: Point, kind: PointerKind): boolean;
  move(world: Point, kind: PointerKind): void;
  // Runs once per pointermove that drove the stroke, after its coalesced samples.
  afterMove(world: Point, kind: PointerKind): void;
  end(world: Point, kind: PointerKind): void;
  tap(world: Point, kind: PointerKind): void;
  cancel(): void;
  // True when the press takes hold of something the page shows; the pointer
  // then drives drag until drop instead of drawing a stroke.
  grab(world: Point, kind: PointerKind): boolean;
  drag(world: Point, kind: PointerKind): void;
  // Pointer up, a second finger, or a lost pointer; what was held stays put.
  drop(): void;
}

export interface InputOptions {
  // World units; a shorter release is a tap.
  minStrokeLength: number;
}

interface Tracked {
  kind: PointerKind;
  screen: Point;
}

interface Stroke {
  id: number;
  drawing: boolean;
  length: number;
  last: Point;
}

interface Pinch {
  ids: [number, number];
  startDist: number;
  startWorld: Point;
  startScale: number;
}

const DOUBLE_TAP_MS = 300;
const DOUBLE_TAP_SLOP = 30;

function kindOf(e: PointerEvent): PointerKind {
  return e.pointerType === "touch" || e.pointerType === "pen" ? e.pointerType : "mouse";
}

function distance(a: Point, b: Point): number {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

function midpoint(a: Point, b: Point): Point {
  return { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
}

function wheelDelta(e: WheelEvent): Point {
  const unit = e.deltaMode === 1 ? 16 : e.deltaMode === 2 ? 100 : 1;
  return { x: e.deltaX * unit, y: e.deltaY * unit };
}

export function attachInput(
  canvas: HTMLCanvasElement,
  view: View,
  handlers: StrokeHandlers,
  options: InputOptions,
): void {
  const pointers = new Map<number, Tracked>();
  let stroke: Stroke | null = null;
  let held: number | null = null;
  let pinch: Pinch | null = null;
  let panPointer: number | null = null;
  let spaceHeld = false;
  let lastTap: { at: number; screen: Point } | null = null;

  const local = (e: { clientX: number; clientY: number }): Point => {
    const rect = canvas.getBoundingClientRect();
    return { x: e.clientX - rect.left, y: e.clientY - rect.top };
  };

  const toggleZoom = (screen: Point): void => {
    if (view.zoom > 1.01) view.fit();
    else view.anchor(view.toWorld(screen), screen, view.fitScale * DOUBLE_TAP_ZOOM);
  };

  const letGo = (): void => {
    if (held === null) return;
    held = null;
    handlers.drop();
  };

  const beginPinch = (): void => {
    letGo();
    if (stroke && !stroke.drawing) stroke = null;
    const [a, b] = [...pointers.entries()];
    if (!a || !b) return;
    const mid = midpoint(a[1].screen, b[1].screen);
    pinch = {
      ids: [a[0], b[0]],
      startDist: Math.max(1, distance(a[1].screen, b[1].screen)),
      startWorld: view.toWorld(mid),
      startScale: view.scale,
    };
  };

  const updatePinch = (): void => {
    if (!pinch) return;
    const a = pointers.get(pinch.ids[0]);
    const b = pointers.get(pinch.ids[1]);
    if (!a || !b) return;
    const ratio = distance(a.screen, b.screen) / pinch.startDist;
    view.anchor(pinch.startWorld, midpoint(a.screen, b.screen), pinch.startScale * ratio);
  };

  canvas.addEventListener("pointerdown", (e) => {
    const kind = kindOf(e);
    if (kind === "mouse" && e.button !== 0) return;
    const screen = local(e);
    pointers.set(e.pointerId, { kind, screen });
    try {
      canvas.setPointerCapture(e.pointerId);
    } catch {
      // The pointer is already gone; nothing to capture.
    }
    if (pointers.size === 2) {
      beginPinch();
      return;
    }
    if (pointers.size > 2) return;
    if (spaceHeld && kind === "mouse") {
      panPointer = e.pointerId;
      return;
    }
    const world = view.toWorld(screen);
    if (handlers.grab(world, kind)) {
      held = e.pointerId;
      return;
    }
    stroke = { id: e.pointerId, drawing: false, length: 0, last: world };
    stroke.drawing = handlers.start(world, kind);
  });

  canvas.addEventListener("pointermove", (e) => {
    const kind = kindOf(e);
    const tracked = pointers.get(e.pointerId);
    const events = e.getCoalescedEvents?.() ?? [];
    const samples = events.length > 0 ? events : [e];
    if (!tracked) {
      if (kind === "mouse") handlers.move(view.toWorld(local(e)), kind);
      return;
    }
    const screen = local(e);
    if (panPointer === e.pointerId) {
      view.panBy(screen.x - tracked.screen.x, screen.y - tracked.screen.y);
      tracked.screen = screen;
      return;
    }
    tracked.screen = screen;
    if (held === e.pointerId) {
      handlers.drag(view.toWorld(screen), kind);
      return;
    }
    if (pinch) {
      updatePinch();
      return;
    }
    if (!stroke || stroke.id !== e.pointerId) return;
    for (const sample of samples) {
      const world = view.toWorld(local(sample));
      stroke.length += distance(world, stroke.last);
      stroke.last = world;
      handlers.move(world, kind);
    }
    handlers.afterMove(stroke.last, kind);
  });

  const release = (e: PointerEvent): void => {
    const tracked = pointers.get(e.pointerId);
    if (!tracked) return;
    pointers.delete(e.pointerId);
    const kind = kindOf(e);
    const screen = local(e);
    if (panPointer === e.pointerId) panPointer = null;
    if (held === e.pointerId) {
      letGo();
      return;
    }
    if (pinch && pinch.ids.includes(e.pointerId)) {
      pinch = null;
      const remaining = pointers.entries().next().value;
      if (stroke && remaining) {
        stroke.id = remaining[0];
        stroke.last = view.toWorld(remaining[1].screen);
      } else if (stroke) {
        stroke = null;
        handlers.cancel();
      }
      return;
    }
    if (!stroke || stroke.id !== e.pointerId) return;
    const world = view.toWorld(screen);
    stroke.length += distance(world, stroke.last);
    const length = stroke.length;
    stroke = null;
    if (length >= options.minStrokeLength) {
      handlers.end(world, kind);
      return;
    }
    handlers.tap(world, kind);
    if (kind !== "touch") return;
    const now = e.timeStamp;
    if (
      lastTap &&
      now - lastTap.at < DOUBLE_TAP_MS &&
      distance(lastTap.screen, screen) < DOUBLE_TAP_SLOP
    ) {
      lastTap = null;
      toggleZoom(screen);
    } else {
      lastTap = { at: now, screen };
    }
  };
  canvas.addEventListener("pointerup", release);

  // A cancelled pointer never finishes a stroke; everything it drove resets.
  const cancelAll = (): void => {
    const hadStroke = stroke !== null;
    pointers.clear();
    stroke = null;
    pinch = null;
    panPointer = null;
    letGo();
    if (hadStroke) handlers.cancel();
  };
  const cancel = (e: PointerEvent): void => {
    if (pointers.has(e.pointerId)) cancelAll();
  };
  canvas.addEventListener("pointercancel", cancel);
  canvas.addEventListener("lostpointercapture", cancel);

  canvas.addEventListener(
    "wheel",
    (e) => {
      e.preventDefault();
      const delta = wheelDelta(e);
      if (e.ctrlKey || e.metaKey) view.zoomAt(local(e), wheelZoomFactor(delta.y));
      else view.panBy(-delta.x, -delta.y);
    },
    { passive: false },
  );

  document.addEventListener("keydown", (e) => {
    if (e.target instanceof HTMLButtonElement) return;
    if (e.code === "Space") {
      e.preventDefault();
      spaceHeld = true;
      canvas.classList.add("pan");
    } else if (e.key === "0" && !e.ctrlKey && !e.metaKey) {
      view.fit();
    }
  });
  document.addEventListener("keyup", (e) => {
    if (e.code === "Space") {
      spaceHeld = false;
      canvas.classList.remove("pan");
    }
  });
  window.addEventListener("blur", () => {
    spaceHeld = false;
    canvas.classList.remove("pan");
    cancelAll();
  });
}
