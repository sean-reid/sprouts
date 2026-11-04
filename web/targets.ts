import type { GameState, Node } from "./state";
import type { View } from "./view";

export function nodeLabel(node: Node): string {
  const left = 3 - node.connectionCount;
  return `Node ${node.id}, ${left} ${left === 1 ? "line" : "lines"} left`;
}

export interface TargetHandlers {
  // Enter, Space or an assistive-technology activation on a node.
  activate(id: number): void;
  // The node whose target holds keyboard focus, null when none does.
  focus(id: number | null): void;
}

// One focusable 44 px button per active node, over its disc, so Tab and
// assistive technology reach the board. Presses pass through to the canvas.
export class NodeTargets {
  private readonly buttons = new Map<number, HTMLButtonElement>();
  private partners: Set<number> | null = null;

  constructor(
    private readonly container: HTMLElement,
    private readonly fallback: HTMLElement,
    private readonly state: GameState,
    private readonly view: View,
    handlers: TargetHandlers,
  ) {
    container.addEventListener("click", (e) => {
      const id = this.idOf(e.target);
      if (id !== null) handlers.activate(id);
    });
    container.addEventListener("focusin", (e) => {
      const target = e.target instanceof HTMLElement ? e.target : null;
      const id = this.idOf(target);
      if (id !== null) this.reveal(id);
      handlers.focus(target?.matches(":focus-visible") ? id : null);
    });
    container.addEventListener("focusout", (e) => {
      if (e.relatedTarget instanceof Element && container.contains(e.relatedTarget)) return;
      handlers.focus(null);
    });
    container.addEventListener("keydown", (e) => {
      if (e.key !== "Tab" || !this.partners) return;
      const order = [...this.buttons.keys()].filter((id) => this.partners?.has(id));
      if (order.length === 0) return;
      const at = order.indexOf(this.idOf(e.target) ?? -1);
      const next =
        at < 0
          ? order[e.shiftKey ? order.length - 1 : 0]
          : order[(at + (e.shiftKey ? -1 : 1) + order.length) % order.length];
      e.preventDefault();
      this.buttons.get(next ?? -1)?.focus();
    });
  }

  // Keeps one button per active node in id order, relabelled with its lives.
  sync(): void {
    const active = this.state.nodes
      .filter((node) => this.state.activeNodes.has(node.id))
      .sort((a, b) => a.id - b.id);
    const wanted = new Set(active.map((node) => node.id));
    let lostFocus = false;
    for (const [id, button] of this.buttons) {
      if (wanted.has(id)) continue;
      if (button === document.activeElement) lostFocus = true;
      button.remove();
      this.buttons.delete(id);
    }
    let cursor = this.container.firstElementChild;
    for (const node of active) {
      let button = this.buttons.get(node.id);
      if (!button) {
        button = document.createElement("button");
        button.type = "button";
        button.dataset.node = String(node.id);
        this.buttons.set(node.id, button);
        this.container.insertBefore(button, cursor);
      } else {
        cursor = button.nextElementSibling;
      }
      button.setAttribute("aria-label", nodeLabel(node));
    }
    this.restrict(this.partners);
    this.place();
    if (lostFocus) this.fallback.focus({ preventScroll: true });
  }

  // Puts every button over its node as drawn at the current view, which
  // while a settle animates is the interpolated board.
  place(): void {
    for (const node of (this.state.settling ?? this.state).nodes) {
      const button = this.buttons.get(node.id);
      if (!button) continue;
      const at = this.view.toScreen(node.position);
      button.style.left = `${at.x}px`;
      button.style.top = `${at.y}px`;
    }
  }

  // Limits Tab to the given nodes; null lets it reach every node again.
  restrict(ids: Set<number> | null): void {
    this.partners = ids;
    for (const [id, button] of this.buttons) {
      button.tabIndex = ids === null || ids.has(id) ? 0 : -1;
    }
  }

  private idOf(target: EventTarget | null): number | null {
    const button = target instanceof Element ? target.closest<HTMLElement>("[data-node]") : null;
    return button ? Number(button.dataset.node) : null;
  }

  // Pans a node that sits outside the viewport to its centre.
  private reveal(id: number): void {
    const node = this.state.nodes.find((n) => n.id === id);
    if (!node) return;
    const at = this.view.toScreen(node.position);
    const { width, height } = this.view;
    if (at.x >= 0 && at.y >= 0 && at.x <= width && at.y <= height) return;
    this.view.panBy(width / 2 - at.x, height / 2 - at.y);
  }
}
