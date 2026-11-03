// A modal that takes focus when shown, keeps Tab inside, and hands focus
// back to the element that had it when it hides.
export class Dialog {
  private opener: HTMLElement | null = null;

  constructor(
    private readonly element: HTMLElement,
    onEscape: () => void,
  ) {
    element.addEventListener("keydown", (e) => {
      if (e.key === "Escape") {
        e.preventDefault();
        e.stopPropagation();
        onEscape();
        return;
      }
      // Tab is walked by hand because Safari skips buttons on its own.
      if (e.key !== "Tab") return;
      const items = this.focusable();
      if (items.length === 0) return;
      const at = items.findIndex((item) => item === document.activeElement);
      const step = e.shiftKey ? -1 : 1;
      const next = at < 0 ? 0 : (at + step + items.length) % items.length;
      e.preventDefault();
      items[next]?.focus();
    });
    document.addEventListener("focusin", (e) => {
      if (!this.open || (e.target instanceof Node && element.contains(e.target))) return;
      this.focusable()[0]?.focus();
    });
  }

  get open(): boolean {
    return this.element.classList.contains("visible");
  }

  show(): void {
    if (this.open) return;
    this.opener = document.activeElement instanceof HTMLElement ? document.activeElement : null;
    this.element.classList.add("visible");
    this.focusable()[0]?.focus();
  }

  // Focus returns to the opener, or to `fallback` when the opener is gone.
  hide(fallback: HTMLElement): void {
    if (!this.open) return;
    this.element.classList.remove("visible");
    const back = this.opener?.isConnected ? this.opener : fallback;
    this.opener = null;
    back.focus({ preventScroll: true });
  }

  private focusable(): HTMLElement[] {
    return [...this.element.querySelectorAll<HTMLElement>("button:not([disabled])")];
  }
}
