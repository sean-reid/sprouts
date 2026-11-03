import { mkdir, readFile, writeFile } from "node:fs/promises";
import AxeBuilder from "@axe-core/playwright";
import { expect, test, type Page } from "@playwright/test";
import {
  BOARD_BACKGROUND,
  CANDIDATE_ALPHA,
  HIGHLIGHT_COLOR,
  PLAYER_COLORS,
} from "../../web/render";
import {
  colourCentroid,
  drawBetween,
  expectDotAt,
  findDot,
  frameOf,
  nodeOnScreen,
  openBoard,
  ringSamplesAround,
  shot,
  tabKey,
} from "./helpers";

// The focus ring sits at NODE_RADIUS + 6 in web/render.ts.
const RING_RADIUS = 14;
// The colour `over` leaves on `under` at the given alpha.
function blend(over: string, under: string, alpha: number): string {
  const channel = (i: number): string => {
    const o = parseInt(over.slice(i, i + 2), 16);
    const u = parseInt(under.slice(i, i + 2), 16);
    return Math.round(o * alpha + u * (1 - alpha))
      .toString(16)
      .padStart(2, "0");
  };
  return `#${channel(1)}${channel(3)}${channel(5)}`;
}

const CANDIDATE = blend(PLAYER_COLORS.human, BOARD_BACKGROUND, CANDIDATE_ALPHA);
const REPLY_MS = 60_000;

const nodeButton = (page: Page, id: number) =>
  page.locator(`#nodeTargets button[data-node="${id}"]`);
const status = (page: Page) => page.locator("#status");

async function expectRing(page: Page, id: number): Promise<void> {
  const at = await nodeOnScreen(page, id, 4);
  await expect
    .poll(() => ringSamplesAround(page, at, RING_RADIUS, HIGHLIGHT_COLOR))
    .toBeGreaterThanOrEqual(8);
}

// Tab from the last toolbar button lands on the first node target.
async function tabToNode(page: Page, id: number): Promise<void> {
  await page.locator('button[data-first="ai"]').focus();
  for (let k = 0; k <= id; k++) await page.keyboard.press(tabKey());
  await expect(nodeButton(page, id)).toBeFocused();
}

async function pick(page: Page, id: number): Promise<void> {
  await tabToNode(page, id);
  await page.keyboard.press("Enter");
  await expect(status(page)).toHaveText(new RegExp(`^Node ${id} picked`));
}

async function loadFinished(page: Page): Promise<void> {
  const record = await readFile("fixtures/saved/finished-4n-500.json", "utf8");
  await page.addInitScript((saved) => localStorage.setItem("sprouts.game", saved), record);
  await page.goto("/");
  await expect(status(page)).toHaveText(/^Reviewing: move 9 of 9\./, { timeout: 60_000 });
}

async function openModal(page: Page): Promise<void> {
  const frame = await frameOf(page);
  await page.mouse.click(frame.left + frame.width / 2, frame.top + frame.height / 2);
  await expect(page.locator("#gameOverModal")).toBeVisible();
}

async function expectNoSeriousViolations(page: Page, where: string): Promise<void> {
  const { violations } = await new AxeBuilder({ page }).analyze();
  const serious = violations.filter((v) => v.impact === "serious" || v.impact === "critical");
  const report = serious.map(
    (v) => `${v.id}: ${v.nodes.map((n) => n.target.join(" ")).join(", ")}`,
  );
  expect(report, `axe on the ${where}`).toEqual([]);
}

test("every active node has a labelled 44 px target in id order", async ({ page }) => {
  await openBoard(page);
  const buttons = page.locator("#nodeTargets button");
  await expect(buttons).toHaveCount(4);
  for (let id = 0; id < 4; id++) {
    const button = nodeButton(page, id);
    await expect(button).toHaveAttribute("aria-label", `Node ${id}, 3 lines left`);
    const box = (await button.boundingBox())!;
    expect(box.width).toBeGreaterThanOrEqual(44);
    expect(box.height).toBeGreaterThanOrEqual(44);
    const at = await nodeOnScreen(page, id, 4);
    expect(Math.abs(box.x + box.width / 2 - at.x)).toBeLessThan(1.5);
    expect(Math.abs(box.y + box.height / 2 - at.y)).toBeLessThan(1.5);
  }
  await expect(buttons.nth(2)).toHaveAttribute("data-node", "2");
});

test("a move made with the keyboard alone gets the AI's reply", async ({ page }) => {
  await openBoard(page);
  await tabToNode(page, 0);
  await expectRing(page, 0);
  await shot(page, "keyboard-focus");

  await page.keyboard.press("Enter");
  await expect(status(page)).toHaveText(/^Node 0 picked/);
  await page.keyboard.press(tabKey());
  await expect(nodeButton(page, 1)).toBeFocused();
  await expectRing(page, 0);
  await expectRing(page, 1);

  await page.keyboard.press("Enter");
  await expect(status(page)).toHaveText(/^Node 0 to node 1\. Enter places the node/);
  const a = await nodeOnScreen(page, 0, 4);
  const b = await nodeOnScreen(page, 1, 4);
  await expectDotAt(page, { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 });
  await expect.poll(async () => (await colourCentroid(page, CANDIDATE)).count).toBeGreaterThan(20);
  await expect(page.locator("#cancelBtn")).toBeVisible();

  await page.keyboard.press("Enter");
  await expect(status(page)).toHaveText("Your turn!", { timeout: REPLY_MS });
  await expect(page.locator("#moveList li")).toHaveCount(2);
  await expect(page.locator("#moveList li").first()).toHaveText("1. You, joins two nodes");
  await shot(page, "keyboard-played");
});

test("Right and Left step through the moves a pair has and redraw the candidate", async ({
  page,
}) => {
  await openBoard(page);
  await pick(page, 0);
  await page.keyboard.press("Enter");
  await expect(status(page)).toHaveText(/^Move 1 of 4, loop on node 0\./);
  const first = await colourCentroid(page, CANDIDATE);
  expect(first.count).toBeGreaterThan(20);
  await shot(page, "keyboard-candidate-1");

  await page.keyboard.press("ArrowRight");
  await expect(status(page)).toHaveText(/^Move 2 of 4, loop on node 0\./);
  const second = await colourCentroid(page, CANDIDATE);
  expect(second.count).toBeGreaterThan(20);
  expect(Math.hypot(second.at.x - first.at.x, second.at.y - first.at.y)).toBeGreaterThan(8);
  await shot(page, "keyboard-candidate-2");

  await page.keyboard.press("ArrowLeft");
  await expect(status(page)).toHaveText(/^Move 1 of 4, loop on node 0\./);
  const again = await colourCentroid(page, CANDIDATE);
  expect(Math.hypot(again.at.x - first.at.x, again.at.y - first.at.y)).toBeLessThan(2);
  await expect(page.locator("#moveList")).toBeHidden();
});

test("Escape at each step returns to idle and keeps focus where it was", async ({ page }) => {
  await openBoard(page);
  await pick(page, 0);
  // Every node is a partner on a fresh board, so Tab walks all four and wraps.
  for (const id of [1, 2, 3, 0]) {
    await page.keyboard.press(tabKey());
    await expect(nodeButton(page, id)).toBeFocused();
  }
  await page.keyboard.press("Escape");
  await expect(status(page)).toHaveText("Move cancelled");
  await expect(nodeButton(page, 0)).toBeFocused();

  await page.keyboard.press("Enter");
  await expect(status(page)).toHaveText(/^Node 0 picked/);
  await page.keyboard.press(tabKey());
  await expect(nodeButton(page, 1)).toBeFocused();
  await page.keyboard.press("Enter");
  await expect(status(page)).toHaveText(/^Node 0 to node 1\./);
  await expect.poll(() => findDot(page)).not.toBeNull();
  await page.keyboard.press("Escape");
  await expect(status(page)).toHaveText("Move cancelled");
  await expect(nodeButton(page, 1)).toBeFocused();
  await expect(page.locator("#cancelBtn")).toBeHidden();
  expect(await findDot(page)).toBeNull();
  await expect(page.locator("#undoBtn")).toBeDisabled();
});

test("the game over dialog takes focus, keeps Tab inside and gives focus back", async ({
  page,
}) => {
  await loadFinished(page);
  await openModal(page);
  const modal = page.locator("#gameOverModal");
  await expect(modal).toHaveAttribute("role", "dialog");
  await expect(modal).toHaveAttribute("aria-modal", "true");
  await expect(modal).toHaveAccessibleName(/AI WINS|YOU WIN/);
  await expect(page.locator("#reviewBtn")).toBeFocused();
  await shot(page, "keyboard-modal");

  await page.keyboard.press(tabKey());
  await expect(page.locator("#newGameBtn")).toBeFocused();
  await page.keyboard.press(tabKey());
  await expect(page.locator("#reviewBtn")).toBeFocused();
  await page.keyboard.press(`Shift+${tabKey()}`);
  await expect(page.locator("#newGameBtn")).toBeFocused();

  await page.keyboard.press("Escape");
  await expect(modal).toBeHidden();
  await expect(status(page)).toHaveText(/^Reviewing: move 9 of 9\./);
  await expect(page.locator("#board")).toBeFocused();

  await openModal(page);
  await page.keyboard.press(tabKey());
  await expect(page.locator("#newGameBtn")).toBeFocused();
  await page.keyboard.press("Enter");
  await expect(modal).toBeHidden();
  await expect(status(page)).toHaveText("Draw a line between two nodes");
  await expect(page.locator("#board")).toBeFocused();
  await expect(page.locator("#nodeTargets button")).toHaveCount(4);
});

interface MotionProbe {
  replyAt: number | null;
  turnAt: number | null;
}

// Times the AI's reply leaving the worker and "Your turn!" reaching the
// status line, both on the page's clock.
function armMotionProbe(): void {
  const probe: MotionProbe = { replyAt: null, turnAt: null };
  (window as unknown as { motionProbe: MotionProbe }).motionProbe = probe;
  const methods = new Map<number, string>();
  const post = Worker.prototype.postMessage;
  Worker.prototype.postMessage = function (this: Worker, ...args: unknown[]) {
    const envelope = args[0] as { id: number; method: string };
    methods.set(envelope.id, envelope.method);
    return Reflect.apply(post, this, args);
  };
  const onmessage = Object.getOwnPropertyDescriptor(Worker.prototype, "onmessage")!;
  Object.defineProperty(Worker.prototype, "onmessage", {
    get() {
      return onmessage.get!.call(this);
    },
    set(handler: (event: MessageEvent<{ id: number }>) => void) {
      onmessage.set!.call(this, (event: MessageEvent<{ id: number }>) => {
        if (methods.get(event.data.id) === "aiMove") probe.replyAt = performance.now();
        handler(event);
      });
    },
  });
  document.addEventListener("DOMContentLoaded", () => {
    const el = document.getElementById("status")!;
    new MutationObserver(() => {
      if (el.textContent === "Your turn!" && probe.turnAt === null)
        probe.turnAt = performance.now();
    }).observe(el, { childList: true, characterData: true, subtree: true });
  });
}

// Plays one exchange and returns the milliseconds from the AI's reply to
// "Your turn!", also written under test-results so CI keeps the number.
async function replyToTurn(page: Page, mode: string): Promise<number> {
  const mid = await drawBetween(page, 0, 1);
  await expectDotAt(page, mid);
  await page.keyboard.press("Enter");
  await expect(status(page)).toHaveText("Your turn!", { timeout: REPLY_MS });
  const probe = await page.evaluate(
    () => (window as unknown as { motionProbe: MotionProbe }).motionProbe,
  );
  expect(probe.replyAt).not.toBeNull();
  expect(probe.turnAt).not.toBeNull();
  const took = probe.turnAt! - probe.replyAt!;
  await mkdir("test-results/fixtures", { recursive: true });
  await writeFile(
    `test-results/fixtures/${test.info().project.name}-motion-${mode}.txt`,
    `${took.toFixed(1)} ms\n`,
  );
  return took;
}

test("under reduced motion the AI's line lands as soon as the worker replies", async ({ page }) => {
  await page.addInitScript(armMotionProbe);
  await page.emulateMedia({ reducedMotion: "reduce" });
  await openBoard(page);
  const reduced = await replyToTurn(page, "reduced");
  expect(reduced).toBeLessThan(100);

  await page.emulateMedia({ reducedMotion: "no-preference" });
  await page.evaluate(() => localStorage.clear());
  await openBoard(page);
  const animated = await replyToTurn(page, "animated");
  expect(animated).toBeGreaterThanOrEqual(400);
});

test("the idle board, the placing state and the dialog have no serious axe violations", async ({
  page,
}) => {
  await openBoard(page);
  await expectNoSeriousViolations(page, "idle board");
  await drawBetween(page, 0, 1);
  await expectNoSeriousViolations(page, "placing state");
  await loadFinished(page);
  await openModal(page);
  await expectNoSeriousViolations(page, "open dialog");
});
