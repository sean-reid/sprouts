import { readFile } from "node:fs/promises";
import { expect, test, type Page } from "@playwright/test";
import { awaitReply, openBoard, playExchange, shot, startGame } from "./helpers";

interface Box {
  id: string;
  left: number;
  top: number;
  width: number;
  height: number;
}

interface Toolbar {
  titleHeight: number;
  playTop: number;
  boardTop: number;
  canvas: { left: number; top: number; width: number; height: number };
  listHeight: number;
  setupShown: boolean;
  turnShown: boolean;
  compactShown: boolean;
  undoInHistoryRow: boolean;
  controlsTop: number;
  startTop: number;
  compactTop: number;
  targets: Box[];
}

function phone(): boolean {
  return (test.info().project.use.viewport?.width ?? 0) < 768;
}

// Every control a finger can press, with its box, plus where the main parts sit.
function toolbarOf(page: Page): Promise<Toolbar> {
  return page.evaluate(() => {
    const rect = (el: Element) => el.getBoundingClientRect();
    const shown = (el: Element | null) => !!el && rect(el).width > 0 && rect(el).height > 0;
    const targets: Box[] = [];
    for (const el of document.querySelectorAll<HTMLElement>("button, input")) {
      if (!shown(el) || el.closest("#nodeTargets, #moveList") || el.closest("[hidden]")) continue;
      if (getComputedStyle(el).visibility === "hidden") continue;
      const r = rect(el);
      targets.push({
        id: el.id || el.textContent?.trim() || el.tagName,
        left: r.left,
        top: r.top,
        width: r.width,
        height: r.height,
      });
    }
    const canvas = rect(document.getElementById("canvas")!);
    return {
      titleHeight: rect(document.querySelector("h1")!).height,
      playTop: rect(document.querySelector(".play-area")!).top,
      boardTop: canvas.top,
      canvas: { left: canvas.left, top: canvas.top, width: canvas.width, height: canvas.height },
      listHeight: rect(document.getElementById("moveList")!).height,
      setupShown: shown(document.querySelector(".controls.setup")),
      turnShown: shown(document.getElementById("turn")),
      compactShown: shown(document.getElementById("aiStartsBtn")),
      undoInHistoryRow: !!document.getElementById("undoBtn")?.closest("#historyNav"),
      controlsTop: rect(document.getElementById("gameControls")!).top,
      startTop: rect(document.getElementById("startBtn")!).top,
      compactTop: rect(document.getElementById("aiStartsBtn")!).top,
      targets,
    };
  });
}

function expectTargetsApart(targets: Box[]): void {
  for (const t of targets) {
    expect(t.width, t.id).toBeGreaterThanOrEqual(44);
    expect(t.height, t.id).toBeGreaterThanOrEqual(44);
  }
  for (const a of targets) {
    for (const b of targets) {
      if (a === b) continue;
      // A segmented pair shares an edge; anything else keeps 8 px clear.
      const segmented =
        (a.id === "You start" || a.id === "AI starts") &&
        a.id !== b.id &&
        (b.id === "You start" || b.id === "AI starts");
      if (segmented) continue;
      const dx = Math.max(
        0,
        Math.max(a.left, b.left) - Math.min(a.left + a.width, b.left + b.width),
      );
      const dy = Math.max(0, Math.max(a.top, b.top) - Math.min(a.top + a.height, b.top + b.height));
      expect(Math.max(dx, dy), `${a.id} and ${b.id}`).toBeGreaterThanOrEqual(8);
    }
  }
}

test("a phone shows one toolbar row with a compact first player button and Undo by the move list", async ({
  page,
}) => {
  await openBoard(page);
  const idle = await toolbarOf(page);
  if (phone()) {
    expect(idle.titleHeight).toBeLessThanOrEqual(34);
    expect(idle.setupShown).toBe(false);
    expect(idle.turnShown).toBe(false);
    expect(idle.compactShown).toBe(true);
    expect(idle.undoInHistoryRow).toBe(true);
    expect(idle.compactTop).toBe(idle.startTop);
    expect(idle.playTop).toBeLessThanOrEqual(160);
  } else {
    expect(idle.setupShown).toBe(true);
    expect(idle.turnShown).toBe(true);
    expect(idle.compactShown).toBe(false);
    expect(idle.undoInHistoryRow).toBe(false);
  }
  expectTargetsApart(idle.targets);
  await shot(page, "phone-toolbar-idle");
  test.info().annotations.push({
    type: "toolbar",
    description: `title ${idle.titleHeight} play top ${idle.playTop} board top ${idle.boardTop} list ${idle.listHeight}`,
  });

  await playExchange(page);
  const played = await toolbarOf(page);
  expect(played.canvas).toEqual(idle.canvas);
  expectTargetsApart(played.targets);
  await expect(page.locator("#undoBtn")).toBeEnabled();
  await shot(page, "phone-toolbar-played");

  await page.locator("#backBtn").click();
  await expect(page.locator("#moveList [aria-current]")).toHaveCount(0);
  expect((await toolbarOf(page)).canvas).toEqual(idle.canvas);
  await shot(page, "phone-toolbar-review");
});

test("the compact button and the pair share one state", async ({ page }) => {
  test.skip(!phone(), "the compact button is for phones");
  await openBoard(page);
  const compact = page.locator("#aiStartsBtn");
  await expect(compact).toHaveAttribute("aria-pressed", "false");
  await compact.click();
  await expect(compact).toHaveAttribute("aria-pressed", "true");
  await expect(page.locator('button[data-first="ai"]')).toHaveAttribute("aria-pressed", "true");
  await expect(page.locator("#status")).toHaveText("AI is thinking", { timeout: 20_000 });
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
  await expect(page.locator("#moveList li")).toHaveText([/^1\. AI, /]);

  // Mid game the choice waits for the next game, and pressing again turns it off.
  await compact.click();
  await expect(compact).toHaveAttribute("aria-pressed", "false");
  await expect(page.locator("#status")).toHaveText("You start next game");
  await expect(page.locator('button[data-first="human"]')).toHaveAttribute("aria-pressed", "true");
});

test("a tall phone gives the saved height to the move list", async ({ page }) => {
  test.skip(test.info().project.name !== "mobile", "a phone height");
  await page.setViewportSize({ width: 390, height: 844 });
  const record = await readFile("fixtures/saved/ten-moves-9n-1000.json", "utf8");
  await page.addInitScript((saved) => localStorage.setItem("sprouts.game", saved), record);
  await page.goto("/");
  await awaitReply(page);
  const bar = await toolbarOf(page);
  expect(bar.playTop).toBeLessThanOrEqual(160);
  expectTargetsApart(bar.targets);
  await shot(page, "phone-toolbar-390");
  test.info().annotations.push({
    type: "toolbar-390",
    description: `title ${bar.titleHeight} play top ${bar.playTop} board top ${bar.boardTop} list ${bar.listHeight}`,
  });
  await startGame(page, 4);
  await expect(page.locator("#status")).toHaveText("Draw a line between two nodes");
  expect((await toolbarOf(page)).canvas).toEqual(bar.canvas);
  await shot(page, "phone-toolbar-390-idle");
});
