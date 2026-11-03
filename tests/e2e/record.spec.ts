import { readFile } from "node:fs/promises";
import { expect, test } from "@playwright/test";
import type { Fixture } from "../../web/record";
import { frameOf, openBoard, playExchange, shot } from "./helpers";

test("Save Game stays hidden without the record flag", async ({ page }) => {
  await openBoard(page);
  await expect(page.locator("#saveBtn")).toHaveCount(0);
});

test("Save Game downloads the played moves as a replay fixture", async ({ page }) => {
  await openBoard(page, "/?record");
  const save = page.locator("#saveBtn");
  await expect(save).toBeVisible();
  const box = await save.boundingBox();
  expect(box?.height ?? 0).toBeGreaterThanOrEqual(44);
  expect(box?.width ?? 0).toBeGreaterThanOrEqual(44);

  await playExchange(page);
  await shot(page, "record-ready");

  const [download] = await Promise.all([page.waitForEvent("download"), save.click()]);
  const { board: boardSize } = await frameOf(page);
  expect(download.suggestedFilename()).toBe(`sprouts-4n-${boardSize}.json`);

  const saved = `test-results/fixtures/${test.info().project.name}-game.json`;
  await download.saveAs(saved);
  const fixture = JSON.parse(await readFile(saved, "utf8")) as Fixture;

  expect(fixture.version).toBe(1);
  expect(fixture.initialNodes).toBe(4);
  expect(fixture.boardSize).toBe(boardSize);
  expect(fixture.moves).toHaveLength(2);
  expect(fixture.moves[0]).toMatchObject({ from: 0, to: 1 });
  for (const move of fixture.moves) {
    expect(move.polyline.length).toBeGreaterThanOrEqual(2);
    for (const [x, y] of [...move.polyline, move.newNode]) {
      expect(x).toBeGreaterThanOrEqual(0);
      expect(x).toBeLessThanOrEqual(boardSize);
      expect(y).toBeGreaterThanOrEqual(0);
      expect(y).toBeLessThanOrEqual(boardSize);
    }
  }
});
