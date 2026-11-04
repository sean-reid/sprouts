import { describe, expect, it } from "vitest";
import { undrawnStatus } from "../../web/undrawn";

describe("undrawnStatus", () => {
  it("says the AI ran out of time for the engine's limit and the client's", () => {
    const text = "The AI ran out of time drawing its move. Undo or start a new game.";
    expect(undrawnStatus("outOfTime")).toBe(text);
    expect(undrawnStatus("timeout")).toBe(text);
  });

  it("says no move can be drawn when every move was tried or the curve would not read", () => {
    const text = "The AI could not draw a move here. Undo or start a new game.";
    expect(undrawnStatus("exhausted")).toBe(text);
    expect(undrawnStatus("unreadable")).toBe(text);
  });
});
