import { describe, expect, it } from "vitest";
import { blockedReason } from "../../web/keyboard";
import { freshBoard, type GameState } from "../../web/state";

function board(changes: Partial<GameState> = {}): GameState {
  return { ...freshBoard(), reducedMotion: false, ...changes };
}

describe("blockedReason", () => {
  it("asks for patience while the AI thinks, draws, or a seek runs", () => {
    expect(blockedReason(board({ thinking: true }), false)).toBe("Wait for your turn");
    expect(blockedReason(board({ mode: "animating_ai" }), false)).toBe("Wait for your turn");
    expect(blockedReason(board(), true)).toBe("Wait for your turn");
    expect(blockedReason(board({ currentPlayer: "ai" }), false)).toBe("Wait for your turn");
  });

  it("names game over before review and before a stroke", () => {
    const over = board({ gameOver: true, reviewing: true, mode: "placing" });
    expect(blockedReason(over, false)).toBe("The game is over. Undo or start a new game");
  });

  it("points at Undo in a review", () => {
    expect(blockedReason(board({ reviewing: true }), false)).toBe("Undo to play on from here.");
  });

  it("asks to finish or cancel a stroke in progress", () => {
    const text = "Finish or cancel the move in progress";
    expect(blockedReason(board({ mode: "placing" }), false)).toBe(text);
    expect(blockedReason(board({ mode: "dragging" }), false)).toBe(text);
  });

  it("puts the AI's turn ahead of a stroke", () => {
    expect(blockedReason(board({ thinking: true, mode: "placing" }), false)).toBe(
      "Wait for your turn",
    );
  });
});
