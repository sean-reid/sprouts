import "./style.css";
import type { Move, Outcome, Snapshot } from "../pkg/sprouts";
import { Engine, StaleRequest } from "./engine";
import { attachInput, type PointerKind } from "./input";
import { healPath } from "./path";
import { wireRecording } from "./record";
import { draw } from "./render";
import {
  freshBoard,
  watched,
  type GameState,
  type Node,
  type PlacementDot,
  type Point,
} from "./state";
import { trackCanvasSize, View } from "./view";

// A smaller grid on small screens keeps the skeleton pass fast; the view
// scales it to the screen.
const BOARD_SIZE = window.innerWidth <= 768 || window.innerHeight <= 768 ? 500 : 1000;
// Matches MIN_PATH_LENGTH in src/types.rs under the engine's board scale.
const MIN_STROKE_LENGTH = 20 * (BOARD_SIZE / 1000);
const AI_ANIMATION_MS = 500;
const DEFAULT_NODES = 4;

type StatusKind = "" | "win" | "lose";

function byId<T extends HTMLElement>(id: string): T {
  const element = document.getElementById(id);
  if (!element) throw new Error(`missing #${id}`);
  return element as T;
}

const canvas = byId<HTMLCanvasElement>("canvas");
const statusEl = byId<HTMLElement>("status");
const undoBtn = byId<HTMLButtonElement>("undoBtn");
const overlay = byId<HTMLElement>("mobileOverlay");
const modal = byId<HTMLElement>("gameOverModal");
const winnerText = byId<HTMLElement>("winnerText");

const context = canvas.getContext("2d");
if (!context) throw new Error("2d canvas context unavailable");
const ctx = context;

let dirty = true;
function invalidate(): void {
  dirty = true;
}

const view = new View(BOARD_SIZE);
const state: GameState = watched({ ...freshBoard(), lastInputWasTouch: false }, invalidate);
let lastNodeCount = DEFAULT_NODES;
let thinkingInterval: ReturnType<typeof setInterval> | null = null;
// Bumped by newGame so replies and animations from the previous game stop.
let gameEpoch = 0;
let submitting = false;

const engine = new Engine(() => setStatus("Failed to load game engine", "lose"));

function setStatus(message: string, kind: StatusKind = ""): void {
  statusEl.textContent = message;
  statusEl.className = kind;
}

function startThinking(): void {
  let dots = 0;
  setStatus("AI is thinking");
  thinkingInterval = setInterval(() => {
    dots = (dots + 1) % 4;
    setStatus("AI is thinking" + ".".repeat(dots));
  }, 400);
}

function stopThinking(): void {
  if (thinkingInterval !== null) {
    clearInterval(thinkingInterval);
    thinkingInterval = null;
  }
}

function updateUndoButton(): void {
  undoBtn.disabled = state.lines.length === 0 || state.mode !== "idle";
}

function closeModal(): void {
  modal.classList.remove("visible");
}

function showGameOver(): void {
  stopThinking();
  const humanWins = state.currentPlayer === "ai";
  setStatus(humanWins ? "You win!" : "AI wins", humanWins ? "win" : "lose");
  winnerText.textContent = humanWins ? "YOU WIN" : "AI WINS";
  winnerText.style.color = humanWins ? "#10B981" : "#EF4444";
  modal.classList.add("visible");
}

function clearStroke(): void {
  engine.cancel("placement");
  state.mode = "idle";
  state.dragPath = [];
  state.lockedPath = [];
  state.placementDot = null;
}

// Thresholds are screen pixels; the view turns them into world units.
function screenToWorld(px: number): number {
  return px / view.scale;
}

function findNodeAt(at: Point, screenRadius: number): Node | null {
  let closest: Node | null = null;
  let minDist = screenToWorld(screenRadius);
  for (const node of state.nodes) {
    const d = Math.hypot(at.x - node.position.x, at.y - node.position.y);
    if (d < minDist) {
      minDist = d;
      closest = node;
    }
  }
  return closest;
}

// Resolves null once the stroke it was asked about is no longer on screen.
async function closestValidSpot(at: Point): Promise<PlacementDot | null> {
  const path = state.lockedPath;
  const spot = await engine.call("placement", { path, at });
  return state.mode === "placing" && state.lockedPath === path ? spot : null;
}

function handleStart(world: Point, kind: PointerKind): void {
  state.lastInputWasTouch = kind === "touch";
  if (state.reviewing) {
    state.reviewing = false;
    modal.classList.add("visible");
    return;
  }
  if (state.mode === "placing") {
    if (state.lastInputWasTouch) void handleMove(world, kind);
    return;
  }
  if (state.mode !== "idle" || state.gameOver) return;
  const node = findNodeAt(world, state.lastInputWasTouch ? 30 : 15);
  if (node && state.activeNodes.has(node.id)) {
    state.mode = "dragging";
    state.dragStartNode = node.id;
    state.dragPath = [{ x: node.position.x, y: node.position.y }];
  }
}

async function handleMove(point: Point, kind: PointerKind): Promise<void> {
  state.lastInputWasTouch = kind === "touch";
  if (state.mode === "dragging") {
    const last = state.dragPath.at(-1);
    if (last && Math.hypot(point.x - last.x, point.y - last.y) > screenToWorld(5)) {
      state.dragPath = healPath([...state.dragPath, point]);
    }
  } else if (state.mode === "placing") {
    const snapDist = state.lastInputWasTouch ? 80 : 30;
    const path = state.lockedPath;
    let spot: PlacementDot;
    try {
      spot = await engine.latest("placement", "placement", { path, at: point });
    } catch (error) {
      if (error instanceof StaleRequest) return;
      throw error;
    }
    if (state.mode !== "placing" || state.lockedPath !== path) return;
    const { x, y } = spot.position;
    state.placementDot = Math.hypot(point.x - x, point.y - y) < snapDist ? spot : null;
  }
}

function handleEnd(world: Point, kind: PointerKind): void {
  if (state.mode !== "dragging") return;
  const touch = kind === "touch";
  let endNode = findNodeAt(world, touch ? 30 : 15);

  if (!endNode || !state.activeNodes.has(endNode.id)) {
    let bestDist = screenToWorld(touch ? 60 : 30);
    let bestNode: Node | null = null;
    for (const node of state.nodes) {
      if (!state.activeNodes.has(node.id)) continue;
      const dist = Math.hypot(world.x - node.position.x, world.y - node.position.y);
      if (dist < bestDist) {
        bestDist = dist;
        bestNode = node;
      }
    }
    if (bestNode) endNode = bestNode;
  }

  if (!endNode || !state.activeNodes.has(endNode.id)) {
    state.mode = "idle";
    state.dragPath = [];
    setStatus("Release near a node to connect");
    return;
  }

  state.lockedPath = [...state.dragPath.slice(0, -1), { ...endNode.position }];
  state.dragEndNode = endNode.id;
  state.mode = "placing";

  if (touch) {
    overlay.classList.add("visible");
    setStatus("Tap Confirm to place node, or drag on curve to adjust");
    void autoPlaceMidpoint();
  } else {
    setStatus("Click on the curve to place a new node (ESC to cancel)", "win");
  }
}

async function autoPlaceMidpoint(): Promise<void> {
  const mid = state.lockedPath[Math.floor(state.lockedPath.length / 2)];
  if (state.mode !== "placing" || state.lockedPath.length < 2 || !mid) return;
  const spot = await closestValidSpot(mid);
  if (spot && !state.placementDot) state.placementDot = spot;
}

function abandonStroke(): void {
  if (state.mode !== "dragging") return;
  state.mode = "idle";
  state.dragPath = [];
}

async function handleTap(point: Point, kind: PointerKind): Promise<void> {
  if (state.mode === "dragging") {
    abandonStroke();
    return;
  }
  if (state.mode !== "placing" || kind === "touch") return;
  const spot = await closestValidSpot(point);
  if (!spot) return;
  const snap = screenToWorld(40);
  if (Math.hypot(point.x - spot.position.x, point.y - spot.position.y) < snap && spot.valid) {
    state.placementDot = spot;
    await executeMove();
    return;
  }
  if (state.placementDot?.valid) await executeMove();
}

async function confirmMove(): Promise<void> {
  if (state.mode !== "placing") return;
  if (!state.placementDot?.valid) {
    const mid = state.lockedPath[Math.floor(state.lockedPath.length / 2)] ?? state.lockedPath[0];
    if (mid) {
      const spot = await closestValidSpot(mid);
      if (!spot) return;
      if (spot.valid) state.placementDot = spot;
    }
  }
  await executeMove();
}

function cancelPlacement(): void {
  clearStroke();
  overlay.classList.remove("visible");
  setStatus("Move cancelled");
}

async function executeMove(): Promise<void> {
  if (submitting) return;
  if (!state.placementDot?.valid || state.dragStartNode === null || state.dragEndNode === null) {
    setStatus("Select a valid spot on the line", "lose");
    return;
  }
  const move: Move = {
    fromNode: state.dragStartNode,
    toNode: state.dragEndNode,
    polyline: state.lockedPath,
    newNodePos: state.placementDot.position,
  };
  overlay.classList.remove("visible");

  const epoch = gameEpoch;
  submitting = true;
  const outcome = await engine.call("applyMove", move).catch(failure);
  submitting = false;
  if (epoch !== gameEpoch) return;
  if (outcome.kind === "rejected") {
    setStatus(outcome.reason, "lose");
    clearStroke();
    return;
  }

  applySnapshot(outcome.snapshot);
  clearStroke();
  updateUndoButton();
  if (!state.gameOver) {
    startThinking();
    void performAIMove();
  }
}

function failure(error: unknown): Outcome {
  return { kind: "rejected", reason: error instanceof Error ? error.message : String(error) };
}

function applySnapshot(snapshot: Snapshot): void {
  state.nodes = snapshot.nodes;
  state.lines = snapshot.lines;
  state.currentPlayer = snapshot.currentPlayer;
  state.activeNodes = new Set(snapshot.activeNodes);
  if (snapshot.gameOver) {
    state.gameOver = true;
    showGameOver();
  }
  updateUndoButton();
}

async function refreshBoard(): Promise<void> {
  applySnapshot(await engine.call("snapshot"));
}

async function performAIMove(): Promise<void> {
  const epoch = gameEpoch;
  try {
    const move = await engine.latest("turn", "aiMove");
    stopThinking();
    if (!move) {
      state.gameOver = true;
      showGameOver();
      return;
    }

    state.mode = "animating_ai";
    state.animatingLine = { path: move.polyline, newPos: move.newNodePos };
    state.animationProgress = 0;
    const startTime = performance.now();
    const animate = (now: number): void => {
      if (epoch !== gameEpoch) return;
      state.animationProgress = Math.min((now - startTime) / AI_ANIMATION_MS, 1);
      if (state.animationProgress < 1) requestAnimationFrame(animate);
      else void finishAIMove(move, epoch);
    };
    requestAnimationFrame(animate);
  } catch (error) {
    if (error instanceof StaleRequest) return;
    stopThinking();
    setStatus("AI error: " + (error instanceof Error ? error.message : String(error)), "lose");
    state.mode = "idle";
  }
}

async function finishAIMove(move: Move, epoch: number): Promise<void> {
  const outcome = await engine.call("applyMove", move).catch(failure);
  if (epoch !== gameEpoch) return;
  if (outcome.kind === "applied") {
    applySnapshot(outcome.snapshot);
    if (!state.gameOver) setStatus("Your turn!");
  } else {
    setStatus("AI error: " + outcome.reason, "lose");
  }
  state.mode = "idle";
  state.animatingLine = null;
  updateUndoButton();
}

async function undoMove(): Promise<void> {
  if (state.mode !== "idle" || thinkingInterval !== null) return;
  let undone = false;
  for (let i = 0; i < 2; i++) {
    if (!(await engine.call("undo"))) break;
    undone = true;
  }
  if (undone) {
    state.gameOver = false;
    state.reviewing = false;
    closeModal();
    await refreshBoard();
    setStatus("Move undone: your turn");
  }
}

async function newGame(nodeCount: number): Promise<void> {
  lastNodeCount = nodeCount;
  Object.assign(state, freshBoard());
  view.fit();
  clearStroke();
  overlay.classList.remove("visible");
  stopThinking();
  gameEpoch++;
  engine.cancel("turn");
  setStatus("Initializing...");
  closeModal();
  try {
    await engine.call("init", nodeCount, BOARD_SIZE);
    await refreshBoard();
    setStatus("Draw a line between two nodes");
  } catch (error) {
    setStatus(
      "Failed to start game: " + (error instanceof Error ? error.message : String(error)),
      "lose",
    );
  }
}

function wireInput(): void {
  attachInput(
    canvas,
    view,
    {
      start: handleStart,
      move: (world, kind) => void handleMove(world, kind),
      end: handleEnd,
      tap: (world, kind) => void handleTap(world, kind),
      cancel: abandonStroke,
    },
    { minStrokeLength: MIN_STROKE_LENGTH },
  );
  canvas.addEventListener("contextmenu", (e) => {
    if (state.mode === "placing") {
      e.preventDefault();
      cancelPlacement();
    }
  });
  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape" && state.mode === "placing") cancelPlacement();
  });

  for (const button of document.querySelectorAll<HTMLButtonElement>("button[data-nodes]")) {
    button.addEventListener("click", () => void newGame(Number(button.dataset.nodes)));
  }
  undoBtn.addEventListener("click", () => void undoMove());
  byId("cancelBtn").addEventListener("click", cancelPlacement);
  byId("confirmBtn").addEventListener("click", () => void confirmMove());
  byId("reviewBtn").addEventListener("click", () => {
    state.reviewing = true;
    closeModal();
  });
  byId("newGameBtn").addEventListener("click", () => void newGame(lastNodeCount));
}

function frame(): void {
  if (dirty) {
    dirty = false;
    draw(ctx, state, view);
  }
  requestAnimationFrame(frame);
}

view.onChange(invalidate);
trackCanvasSize(canvas, view, invalidate);
wireInput();
wireRecording(state, BOARD_SIZE);
requestAnimationFrame(frame);
void newGame(DEFAULT_NODES);
