import "./style.css";
import type { Move, Outcome, Snapshot } from "../pkg/sprouts";
import { Engine, StaleRequest } from "./engine";
import { healPath } from "./path";
import { draw } from "./render";
import { freshBoard, type GameState, type Node, type PlacementDot, type Point } from "./state";

// A smaller grid on small screens keeps the skeleton pass fast; CSS scaling
// handles display resolution.
const BOARD_SIZE = window.innerWidth <= 768 || window.innerHeight <= 768 ? 500 : 1000;
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

const state: GameState = { ...freshBoard(), lastInputWasTouch: false };
let lastNodeCount = DEFAULT_NODES;
let thinkingInterval: ReturnType<typeof setInterval> | null = null;
let placingModeEnteredAt = 0;
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

function canvasCoords(clientX: number, clientY: number): Point {
  const rect = canvas.getBoundingClientRect();
  return {
    x: (clientX - rect.left) * (canvas.width / rect.width),
    y: (clientY - rect.top) * (canvas.height / rect.height),
  };
}

function findNodeAt(x: number, y: number): Node | null {
  let closest: Node | null = null;
  let minDist = state.lastInputWasTouch ? 30 : 15;
  for (const node of state.nodes) {
    const d = Math.hypot(x - node.position.x, y - node.position.y);
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

function handleStart(clientX: number, clientY: number): void {
  if (state.reviewing) {
    state.reviewing = false;
    modal.classList.add("visible");
    return;
  }
  if (state.mode !== "idle" || state.gameOver) return;
  const { x, y } = canvasCoords(clientX, clientY);
  const node = findNodeAt(x, y);
  if (node && state.activeNodes.has(node.id)) {
    state.mode = "dragging";
    state.dragStartNode = node.id;
    state.dragPath = [{ x: node.position.x, y: node.position.y }];
  }
}

async function handleMove(clientX: number, clientY: number): Promise<void> {
  const point = canvasCoords(clientX, clientY);
  const node = findNodeAt(point.x, point.y);
  state.hoveredNode = node ? node.id : null;

  if (state.mode === "dragging") {
    const last = state.dragPath.at(-1);
    if (last && Math.hypot(point.x - last.x, point.y - last.y) > 5) {
      state.dragPath.push(point);
      state.dragPath = healPath(state.dragPath);
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

function handleEnd(clientX: number, clientY: number): void {
  if (state.mode !== "dragging") return;
  const { x, y } = canvasCoords(clientX, clientY);
  let endNode = findNodeAt(x, y);

  if (!endNode || !state.activeNodes.has(endNode.id)) {
    let bestDist = state.lastInputWasTouch ? 60 : 30;
    let bestNode: Node | null = null;
    for (const node of state.nodes) {
      if (!state.activeNodes.has(node.id)) continue;
      const dist = Math.hypot(x - node.position.x, y - node.position.y);
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
  placingModeEnteredAt = performance.now();

  if (state.lastInputWasTouch) {
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

async function handleDesktopClick(clientX: number, clientY: number): Promise<void> {
  if (state.mode !== "placing" || state.lastInputWasTouch) return;
  // The click that ends the drag gesture lands here too.
  if (performance.now() - placingModeEnteredAt < 300) return;

  const point = canvasCoords(clientX, clientY);
  const spot = await closestValidSpot(point);
  if (!spot) return;
  if (Math.hypot(point.x - spot.position.x, point.y - spot.position.y) < 40 && spot.valid) {
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
  canvas.addEventListener("mousedown", (e) => {
    if (!state.lastInputWasTouch) handleStart(e.clientX, e.clientY);
  });
  canvas.addEventListener("mousemove", (e) => {
    if (!state.lastInputWasTouch) void handleMove(e.clientX, e.clientY);
  });
  canvas.addEventListener("mouseup", (e) => {
    if (!state.lastInputWasTouch) handleEnd(e.clientX, e.clientY);
  });
  canvas.addEventListener("click", (e) => {
    if (!state.lastInputWasTouch) void handleDesktopClick(e.clientX, e.clientY);
  });
  canvas.addEventListener("contextmenu", (e) => {
    if (state.mode === "placing") {
      e.preventDefault();
      cancelPlacement();
    }
  });
  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape" && state.mode === "placing") cancelPlacement();
  });

  canvas.addEventListener(
    "touchstart",
    (e) => {
      state.lastInputWasTouch = true;
      e.preventDefault();
      const touch = e.touches[0];
      if (!touch) return;
      if (state.mode === "placing") void handleMove(touch.clientX, touch.clientY);
      else handleStart(touch.clientX, touch.clientY);
    },
    { passive: false },
  );
  canvas.addEventListener(
    "touchmove",
    (e) => {
      e.preventDefault();
      const touch = e.touches[0];
      if (touch) void handleMove(touch.clientX, touch.clientY);
    },
    { passive: false },
  );
  canvas.addEventListener(
    "touchend",
    (e) => {
      e.preventDefault();
      const touch = e.changedTouches[0];
      if (touch) handleEnd(touch.clientX, touch.clientY);
      // Browsers synthesize mouse events about 300ms after touchend.
      setTimeout(() => {
        state.lastInputWasTouch = false;
      }, 400);
    },
    { passive: false },
  );

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
  draw(ctx, state);
  requestAnimationFrame(frame);
}

canvas.width = BOARD_SIZE;
canvas.height = BOARD_SIZE;
wireInput();
requestAnimationFrame(frame);
void newGame(DEFAULT_NODES);
