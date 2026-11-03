import "./style.css";
import type { Move, Outcome, Snapshot } from "../pkg/sprouts";
import { Engine, StaleRequest } from "./engine";
import {
  clearSaved,
  dropHash,
  History,
  loadSaved,
  recordFromHash,
  sameGame,
  save,
  shareUrl,
  type GameRecord,
} from "./history";
import { attachInput, type InputOptions, type PointerKind } from "./input";
import { Loupe } from "./loupe";
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
function defaultBoardSize(): number {
  return window.innerWidth <= 768 || window.innerHeight <= 768 ? 500 : 1000;
}
// Matches MIN_PATH_LENGTH in src/types.rs under the engine's board scale.
function minStrokeLength(boardSize: number): number {
  return 20 * (boardSize / 1000);
}
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
const backBtn = byId<HTMLButtonElement>("backBtn");
const forwardBtn = byId<HTMLButtonElement>("forwardBtn");
const shareBtn = byId<HTMLButtonElement>("shareBtn");
const historyEl = byId<HTMLElement>("history");
const moveListEl = byId<HTMLOListElement>("moveList");
const shareUrlEl = byId<HTMLInputElement>("shareUrl");
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

const view = new View(defaultBoardSize());
const loupe = new Loupe();
const inputOptions: InputOptions = { minStrokeLength: minStrokeLength(view.boardSize) };
const state: GameState = watched({ ...freshBoard(), lastInputWasTouch: false }, invalidate);
let timeline = new History(DEFAULT_NODES, view.boardSize);
// A page opened from a share link reviews that game and leaves the saved one alone.
let shared = false;
let lastNodeCount = DEFAULT_NODES;
let thinkingInterval: ReturnType<typeof setInterval> | null = null;
// Bumped by newGame so replies and animations from the previous game stop.
let gameEpoch = 0;
let submitting = false;
let seeking = false;
let booted = false;

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
  updateControls();
}

function stopThinking(): void {
  if (thinkingInterval !== null) {
    clearInterval(thinkingInterval);
    thinkingInterval = null;
  }
  updateControls();
}

function busy(): boolean {
  return state.mode !== "idle" || thinkingInterval !== null || submitting || seeking;
}

function updateControls(): void {
  const idle = !busy();
  undoBtn.disabled = !idle || timeline.atStart;
  backBtn.disabled = !idle || timeline.atStart;
  forwardBtn.disabled = !idle || timeline.atEnd;
  historyEl.hidden = timeline.moves.length === 0 && !state.reviewing;
}

function renderMoveList(): void {
  const items = timeline.moves.map((_, index) => {
    const item = document.createElement("li");
    const button = document.createElement("button");
    button.type = "button";
    button.textContent = timeline.label(index);
    if (index === timeline.cursor - 1) button.setAttribute("aria-current", "step");
    button.addEventListener("click", () => void jumpTo(index + 1));
    item.append(button);
    return item;
  });
  moveListEl.replaceChildren(...items);
  moveListEl.querySelector("[aria-current]")?.scrollIntoView({ block: "nearest" });
  updateControls();
}

function persist(): void {
  if (!shared) save(timeline);
  renderMoveList();
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

function hideLoupe(): void {
  if (loupe.release()) invalidate();
}

function clearStroke(): void {
  hideLoupe();
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
    if (state.gameOver && timeline.atEnd) modal.classList.add("visible");
    return;
  }
  if (state.mode === "placing") {
    if (state.lastInputWasTouch) void handleMove(world, kind);
    return;
  }
  if (state.mode !== "idle" || state.gameOver || seeking) return;
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

function handleAfterMove(world: Point, kind: PointerKind): void {
  if (kind !== "touch" || state.mode !== "dragging") {
    hideLoupe();
    return;
  }
  const start = state.dragPath[0];
  if (start && loupe.update(world, start, state, view)) invalidate();
}

function handleEnd(world: Point, kind: PointerKind): void {
  hideLoupe();
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
  updateControls();

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
  hideLoupe();
  if (state.mode !== "dragging") return;
  state.mode = "idle";
  state.dragPath = [];
}

async function handleTap(point: Point, kind: PointerKind): Promise<void> {
  hideLoupe();
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
  updateControls();
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
    updateControls();
    return;
  }

  recordMove(move, outcome.snapshot);
  clearStroke();
  if (state.gameOver) {
    showGameOver();
  } else {
    startThinking();
    void performAIMove();
  }
}

function recordMove(move: Move, snapshot: Snapshot): void {
  timeline.push(move);
  applySnapshot(snapshot);
  persist();
}

function failure(error: unknown): Outcome {
  return { kind: "rejected", reason: error instanceof Error ? error.message : String(error) };
}

function applySnapshot(snapshot: Snapshot): void {
  state.nodes = snapshot.nodes;
  state.lines = snapshot.lines;
  state.currentPlayer = snapshot.currentPlayer;
  state.activeNodes = new Set(snapshot.activeNodes);
  state.gameOver = snapshot.gameOver;
  updateControls();
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
    updateControls();
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
    updateControls();
  }
}

async function finishAIMove(move: Move, epoch: number): Promise<void> {
  const outcome = await engine.call("applyMove", move).catch(failure);
  if (epoch !== gameEpoch) return;
  state.mode = "idle";
  state.animatingLine = null;
  if (outcome.kind === "applied") {
    recordMove(move, outcome.snapshot);
    if (state.gameOver) showGameOver();
    else setStatus("Your turn!");
  } else {
    setStatus("AI error: " + outcome.reason, "lose");
  }
  updateControls();
}

// Moves the engine to the board after move number target by undoing or
// re-applying stored moves; a stored move the engine now rejects ends the list.
async function seek(target: number): Promise<void> {
  const epoch = gameEpoch;
  seeking = true;
  updateControls();
  try {
    while (timeline.cursor > target) {
      if (!(await engine.call("undo")) || epoch !== gameEpoch) return;
      timeline.cursor--;
    }
    while (timeline.cursor < target) {
      const move = timeline.moves[timeline.cursor];
      if (!move) break;
      const outcome = await engine.call("applyMove", move).catch(failure);
      if (epoch !== gameEpoch) return;
      if (outcome.kind === "rejected") {
        timeline.truncate();
        setStatus(`Replay stopped at move ${timeline.cursor + 1}: ${outcome.reason}`, "lose");
        break;
      }
      timeline.cursor++;
    }
    if (epoch === gameEpoch) await refreshBoard();
  } finally {
    if (epoch === gameEpoch) {
      seeking = false;
      persist();
    }
  }
}

// Review steps one move; a live game steps to the next board where it is
// the human's turn, so an undo takes back the reply as well.
function stepTarget(direction: -1 | 1): number {
  let target = timeline.cursor + direction;
  if (state.reviewing) return target;
  const inRange = (n: number): boolean => n >= 0 && n <= timeline.moves.length;
  if (inRange(target) && timeline.moverOf(target) !== "human" && inRange(target + direction)) {
    target += direction;
  }
  return target;
}

async function jumpTo(target: number): Promise<void> {
  if (busy() || target < 0 || target > timeline.moves.length) return;
  closeModal();
  await seek(target);
  if (state.reviewing) {
    setStatus(reviewStatus());
    return;
  }
  if (!state.gameOver && state.currentPlayer === "ai") {
    startThinking();
    void performAIMove();
  } else if (state.gameOver) {
    showGameOver();
  } else {
    setStatus("Your turn!");
  }
}

function reviewStatus(): string {
  const total = timeline.moves.length;
  const at = timeline.atStart ? "start" : `move ${timeline.cursor} of ${total}`;
  const hint = shared && !timeline.atStart ? "Undo to play on from here." : "";
  return `Reviewing: ${at}. ${hint}`.trim();
}

async function step(direction: -1 | 1): Promise<void> {
  if (busy()) return;
  const target = stepTarget(direction);
  if (target < 0 || target > timeline.moves.length) return;
  await jumpTo(target);
}

// Undo in a live game is back one pair; from a review it returns to play.
async function undoMove(): Promise<void> {
  if (busy() || timeline.atStart) return;
  state.reviewing = false;
  if (shared) {
    shared = false;
    dropHash();
  }
  const target = stepTarget(-1);
  await seek(Math.max(target, 0));
  if (!state.gameOver && state.currentPlayer === "ai") {
    startThinking();
    void performAIMove();
  } else {
    setStatus("Move undone: your turn");
  }
}

async function share(): Promise<void> {
  const url = shareUrl(timeline.record());
  window.history.replaceState(null, "", url);
  try {
    await navigator.clipboard.writeText(url);
    setStatus("Link copied", "win");
  } catch {
    shareUrlEl.hidden = false;
    shareUrlEl.value = url;
    shareUrlEl.focus();
    shareUrlEl.select();
    setStatus("Copy the link below to share this game");
  }
}

function setBoardSize(boardSize: number): void {
  view.setBoardSize(boardSize);
  inputOptions.minStrokeLength = minStrokeLength(boardSize);
}

function resetTable(): void {
  Object.assign(state, freshBoard());
  clearStroke();
  overlay.classList.remove("visible");
  shareUrlEl.hidden = true;
  stopThinking();
  gameEpoch++;
  engine.cancel("turn");
  seeking = false;
  submitting = false;
  closeModal();
}

async function newGame(nodeCount: number): Promise<void> {
  lastNodeCount = nodeCount;
  resetTable();
  shared = false;
  dropHash();
  clearSaved();
  timeline = new History(nodeCount, defaultBoardSize());
  setBoardSize(timeline.boardSize);
  renderMoveList();
  setStatus("Initializing...");
  try {
    await engine.call("init", nodeCount, timeline.boardSize);
    await refreshBoard();
    setStatus("Draw a line between two nodes");
  } catch (error) {
    setStatus(
      "Failed to start game: " + (error instanceof Error ? error.message : String(error)),
      "lose",
    );
  }
}

// Rebuilds a recorded game in the engine; a shared one opens at its last
// move in review, a saved one at the cursor it was left at.
async function restore(record: GameRecord, fromLink: boolean): Promise<void> {
  resetTable();
  shared = fromLink;
  lastNodeCount = record.initialNodes;
  timeline = History.from(record);
  if (fromLink) timeline.cursor = timeline.moves.length;
  const target = timeline.cursor;
  timeline.cursor = 0;
  setBoardSize(timeline.boardSize);
  setStatus("Initializing...");
  await engine.call("init", timeline.initialNodes, timeline.boardSize);
  state.reviewing = fromLink;
  await seek(target);
  if (state.gameOver) state.reviewing = true;
  if (state.reviewing) {
    setStatus(reviewStatus());
  } else if (state.currentPlayer === "ai") {
    startThinking();
    void performAIMove();
  } else {
    setStatus(timeline.atStart ? "Draw a line between two nodes" : "Your turn!");
  }
}

async function boot(): Promise<void> {
  const linked = recordFromHash(location.hash);
  const saved = loadSaved();
  try {
    if (linked && linked.firstPlayer === "human") {
      if (saved && sameGame(saved, linked)) {
        dropHash();
        await restore(saved, false);
      } else {
        await restore(linked, true);
      }
    } else if (saved && saved.firstPlayer === "human") {
      await restore(saved, false);
    } else {
      await newGame(DEFAULT_NODES);
    }
  } catch (error) {
    setStatus(
      "Failed to start game: " + (error instanceof Error ? error.message : String(error)),
      "lose",
    );
  } finally {
    booted = true;
    invalidate();
  }
}

function typingInField(target: EventTarget | null): boolean {
  return target instanceof HTMLInputElement || target instanceof HTMLTextAreaElement;
}

function wireInput(): void {
  attachInput(
    canvas,
    view,
    {
      start: handleStart,
      move: (world, kind) => void handleMove(world, kind),
      afterMove: handleAfterMove,
      end: handleEnd,
      tap: (world, kind) => void handleTap(world, kind),
      cancel: abandonStroke,
    },
    inputOptions,
  );
  canvas.addEventListener("contextmenu", (e) => {
    if (state.mode === "placing") {
      e.preventDefault();
      cancelPlacement();
    }
  });
  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape" && state.mode === "placing") cancelPlacement();
    if (historyEl.hidden || typingInField(e.target) || e.altKey || e.ctrlKey || e.metaKey) return;
    if (e.key === "ArrowLeft") void step(-1);
    if (e.key === "ArrowRight") void step(1);
  });

  for (const button of document.querySelectorAll<HTMLButtonElement>("button[data-nodes]")) {
    button.addEventListener("click", () => void newGame(Number(button.dataset.nodes)));
  }
  undoBtn.addEventListener("click", () => void undoMove());
  backBtn.addEventListener("click", () => void step(-1));
  forwardBtn.addEventListener("click", () => void step(1));
  shareBtn.addEventListener("click", () => void share());
  byId("cancelBtn").addEventListener("click", cancelPlacement);
  byId("confirmBtn").addEventListener("click", () => void confirmMove());
  byId("reviewBtn").addEventListener("click", () => {
    state.reviewing = true;
    closeModal();
    updateControls();
  });
  byId("newGameBtn").addEventListener("click", () => void newGame(lastNodeCount));
}

function frame(): void {
  if (dirty && booted) {
    dirty = false;
    draw(ctx, state, view);
    loupe.draw(ctx, state, view);
  }
  requestAnimationFrame(frame);
}

view.onChange(() => {
  loupe.hide();
  invalidate();
});
trackCanvasSize(canvas, view, invalidate);
wireInput();
wireRecording(state, BOARD_SIZE);
requestAnimationFrame(frame);
void boot();
