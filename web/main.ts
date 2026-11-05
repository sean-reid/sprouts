import "./style.css";
import type {
  AutoPlacement,
  FaceMove,
  Move,
  Outcome,
  Player,
  Rejection,
  Snapshot,
} from "../pkg/sprouts";
import { Dialog } from "./dialog";
import { CallTimeout, Engine, StaleRequest } from "./engine";
import {
  clearSaved,
  dropHash,
  History,
  loadSaved,
  pendingFace,
  drawn,
  indicesCached,
  isIndexed,
  ownLinkDeferred,
  pendingIndex,
  prepareLink,
  recordFromHash,
  sameGame,
  sameIndexedGame,
  save,
  shareUrl,
  StaleLink,
  toMove,
  type Applied,
  type GameRecord,
} from "./history";
import { attachInput, type InputOptions, type PointerKind } from "./input";
import { blockedReason, KeyboardPlay, type Routed } from "./keyboard";
import { Loupe } from "./loupe";
import { healPath } from "./path";
import { startingNodes, wireRecording } from "./record";
import { draw } from "./render";
import { NodeTargets } from "./targets";
import { undrawnStatus } from "./undrawn";
import {
  freshBoard,
  watched,
  winner,
  type BoardLocation,
  type Cubic,
  type GameState,
  type Geometry,
  type Line,
  type Node,
  type PlacementDot,
  type Point,
} from "./state";
import { trackCanvasSize, View } from "./view";

// World units across the board; the view scales it to the screen.
const BOARD_SIZE = 1000;
// Matches MIN_PATH_LENGTH in src/types.rs under the engine's board scale.
function minStrokeLength(boardSize: number): number {
  return 20 * (boardSize / 1000);
}
const AI_ANIMATION_MS = 500;
// The board eases from the last snapshot to the settled one in this long,
// with canvas input held until it lands.
const SETTLE_MS = 500;
const FLASH_MS = 600;
// How long the board points out the move a seek landed on.
const HIGHLIGHT_MS = 2000;
const DEFAULT_NODES = 4;
// Screen pixels around the placement dot where a press takes hold of it.
const DOT_GRAB_PX = 22;
const PLACING_HINT = "Drag the dot to adjust it, or press anywhere else to place it";
const DRAW_HINT = "Draw a line between two nodes";
// Screen pixels from the canvas edge within which a zoomed stroke gets the pan hint.
const EDGE_HINT_PX = 12;
const PAUSED_HINT = "The line stopped where the board moved it. Pick it up from its end to go on.";
const PAN_HINT: Record<PointerKind, string> = {
  touch: "Pinch or two-finger drag to move the board",
  pen: "Pinch or two-finger drag to move the board",
  mouse: "Hold Space and drag, or scroll, to move the board",
};

type StatusKind = "" | "win" | "lose";

function byId<T extends HTMLElement>(id: string): T {
  const element = document.getElementById(id);
  if (!element) throw new Error(`missing #${id}`);
  return element as T;
}

const canvas = byId<HTMLCanvasElement>("canvas");
const statusEl = byId<HTMLElement>("status");
const undoBtn = byId<HTMLButtonElement>("undoBtn");
const nodeCountEl = byId<HTMLInputElement>("nodeCount");
const fewerBtn = byId<HTMLButtonElement>("fewerBtn");
const moreBtn = byId<HTMLButtonElement>("moreBtn");
const countNote = byId<HTMLElement>("countNote");
const startBtn = byId<HTMLButtonElement>("startBtn");
const startCount = byId<HTMLElement>("startCount");
const turnEl = byId<HTMLElement>("turn");
const turnText = byId<HTMLElement>("turnText");
const firstPlayerButtons = document.querySelectorAll<HTMLButtonElement>("button[data-first]");
const aiStartsBtn = byId<HTMLButtonElement>("aiStartsBtn");
const historyNav = byId<HTMLElement>("historyNav");
const gameControls = byId<HTMLElement>("gameControls");
const phoneQuery = window.matchMedia("(max-width: 767px)");
const backBtn = byId<HTMLButtonElement>("backBtn");
const forwardBtn = byId<HTMLButtonElement>("forwardBtn");
const shareBtn = byId<HTMLButtonElement>("shareBtn");
const historyEl = byId<HTMLElement>("history");
const moveListEl = byId<HTMLOListElement>("moveList");
const shareUrlEl = byId<HTMLInputElement>("shareUrl");
const cancelBtn = byId<HTMLButtonElement>("cancelBtn");
const modal = byId<HTMLElement>("gameOverModal");
const winnerText = byId<HTMLElement>("winnerText");
const boardEl = byId<HTMLElement>("board");
const targetsEl = byId<HTMLElement>("nodeTargets");

const context = canvas.getContext("2d");
if (!context) throw new Error("2d canvas context unavailable");
const ctx = context;

let dirty = true;
function invalidate(): void {
  dirty = true;
}

const view = new View(BOARD_SIZE);
const loupe = new Loupe();
const inputOptions: InputOptions = { minStrokeLength: minStrokeLength(view.boardSize) };
const motionQuery = window.matchMedia("(prefers-reduced-motion: reduce)");
const state: GameState = watched(
  { ...freshBoard(), reducedMotion: motionQuery.matches },
  invalidate,
);
let timeline = new History(DEFAULT_NODES, view.boardSize);
let firstPlayer: Player = "human";
let flashTimer: ReturnType<typeof setTimeout> | null = null;
// The row under the pointer, the row focused from the keyboard, and the
// move the last seek landed on; the board points out the first one set.
let pointedMove: number | null = null;
let focusedMove: number | null = null;
let recentMove: number | null = null;
let highlightTimer: ReturnType<typeof setTimeout> | null = null;
// A page opened from a share link reviews that game and leaves the saved one alone.
let shared = false;
// The board shown after a failed load keeps the saved game for the next move.
let fallback = false;
// A review the player opened, a link, or a finished game stays a review
// until Undo; one entered by stepping onto an AI-to-move board does not.
let pinnedReview = false;
// Bumped whenever the table is reset so replies and animations from the
// previous game stop.
let gameEpoch = 0;
let submitting = false;
let seeking = false;
let booted = false;
// Bumped per settle animation so a frame from an older one stops.
let settleRun = 0;
// Settles once the locked stroke's dot is shown or the stroke is abandoned.
let suggestion: Promise<void> = Promise.resolve();
// Settles once the last drag of the dot has been answered.
let dotUpdate: Promise<void> = Promise.resolve();
// The pan hint shows once per stroke.
let panHinted = false;

const engine = new Engine(() => setStatus("Failed to load game engine", "lose"));
const targets = new NodeTargets(targetsEl, boardEl, state, view, {
  activate: (id) => void keys.activate(id),
  focus: (id) => {
    state.focusedNode = id;
  },
});
const keys = new KeyboardPlay(
  {
    canPick: () => canPlay() && state.mode === "idle",
    canPlay,
    whyNot: () => blockedReason(state, seeking || submitting),
    movesFrom: async (node) => (await engine.call("movesFrom", node)).moves,
    route: (mv) => engine.latest("route", "routeMove", mv),
    lock: lockRouted,
    commit: () => void commitPlacement(),
    cancel: cancelPlacement,
    status: setStatus,
  },
  state,
  targets,
);
const dialog = new Dialog(modal, review);

function describe(error: unknown): string {
  return error instanceof Error ? error.message : String(error);
}

function setStatus(message: string, kind: StatusKind = ""): void {
  statusEl.textContent = message;
  statusEl.className = kind;
}

const TURN_TEXT: Record<Player, string> = { human: "Your turn", ai: "AI's turn" };
const WON_TEXT: Record<Player, string> = { human: "You won", ai: "AI won" };

function updateTurn(): void {
  const won = winner(state);
  const player = won ?? state.currentPlayer;
  turnEl.dataset.player = player;
  turnText.textContent = (won ? WON_TEXT : TURN_TEXT)[player];
}

function flash(at: BoardLocation, path: Point[]): void {
  if (flashTimer !== null) clearTimeout(flashTimer);
  state.flash = { at, path };
  flashTimer = setTimeout(() => {
    state.flash = null;
    flashTimer = null;
  }, FLASH_MS);
}

function showHighlight(): void {
  state.highlightedMove = pointedMove ?? focusedMove ?? recentMove;
}

function pointOutCurrent(): void {
  if (highlightTimer !== null) clearTimeout(highlightTimer);
  recentMove = timeline.cursor > 0 ? timeline.cursor - 1 : null;
  showHighlight();
  highlightTimer = setTimeout(() => {
    recentMove = null;
    highlightTimer = null;
    showHighlight();
  }, HIGHLIGHT_MS);
}

function clearHighlight(): void {
  if (highlightTimer !== null) clearTimeout(highlightTimer);
  highlightTimer = null;
  pointedMove = null;
  focusedMove = null;
  recentMove = null;
  showHighlight();
}

function rowIndex(target: EventTarget | null): number | null {
  const row = target instanceof Element ? target.closest<HTMLElement>("[data-move]") : null;
  return row ? Number(row.dataset.move) : null;
}

function startAITurn(): void {
  state.thinking = true;
  setStatus("AI is thinking");
  updateControls();
  void performAIMove();
}

function stopThinking(): void {
  state.thinking = false;
  updateControls();
}

function busy(): boolean {
  return state.mode !== "idle" || state.thinking || submitting || seeking || sharing;
}

function updateControls(): void {
  const idle = !busy();
  shareBtn.disabled = !idle;
  undoBtn.disabled = !idle || timeline.atStart;
  backBtn.disabled = !idle || timeline.atStart;
  forwardBtn.disabled = !idle || timeline.atEnd;
  historyEl.hidden = timeline.moves.length === 0 && !state.reviewing;
}

function renderMoveList(): void {
  const hadFocus = moveListEl.contains(document.activeElement);
  const items = timeline.moves.map((_, index) => {
    const item = document.createElement("li");
    const button = document.createElement("button");
    button.type = "button";
    const swatch = document.createElement("span");
    swatch.className = "swatch";
    swatch.setAttribute("aria-hidden", "true");
    button.append(swatch, timeline.label(index));
    button.title = timeline.ids(index);
    button.dataset.move = String(index);
    button.dataset.player = timeline.moverOf(index);
    if (index === timeline.cursor - 1) button.setAttribute("aria-current", "step");
    button.addEventListener("click", () => void jumpTo(index + 1));
    item.append(button);
    return item;
  });
  moveListEl.replaceChildren(...items);
  const current = moveListEl.querySelector<HTMLElement>("[aria-current]");
  current?.scrollIntoView({ block: "nearest" });
  if (hadFocus) (current ?? moveListEl.querySelector("button") ?? backBtn).focus();
  updateControls();
}

// A link in the address bar describes the game only until it changes.
function persist(): void {
  if (!shared) {
    save(timeline);
    dropHash();
    fallback = false;
  }
  renderMoveList();
}

function closeModal(): void {
  dialog.hide(boardEl);
}

function review(): void {
  state.reviewing = true;
  pinnedReview = true;
  closeModal();
  updateControls();
}

function showGameOver(): void {
  stopThinking();
  updateTurn();
  const humanWins = winner(state) === "human";
  setStatus(humanWins ? "You win!" : "AI wins", humanWins ? "win" : "lose");
  openDialog();
}

// The heading names the dialog, so it is filled whenever the dialog opens.
function openDialog(): void {
  const humanWins = winner(state) === "human";
  winnerText.textContent = humanWins ? "YOU WIN" : "AI WINS";
  winnerText.style.color = humanWins ? "#10B981" : "#EF4444";
  dialog.show();
}

function hideLoupe(): void {
  if (loupe.release()) invalidate();
}

function clearStroke(): void {
  hideLoupe();
  engine.cancel("lock");
  engine.cancel("placement");
  canvas.classList.remove("grab", "grabbing");
  cancelBtn.hidden = true;
  state.mode = "idle";
  state.dragPath = [];
  state.lockedPath = [];
  state.lockedSegments = null;
  state.placementDot = null;
  keys.reset();
}

// A routed curve takes the place of a locked stroke, its dot already on it.
function lockRouted(from: number, to: number, routed: Routed): void {
  hideLoupe();
  engine.cancel("lock");
  engine.cancel("placement");
  state.dragStartNode = from;
  state.dragEndNode = to;
  state.dragPath = [];
  state.lockedPath = routed.mov.polyline;
  state.lockedSegments = routed.segments;
  state.placementDot = { position: routed.mov.newNodePos, valid: true };
  state.routed = true;
  state.mode = "placing";
  suggestion = Promise.resolve();
  dotUpdate = Promise.resolve();
  cancelBtn.hidden = false;
  updateControls();
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

function overDot(world: Point): boolean {
  const dot = state.placementDot;
  if (state.mode !== "placing" || !dot) return false;
  const a = view.toScreen(dot.position);
  const b = view.toScreen(world);
  return Math.hypot(a.x - b.x, a.y - b.y) <= DOT_GRAB_PX;
}

function handleStart(world: Point, kind: PointerKind): boolean {
  if (state.reviewing) {
    if (state.gameOver && timeline.atEnd) openDialog();
    return false;
  }
  if (busy() || state.settling !== null || state.gameOver) return false;
  const node = findNodeAt(world, kind === "touch" ? 30 : 15);
  if (!node || !state.activeNodes.has(node.id)) return false;
  keys.reset();
  state.mode = "dragging";
  state.dragStartNode = node.id;
  state.dragPath = [{ x: node.position.x, y: node.position.y }];
  panHinted = false;
  return true;
}

function handleMove(point: Point, kind: PointerKind): void {
  if (state.mode === "dragging") {
    const last = state.dragPath.at(-1);
    if (last && Math.hypot(point.x - last.x, point.y - last.y) > screenToWorld(5)) {
      state.dragPath = healPath([...state.dragPath, point]);
    }
  } else if (state.mode === "placing" && kind === "mouse") {
    canvas.classList.toggle("grab", overDot(point));
  }
}

// A stroke at the edge of a zoomed view with more board beyond it.
function strokeAtEdge(world: Point): boolean {
  if (view.zoom <= 1.01) return false;
  const at = view.toScreen(world);
  const board = view.boardSize * view.scale;
  return (
    (at.x < EDGE_HINT_PX && view.tx < -0.5) ||
    (at.y < EDGE_HINT_PX && view.ty < -0.5) ||
    (at.x > view.width - EDGE_HINT_PX && view.tx + board > view.width + 0.5) ||
    (at.y > view.height - EDGE_HINT_PX && view.ty + board > view.height + 0.5)
  );
}

function handleAfterMove(world: Point, kind: PointerKind): void {
  if (state.mode === "dragging" && !panHinted && strokeAtEdge(world)) {
    panHinted = true;
    setStatus(PAN_HINT[kind]);
  }
  if (kind !== "touch" || state.mode !== "dragging") {
    hideLoupe();
    return;
  }
  const start = state.dragPath[0];
  if (start && loupe.update(world, start, state, view)) invalidate();
}

function handleGrab(world: Point): boolean {
  if (submitting || !overDot(world)) return false;
  canvas.classList.remove("grab");
  canvas.classList.add("grabbing");
  return true;
}

function handleDrag(world: Point, kind: PointerKind): void {
  dotUpdate = slideDot(world);
  if (kind === "touch" && loupe.follow(world)) invalidate();
}

function handleDrop(): void {
  canvas.classList.remove("grabbing");
  hideLoupe();
}

// Moves the dot to the point of the curve nearest the pointer. A reply to an
// older position is dropped, so the dot never jumps back.
async function slideDot(at: Point): Promise<void> {
  const segments = state.lockedSegments;
  if (!segments) return;
  let spot: PlacementDot;
  try {
    spot = await engine.latest("placement", "placement", { segments, at });
  } catch (error) {
    if (!(error instanceof StaleRequest)) setStatus("Engine error: " + describe(error), "lose");
    return;
  }
  if (state.mode === "placing" && state.lockedSegments === segments) state.placementDot = spot;
}

function handleEnd(world: Point, kind: PointerKind): void {
  hideLoupe();
  if (state.mode === "placing") {
    void commitPlacement();
    return;
  }
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
  state.lockedSegments = null;
  state.dragEndNode = endNode.id;
  state.mode = "placing";
  cancelBtn.hidden = false;
  updateControls();
  setStatus(PLACING_HINT);
  suggestion = suggestPlacement();
}

// Swaps the raw stroke for the curve it commits as, then shows the best spot
// on it; a curve with no room for a node is rejected here.
async function suggestPlacement(): Promise<void> {
  const path = state.lockedPath;
  let segments: Cubic[];
  let suggested: AutoPlacement;
  try {
    ({ segments } = await engine.latest("lock", "fit", { path }));
    if (state.mode !== "placing" || state.lockedPath !== path) return;
    state.lockedSegments = segments;
    suggested = await engine.latest("lock", "autoPlacement", { segments });
  } catch (error) {
    if (error instanceof StaleRequest) return;
    setStatus("Could not fit the stroke: " + describe(error), "lose");
    clearStroke();
    updateControls();
    return;
  }
  if (state.mode !== "placing" || state.lockedSegments !== segments) return;
  if (suggested.kind === "rejected") rejectStroke(suggested.rejection, path);
  else state.placementDot = suggested.placement;
}

function rejectStroke(rejection: Rejection, path: Point[]): void {
  setStatus(rejection.message, "lose");
  flash(rejection.at, path);
  clearStroke();
  updateControls();
}

function abandonStroke(reason?: "paused"): void {
  hideLoupe();
  if (state.mode !== "dragging") return;
  state.mode = "idle";
  state.dragPath = [];
  if (reason === "paused") setStatus(PAUSED_HINT);
}

function handleTap(): void {
  hideLoupe();
  if (state.mode === "dragging") abandonStroke();
  else if (state.mode === "placing") void commitPlacement();
}

// Waits for the dot and the last drag of it before the move goes in.
async function commitPlacement(): Promise<void> {
  const path = state.lockedPath;
  if (state.mode !== "placing" || submitting) return;
  await suggestion;
  await dotUpdate;
  if (state.mode !== "placing" || state.lockedPath !== path) return;
  await executeMove();
}

function cancelPlacement(): void {
  if (submitting) return;
  clearStroke();
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
  const drawn: Drawn | undefined = state.lockedSegments
    ? { segments: state.lockedSegments, newNodePos: move.newNodePos }
    : undefined;
  cancelBtn.hidden = true;

  const epoch = gameEpoch;
  submitting = true;
  const outcome = await engine.call("applyMove", move).catch(describe);
  submitting = false;
  if (epoch !== gameEpoch) return;
  if (typeof outcome === "string") {
    setStatus("Engine error: " + outcome, "lose");
    clearStroke();
    updateControls();
    return;
  }
  if (outcome.kind === "rejected") {
    rejectStroke(outcome.rejection, move.polyline);
    return;
  }

  recordMove(outcome, drawn);
  clearStroke();
  if (state.gameOver) showGameOver();
  else startAITurn();
}

// The curve and node a move was drawn with, where the settle animation
// starts from for the line the move added.
interface Drawn {
  segments: Cubic[];
  newNodePos: Point;
}

// Records the move as the engine applied it, on its grid with the spline
// it committed as, so a replay sends the engine its own input.
function recordMove(outcome: Applied, drawn?: Drawn, topology?: FaceMove): void {
  timeline.push(outcome.mov, topology, outcome.cutOff);
  applySnapshot(outcome.snapshot, drawn);
  persist();
}

// A keyboard move in progress may go on while it is the human's turn on
// an idle board or with a routed candidate up and nothing else pending.
function canPlay(): boolean {
  const free = state.mode === "idle" || (state.mode === "placing" && state.routed);
  const settled = !state.thinking && !submitting && !seeking && !sharing && state.settling === null;
  return free && settled && !state.gameOver && !state.reviewing && state.currentPlayer === "human";
}

// A board that changes under a stroke or a candidate takes it down with it.
function applySnapshot(snapshot: Snapshot, drawn?: Drawn): void {
  timeline.adopt(snapshot.lines);
  // A snapshot that lands mid-animation eases on from what is on screen.
  const shown = state.settling ?? state;
  const before: Geometry = { nodes: [...shown.nodes], lines: [...shown.lines] };
  const added = snapshot.lines.at(-1);
  if (drawn && added && !before.lines.some((line) => line.id === added.id)) {
    before.lines.push({ ...added, segments: drawn.segments, newNodePos: drawn.newNodePos });
    const node = snapshot.nodes.at(-1);
    if (node) before.nodes.push({ ...node, position: drawn.newNodePos });
  }
  state.nodes = snapshot.nodes;
  state.lines = snapshot.lines;
  animateSettle(before);
  state.currentPlayer = snapshot.currentPlayer;
  state.activeNodes = new Set(snapshot.activeNodes);
  state.gameOver = snapshot.gameOver;
  if (state.mode !== "idle") clearStroke();
  keys.reset();
  targets.sync();
  updateTurn();
  updateControls();
}

function lerp(a: Point, b: Point, t: number): Point {
  return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
}

function samePoint(a: Point, b: Point): boolean {
  return a.x === b.x && a.y === b.y;
}

function sameCubic(a: Cubic, b: Cubic): boolean {
  return samePoint(a.p1, b.p1) && samePoint(a.p2, b.p2) && samePoint(a.p3, b.p3);
}

// Eases the nodes and lines the last snapshot already had from where they
// were to where the settle put them; new ones appear in place. Nothing
// animates when nothing moved or motion is reduced.
function animateSettle(before: Geometry): void {
  const nodes = new Map(before.nodes.map((node) => [node.id, node]));
  const lines = new Map(
    before.lines
      .filter((line) => state.lines.some((now) => now.id === line.id))
      .map((line) => [line.id, line]),
  );
  const moved =
    state.nodes.some((node) => {
      const was = nodes.get(node.id);
      return was !== undefined && !samePoint(was.position, node.position);
    }) ||
    state.lines.some((line) => {
      const was = lines.get(line.id);
      return (
        was !== undefined &&
        was.segments.length === line.segments.length &&
        was.segments.some((c, i) => !sameCubic(c, line.segments[i]!))
      );
    });
  settleRun++;
  if (!moved || state.reducedMotion) {
    state.settling = null;
    canvas.classList.remove("settling");
    return;
  }
  const run = settleRun;
  const epoch = gameEpoch;
  const target = state.nodes;
  const started = performance.now();
  // The board part way from the previous one to the settled one.
  const shown = (eased: number): Geometry => ({
    nodes: state.nodes.map((node) => {
      const was = nodes.get(node.id);
      return was ? { ...node, position: lerp(was.position, node.position, eased) } : node;
    }),
    lines: state.lines.map((line): Line => {
      const was = lines.get(line.id);
      if (!was || was.segments.length !== line.segments.length) return line;
      return {
        ...line,
        newNodePos: lerp(was.newNodePos, line.newNodePos, eased),
        segments: line.segments.map((c, i) => {
          const from = was.segments[i]!;
          return {
            p0: lerp(from.p0, c.p0, eased),
            p1: lerp(from.p1, c.p1, eased),
            p2: lerp(from.p2, c.p2, eased),
            p3: lerp(from.p3, c.p3, eased),
          };
        }),
      };
    }),
  });
  state.settling = shown(0);
  canvas.classList.add("settling");
  const frame = (now: number): void => {
    if (run !== settleRun) return;
    if (epoch !== gameEpoch || state.nodes !== target) {
      state.settling = null;
      canvas.classList.remove("settling");
      return;
    }
    const t = Math.min(1, (now - started) / SETTLE_MS);
    state.settling = shown(1 - (1 - t) * (1 - t));
    if (t < 1) requestAnimationFrame(frame);
    else {
      state.settling = null;
      canvas.classList.remove("settling");
    }
    targets.place();
  };
  requestAnimationFrame(frame);
}

async function refreshBoard(): Promise<void> {
  const epoch = gameEpoch;
  const snapshot = await engine.call("snapshot");
  if (epoch === gameEpoch) applySnapshot(snapshot);
}

// One line per reply on the console, so a report of a stuck or slow AI
// carries what the engine said and how long it took.
function logReply(what: string, since: number): void {
  // eslint-disable-next-line no-console
  console.log(`aiMove: ${what} in ${Math.round(performance.now() - since)} ms`);
}

async function performAIMove(): Promise<void> {
  const epoch = gameEpoch;
  const since = performance.now();
  try {
    const reply = await engine.latest("turn", "aiMove");
    stopThinking();
    logReply(reply.kind === "undrawn" ? `undrawn (${reply.reason})` : reply.kind, since);
    if (reply.kind === "gameOver") {
      state.gameOver = true;
      showGameOver();
      return;
    }
    if (reply.kind === "undrawn") {
      setStatus(undrawnStatus(reply.reason), "lose");
      state.mode = "idle";
      updateControls();
      return;
    }

    state.mode = "animating_ai";
    updateControls();
    state.animatingLine = { segments: reply.segments, newPos: reply.mov.newNodePos };
    if (state.reducedMotion) {
      state.animationProgress = 1;
      void finishAIMove(reply.mov, reply.topology, epoch);
      return;
    }
    state.animationProgress = 0;
    const startTime = performance.now();
    const animate = (now: number): void => {
      if (epoch !== gameEpoch) return;
      state.animationProgress = Math.min((now - startTime) / AI_ANIMATION_MS, 1);
      if (state.animationProgress < 1) requestAnimationFrame(animate);
      else void finishAIMove(reply.mov, reply.topology, epoch);
    };
    requestAnimationFrame(animate);
  } catch (error) {
    if (error instanceof StaleRequest) return;
    stopThinking();
    logReply(error instanceof CallTimeout ? "undrawn (timeout)" : "error", since);
    // A reply that never came leaves the board as an undrawn one does:
    // the engine never applies the AI's move itself, so Undo still works.
    if (error instanceof CallTimeout) setStatus(undrawnStatus("timeout"), "lose");
    else setStatus("AI error: " + describe(error), "lose");
    state.mode = "idle";
    updateControls();
  }
}

// The AI's line is router output, so its face move is kept with it for the
// share link.
async function finishAIMove(move: Move, topology: FaceMove, epoch: number): Promise<void> {
  const outcome = await engine.call("applyMove", move).catch(describe);
  if (epoch !== gameEpoch) return;
  const drawn: Drawn | undefined = state.animatingLine
    ? { segments: state.animatingLine.segments, newNodePos: move.newNodePos }
    : undefined;
  state.mode = "idle";
  state.animatingLine = null;
  if (typeof outcome === "string") {
    setStatus("AI error: " + outcome, "lose");
  } else if (outcome.kind === "rejected") {
    setStatus("AI error: " + outcome.rejection.message, "lose");
  } else {
    recordMove(outcome, drawn, topology);
    if (state.gameOver) showGameOver();
    else setStatus("Your turn!");
  }
  updateControls();
}

// Moves the engine to the board after move number target by undoing or
// re-applying stored moves; a move that arrived as topology is drawn by
// the engine. A stored move the engine now rejects ends the list; the
// result says whether the target was reached in this game.
async function seek(target: number): Promise<boolean> {
  const epoch = gameEpoch;
  const from = timeline.cursor;
  seeking = true;
  updateControls();
  try {
    while (timeline.cursor > target) {
      const undone = await engine.call("undo").catch(() => false);
      if (epoch !== gameEpoch) return false;
      if (!undone) {
        setStatus("Undo failed in the engine", "lose");
        return false;
      }
      timeline.cursor--;
    }
    while (timeline.cursor < target) {
      const move = timeline.moves[timeline.cursor];
      if (!move) break;
      let outcome: Outcome;
      try {
        const face = pendingFace(move);
        const indexed = pendingIndex(move);
        outcome = indexed
          ? await engine.call("applyIndexedMove", indexed)
          : face
            ? await engine.call("applyFaceMove", face, move.newNodePos)
            : await engine.call("applyMove", toMove(move));
      } catch (error) {
        if (epoch === gameEpoch) setStatus("Replay failed: " + describe(error), "lose");
        return false;
      }
      if (epoch !== gameEpoch) return false;
      if (outcome.kind === "rejected") {
        timeline.truncate();
        const { message } = outcome.rejection;
        setStatus(`Replay stopped at move ${timeline.cursor + 1}: ${message}`, "lose");
        return false;
      }
      timeline.applied(timeline.cursor, outcome);
      timeline.cursor++;
    }
    return true;
  } finally {
    if (epoch === gameEpoch) {
      await resyncCursor(epoch).catch(() => setStatus("Lost contact with the engine", "lose"));
      if (epoch === gameEpoch) {
        seeking = false;
        if (timeline.cursor !== from) pointOutCurrent();
        persist();
      }
    }
  }
}

// The engine's line count is the truth after a seek; a move it holds that
// the list no longer has is taken back.
async function resyncCursor(epoch: number): Promise<void> {
  let snapshot = await engine.call("snapshot");
  try {
    while (epoch === gameEpoch && snapshot.lines.length > timeline.moves.length) {
      if (!(await engine.call("undo"))) break;
      snapshot = await engine.call("snapshot");
    }
  } finally {
    if (epoch === gameEpoch) {
      timeline.cursor = Math.min(snapshot.lines.length, timeline.moves.length);
      applySnapshot(snapshot);
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

// After a seek: a shared or finished game stays in review. A live game
// hands the AI its turn at the end of the list and, on an earlier
// AI-to-move board, enters review rather than letting the AI replace the
// moves ahead; reaching the end again makes it live.
function settleAfterSeek(): void {
  if (!pinnedReview && !state.gameOver) {
    state.reviewing = state.currentPlayer === "ai" && !timeline.atEnd;
  }
  if (state.reviewing) {
    setStatus(reviewStatus());
  } else if (!state.gameOver && state.currentPlayer === "ai") {
    startAITurn();
  } else if (state.gameOver) {
    showGameOver();
  } else {
    setStatus("Your turn!");
  }
}

async function jumpTo(target: number): Promise<void> {
  if (busy() || target < 0 || target > timeline.moves.length) return;
  const epoch = gameEpoch;
  closeModal();
  const reached = await seek(target);
  if (epoch !== gameEpoch) return;
  if (reached) settleAfterSeek();
  else if (state.currentPlayer === "ai" && !state.gameOver) state.reviewing = true;
}

function reviewStatus(): string {
  const total = timeline.moves.length;
  const at = timeline.atStart ? "start" : `move ${timeline.cursor} of ${total}`;
  const playable = !state.gameOver && state.currentPlayer === "human" && !timeline.atStart;
  const hint = playable ? "Undo to play on from here." : "";
  return `Reviewing: ${at}. ${hint}`.trim();
}

async function step(direction: -1 | 1): Promise<void> {
  if (busy()) return;
  const target = stepTarget(direction);
  if (target < 0 || target > timeline.moves.length) return;
  await jumpTo(target);
}

// Undo in a live game is back one pair. From a review it returns to play:
// on the spot when it is already the human's turn, else one pair back.
async function undoMove(): Promise<void> {
  if (busy() || timeline.atStart) return;
  const epoch = gameEpoch;
  const wasReviewing = state.reviewing;
  state.reviewing = false;
  shared = false;
  pinnedReview = false;
  if (wasReviewing && !state.gameOver && state.currentPlayer === "human") {
    persist();
    setStatus("Your turn!");
    return;
  }
  const target = stepTarget(-1);
  const reached = await seek(Math.max(target, 0));
  if (epoch !== gameEpoch || !reached) return;
  if (!state.gameOver && state.currentPlayer === "ai") {
    startAITurn();
  } else {
    setStatus("Move undone: your turn");
  }
}

// Copies text that is still being worked out. The clipboard write starts
// inside the click, which is when Safari allows it, and takes the text
// once it is ready; without ClipboardItem the write waits for the text
// and may have lost the activation by then. False whenever the browser
// would not take it, including on an origin with no clipboard at all.
async function copyLater(text: Promise<string>): Promise<boolean> {
  try {
    if (typeof ClipboardItem === "undefined") {
      await navigator.clipboard.writeText(await text);
    } else {
      await navigator.clipboard.write([
        new ClipboardItem({
          "text/plain": text.then((t) => new Blob([t], { type: "text/plain" })),
        }),
      ]);
    }
    return true;
  } catch {
    return false;
  }
}

// A copy of the game is checked and encoded while play goes on; the link
// goes out with a note when a visitor's replay would stop short.
let sharing = false;
async function share(): Promise<void> {
  if (sharing || busy()) return;
  sharing = true;
  setStatus("Checking the link. A long game can take a while.");
  updateControls();
  try {
    const prepared = prepareLink(
      timeline,
      {
        classify: (plan) => engine.call("classify", plan),
        index: (plan) => engine.call("indexMoves", plan),
        rehearse: (plan) => engine.call("rehearse", plan),
      },
      // A review position is not the record's last board, which the rehearsal settles to.
      timeline.atEnd ? state.nodes.map((node) => node.position) : undefined,
    );
    const copied = copyLater(prepared.then(({ history }) => shareUrl(history)));
    const { history, refusedAt, mismatch } = await prepared;
    // The indices the share worked out hold for the timeline's own moves.
    history.moves.forEach((move, i) => {
      const own = timeline.moves[i];
      if (!own || !drawn(own) || own.polyline.length !== move.polyline.length) return;
      if (move.indexed) {
        own.indexed = move.indexed;
        delete own.refused;
      } else if (move.refused) {
        own.refused = true;
        delete own.indexed;
      }
    });
    persist();
    const url = shareUrl(history);
    window.history.replaceState(null, "", url);
    const warning =
      refusedAt !== undefined
        ? ` A visitor's replay may stop at move ${refusedAt + 1}.`
        : mismatch
          ? " A visitor's board may differ from this one."
          : "";
    if (await copied) {
      setStatus("Link copied." + warning, warning ? "" : "win");
    } else {
      shareUrlEl.hidden = false;
      shareUrlEl.value = url;
      shareUrlEl.focus();
      shareUrlEl.select();
      setStatus("Copy the link below to share this game." + warning);
    }
  } catch (error) {
    setStatus("Could not make the link: " + describe(error), "lose");
  } finally {
    sharing = false;
    updateControls();
  }
}

function setBoardSize(boardSize: number): void {
  view.setBoardSize(boardSize);
  inputOptions.minStrokeLength = minStrokeLength(boardSize);
}

function resetTable(): void {
  Object.assign(state, freshBoard());
  clearStroke();
  targets.sync();
  shareUrlEl.hidden = true;
  stopThinking();
  gameEpoch++;
  engine.cancel("turn");
  seeking = false;
  submitting = false;
  clearHighlight();
  closeModal();
}

function clampCount(value: number): number {
  const low = Number(nodeCountEl.min);
  const high = Number(nodeCountEl.max);
  if (!Number.isFinite(value)) return DEFAULT_NODES;
  return Math.min(Math.max(Math.round(value), low), high);
}

// The input's value, rounded and held to its range, written back so the
// field shows what the game got.
function chosenNodeCount(): number {
  const count = clampCount(nodeCountEl.valueAsNumber);
  showNodeCount(count);
  return count;
}

// Below 768 px the count shows as a badge drawn from data-count.
function showNodeCount(count: number): void {
  const active = document.activeElement;
  nodeCountEl.value = String(count);
  fewerBtn.disabled = count <= Number(nodeCountEl.min);
  moreBtn.disabled = count >= Number(nodeCountEl.max);
  const pending = count !== timeline.initialNodes;
  startBtn.classList.toggle("pending", pending);
  startBtn.dataset.count = String(count);
  startCount.textContent = pending ? `, ${count} nodes` : "";
  startBtn.setAttribute("aria-label", pending ? `New game, ${count} nodes` : "New game");
  if (!pending) countNote.textContent = "";
  for (const button of [fewerBtn, moreBtn]) {
    if (button.disabled && active === button) nodeCountEl.focus();
  }
}

// The status line describes the board, so a pending count goes to the note.
function setNodeCount(count: number): void {
  showNodeCount(count);
  const untouched = timeline.moves.length === 0 && !state.reviewing && !busy();
  if (untouched) {
    if (count !== timeline.initialNodes) {
      void newGame(count, !fallback, `${count} nodes. ${DRAW_HINT}`);
    }
  } else {
    countNote.textContent = `Next game: ${count} nodes`;
  }
}

function stepCount(delta: number): void {
  setNodeCount(clampCount(clampCount(nodeCountEl.valueAsNumber) + delta));
}

// A fresh game forgets the saved one and the link; the fallback after a
// failed load keeps both so the next move, not the failure, replaces them.
async function newGame(nodeCount: number, fresh = true, hint = DRAW_HINT): Promise<void> {
  fallback = !fresh;
  resetTable();
  shared = false;
  pinnedReview = false;
  if (fresh) {
    dropHash();
    clearSaved();
  }
  const epoch = gameEpoch;
  timeline = new History(nodeCount, BOARD_SIZE, firstPlayer);
  showNodeCount(nodeCount);
  state.currentPlayer = firstPlayer;
  updateTurn();
  setBoardSize(timeline.boardSize);
  renderMoveList();
  setStatus("Initializing...");
  try {
    await engine.call(
      "init",
      nodeCount,
      timeline.boardSize,
      firstPlayer,
      undefined,
      timeline.settleVersion,
    );
    if (epoch !== gameEpoch) return;
    await refreshBoard();
    if (epoch !== gameEpoch) return;
    timeline.nodes = startingNodes(state.nodes, nodeCount);
    if (state.currentPlayer === "ai") startAITurn();
    else setStatus(hint);
  } catch (error) {
    if (epoch === gameEpoch) setStatus("Failed to start game: " + describe(error), "lose");
  }
}

// Rebuilds a recorded game in the engine; a shared one opens at its last
// move in review, a saved one at the cursor it was left at.
async function restore(record: GameRecord, fromLink: boolean): Promise<void> {
  resetTable();
  const epoch = gameEpoch;
  shared = fromLink;
  pinnedReview = fromLink;
  timeline = History.from(record);
  showNodeCount(timeline.initialNodes);
  if (fromLink) timeline.cursor = timeline.moves.length;
  const target = timeline.cursor;
  timeline.cursor = 0;
  chooseFirstPlayer(timeline.firstPlayer);
  state.currentPlayer = firstPlayer;
  updateTurn();
  setBoardSize(timeline.boardSize);
  setStatus("Initializing...");
  const layout = timeline.nodes ? { nodes: timeline.nodes.map(([x, y]) => ({ x, y })) } : undefined;
  await engine.call(
    "init",
    timeline.initialNodes,
    timeline.boardSize,
    firstPlayer,
    layout,
    timeline.settleVersion,
  );
  if (epoch !== gameEpoch) return;
  state.reviewing = fromLink;
  const reached = await seek(target);
  if (epoch !== gameEpoch) return;
  timeline.nodes ??= startingNodes(state.nodes, timeline.initialNodes);
  if (state.gameOver) {
    state.reviewing = true;
    pinnedReview = true;
  }
  if (!reached) {
    if (state.currentPlayer === "ai" && !state.gameOver) state.reviewing = true;
    return;
  }
  if (!state.reviewing && timeline.atStart && state.currentPlayer === "human") {
    setStatus(DRAW_HINT);
  } else {
    settleAfterSeek();
  }
}

// An idle board with no moves restarts with the choice; otherwise the
// choice waits for the next game.
function pickFirstPlayer(player: Player): void {
  if (player === firstPlayer) return;
  chooseFirstPlayer(player);
  if (busy()) return;
  if (timeline.moves.length === 0) {
    void newGame(chosenNodeCount());
  } else {
    setStatus(player === "human" ? "You start next game" : "AI starts next game");
  }
}

function chooseFirstPlayer(player: Player): void {
  firstPlayer = player;
  for (const button of firstPlayerButtons) {
    button.setAttribute("aria-pressed", String(button.dataset.first === player));
  }
  aiStartsBtn.setAttribute("aria-pressed", String(player === "ai"));
}

// Undo sits with Back, Forward and Share on a phone, where the toolbar is one row.
function placeUndo(): void {
  if (phoneQuery.matches) historyNav.append(undoBtn);
  else gameControls.append(undoBtn);
}

// An index link to the saved game: the cached indices are compared to the
// link's without the engine, and only a record without them is indexed,
// in a worker of its own so the game the page shows is never touched.
// That worker answers every call through a game instance, so one is
// built on the saved record's header first. A failure reads as another
// game, which the link then opens.
async function ownIndexedGame(saved: GameRecord, linked: GameRecord): Promise<boolean> {
  if (!linked.moves.every(isIndexed) || saved.moves.length < linked.moves.length) return false;
  if (indicesCached(saved, linked.moves.length)) {
    return sameIndexedGame(saved, linked, () => Promise.reject(new Error("cached")));
  }
  let checker: Engine | undefined;
  try {
    checker = new Engine(() => {});
    const layout = saved.nodes ? { nodes: saved.nodes.map(([x, y]) => ({ x, y })) } : undefined;
    await checker.call(
      "init",
      saved.initialNodes,
      saved.boardSize,
      saved.firstPlayer,
      layout,
      saved.settleVersion,
    );
    const engine = checker;
    return await sameIndexedGame(saved, linked, (plan) => engine.call("indexMoves", plan));
  } catch {
    return false;
  } finally {
    checker?.dispose();
  }
}

// A long uncached game opens in review first, then the engine indexes the
// saved record while the page is held as during a share; when the indices
// match the link, the saved game resumes live.
async function verifyOwnLink(saved: GameRecord, linked: GameRecord): Promise<void> {
  const epoch = gameEpoch;
  sharing = true;
  setStatus("Checking whether this is your saved game. A long game can take a while.");
  updateControls();
  const own = await ownIndexedGame(saved, linked).finally(() => {
    sharing = false;
    updateControls();
  });
  if (epoch !== gameEpoch) return;
  if (own) {
    dropHash();
    await restore(saved, false);
  } else {
    settleAfterSeek();
  }
}

// A link made for another router opens whatever game is saved, with a
// word about why.
async function boot(): Promise<void> {
  let linked: GameRecord | null = null;
  let stale: StaleLink | null = null;
  try {
    linked = recordFromHash(location.hash);
  } catch (error) {
    if (!(error instanceof StaleLink)) throw error;
    stale = error;
    dropHash();
  }
  const saved = loadSaved();
  const epoch = gameEpoch;
  try {
    if (linked) {
      const deferred = saved !== null && ownLinkDeferred(saved, linked);
      if (
        saved &&
        !deferred &&
        (sameGame(saved, linked) || (await ownIndexedGame(saved, linked)))
      ) {
        dropHash();
        await restore(saved, false);
      } else {
        await restore(linked, true);
        if (saved && deferred && epoch + 1 === gameEpoch) await verifyOwnLink(saved, linked);
      }
    } else if (saved) {
      await restore(saved, false);
    } else {
      await newGame(DEFAULT_NODES);
    }
    // Starting or restoring the table bumped the epoch once.
    if (stale && epoch + 1 === gameEpoch) setStatus(stale.message, "lose");
  } catch (error) {
    // A New Game clicked during the load has already replaced the table.
    if (epoch + 1 !== gameEpoch) return;
    await newGame(DEFAULT_NODES, false);
    setStatus("Could not load the saved game: " + describe(error), "lose");
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
      move: handleMove,
      afterMove: handleAfterMove,
      end: handleEnd,
      tap: handleTap,
      cancel: abandonStroke,
      grab: handleGrab,
      drag: handleDrag,
      drop: handleDrop,
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
    if (e.key === "Escape" && keys.escape()) return;
    if (state.mode === "placing") {
      if (e.key === "Escape") cancelPlacement();
      if (e.key === "Enter" && !(e.target instanceof HTMLButtonElement)) void commitPlacement();
    }
    if (typingInField(e.target) || e.altKey || e.ctrlKey || e.metaKey) return;
    const direction = e.key === "ArrowLeft" ? -1 : e.key === "ArrowRight" ? 1 : null;
    if (direction === null) return;
    if (keys.cycle(direction)) {
      e.preventDefault();
      return;
    }
    if (!historyEl.hidden) void step(direction);
  });

  startBtn.addEventListener("click", () => void newGame(chosenNodeCount()));
  fewerBtn.addEventListener("click", () => stepCount(-1));
  moreBtn.addEventListener("click", () => stepCount(1));
  nodeCountEl.addEventListener("change", () => setNodeCount(chosenNodeCount()));
  nodeCountEl.addEventListener("keydown", (e) => {
    if (e.key === "Enter") void newGame(chosenNodeCount());
  });
  undoBtn.addEventListener("click", () => void undoMove());
  for (const button of firstPlayerButtons) {
    button.addEventListener("click", () => pickFirstPlayer(button.dataset.first as Player));
  }
  aiStartsBtn.addEventListener("click", () => {
    pickFirstPlayer(firstPlayer === "ai" ? "human" : "ai");
  });
  phoneQuery.addEventListener("change", placeUndo);
  placeUndo();
  moveListEl.addEventListener("pointerover", (e) => {
    pointedMove = rowIndex(e.target);
    showHighlight();
  });
  moveListEl.addEventListener("pointerleave", () => {
    pointedMove = null;
    showHighlight();
  });
  // A row reached with Tab shows its move; one focused by a click or tap
  // does not, so the pointer and the seek timer decide instead.
  moveListEl.addEventListener("focusin", (e) => {
    const target = e.target instanceof Element ? e.target : null;
    focusedMove = target?.matches(":focus-visible") ? rowIndex(target) : null;
    showHighlight();
  });
  moveListEl.addEventListener("focusout", (e) => {
    if (e.relatedTarget instanceof Node && moveListEl.contains(e.relatedTarget)) return;
    focusedMove = null;
    showHighlight();
  });
  backBtn.addEventListener("click", () => void step(-1));
  forwardBtn.addEventListener("click", () => void step(1));
  shareBtn.addEventListener("click", () => void share());
  cancelBtn.addEventListener("click", cancelPlacement);
  byId("reviewBtn").addEventListener("click", review);
  byId("newGameBtn").addEventListener("click", () => void newGame(chosenNodeCount()));
}

// The pulse on the ring is the one thing that animates without a state change.
function frame(): void {
  if (booted && (dirty || (state.thinking && !state.reducedMotion))) {
    dirty = false;
    draw(ctx, state, view);
    if (loupe.draw(ctx, state, view)) invalidate();
  }
  requestAnimationFrame(frame);
}

motionQuery.addEventListener("change", () => {
  state.reducedMotion = motionQuery.matches;
});
view.onChange(() => {
  loupe.hide();
  targets.place();
  invalidate();
});
trackCanvasSize(canvas, view, () => {
  targets.place();
  invalidate();
});
wireInput();
// The saved game stops at the board shown, not at a redo tail.
wireRecording(() => {
  const record = timeline.record();
  return { ...record, moves: record.moves.slice(0, record.cursor) };
});
requestAnimationFrame(frame);
void boot();
