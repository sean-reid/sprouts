import type { Cubic, FaceMove, Move, RouteReply } from "../pkg/sprouts";
import { StaleRequest } from "./engine";
import type { GameState } from "./state";
import type { NodeTargets } from "./targets";

export interface Routed {
  mov: Move;
  segments: Cubic[];
}

export interface Table {
  // Whether the human can start a move now.
  canPick(): boolean;
  // Whether a keyboard move in progress may go on: the human's turn on an
  // idle board or with a routed candidate up, nothing else pending.
  canPlay(): boolean;
  // Why neither holds, for the status line.
  whyNot(): string;
  movesFrom(node: number): Promise<FaceMove[]>;
  route(mv: FaceMove): Promise<RouteReply>;
  // Shows the routed curve as the locked stroke with its node dot.
  lock(from: number, to: number, routed: Routed): void;
  commit(): void;
  cancel(): void;
  status(message: string, kind?: "lose"): void;
}

interface Picked {
  start: number;
  moves: FaceMove[];
  partners: Set<number>;
}

interface Shown extends Picked {
  partner: number;
  candidates: FaceMove[];
  index: number;
}

type Phase = { kind: "idle" } | ({ kind: "picked" } & Picked) | ({ kind: "shown" } & Shown);

function touching(moves: FaceMove[], a: number, b: number): FaceMove[] {
  return moves.filter((m) => (m.a === a && m.b === b) || (m.a === b && m.b === a));
}

// A move made from the node targets: pick a start, pick a partner among
// the nodes it can reach, step through the moves between them, and commit
// through the placing flow.
export class KeyboardPlay {
  private phase: Phase = { kind: "idle" };
  private ticket = 0;
  private routing = false;

  constructor(
    private readonly table: Table,
    private readonly state: GameState,
    private readonly targets: NodeTargets,
  ) {}

  get active(): boolean {
    return this.phase.kind !== "idle";
  }

  async activate(id: number): Promise<void> {
    const phase = this.phase;
    if (phase.kind === "idle") {
      if (!this.table.canPick()) {
        this.table.status(this.table.whyNot());
        return;
      }
      const ticket = ++this.ticket;
      let moves: FaceMove[];
      try {
        moves = await this.table.movesFrom(id);
      } catch (error) {
        this.table.status("Engine error: " + describe(error), "lose");
        return;
      }
      if (ticket !== this.ticket || !this.table.canPick()) return;
      if (moves.length === 0) {
        this.table.status(`Node ${id} has no move`);
        return;
      }
      const partners = new Set(moves.map((m) => (m.a === id ? m.b : m.a)));
      this.phase = { kind: "picked", start: id, moves, partners };
      this.state.pickedNode = id;
      this.targets.restrict(partners);
      const loop = partners.has(id) ? ", or Enter again for a loop" : "";
      this.table.status(`Node ${id} picked. Tab to a partner and press Enter${loop}`);
      return;
    }
    if (!phase.partners.has(id) || this.routing) return;
    if (!this.table.canPlay()) {
      this.table.status(this.table.whyNot());
      return;
    }
    if (phase.kind === "shown" && phase.partner === id) {
      this.table.commit();
      return;
    }
    await this.show(phase, id, touching(phase.moves, phase.start, id), 0, 1);
  }

  // Left and Right step through the moves between the pair; both keys are
  // taken while a start is picked.
  cycle(step: -1 | 1): boolean {
    const phase = this.phase;
    if (phase.kind === "idle") return false;
    if (phase.kind === "shown" && phase.candidates.length > 1) {
      void this.show(phase, phase.partner, phase.candidates, phase.index + step, step);
    }
    return true;
  }

  // Escape returns to the idle board and leaves focus where it is.
  escape(): boolean {
    const phase = this.phase;
    if (phase.kind === "idle") return false;
    if (phase.kind === "shown") this.table.cancel();
    else {
      this.reset();
      this.table.status("Move cancelled");
    }
    return true;
  }

  reset(): void {
    this.ticket++;
    this.routing = false;
    this.phase = { kind: "idle" };
    this.state.pickedNode = null;
    this.state.routed = false;
    this.targets.restrict(null);
  }

  // Routes the candidates from `from` in the direction of `step` until one
  // draws, then shows it.
  private async show(
    picked: Picked,
    partner: number,
    candidates: FaceMove[],
    from: number,
    step: -1 | 1,
  ): Promise<void> {
    if (!this.table.canPlay()) return;
    const ticket = ++this.ticket;
    const count = candidates.length;
    for (let k = 0; k < count; k++) {
      const index = (((from + step * k) % count) + count) % count;
      const candidate = candidates[index];
      if (!candidate) break;
      let reply: RouteReply;
      this.routing = true;
      try {
        reply = await this.table.route(candidate);
      } catch (error) {
        if (error instanceof StaleRequest) return;
        this.table.status("Engine error: " + describe(error), "lose");
        return;
      } finally {
        if (ticket === this.ticket) this.routing = false;
      }
      if (ticket !== this.ticket || !this.table.canPlay()) return;
      if (reply.kind !== "routed") continue;
      this.phase = {
        kind: "shown",
        start: picked.start,
        moves: picked.moves,
        partners: picked.partners,
        partner,
        candidates,
        index,
      };
      this.table.lock(picked.start, partner, reply);
      this.table.status(this.describe(picked.start, partner, index, count));
      return;
    }
    if (ticket !== this.ticket) return;
    const pair =
      partner === picked.start ? `node ${partner}` : `nodes ${picked.start} and ${partner}`;
    const which = count === 1 ? "The move" : `None of the ${count} moves`;
    this.table.status(`${which} between ${pair} can be drawn here`, "lose");
  }

  private describe(start: number, partner: number, index: number, count: number): string {
    const shape = start === partner ? `loop on node ${start}` : `node ${start} to node ${partner}`;
    if (count === 1) return `${capital(shape)}. Enter places the node, Escape cancels`;
    return `Move ${index + 1} of ${count}, ${shape}. Enter places the node, Left and Right show another`;
  }
}

// Why the board takes no keyboard move right now; `waiting` is a seek or
// a submission in flight.
export function blockedReason(state: GameState, waiting: boolean): string {
  if (state.thinking || state.mode === "animating_ai" || waiting) return "Wait for your turn";
  if (state.gameOver) return "The game is over. Undo or start a new game";
  if (state.reviewing) return "Undo to play on from here.";
  if (state.mode !== "idle") return "Finish or cancel the move in progress";
  return "Wait for your turn";
}

function capital(text: string): string {
  return text.charAt(0).toUpperCase() + text.slice(1);
}

function describe(error: unknown): string {
  return error instanceof Error ? error.message : String(error);
}
