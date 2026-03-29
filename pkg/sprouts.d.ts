/* tslint:disable */
/* eslint-disable */

export class SproutsGame {
  free(): void;
  [Symbol.dispose](): void;
  apply_move(from_node: number, to_node: number, path_data: Float64Array, new_node_x: number, new_node_y: number): boolean;
  get_ai_move(): Float64Array;
  is_game_over(): boolean;
  get_move_error(from_node: number, to_node: number, path_data: Float64Array, new_node_x: number, new_node_y: number): string;
  get_active_nodes(): Float64Array;
  validate_placement(path_data: Float64Array, new_node_x: number, new_node_y: number): boolean;
  static new_with_board_size(initial_nodes: number, board_size: number): SproutsGame;
  get_closest_point_on_path(path_data: Float64Array, target_x: number, target_y: number): Float64Array;
  constructor(initial_nodes: number);
  undo(): boolean;
  /**
   * Get current game state as Float64Array.
   * Format: [node_count, ...nodes, line_count, ...lines, current_player]
   */
  get_state(): Float64Array;
}

export type InitInput = RequestInfo | URL | Response | BufferSource | WebAssembly.Module;

export interface InitOutput {
  readonly memory: WebAssembly.Memory;
  readonly __wbg_sproutsgame_free: (a: number, b: number) => void;
  readonly sproutsgame_apply_move: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => number;
  readonly sproutsgame_get_active_nodes: (a: number) => [number, number];
  readonly sproutsgame_get_ai_move: (a: number) => [number, number];
  readonly sproutsgame_get_closest_point_on_path: (a: number, b: number, c: number, d: number, e: number) => [number, number];
  readonly sproutsgame_get_move_error: (a: number, b: number, c: number, d: number, e: number, f: number, g: number) => [number, number];
  readonly sproutsgame_get_state: (a: number) => [number, number];
  readonly sproutsgame_is_game_over: (a: number) => number;
  readonly sproutsgame_new: (a: number) => number;
  readonly sproutsgame_new_with_board_size: (a: number, b: number) => number;
  readonly sproutsgame_undo: (a: number) => number;
  readonly sproutsgame_validate_placement: (a: number, b: number, c: number, d: number, e: number) => number;
  readonly __wbindgen_externrefs: WebAssembly.Table;
  readonly __wbindgen_malloc: (a: number, b: number) => number;
  readonly __wbindgen_free: (a: number, b: number, c: number) => void;
  readonly __wbindgen_start: () => void;
}

export type SyncInitInput = BufferSource | WebAssembly.Module;

/**
* Instantiates the given `module`, which can either be bytes or
* a precompiled `WebAssembly.Module`.
*
* @param {{ module: SyncInitInput }} module - Passing `SyncInitInput` directly is deprecated.
*
* @returns {InitOutput}
*/
export function initSync(module: { module: SyncInitInput } | SyncInitInput): InitOutput;

/**
* If `module_or_path` is {RequestInfo} or {URL}, makes a request and
* for everything else, calls `WebAssembly.instantiate` directly.
*
* @param {{ module_or_path: InitInput | Promise<InitInput> }} module_or_path - Passing `InitInput` directly is deprecated.
*
* @returns {Promise<InitOutput>}
*/
export default function __wbg_init (module_or_path?: { module_or_path: InitInput | Promise<InitInput> } | InitInput | Promise<InitInput>): Promise<InitOutput>;
