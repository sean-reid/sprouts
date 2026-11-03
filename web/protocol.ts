export type Request =
  | { type: "init"; nodeCount: number; boardSize: number }
  | { type: "get_state" }
  | { type: "get_active_nodes" }
  | { type: "is_game_over" }
  | { type: "get_ai_move" }
  | { type: "undo" }
  | { type: "get_closest_point"; pathData: number[]; x: number; y: number }
  | { type: "validate_placement"; pathData: number[]; newNodeX: number; newNodeY: number }
  | {
      type: "apply_move";
      fromNode: number;
      toNode: number;
      pathData: number[];
      newNodeX: number;
      newNodeY: number;
    }
  | {
      type: "get_move_error";
      fromNode: number;
      toNode: number;
      pathData: number[];
      newNodeX: number;
      newNodeY: number;
    };

export interface Results {
  init: null;
  get_state: Float64Array;
  get_active_nodes: Float64Array;
  is_game_over: boolean;
  get_ai_move: Float64Array;
  undo: boolean;
  get_closest_point: Float64Array;
  validate_placement: boolean;
  apply_move: boolean;
  get_move_error: string;
}

export type RequestType = Request["type"];
export type RequestOf<K extends RequestType> = Extract<Request, { type: K }>;

export interface Envelope {
  id: number;
  request: Request;
}

export type Reply = { id: number; result: Results[RequestType] } | { id: number; error: string };
