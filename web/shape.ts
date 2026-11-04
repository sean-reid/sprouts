// What a move did to the board, for a reader who cannot see it; cutOff is
// the number of nodes the move put on its new side.
export function shapeText(loop: boolean, cutOff: number): string {
  if (loop) return cutOff > 0 ? `loop on one node, encloses ${cutOff}` : "loop on one node";
  return cutOff > 0 ? `joins two nodes, cuts off ${cutOff}` : "joins two nodes";
}
