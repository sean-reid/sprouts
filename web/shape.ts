// What a move did to the board, for a reader who cannot see it; cutOff is
// the number of nodes on the smaller side of the line, which holds from
// either side of a loop. "cuts off N" is one unbreakable piece, so a row
// that wraps breaks at the comma.
export function shapeText(loop: boolean, cutOff: number): string {
  const shape = loop ? "loop on one node" : "joins two nodes";
  return cutOff > 0 ? `${shape}, cuts\u00a0off\u00a0${cutOff}` : shape;
}
