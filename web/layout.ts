// Mirrors opening_layout in src/types.rs: a sunflower spiral whose last
// node sits LAYOUT_REACH of the board from the centre.
const LAYOUT_REACH = 0.35;
const GOLDEN_ANGLE = Math.PI * (3 - Math.sqrt(5));

export type Position = [number, number];

export function openingLayout(count: number, boardSize: number): Position[] {
  const centre = boardSize / 2;
  const unit = (LAYOUT_REACH * boardSize) / Math.sqrt(count - 0.5);
  return Array.from({ length: count }, (_, i) => {
    const radius = unit * Math.sqrt(i + 0.5);
    const angle = i * GOLDEN_ANGLE;
    return [centre + radius * Math.cos(angle), centre + radius * Math.sin(angle)];
  });
}

// The layout of a record or link that carries no positions: a circle of
// radius 120 for up to four nodes, 160 beyond.
export function circleLayout(count: number, boardSize: number): Position[] {
  const radius = ((count <= 4 ? 120 : 160) * boardSize) / 1000;
  return Array.from({ length: count }, (_, i) => {
    const angle = (2 * Math.PI * i) / count;
    return [boardSize / 2 + radius * Math.cos(angle), boardSize / 2 + radius * Math.sin(angle)];
  });
}

// Whether two layouts agree within `tolerance` at every node; a missing
// layout means the default one for that count and board.
export function sameLayout(
  a: Position[] | undefined,
  b: Position[] | undefined,
  count: number,
  boardSize: number,
  tolerance: number,
): boolean {
  const left = a ?? openingLayout(count, boardSize);
  const right = b ?? openingLayout(count, boardSize);
  if (left.length !== right.length) return false;
  return left.every(
    ([x, y], i) =>
      Math.abs(x - right[i]![0]) <= tolerance && Math.abs(y - right[i]![1]) <= tolerance,
  );
}
