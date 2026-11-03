import type { SproutsGame } from "../pkg/sprouts";

type GameMethods = {
  [
    K in keyof SproutsGame as K extends "free"
      ? never
      : K extends string
        ? SproutsGame[K] extends (...args: never[]) => unknown
          ? K
          : never
        : never
  ]: SproutsGame[K];
};

// Every public method of the wasm class is a request; init builds the game.
export interface Api extends GameMethods {
  init(nodeCount: number, boardSize: number): void;
}

export type Method = keyof Api;
export type Args<M extends Method> = Parameters<Api[M]>;
export type Result<M extends Method> = ReturnType<Api[M]>;

export type Envelope = { [M in Method]: { id: number; method: M; args: Args<M> } }[Method];

export type Reply =
  { id: number; ok: true; result: unknown } | { id: number; ok: false; error: string };
