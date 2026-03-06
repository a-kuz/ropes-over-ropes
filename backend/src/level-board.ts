import { DurableObject } from "cloudflare:workers";

interface TopEntry {
  player_id: string;
  username: string;
  moves: number;
  time_ms: number;
  date: string;
}

interface BoardState {
  total: number;
  distribution: Record<number, number>;
  top50: TopEntry[];
}

export class LevelBoard extends DurableObject<Env> {
  private state: BoardState | null = null;

  private async load(): Promise<BoardState> {
    if (this.state) return this.state;
    const stored = await this.ctx.storage.get<BoardState>("board");
    this.state = stored ?? { total: 0, distribution: {}, top50: [] };
    return this.state;
  }

  private async save() {
    if (this.state) {
      await this.ctx.storage.put("board", this.state);
    }
  }

  async submit(playerId: string, username: string, moves: number, timeMs: number): Promise<Response> {
    const board = await this.load();

    board.total++;
    board.distribution[moves] = (board.distribution[moves] ?? 0) + 1;

    const existingIdx = board.top50.findIndex((e) => e.player_id === playerId);
    let isPersonalBest = false;

    if (existingIdx >= 0) {
      const existing = board.top50[existingIdx];
      if (moves < existing.moves || (moves === existing.moves && timeMs < existing.time_ms)) {
        board.top50[existingIdx] = { player_id: playerId, username, moves, time_ms: timeMs, date: new Date().toISOString() };
        isPersonalBest = true;
      }
    } else {
      const shouldInsert = board.top50.length < 50 || moves <= board.top50[board.top50.length - 1].moves;
      if (shouldInsert) {
        board.top50.push({ player_id: playerId, username, moves, time_ms: timeMs, date: new Date().toISOString() });
        isPersonalBest = true;
      }
    }

    board.top50.sort((a, b) => a.moves - b.moves || a.time_ms - b.time_ms);
    if (board.top50.length > 50) board.top50.length = 50;

    await this.save();

    const playerEntry = board.top50.find((e) => e.player_id === playerId);
    const playerBest = playerEntry?.moves ?? null;

    return Response.json({
      ...this.computeStats(board, playerBest),
      is_personal_best: isPersonalBest,
    });
  }

  async stats(playerId?: string, moves?: number): Promise<Response> {
    const board = await this.load();

    let playerBest: number | null = moves ?? null;
    if (playerBest === null && playerId) {
      const entry = board.top50.find((e) => e.player_id === playerId);
      playerBest = entry?.moves ?? null;
    }

    return Response.json({
      ...this.computeStats(board, playerBest),
      top50: board.top50,
    });
  }

  private computeStats(board: BoardState, playerBest: number | null) {
    if (board.total === 0) {
      return {
        total_completions: 0,
        avg_moves: 0,
        best_moves: 0,
        player_best: null,
        percentile: null,
      };
    }

    let sum = 0;
    let best = Infinity;
    for (const [movesStr, count] of Object.entries(board.distribution)) {
      const m = Number(movesStr);
      sum += m * count;
      if (m < best) best = m;
    }

    const avgMoves = Math.round((sum / board.total) * 10) / 10;

    let percentile: number | null = null;
    if (playerBest !== null) {
      let worseOrEqual = 0;
      for (const [movesStr, count] of Object.entries(board.distribution)) {
        if (Number(movesStr) >= playerBest) worseOrEqual += count;
      }
      percentile = board.total <= 1 ? 100 : Math.round(((worseOrEqual - 1) * 100) / (board.total - 1));
    }

    return {
      total_completions: board.total,
      avg_moves: avgMoves,
      best_moves: best === Infinity ? 0 : best,
      player_best: playerBest,
      percentile,
    };
  }
}

export interface Env {
  LEVEL_BOARD: DurableObjectNamespace<LevelBoard>;
}
