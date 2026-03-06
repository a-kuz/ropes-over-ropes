import { Hono } from "hono";
import { cors } from "hono/cors";
import { generateUsername } from "./username";
import { Env, LevelBoard } from "./level-board";

export { LevelBoard };

type Bindings = Env;
const app = new Hono<{ Bindings: Bindings }>();

app.use("/*", cors());

function getLevelStub(env: Env, levelId: number) {
  const id = env.LEVEL_BOARD.idFromName(`level:${levelId}`);
  return env.LEVEL_BOARD.get(id);
}

app.post("/api/player", async (c) => {
  const body = await c.req.json<{ id?: string; username?: string }>();
  const id = body.id || crypto.randomUUID();
  const username = body.username || generateUsername();
  return c.json({ id, username });
});

app.post("/api/submit", async (c) => {
  const body = await c.req.json<{
    player_id: string;
    username: string;
    level_id: number;
    moves: number;
    time_ms: number;
  }>();

  const stub = getLevelStub(c.env, body.level_id);
  return stub.submit(body.player_id, body.username || "anon", body.moves, body.time_ms);
});

app.get("/api/stats/:level_id", async (c) => {
  const levelId = parseInt(c.req.param("level_id"), 10);
  const playerId = c.req.query("player_id");
  const moves = c.req.query("moves") ? parseInt(c.req.query("moves")!, 10) : undefined;
  const stub = getLevelStub(c.env, levelId);
  return stub.stats(playerId, moves);
});

export default app;
