import { test } from "node:test";
import assert from "node:assert/strict";
import http from "node:http";
import { once } from "node:events";
import { WebSocket } from "ws";
import { allowedRoute, createBridge, localRequest } from "../server/bridge.mjs";
const key = "fixture-dashboard-access-key-0000000000",
  control = "fixture-backend-control-token-000000000",
  view = "fixture-backend-view-token-000000000000";
test("only explicit API paths and methods are proxied", () => {
  assert.equal(allowedRoute("/api/status", "GET"), "/v1/status");
  assert.equal(allowedRoute("/api/sitl", "GET"), "/v1/sitl");
  assert.equal(allowedRoute("/api/sitl", "POST"), null);
  for (const [path, method] of [
    ["/api/status", "POST"],
    ["/api/commands", "DELETE"],
    ["/api/../secrets", "GET"],
    ["/api/results/evil/replay", "POST"],
  ])
    assert.equal(allowedRoute(path, method), null);
});
test("loopback, host and origin are all required for commands", () => {
  const req = {
    socket: { remoteAddress: "127.0.0.1" },
    headers: { host: "127.0.0.1:3000", origin: "http://127.0.0.1:3000" },
  };
  assert.equal(localRequest(req, 3000, true), true);
  for (const headers of [
    { host: "evil.test:3000", origin: "http://evil.test:3000" },
    { host: "127.0.0.1:3000", origin: "https://evil.test" },
    { host: "127.0.0.1:3000" },
  ])
    assert.equal(localRequest({ ...req, headers }, 3000, true), false);
  assert.equal(
    localRequest({ ...req, socket: { remoteAddress: "192.168.0.3" } }, 3000),
    false,
  );
});
test("nonlocal upstreams and shared credentials are rejected", () => {
  assert.throws(() =>
    createBridge({
      port: 3000,
      backend: "https://example.com",
      controlToken: control,
      viewToken: view,
      accessKey: key,
    }),
  );
  assert.throws(() =>
    createBridge({
      port: 3000,
      backend: "http://127.0.0.1:8000",
      controlToken: control,
      viewToken: control,
      accessKey: key,
    }),
  );
});
test("sessions protect REST, route tokens server-side, reject cross-origin WS and revoke on logout", async (t) => {
  const seen = [];
  const upstream = http.createServer((req, res) => {
    seen.push({ path: req.url, auth: req.headers.authorization });
    res.setHeader("Content-Type", "application/json");
    res.end(JSON.stringify({ state: "empty", run_id: null }));
  });
  upstream.listen(0, "127.0.0.1");
  await once(upstream, "listening");
  const server = http.createServer();
  server.listen(0, "127.0.0.1");
  await once(server, "listening");
  const port = server.address().port,
    origin = `http://127.0.0.1:${port}`;
  const bridge = createBridge({
    port,
    backend: `http://127.0.0.1:${upstream.address().port}/`,
    controlToken: control,
    viewToken: view,
    accessKey: key,
  });
  server.on("request", (req, res) => bridge.handle(req, res));
  server.on("upgrade", bridge.upgrade);
  t.after(() => {
    bridge.close();
    server.closeAllConnections();
    server.close();
    upstream.closeAllConnections();
    upstream.close();
  });
  assert.equal((await fetch(origin + "/api/status")).status, 401);
  assert.equal(
    (
      await fetch(origin + "/api/session", {
        method: "POST",
        body: JSON.stringify({ key }),
      })
    ).status,
    403,
  );
  const login = await fetch(origin + "/api/session", {
    method: "POST",
    headers: { Origin: origin },
    body: JSON.stringify({ key }),
  });
  assert.equal(login.status, 200);
  const setCookie = login.headers.get("set-cookie");
  assert.match(setCookie, /HttpOnly; SameSite=Strict/);
  const cookie = setCookie.split(";")[0],
    headers = { Origin: origin, Cookie: cookie };
  assert.equal((await fetch(origin + "/api/status", { headers })).status, 200);
  assert.equal(seen.at(-1).auth, `Bearer ${view}`);
  assert.equal(seen.at(-1).path, "/v1/status");
  assert.equal(
    (
      await fetch(origin + "/api/commands", {
        method: "POST",
        headers,
        body: "{}",
      })
    ).status,
    200,
  );
  assert.equal(seen.at(-1).auth, `Bearer ${control}`);
  assert.equal(
    (
      await fetch(origin + "/api/playback", {
        method: "POST",
        headers: { ...headers, Origin: "http://evil.test" },
        body: "{}",
      })
    ).status,
    403,
  );
  const ws = new WebSocket(origin.replace("http:", "ws:") + "/telemetry", {
    headers: { Cookie: cookie, Origin: "http://evil.test" },
  });
  ws.on("error", () => {});
  const rejected = await new Promise((resolve) =>
    ws.on("unexpected-response", (_, res) => {
      resolve(res.statusCode);
      res.destroy();
      ws.terminate();
    }),
  );
  assert.equal(rejected, 403);
  await fetch(origin + "/api/session", { method: "DELETE", headers });
  assert.equal((await fetch(origin + "/api/status", { headers })).status, 401);
});

test("malformed WebSocket upgrade URLs cannot crash the local server", () => {
  const bridge = createBridge({
    port: 3000,
    backend: "http://127.0.0.1:8000",
    controlToken: control,
    viewToken: view,
    accessKey: key,
  });
  let destroyed = false;
  assert.doesNotThrow(() =>
    bridge.upgrade(
      { url: "http://[", headers: { host: "127.0.0.1:3000" } },
      {
        destroy() {
          destroyed = true;
        },
      },
      Buffer.alloc(0),
    ),
  );
  assert.equal(destroyed, true);
  bridge.close();
});
