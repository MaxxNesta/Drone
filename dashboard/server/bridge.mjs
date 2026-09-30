// Local-only transport. This file is never imported into the browser application.
import { randomBytes, timingSafeEqual } from "node:crypto";
import { readFile } from "node:fs/promises";
import { WebSocket, WebSocketServer } from "ws";

export function equal(a, b) {
  const x = Buffer.from(a),
    y = Buffer.from(b);
  return x.length === y.length && timingSafeEqual(x, y);
}
export function allowedRoute(path, method) {
  const fixed = {
    "/api/status": ["GET", "/v1/status"],
    "/api/snapshot": ["GET", "/v1/snapshot"],
    "/api/validate": ["POST", "/v1/configurations/validate"],
    "/api/runs": ["POST", "/v1/runs"],
    "/api/playback": ["POST", "/v1/playback"],
    "/api/commands": [method === "GET" ? "GET" : "POST", "/v1/commands"],
    "/api/results": ["GET", "/v1/results"],
  };
  if (fixed[path]?.[0] === method) return fixed[path][1];
  const match = path.match(
    /^\/api\/results\/([a-f0-9]{32})(\/(replay|verify-replay))?$/,
  );
  if (match && method === (match[2] ? "POST" : "GET"))
    return "/v1/results/" + match[1] + (match[2] || "");
  return null;
}
export function localRequest(req, port, mutation = false) {
  const address = req.socket.remoteAddress;
  if (!["127.0.0.1", "::1", "::ffff:127.0.0.1"].includes(address)) return false;
  const host = req.headers.host;
  if (![`127.0.0.1:${port}`, `localhost:${port}`].includes(host)) return false;
  const origin = req.headers.origin;
  return mutation
    ? origin === `http://${host}`
    : !origin || origin === `http://${host}`;
}
export async function body(req) {
  let size = 0;
  const chunks = [];
  for await (const part of req) {
    size += part.length;
    if (size > 1024 * 1024) throw new Error("Request exceeds 1 MiB");
    chunks.push(part);
  }
  return Buffer.concat(chunks).toString("utf8");
}
export function createBridge({
  port,
  backend,
  controlToken,
  viewToken,
  accessKey,
}) {
  const url = new URL(backend);
  if (
    url.protocol !== "http:" ||
    url.hostname !== "127.0.0.1" ||
    url.pathname !== "/" ||
    url.search ||
    url.username ||
    url.password ||
    url.hash
  )
    throw new Error("Backend must be an IPv4 loopback HTTP origin");
  if (
    [controlToken, viewToken, accessKey].some(
      (v) => typeof v !== "string" || v.length < 32,
    ) ||
    controlToken === viewToken ||
    accessKey === controlToken ||
    accessKey === viewToken
  )
    throw new Error(
      "Three distinct secrets of at least 32 characters are required",
    );
  backend = url.origin;
  const sessions = new Map(),
    attempts = new Map();
  const sockets = new WebSocketServer({ noServer: true, maxPayload: 1024 });
  function session(req) {
    const value = (req.headers.cookie || "")
      .split(";")
      .map((v) => v.trim())
      .find((v) => v.startsWith("skyview_session="))
      ?.split("=")[1];
    const entry = sessions.get(value);
    if (!entry || entry.expires < Date.now()) {
      sessions.delete(value);
      return null;
    }
    return value;
  }
  function send(res, status, value, headers = {}) {
    res.writeHead(status, {
      "Content-Type": "application/json",
      "Cache-Control": "no-store",
      ...headers,
    });
    res.end(JSON.stringify(value));
  }
  async function upstream(path, method = "GET", data) {
    const token =
      method === "POST" && path !== "/v1/configurations/validate"
        ? controlToken
        : viewToken;
    const response = await fetch(backend + path, {
      method,
      headers: {
        Authorization: `Bearer ${token}`,
        "Content-Type": "application/json",
      },
      body: data,
      signal: AbortSignal.timeout(30000),
      redirect: "error",
    });
    const value = await response.json();
    return { status: response.status, value };
  }
  async function handle(req, res) {
    if (
      !localRequest(req, port, req.method !== "GET" && req.method !== "HEAD")
    ) {
      send(res, 403, { detail: "Local same-origin access required" });
      return true;
    }
    const path = new URL(req.url, `http://${req.headers.host}`).pathname;
    if (!path.startsWith("/api/")) return false;
    try {
      if (path === "/api/session" && req.method === "POST") {
        const peer = req.socket.remoteAddress;
        const now = Date.now();
        const recent = (attempts.get(peer) || []).filter(
          (t) => t > now - 60000,
        );
        attempts.set(peer, recent);
        if (recent.length >= 10) {
          send(res, 429, {
            detail: "Too many sign-in attempts. Retry in one minute.",
          });
          return true;
        }
        recent.push(now);
        const data = JSON.parse(await body(req));
        if (typeof data.key !== "string" || !equal(data.key, accessKey)) {
          send(res, 401, { detail: "Invalid local access key" });
          return true;
        }
        for (const [key, v] of sessions)
          if (v.expires < now) sessions.delete(key);
        if (sessions.size >= 16) {
          send(res, 429, { detail: "Local session limit reached" });
          return true;
        }
        const key = randomBytes(32).toString("hex");
        sessions.set(key, { expires: now + 8 * 60 * 60 * 1000 });
        send(
          res,
          200,
          { authenticated: true },
          {
            "Set-Cookie": `skyview_session=${key}; HttpOnly; SameSite=Strict; Path=/; Max-Age=28800`,
          },
        );
        return true;
      }
      const key = session(req);
      if (!key) {
        send(res, 401, { detail: "Unlock this local dashboard first" });
        return true;
      }
      if (path === "/api/session" && req.method === "GET") {
        send(res, 200, { authenticated: true });
        return true;
      }
      if (path === "/api/session" && req.method === "DELETE") {
        sessions.delete(key);
        for (const client of sockets.clients)
          if (client.sessionKey === key) client.close(1008);
        send(
          res,
          200,
          { authenticated: false },
          {
            "Set-Cookie":
              "skyview_session=; HttpOnly; SameSite=Strict; Path=/; Max-Age=0",
          },
        );
        return true;
      }
      if (path === "/api/presets" && req.method === "GET") {
        const names = [
          "independent-surveys",
          "crossing",
          "one-battery-failure",
        ];
        const presets = await Promise.all(
          names.map(async (name) => ({
            name,
            request: {
              kind: "fleet",
              config: JSON.parse(
                await readFile(
                  new URL(
                    `../../scenarios/stage8/${name}.json`,
                    import.meta.url,
                  ),
                  "utf8",
                ),
              ),
            },
          })),
        );
        send(res, 200, presets);
        return true;
      }
      if (path === "/api/plan" && req.method === "GET") {
        const result = await upstream("/v1/configurations/active");
        send(res, result.status, result.value);
        return true;
      }
      const route = allowedRoute(path, req.method);
      if (!route) {
        send(res, 404, { detail: "Unsupported dashboard endpoint" });
        return true;
      }
      const data = req.method === "POST" ? await body(req) : undefined;
      const result = await upstream(route, req.method, data);
      send(res, result.status, result.value);
      return true;
    } catch (error) {
      send(res, error instanceof SyntaxError ? 400 : 502, {
        detail:
          error instanceof SyntaxError
            ? "Invalid JSON"
            : "Backend unavailable or request failed",
      });
      return true;
    }
  }
  function upgrade(req, socket, head) {
    let u;
    try {
      u = new URL(req.url, `http://${req.headers.host}`);
    } catch {
      socket.destroy();
      return;
    }
    if (u.pathname !== "/telemetry") {
      socket.destroy();
      return;
    }
    const key = session(req);
    if (!localRequest(req, port, true) || !key || sockets.clients.size >= 8) {
      socket.write("HTTP/1.1 403 Forbidden\r\n\r\n");
      socket.destroy();
      return;
    }
    sockets.handleUpgrade(req, socket, head, (client) => {
      client.sessionKey = key;
      const remote = new WebSocket(
        backend.replace("http:", "ws:") + "/v1/telemetry",
        { maxPayload: 2 * 1024 * 1024, handshakeTimeout: 5000 },
      );
      let done = false;
      const close = () => {
        if (done) return;
        done = true;
        clearInterval(expiry);
        remote.close();
        client.close(1013, "Reconnect for a fresh snapshot");
        setTimeout(() => {
          remote.terminate();
          client.terminate();
        }, 2000).unref();
      };
      const expiry = setInterval(() => {
        if (!sessions.has(key) || sessions.get(key).expires < Date.now())
          close();
      }, 5000);
      expiry.unref();
      remote.on("open", () => {
        const auth = { token: viewToken };
        const sequence = u.searchParams.get("after");
        if (
          sequence &&
          /^\d+$/.test(sequence) &&
          Number.isSafeInteger(Number(sequence))
        ) {
          auth.after_sequence = Number(sequence);
          auth.epoch = u.searchParams.get("epoch");
        }
        remote.send(JSON.stringify(auth));
      });
      remote.on("message", (data) => {
        if (client.readyState !== WebSocket.OPEN) return;
        if (client.bufferedAmount > 256 * 1024) {
          close();
          return;
        }
        client.send(data.toString(), (error) => {
          if (error) close();
        });
      });
      client.on("message", () => {
        client.close(1008, "Telemetry is read-only");
        remote.close();
      });
      remote.on("close", close);
      remote.on("error", close);
      client.on("close", close);
      client.on("error", close);
    });
  }
  return {
    handle,
    upgrade,
    close: () => {
      for (const client of sockets.clients) client.close(1001);
      sockets.close();
    },
  };
}
