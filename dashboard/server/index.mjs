import http from "node:http";
import next from "next";
import { createBridge } from "./bridge.mjs";
const port = Number(process.env.DASHBOARD_PORT || 3000);
if (!Number.isInteger(port) || port < 1024 || port > 65535)
  throw new Error("Invalid dashboard port");
const bridge = createBridge({
  port,
  backend: process.env.DRONE_BACKEND_URL || "http://127.0.0.1:8000",
  controlToken: process.env.DRONE_CONTROL_TOKEN,
  viewToken: process.env.DRONE_VIEW_TOKEN,
  accessKey: process.env.DASHBOARD_ACCESS_KEY,
});
const dev = process.argv.includes("--dev");
const app = next({ dev, hostname: "127.0.0.1", port });
await app.prepare();
const handler = app.getRequestHandler();
const server = http.createServer(async (req, res) => {
  res.setHeader("X-Content-Type-Options", "nosniff");
  res.setHeader("Referrer-Policy", "no-referrer");
  res.setHeader("X-Frame-Options", "DENY");
  try {
    if (!(await bridge.handle(req, res))) await handler(req, res);
  } catch {
    if (!res.headersSent) res.writeHead(500);
    res.end("Local dashboard error");
  }
});
server.on("upgrade", (req, socket, head) => {
  if (dev && req.url?.startsWith("/_next/webpack-hmr")) {
    socket.destroy();
    return;
  }
  bridge.upgrade(req, socket, head);
});
server.requestTimeout = 10000;
server.headersTimeout = 10000;
server.listen(port, "127.0.0.1", () =>
  console.log(`SKYVIEW local dashboard: http://127.0.0.1:${port}`),
);
const stop = () => {
  bridge.close();
  server.close(() => process.exit(0));
  setTimeout(() => process.exit(0), 3000).unref();
};
process.on("SIGINT", stop);
process.on("SIGTERM", stop);
