// Test-only process owner. Never writes credentials into project files.
import { spawn } from "node:child_process";
import { mkdtemp, rm, mkdir, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import path from "node:path";
import { fileURLToPath } from "node:url";
const root = fileURLToPath(new URL("../../", import.meta.url));
const records = await mkdtemp(path.join(tmpdir(), "drone-dashboard-tests-"));
const env = {
  ...process.env,
  DRONE_CONTROL_TOKEN: "test-only-backend-control-secret-not-for-browser-0000",
  DRONE_VIEW_TOKEN: "test-only-backend-view-secret-not-for-browser-0000000",
  DASHBOARD_ACCESS_KEY: "test-only-dashboard-access-key-000000000000",
  DASHBOARD_PORT: "13000",
  SKYVIEW_SITL_SNAPSHOT: path.join(records, "sitl-snapshot.json"),
  DRONE_BACKEND_URL: "http://127.0.0.1:18000",
};
const children = [];
let stopping = false;
let restartingChild = null;
async function stop(code = 0) {
  if (stopping) return;
  stopping = true;
  for (const child of children) child.kill("SIGTERM");
  await Promise.all(
    children.map(
      (child) =>
        new Promise((resolve) => {
          if (child.exitCode !== null || child.signalCode !== null)
            return resolve();
          child.once("exit", resolve);
          setTimeout(() => {
            child.kill("SIGKILL");
            resolve();
          }, 6000).unref();
        }),
    ),
  );
  await rm(records, { recursive: true, force: true });
  process.exit(code);
}
process.on("SIGTERM", () => stop());
process.on("SIGINT", () => stop());
function start(cmd, args, cwd) {
  const p = spawn(cmd, args, { env, cwd, stdio: "inherit" });
  children.push(p);
  p.on("error", () => stop(1));
  p.on("exit", () => {
    if (!stopping && p !== restartingChild) void stop(1);
  });
  return p;
}
start(
  process.env.BACKEND_PYTHON || path.join(root, ".venv-backend/bin/python"),
  ["-m", "mission_control", "--port", "18000", "--record-dir", records],
  root,
);
let dashboard = start(
  process.execPath,
  ["server/index.mjs"],
  path.join(root, "dashboard"),
);
await mkdir(path.join(root, "dashboard/test-results"), { recursive: true });
await writeFile(
  path.join(root, "dashboard/.next/stage11-test-owner.json"),
  JSON.stringify({ pid: process.pid, sitlFixture: env.SKYVIEW_SITL_SNAPSHOT }),
);
// Test-process IPC only; no production restart endpoint is exposed.
process.on("SIGUSR2", () => {
  if (stopping || restartingChild) return;
  restartingChild = dashboard;
  dashboard.once("exit", () => {
    if (!stopping)
      dashboard = start(
        process.execPath,
        ["server/index.mjs"],
        path.join(root, "dashboard"),
      );
    restartingChild = null;
  });
  dashboard.kill("SIGTERM");
});
