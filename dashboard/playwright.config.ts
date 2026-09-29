import { defineConfig } from "@playwright/test";
export default defineConfig({
  testDir: "./tests/e2e",
  fullyParallel: false,
  workers: 1,
  timeout: 45000,
  expect: { timeout: 10000 },
  use: {
    baseURL: "http://127.0.0.1:13000",
    viewport: { width: 1440, height: 1080 },
    trace: "retain-on-failure",
    screenshot: "only-on-failure",
  },
  webServer: {
    command: "node tests/stack.mjs",
    url: "http://127.0.0.1:13000",
    reuseExistingServer: false,
    timeout: 45000,
  },
  reporter: [["list"], ["html", { open: "never" }]],
});
