import { test, expect, Page } from "@playwright/test";
const accessKey = "test-only-dashboard-access-key-000000000000";
async function unlock(page: Page) {
  await page.goto("/");
  await page.getByLabel("Dashboard access key").fill(accessKey);
  await page.getByRole("button", { name: "Open workspace" }).click();
  await expect(
    page.getByRole("heading", { name: "Mission overview" }),
  ).toBeVisible();
}
async function request(page: Page, path: string, body?: unknown) {
  return page.evaluate(
    async ({ path, body }) => {
      const r = await fetch("/api/" + path, {
        method: body === undefined ? "GET" : "POST",
        headers: { "Content-Type": "application/json" },
        body: body === undefined ? undefined : JSON.stringify(body),
      });
      return r.json();
    },
    { path, body },
  );
}
test("complete local workflow, command outcomes, errors, recording and replay", async ({
  page,
}) => {
  const errors: string[] = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await unlock(page);
  await expect(page.getByText("No vehicles loaded")).toBeVisible();
  await page.getByRole("button", { name: "Load mission", exact: true }).click();
  await page
    .getByLabel("Repository scenario")
    .selectOption("independent-surveys");
  const input = page.getByLabel("Mission configuration JSON");
  const cfg = JSON.parse(await input.inputValue());
  cfg.config.steps = 400;
  await input.fill(JSON.stringify(cfg));
  await page.getByRole("button", { name: "Validate configuration" }).click();
  await expect(
    page.getByText("Validated · 2 vehicles · 400 ticks"),
  ).toBeVisible();
  await page.getByRole("button", { name: "Load validated mission" }).click();
  await expect(
    page.getByRole("button", { name: "Resume playback", exact: true }),
  ).toBeEnabled();
  const first = cfg.config.members[0].vehicle_id;
  await page.getByRole("button", { name: `Select ${first} on map` }).click();
  await expect(
    page.getByRole("heading", { name: first, exact: true }),
  ).toBeVisible();
  await expect(page.getByText("3D SPEED")).toBeVisible();
  // Targeted resume on an idle vehicle queues successfully but is rejected when applied.
  await page
    .locator(".vehicle-controls")
    .getByRole("button", { name: "Resume", exact: true })
    .click();
  await expect(page.locator(".command-feedback")).toContainText("QUEUED");
  await expect(page.locator(".command-feedback")).toContainText(
    "resume playback to apply",
  );
  const paused = await request(page, "status");
  await page.waitForTimeout(200);
  expect((await request(page, "status")).tick).toBe(paused.tick);
  await page.route("**/api/playback", (route) =>
    route.fulfill({
      status: 409,
      contentType: "application/json",
      body: JSON.stringify({ detail: "Fixture: stale run rejected" }),
    }),
  );
  await page
    .getByRole("button", { name: "Resume playback", exact: true })
    .click();
  await expect(page.locator(".error-banner")).toContainText(
    "Fixture: stale run rejected",
  );
  await page.unroute("**/api/playback");
  await page.getByLabel("Dismiss error").click();
  await page
    .getByRole("button", { name: "Resume playback", exact: true })
    .click();
  await expect(page.locator(".command-feedback")).toContainText("REJECTED");
  await page
    .getByRole("button", { name: "Pause playback", exact: true })
    .click();
  await expect(
    page.getByText("Playback paused", { exact: true }),
  ).toBeVisible();
  await page
    .locator(".vehicle-controls")
    .getByRole("button", { name: "Hold", exact: true })
    .click();
  await expect(page.locator(".command-feedback")).toContainText("QUEUED");
  await page
    .getByRole("button", { name: "Resume playback", exact: true })
    .click();
  await expect(page.locator(".command-feedback")).toContainText("APPLIED");
  await page.route("**/api/playback", (route) =>
    route.fulfill({
      status: 409,
      contentType: "application/json",
      body: JSON.stringify({
        detail: "Fixture: command error while telemetry is live",
      }),
    }),
  );
  await page.getByLabel("Playback speed").selectOption("2");
  await page.waitForTimeout(150);
  await expect(page.locator(".error-banner")).toContainText(
    "Fixture: command error while telemetry is live",
  );
  await page.unroute("**/api/playback");
  await page.getByLabel("Dismiss error").click();
  await page
    .getByRole("button", { name: "Pause playback", exact: true })
    .click();
  const viewer = await page.context().newPage();
  await viewer.goto("/");
  await expect(
    viewer.getByRole("button", { name: `Select ${first} on map` }),
  ).toBeVisible();
  expect((await request(viewer, "status")).tick).toBe(
    (await request(page, "status")).tick,
  );
  await viewer.close();
  await page.screenshot({
    path: "test-results/dashboard-desktop.png",
    fullPage: true,
  });
  await page.setViewportSize({ width: 390, height: 844 });
  await page.screenshot({
    path: "test-results/dashboard-mobile.png",
    fullPage: true,
  });
  expect(
    await page.evaluate(
      () => document.documentElement.scrollWidth <= innerWidth,
    ),
  ).toBe(true);
  await page.setViewportSize({ width: 1440, height: 1080 });
  await page.getByLabel("Playback speed").selectOption("20");
  await page
    .getByRole("button", { name: "Resume playback", exact: true })
    .click();
  await expect
    .poll(async () => (await request(page, "status")).state)
    .toBe("finished");
  await page.getByRole("button", { name: "Recordings" }).click();
  await page
    .getByRole("button", { name: /View results/ })
    .first()
    .click();
  await expect(page.getByText("Not all missions complete")).toBeVisible();
  await page
    .getByRole("button", { name: "Verify replay", exact: true })
    .click();
  await expect(
    page.getByText("Exact deterministic replay verified"),
  ).toBeVisible();
  const old = (await request(page, "status")).run_id;
  await page.getByRole("button", { name: "Replay as new run" }).click();
  await expect
    .poll(async () => (await request(page, "status")).run_id)
    .not.toBe(old);
  await expect(
    page.getByRole("button", { name: "Resume playback", exact: true }),
  ).toBeEnabled();
  await page.getByLabel("Playback speed").selectOption("20");
  await page
    .getByRole("button", { name: "Resume playback", exact: true })
    .click();
  await expect
    .poll(async () => (await request(page, "status")).state)
    .toBe("finished");
  expect(errors).toEqual([]);
  const scripts = await page
    .locator("script[src]")
    .evaluateAll((nodes) => nodes.map((n) => (n as HTMLScriptElement).src));
  for (const src of scripts) {
    const text = await (await page.request.get(src)).text();
    expect(text).not.toContain("test-only-backend-control-secret");
    expect(text).not.toContain("test-only-backend-view-secret");
  }
});
test("reconnect requests a cursor, resync preserves gaps, older samples are ignored", async ({
  page,
}) => {
  await unlock(page);
  if ((await request(page, "status")).state !== "loaded") {
    const presets = await request(page, "presets");
    await request(
      page,
      "runs",
      presets.find((p: { name: string }) => p.name === "crossing").request,
    );
  }
  const snapshot = await request(page, "snapshot");
  let connections = 0;
  let cursor = "";
  await page.routeWebSocket("**/telemetry**", (ws) => {
    connections++;
    cursor = ws.url();
    const sample = structuredClone(snapshot);
    sample.delivery = connections === 1 ? "snapshot" : "resync";
    sample.sequence += connections === 1 ? 0 : 5;
    sample.status.sequence = sample.sequence;
    ws.send(JSON.stringify(sample));
    if (connections === 1) setTimeout(() => ws.close(), 100);
    else {
      const older = structuredClone(sample);
      older.sequence--;
      older.simulation_truth.vehicles[
        Object.keys(older.simulation_truth.vehicles)[0]
      ].vehicle.position_enu_m[0] = 99999;
      setTimeout(() => ws.send(JSON.stringify(older)), 100);
    }
  });
  await page.reload();
  await expect(
    page.getByRole("heading", { name: "Mission overview" }),
  ).toBeVisible();
  await expect(
    page.getByText(
      "Snapshot restored. Missing trajectory samples are shown as gaps.",
    ),
  ).toBeVisible();
  expect(connections).toBeGreaterThanOrEqual(2);
  expect(cursor).toContain("after=");
  await expect(page.locator(".app-footer")).toContainText(
    `SEQ ${snapshot.sequence + 5}`,
  );
  await expect(page.locator(".event-strip")).toContainText(
    "Telemetry resynchronized",
  );
});

test("invalid configuration and disconnected backend produce usable error states", async ({
  page,
}) => {
  await unlock(page);
  await page.getByRole("button", { name: "Load mission", exact: true }).click();
  await page.getByLabel("Mission configuration JSON").fill("{}");
  await page.getByRole("button", { name: "Validate configuration" }).click();
  await expect(page.getByRole("dialog").getByRole("alert")).toBeVisible();
  await expect(
    page.getByRole("button", { name: "Load validated mission" }),
  ).toBeDisabled();
  await page.keyboard.press("Escape");
  await expect(page.getByRole("dialog")).not.toBeVisible();
  await page.route("**/api/status", (route) =>
    route.fulfill({
      status: 503,
      contentType: "application/json",
      body: JSON.stringify({ detail: "Fixture: backend unavailable" }),
    }),
  );
  await expect(page.locator(".error-banner")).toContainText(
    "Fixture: backend unavailable",
  );
  await expect(
    page.getByRole("button", { name: "Resume playback", exact: true }),
  ).toBeDisabled();
  await page.getByRole("button", { name: "Lock workspace" }).click();
  await expect(page.getByLabel("Dashboard access key")).toBeVisible();
});
