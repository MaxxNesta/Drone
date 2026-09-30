import { test, expect, Page } from "@playwright/test";
import AxeBuilder from "@axe-core/playwright";
import { readFile } from "node:fs/promises";
const key = "test-only-dashboard-access-key-000000000000";
async function unlock(page: Page) {
  await page.goto("/");
  await page.getByLabel("Dashboard access key").fill(key);
  await page.getByRole("button", { name: "Open workspace" }).click();
  await expect(
    page.getByRole("heading", { name: "Mission overview" }),
  ).toBeVisible();
}
async function api(page: Page, path: string, body?: unknown) {
  return page.evaluate(
    async ({ path, body }) => {
      const r = await fetch("/api/" + path, {
        method: body === undefined ? "GET" : "POST",
        headers: { "Content-Type": "application/json" },
        body: body === undefined ? undefined : JSON.stringify(body),
      });
      if (!r.ok) throw new Error(await r.text());
      return r.json();
    },
    { path, body },
  );
}
async function finish(page: Page) {
  const s = await api(page, "status");
  if (s.state === "loaded") {
    await api(page, "playback", { run_id: s.run_id, paused: false, speed: 20 });
    await expect
      .poll(async () => (await api(page, "status")).state, { timeout: 60000 })
      .toBe("finished");
  }
}
async function load(page: Page, name: string) {
  await finish(page);
  await page.getByRole("button", { name: "Load mission", exact: true }).click();
  await page.getByLabel("Repository scenario").selectOption(name);
  await page.getByRole("button", { name: "Validate configuration" }).click();
  await expect(page.getByText(/Validated ·/)).toBeVisible();
  await page.getByRole("button", { name: "Load validated mission" }).click();
  await expect(
    page.getByRole("button", { name: "Resume playback", exact: true }),
  ).toBeEnabled();
  return api(page, "status");
}

test("complete survey and browser geometry recovery after actual bridge restart", async ({
  page,
}) => {
  test.setTimeout(120000);
  await unlock(page);
  const status = await load(page, "independent-surveys");
  const before = await api(page, "plan");
  expect(before.config.members).toHaveLength(2);
  const owner = JSON.parse(
    await readFile(".next/stage11-test-owner.json", "utf8"),
  );
  process.kill(owner.pid, "SIGUSR2");
  await expect(page.getByLabel("Dashboard access key")).toBeVisible({
    timeout: 20000,
  });
  await page.getByLabel("Dashboard access key").fill(key);
  await page.getByRole("button", { name: "Open workspace" }).click();
  await expect(page.locator(".map-top h2")).toHaveText("independent surveys");
  expect(await api(page, "plan")).toEqual(before);
  expect((await api(page, "status")).tick).toBe(status.tick);
  await page.getByLabel("Playback speed").selectOption("20");
  await page
    .getByRole("button", { name: "Resume playback", exact: true })
    .click();
  await expect
    .poll(async () => (await api(page, "status")).state, { timeout: 60000 })
    .toBe("finished");
  const snapshot = await api(page, "snapshot");
  expect(snapshot.simulation_truth.tick).toBe(5000);
  expect(snapshot.simulation_truth.summary.all_completed).toBe(true);
  for (const v of Object.values(snapshot.simulation_truth.vehicles) as any[]) {
    expect(v.tick).toBe(5000);
    expect(v.simulation_time_s).toBe(100);
    expect(v.mission_state).toBe("completed");
  }
  await expect(
    page.locator(".map-metrics").getByText("01:40.00", { exact: false }),
  ).toBeVisible();
  await page.getByRole("button", { name: "Recordings" }).click();
  await page
    .getByRole("button", { name: new RegExp(status.run_id.slice(0, 12)) })
    .click();
  await expect(
    page.getByText("All missions complete", { exact: true }),
  ).toBeVisible();
  await page
    .getByRole("button", { name: "Verify replay", exact: true })
    .click();
  await expect(
    page.getByText("Exact deterministic replay verified"),
  ).toBeVisible();
  await page.getByRole("button", { name: "Replay as new run" }).click();
  await expect
    .poll(async () => (await api(page, "status")).run_id)
    .not.toBe(status.run_id);
  await finish(page);
  const replay = await api(page, "snapshot");
  expect(replay.simulation_truth).toEqual(snapshot.simulation_truth);
});

test("battery failure remains isolated and mixed fleet states render accurately", async ({
  page,
}) => {
  await unlock(page);
  await load(page, "one-battery-failure");
  await finish(page);
  const snapshot = await api(page, "snapshot");
  expect(snapshot.simulation_truth.vehicles.alpha.stop_reason).toBe(
    "critical_battery",
  );
  expect(snapshot.simulation_truth.vehicles.bravo.mission_state).toBe(
    "completed",
  );
  await page.getByRole("button", { name: "Select alpha on map" }).click();
  await expect(page.getByText("Stopped: critical battery")).toBeVisible();
  await expect(page.locator(".telemetry-tag")).toHaveText("aborted");
  await page.getByRole("button", { name: "Select bravo on map" }).click();
  await expect(page.locator(".telemetry-tag")).toHaveText("completed");
});

test("crossing advisories and proximity alerts are distinct; keyboard and basic accessibility", async ({
  page,
}) => {
  await unlock(page);
  await load(page, "crossing");
  await expect(page.getByText("Route overlap advisory")).toBeVisible();
  const marker = page.getByRole("button", { name: "Select eastbound on map" });
  await marker.focus();
  await page.keyboard.press("Enter");
  await expect(
    page.getByRole("heading", { name: "eastbound", exact: true }),
  ).toBeVisible();
  const results = await new AxeBuilder({ page })
    .withTags(["wcag2a", "wcag2aa", "wcag21a", "wcag21aa"])
    .analyze();
  await test.info().attach("axe-dashboard.json", {
    body: JSON.stringify(results, null, 2),
    contentType: "application/json",
  });
  expect(results.violations).toEqual([]);
  await page.getByRole("button", { name: "Load mission", exact: true }).focus();
  await page.keyboard.press("Enter");
  await expect(page.getByRole("dialog")).toBeVisible();
  await page.keyboard.press("Tab");
  expect(
    await page.evaluate(
      () => !!document.activeElement?.closest('[role="dialog"]'),
    ),
  ).toBe(true);
  await page.keyboard.press("Escape");
  await expect(
    page.getByRole("button", { name: "Load mission", exact: true }),
  ).toBeFocused();
  for (const width of [1440, 768, 390]) {
    await page.setViewportSize({ width, height: 1000 });
    expect(
      await page.evaluate(
        () => document.documentElement.scrollWidth <= innerWidth,
      ),
    ).toBe(true);
    await page.screenshot({
      path: `test-results/stage11-${width}.png`,
      fullPage: true,
    });
  }
  await page.setViewportSize({ width: 1440, height: 1080 });
  const s = await api(page, "status");
  await api(page, "playback", { run_id: s.run_id, paused: false, speed: 1 });
  await expect(page.getByText("Proximity alert", { exact: true })).toBeVisible({
    timeout: 15000,
  });
  await api(page, "playback", { run_id: s.run_id, paused: true, speed: 1 });
  await expect(page.locator(".alert.route")).toBeVisible();
  await expect(page.locator(".alert.proximity")).toBeVisible();
  await finish(page);
});
