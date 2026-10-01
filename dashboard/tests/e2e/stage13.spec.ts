import { test, expect } from "@playwright/test";
import { readFile, writeFile, unlink } from "node:fs/promises";
import { execFileSync } from "node:child_process";
import path from "node:path";

test("read-only SITL handoff renders through the authenticated backend and bridge", async ({
  page,
}) => {
  const owner = JSON.parse(
    await readFile(".next/stage11-test-owner.json", "utf8"),
  );
  const root = path.resolve("..");
  const python =
    process.env.BACKEND_PYTHON || path.join(root, ".venv-backend/bin/python");
  const publish = (age: string, epoch: string) =>
    execFileSync(
      python,
      [
        "-m",
        "tests.sitl_fixture",
        "--output",
        owner.sitlFixture,
        "--age",
        age,
        "--epoch",
        epoch,
      ],
      { cwd: root },
    );
  await page.goto("/");
  await page
    .getByLabel("Dashboard access key")
    .fill("test-only-dashboard-access-key-000000000000");
  await page.getByRole("button", { name: "Open workspace" }).click();
  await page
    .locator("summary")
    .filter({ hasText: "PX4 / Gazebo telemetry" })
    .click();
  const panel = page.locator(".sitl-panel");
  await expect(
    panel.getByText("Waiting for local SITL collector."),
  ).toBeVisible();
  publish("0", "first-fixture-epoch");
  await expect(
    panel.getByText("Synthetic fixture — not real SITL evidence"),
  ).toBeVisible();
  await expect(
    panel.getByRole("heading", { name: "Gazebo simulation truth" }),
  ).toBeVisible();
  await expect(
    panel.getByRole("heading", { name: "PX4 autopilot estimate" }),
  ).toBeVisible();
  await expect(
    panel.getByText("1.00 / 2.00 / 3.00 m", { exact: true }).first(),
  ).toBeVisible();
  await expect(panel.getByText(/Mission progress unavailable/)).toBeVisible();
  await expect(panel.getByRole("button")).toHaveCount(0);
  publish("10", "first-fixture-epoch");
  await expect(panel.getByText("stale", { exact: true })).toBeVisible();
  publish("0", "restarted-fixture-epoch");
  await expect(panel.getByText(/Epoch restarted-fixture-epoch/)).toBeVisible();
  for (const width of [1440, 390]) {
    await page.setViewportSize({ width, height: 1080 });
    await expect
      .poll(() =>
        page.evaluate(() => document.documentElement.scrollWidth <= innerWidth),
      )
      .toBe(true);
    await panel.screenshot({ path: `test-results/stage13-sitl-${width}.png` });
  }
  await writeFile(owner.sitlFixture, "invalid JSON");
  await expect(
    panel.getByText(/Invalid or unreadable SITL snapshot/),
  ).toBeVisible();
  await expect(
    panel.getByRole("heading", { name: "Gazebo simulation truth" }),
  ).toHaveCount(0);
  await unlink(owner.sitlFixture);
});
