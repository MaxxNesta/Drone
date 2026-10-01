import { test } from "node:test";
import assert from "node:assert/strict";
import { decodeSitl } from "../lib/sitl";
test("disabled and unavailable SITL stay distinct from numerical truth", () => {
  assert.equal(
    decodeSitl({
      configured: false,
      status: "disabled",
      detail: "Not configured",
      telemetry: null,
    }).status,
    "disabled",
  );
  assert.throws(() =>
    decodeSitl({
      configured: true,
      status: "live",
      detail: "",
      telemetry: { evidence: "real_verified" },
    }),
  );
});
test("SITL decoder rejects malformed payloads instead of inventing state", () => {
  for (const data of [
    null,
    {},
    { configured: true, status: "armed", detail: "", telemetry: null },
  ])
    assert.throws(() => decodeSitl(data));
});
