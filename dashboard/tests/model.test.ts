import { test } from "node:test";
import assert from "node:assert/strict";
import {
  decode,
  emptyStream,
  receive,
  fitBounds,
  speed,
  Envelope,
} from "../lib/model";
function frame(sequence = 1): Envelope {
  return {
    schema_version: 1,
    type: "telemetry",
    epoch: "epoch-a",
    sequence,
    run_id: "run-a",
    delivery: "live",
    status: {
      run_id: "run-a",
      epoch: "epoch-a",
      sequence,
      state: "loaded",
      playback_paused: false,
      playback_speed: 1,
      tick: sequence,
      dt_s: 0.02,
      recorded: false,
      error: null,
    },
    simulation_truth: {
      tick: sequence,
      simulation_time_s: sequence * 0.02,
      dt_s: 0.02,
      fleet_id: "test",
      vehicles: {
        alpha: {
          vehicle_id: "alpha",
          mission_state: "running",
          completed_waypoints: 0,
          waypoint_count: 1,
          waypoint_error_m: 3,
          stop_reason: null,
          vehicle: {
            position_enu_m: [sequence, 2, 3],
            velocity_enu_mps: [3, 4, 0],
            acceleration_enu_mps2: [0, 0, 0],
            heading_rad: 0,
            battery_fraction: 0.8,
          },
          events: [{ kind: "started", tick: 1, time_s: 0.02 }],
        },
      },
      summary: {
        completed_waypoints: 0,
        waypoint_count: 1,
        all_completed: false,
        degraded: false,
      },
    },
    advisories: { proximity: [], route_conflicts: [] },
  };
}
test("schema and finite coordinates are checked before rendering", () => {
  assert.equal(decode(frame()).sequence, 1);
  for (const bad of [null, {}, { ...frame(), schema_version: 2 }])
    assert.throws(() => decode(bad));
  const f = frame();
  f.simulation_truth.vehicles.alpha.vehicle.position_enu_m[0] = NaN;
  assert.throws(() => decode(f));
});
test("old and duplicate packets do not append samples or duplicate events", () => {
  const first = receive(emptyStream, frame(2));
  assert.equal(receive(first, frame(1)), first);
  assert.equal(receive(first, frame(2)).trails.alpha.length, 1);
  assert.equal(receive(first, frame(3)).events.length, 1);
});
test("missing sequence and explicit resync break the plotted trajectory", () => {
  const first = receive(emptyStream, frame());
  const gap = receive(first, frame(3));
  assert.equal(gap.trails.alpha[1].gap, true);
  assert.equal(gap.events.at(-1)?.kind, "resync");
  const f = frame(4);
  f.delivery = "resync";
  assert.equal(receive(gap, f).gap, true);
});
test("new run and epoch clear old histories", () => {
  let s = receive(emptyStream, frame(5));
  const f = frame(6);
  f.run_id = "run-b";
  s = receive(s, f);
  assert.equal(s.trails.alpha.length, 1);
  f.epoch = "epoch-b";
  f.sequence = 1;
  assert.equal(receive(s, f).trails.alpha.length, 1);
});
test("client histories remain bounded", () => {
  let s = emptyStream;
  for (let i = 1; i <= 700; i++) {
    const f = frame(i);
    f.simulation_truth.vehicles.alpha.events = [
      { kind: "sample", tick: i, time_s: i * 0.02 },
    ];
    s = receive(s, f);
  }
  assert.equal(s.trails.alpha.length, 600);
  assert.equal(s.events.length, 100);
});
test("map bounds and speed use ENU meters without geographic conversion", () => {
  assert.deepEqual(fitBounds(null, []), { cx: 0, cy: 0, span: 40 });
  const v = frame().simulation_truth.vehicles.alpha;
  assert.equal(speed(v), 5);
  assert.deepEqual(fitBounds(null, [v]), { cx: 1, cy: 2, span: 13 });
});

test("valid vehicle IDs cannot collide with object prototype keys", () => {
  const f = frame();
  f.simulation_truth.vehicles = {
    constructor: {
      ...f.simulation_truth.vehicles.alpha,
      vehicle_id: "constructor",
    },
  };
  const state = receive(emptyStream, f);
  assert.equal(state.trails.constructor.length, 1);
});
