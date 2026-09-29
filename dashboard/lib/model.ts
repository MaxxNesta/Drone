export type Vec3 = [number, number, number];
export interface Vehicle {
  vehicle_id: string;
  mission_state: string;
  completed_waypoints: number;
  waypoint_count: number;
  waypoint_error_m: number | null;
  stop_reason: string | null;
  vehicle: {
    position_enu_m: Vec3;
    velocity_enu_mps: Vec3;
    acceleration_enu_mps2: Vec3;
    heading_rad: number;
    battery_fraction: number;
  };
  events: { kind: string; tick: number; time_s: number; reason?: string }[];
}
export interface Status {
  run_id: string | null;
  epoch: string;
  sequence: number;
  state: string;
  playback_paused: boolean;
  playback_speed: number;
  tick: number | null;
  dt_s: number | null;
  recorded: boolean;
  error: string | null;
}
export interface Advisory {
  vehicle_ids: string[];
  minimum_separation_m?: number;
  minimum_route_separation_m?: number;
}
export interface Envelope {
  schema_version: 1;
  type: "telemetry";
  epoch: string;
  sequence: number;
  run_id: string;
  delivery: string;
  status: Status;
  simulation_truth: {
    tick: number;
    simulation_time_s: number;
    dt_s: number;
    fleet_id: string;
    vehicles: Record<string, Vehicle>;
    summary: {
      completed_waypoints: number;
      waypoint_count: number;
      all_completed: boolean;
      degraded: boolean;
    };
  };
  advisories: { proximity: Advisory[]; route_conflicts: Advisory[] };
}
export interface Mission {
  mission_id: string;
  home_enu_m: Vec3;
  start_enu_m: Vec3;
  survey_area: [number, number][] | null;
  flight_boundary: [number, number][];
  waypoints: { position_enu_m: Vec3; leg_kind: string }[];
}
export interface FleetConfig {
  fleet_id: string;
  steps: number;
  members: { vehicle_id: string; mission: Mission }[];
}
export interface Outcome {
  request_id: string | null;
  tick: number;
  command: string;
  vehicle_id: string | null;
  accepted: boolean;
  reason?: string;
  affected_vehicle_ids?: string[];
}
export interface Ack {
  request_id: string;
  target_tick: number;
  command: string;
  vehicle_id?: string;
  run_id: string;
}
export interface TrailPoint {
  position: Vec3;
  gap: boolean;
}
export interface EventRow {
  id: string;
  time: number;
  text: string;
  kind: string;
}
export interface StreamState {
  envelope: Envelope | null;
  trails: Record<string, TrailPoint[]>;
  events: EventRow[];
  gap: boolean;
}
export const emptyStream: StreamState = {
  envelope: null,
  trails: {},
  events: [],
  gap: false,
};
const vec = (v: unknown): v is Vec3 =>
  Array.isArray(v) &&
  v.length === 3 &&
  v.every((n) => typeof n === "number" && Number.isFinite(n));
export function decode(value: unknown): Envelope {
  const v = value as Envelope;
  if (
    !v ||
    v.schema_version !== 1 ||
    v.type !== "telemetry" ||
    !Number.isSafeInteger(v.sequence) ||
    v.sequence < 1 ||
    typeof v.epoch !== "string" ||
    typeof v.run_id !== "string" ||
    !v.status ||
    !v.simulation_truth?.vehicles ||
    !Array.isArray(v.advisories?.proximity) ||
    !Array.isArray(v.advisories?.route_conflicts)
  )
    throw new Error("Unsupported telemetry schema");
  if (
    !Number.isFinite(v.simulation_truth.simulation_time_s) ||
    !Number.isSafeInteger(v.simulation_truth.tick)
  )
    throw new Error("Invalid simulation clock");
  const summary = v.simulation_truth.summary;
  if (
    !summary ||
    !Number.isSafeInteger(summary.completed_waypoints) ||
    !Number.isSafeInteger(summary.waypoint_count) ||
    !["live", "snapshot", "replay", "resync"].includes(v.delivery)
  )
    throw new Error("Invalid fleet telemetry");
  for (const vehicle of Object.values(v.simulation_truth.vehicles))
    if (
      !vec(vehicle.vehicle?.position_enu_m) ||
      !vec(vehicle.vehicle?.velocity_enu_mps) ||
      !Number.isFinite(vehicle.vehicle?.battery_fraction) ||
      vehicle.vehicle.battery_fraction < 0 ||
      vehicle.vehicle.battery_fraction > 1 ||
      !Number.isFinite(vehicle.vehicle.heading_rad) ||
      typeof vehicle.vehicle_id !== "string" ||
      typeof vehicle.mission_state !== "string" ||
      !Number.isSafeInteger(vehicle.completed_waypoints) ||
      !Number.isSafeInteger(vehicle.waypoint_count) ||
      !Array.isArray(vehicle.events)
    )
      throw new Error("Invalid vehicle telemetry");
  return v;
}
export function receive(state: StreamState, next: Envelope): StreamState {
  const previous = state.envelope;
  const same =
    previous?.run_id === next.run_id && previous.epoch === next.epoch;
  if (same && next.sequence < previous.sequence) return state;
  if (same && next.sequence === previous.sequence)
    return { ...state, envelope: next };
  const gap =
    !!same &&
    (next.sequence > previous.sequence + 1 || next.delivery === "resync");
  const trails: Record<string, TrailPoint[]> = same
    ? { ...state.trails }
    : Object.create(null);
  const events = same ? [...state.events] : [];
  for (const [id, v] of Object.entries(next.simulation_truth.vehicles)) {
    trails[id] = [
      ...(Object.hasOwn(trails, id) ? trails[id] : []),
      { position: v.vehicle.position_enu_m, gap },
    ].slice(-600);
    for (const event of v.events) {
      const key = `${id}:${event.tick}:${event.kind}`;
      if (!events.some((e) => e.id === key))
        events.push({
          id: key,
          time: event.time_s,
          text: `${id} · ${event.kind.replaceAll("_", " ")}${event.reason ? " · " + event.reason : ""}`,
          kind: event.kind,
        });
    }
  }
  if (gap)
    events.push({
      id: `gap:${next.sequence}`,
      time: next.simulation_truth.simulation_time_s,
      text: "Telemetry resynchronized · trajectory gap preserved",
      kind: "resync",
    });
  return { envelope: next, trails, events: events.slice(-100), gap };
}
export const colors = ["#F36B39", "#4EADE1", "#52D596", "#DFC57C"];
export const speed = (v: Vehicle) => Math.hypot(...v.vehicle.velocity_enu_mps);
export const timestamp = (seconds: number) => {
  const whole = Math.floor(seconds);
  return `${String(Math.floor(whole / 60)).padStart(2, "0")}:${String(whole % 60).padStart(2, "0")}.${String(Math.floor((seconds % 1) * 100)).padStart(2, "0")}`;
};
export function fitBounds(config: FleetConfig | null, vehicles: Vehicle[]) {
  const points: number[][] = vehicles.map((v) => v.vehicle.position_enu_m);
  for (const m of config?.members || [])
    points.push(
      m.mission.home_enu_m,
      m.mission.start_enu_m,
      ...m.mission.waypoints.map((w) => w.position_enu_m),
      ...(m.mission.survey_area || []),
    );
  if (!points.length) return { cx: 0, cy: 0, span: 40 };
  const minX = Math.min(...points.map((p) => p[0])),
    maxX = Math.max(...points.map((p) => p[0])),
    minY = Math.min(...points.map((p) => p[1])),
    maxY = Math.max(...points.map((p) => p[1]));
  return {
    cx: (minX + maxX) / 2,
    cy: (minY + maxY) / 2,
    span: Math.max(maxX - minX, (maxY - minY) * 1.76, 10) * 1.3,
  };
}
