export type SitlVehicle = {
  vehicle_id: string;
  source_id: string;
  provenance: "simulation_truth" | "autopilot_estimate";
  position_enu_m: number[] | null;
  velocity_enu_mps: number[] | null;
  heading_rad: number | null;
  battery_fraction: number | null;
  source_time_s: number;
  freshness: "fresh" | "stale" | "lost";
};
export type SitlSnapshot = {
  availability: "available" | "unavailable";
  simulation_time_s: number;
  vehicles: SitlVehicle[];
};
export type SitlResponse = {
  configured: boolean;
  status: "disabled" | "waiting" | "live" | "stale" | "error";
  detail: string;
  telemetry: {
    evidence: "synthetic_fixture" | "live_transport_unqualified";
    epoch: string;
    sequence: number;
    estimator_validity: string;
    reason: string | null;
    simulation_truth: SitlSnapshot;
    autopilot_estimate: SitlSnapshot | null;
  } | null;
};
const finite = (x: unknown) => typeof x === "number" && Number.isFinite(x);
export function decodeSitl(value: unknown): SitlResponse {
  const v = value as SitlResponse;
  if (
    !v ||
    typeof v.configured !== "boolean" ||
    typeof v.detail !== "string" ||
    !["disabled", "waiting", "live", "stale", "error"].includes(v.status)
  )
    throw new Error("Unsupported SITL response");
  if (v.telemetry !== null) {
    const t = v.telemetry;
    const raw = t as unknown as Record<string, unknown>;
    if (
      raw.schema_version !== 1 ||
      raw.type !== "sitl_telemetry" ||
      raw.read_only !== true ||
      !["synthetic_fixture", "live_transport_unqualified"].includes(
        t.evidence,
      ) ||
      typeof t.epoch !== "string" ||
      !Number.isSafeInteger(t.sequence) ||
      t.sequence < 0 ||
      !["missing", "valid", "invalid_or_unknown"].includes(
        t.estimator_validity,
      ) ||
      (t.reason !== null && typeof t.reason !== "string")
    )
      throw new Error("Invalid SITL telemetry");
    for (const [snapshot, provenance] of [
      [t.simulation_truth, "simulation_truth"],
      [t.autopilot_estimate, "autopilot_estimate"],
    ] as const) {
      if (snapshot === null && provenance === "autopilot_estimate") continue;
      if (
        !snapshot ||
        !finite(snapshot.simulation_time_s) ||
        !["available", "unavailable"].includes(snapshot.availability) ||
        !Array.isArray(snapshot.vehicles) ||
        snapshot.vehicles.length > 1
      )
        throw new Error("Invalid SITL source");
      for (const vehicle of snapshot.vehicles) {
        if (
          typeof vehicle.vehicle_id !== "string" ||
          typeof vehicle.source_id !== "string" ||
          vehicle.provenance !== provenance ||
          !finite(vehicle.source_time_s) ||
          !["fresh", "stale", "lost"].includes(vehicle.freshness)
        )
          throw new Error("Invalid SITL vehicle");
        for (const vector of [vehicle.position_enu_m, vehicle.velocity_enu_mps])
          if (
            vector !== null &&
            (!Array.isArray(vector) ||
              vector.length !== 3 ||
              !vector.every(finite))
          )
            throw new Error("Invalid SITL vector");
        if (
          (vehicle.heading_rad !== null && !finite(vehicle.heading_rad)) ||
          (vehicle.battery_fraction !== null &&
            (!finite(vehicle.battery_fraction) ||
              vehicle.battery_fraction < 0 ||
              vehicle.battery_fraction > 1))
        )
          throw new Error("Invalid SITL measurement");
      }
    }
  }
  return v;
}
