"use client";
import { useEffect, useState } from "react";
import { api } from "@/lib/useMission";
import { decodeSitl, SitlResponse, SitlSnapshot } from "@/lib/sitl";
const vector = (v: number[] | null) =>
  v ? v.map((n) => n.toFixed(2)).join(" / ") : "Unavailable";
function Source({
  title,
  snapshot,
}: {
  title: string;
  snapshot: SitlSnapshot | null;
}) {
  const vehicle = snapshot?.vehicles[0];
  return (
    <section className="sitl-source" aria-label={title}>
      <h3>{title}</h3>
      <p>
        {snapshot?.availability || "unavailable"} ·{" "}
        {vehicle?.freshness || "No samples"}
      </p>
      {vehicle ? (
        <dl>
          <div>
            <dt>Vehicle / source</dt>
            <dd>
              {vehicle.vehicle_id} / {vehicle.source_id}
            </dd>
          </div>
          <div>
            <dt>Sample time</dt>
            <dd>{vehicle.source_time_s.toFixed(3)} s</dd>
          </div>
          <div>
            <dt>Position E / N / U</dt>
            <dd>
              {vector(vehicle.position_enu_m)}
              {vehicle.position_enu_m && " m"}
            </dd>
          </div>
          <div>
            <dt>Velocity E / N / U</dt>
            <dd>
              {vector(vehicle.velocity_enu_mps)}
              {vehicle.velocity_enu_mps && " m/s"}
            </dd>
          </div>
          <div>
            <dt>Heading (north zero)</dt>
            <dd>
              {vehicle.heading_rad === null
                ? "Unavailable"
                : vehicle.heading_rad.toFixed(3) + " rad"}
            </dd>
          </div>
          <div>
            <dt>Battery</dt>
            <dd>
              {vehicle.battery_fraction === null
                ? "Unavailable"
                : (vehicle.battery_fraction * 100).toFixed(1) + "%"}
            </dd>
          </div>
        </dl>
      ) : (
        <p>Waiting for a compatible source. Missing values are not inferred.</p>
      )}
    </section>
  );
}
export default function SitlPanel() {
  const [open, setOpen] = useState(false);
  const [data, setData] = useState<SitlResponse | null>(null);
  const [error, setError] = useState("");
  useEffect(() => {
    if (!open) return;
    let cancelled = false;
    let timer: ReturnType<typeof setTimeout>;
    async function refresh() {
      try {
        const value = decodeSitl(await api("sitl"));
        if (!cancelled) {
          setData(value);
          setError("");
        }
      } catch (e) {
        if (!cancelled) {
          setData(null);
          setError((e as Error).message);
        }
      } finally {
        if (!cancelled) timer = setTimeout(refresh, 1000);
      }
    }
    void refresh();
    return () => {
      cancelled = true;
      clearTimeout(timer);
    };
  }, [open]);
  const telemetry = data?.telemetry;
  return (
    <details
      className="sitl-panel"
      onToggle={(e) => setOpen(e.currentTarget.open)}
    >
      <summary>
        PX4 / Gazebo telemetry <span>Read only</span>
      </summary>
      {open && (
        <div className="sitl-content">
          <p role="status">
            {error
              ? "Disconnected: " + error
              : data
                ? data.detail
                : "Loading local SITL status…"}
          </p>
          {data && (
            <p>
              Collector: <strong>{data.status}</strong>. The numerical mission
              workspace below remains independent.
            </p>
          )}
          {telemetry && (
            <>
              <p className="sitl-evidence">
                {telemetry.evidence === "synthetic_fixture"
                  ? "Synthetic fixture — not real SITL evidence"
                  : "Live transport — real SITL qualification not established"}
              </p>
              <p>
                Epoch {telemetry.epoch} · Sequence {telemetry.sequence} · EKF
                validity: {telemetry.estimator_validity}
              </p>
              {telemetry.reason && (
                <p role="status">{telemetry.reason.replaceAll("_", " ")}</p>
              )}
              <div className="sitl-sources">
                <Source
                  title="Gazebo simulation truth"
                  snapshot={telemetry.simulation_truth}
                />
                <Source
                  title="PX4 autopilot estimate"
                  snapshot={telemetry.autopilot_estimate}
                />
              </div>
              <p>
                Mission progress unavailable. No flight or playback commands are
                connected to this source.
              </p>
            </>
          )}
        </div>
      )}
    </details>
  );
}
