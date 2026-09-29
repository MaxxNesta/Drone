"use client";
import { useEffect, useState } from "react";
import * as Dialog from "@radix-ui/react-dialog";
import {
  AirplaneTilt,
  ArrowClockwise,
  BatteryCharging,
  Broadcast,
  CheckCircle,
  Compass,
  Crosshair,
  FolderOpen,
  LockKey,
  Pause,
  Play,
  ShieldCheck,
  SignOut,
  Stack,
  Stop,
  UploadSimple,
  Warning,
  X,
} from "@phosphor-icons/react";
import MissionMap from "./MissionMap";
import { api, useMission } from "@/lib/useMission";
import { colors, FleetConfig, speed, Status, timestamp } from "@/lib/model";

function Modal({
  open,
  onOpenChange,
  title,
  description,
  children,
}: {
  open: boolean;
  onOpenChange: (v: boolean) => void;
  title: string;
  description: string;
  children: React.ReactNode;
}) {
  return (
    <Dialog.Root open={open} onOpenChange={onOpenChange}>
      <Dialog.Portal>
        <Dialog.Overlay className="dialog-overlay" />
        <Dialog.Content className="dialog-content">
          <div className="dialog-header">
            <div>
              <Dialog.Title>{title}</Dialog.Title>
              <Dialog.Description>{description}</Dialog.Description>
            </div>
            <Dialog.Close className="icon-button" aria-label="Close dialog">
              <X size={20} />
            </Dialog.Close>
          </div>
          {children}
        </Dialog.Content>
      </Dialog.Portal>
    </Dialog.Root>
  );
}
export default function Dashboard() {
  const [unlocked, setUnlocked] = useState<boolean | null>(null),
    [key, setKey] = useState(""),
    [loginError, setLoginError] = useState(""),
    [busy, setBusy] = useState(false);
  const mission = useMission(!!unlocked),
    { stream, status, config, connection, error, outcomes, acks } = mission;
  const [selected, setSelected] = useState<string | null>(null),
    [loadOpen, setLoadOpen] = useState(false),
    [recordsOpen, setRecordsOpen] = useState(false),
    [stopOpen, setStopOpen] = useState(false);
  const [presets, setPresets] = useState<{ name: string; request: unknown }[]>(
      [],
    ),
    [request, setRequest] = useState(""),
    [validated, setValidated] = useState<FleetConfig | null>(null),
    [formError, setFormError] = useState("");
  const [records, setRecords] = useState<string[]>([]),
    [record, setRecord] = useState<{
      run_id: string;
      state: string;
      summary: {
        all_completed: boolean;
        duration_s: number;
        completed_fraction: number;
      };
      telemetry_sha256: string;
    } | null>(null),
    [recordError, setRecordError] = useState(""),
    [verified, setVerified] = useState(false);
  useEffect(() => {
    void api("session")
      .then(() => setUnlocked(true))
      .catch(() => setUnlocked(false));
  }, []);
  const envelope =
    stream.envelope?.run_id === status?.run_id ? stream.envelope : null;
  const vehicles = Object.values(envelope?.simulation_truth.vehicles || {}),
    vehicle = vehicles.find((v) => v.vehicle_id === selected);
  useEffect(() => {
    if (selected && !vehicles.some((v) => v.vehicle_id === selected))
      setSelected(null);
  }, [status?.run_id]);
  const online = ["live", "ready", "resynchronizing"].includes(connection),
    controllable = online && status?.state === "loaded";
  const elapsed = envelope?.simulation_truth.simulation_time_s || 0,
    total = config && status?.dt_s ? config.steps * status.dt_s : null;
  const completed = envelope?.simulation_truth.summary.completed_waypoints || 0,
    count = envelope?.simulation_truth.summary.waypoint_count || 0;
  const issues = envelope?.advisories.proximity || [],
    routes = envelope?.advisories.route_conflicts || [];
  async function unlock(e: React.FormEvent) {
    e.preventDefault();
    setBusy(true);
    try {
      await api("session", { key });
      setKey("");
      setUnlocked(true);
      setLoginError("");
    } catch (e) {
      setLoginError((e as Error).message);
    } finally {
      setBusy(false);
    }
  }
  async function openLoad() {
    setLoadOpen(true);
    setFormError("");
    try {
      setPresets(await api("presets"));
    } catch (e) {
      setFormError((e as Error).message);
    }
  }
  async function validate() {
    setBusy(true);
    setFormError("");
    try {
      const parsed = JSON.parse(request);
      const result = await api<{ normalized_config: FleetConfig }>(
        "validate",
        parsed.kind ? parsed : { kind: "fleet", config: parsed },
      );
      setValidated(result.normalized_config);
    } catch (e) {
      setValidated(null);
      setFormError((e as Error).message);
    } finally {
      setBusy(false);
    }
  }
  async function load() {
    if (!validated) return;
    setBusy(true);
    try {
      await api<Status>("runs", { kind: "fleet", config: validated });
      setLoadOpen(false);
      mission.refresh();
      setSelected(null);
    } catch (e) {
      setFormError((e as Error).message);
    } finally {
      setBusy(false);
    }
  }
  async function openRecords() {
    setRecordsOpen(true);
    setRecordError("");
    try {
      setRecords((await api<{ run_ids: string[] }>("results")).run_ids);
    } catch (e) {
      setRecordError((e as Error).message);
    }
  }
  async function replay() {
    if (!record) return;
    setBusy(true);
    try {
      await api(`results/${record.run_id}/replay`, {});
      setRecordsOpen(false);
      mission.refresh();
    } catch (e) {
      setRecordError((e as Error).message);
    } finally {
      setBusy(false);
    }
  }
  async function verify() {
    if (!record) return;
    setBusy(true);
    try {
      const result = await api<{ matches: boolean }>(
        `results/${record.run_id}/verify-replay`,
        {},
      );
      setVerified(result.matches);
      if (!result.matches)
        setRecordError("The recording did not reproduce exactly.");
    } catch (e) {
      setRecordError((e as Error).message);
    } finally {
      setBusy(false);
    }
  }
  const selectedIndex = Math.max(
    0,
    vehicles.findIndex((v) => v.vehicle_id === selected),
  );
  if (unlocked === null)
    return (
      <main className="unlock">
        <div className="brand">
          <AirplaneTilt size={28} />
          <span>SKYVIEW</span>
        </div>
        <div className="loading-line" />
        <p>Connecting to your local workspace…</p>
      </main>
    );
  if (!unlocked)
    return (
      <main className="unlock">
        <div className="unlock-orbit" />
        <form onSubmit={unlock} className="unlock-card">
          <span className="eyebrow">
            <ShieldCheck />
            LOCAL SIMULATION WORKSPACE
          </span>
          <div className="brand">
            <AirplaneTilt size={38} />
            <span>SKYVIEW</span>
          </div>
          <h1>
            A clearer view
            <br />
            of every mission.
          </h1>
          <p>
            Unlock your local mission-control dashboard.
            <br />
            Your virtual fleet stays on this computer.
          </p>
          <label htmlFor="access">Dashboard access key</label>
          <input
            id="access"
            autoComplete="current-password"
            type="password"
            value={key}
            onChange={(e) => setKey(e.target.value)}
            required
            placeholder="Enter your local access key"
          />
          <button className="primary" disabled={busy}>
            <LockKey size={17} />
            {busy ? "Unlocking…" : "Open workspace"}
          </button>
          {loginError && (
            <p role="alert" className="error-text">
              {loginError}
            </p>
          )}
          <small>SOFTWARE-ONLY · CIVILIAN SURVEY SIMULATOR</small>
        </form>
      </main>
    );
  return (
    <main className="dashboard">
      <header className="topbar">
        <div className="brand">
          <AirplaneTilt size={30} weight="duotone" />
          <div>
            SKYVIEW<small>VIRTUAL MISSION CONTROL</small>
          </div>
        </div>
        <nav aria-label="Workspace">
          <button className="nav-active" onClick={() => setSelected(null)}>
            Mission space
          </button>
          <button onClick={openRecords}>Recordings </button>
        </nav>
        <div className="top-right">
          <span className="local-badge">
            <ShieldCheck size={15} /> LOCAL ONLY
          </span>
          <div
            className={`connection ${online ? "connected" : ""}`}
            role="status"
          >
            <Broadcast size={15} />
            {connection === "live"
              ? "Live telemetry"
              : connection.replaceAll("-", " ")}
          </div>
          <button
            className="icon-button"
            title="Lock workspace"
            aria-label="Lock workspace"
            onClick={async () => {
              try {
                await api("session", undefined, "DELETE");
                setUnlocked(false);
              } catch (e) {
                mission.setError(
                  `Could not lock workspace: ${(e as Error).message}`,
                );
              }
            }}
          >
            <SignOut size={19} />
          </button>
        </div>
      </header>
      <div className="workspace-heading">
        <div>
          <h1>
            Mission overview
            <span className="heading-dot" />
          </h1>
        </div>
        <div className="heading-actions">
          <span className="run-status">
            {status?.state === "loaded"
              ? status.playback_paused
                ? "Playback paused"
                : "Simulation running"
              : status?.state || "Connecting"}
          </span>
          <button className="primary" onClick={openLoad}>
            <FolderOpen size={17} /> Load mission
          </button>
        </div>
      </div>
      {(error || status?.error) && (
        <div className="error-banner" role="alert">
          <Warning />
          {error || status?.error}
          <button onClick={mission.refresh}>Reconnect</button>
          <button
            aria-label="Dismiss error"
            onClick={() => mission.setError("")}
          >
            <X />
          </button>
        </div>
      )}
      {connection === "resynchronizing" && (
        <div className="resync-banner" role="status">
          Snapshot restored. Missing trajectory samples are shown as gaps.
        </div>
      )}
      <div className="workspace-grid">
        <aside className="fleet-panel panel">
          <div className="panel-title">
            <span>VIRTUAL FLEET</span>
            <span className="count">
              {vehicles.length.toString().padStart(2, "0")}
            </span>
          </div>
          <button
            className={`fleet-overview ${selected === null ? "selected" : ""}`}
            onClick={() => setSelected(null)}
          >
            <Stack size={20} />
            <div>
              <strong>All vehicles</strong>
              <small>Independent missions · shared clock</small>
            </div>
          </button>
          <div className="fleet-list">
            {vehicles.map((v, i) => (
              <button
                key={v.vehicle_id}
                className={`vehicle-card ${selected === v.vehicle_id ? "selected" : ""}`}
                onClick={() => setSelected(v.vehicle_id)}
                style={{ "--vehicle": colors[i % 4] } as React.CSSProperties}
              >
                <div className="vehicle-card-top">
                  <span className="vehicle-symbol">
                    <AirplaneTilt size={21} />
                  </span>
                  <strong>{v.vehicle_id}</strong>
                  <span className={`state-dot ${v.mission_state}`} />
                </div>
                <span className="vehicle-state">
                  {v.mission_state.replaceAll("_", " ")}
                </span>
                <div className="vehicle-stats">
                  <span>
                    <BatteryCharging size={14} />
                    {(v.vehicle.battery_fraction * 100).toFixed(0)}%
                  </span>
                  <span>
                    {v.vehicle.position_enu_m[2].toFixed(1)} <small>m UP</small>
                  </span>
                </div>
                <div className="mini-progress">
                  <i
                    style={{
                      width: `${(v.completed_waypoints / Math.max(1, v.waypoint_count)) * 100}%`,
                    }}
                  />
                </div>
                <small className="waypoint-count">
                  {v.completed_waypoints} / {v.waypoint_count} waypoints
                </small>
              </button>
            ))}
            {!vehicles.length && (
              <div className="quiet-empty">
                <AirplaneTilt size={28} />
                <p>No vehicles loaded</p>
                <small>
                  Choose a mission configuration
                  <br />
                  to populate your fleet.
                </small>
              </div>
            )}
          </div>
          <div className="fleet-footer">
            <span className="eyebrow">FLEET COMMANDS</span>
            <div className="command-row">
              <button
                disabled={!controllable}
                onClick={() => mission.command("start")}
              >
                Start
              </button>
              <button
                disabled={!controllable}
                onClick={() => mission.command("pause")}
              >
                Hold
              </button>
              <button
                disabled={!controllable}
                onClick={() => mission.command("resume")}
              >
                Resume
              </button>
            </div>
            <small>Commands apply at the next simulation tick.</small>
            <button
              className="fleet-stop"
              disabled={!controllable}
              onClick={() => {
                setSelected(null);
                setStopOpen(true);
              }}
            >
              Stop or abort fleet
            </button>
          </div>
        </aside>
        <section className="center-column">
          <MissionMap
            config={config}
            vehicles={vehicles}
            selected={selected}
            select={setSelected}
            trails={envelope ? stream.trails : {}}
            activePairs={issues.map((a) => a.vehicle_ids)}
          />
          <div className="map-metrics">
            <div>
              <span>SIMULATION TIME</span>
              <strong>
                {timestamp(elapsed)}
                <small>s</small>
              </strong>
            </div>
            <div>
              <span>WAYPOINT PROGRESS</span>
              <strong>
                {completed}
                <small>/ {count}</small>
              </strong>
            </div>
            <div>
              <span>ROUTE ADVISORIES</span>
              <strong className={routes.length ? "amber" : ""}>
                {routes.length.toString().padStart(2, "0")}
                <small>spatial</small>
              </strong>
            </div>
            <div>
              <span>PROXIMITY ALERTS</span>
              <strong className={issues.length ? "red" : ""}>
                {issues.length.toString().padStart(2, "0")}
                <small>active</small>
              </strong>
            </div>
          </div>
        </section>
        <aside className="telemetry-panel panel">
          <div className="panel-title">
            <span>{vehicle ? "VEHICLE TELEMETRY" : "FLEET TELEMETRY"}</span>
            <Crosshair size={16} />
          </div>
          <div className="selected-vehicle">
            <div
              className="vehicle-hero"
              style={{ color: colors[selectedIndex % 4] }}
            >
              <AirplaneTilt size={45} weight="duotone" />
              <span className="hero-ring" />
            </div>
            <h2>{vehicle?.vehicle_id || "Fleet overview"}</h2>
            <span className={`telemetry-tag ${vehicle?.mission_state || ""}`}>
              {vehicle?.mission_state || `${vehicles.length} virtual vehicles`}
            </span>
          </div>
          {vehicle ? (
            <>
              <div className="telemetry-pair">
                <div>
                  <span>3D SPEED</span>
                  <strong>
                    {speed(vehicle).toFixed(2)}
                    <small>m/s</small>
                  </strong>
                </div>
                <div>
                  <span>UP POSITION</span>
                  <strong>
                    {vehicle.vehicle.position_enu_m[2].toFixed(2)}
                    <small>m</small>
                  </strong>
                </div>
              </div>
              <div className="telemetry-section">
                <h3>
                  Local position <span>ENU · m</span>
                </h3>
                <div className="coordinate-row">
                  {["EAST", "NORTH", "UP"].map((label, i) => (
                    <div key={label}>
                      <span>{label}</span>
                      <strong>
                        {vehicle.vehicle.position_enu_m[i].toFixed(2)}
                      </strong>
                    </div>
                  ))}
                </div>
                <h3>
                  Velocity <span>m/s</span>
                </h3>
                <div className="coordinate-row">
                  {["E", "N", "U"].map((label, i) => (
                    <div key={label}>
                      <span>{label}</span>
                      <strong>
                        {vehicle.vehicle.velocity_enu_mps[i].toFixed(2)}
                      </strong>
                    </div>
                  ))}
                </div>
              </div>
              <div className="telemetry-section">
                <h3>
                  Battery{" "}
                  <strong className="green">
                    {(vehicle.vehicle.battery_fraction * 100).toFixed(1)}%
                  </strong>
                </h3>
                <div className="battery-track">
                  <i
                    style={{
                      width: `${vehicle.vehicle.battery_fraction * 100}%`,
                    }}
                  />
                </div>
                <h3>
                  Waypoint progress{" "}
                  <span>
                    {vehicle.completed_waypoints} / {vehicle.waypoint_count}
                  </span>
                </h3>
                <div className="progress-track">
                  <i
                    style={{
                      width: `${(vehicle.completed_waypoints / Math.max(1, vehicle.waypoint_count)) * 100}%`,
                    }}
                  />
                </div>
                <p className="muted">
                  Target error{" "}
                  <b>{vehicle.waypoint_error_m?.toFixed(2) ?? "—"} m</b>
                </p>
              </div>
              <div className="command-row vehicle-controls">
                {vehicle.mission_state === "idle" && (
                  <button
                    disabled={!controllable}
                    onClick={() => mission.command("start", vehicle.vehicle_id)}
                  >
                    Start
                  </button>
                )}
                <button
                  disabled={!controllable}
                  onClick={() => mission.command("pause", vehicle.vehicle_id)}
                >
                  Hold
                </button>
                <button
                  disabled={!controllable}
                  onClick={() => mission.command("resume", vehicle.vehicle_id)}
                >
                  Resume
                </button>
                <button
                  disabled={!controllable}
                  aria-label="Emergency stop selected vehicle"
                  onClick={() => setStopOpen(true)}
                >
                  <Stop size={14} />
                </button>
              </div>
              {vehicle.stop_reason && (
                <p className="error-text">
                  Stopped: {vehicle.stop_reason.replaceAll("_", " ")}
                </p>
              )}
            </>
          ) : (
            <div className="fleet-summary">
              <p>
                Select a vehicle on the map or in the fleet to inspect its live
                telemetry.
              </p>
              <div>
                <span>Clock timestep</span>
                <strong>
                  {status?.dt_s ? `${status.dt_s.toFixed(2)} s` : "—"}
                </strong>
              </div>
              <div>
                <span>Current tick</span>
                <strong>{status?.tick ?? "—"}</strong>
              </div>
              <div>
                <span>Source</span>
                <strong>Simulation truth</strong>
              </div>
              <div>
                <span>Tracking estimate</span>
                <strong>Not connected</strong>
              </div>
            </div>
          )}
          <div className="advisory-section">
            <h3>Airspace diagnostics</h3>
            {issues.length ? (
              issues.map((a, i) => (
                <div className="alert proximity" key={i}>
                  <Warning />
                  <div>
                    <strong>Proximity alert</strong>
                    <p>
                      {a.vehicle_ids.join(" / ")} ·{" "}
                      {a.minimum_separation_m?.toFixed(2)} m
                    </p>
                  </div>
                </div>
              ))
            ) : (
              <div className="clear-status">
                <CheckCircle />
                {envelope
                  ? "No active proximity alerts"
                  : "Awaiting proximity telemetry"}
              </div>
            )}
            {routes.map((a, i) => (
              <div className="alert route" key={i}>
                <Compass />
                <div>
                  <strong>Route overlap advisory</strong>
                  <p>{a.vehicle_ids.join(" / ")}</p>
                </div>
              </div>
            ))}
            <small>Advisory diagnostics only. No automatic avoidance.</small>
          </div>
        </aside>
      </div>
      <section className="timeline panel">
        <div className="playback-main">
          <button
            className="play-button"
            disabled={!controllable}
            aria-label={
              status?.playback_paused ? "Resume playback" : "Pause playback"
            }
            onClick={() => mission.playback(!status?.playback_paused)}
          >
            {status?.playback_paused ? (
              <Play size={20} weight="fill" />
            ) : (
              <Pause size={20} weight="fill" />
            )}
          </button>
          <div className="timecode">
            <strong>{timestamp(elapsed)}</strong>
            <span>/ {total === null ? "—" : timestamp(total)}</span>
          </div>
          <div
            className="timeline-track"
            role="progressbar"
            aria-label="Simulation playback progress"
            aria-valuenow={total ? Math.min(100, (elapsed / total) * 100) : 0}
          >
            <i
              style={{
                width: `${total ? Math.min(100, (elapsed / total) * 100) : 0}%`,
              }}
            />
            {stream.events
              .filter((e) => e.kind === "waypoint_reached")
              .slice(-30)
              .map((e) => (
                <span
                  key={e.id}
                  style={{ left: `${total ? (e.time / total) * 100 : 0}%` }}
                  title={e.text}
                />
              ))}
          </div>
          <label className="speed-select">
            SPEED
            <select
              aria-label="Playback speed"
              value={status?.playback_speed || 1}
              disabled={!controllable}
              onChange={(e) =>
                mission.playback(
                  status?.playback_paused ?? true,
                  Number(e.target.value),
                )
              }
            >
              {[0.5, 1, 2, 5, 10, 20].map((n) => (
                <option key={n} value={n}>
                  {n}×
                </option>
              ))}
            </select>
          </label>
        </div>
        <div className="event-strip">
          <span className="eyebrow">EVENT LOG</span>
          {stream.events.length ? (
            <>
              <time>{timestamp(stream.events.at(-1)!.time)}</time>
              <span>{stream.events.at(-1)!.text}</span>
            </>
          ) : (
            <span className="muted">Waiting for mission events</span>
          )}
          <span className="event-count">{stream.events.length} retained</span>
        </div>
        {stream.events.length > 0 && (
          <details className="event-history">
            <summary>View retained events</summary>
            <ol>
              {stream.events
                .slice()
                .reverse()
                .map((e) => (
                  <li key={e.id}>
                    <time>{timestamp(e.time)}</time>
                    <span>{e.text}</span>
                  </li>
                ))}
            </ol>
          </details>
        )}
      </section>
      {acks.length > 0 && (
        <section className="command-feedback" aria-live="polite">
          {acks.slice(-3).map((a) => {
            const outcome = outcomes.find((o) => o.request_id === a.request_id);
            return (
              <div
                key={a.request_id}
                className={outcome?.accepted === false ? "red" : ""}
              >
                <span>
                  {outcome
                    ? outcome.accepted
                      ? "APPLIED"
                      : "REJECTED"
                    : "QUEUED"}
                </span>
                {a.command} · {a.vehicle_id || "fleet"} · tick {a.target_tick}
                {outcome?.reason && ` · ${outcome.reason}`}
                {!outcome &&
                  status?.playback_paused &&
                  " · resume playback to apply"}
              </div>
            );
          })}
        </section>
      )}
      {!config && vehicles.length > 0 && (
        <p className="plan-notice">
          Plan geometry unavailable for this externally loaded run. Live vehicle
          positions remain authoritative.
        </p>
      )}
      <footer className="app-footer">
        <span>
          <ShieldCheck />
          SOFTWARE-ONLY ENVIRONMENT
        </span>
        <span>
          ENU / METERS <i /> LOCAL BACKEND <i />{" "}
          {envelope ? `SEQ ${envelope.sequence}` : "AWAITING TELEMETRY"}
        </span>
      </footer>
      <Modal
        open={loadOpen}
        onOpenChange={setLoadOpen}
        title="Load a mission"
        description="Validate an existing Stage 7 wrapper or Stage 8 fleet configuration before loading."
      >
        <div className="preset-row">
          <label>
            Repository scenarios
            <select
              aria-label="Repository scenario"
              defaultValue=""
              onChange={(e) => {
                const p = presets.find((p) => p.name === e.target.value);
                if (p) {
                  setRequest(JSON.stringify(p.request, null, 2));
                  setValidated(null);
                }
              }}
            >
              <option value="" disabled>
                Choose a scenario
              </option>
              {presets.map((p) => (
                <option key={p.name} value={p.name}>
                  {p.name.replaceAll("-", " ")}
                </option>
              ))}
            </select>
          </label>
          <label className="upload">
            <UploadSimple />
            Import JSON
            <input
              type="file"
              accept="application/json,.json"
              onChange={async (e) => {
                const file = e.target.files?.[0];
                if (!file) return;
                if (file.size > 1024 * 1024) {
                  setFormError("File exceeds 1 MiB");
                  return;
                }
                setRequest(await file.text());
                setValidated(null);
              }}
            />
          </label>
        </div>
        <textarea
          aria-label="Mission configuration JSON"
          spellCheck={false}
          value={request}
          onChange={(e) => {
            setRequest(e.target.value);
            setValidated(null);
          }}
          placeholder="Paste a complete mission or fleet request…"
        />
        {validated && (
          <div className="validation-success">
            <CheckCircle />
            Validated · {validated.members.length} vehicles · {validated.steps}{" "}
            ticks
          </div>
        )}
        {formError && (
          <p role="alert" className="error-text">
            {formError}
          </p>
        )}
        {status?.state === "loaded" && (
          <p className="muted">
            Finish the current simulation before loading another mission.
          </p>
        )}
        <div className="dialog-actions">
          <button disabled={!request || busy} onClick={validate}>
            Validate configuration
          </button>
          <button
            className="primary"
            disabled={!validated || busy || status?.state === "loaded"}
            onClick={load}
          >
            {busy ? "Working…" : "Load validated mission"}
          </button>
        </div>
      </Modal>
      <Modal
        open={recordsOpen}
        onOpenChange={setRecordsOpen}
        title="Mission recordings"
        description="Completed simulation budgets may contain unfinished or failed missions."
      >
        <div className="records-list">
          {records.length ? (
            records.map((id) => (
              <button
                key={id}
                className={record?.run_id === id ? "selected" : ""}
                onClick={async () => {
                  setVerified(false);
                  setRecordError("");
                  try {
                    setRecord(await api(`results/${id}`));
                  } catch (e) {
                    setRecordError((e as Error).message);
                  }
                }}
              >
                <FolderOpen />
                <span>{id.slice(0, 12)}…</span>
                <span>View results</span>
              </button>
            ))
          ) : (
            <p className="muted">
              No recordings yet. Finish a simulation to create one.
            </p>
          )}
        </div>
        {record && (
          <div className="record-detail">
            <h3>Run {record.run_id.slice(0, 12)}</h3>
            <p>
              Run state <b>{record.state}</b>
            </p>
            <p>
              Mission completion{" "}
              <b>
                {record.summary.all_completed
                  ? "All missions complete"
                  : "Not all missions complete"}
              </b>
            </p>
            <p>
              Recorded duration <b>{record.summary.duration_s.toFixed(2)} s</b>
            </p>
            {verified && (
              <p className="green">Exact deterministic replay verified</p>
            )}
            <div className="dialog-actions">
              <button
                disabled={busy || record.state !== "finished"}
                onClick={verify}
              >
                Verify replay
              </button>
              <button
                className="primary"
                disabled={
                  busy ||
                  record.state !== "finished" ||
                  status?.state === "loaded"
                }
                onClick={replay}
              >
                <ArrowClockwise />
                Replay as new run
              </button>
            </div>
          </div>
        )}
        {recordError && (
          <p role="alert" className="error-text">
            {recordError}
          </p>
        )}
      </Modal>
      <Modal
        open={stopOpen}
        onOpenChange={setStopOpen}
        title={`Stop ${selected || "fleet"}?`}
        description="Both abort and emergency stop freeze the selected virtual vehicle or fleet. Stopped missions cannot resume within this run."
      >
        <div className="dialog-actions">
          <button onClick={() => setStopOpen(false)}>Keep running</button>
          <button
            disabled={!controllable}
            onClick={() => {
              void mission.command("abort", selected || undefined);
              setStopOpen(false);
            }}
          >
            Abort mission
          </button>
          <button
            className="danger"
            disabled={!controllable}
            onClick={() => {
              void mission.command("emergency_stop", selected || undefined);
              setStopOpen(false);
            }}
          >
            Confirm virtual stop
          </button>
        </div>
      </Modal>
    </main>
  );
}
