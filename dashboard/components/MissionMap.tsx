"use client";
import { useMemo, useState } from "react";
import { ArrowsOut, Minus, Plus, NavigationArrow } from "@phosphor-icons/react";
import {
  colors,
  fitBounds,
  FleetConfig,
  TrailPoint,
  Vehicle,
} from "@/lib/model";
interface Props {
  config: FleetConfig | null;
  vehicles: Vehicle[];
  selected: string | null;
  select: (id: string) => void;
  trails: Record<string, TrailPoint[]>;
  activePairs: string[][];
}
export default function MissionMap({
  config,
  vehicles,
  selected,
  select,
  trails,
  activePairs,
}: Props) {
  const [zoom, setZoom] = useState(1),
    [routes, setRoutes] = useState(true),
    [areas, setAreas] = useState(true),
    [trail, setTrail] = useState(true);
  const bounds = useMemo(
    () => fitBounds(config, vehicles),
    [config, vehicles.length],
  ); // fixed extent; never chase live positions
  const scale = (740 / bounds.span) * zoom;
  const point = (p: number[]) => [
    440 + (p[0] - bounds.cx) * scale,
    310 - (p[1] - bounds.cy) * scale,
  ];
  const polygon = (points: number[][]) =>
    points.map((p) => point(p).join(",")).join(" ");
  const base = 10 ** Math.floor(Math.log10(bounds.span / 8));
  const gridStep =
    [1, 2, 5, 10].find((n) => n * base >= bounds.span / 8)! * base;
  const ticks = Array.from({ length: 21 }, (_, i) => (i - 10) * gridStep);
  const ids =
    config?.members.map((m) => m.vehicle_id) ||
    vehicles.map((v) => v.vehicle_id);
  return (
    <div className="map-shell">
      <div className="map-top">
        <div>
          <h2>
            {config?.fleet_id.replaceAll("-", " ") || "Local survey workspace"}
          </h2>
          <p>
            East / North / Up · meters <span className="small-dot" /> simulation
            truth
          </p>
        </div>
        <span className="map-mode">
          2D <span>LOCAL ENU</span>
        </span>
      </div>
      <div className="map-layers">
        <label>
          <input
            type="checkbox"
            checked={areas}
            onChange={(e) => setAreas(e.target.checked)}
          />
          Survey zones
        </label>
        <label>
          <input
            type="checkbox"
            checked={routes}
            onChange={(e) => setRoutes(e.target.checked)}
          />
          Planned routes
        </label>
        <label>
          <input
            type="checkbox"
            checked={trail}
            onChange={(e) => setTrail(e.target.checked)}
          />
          Live trails
        </label>
      </div>
      <svg
        className="mission-canvas"
        viewBox="0 0 880 620"
        role="group"
        aria-label="Local ENU mission map in meters"
      >
        <defs>
          <radialGradient id="mapLight">
            <stop stopColor="#F36B39" stopOpacity=".09" />
            <stop offset="1" stopColor="#101C24" stopOpacity="0" />
          </radialGradient>
          <filter id="markerGlow">
            <feGaussianBlur stdDeviation="5" />
          </filter>
          <pattern
            id="microgrid"
            width="22"
            height="22"
            patternUnits="userSpaceOnUse"
          >
            <circle cx="1" cy="1" r=".65" fill="#91A0AA" opacity=".2" />
          </pattern>
        </defs>
        <rect width="880" height="620" fill="url(#microgrid)" />
        <ellipse cx="470" cy="290" rx="410" ry="310" fill="url(#mapLight)" />
        {ticks.map((t) => {
          const [x, y] = point([bounds.cx + t, bounds.cy + t]);
          return (
            <g key={t} className="gridline">
              <line x1={x} y1="0" x2={x} y2="620" />
              <line x1="0" y1={y} x2="880" y2={y} />
              <text x={x + 5} y="600">
                {(bounds.cx + t).toFixed(0)}
              </text>
              <text x="10" y={y - 5}>
                {(bounds.cy + t).toFixed(0)}
              </text>
            </g>
          );
        })}
        {config?.members.map((member, index) => {
          const color = colors[index % colors.length],
            m = member.mission;
          const [hx, hy] = point(m.home_enu_m);
          return (
            <g
              key={member.vehicle_id}
              opacity={selected && selected !== member.vehicle_id ? 0.48 : 1}
            >
              {areas && m.survey_area && (
                <polygon
                  points={polygon(m.survey_area)}
                  fill={color}
                  fillOpacity=".085"
                  stroke={color}
                  strokeOpacity=".48"
                  strokeDasharray="3 5"
                />
              )}
              {routes && (
                <>
                  <polyline
                    points={polygon([
                      m.start_enu_m,
                      ...m.waypoints.map((w) => w.position_enu_m),
                    ])}
                    fill="none"
                    stroke={color}
                    strokeOpacity=".5"
                    strokeWidth="1.4"
                    strokeDasharray="2 6"
                  />
                  {m.waypoints.map((w, i) => {
                    const [x, y] = point(w.position_enu_m);
                    return (
                      <circle
                        key={i}
                        cx={x}
                        cy={y}
                        r="2.4"
                        fill={color}
                        fillOpacity=".75"
                      >
                        <title>
                          {member.vehicle_id} waypoint {i + 1}
                        </title>
                      </circle>
                    );
                  })}
                </>
              )}
              <g transform={`translate(${hx},${hy})`}>
                <rect
                  x="-5"
                  y="-5"
                  width="10"
                  height="10"
                  transform="rotate(45)"
                  fill="#101C24"
                  stroke={color}
                />
                <text x="12" y="4" fill={color} fontSize="9">
                  H
                </text>
              </g>
            </g>
          );
        })}
        {trail &&
          Object.entries(trails).map(([id, points]) => {
            const segments: number[][][] = [[]];
            for (const p of points) {
              if (p.gap) segments.push([]);
              segments[segments.length - 1].push(p.position);
            }
            return (
              <g key={id}>
                {segments.map((s, i) => (
                  <polyline
                    key={i}
                    points={polygon(s)}
                    fill="none"
                    stroke={colors[Math.max(0, ids.indexOf(id)) % 4]}
                    strokeWidth="2"
                    strokeOpacity=".8"
                  />
                ))}
              </g>
            );
          })}
        {activePairs.map(([a, b]) => {
          const va = vehicles.find((v) => v.vehicle_id === a),
            vb = vehicles.find((v) => v.vehicle_id === b);
          if (!va || !vb) return null;
          const [x1, y1] = point(va.vehicle.position_enu_m),
            [x2, y2] = point(vb.vehicle.position_enu_m);
          return (
            <line
              key={a + b}
              x1={x1}
              y1={y1}
              x2={x2}
              y2={y2}
              stroke="#FF7474"
              strokeWidth="3"
              strokeDasharray="5 4"
            />
          );
        })}
        {vehicles.map((v, index) => {
          const [x, y] = point(v.vehicle.position_enu_m),
            color = colors[Math.max(0, ids.indexOf(v.vehicle_id)) % 4],
            active = selected === v.vehicle_id;
          return (
            <g
              key={v.vehicle_id}
              role="button"
              tabIndex={0}
              aria-label={`Select ${v.vehicle_id} on map`}
              onClick={() => select(v.vehicle_id)}
              onKeyDown={(e) => {
                if (e.key === "Enter" || e.key === " ") {
                  e.preventDefault();
                  select(v.vehicle_id);
                }
              }}
              transform={`translate(${x},${y})`}
              className="map-vehicle"
            >
              <circle
                r={active ? 29 : 20}
                fill={color}
                opacity=".15"
                filter="url(#markerGlow)"
              />
              <circle
                r={active ? 19 : 13}
                fill={color}
                fillOpacity=".12"
                stroke={color}
                strokeOpacity={active ? 0.8 : 0.4}
              />
              <circle r="4" fill={color} />
              <line x1="-8" y1="0" x2="8" y2="0" stroke={color} />
              <line x1="0" y1="-8" x2="0" y2="8" stroke={color} />
              <path
                d="M -4 -11 L 0 -17 L 4 -11"
                stroke={color}
                fill="none"
                transform={`rotate(${(v.vehicle.heading_rad * 180) / Math.PI})`}
              />
              <g transform={`translate(0,${index % 2 === 0 ? -42 : 40})`}>
                <rect
                  x="-51"
                  y="-13"
                  width="102"
                  height="25"
                  rx="5"
                  fill="#142730"
                  stroke={color}
                  strokeOpacity={active ? 1 : 0.4}
                />
                <text textAnchor="middle" y="4" fill="#F1F5F7" fontSize="11">
                  {v.vehicle_id}
                </text>
              </g>
            </g>
          );
        })}
        <g className="svg-scale" transform="translate(55,560)">
          <path
            d={`M 0 -4 V 0 H ${gridStep * scale} V -4`}
            fill="none"
            stroke="#91A0AA"
          />
          <text x="0" y="18" fill="#91A0AA" fontSize="11">
            {gridStep} m
          </text>
        </g>
      </svg>
      {!vehicles.length && (
        <div className="map-empty">
          <NavigationArrow size={32} />
          <h3>Your next mission starts here.</h3>
          <p>
            Load a validated configuration to reveal the fleet,
            <br />
            survey areas and planned routes.
          </p>
        </div>
      )}
      <div className="map-tools">
        <button
          aria-label="Zoom in"
          onClick={() => setZoom((z) => Math.min(z * 1.2, 3))}
        >
          <Plus />
        </button>
        <button
          aria-label="Zoom out"
          onClick={() => setZoom((z) => Math.max(z / 1.2, 0.5))}
        >
          <Minus />
        </button>
        <button aria-label="Fit mission" onClick={() => setZoom(1)}>
          <ArrowsOut />
        </button>
      </div>
      <div className="map-bottom">
        <span className="north">
          <NavigationArrow size={17} /> N
        </span>
        <span>LOCAL FRAME · NO GEOGRAPHIC ORIGIN</span>
      </div>
    </div>
  );
}
