# SKYVIEW dashboard design contract

This is a local desktop workspace for the existing civilian survey simulator. The primary workflow is: unlock → validate/load a complete mission → inspect the fleet → control playback or issue a virtual command → review results/replay. Python owns every simulation result. The browser owns selection, map projection, presentation and transient connection state.

The owner's reference supplies the dark visual atmosphere, restrained orange emphasis and clear vehicle colors. The final scope explicitly selects a local ENU 2D map without terrain, geographic coordinates or Gazebo integration.

| Token | Value | Use |
| --- | --- | --- |
| Background | `#101C24` | Workspace and map |
| Panel | `#1D3039` | Secondary surfaces |
| Accent | `#F36B39` | Primary actions, first vehicle, route advisories |
| Active | `#52D596` | Connected/running indicators |
| Secondary | `#4EADE1` | Secondary vehicle, keyboard focus |
| Text | `#F1F5F7` | Primary content |
| Muted | `#91A0AA` | Units and supporting labels |

Barlow is the interface typeface; IBM Plex Mono is reserved for measurements and time. Both are self-hosted. Phosphor supplies icons; Radix supplies accessible modal behavior. Tailwind's base and the explicit CSS token system share the same palette. Distinguishable text labels accompany warning colors.

Desktop hierarchy: compact navigation; fleet at left; dominant ENU map; selected telemetry at right; playback and expandable event history below. Narrow layouts stack these regions and retain recordings and selection. Survey areas, dotted planned routes, solid measured trails and home diamonds have distinct representations. Trails never bridge known sequence gaps. Scale marks live in the same SVG coordinate transform as the routes.

Commands are visibly QUEUED until the backend reports APPLIED or REJECTED. Playback pause and vehicle Hold remain distinct. No placeholder vehicles, invented completion percentages, fake latency, terrain, latitude/longitude or tracking estimates are allowed. Empty, loading, disconnected, stale, resynchronizing and failed states must remain understandable.

The requested Taste and Impeccable skills inform spacing, typography, restraint and the functional dashboard review. The user's palette, reference and simulator contracts take priority over generic style suggestions. Focus outlines, keyboard-selectable vehicles, modal focus handling and reduced-motion behavior are retained. Validation includes desktop/narrow screenshots and browser workflows; it is not a claim of a comprehensive accessibility certification.
