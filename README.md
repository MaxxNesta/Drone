# SKYVIEW — local civilian survey simulator

**Prototype release candidate: 0.1.0-rc.1.** Python owns vehicle dynamics, mission planning and the shared fleet clock. The local Next.js dashboard displays ENU telemetry and submits authenticated simulation-only commands.

Start with [Stage 11 setup, release results and troubleshooting](docs/STAGE11_INTEGRATION.md). Architecture and history are in [ARCHITECTURE.md](docs/ARCHITECTURE.md) and [DEVELOPMENT_PLAN.md](docs/DEVELOPMENT_PLAN.md).

```sh
python3.12 -m venv .venv-backend
.venv-backend/bin/python -m pip install -r requirements-backend-lock.txt
(cd dashboard && npm ci)
python3 scripts/skyview.py --init-env
python3 scripts/skyview.py
```

Use Node 24. Open `http://127.0.0.1:3000` and unlock with `DASHBOARD_ACCESS_KEY` from the generated private `.skyview.env`. Ctrl-C stops the two local services. Do not commit or share the environment file.

This is software-only simulation. It does not connect aircraft or motors, implement collision avoidance, provide realistic terrain or certify real-world safety. `existing-tracker/` is preserved reference material, separate from the SKYVIEW runtime.
