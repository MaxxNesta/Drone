"""Read-only optional SITL preflight. Never installs, starts or connects to PX4."""
import argparse
import json
import platform
from pathlib import Path
import shutil
import subprocess


def probe(command):
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=10)
        return {'exit_code': result.returncode, 'output': (result.stdout+result.stderr)[:4096].strip()}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {'exit_code': None, 'output': str(error)}


def diagnose(px4=None):
    spec = json.loads((Path(__file__).resolve().parents[1]/'config/sitl/environment.json').read_text())
    binaries = {name: shutil.which(name) for name in ('gz', 'cmake', 'ninja', 'git', 'docker', 'colima', 'podman')}
    reasons = []
    if platform.system() != 'Linux' or platform.machine() not in ('x86_64', 'amd64'):
        reasons.append('Host differs from selected Ubuntu amd64 baseline; use a separately qualified environment')
    if not binaries['gz']:
        reasons.append('Gazebo executable missing')
    source = None
    if px4 is None:
        reasons.append('No PX4 checkout supplied')
    else:
        source = probe(['git', '-C', str(Path(px4).resolve()), 'rev-parse', 'HEAD'])
        if source['output'] != spec['px4_commit']:
            reasons.append('PX4 source does not match pinned revision')
    return {'schema_version': 1, 'host': {'os': platform.system(), 'architecture': platform.machine()},
            'executables': binaries, 'px4_revision': source,
            'gazebo_version': probe(['gz', 'sim', '--versions']) if binaries['gz'] else None,
            'blockers': reasons, 'sitl_executed': False,
            'note': 'Preflight only; presence of tools does not establish successful integration.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4', type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    output = json.dumps(diagnose(args.px4), indent=2)+'\n'
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(output)
    else:
        print(output, end='')


if __name__ == '__main__':
    main()
