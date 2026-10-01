"""Read-only optional SITL preflight. Never installs, starts or connects to PX4."""
import argparse
import json
import platform
import re
import shlex
from pathlib import Path
import shutil
import subprocess


def probe(command):
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=10)
        return {'exit_code': result.returncode, 'output': (result.stdout+result.stderr)[:4096].strip()}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {'exit_code': None, 'output': str(error)}


def os_release(path=Path('/etc/os-release')):
    """Read distro metadata as data, without executing shell syntax (Python 3.9)."""
    try:
        fields = {}
        for line in path.read_text().splitlines():
            if not line.strip() or line.lstrip().startswith('#'):
                continue
            key, separator, value = line.partition('=')
            if not separator:
                return {}
            words = shlex.split(value, comments=True)
            if len(words) != 1 or key in fields:
                return {}
            fields[key] = words[0]
        return fields
    except (OSError, ValueError, UnicodeError):
        return {}


def gazebo_matches(result, major):
    """Fail closed on failed probes, unknown format, or ambiguous major installs."""
    if result['exit_code'] != 0:
        return False
    versions = result['output'].strip().splitlines()
    return bool(versions) and all(
        re.fullmatch(str(major)+r'\.\d+\.\d+', version.strip()) for version in versions)


def diagnose(px4=None):
    spec = json.loads((Path(__file__).resolve().parents[1]/'config/sitl/environment.json').read_text())
    binaries = {name: shutil.which(name) for name in ('gz', 'cmake', 'ninja', 'git', 'docker', 'colima', 'podman')}
    reasons = []
    distro = os_release() if platform.system() == 'Linux' else {}
    if (platform.system() != 'Linux' or platform.machine() not in ('x86_64', 'amd64')
            or distro.get('ID') != spec['os_id'] or distro.get('VERSION_ID') != spec['os_version_id']):
        reasons.append('Host differs from selected Ubuntu 22.04 amd64 baseline; use a separately qualified environment')
    if not binaries['gz']:
        reasons.append('Gazebo executable missing')
    gazebo = probe(['gz', 'sim', '--versions']) if binaries['gz'] else None
    if gazebo is not None and not gazebo_matches(gazebo, spec['gazebo_sim_major']):
        reasons.append('Gazebo version probe must succeed and identify only gz-sim major '+str(spec['gazebo_sim_major']))
    source = None
    if px4 is None:
        reasons.append('No PX4 checkout supplied')
    else:
        source = probe(['git', '-C', str(Path(px4).resolve()), 'rev-parse', 'HEAD'])
        if source['exit_code'] != 0 or source['output'] != spec['px4_commit']:
            reasons.append('PX4 source does not match pinned revision')
    return {'schema_version': 1, 'host': {'os': platform.system(), 'architecture': platform.machine(),
                                               'distribution_id': distro.get('ID'), 'version_id': distro.get('VERSION_ID')},
            'executables': binaries, 'px4_revision': source,
            'gazebo_version': gazebo,
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
