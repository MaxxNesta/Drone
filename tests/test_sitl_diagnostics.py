"""Preflight compatibility checks, without installing or running SITL."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch
from scripts.sitl_diagnostics import diagnose, gazebo_matches, os_release


class SitlDiagnosticsTests(unittest.TestCase):
    def test_os_release_parser(self):
        with TemporaryDirectory() as folder:
            path = Path(folder)/'os-release'
            self.assertEqual(os_release(path), {})
            path.write_text('ID=ubuntu\nVERSION_ID="22.04"\nNAME="Ubuntu Linux"\n')
            self.assertEqual(os_release(path)['VERSION_ID'], '22.04')
            for text in ('ID="unterminated', 'ID=ubuntu\nID=debian', 'invalid'):
                path.write_text(text)
                self.assertEqual(os_release(path), {})

    def test_gazebo_probe_requires_unambiguous_major_and_success(self):
        for output in ('8.0.0', '8.9.0\n8.8.0\n'):
            self.assertTrue(gazebo_matches({'exit_code': 0, 'output': output}, 8))
        for output in ('', '7.9.0', '9.0.0', '8.0.0\n9.0.0', 'unrecognized 8.0.0', '8.0.0-pre'):
            self.assertFalse(gazebo_matches({'exit_code': 0, 'output': output}, 8))
        for code in (1, None):
            self.assertFalse(gazebo_matches({'exit_code': code, 'output': '8.0.0'}, 8))

    def report(self, distro, version='8.9.0', exit_code=0):
        commit = json.loads(Path('config/sitl/environment.json').read_text())['px4_commit']
        def probe(command):
            return {'exit_code': exit_code, 'output': version} if command[0] == 'gz' else {'exit_code': 0, 'output': commit}
        with patch('scripts.sitl_diagnostics.platform.system', return_value='Linux'), \
             patch('scripts.sitl_diagnostics.platform.machine', return_value='x86_64'), \
             patch('scripts.sitl_diagnostics.os_release', return_value=distro), \
             patch('scripts.sitl_diagnostics.shutil.which', side_effect=lambda name: '/bin/'+name), \
             patch('scripts.sitl_diagnostics.probe', side_effect=probe):
            return diagnose('/fixture/px4')

    def test_only_selected_ubuntu_release_matches(self):
        self.assertEqual(self.report({'ID': 'ubuntu', 'VERSION_ID': '22.04'})['blockers'], [])
        for distro in ({'ID': 'debian', 'VERSION_ID': '12'}, {'ID': 'fedora', 'VERSION_ID': '42'},
                       {'ID': 'ubuntu', 'VERSION_ID': '24.04'}, {}, {'ID': 'ubuntu'}):
            self.assertTrue(any('Host differs' in reason for reason in self.report(distro)['blockers']))

    def test_incompatible_gazebo_is_a_reported_blocker(self):
        for version, code in [('7.0.0', 0), ('8.0.0', 1), ('unknown', 0), ('', None)]:
            report = self.report({'ID': 'ubuntu', 'VERSION_ID': '22.04'}, version, code)
            self.assertTrue(any('Gazebo version probe' in reason for reason in report['blockers']))
            self.assertFalse(report['sitl_executed'])
