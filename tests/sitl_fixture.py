"""Test-only handoff publisher. Always labeled synthetic; not a SITL recording."""
import argparse
import time
from tests.test_real_sitl import populated
from simulator_adapter.sitl_runtime import atomic_json

if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    parser.add_argument('--epoch',default='browser-fixture')
    parser.add_argument('--age',type=float,default=0)
    args=parser.parse_args()
    row=populated().telemetry(10)
    now=time.monotonic()-args.age
    row['epoch']=args.epoch
    row['generated_monotonic_s']=now
    for key in ('simulation_truth','autopilot_estimate'):
        row[key]['epoch']=args.epoch
        for sample in row[key]['vehicles']:
            sample['received_monotonic_s']=now
    atomic_json(args.output,row)
