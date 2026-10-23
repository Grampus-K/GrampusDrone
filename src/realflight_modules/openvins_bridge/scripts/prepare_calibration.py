#!/usr/bin/env python3
"""Import a measured stereo/IMU Kalibr chain; never supply guessed extrinsics."""
import argparse
from openvins_bridge.calibration import create_config,load_yaml


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--upstream',required=True)
    p.add_argument('--camchain',required=True)
    p.add_argument('--imu',required=True)
    p.add_argument('--mount',help='YAML containing measured T_base_imu, required only for PX4 ground output')
    p.add_argument('--output',required=True)
    a=p.parse_args()
    mount=load_yaml(a.mount)['T_base_imu'] if a.mount else None
    out=create_config(a.upstream,a.output,load_yaml(a.camchain),load_yaml(a.imu),'kalibr',
                      metadata=dict(camchain=a.camchain,imu_noise=a.imu,mount=a.mount),mounting=mount)
    print('Wrote',out)


if __name__=='__main__':
    main()
