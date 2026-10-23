"""Load genuine device calibration; keep provisional factory data explicit."""
import copy
from pathlib import Path
import math
import numpy as np
import yaml
from .core import transform


def load_yaml(path):
    text = Path(path).read_text(encoding='utf-8-sig')
    text = '\n'.join(line for line in text.splitlines() if not line.startswith('%YAML:'))
    data = yaml.safe_load(text)
    if not isinstance(data,dict):
        raise ValueError('expected YAML mapping: '+str(path))
    return data


def save_yaml(path, value, opencv=False):
    class OpenCVDumper(yaml.SafeDumper):
        def increase_indent(self, flow=False, indentless=False):
            return super().increase_indent(flow, False)

    def sequence(dumper, values):
        return dumper.represent_sequence('tag:yaml.org,2002:seq', values,
                                         flow_style=not any(isinstance(v,(list,dict)) for v in values))

    OpenCVDumper.add_representer(list, sequence)
    with Path(path).open('w',encoding='utf-8',newline='\n') as f:
        if opencv:
            f.write('%YAML:1.0\n---\n')
        f.write(yaml.dump(value,Dumper=OpenCVDumper if opencv else yaml.SafeDumper,sort_keys=False))


def validate_chain(chain):
    if set(chain) != {'cam0','cam1'}:
        raise ValueError('exactly cam0 and cam1 required (left and right infrared)')
    for name,cam in chain.items():
        transform(cam['T_cam_imu'])
        if cam.get('camera_model') != 'pinhole' or cam.get('distortion_model') not in ('radtan','equidistant'):
            raise ValueError(name+': unsupported camera/distortion model')
        k,d = np.asarray(cam['intrinsics'],float),np.asarray(cam['distortion_coeffs'],float)
        if k.shape!=(4,) or d.shape!=(4,) or not np.isfinite(k).all() or not np.isfinite(d).all() or min(k[:2])<=0:
            raise ValueError(name+': invalid intrinsics/distortion')
        if cam.get('resolution') != [848,480]:
            raise ValueError(name+': this launch requires calibration at 848x480')
        if not math.isfinite(float(cam.get('timeshift_cam_imu',float('nan')))):
            raise ValueError(name+': camera/IMU time offset is required')
    a,b = (transform(chain[c]['T_cam_imu']) for c in ('cam0','cam1'))
    rel = b @ np.linalg.inv(a)
    if not .05 < np.linalg.norm(rel[:3,3]) < .15:
        raise ValueError('stereo baseline inconsistent with a D455 (expect about 0.095 m)')
    if abs(float(chain['cam0']['timeshift_cam_imu'])-float(chain['cam1']['timeshift_cam_imu'])) > .002:
        raise ValueError('stereo camera time offsets disagree by more than 2 ms; inspect calibration')


def validate_imu(imu):
    # Kalibr imu.yaml is flat; OpenVINS uses imu0 nesting.
    data = copy.deepcopy(imu.get('imu0',imu))
    for key in ('accelerometer_noise_density','accelerometer_random_walk','gyroscope_noise_density','gyroscope_random_walk','update_rate'):
        value = float(data[key])
        if not math.isfinite(value) or value<=0:
            raise ValueError('invalid IMU '+key)
    return data


def create_config(upstream, output, chain, imu, status, metadata=None, mounting=None):
    """Generate new directory; never silently replace a device calibration."""
    validate_chain(chain)
    imu = validate_imu(imu)
    if status not in ('factory_preview','kalibr'):
        raise ValueError('unknown calibration status')
    if mounting is not None:
        mounting = transform(mounting).tolist()
    upstream,output = Path(upstream),Path(output)
    estimator = load_yaml(upstream/'config/rs_d455/estimator_config.yaml')
    # The official rs_d455 example is COLOR MONOCULAR. Replace both cameras.
    estimator.update(use_stereo=True,max_cameras=2,calib_cam_extrinsics=False,
                     calib_cam_intrinsics=False,calib_cam_timeoffset=False,
                     calib_imu_intrinsics=False,calib_imu_g_sensitivity=False,
                     try_zupt=False,init_dyn_use=False,track_frequency=30.,
                     relative_config_imu='kalibr_imu_chain.yaml',
                     relative_config_imucam='kalibr_imucam_chain.yaml')
    chain = copy.deepcopy(chain)
    for idx in (0,1):
        chain['cam'+str(idx)]['rostopic']='/vio_inputs/'+('left' if idx==0 else 'right')+'/image_raw'
        chain['cam'+str(idx)]['cam_overlaps']=[1-idx]
    imu['rostopic']='/vio_inputs/imu'
    imu.setdefault('time_offset',0.)
    imu.setdefault('T_i_b',np.eye(4).tolist())
    # Kalibr --imu-models calibrated uses an already calibrated sensor stream.
    # These are identity intrinsic corrections, not guessed camera/body extrinsics.
    imu.setdefault('model','kalibr')
    for key in ('Tw','Ta','R_IMUtoGYRO','R_IMUtoACC'):
        imu.setdefault(key,np.eye(3).tolist())
    imu.setdefault('Tg',np.zeros((3,3)).tolist())
    if imu['model'] not in ('kalibr','calibrated'):
        raise ValueError('only the calibrated/Kalibr IMU model is supported')
    for key in ('Tw','Ta','R_IMUtoGYRO','R_IMUtoACC','Tg'):
        mat=np.asarray(imu[key],float)
        if mat.shape!=(3,3) or not np.isfinite(mat).all():
            raise ValueError('invalid IMU intrinsic matrix '+key)
    rig = dict(calibration_status=status,T_base_imu=mounting,
               provenance=metadata or {},width=848,height=480,
               limitations=('Ground validation only; flight health/reset recovery not implemented.' if status=='kalibr' else
                            'Factory preview only: temporal offset and device IMU noise are not measured.'))
    output.mkdir(parents=True,exist_ok=False)
    save_yaml(output/'estimator_config.yaml',estimator,True)
    save_yaml(output/'kalibr_imucam_chain.yaml',chain,True)
    save_yaml(output/'kalibr_imu_chain.yaml',{'imu0':imu},True)
    save_yaml(output/'rig.yaml',rig)
    return output
