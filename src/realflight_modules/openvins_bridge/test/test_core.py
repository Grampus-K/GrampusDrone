#!/usr/bin/env python3
"""Offline numerical checks: python3 -m unittest discover -s test -p test_core.py."""
import copy
import tempfile
import unittest
from pathlib import Path
import numpy as np
from openvins_bridge.core import Freshness,convert,covariance,quaternion,rotation,skew,transform
from openvins_bridge.calibration import create_config,load_yaml,validate_chain

ROOT=Path(__file__).resolve().parents[4]
UPSTREAM=ROOT/'src/third_party/open_vins'


def axis_angle(v):
    a=np.linalg.norm(v)
    return np.r_[v/a*np.sin(a/2),np.cos(a/2)] if a else np.array([0.,0.,0.,1.])


class GeometryTest(unittest.TestCase):
    def setUp(self):
        self.q=axis_angle(np.array([.7,-.4,.8]))
        self.mount=np.eye(4)
        self.mount[:3,:3]=rotation(axis_angle(np.array([1.,-.5,.3])))
        self.mount[:3,3]=[.2,-.1,.15]
        self.p=np.array([2.,3.,1.])
        self.v=np.array([.4,-.2,.1])
        self.w=np.array([.2,.3,-.1])
        rng=np.random.default_rng(17)
        a=rng.normal(size=(6,6)); self.c=a@a.T+.01*np.eye(6)

    def result(self,p=None,q=None,v=None,w=None):
        return convert(self.p if p is None else p,self.q if q is None else q,
                       self.v if v is None else v,self.w if w is None else w,
                       self.c,self.c,self.mount,yaw=.4)

    def test_quaternion_roundtrip_including_half_turn(self):
        for v in ([0,0,0],[np.pi,0,0],[0,np.pi,0],[.7,-.4,.8]):
            r=rotation(axis_angle(np.array(v)))
            np.testing.assert_allclose(rotation(quaternion(r)),r,atol=1e-12)

    def test_stationary_base_lever_arm_velocity(self):
        offset=-self.mount[:3,:3].T@self.mount[:3,3]
        result=self.result(v=-np.cross(self.w,offset))
        np.testing.assert_allclose(result[2],0,atol=1e-12)

    def test_pose_covariance_matches_numerical_local_imu_error(self):
        nominal=self.result(); r=rotation(nominal[1]); j=np.zeros((6,6)); eps=1e-6
        for k in range(6):
            dx=np.zeros(6); dx[k]=eps
            perturbed_q=quaternion(rotation(self.q)@rotation(axis_angle(dx[3:])))
            out=self.result(p=self.p+dx[:3],q=perturbed_q)
            dr=rotation(out[1])@r.T
            j[:,k]=np.r_[(out[0]-nominal[0])/eps,
                         [dr[2,1]-dr[1,2],dr[0,2]-dr[2,0],dr[1,0]-dr[0,1]]]/np.r_[np.ones(3),[2*eps]*3]
        np.testing.assert_allclose(nominal[4],j@self.c@j.T,rtol=2e-5,atol=2e-5)

    def test_twist_covariance_matches_numerical_lever_arm(self):
        nominal=self.result(); j=np.zeros((6,6)); eps=1e-6
        for k in range(6):
            dx=np.zeros(6); dx[k]=eps
            out=self.result(v=self.v+dx[:3],w=self.w+dx[3:])
            j[:,k]=np.r_[out[2]-nominal[2],out[3]-nominal[3]]/eps
        np.testing.assert_allclose(nominal[5],j@self.c@j.T,rtol=1e-8,atol=1e-8)

    def test_invalid_geometry_and_covariance_rejected(self):
        for value in (np.zeros((4,4)),np.diag([-1,1,1,1])):
            with self.assertRaises(ValueError): transform(value)
        for value in (np.zeros((6,6)),np.full((6,6),np.nan),np.eye(6)-2*np.ones((6,6))):
            with self.assertRaises(ValueError): covariance(value)


class TimeTest(unittest.TestCase):
    def test_duplicate_does_not_refresh_arrival(self):
        f=Freshness(); self.assertTrue(f.observe('x',10,10,20))
        self.assertFalse(f.observe('x',10,10.1,20.1))
        self.assertFalse(f.recent('x',10.2,20.6,.5))

    def test_bad_stamps(self):
        f=Freshness(); f.observe('x',10,10,20)
        for stamp in (0,9.9,9,10.2,float('nan')):
            with self.assertRaises(ValueError): f.observe('x',stamp,10,20)

    def test_source_and_arrival_clocks_both_checked(self):
        f=Freshness(); f.observe('x',10,10,20)
        self.assertFalse(f.recent('x',11,20.1,.5))
        self.assertFalse(f.recent('x',10.1,21,.5))


@unittest.skipUnless(UPSTREAM.is_dir(),'fetch pinned OpenVINS source first')
class CalibrationTest(unittest.TestCase):
    def setUp(self):
        # Synthetic fixture, never a deliverable calibration for a real device.
        cam=dict(T_cam_imu=np.eye(4).tolist(),camera_model='pinhole',distortion_model='radtan',
                 intrinsics=[400.,400.,424.,240.],distortion_coeffs=[0.,0.,0.,0.],
                 resolution=[848,480],timeshift_cam_imu=.003)
        self.chain={'cam0':cam,'cam1':copy.deepcopy(cam)}
        self.chain['cam1']['T_cam_imu'][0][3]=-.095
        self.imu=dict(accelerometer_noise_density=.01,accelerometer_random_walk=.001,
                      gyroscope_noise_density=.001,gyroscope_random_walk=.0001,update_rate=400.)

    def test_invalid_baseline_resolution_time_rejected(self):
        for key,value in [('T_cam_imu',np.eye(4).tolist()),('resolution',[640,480]),('timeshift_cam_imu',.02)]:
            chain=copy.deepcopy(self.chain); chain['cam1'][key]=value
            with self.assertRaises(ValueError): validate_chain(chain)

    def test_config_and_opencv_parser(self):
        # OpenVINS consumes OpenCV FileStorage, not a general YAML parser.
        import cv2
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'cfg'
            create_config(UPSTREAM,out,self.chain,self.imu,'factory_preview')
            self.assertIsNone(load_yaml(out/'rig.yaml')['T_base_imu'])
            cfg=load_yaml(out/'estimator_config.yaml')
            self.assertEqual(cfg['max_cameras'],2)
            for filename in ('estimator_config.yaml','kalibr_imucam_chain.yaml','kalibr_imu_chain.yaml'):
                fs=cv2.FileStorage(str(out/filename),cv2.FILE_STORAGE_READ)
                self.assertTrue(fs.isOpened(),filename)
                if filename.startswith('kalibr_imucam'):
                    cam=fs.getNode('cam1')
                    self.assertAlmostEqual(cam.getNode('T_cam_imu').at(0).at(3).real(),-.095)
                    self.assertEqual(cam.getNode('rostopic').string(),'/vio_inputs/right/image_raw')
                if filename=='kalibr_imu_chain.yaml':
                    self.assertEqual(fs.getNode('imu0').getNode('Tw').at(0).at(0).real(),1.)
                fs.release()
            with self.assertRaises(FileExistsError): create_config(UPSTREAM,out,self.chain,self.imu,'factory_preview')


if __name__=='__main__': unittest.main()
