#!/usr/bin/env python3
"""Offline guard failure tests with mocked ROS transport, not a hardware test."""
import importlib.util
import sys
import unittest
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch


class CameraGuardTest(unittest.TestCase):
    def setUp(self):
        self.ros=ModuleType('rospy')
        self.ros.ROSException=type('ROSException',(Exception,),{})
        self.ros.ServiceException=type('ServiceException',(Exception,),{})
        self.stamp=SimpleNamespace(to_sec=lambda:10.)
        self.ros.Time=SimpleNamespace(now=lambda:self.stamp)
        self.ros.Publisher=Mock(side_effect=lambda *a,**k:Mock())
        self.ros.Subscriber=Mock()
        self.ros.loginfo=Mock()
        self.ros.logerr_throttle=Mock()
        self.ros.logwarn_throttle=Mock()
        filters=ModuleType('message_filters')
        filters.Subscriber=Mock()
        filters.ApproximateTimeSynchronizer=Mock()
        dynamic=ModuleType('dynamic_reconfigure')
        dynamic.DynamicReconfigureException=type('DynamicReconfigureException',(Exception,),{})
        self.dynamic_error=dynamic.DynamicReconfigureException
        client_module=ModuleType('dynamic_reconfigure.client')
        self.client=Mock()
        self.client.get_parameter_descriptions.return_value=[{'name':'emitter_enabled'}]
        self.client.update_configuration.return_value={'emitter_enabled':0}
        self.client_factory=Mock(return_value=self.client)
        client_module.Client=self.client_factory
        sensors=ModuleType('sensor_msgs.msg')
        sensors.Image=sensors.Imu=SimpleNamespace
        standard=ModuleType('std_msgs.msg')
        standard.Bool=SimpleNamespace
        modules={'rospy':self.ros,'message_filters':filters,
                 'dynamic_reconfigure':dynamic,'dynamic_reconfigure.client':client_module,
                 'sensor_msgs':ModuleType('sensor_msgs'),'sensor_msgs.msg':sensors,
                 'std_msgs':ModuleType('std_msgs'),'std_msgs.msg':standard}
        script=Path(__file__).resolve().parents[1]/'scripts/camera_guard.py'
        spec=importlib.util.spec_from_file_location('camera_guard_under_test',script)
        self.module=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,modules):
            spec.loader.exec_module(self.module)

    def send_samples(self,node):
        header=SimpleNamespace(stamp=self.stamp,frame_id='sensor')
        image=SimpleNamespace(header=header,width=848,height=480,encoding='mono8',
                              step=848,data=bytes(848*480))
        vector=SimpleNamespace(x=0.,y=0.,z=0.)
        imu=SimpleNamespace(header=header,angular_velocity=vector,linear_acceleration=vector)
        node.images(image,image)
        node.on_imu(imu)
        node.tick()

    def assert_blocked(self,node):
        self.assertTrue(node.failed)
        self.assertFalse(node.enabled)
        # Even a late configuration callback cannot clear a startup fault.
        node.configuration({'emitter_enabled':0})
        self.send_samples(node)
        self.assertFalse(node.enabled)
        node.left.publish.assert_not_called()
        node.right.publish.assert_not_called()
        node.imu.publish.assert_not_called()
        self.assertFalse(node.ready.publish.call_args[0][0].data)
        self.ros.logerr_throttle.assert_called()

    def test_missing_emitter_parameter_stays_alive_and_blocks_data(self):
        self.client.get_parameter_descriptions.return_value=[{'name':'gain'}]
        node=self.module.CameraGuard()
        self.client.update_configuration.assert_not_called()
        self.assert_blocked(node)

    def test_service_discovery_timeout_stays_blocked(self):
        self.client_factory.side_effect=self.ros.ROSException('service unavailable')
        self.assert_blocked(self.module.CameraGuard())

    def test_parameter_description_timeout_stays_blocked(self):
        self.client.get_parameter_descriptions.return_value=None
        node=self.module.CameraGuard()
        self.client.update_configuration.assert_not_called()
        self.assert_blocked(node)

    def test_update_failures_stay_blocked(self):
        for error in (self.dynamic_error("don't know parameter: emitter_enabled"),
                      self.ros.ServiceException('device disconnected')):
            with self.subTest(error=type(error).__name__):
                self.client.update_configuration.side_effect=error
                self.assert_blocked(self.module.CameraGuard())

    def test_unconfirmed_emitter_stays_blocked(self):
        for response in (None,{}, {'emitter_enabled':1}):
            with self.subTest(response=response):
                self.client.update_configuration.return_value=response
                self.assert_blocked(self.module.CameraGuard())

    def test_confirmed_emitter_and_fresh_streams_become_ready(self):
        node=self.module.CameraGuard()
        self.client.update_configuration.assert_called_once_with({'emitter_enabled':0})
        self.assertFalse(node.failed)
        self.send_samples(node)
        node.left.publish.assert_called_once()
        node.right.publish.assert_called_once()
        node.imu.publish.assert_called_once()
        self.assertTrue(node.ready.publish.call_args[0][0].data)
        # Runtime emitter changes still latch a fault after readiness.
        node.configuration({'emitter_enabled':1})
        self.assertTrue(node.failed)
        node.configuration({'emitter_enabled':0})
        self.send_samples(node)
        node.left.publish.assert_called_once()
        node.right.publish.assert_called_once()
        node.imu.publish.assert_called_once()
        self.assertFalse(node.ready.publish.call_args[0][0].data)


if __name__=='__main__':
    unittest.main()
