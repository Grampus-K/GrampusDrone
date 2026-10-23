#!/usr/bin/env python3
"""Preview or disarmed PX4 ground input. Deliberately no flight-enable switch."""
import copy
import math
import threading
import time
import numpy as np
import rospy
import rosgraph
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from openvins_bridge.core import Freshness, convert, rotation, transform
from openvins_bridge.calibration import load_yaml

TF_ERRORS = (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException)


class Bridge:
    def __init__(self):
        self.lock = threading.RLock()
        self.mode = rospy.get_param('~mode', 'preview')
        if self.mode not in ('preview', 'ground'):
            raise ValueError('mode must be preview or ground; flight is not implemented')
        self.rig = load_yaml(rospy.get_param('~rig_file'))
        self.mount = self.rig.get('T_base_imu')
        if self.mode == 'ground' and (self.rig.get('calibration_status') != 'kalibr' or self.mount is None):
            raise ValueError('ground output requires imported Kalibr calibration and measured T_base_imu')
        self.mount = transform(self.mount) if self.mount is not None else np.eye(4)
        self.values = {}
        defaults = dict(max_age=.5,future_tolerance=.05,visual_timeout=.5,odom_timeout=.5,
                        state_timeout=3.,stable_seconds=3.,stationary_speed=.35,stationary_angular=.2,
                        stationary_drift=.25,max_speed=20.,max_angular=12.,max_step=1.,max_angle_step=.35,
                        max_position=10000.,max_output_rate=30.)
        for name, default in defaults.items():
            value = float(rospy.get_param('~'+name,default))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' must be positive and finite')
            self.values[name] = value
        self.yaw = float(rospy.get_param('~reference_yaw',0.))
        if not math.isfinite(self.yaw):
            raise ValueError('invalid reference_yaw')
        self.zero = rospy.get_param('~zero_position',True)
        self.fresh = Freshness(self.values['max_age'],self.values['future_tolerance'])
        self.fault = None
        self.ready = False
        self.started = False
        self.state, self.state_at = None, 0.
        self.camera_ready, self.camera_at = False, 0.
        self.anchor, self.origin, self.previous = None, None, None
        self.last_clock, self.last_send = None, 0.
        self.last_sender_check = 0.
        self.master = rosgraph.Master(rospy.get_name())
        self.last_status = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.input = rospy.resolve_name(rospy.get_param('~input_topic','/ov_msckf/odomimu'))
        topic = rospy.resolve_name(rospy.get_param('~preview_topic','/openvins_bridge/odometry_preview')
                                   if self.mode == 'preview' else rospy.get_param('~output_topic','/mavros/odometry/out'))
        if topic == self.input or (self.mode == 'preview' and topic.startswith('/mavros/')):
            raise ValueError('unsafe output topic/remapping')
        if self.mode == 'ground' and topic != rospy.resolve_name('/mavros/odometry/out'):
            raise ValueError('ground mode only supports /mavros/odometry/out')
        self.pub = rospy.Publisher(topic,Odometry,queue_size=3)
        self.status = rospy.Publisher('~status',String,queue_size=1,latch=True)
        self.health = rospy.Publisher('~ready',Bool,queue_size=1,latch=True)
        self.subs = [rospy.Subscriber(self.input,Odometry,self.on_odom,queue_size=5,tcp_nodelay=True),
                     rospy.Subscriber(rospy.get_param('~pose_topic','/ov_msckf/poseimu'),PoseWithCovarianceStamped,self.on_pose,queue_size=5),
                     rospy.Subscriber(rospy.get_param('~camera_ready_topic','/d455_input/ready'),Bool,self.on_camera,queue_size=1),
                     rospy.Subscriber('/mavros/state',State,self.on_state,queue_size=1)]
        self.report('WAITING: camera streams, estimator poses and stationary initialization')

    def report(self, text):
        if text != self.last_status:
            self.status.publish(String(data=text))
            self.health.publish(Bool(data=self.ready and self.fault is None))
            rospy.loginfo(text)
            self.last_status = text

    def fail(self, text):
        if self.fault is None:
            self.fault = text
            self.ready = False
            self.report('FAULT: '+text+'; inspect and restart on ground')

    def on_state(self, msg):
        with self.lock:
            self.state, self.state_at = msg, time.monotonic()
            if self.mode == 'ground' and msg.armed:
                self.fail('ground-test bridge cannot run armed')
            if self.mode == 'ground' and self.started and not msg.connected:
                self.fail('MAVROS disconnected')

    def on_camera(self, msg):
        with self.lock:
            self.camera_ready, self.camera_at = msg.data,time.monotonic()
            if self.started and not msg.data:
                self.fail('camera input guard failed')

    def on_pose(self, msg):
        with self.lock:
            try:
                if msg.header.frame_id != 'global':
                    raise ValueError('expected OpenVINS poseimu global frame')
                self.fresh.observe('pose',msg.header.stamp.to_sec(),rospy.Time.now().to_sec(),time.monotonic())
            except ValueError as e:
                self.fail(str(e))

    def prerequisites(self, now, steady):
        if not self.camera_ready or steady-self.camera_at > self.values['visual_timeout']:
            return False
        # poseimu proves a new camera-time estimator state, NOT a successful feature update.
        if not self.fresh.recent('pose',now,steady,self.values['visual_timeout']):
            return False
        if self.mode == 'ground':
            if self.state is None or not self.state.connected or self.state.armed or steady-self.state_at > self.values['state_timeout']:
                return False
        return True

    def check_tf(self):
        for target, source, expected in [('odom_ned','odom',np.array([[0,1,0],[1,0,0],[0,0,-1]])),
                                          ('base_link_frd','base_link',np.diag([1,-1,-1]))]:
            tf = self.tf_buffer.lookup_transform(target,source,rospy.Time(0)).transform
            q, t = tf.rotation,tf.translation
            if (not np.allclose(rotation([q.x,q.y,q.z,q.w]),expected,atol=1e-6) or
                    np.linalg.norm([t.x,t.y,t.z]) > 1e-6):
                raise ValueError('MAVROS TF must apply axis conversions exactly once')

    def check_time(self, now, steady):
        if self.last_clock is not None:
            if abs((now-self.last_clock[0])-(steady-self.last_clock[1])) > .5:
                self.fail('ROS clock jumped')
        self.last_clock = (now,steady)

    def tick(self):
        with self.lock:
            now, steady = rospy.Time.now().to_sec(),time.monotonic()
            self.check_time(now,steady)
            if self.mode == 'ground' and not self.fault and steady-self.last_sender_check > .5:
                self.last_sender_check = steady
                try:
                    publishers = self.master.getSystemState()[0]
                    ev_topics = {'/mavros/odometry/out','/mavros/vision_pose/pose',
                                 '/mavros/vision_pose/pose_cov','/mavros/vision_speed/speed_twist_cov',
                                 '/mavros/vision_speed/speed_twist','/mavros/vision_speed/speed_vector'}
                    for topic, nodes in publishers:
                        if topic in ev_topics and any(node != rospy.get_name() for node in nodes):
                            self.fail('another external-vision sender is registered: '+topic)
                except (OSError,rosgraph.MasterException,rosgraph.MasterError) as error:
                    self.fail('cannot verify exclusive external-vision sender: '+str(error))
            if self.started and (not self.prerequisites(now,steady) or
                    not self.fresh.recent('odom',now,steady,self.values['odom_timeout'])):
                self.fail('input, camera-state or MAVROS heartbeat expired')

    def on_odom(self, msg):
        with self.lock:
            self.tick()  # Catch a lapse BEFORE a queued sample can hide it.
            if self.fault:
                return
            now, steady = rospy.Time.now().to_sec(),time.monotonic()
            try:
                if (msg.header.frame_id,msg.child_frame_id) != ('global','imu'):
                    raise ValueError('expected pinned OpenVINS global/imu frames')
                if not self.fresh.observe('odom',msg.header.stamp.to_sec(),now,steady):
                    return
                if not self.prerequisites(now,steady):
                    self.anchor = None
                    return
                if self.mode == 'ground':
                    self.check_tf()
                p,q = msg.pose.pose.position,msg.pose.pose.orientation
                v,w = msg.twist.twist.linear,msg.twist.twist.angular
                out = convert([p.x,p.y,p.z],[q.x,q.y,q.z,q.w],[v.x,v.y,v.z],[w.x,w.y,w.z],
                              msg.pose.covariance,msg.twist.covariance,self.mount,self.yaw)
                pos,quat,vel,ang,cp,ct = out
                if (np.linalg.norm(vel)>self.values['max_speed'] or np.linalg.norm(ang)>self.values['max_angular'] or
                        np.linalg.norm(pos)>self.values['max_position']):
                    raise ValueError('position or velocity outside configured limits')
                stamp = msg.header.stamp.to_sec()
                if self.previous is not None:
                    old_stamp, old_pos, old_r = self.previous
                    dt = stamp-old_stamp
                    angle = math.acos(float(np.clip((np.trace(old_r.T @ rotation(quat))-1)/2,-1,1)))
                    if (np.linalg.norm(pos-old_pos)>self.values['max_step']+self.values['max_speed']*dt or
                            angle>self.values['max_angle_step']+self.values['max_angular']*dt):
                        raise ValueError('estimator pose jumped')
                self.previous = (stamp,pos,rotation(quat))
                if not self.ready:
                    if (self.anchor is None or np.linalg.norm(vel)>self.values['stationary_speed'] or
                            np.linalg.norm(ang)>self.values['stationary_angular'] or
                            np.linalg.norm(pos-self.anchor[1])>self.values['stationary_drift']):
                        self.anchor = (steady,pos)
                        return
                    if steady-self.anchor[0]<self.values['stable_seconds']:
                        return
                    self.origin = pos.copy() if self.zero else np.zeros(3)
                    self.ready = True
                if steady-self.last_send < 1/self.values['max_output_rate']:
                    return
                result = copy.deepcopy(msg)
                result.header.frame_id = 'vio_preview_odom' if self.mode=='preview' else 'odom'
                result.child_frame_id = ('vio_preview_base_link' if self.rig.get('T_base_imu') is not None else 'vio_preview_imu') if self.mode=='preview' else 'base_link'
                for obj, data, fields in [(result.pose.pose.position,pos-self.origin,'xyz'),
                                           (result.pose.pose.orientation,quat,'xyzw'),
                                           (result.twist.twist.linear,vel,'xyz'),(result.twist.twist.angular,ang,'xyz')]:
                    for field,value in zip(fields,data):
                        setattr(obj,field,float(value))
                result.pose.covariance = cp.ravel().tolist()
                result.twist.covariance = ct.ravel().tolist()
                self.pub.publish(result)
                self.started,self.last_send = True,steady
                self.report('READY: '+self.mode+' only; stream checks are NOT visual accuracy/flight health')
            except (ValueError,) + TF_ERRORS as e:
                if isinstance(e,TF_ERRORS) and not self.started:
                    self.anchor = None
                    self.report('WAITING: MAVROS axis-conversion TF')
                else:
                    self.fail(str(e))


if __name__ == '__main__':
    rospy.init_node('openvins_bridge')
    try:
        bridge = Bridge()
        # Wall-time checks continue even if ROS /clock stops.
        while not rospy.is_shutdown():
            bridge.tick()
            time.sleep(.02)
    except (ValueError,KeyError,OSError) as error:
        rospy.logfatal(str(error))
        raise SystemExit(1)
