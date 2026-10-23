#!/usr/bin/env python3
"""ROS transport test. Run on Noetic: rostest openvins_bridge bridge.test."""
import time
import threading
import unittest
import rospy
import rostest
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool,String


class PreviewTest(unittest.TestCase):
    def test_stamp_and_latched_timeout(self):
        self.outputs=[]; self.status=''; self.sent=set(); self.lock=threading.Lock()
        def receive(msg):
            with self.lock: self.outputs.append(msg)
        def status(msg): self.status=msg.data
        subscribers=[rospy.Subscriber('/test/preview',Odometry,receive),
                     rospy.Subscriber('/test_bridge/status',String,status)]
        odom=rospy.Publisher('/test/odom',Odometry,queue_size=5)
        pose=rospy.Publisher('/test/pose',PoseWithCovarianceStamped,queue_size=5)
        ready=rospy.Publisher('/test/camera_ready',Bool,queue_size=1)
        deadline=time.monotonic()+5
        while time.monotonic()<deadline and not all(p.get_num_connections() for p in (odom,pose,ready)):
            time.sleep(.02)
        self.assertTrue(all(p.get_num_connections() for p in (odom,pose,ready)))
        def sample():
            stamp=rospy.Time.now()-rospy.Duration(.02)
            msg=Odometry(); msg.header.stamp=stamp; msg.header.frame_id='global'; msg.child_frame_id='imu'
            msg.pose.pose.orientation.w=1
            msg.pose.covariance=[.01 if i%7==0 else 0. for i in range(36)]
            msg.twist.covariance=list(msg.pose.covariance)
            camera=PoseWithCovarianceStamped(); camera.header=msg.header
            self.sent.add(stamp.to_nsec())
            ready.publish(Bool(data=True)); pose.publish(camera); odom.publish(msg)
        for _ in range(70): sample(); time.sleep(.02)
        self.assertGreater(len(self.outputs),5,self.status)
        for msg in self.outputs:
            self.assertIn(msg.header.stamp.to_nsec(),self.sent)
            self.assertEqual((msg.header.frame_id,msg.child_frame_id),('vio_preview_odom','vio_preview_imu'))
        time.sleep(.8)
        self.assertTrue(self.status.startswith('FAULT:'),self.status)
        time.sleep(.1); count=len(self.outputs)
        for _ in range(30): sample(); time.sleep(.02)
        self.assertEqual(count,len(self.outputs),'fault must remain latched when streams resume')


if __name__=='__main__':
    rospy.init_node('test_bridge_ros')
    rostest.rosrun('openvins_bridge','preview_stamp_timeout',PreviewTest)
