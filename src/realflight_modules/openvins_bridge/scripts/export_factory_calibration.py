#!/usr/bin/env python3
"""Export actual ROS CameraInfo + factory TF; intended ONLY for preview."""
import argparse
from pathlib import Path
import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import CameraInfo, Image, Imu
from openvins_bridge.calibration import create_config, load_yaml
from openvins_bridge.core import rotation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--upstream',required=True)
    parser.add_argument('--output',required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('export_d455_factory_calibration',anonymous=True)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    imu_msg = rospy.wait_for_message('/d455/imu',Imu,timeout=20)
    chain, frames = {},{}
    for idx in (0,1):
        msg=rospy.wait_for_message('/d455/infra%d/camera_info'%(idx+1),CameraInfo,timeout=20)
        image=rospy.wait_for_message('/d455/infra%d/image_rect_raw'%(idx+1),Image,timeout=20)
        if (image.width,image.height)!=(msg.width,msg.height) or not image.header.frame_id:
            raise ValueError('Image and CameraInfo geometry disagree')
        if msg.distortion_model != 'plumb_bob' or len(msg.D)<4 or any(abs(x)>1e-10 for x in msg.D[4:]):
            raise ValueError('Factory distortion cannot be represented by OpenVINS radtan4; use Kalibr')
        if not np.allclose(np.asarray(msg.R).reshape(3,3),np.eye(3),atol=1e-6):
            raise ValueError('Non-identity rectification requires explicit image/extrinsic treatment; use Kalibr')
        # ROS1 realsense-ros sets RIGHT CameraInfo.frame_id to the LEFT frame
        # for stereo projection. Image.frame_id retains the actual optical frame.
        tf=buffer.lookup_transform(image.header.frame_id,imu_msg.header.frame_id,rospy.Time(0),rospy.Duration(10)).transform
        q,t=tf.rotation,tf.translation
        mat=np.eye(4)
        mat[:3,:3]=rotation([q.x,q.y,q.z,q.w])
        mat[:3,3]=[t.x,t.y,t.z]
        chain['cam'+str(idx)]=dict(T_cam_imu=mat.tolist(),camera_model='pinhole',distortion_model='radtan',
            intrinsics=[msg.K[0],msg.K[4],msg.K[2],msg.K[5]],distortion_coeffs=list(msg.D[:4]),
            resolution=[msg.width,msg.height],timeshift_cam_imu=0.)
        frames['cam'+str(idx)]=image.header.frame_id
    # Deliberate provisional tuning from the pinned upstream example, NOT this device's measurement.
    imu=load_yaml(Path(args.upstream)/'config/rs_d455/kalibr_imu_chain.yaml')
    result=create_config(args.upstream,args.output,chain,imu,'factory_preview',metadata=dict(
        source='Live D455 CameraInfo and factory optical-frame TF',camera_frames=frames,
        imu_frame=imu_msg.header.frame_id,temporal_offset='ZERO ASSUMPTION: not calibrated',
        imu_noise='Provisional values from pinned OpenVINS rs_d455 example, not measured on this device'))
    rospy.logwarn('Factory-only preview written to %s. Joint calibration remains required.',result)


if __name__=='__main__':
    main()
