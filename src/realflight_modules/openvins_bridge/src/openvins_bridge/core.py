"""Pure geometry and time checks for the pinned OpenVINS ROS1 odomimu output.

OpenVINS: pose [dp_global, dtheta_imu], twist [v_imu, omega_imu].
Hamilton xyzw in the ROS message already represents IMU -> global. Do NOT
conjugate it again because the filter internally uses a JPL quaternion.
"""
import math
import numpy as np


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def rotation(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > .01:
        raise ValueError('invalid xyzw quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def quaternion(r):
    # Symmetric eigenproblem is well behaved at 180 degrees too.
    k = np.array([[r[0,0]-r[1,1]-r[2,2], r[1,0]+r[0,1], r[2,0]+r[0,2], r[2,1]-r[1,2]],
                  [r[1,0]+r[0,1], r[1,1]-r[0,0]-r[2,2], r[2,1]+r[1,2], r[0,2]-r[2,0]],
                  [r[2,0]+r[0,2], r[2,1]+r[1,2], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
                  [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], np.trace(r)]]) / 3
    q = np.linalg.eigh(k)[1][:, -1]
    return q if q[3] >= 0 else -q


def transform(value):
    t = np.asarray(value, dtype=float)
    if (t.shape != (4,4) or not np.isfinite(t).all() or
            not np.allclose(t[3], [0,0,0,1], atol=1e-8) or
            not np.allclose(t[:3,:3].T @ t[:3,:3], np.eye(3), atol=1e-6) or
            abs(np.linalg.det(t[:3,:3])-1) > 1e-6):
        raise ValueError('expected a finite rigid 4x4 transform')
    return t


def covariance(value):
    c = np.asarray(value, dtype=float).reshape(6,6)
    if not np.isfinite(c).all() or np.any(np.diag(c) <= 0):
        raise ValueError('missing or invalid covariance')
    scale = max(1., np.max(np.abs(c)))
    if np.max(np.abs(c-c.T)) > 1e-7*scale:
        raise ValueError('asymmetric covariance')
    c = (c+c.T)/2
    eig, vec = np.linalg.eigh(c)
    if eig.min() < -1e-9*scale:
        raise ValueError('non-positive-semidefinite covariance')
    return (vec * np.maximum(eig, 0)) @ vec.T


def convert(p, q, v, w, cp, ct, t_bi, yaw=0., origin=None):
    """T_B_I maps IMU into base_link. Output body twist; fixed-world pose error.

    Upstream's approximate angular covariance and missing pose/twist cross-block
    correlations remain limitations; this function does not invent correlations.
    """
    t_bi = transform(t_bi)
    p, v, w = [np.asarray(a, dtype=float) for a in (p,v,w)]
    if any(a.shape != (3,) or not np.isfinite(a).all() for a in (p,v,w)):
        raise ValueError('non-finite position or velocity')
    r_wi = rotation(q)
    r_bi = t_bi[:3,:3]
    r_ib = r_bi.T
    offset_i = -r_ib @ t_bi[:3,3]
    c, s = math.cos(yaw), math.sin(yaw)
    r_aw = np.array([[c,-s,0],[s,c,0],[0,0,1.]])
    pos = r_aw @ (p + r_wi @ offset_i)
    if origin is not None:
        pos = pos - np.asarray(origin)
    j_pose = np.zeros((6,6))
    j_pose[:3,:3] = r_aw
    j_pose[:3,3:] = -r_aw @ r_wi @ skew(offset_i)
    j_pose[3:,3:] = r_aw @ r_wi
    j_twist = np.zeros((6,6))
    j_twist[:3,:3] = r_bi
    j_twist[:3,3:] = -r_bi @ skew(offset_i)
    j_twist[3:,3:] = r_bi
    return (pos, quaternion(r_aw @ r_wi @ r_ib),
            r_bi @ (v + np.cross(w,offset_i)), r_bi @ w,
            j_pose @ covariance(cp) @ j_pose.T,
            j_twist @ covariance(ct) @ j_twist.T)


class Freshness:
    """Source and monotonic arrival timestamps; duplicates never refresh age."""
    def __init__(self, max_age=.5, future=.05):
        self.max_age, self.future = max_age, future
        self.samples = {}

    def observe(self, name, stamp, now, steady):
        if not all(math.isfinite(x) for x in (stamp,now,steady)):
            raise ValueError(name + ': invalid time')
        if stamp <= 0 or now-stamp > self.max_age or stamp-now > self.future:
            raise ValueError(name + ': stale, zero or future timestamp')
        previous = self.samples.get(name)
        if previous and stamp < previous[0]:
            raise ValueError(name + ': timestamp moved backwards')
        if previous and stamp == previous[0]:
            return False
        self.samples[name] = (stamp,steady)
        return True

    def recent(self, name, now, steady, timeout):
        if name not in self.samples:
            return False
        stamp, arrival = self.samples[name]
        return (-self.future <= now-stamp <= timeout and 0 <= steady-arrival <= timeout)
