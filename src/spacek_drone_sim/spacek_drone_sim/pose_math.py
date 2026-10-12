"""자세 계산 도우미 (ROS 비의존).

rpy = 고정축 X-Y-Z 회전 (SDF 와 같음, R = Rz(yaw) Ry(pitch) Rx(roll)), 쿼터니언 = (x, y, z, w).
자세 = (xyz, rpy) 튜플 쌍.
"""

import math


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def matrix_to_rpy(m):
    pitch = math.asin(max(-1.0, min(1.0, -m[2][0])))
    if abs(m[2][0]) < 1.0 - 1e-9:
        return math.atan2(m[2][1], m[2][2]), pitch, math.atan2(m[1][0], m[0][0])
    # 짐벌락 (pitch = ±90°): yaw 를 0 으로 둔다
    return math.atan2(-m[1][2], m[1][1]), pitch, 0.0


def mat_mul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))


def mat_vec(m, v):
    return tuple(sum(m[i][k] * v[k] for k in range(3)) for i in range(3))


def compose(a, b):
    """자세 a (부모 기준) 위에 자세 b (a 기준) 를 얹은 자세."""
    ra = rpy_to_matrix(*a[1])
    xyz = tuple(p + q for p, q in zip(a[0], mat_vec(ra, b[0])))
    return xyz, matrix_to_rpy(mat_mul(ra, rpy_to_matrix(*b[1])))


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_rotate(q, v):
    x, y, z, w = q
    r = quat_mul(quat_mul(q, (v[0], v[1], v[2], 0.0)), (-x, -y, -z, w))
    return r[:3]


def pose_text(xyz, rpy=(0.0, 0.0, 0.0)):
    return ' '.join('%.9g' % float(v) for v in tuple(xyz) + tuple(rpy))


def parse_pose(text):
    vals = [float(t) for t in (text or '').split()] + [0.0] * 6
    return tuple(vals[:3]), tuple(vals[3:6])
