"""drone.yaml 의 측정값과 메시 시험 (Task 2 SolidWorks 측정 결과를 고정)."""

import math
import os
import struct

import pytest
from spacek_drone_sim.drone_params import derive, load_config
from spacek_drone_sim.pose_math import quat_from_rpy, quat_rotate

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(PKG_DIR, 'config', 'drone.yaml')
MESH_DIR = os.path.join(PKG_DIR, 'models', 'spacek_drone', 'meshes')
LOWER_FRAME_HALF = 0.0915          # 하부 프레임 앞면 |x| (6S5200: x -92.49..91.49 mm)
BATTERY_FRONT_X = 0.0814           # 배터리 케이스 앞면 (전방 카메라 = 앞 양 다리 사이 가운데, 사용자)
CENTER_BLOCK_HALF = 0.0175         # 가운데 클립 블록 (하방 카메라 자리)


def _triangles(path, scale=0.001):
    data = open(path, 'rb').read()
    n = struct.unpack('<I', data[80:84])[0]
    for i in range(n):
        v = struct.unpack('<12f', data[84 + 50 * i:84 + 50 * i + 48])
        yield [tuple(c * scale for c in v[3 * k + 3:3 * k + 6]) for k in range(3)]


def _ray_hits(origin, direction, tris, max_dist):
    """Moller-Trumbore. origin 에서 direction 으로 max_dist 안에 삼각형이 있으면 True."""
    d = direction
    for a, b, c in tris:
        e1 = [b[i] - a[i] for i in range(3)]
        e2 = [c[i] - a[i] for i in range(3)]
        h = (d[1] * e2[2] - d[2] * e2[1], d[2] * e2[0] - d[0] * e2[2], d[0] * e2[1] - d[1] * e2[0])
        det = sum(e1[i] * h[i] for i in range(3))
        if abs(det) < 1e-12:
            continue
        s = [origin[i] - a[i] for i in range(3)]
        u = sum(s[i] * h[i] for i in range(3)) / det
        if u < 0.0 or u > 1.0:
            continue
        q = (s[1] * e1[2] - s[2] * e1[1], s[2] * e1[0] - s[0] * e1[2],
             s[0] * e1[1] - s[1] * e1[0])
        v = sum(d[i] * q[i] for i in range(3)) / det
        if v < 0.0 or u + v > 1.0:
            continue
        t = sum(e2[i] * q[i] for i in range(3)) / det
        if 1e-4 < t < max_dist:
            return True
    return False


def _drone_tris():
    cfg = load_config(CFG)
    yaw = float(cfg['visual']['drone_mesh_yaw_rad'])
    c, s = math.cos(yaw), math.sin(yaw)
    return [[(c * x - s * y, s * x + c * y, z) for x, y, z in t]
            for t in _triangles(os.path.join(MESH_DIR, 'drone.stl'))]


def test_meshes_exist_with_lowercase_names():
    names = os.listdir(MESH_DIR)
    assert 'drone.stl' in names and 'front_cam_case_60.stl' in names


def test_front_camera_looks_forward_60deg_down():
    """60° 케이스 = 수평(0°)에서 아래로 60° 내린 설계 (사용자 2026-10-12)."""
    _xyz, q = derive(load_config(CFG))['cameras']['front_camera']
    axis = quat_rotate(q, (0.0, 0.0, 1.0))
    assert axis[0] > 0.45
    assert abs(axis[1]) < 0.02
    assert math.degrees(math.asin(-axis[2])) == pytest.approx(60.0, abs=0.5)


# 케이스 부품 좌표 (front_cam_case_60.stl): 렌즈판 법선과 T자 클립 방향
CASE_LENS_NORMAL = (-0.866025, 0.0, 0.5)
CASE_CLIP_DIR = (0.0, 0.0, -1.0)


def test_front_case_clips_onto_battery_front_and_matches_camera():
    """클립은 뒤(배터리 케이스 앞면 쪽)를 향하고, 렌즈판 법선 = 카메라 광축."""
    fc = load_config(CFG)['sensors']['front_camera']
    q_case = quat_from_rpy(*fc['case_rpy_rad'])
    assert quat_rotate(q_case, CASE_CLIP_DIR) == pytest.approx((-1.0, 0.0, 0.0), abs=1e-6)
    _xyz, q_cam = derive(load_config(CFG))['cameras']['front_camera']
    assert quat_rotate(q_case, CASE_LENS_NORMAL) == pytest.approx(
        quat_rotate(q_cam, (0.0, 0.0, 1.0)), abs=1e-4)
    # 케이스 몸통 밑면(부품 z 13.5 mm) 이 배터리 케이스 앞면에 닿는다
    body = quat_rotate(q_case, (0.0, 0.0, 0.0135))
    assert fc['case_xyz_m'][0] + body[0] == pytest.approx(BATTERY_FRONT_X, abs=1e-4)


def test_front_camera_is_on_the_front():
    cfg = load_config(CFG)
    xyz = cfg['sensors']['front_camera']['xyz_m']
    assert BATTERY_FRONT_X < xyz[0] < BATTERY_FRONT_X + 0.05   # 앞면에 붙은 케이스의 렌즈
    assert abs(xyz[1]) < 0.005
    assert -0.0919 < xyz[2] < 0.0                               # 배터리 케이스 밑면보다 위
    case_x = cfg['sensors']['front_camera']['case_xyz_m'][0]
    assert case_x > CENTER_BLOCK_HALF            # 케이스가 하방 카메라 블록보다 앞


def test_down_sensors_inside_lower_frame():
    cfg = load_config(CFG)
    for name in ('down_camera', 'flow', 'lidar'):
        x, y, z = cfg['sensors'][name]['xyz_m']
        assert abs(x) < LOWER_FRAME_HALF and abs(y) < LOWER_FRAME_HALF, name
        assert cfg['gear_bottom_z_m'] < z < 0.0, name


def test_sensors_see_past_the_drone_body():
    cfg = load_config(CFG)
    p = derive(cfg)
    tris = _drone_tris()
    down = (0.0, 0.0, -1.0)
    for name in ('flow', 'lidar'):
        assert not _ray_hits(cfg['sensors'][name]['xyz_m'], down, tris, 1.0), name
    for name in ('front_camera', 'down_camera'):
        xyz, q = p['cameras'][name]
        assert not _ray_hits(xyz, quat_rotate(q, (0.0, 0.0, 1.0)), tris, 1.0), name
