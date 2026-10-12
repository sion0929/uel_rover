"""pose_math 시험 (ROS 없이 실행)."""

import math

import pytest
from spacek_drone_sim.pose_math import (compose, matrix_to_rpy, parse_pose, pose_text,
                                        quat_from_rpy, quat_rotate, rpy_to_matrix)


def test_rpy_matrix_roundtrip():
    for rpy in [(0.1, -0.2, 0.3), (0.0, 0.180938, 0.0), (-0.180938, 0.0, 3.0)]:
        assert matrix_to_rpy(rpy_to_matrix(*rpy)) == pytest.approx(rpy, abs=1e-9)


def test_compose_offsets_along_rotated_axis():
    # 부모가 y축으로 -0.180938 기울면 자식의 +z 0.003 은 -x 쪽으로 기운다
    xyz, rpy = compose(((0.15995, 0.0, 0.12535), (0.0, -0.180938, 0.0)),
                       ((0.0, 0.0, 0.003), (0.0, 0.0, 0.0)))
    assert xyz == pytest.approx((0.15995 - 0.003 * math.sin(0.180938), 0.0,
                                 0.12535 + 0.003 * math.cos(0.180938)), abs=1e-12)
    assert rpy == pytest.approx((0.0, -0.180938, 0.0), abs=1e-12)


def test_quat_matches_matrix():
    rpy = (0.3, -0.4, 1.2)
    q = quat_from_rpy(*rpy)
    m = rpy_to_matrix(*rpy)
    for v in [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]:
        expected = tuple(m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2] for i in range(3))
        assert quat_rotate(q, v) == pytest.approx(expected, abs=1e-12)


def test_pose_text_roundtrip():
    xyz, rpy = parse_pose(pose_text((1.0, 2.0, 3.0), (0.1, 0.2, 0.3)))
    assert xyz == pytest.approx((1.0, 2.0, 3.0))
    assert rpy == pytest.approx((0.1, 0.2, 0.3))
    assert parse_pose(None) == ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
