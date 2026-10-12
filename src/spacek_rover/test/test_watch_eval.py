"""watch_eval 판정 시험 (ROS/Gazebo 없이 실행 가능)."""

import math

import pytest
from spacek_rover.watch_eval import WatchTracker

LIMITS = (0.005, 1.0, 0.5)
Z = -0.2196


def _run(poses, settle_s=2.0, dt=0.02):
    tr = WatchTracker(settle_s)
    for i, pose in enumerate(poses):
        tr.update(i * dt, pose)
    return tr.result(*LIMITS)


def _still(n, yaw=0.0):
    return [(0.0, 0.0, Z, 0.0, 0.0, yaw)] * n


def test_still_rover_passes():
    res = _run(_still(300))
    assert res['passed'], res
    assert res['measured']['samples'] >= 199


def test_motion_during_settle_is_ignored():
    poses = [(0.0003 * i, 0.0, Z, 0.0, 0.0, 0.0) for i in range(100)]
    poses += [(0.03, 0.0, Z, 0.0, 0.0, 0.0)] * 200
    assert _run(poses)['passed']


def test_shift_after_settle_fails():
    poses = _still(150) + [(0.006, 0.0, Z, 0.0, 0.0, 0.0)] * 50
    res = _run(poses)
    assert not res['checks']['shift']
    assert res['measured']['max_shift_mm'] == pytest.approx(6.0)


def test_tilt_after_settle_fails():
    poses = _still(150) + [(0.0, 0.0, Z, math.radians(1.5), 0.0, 0.0)] * 50
    assert not _run(poses)['checks']['tilt']


def test_yaw_wrap_near_pi_is_not_a_turn():
    poses = [(0.0, 0.0, Z, 0.0, 0.0, math.pi - 1e-4 if i % 2 else -math.pi + 1e-4)
             for i in range(300)]
    res = _run(poses)
    assert res['checks']['yaw'], res
    assert res['measured']['max_yaw_change_deg'] < 0.05


def test_no_samples_after_settle_fails():
    res = _run(_still(50))
    assert not res['passed']
    assert not res['checks']['settled']


def test_start_freezes_reference_inside_settle_window():
    # 드론 스폰(start) 직후 착지 충격이 settle_s 안에 끝나도 놓치지 않아야 한다
    tr = WatchTracker(settle_s=10.0)
    for i in range(20):
        tr.update(i * 0.02, (0.0, 0.0, Z, 0.0, 0.0, 0.0))
    tr.start()
    for i in range(20, 70):
        tr.update(i * 0.02, (0.006, 0.0, Z, 0.0, 0.0, 0.0))
    res = tr.result(*LIMITS)
    assert res['checks']['settled']
    assert not res['checks']['shift']
    assert res['measured']['max_shift_mm'] == pytest.approx(6.0)


def test_start_before_first_pose_uses_first_pose_as_reference():
    tr = WatchTracker(settle_s=10.0)
    tr.start()
    tr.update(0.0, (0.0, 0.0, Z, 0.0, 0.0, 0.0))
    tr.update(0.02, (0.002, 0.0, Z, 0.0, 0.0, 0.0))
    res = tr.result(*LIMITS)
    assert res['passed'], res
    assert res['measured']['samples'] == 2
