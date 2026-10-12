"""rover_watch 판정: 착륙 중 로버가 밀리거나 기울었는지 (ROS 비의존, 단위시험 대상).

자세 = (x, y, z, roll, pitch, yaw): /rover/odom_gt, 월드 좌표, m / rad.
settle_s 동안은 안착 시간으로 보고, 그 뒤 첫 자세를 기준으로 최대 변화를 추적한다.
start() 를 부르면 settle_s 를 기다리지 않고 그 순간의 자세(마지막으로 받은 자세, 없으면 다음 자세)를
기준으로 삼는다 (예: 드론 스폰 시점).
"""

import math

from spacek_rover.drive_eval import wrap_pi


class WatchTracker:
    """기준 자세 대비 수평 이동, 기울기 변화, yaw 변화의 최대값을 추적한다."""

    def __init__(self, settle_s):
        self.settle_s = float(settle_s)
        self.t0 = None
        self.ref = None
        self.samples = 0
        self.max_shift = 0.0
        self.max_tilt = 0.0
        self.max_dyaw = 0.0
        self.started = False
        self.last = None

    def start(self):
        """안착 대기를 끝낸다: 지금 자세(마지막으로 받은 자세)가 기준이 된다."""
        self.started = True
        if self.ref is None and self.last is not None:
            self.ref = self.last

    def update(self, t, pose):
        """시각 t [s] 의 자세 pose 를 반영한다."""
        if self.t0 is None:
            self.t0 = t
        self.last = tuple(pose)
        if not self.started and t - self.t0 < self.settle_s:
            return
        if self.ref is None:
            self.ref = tuple(pose)
        x, y, _z, roll, pitch, yaw = pose
        rx, ry, _rz, rroll, rpitch, ryaw = self.ref
        self.samples += 1
        self.max_shift = max(self.max_shift, math.hypot(x - rx, y - ry))
        self.max_tilt = max(self.max_tilt, abs(wrap_pi(roll - rroll)),
                            abs(wrap_pi(pitch - rpitch)))
        self.max_dyaw = max(self.max_dyaw, abs(wrap_pi(yaw - ryaw)))

    def result(self, max_shift_m, max_tilt_deg, max_yaw_deg):
        """판정 {'passed', 'checks', 'measured'} 를 돌려준다."""
        ready = self.samples > 0
        tilt_deg = math.degrees(self.max_tilt)
        yaw_deg = math.degrees(self.max_dyaw)
        checks = {
            'settled': ready,
            'shift': ready and self.max_shift < max_shift_m,
            'tilt': ready and tilt_deg < max_tilt_deg,
            'yaw': ready and yaw_deg < max_yaw_deg,
        }
        measured = {'samples': self.samples, 'max_shift_mm': self.max_shift * 1e3,
                    'max_tilt_deg': tilt_deg, 'max_yaw_change_deg': yaw_deg}
        return {'passed': all(checks.values()), 'checks': checks, 'measured': measured}
