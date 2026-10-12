"""바닥 침하량 d -> 플레이트 각도 theta 커플링 법칙 (ROS 의존성 없음).

기구 (plate_px 기준, 플랫폼 좌표계, mm):
  * 힌지 축  (대기): x = 107.2, z = 115.7  -> 바닥과 함께 d 만큼 내려감
  * 끝단 핀  (대기): x = 212.7, z = 135.0  -> 프레임 고정 수평 가이드(z = 135.0) 위를 미끄러짐
  * 힌지-핀 거리 L = 107.25, 높이차 h0 = 135.0 - 115.7 = 19.3

  sin(theta) = (h0 + d) / L
  theta(0) = asin(19.3 / 107.25) = 10.37 deg (대기 깔때기 각도)
  theta = 48.2 deg @ d = 60.6 mm (실제 하단 스톱)
  theta = 90 deg  @ d = 87.95 mm (특이점, 기구상 도달 불가)

SolidWorks 측정값으로 바꿀 때는 CouplingTable(d_mm, theta_deg) 를 사용한다.
"""

import bisect
import math

DEFAULT_HINGE_PIN_LENGTH_MM = 107.25
DEFAULT_PIN_HEIGHT_OFFSET_MM = 19.3
DEFAULT_MAX_SINK_MM = 84.0


def law_theta_deg(d_mm,
                  length_mm=DEFAULT_HINGE_PIN_LENGTH_MM,
                  h0_mm=DEFAULT_PIN_HEIGHT_OFFSET_MM,
                  max_sink_mm=DEFAULT_MAX_SINK_MM):
    """기본 법칙 sin(theta) = (h0 + d) / L 로 절대 각도 theta[deg] 를 계산한다."""
    if length_mm <= 0.0:
        raise ValueError('length_mm 는 양수여야 합니다')
    d = min(max(float(d_mm), 0.0), float(max_sink_mm))
    s = (h0_mm + d) / length_mm
    s = min(max(s, -1.0), 1.0)
    return math.degrees(math.asin(s))


class CouplingLaw:
    """해석적 커플링 법칙."""

    def __init__(self,
                 length_mm=DEFAULT_HINGE_PIN_LENGTH_MM,
                 h0_mm=DEFAULT_PIN_HEIGHT_OFFSET_MM,
                 max_sink_mm=DEFAULT_MAX_SINK_MM):
        self.length_mm = float(length_mm)
        self.h0_mm = float(h0_mm)
        self.max_sink_mm = float(max_sink_mm)
        if self.length_mm <= 0.0:
            raise ValueError('length_mm 는 양수여야 합니다')

    def theta_deg(self, d_mm):
        """침하량 d[mm] 에 대한 절대 플레이트 각도[deg]."""
        return law_theta_deg(d_mm, self.length_mm, self.h0_mm, self.max_sink_mm)


class CouplingTable:
    """d_mm / theta_deg 룩업 테이블 (선형 보간, 양 끝은 클램프)."""

    def __init__(self, d_mm, theta_deg):
        d = [float(v) for v in d_mm]
        t = [float(v) for v in theta_deg]
        if len(d) != len(t):
            raise ValueError('d_mm 와 theta_deg 의 길이가 다릅니다')
        if len(d) < 2:
            raise ValueError('테이블에는 최소 2개의 점이 필요합니다')
        for i in range(1, len(d)):
            if d[i] <= d[i - 1]:
                raise ValueError('d_mm 는 엄격히 증가해야 합니다')
        self.d_mm = d
        self.theta = t

    def theta_deg(self, d_mm):
        """침하량 d[mm] 에 대한 절대 플레이트 각도[deg] (선형 보간)."""
        x = float(d_mm)
        if x <= self.d_mm[0]:
            return self.theta[0]
        if x >= self.d_mm[-1]:
            return self.theta[-1]
        i = bisect.bisect_right(self.d_mm, x)
        x0, x1 = self.d_mm[i - 1], self.d_mm[i]
        y0, y1 = self.theta[i - 1], self.theta[i]
        return y0 + (y1 - y0) * (x - x0) / (x1 - x0)

    def is_monotonic(self):
        """각도 theta 가 d 에 대해 단조 비감소인지 확인한다."""
        return all(self.theta[i] >= self.theta[i - 1] for i in range(1, len(self.theta)))


def make_coupling(mode, law_params=None, table_d_mm=None, table_theta_deg=None):
    """mode('law' | 'table') 에 맞는 커플링 객체를 만든다."""
    if mode == 'law':
        p = law_params or {}
        return CouplingLaw(p.get('length_mm', DEFAULT_HINGE_PIN_LENGTH_MM),
                           p.get('h0_mm', DEFAULT_PIN_HEIGHT_OFFSET_MM),
                           p.get('max_sink_mm', DEFAULT_MAX_SINK_MM))
    if mode == 'table':
        return CouplingTable(table_d_mm or [], table_theta_deg or [])
    raise ValueError('알 수 없는 coupling mode: %r' % (mode,))


def relative_command_rad(theta_abs_deg, zero_angle_deg, min_deg=0.0, max_deg=95.0):
    """절대 각도를 SDF 조인트 명령(대기 자세 = 0 rad 인 상대 각도)으로 변환한다."""
    t = min(max(float(theta_abs_deg), float(min_deg)), float(max_deg))
    return math.radians(t - float(zero_angle_deg))
