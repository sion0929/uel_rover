"""커플링 법칙/테이블 단위 시험 (ROS 없이 pytest 로 실행 가능)."""

import math
import os

from drone_landing_platform.coupling import (CouplingLaw, CouplingTable, law_theta_deg,
                                             make_coupling, relative_command_rad)
import pytest
import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_law_standby_angle():
    # sin(theta0) = 19.3 / 107.25 -> 10.367 deg
    assert abs(law_theta_deg(0.0) - 10.367) < 0.01


def test_law_at_hard_stop():
    # 하단 스톱 60.6 mm 에서 48.2 deg (SolidWorks Motion 결과)
    assert abs(law_theta_deg(60.6) - 48.16) < 0.05


def test_law_monotonic_and_clamped():
    prev = -1.0
    for i in range(0, 1001):
        d = 84.0 * i / 1000.0
        th = law_theta_deg(d)
        assert th >= prev - 1e-12
        prev = th
    assert law_theta_deg(-5.0) == pytest.approx(law_theta_deg(0.0))
    assert law_theta_deg(200.0) == pytest.approx(law_theta_deg(84.0))
    assert law_theta_deg(200.0, max_sink_mm=200.0) == pytest.approx(90.0)


def test_law_matches_formula():
    law = CouplingLaw(107.25, 19.3)
    for d in (0.0, 12.5, 37.0, 60.6):
        expected = math.degrees(math.asin((19.3 + d) / 107.25))
        assert law.theta_deg(d) == pytest.approx(expected)


def test_table_interpolation_and_clamp():
    tab = CouplingTable([0.0, 10.0, 20.0], [10.0, 20.0, 40.0])
    assert tab.theta_deg(5.0) == pytest.approx(15.0)
    assert tab.theta_deg(15.0) == pytest.approx(30.0)
    assert tab.theta_deg(10.0) == pytest.approx(20.0)
    assert tab.theta_deg(-3.0) == pytest.approx(10.0)
    assert tab.theta_deg(99.0) == pytest.approx(40.0)
    assert tab.is_monotonic()


def test_table_validation():
    with pytest.raises(ValueError):
        CouplingTable([0.0, 1.0], [1.0])
    with pytest.raises(ValueError):
        CouplingTable([0.0], [1.0])
    with pytest.raises(ValueError):
        CouplingTable([0.0, 5.0, 5.0], [1.0, 2.0, 3.0])
    with pytest.raises(ValueError):
        make_coupling('nonsense')


def test_default_yaml_table_matches_law():
    with open(os.path.join(PKG_DIR, 'config', 'coupling.yaml'), encoding='utf-8') as f:
        cfg = yaml.safe_load(f)['plate_coupler']['ros__parameters']
    tab = make_coupling('table', table_d_mm=cfg['table']['d_mm'],
                        table_theta_deg=cfg['table']['theta_deg'])
    assert tab.is_monotonic()
    assert tab.theta_deg(0.0) == pytest.approx(10.37, abs=0.01)
    for d in cfg['table']['d_mm']:
        assert tab.theta_deg(d) == pytest.approx(law_theta_deg(d), abs=0.02)
    assert cfg['plate_zero_angle_deg'] == pytest.approx(law_theta_deg(0.0), abs=0.001)


def test_relative_command_zero_at_standby():
    zero = law_theta_deg(0.0)
    assert relative_command_rad(zero, zero) == pytest.approx(0.0)
    assert relative_command_rad(48.16, zero) == pytest.approx(math.radians(48.16 - zero))
    # 절대각 0..95 deg 로 클램프
    assert relative_command_rad(120.0, zero) == pytest.approx(math.radians(95.0 - zero))
