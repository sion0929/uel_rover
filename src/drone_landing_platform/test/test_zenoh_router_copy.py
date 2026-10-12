"""zenoh_router.py 사본 시험: spacek_rover 의 원본과 같아야 한다 (고정 플랫폼은 spacek_rover 없이 뜸)."""

import os

import pytest

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OWN = os.path.join(PKG_DIR, 'drone_landing_platform', 'zenoh_router.py')
ROVER = os.path.join(os.path.dirname(PKG_DIR), 'spacek_rover', 'spacek_rover', 'zenoh_router.py')


def test_same_as_spacek_rover():
    if not os.path.exists(ROVER):
        pytest.skip('옆 폴더에 spacek_rover 소스가 없음')
    with open(OWN, encoding='utf-8') as a, open(ROVER, encoding='utf-8') as b:
        assert a.read() == b.read()
