"""화성 월드의 저폴리 바위 20개 (models/mars_rocks) 를 만든다. 시각 + 충돌 메시, 정적 모델 하나.

바위 자리는 2026-10-25 발표 영상과 같다: 로버 출발 직선 (x -0.6~3.6 m, |y| < 0.7 m) 과
(0.5, 0) 중심 반경 2.7 m 원 주행 띠, 드론 출발점 (-1.5, 0) 을 비워 둔다. 시드가 고정이라 다시 돌려도 같은 파일.
  python3 src/spacek_drone_sim/scripts/make_mars_rocks.py     # 저장소 루트에서
"""

import math
import os

import numpy as np

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(PKG_DIR, 'models', 'mars_rocks')

# x, y, 크기 [m], yaw [deg]
PLACEMENTS = [
    (-1.6, 4.6, 0.55, 20), (2.4, 5.3, 0.8, 75), (5.6, 3.6, 0.45, 140), (6.8, -1.2, 0.6, 10),
    (-4.6, 1.5, 0.5, 200), (-4.2, -2.6, 0.35, 95), (4.9, -3.9, 0.4, 300), (-2.1, -4.7, 0.3, 45),
    (0.9, 7.5, 1.3, 160), (-5.5, 6.2, 1.1, 250), (7.8, 5.9, 1.0, 30), (8.5, -4.8, 0.9, 120),
    (0.6, 0.9, 0.12, 0), (1.6, -0.95, 0.10, 60), (-0.5, -1.1, 0.09, 30), (1.9, 1.05, 0.11, 80),
    (3.3, 3.2, 0.18, 10), (-1.9, 2.9, 0.16, 130), (-3.7, -0.4, 0.2, 70), (3.9, -2.2, 0.15, 200),
]
COLOURS = [(0.42, 0.27, 0.19), (0.50, 0.33, 0.22), (0.36, 0.25, 0.20), (0.46, 0.30, 0.21)]


def icosphere(level=2):
    t = (1 + 5 ** 0.5) / 2
    v = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t), (0, -1, -t),
         (0, 1, -t), (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    f = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4),
         (11, 10, 2), (10, 7, 6), (7, 1, 8), (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9),
         (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
    v = [np.array(p, float) / np.linalg.norm(p) for p in v]
    for _ in range(level):
        cache, nf = {}, []

        def mid(a, b):
            key = (min(a, b), max(a, b))
            if key not in cache:
                m = v[a] + v[b]
                v.append(m / np.linalg.norm(m))
                cache[key] = len(v) - 1
            return cache[key]
        for a, b, c in f:
            ab, bc, ca = mid(a, b), mid(b, c), mid(c, a)
            nf += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        f = nf
    return np.array(v), np.array(f)


def rock_obj(path, seed):
    """단위 크기 바위 메시 (밑면 z = 0). 저주파 요철 + 평평한 절단면 3개 (달걀처럼 보이지 않게)."""
    r = np.random.default_rng(seed)
    v, f = icosphere(2)
    k = r.normal(size=(6, 3))
    bump = sum(0.10 * np.sin(v @ kk * 2.2 + r.uniform(0, 6)) for kk in k)
    v = v * (1 + bump)[:, None]
    for _ in range(3):
        n = r.normal(size=3)
        n /= np.linalg.norm(n)
        d = r.uniform(0.6, 0.85)
        v -= np.clip(v @ n - d, 0, None)[:, None] * n
    v *= np.array([1.0, r.uniform(0.7, 0.95), r.uniform(0.45, 0.7)])
    v[:, 2] -= v[:, 2].min()
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('# procedural rock (make_mars_rocks.py seed %d)\n' % seed)
        for p in v:
            fh.write('v %.5f %.5f %.5f\n' % tuple(p))
        for a, b, c in f:
            n = np.cross(v[b] - v[a], v[c] - v[a])
            n /= np.linalg.norm(n) or 1
            fh.write('vn %.5f %.5f %.5f\n' % tuple(n))
        for i, (a, b, c) in enumerate(f):
            fh.write('f %d//%d %d//%d %d//%d\n' % (a + 1, i + 1, b + 1, i + 1, c + 1, i + 1))


def clear(x, y, size):
    """로버 직선·원 주행 띠, 실험 특징점 자리, 드론 출발점과 겹치지 않으면 True."""
    if -0.8 - size < x < 3.8 + size and abs(y) < 0.8 + size:
        return False
    if abs(math.hypot(x - 0.5, y) - 2.7) < 0.85 + size:
        return False
    for sx, sy in [[0.0, 1.6], [1.0, 1.8], [1.0, -1.6], [0.0, -1.8], [-1.5, 0.0]]:
        if math.hypot(x - sx, y - sy) < 0.35 + size:
            return False
    return True


def link_xml(i, x, y, size, yaw):
    name = 'rock_%02d' % i
    c = COLOURS[i % len(COLOURS)]
    mesh = ('<mesh><uri>model://mars_rocks/meshes/%s.obj</uri><scale>%g %g %g</scale></mesh>'
            % (name, size, size, size))
    material = ('<material><ambient>%.3f %.3f %.3f 1</ambient><diffuse>%g %g %g 1</diffuse>'
                '<specular>0.05 0.05 0.05 1</specular></material>'
                % (c[0] * .8, c[1] * .8, c[2] * .8, *c))
    return '\n'.join([
        '    <link name="%s">' % name,
        '      <pose>%g %g %g 0 0 %.6f</pose>' % (x, y, -0.12 * size, math.radians(yaw)),
        '      <collision name="collision"><geometry>%s</geometry></collision>' % mesh,
        '      <visual name="visual">',
        '        <cast_shadows>true</cast_shadows>',
        '        <geometry>%s</geometry>' % mesh,
        '        %s' % material,
        '      </visual>',
        '    </link>'])


def main():
    os.makedirs(os.path.join(OUT, 'meshes'), exist_ok=True)
    links = []
    for i, (x, y, size, yaw) in enumerate(PLACEMENTS):
        assert clear(x, y, size * 0.5), (x, y)
        rock_obj(os.path.join(OUT, 'meshes', 'rock_%02d.obj' % i), 100 + i)
        links.append(link_xml(i, x, y, size, yaw))
    with open(os.path.join(OUT, 'model.sdf'), 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('<?xml version="1.0"?>\n'
                 '<!-- make_mars_rocks.py 가 만든 파일 (직접 고치지 말고 스크립트를 고쳐 다시 만든다).\n'
                 '     바위 20개, 시각 + 충돌. 자리는 월드 원점 기준 (밑면 12%% 를 땅에 묻음). -->\n'
                 '<sdf version="1.9">\n  <model name="mars_rocks">\n    <static>true</static>\n'
                 '%s\n  </model>\n</sdf>\n' % '\n'.join(links))
    with open(os.path.join(OUT, 'model.config'), 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('<?xml version="1.0"?>\n<model>\n  <name>mars_rocks</name>\n'
                 '  <version>1.0</version>\n  <sdf version="1.9">model.sdf</sdf>\n'
                 '  <author><name>sion0</name></author>\n'
                 '  <description>화성 월드 저폴리 바위 20개 (scripts/make_mars_rocks.py 생성).'
                 '</description>\n</model>\n')
    print('wrote %d rocks to %s' % (len(links), OUT))


if __name__ == '__main__':
    main()
