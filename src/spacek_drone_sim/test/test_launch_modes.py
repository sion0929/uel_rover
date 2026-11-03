"""launch 실행 모드 시험: px4:=false 면 PX4 없이 모델만 띄운다 (설치된 ROS 패키지 필요)."""

import importlib.util
import os

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
import pytest

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH = os.path.join(PKG_DIR, 'launch', 'px4_rover_landing.launch.py')


def _load_launch_module():
    try:
        from ament_index_python.packages import get_package_share_directory
        for pkg in ('spacek_drone_sim', 'spacek_rover', 'drone_landing_platform'):
            get_package_share_directory(pkg)
        spec = importlib.util.spec_from_file_location('px4_rover_landing_launch', LAUNCH)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    except Exception as e:      # 설치(colcon build + source) 전이면 건너뜀
        pytest.skip('설치된 패키지가 없어 launch 를 불러올 수 없음: %s' % e)
    return mod


def _run_setup(mod, tmp_path, monkeypatch, **overrides):
    """기본 인자 + overrides 로 launch_setup 을 호출해 만들어진 액션 목록을 돌려준다."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))
    ctx = LaunchContext()
    for ent in mod.generate_launch_description().entities:
        if isinstance(ent, DeclareLaunchArgument):
            ctx.launch_configurations[ent.name] = ''.join(
                s.perform(ctx) for s in ent.default_value or [])
    ctx.launch_configurations.update(overrides)
    runtime = tmp_path / 'runtime'
    original_mkdtemp = mod.tempfile.mkdtemp

    def make_run_dir(**kwargs):
        if kwargs.get('prefix') == 'spacek_drone_sim_':
            os.makedirs(runtime, exist_ok=True)
            return str(runtime)
        return original_mkdtemp(**kwargs)

    monkeypatch.setattr(mod.tempfile, 'mkdtemp', make_run_dir)
    return mod.launch_setup(ctx)


def _walk(entities):
    """이벤트 처리기·타이머 안쪽까지 모든 액션."""
    for e in entities:
        yield e
        subs = list(e.describe_sub_entities())
        for _desc, cond in e.describe_conditional_sub_entities():
            subs += list(cond)
        yield from _walk(subs)


def _text(part):
    return ''.join(s.text for s in part if isinstance(s, TextSubstitution))


def _processes(actions):
    """Node 가 아닌 ExecuteProcess 의 명령 (첫 단어, 전체 문자열)."""
    out = []
    for e in _walk(actions):
        if isinstance(e, ExecuteProcess) and not isinstance(e, Node):
            words = [_text(p) for p in e.cmd]
            out.append((os.path.basename(words[0]), ' '.join(words)))
    return out


def _spawned_names(actions):
    names = []
    for e in _walk(actions):
        if isinstance(e, Node) and e.node_executable == 'create':
            words = [_text(p) for p in e.cmd]
            names.append(words[words.index('-name') + 1])
    return names


def test_without_px4_spawns_models_and_skips_px4(tmp_path, monkeypatch):
    mod = _load_launch_module()
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')   # 라우터 없음 (셸 RMW 무관)
    no_px4 = str(tmp_path / 'no_px4_here')            # PX4 빌드가 없는 폴더
    actions = _run_setup(mod, tmp_path, monkeypatch, px4='false', px4_dir=no_px4)
    procs = _processes(actions)
    assert [p for p, _ in procs] == ['gz']             # PX4, MicroXRCEAgent 없음
    assert procs[0][1].endswith('rover_landing_px4.sdf')
    assert _spawned_names(actions) == ['rover_platform', 'spacek_drone']
    assert not os.path.exists(no_px4)                  # 기체 파일도 설치하지 않음


def test_mars_environment_selects_mars_world(tmp_path, monkeypatch):
    mod = _load_launch_module()
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    actions = _run_setup(mod, tmp_path, monkeypatch, px4='false', environment='mars',
                         px4_dir=str(tmp_path / 'no_px4_here'))
    procs = _processes(actions)
    assert procs[0][1].endswith('rover_landing_mars.sdf')
    assert _spawned_names(actions) == ['rover_platform', 'spacek_drone']


def test_zenoh_router_starts_before_ros_nodes(tmp_path, monkeypatch):
    # 라우터 없이 뜬 노드는 나중에 라우터가 떠도 구독이 살아나지 않아 접이판 명령이 끊긴다 (2026-10-12)
    mod = _load_launch_module()
    from drone_landing_platform import zenoh_router
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_zenoh_cpp')
    monkeypatch.delenv('ZENOH_ROUTER_CHECK_ATTEMPTS', raising=False)
    monkeypatch.setattr(zenoh_router, 'router_running', lambda **_kw: False)
    actions = _run_setup(mod, tmp_path, monkeypatch, px4='false',
                         px4_dir=str(tmp_path / 'no_px4_here'))
    router = next(i for i, e in enumerate(actions)
                  if isinstance(e, Node) and e.node_executable == 'rmw_zenohd')
    first_node = next(i for i, e in enumerate(actions)
                      if isinstance(e, Node) and e.node_executable != 'rmw_zenohd')
    wait = next(i for i, e in enumerate(actions) if isinstance(e, SetEnvironmentVariable)
                and _text(e.name) == 'ZENOH_ROUTER_CHECK_ATTEMPTS')
    assert router < first_node and wait < first_node


def test_with_px4_still_requires_px4_build(tmp_path, monkeypatch):
    mod = _load_launch_module()
    with pytest.raises(FileNotFoundError):
        _run_setup(mod, tmp_path, monkeypatch, px4='true', px4_dir=str(tmp_path / 'no_px4_here'))


def _fake_px4_tree(tmp_path):
    root = tmp_path / 'PX4-Autopilot'
    build = root / 'build' / 'px4_sitl_default'
    (build / 'bin').mkdir(parents=True)
    (build / 'bin' / 'px4').touch()
    frames = build / 'etc' / 'init.d-posix' / 'airframes'
    frames.mkdir(parents=True)
    (frames / '4001_gz_x500').write_text('base\n', encoding='utf-8')
    return root, frames


def test_default_3a_px4_keeps_build_rootfs_command_and_installs_airframe(
        tmp_path, monkeypatch):
    mod = _load_launch_module()
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    root, frames = _fake_px4_tree(tmp_path)
    actions = _run_setup(mod, tmp_path, monkeypatch,
                         px4='true', px4_dir=str(root))
    px4_commands = [cmd for name, cmd in _processes(actions) if name == 'px4']
    assert len(px4_commands) == 1 and ' -w ' not in px4_commands[0]
    assert (frames / '4901_gz_spacek_drone').is_file()
    assert not (tmp_path / 'runtime' / 'etc').exists()


def test_experiment_px4_uses_fresh_subfolder_on_reused_log_root(
        tmp_path, monkeypatch):
    mod = _load_launch_module()
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    root, frames = _fake_px4_tree(tmp_path)
    overrides = tmp_path / 'overrides.yaml'
    overrides.write_text('COM_RC_IN_MODE: 4\n', encoding='utf-8')
    log_root = tmp_path / 'logs'
    for name in ('first', 'second'):
        run = tmp_path / name
        run.mkdir()
        actions = _run_setup(mod, run, monkeypatch, px4='true',
                             px4_dir=str(root), px4_log_dir=str(log_root),
                             px4_overrides_file=str(overrides))
        assert any(' -w ' in cmd for proc, cmd in _processes(actions)
                   if proc == 'px4')
    runs = sorted(log_root.glob('px4_*'))
    assert len(runs) == 2
    for run in runs:
        text = (run / 'etc' / 'init.d-posix' / 'airframes' /
                '4901_gz_spacek_drone').read_text(encoding='utf-8')
        assert 'param set COM_RC_IN_MODE 4' in text
    assert not (frames / '4901_gz_spacek_drone').exists()
