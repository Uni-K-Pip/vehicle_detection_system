# Copyright 2026 kohei
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""Smoke tests for vehicle_detection.launch.py."""

import importlib.util
from pathlib import Path

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import pytest

_PACKAGE_ROOT = Path(__file__).resolve().parent.parent
_LAUNCH_FILE = _PACKAGE_ROOT / 'launch' / 'vehicle_detection.launch.py'
_PRESETS_DIR = _PACKAGE_ROOT / 'config' / 'presets'


def _load_launch_module():
    assert _LAUNCH_FILE.is_file(), f'launch file missing: {_LAUNCH_FILE}'
    spec = importlib.util.spec_from_file_location(
        'vehicle_detection_launch', _LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_launch_description_constructs():
    module = _load_launch_module()
    ld = module.generate_launch_description()
    assert isinstance(ld, LaunchDescription)
    assert ld.entities, 'launch description must contain entities'


def test_launch_arguments_present():
    module = _load_launch_module()
    ld = module.generate_launch_description()
    arg_names = {
        e.name for e in ld.entities if isinstance(e, DeclareLaunchArgument)
    }
    expected = {
        'pcd_file',
        'pcd_directory',
        'pcd_glob',
        'loop',
        'input_frame_id',
        'target_frame_id',
        'publish_once',
        'params_file',
        'use_detector',
        'use_sender',
        'use_rviz',
        'rviz_config',
        'use_gui',
        'gui_host',
        'detector_preset',
        'input_mode',
        'rosbag_path',
        'rosbag_topic',
        'rosbag_loop',
        'rosbag_rate',
    }
    missing = expected - arg_names
    assert not missing, f'missing launch arguments: {sorted(missing)}'


def test_expected_nodes_declared():
    module = _load_launch_module()
    ld = module.generate_launch_description()
    context = LaunchContext()
    node_executables = []
    for entity in ld.entities:
        if isinstance(entity, Node):
            exe = entity.node_executable
            if hasattr(exe, 'perform'):
                exe = exe.perform(context)
            node_executables.append(exe)
    assert 'pcd_loader_node' in node_executables
    assert 'vehicle_detector_node' in node_executables
    assert 'detection_sender_node' in node_executables
    assert 'parameter_bridge_node' in node_executables
    assert 'static_transform_publisher' in node_executables
    assert 'rviz2' in node_executables


def test_rviz_config_present():
    rviz_config = _PACKAGE_ROOT / 'rviz' / 'vehicle_detection.rviz'
    assert rviz_config.is_file(), f'missing RViz config: {rviz_config}'


# FR-014: parameter preset management ---------------------------------------

def test_detector_preset_default_value():
    module = _load_launch_module()
    ld = module.generate_launch_description()
    decls = {
        e.name: e for e in ld.entities if isinstance(e, DeclareLaunchArgument)
    }
    assert 'detector_preset' in decls, 'detector_preset launch arg missing'
    # default_value is a tuple of substitutions; perform each in a launch
    # context so we get the resolved text without depending on the
    # specific substitution class.
    ctx = LaunchContext()
    default_value = ''.join(
        part.perform(ctx) if hasattr(part, 'perform') else str(part)
        for part in decls['detector_preset'].default_value
    )
    assert default_value == 'default', (
        f'detector_preset default must remain "default" so the existing '
        f'`ros2 launch ... vehicle_detection.launch.py` behaviour does not '
        f'change (got {default_value!r}).'
    )


@pytest.mark.parametrize('preset_name', ['default', 'pandaset_balanced', 'near_range'])
def test_known_preset_files_exist(preset_name):
    preset_path = _PRESETS_DIR / f'{preset_name}.yaml'
    assert preset_path.is_file(), (
        f'preset file missing: {preset_path}. FR-014 requires default, '
        f'pandaset_balanced, and near_range presets to ship with the package.'
    )


@pytest.mark.parametrize('preset_name', ['default', 'pandaset_balanced', 'near_range'])
def test_resolve_preset_path_known_names(preset_name):
    module = _load_launch_module()
    resolved = module._resolve_preset_path(preset_name, _PRESETS_DIR)
    assert Path(resolved).is_file()
    assert Path(resolved).name == f'{preset_name}.yaml'


def test_resolve_preset_path_unknown_name_raises():
    module = _load_launch_module()
    with pytest.raises(ValueError) as excinfo:
        module._resolve_preset_path('does_not_exist', _PRESETS_DIR)
    message = str(excinfo.value)
    assert 'does_not_exist' in message, (
        f'error message must include the requested preset name, got: {message}'
    )
    # Available preset list should help users pick a valid name.
    assert 'default' in message
    assert 'pandaset_balanced' in message
    assert 'near_range' in message


def test_resolve_preset_path_missing_directory_raises(tmp_path):
    module = _load_launch_module()
    missing_dir = tmp_path / 'no_such_presets'
    with pytest.raises(ValueError) as excinfo:
        module._resolve_preset_path('default', missing_dir)
    assert 'no presets installed' in str(excinfo.value)


def test_default_preset_is_no_op_overlay():
    """
    Guard default.yaml against silently overriding detector parameters.

    FR-014 requires that detector_preset:=default keeps the existing
    `ros2 launch ... vehicle_detection.launch.py` behaviour. Storing
    real values in default.yaml would silently freeze those values
    even if detector_params.yaml later changes, so we guard the file
    here.
    """
    import yaml  # noqa: WPS433 - imported lazily so the rest of the suite
    # remains usable on environments without PyYAML.

    default_yaml = _PRESETS_DIR / 'default.yaml'
    with default_yaml.open('r', encoding='utf-8') as fh:
        loaded = yaml.safe_load(fh) or {}
    detector = loaded.get('vehicle_detector_node', {})
    params = detector.get('ros__parameters', {}) or {}
    assert params == {}, (
        f'default.yaml must contain an empty ros__parameters block to '
        f'avoid drifting from detector_params.yaml; got: {params!r}'
    )


# FR-015: rosbag input support ---------------------------------------------

def test_input_mode_default_is_pcd():
    """
    FR-015 requires input_mode:=pcd (or unset) to keep the existing
    behaviour. Guard the default so a future change cannot silently flip
    the pipeline to rosbag mode.
    """
    module = _load_launch_module()
    ld = module.generate_launch_description()
    decls = {
        e.name: e for e in ld.entities if isinstance(e, DeclareLaunchArgument)
    }
    assert 'input_mode' in decls, 'input_mode launch arg missing'
    ctx = LaunchContext()
    default_value = ''.join(
        part.perform(ctx) if hasattr(part, 'perform') else str(part)
        for part in decls['input_mode'].default_value
    )
    assert default_value == 'pcd', (
        f'input_mode default must remain "pcd" so the existing '
        f'`ros2 launch ... vehicle_detection.launch.py` behaviour does not '
        f'change (got {default_value!r}).'
    )


def test_rosbag_args_have_safe_defaults():
    """
    FR-015: rosbag_path / rosbag_topic / rosbag_loop / rosbag_rate must
    have inert defaults so input_mode:=pcd users never accidentally
    spawn ``ros2 bag play``.
    """
    module = _load_launch_module()
    ld = module.generate_launch_description()
    decls = {
        e.name: e for e in ld.entities if isinstance(e, DeclareLaunchArgument)
    }
    ctx = LaunchContext()

    def _resolve(arg_name):
        return ''.join(
            part.perform(ctx) if hasattr(part, 'perform') else str(part)
            for part in decls[arg_name].default_value
        )

    assert _resolve('rosbag_path') == ''
    assert _resolve('rosbag_topic') == ''
    assert _resolve('rosbag_loop') == 'false'
    assert _resolve('rosbag_rate') == '1.0'


def test_validate_input_mode_inputs_accepts_pcd():
    module = _load_launch_module()
    # rosbag_path is irrelevant when input_mode=pcd, even if it points at
    # a non-existent path -- pcd users should never be blocked by it.
    module._validate_input_mode_inputs('pcd', '')
    module._validate_input_mode_inputs('pcd', '/nonexistent/path')


def test_validate_input_mode_inputs_accepts_rosbag(tmp_path):
    module = _load_launch_module()
    bag_dir = tmp_path / 'bag'
    bag_dir.mkdir()
    # Existing directory is acceptable (rosbag2 stores bags as a directory).
    module._validate_input_mode_inputs('rosbag', str(bag_dir))
    # Existing file is also acceptable (legacy single-file bags / mcap).
    bag_file = tmp_path / 'sample.mcap'
    bag_file.write_text('')
    module._validate_input_mode_inputs('rosbag', str(bag_file))


def test_validate_input_mode_inputs_rejects_unknown_mode():
    module = _load_launch_module()
    with pytest.raises(ValueError) as excinfo:
        module._validate_input_mode_inputs('bag', '')
    message = str(excinfo.value)
    assert 'bag' in message
    assert 'pcd' in message
    assert 'rosbag' in message


def test_validate_input_mode_inputs_rejects_missing_rosbag_path():
    module = _load_launch_module()
    with pytest.raises(ValueError) as excinfo:
        module._validate_input_mode_inputs('rosbag', '')
    message = str(excinfo.value)
    assert 'rosbag_path' in message


def test_validate_input_mode_inputs_rejects_nonexistent_path(tmp_path):
    module = _load_launch_module()
    missing = tmp_path / 'no_such_bag'
    with pytest.raises(ValueError) as excinfo:
        module._validate_input_mode_inputs('rosbag', str(missing))
    message = str(excinfo.value)
    assert str(missing) in message
    assert 'does not exist' in message


def test_build_rosbag_play_command_minimal():
    module = _load_launch_module()
    cmd = module._build_rosbag_play_command(
        rosbag_path='/abs/path/to/bag',
        rosbag_topic='',
        rosbag_loop='false',
        rosbag_rate='1.0',
    )
    assert cmd[:3] == ['ros2', 'bag', 'play']
    assert '/abs/path/to/bag' in cmd
    assert '--rate' in cmd
    rate_idx = cmd.index('--rate')
    assert cmd[rate_idx + 1] == '1.0'
    # No remap, no loop when topic is empty and loop=false.
    assert '--remap' not in cmd
    assert '--loop' not in cmd


def test_build_rosbag_play_command_with_topic_remap():
    module = _load_launch_module()
    cmd = module._build_rosbag_play_command(
        rosbag_path='/abs/bag',
        rosbag_topic='/lidar/points',
        rosbag_loop='false',
        rosbag_rate='1.0',
    )
    assert '--remap' in cmd
    remap_idx = cmd.index('--remap')
    assert cmd[remap_idx + 1] == '/lidar/points:=/input/points'


def test_build_rosbag_play_command_with_loop_and_rate():
    module = _load_launch_module()
    cmd = module._build_rosbag_play_command(
        rosbag_path='/abs/bag',
        rosbag_topic='',
        rosbag_loop='true',
        rosbag_rate='2.5',
    )
    assert '--loop' in cmd
    rate_idx = cmd.index('--rate')
    assert cmd[rate_idx + 1] == '2.5'


def test_build_rosbag_play_command_accepts_bool_loop():
    module = _load_launch_module()
    cmd_true = module._build_rosbag_play_command(
        rosbag_path='/abs/bag',
        rosbag_topic='',
        rosbag_loop=True,
        rosbag_rate='1.0',
    )
    assert '--loop' in cmd_true
    cmd_false = module._build_rosbag_play_command(
        rosbag_path='/abs/bag',
        rosbag_topic='',
        rosbag_loop=False,
        rosbag_rate='1.0',
    )
    assert '--loop' not in cmd_false


def test_pcd_loader_has_input_mode_condition():
    """
    FR-015: pcd_loader_node must only start when input_mode=pcd. Guard
    against accidentally removing the gating condition, which would
    cause both pcd_loader_node and ``ros2 bag play`` to publish to
    /input/points at the same time.
    """
    from launch.conditions import LaunchConfigurationEquals

    module = _load_launch_module()
    ld = module.generate_launch_description()
    context = LaunchContext()
    pcd_loader_nodes = []
    for entity in ld.entities:
        if isinstance(entity, Node):
            exe = entity.node_executable
            if hasattr(exe, 'perform'):
                exe = exe.perform(context)
            if exe == 'pcd_loader_node':
                pcd_loader_nodes.append(entity)
    assert len(pcd_loader_nodes) == 1, (
        f'expected exactly one pcd_loader_node, got {len(pcd_loader_nodes)}'
    )
    pcd_loader = pcd_loader_nodes[0]
    condition = pcd_loader.condition
    assert condition is not None, (
        'pcd_loader_node must be gated on a launch condition (FR-015 '
        'requires input_mode=rosbag to skip the loader).'
    )
    # The condition object differs across launch versions; we keep the
    # check loose but verify it is the LaunchConfigurationEquals variant
    # we wired up rather than the unconditional default.
    assert isinstance(condition, LaunchConfigurationEquals), (
        f'pcd_loader_node condition must be LaunchConfigurationEquals so '
        f'input_mode:=rosbag suppresses it, got '
        f'{type(condition).__name__}'
    )
