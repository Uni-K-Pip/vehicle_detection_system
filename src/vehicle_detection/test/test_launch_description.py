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
