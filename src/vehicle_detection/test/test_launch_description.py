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

_PACKAGE_ROOT = Path(__file__).resolve().parent.parent
_LAUNCH_FILE = _PACKAGE_ROOT / 'launch' / 'vehicle_detection.launch.py'


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
