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

"""
Launch file for the vehicle_detection pipeline.

Starts pcd_loader_node, vehicle_detector_node, the static transform from
target_frame_id to input_frame_id, the optional detection_sender_node,
the optional parameter_bridge_node Web GUI, and an optional RViz instance
with the bundled detector configuration.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_pcd_file(context, *args, **kwargs):
    pcd_file = LaunchConfiguration('pcd_file').perform(context)
    if not Path(pcd_file).is_absolute():
        # Resolve relative to current working directory at launch time so
        # users can pass paths like data/pcd/sample.pcd from the workspace
        # root. Absolute paths are passed through.
        pcd_file = str(Path.cwd() / pcd_file)
    context.launch_configurations['pcd_file'] = pcd_file
    return []


def _resolve_preset_path(preset_name, presets_dir):
    """
    Map a preset name to an absolute YAML path under presets_dir.

    Pure helper for FR-014 detector_preset resolution: kept free of
    launch context so the unit test suite can drive it directly.
    Raises ValueError with the requested name and the list of available
    preset stems when the YAML file is missing.
    """
    presets_dir = Path(presets_dir)
    preset_path = presets_dir / f'{preset_name}.yaml'
    if preset_path.is_file():
        return str(preset_path)
    if presets_dir.is_dir():
        available = sorted(p.stem for p in presets_dir.glob('*.yaml'))
    else:
        available = []
    available_msg = ', '.join(available) if available else '<no presets installed>'
    raise ValueError(
        f'Unknown detector_preset {preset_name!r}. '
        f'Available presets: {available_msg}. '
        f'Looked for {preset_path}.'
    )


def _resolve_detector_preset(context, *args, **kwargs):
    preset_name = LaunchConfiguration('detector_preset').perform(context)
    pkg_share = get_package_share_directory('vehicle_detection')
    presets_dir = Path(pkg_share) / 'config' / 'presets'
    try:
        preset_file = _resolve_preset_path(preset_name, presets_dir)
    except ValueError as exc:
        raise RuntimeError(str(exc)) from exc
    context.launch_configurations['detector_preset_file'] = preset_file
    return []


def generate_launch_description():
    pkg_share = get_package_share_directory('vehicle_detection')
    default_params = str(Path(pkg_share) / 'config' / 'detector_params.yaml')
    default_rviz_config = str(
        Path(pkg_share) / 'rviz' / 'vehicle_detection.rviz')

    pcd_file_arg = DeclareLaunchArgument(
        'pcd_file',
        default_value='data/pcd/sample.pcd',
        description='Path to the PCD file to publish on /input/points.',
    )
    pcd_directory_arg = DeclareLaunchArgument(
        'pcd_directory',
        default_value='',
        description=(
            'Phase 2: directory of PCDs to play back continuously. When '
            'non-empty, takes priority over pcd_file. Files are scanned '
            'with pcd_glob and sorted lexicographically.'
        ),
    )
    pcd_glob_arg = DeclareLaunchArgument(
        'pcd_glob',
        default_value='*.pcd',
        description='Glob applied under pcd_directory (Phase 2).',
    )
    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='true',
        description=(
            'Phase 2: loop=true restarts the playlist after the last '
            'file; loop=false stops publishing while keeping the node '
            'alive.'
        ),
    )
    input_frame_id_arg = DeclareLaunchArgument(
        'input_frame_id',
        default_value='lidar',
        description='frame_id stamped on the published PointCloud2.',
    )
    target_frame_id_arg = DeclareLaunchArgument(
        'target_frame_id',
        default_value='map',
        description='Target frame for downstream detection results.',
    )
    publish_once_arg = DeclareLaunchArgument(
        'publish_once',
        default_value='false',
        description='If true, publish a single frame and keep the node alive.',
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='YAML parameter file passed to nodes.',
    )
    use_detector_arg = DeclareLaunchArgument(
        'use_detector',
        default_value='true',
        description='If true, start vehicle_detector_node alongside the loader.',
    )
    use_sender_arg = DeclareLaunchArgument(
        'use_sender',
        default_value='false',
        description='If true, start detection_sender_node.',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='If true, start RViz with the bundled detector config.',
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to the RViz configuration file.',
    )
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Start parameter_bridge_node with the browser-based GUI.',
    )
    gui_host_arg = DeclareLaunchArgument(
        'gui_host',
        default_value='127.0.0.1',
        description=(
            'Bind address for parameter_bridge_node. Defaults to loopback to '
            'keep the unauthenticated parameter-write API local. Set to '
            '0.0.0.0 (or a specific interface) when exposing the GUI from a '
            'Docker container to the host.'
        ),
    )
    detector_preset_arg = DeclareLaunchArgument(
        'detector_preset',
        default_value='default',
        description=(
            'Phase 2 (FR-014): name of a detection-parameter preset under '
            'config/presets/<name>.yaml. Applied as an overlay on top of '
            'params_file for vehicle_detector_node only. detector_preset:= '
            'default is a no-op overlay and keeps the existing behaviour. '
            'Unknown names fail launch with the available preset list.'
        ),
    )

    pcd_file = LaunchConfiguration('pcd_file')
    pcd_directory = LaunchConfiguration('pcd_directory')
    pcd_glob = LaunchConfiguration('pcd_glob')
    loop = LaunchConfiguration('loop')
    input_frame_id = LaunchConfiguration('input_frame_id')
    target_frame_id = LaunchConfiguration('target_frame_id')
    publish_once = LaunchConfiguration('publish_once')
    params_file = LaunchConfiguration('params_file')
    use_detector = LaunchConfiguration('use_detector')
    use_sender = LaunchConfiguration('use_sender')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    use_gui = LaunchConfiguration('use_gui')
    gui_host = LaunchConfiguration('gui_host')
    detector_preset_file = LaunchConfiguration('detector_preset_file')

    pcd_loader = Node(
        package='vehicle_detection',
        executable='pcd_loader_node',
        name='pcd_loader_node',
        output='screen',
        parameters=[
            params_file,
            {
                'pcd_file': pcd_file,
                'pcd_directory': pcd_directory,
                'pcd_glob': pcd_glob,
                'loop': loop,
                'input_frame_id': input_frame_id,
                'publish_once': publish_once,
            },
        ],
    )

    vehicle_detector = Node(
        package='vehicle_detection',
        executable='vehicle_detector_node',
        name='vehicle_detector_node',
        output='screen',
        parameters=[
            # FR-014: ROS 2 launch merges YAML / dict entries left-to-right
            # ("last wins"), so the override order is
            #   detector_params.yaml -> presets/<name>.yaml -> launch dict.
            # detector_preset:=default points at a no-op overlay so this
            # keeps the existing behaviour when no preset is selected.
            params_file,
            detector_preset_file,
            {
                'target_frame_id': target_frame_id,
            },
        ],
        condition=IfCondition(use_detector),
    )

    detection_sender = Node(
        package='vehicle_detection',
        executable='detection_sender_node',
        name='detection_sender_node',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(use_sender),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
    )

    # Identity static transform target_frame_id -> input_frame_id.
    # transforms.yaml documents the canonical extrinsics; this matches the
    # use_identity_if_missing=true default until measured values are wired in.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_target_to_input',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', target_frame_id,
            '--child-frame-id', input_frame_id,
        ],
    )

    parameter_bridge = Node(
        package='vehicle_detection',
        executable='parameter_bridge_node',
        name='parameter_bridge_node',
        output='screen',
        parameters=[
            params_file,
            {'host': gui_host},
        ],
        condition=IfCondition(use_gui),
    )

    return LaunchDescription([
        pcd_file_arg,
        pcd_directory_arg,
        pcd_glob_arg,
        loop_arg,
        input_frame_id_arg,
        target_frame_id_arg,
        publish_once_arg,
        params_file_arg,
        use_detector_arg,
        use_sender_arg,
        use_rviz_arg,
        rviz_config_arg,
        use_gui_arg,
        gui_host_arg,
        detector_preset_arg,
        OpaqueFunction(function=_resolve_pcd_file),
        OpaqueFunction(function=_resolve_detector_preset),
        static_tf,
        pcd_loader,
        vehicle_detector,
        detection_sender,
        rviz,
        parameter_bridge,
    ])
