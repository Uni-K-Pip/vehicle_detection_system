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
and an optional RViz instance with the bundled detector configuration.
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

    pcd_file = LaunchConfiguration('pcd_file')
    input_frame_id = LaunchConfiguration('input_frame_id')
    target_frame_id = LaunchConfiguration('target_frame_id')
    publish_once = LaunchConfiguration('publish_once')
    params_file = LaunchConfiguration('params_file')
    use_detector = LaunchConfiguration('use_detector')
    use_sender = LaunchConfiguration('use_sender')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    pcd_loader = Node(
        package='vehicle_detection',
        executable='pcd_loader_node',
        name='pcd_loader_node',
        output='screen',
        parameters=[
            params_file,
            {
                'pcd_file': pcd_file,
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
            params_file,
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

    return LaunchDescription([
        pcd_file_arg,
        input_frame_id_arg,
        target_frame_id_arg,
        publish_once_arg,
        params_file_arg,
        use_detector_arg,
        use_sender_arg,
        use_rviz_arg,
        rviz_config_arg,
        OpaqueFunction(function=_resolve_pcd_file),
        static_tf,
        pcd_loader,
        vehicle_detector,
        detection_sender,
        rviz,
    ])
