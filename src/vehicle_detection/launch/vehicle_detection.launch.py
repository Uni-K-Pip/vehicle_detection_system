"""Launch file for the vehicle_detection pipeline.

Stage v0.3.0 starts only pcd_loader_node and the static transform from
target_frame_id to input_frame_id. vehicle_detector_node and
detection_sender_node will be added in later milestones.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_pcd_file(context, *args, **kwargs):
    pcd_file = LaunchConfiguration("pcd_file").perform(context)
    if not Path(pcd_file).is_absolute():
        # Resolve relative to current working directory at launch time so
        # users can pass paths like data/pcd/sample.pcd from the workspace
        # root. Absolute paths are passed through.
        pcd_file = str(Path.cwd() / pcd_file)
    context.launch_configurations["pcd_file"] = pcd_file
    return []


def generate_launch_description():
    pkg_share = get_package_share_directory("vehicle_detection")
    default_params = str(Path(pkg_share) / "config" / "detector_params.yaml")

    pcd_file_arg = DeclareLaunchArgument(
        "pcd_file",
        default_value="data/pcd/sample.pcd",
        description="Path to the PCD file to publish on /input/points.",
    )
    input_frame_id_arg = DeclareLaunchArgument(
        "input_frame_id",
        default_value="lidar",
        description="frame_id stamped on the published PointCloud2.",
    )
    target_frame_id_arg = DeclareLaunchArgument(
        "target_frame_id",
        default_value="map",
        description="Target frame for downstream detection results.",
    )
    publish_once_arg = DeclareLaunchArgument(
        "publish_once",
        default_value="false",
        description="If true, publish a single frame and keep the node alive.",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="YAML parameter file passed to nodes.",
    )

    pcd_file = LaunchConfiguration("pcd_file")
    input_frame_id = LaunchConfiguration("input_frame_id")
    target_frame_id = LaunchConfiguration("target_frame_id")
    publish_once = LaunchConfiguration("publish_once")
    params_file = LaunchConfiguration("params_file")

    pcd_loader = Node(
        package="vehicle_detection",
        executable="pcd_loader_node",
        name="pcd_loader_node",
        output="screen",
        parameters=[
            params_file,
            {
                "pcd_file": pcd_file,
                "input_frame_id": input_frame_id,
                "publish_once": publish_once,
            },
        ],
    )

    # Identity static transform target_frame_id -> input_frame_id.
    # transforms.yaml documents the canonical extrinsics; this matches the
    # use_identity_if_missing=true default until measured values are wired in.
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_target_to_input",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", target_frame_id,
            "--child-frame-id", input_frame_id,
        ],
    )

    return LaunchDescription([
        pcd_file_arg,
        input_frame_id_arg,
        target_frame_id_arg,
        publish_once_arg,
        params_file_arg,
        OpaqueFunction(function=_resolve_pcd_file),
        static_tf,
        pcd_loader,
    ])
