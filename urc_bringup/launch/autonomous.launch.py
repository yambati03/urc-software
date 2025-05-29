from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_urc_perception = get_package_share_directory("urc_perception")

    follower_server_node = Node(
        package="trajectory_following",
        executable="trajectory_following_FollowerActionServer",
        name="trajectory_following_FollowerActionServer",
    )

    planner_server_node = Node(
        package="path_planning",
        executable="path_planning_PlannerServer",
        name="path_planning_PlannerServer",
    )

    sick_node = Node(
        package="sick_scan_xd",
        executable="sick_generic_caller",
        parameters=[
            {
                "hostname": "192.168.1.10",
                "scanner_type": "sick_multiscan",
                "publish_frame_id": "lidar_link",
                "tf_base_frame_id": "lidar_link2",
                "publish_laserscan_segment_topic": "scan_segment",
                "publish_laserscan_fullframe_topic": "scan_fullframe",
                "custom_pointclouds": "cloud_unstructured_fullframe",
                "verbose_level": 0,
                "cloud_unstructured_fullframe": "coordinateNotation=0 updateMethod=0 echos=0,1,2 layers=1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 reflectors=0,1 infringed=0,1 rangeFilter=0.05,999,1 topic=/cloud_unstructured_fullframe frameid=lidar_link publish=1",
            }
        ],
        output="screen",
    )

    launch_traversability = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_urc_perception, "launch", "mapping.launch.py")
        )
    )

    gps_imu_localizer_node = Node(
        package="urc_localization",
        name="gps_imu_localizer",
        executable="urc_localization_GpsImuLocalizer",
        output="screen",
    )

    return LaunchDescription(
        [
            follower_server_node,
            planner_server_node,
            sick_node,
            launch_traversability,
            gps_imu_localizer_node,
        ]
    )
