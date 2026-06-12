# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # webcam node
    webcam_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='webcam_node',
    )

    # Topic relay needed for movement
    topic_relay_node = Node(
        package='topic_relay',
        executable='topic_relay_node',
        name='topic_relay_node',
    )

    # IMU data
    imu_node = Node(
        package='mpu6050',
        executable='mpu6050_node',
        name='mpu6050_node',
    )

    # lidar
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        output='screen',
        name='sllidar_node',
        parameters=[{
            'serial_port':'/dev/ttyUSB0',
            'serial_baudrate':460800,
            'frame_id':'laser',
            'angle_compensate':True,
            'inverted':False,
            'scan_mode':'Standard',
        }],
    ) 

    # STM32 reset
    stm32_reset_node = Node(
        package='stm32_reset',
        executable='stm32_reset_node',
        name='stm32_reset_node',
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("taubot_description"), "urdf", "taubot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("taubot_bringup"),
            "config",
            "taubot_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay start of control_node after stm32_reset`
    delay_control_node_after_stm32_reset = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=stm32_reset_node,
            on_exit=[control_node],
        )
    )

    # Delay start of robot_state_pub after stm32_reset`
    delay_robot_state_pub_after_stm32_reset = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=stm32_reset_node,
            on_exit=[robot_state_pub_node],
        )
    )

    # Delay start of joint_state_broadcaster_spawner after stm32_reset`
    delay_joint_state_broadcaster_spawner_after_stm32_reset = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=stm32_reset_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
#        webcam_node,
        topic_relay_node,
        imu_node,
        lidar_node,
        stm32_reset_node,
        delay_control_node_after_stm32_reset,
        delay_robot_state_pub_after_stm32_reset,
        delay_joint_state_broadcaster_spawner_after_stm32_reset,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)
