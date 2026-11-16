"""Start the MuJoCo Tracer arena with ROS 2 wheel velocity control."""

from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def tracer_nodes(context):
    directory = Path(__file__).resolve().parent
    description = (directory / "tracer.urdf").read_text()
    controllers = str(directory / "controllers.yaml")
    manager_params = [controllers, {"use_sim_time": True}]
    web_enabled = IfCondition(LaunchConfiguration("web")).evaluate(context)

    if web_enabled:
        manager_params.append(str(directory / "manual_drive.yaml"))
        robot = ET.fromstring(description)
        hardware = robot.find("./ros2_control/hardware")
        hardware.clear()
        ET.SubElement(hardware, "plugin").text = (
            "joint_state_topic_hardware_interface/JointStateTopicSystem"
        )
        for name, value in {
            "joint_commands_topic": "/sim/wheel_commands",
            "joint_states_topic": "/sim/wheel_feedback",
            "trigger_joint_command_threshold": "-1",
        }.items():
            ET.SubElement(hardware, "param", name=name).text = value
        description = ET.tostring(robot, encoding="unicode")

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="sim",
            parameters=[{"robot_description": description, "use_sim_time": True}],
            remappings=[("joint_states", "/joint_states")],
        ),
        Node(
            package="controller_manager" if web_enabled else "mujoco_ros2_control",
            executable="ros2_control_node",
            namespace="sim",
            output="screen",
            parameters=manager_params,
            on_exit=Shutdown(),
        ),
    ]
    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "-c", "/sim/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "diff_drive_controller", "-c", "/sim/controller_manager",
                "--controller-ros-args",
                "--ros-args -r ~/cmd_vel:=/sim/cmd_vel -r ~/odom:=/sim/odom",
            ],
            output="screen",
        ),
    ]
    if not web_enabled:
        return nodes + spawners

    clock_ready = ExecuteProcess(
        cmd=["ros2", "topic", "echo", "/clock", "rosgraph_msgs/msg/Clock", "--once"],
        output="log",
    )
    return nodes + [
        ExecuteProcess(
            cmd=["python3", str(directory / "web_bridge.py")],
            output="screen",
            on_exit=Shutdown(),
        ),
        RegisterEventHandler(OnProcessExit(target_action=clock_ready, on_exit=spawners)),
        clock_ready,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("web", default_value="false", description="Enable the live web controls"),
        OpaqueFunction(function=tracer_nodes),
    ])
