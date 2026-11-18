
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. USB Gamepad Reader (for Logitech F710 when plugged into USB 2.0 port)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'deadzone': 0.05, 'autorepeat_rate': 50.0}]
        ),

        # 2. Converts Gamepad Sticks -> /cmd_vel at 50 Hz (1.0 m/s max forward speed)
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[{
                'axis_linear.x': 1,        # Left stick vertical
                'scale_linear.x': 1.0,     # Full stick = 1.0 m/s survey speed!
                'axis_angular.yaw': 3,     # Right stick horizontal
                'scale_angular.yaw': 1.5,
                'enable_button': 4,        # Hold Left Bumper (LB) as deadman safety trigger!
                'require_enable_button': True
            }]
        ),

        # 3. Your Real-Time C++ Stepper & Safety Node (Pinned to Core 3)
        Node(
            package='gears_control',
            executable='gears_realtime_driver',
            name='gears_realtime_driver',
            output='screen'
        )
    ])