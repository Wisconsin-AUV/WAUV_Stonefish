import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
 
    home = os.path.expanduser('~')
    ardupilot_path = os.path.join(home, 'WAUV', 'ardupilot')
 
    return LaunchDescription([
 
        # ardusub launch comamnds
        ExecuteProcess(
            cmd=['bash', '-c',
                f'source {home}/.profile && cd {ardupilot_path} && '
                'sim_vehicle.py -v ArduSub -w -L RATBeach --map --console '
                '-f vectored_6dof --model JSON --out=udp:127.0.0.1:14551'],
            output='screen'
        ),

        # stonefish sim launch ( need to modify for custom world)
        ExecuteProcess(
            cmd=['bash', '-c',
                'source /opt/ros/humble/setup.bash && '
                f'source {home}/WAUV/WAUV_Stonefish/install/setup.bash && '
                'ros2 launch stonefish_bluerov2 bluerov2_sim.py'],
            output='screen'
        ),

        # mavros launch
        ExecuteProcess(
            cmd=['bash', '-c',
                'source /opt/ros/humble/setup.bash && '
                'ros2 run mavros mavros_node --ros-args '
                '-p fcu_url:="udp://127.0.0.1:14551@" '
                '-p use_sim_time:=false'],
            output='screen'
        ),

        # start ROS nodes
        Node(
            package='wauv_sim',
            executable='vehicle_manager',
            name='vehicle_manager',
            output='screen'
        ),

        Node(
            package='wauv_sim',
            executable='xbox_controller',
            name='xbox_controller',
            output='screen'
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
 
    ])