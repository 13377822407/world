from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    
    odom_node = Node(
        package='world',
        executable='odom_publisher',
        name='odometry_node',
        output='screen' ,              # 输出方式，显示在屏幕上
        parameters=[
            {'linear_speed': 0.0},      # 默认静止,用 /cmd_vel 控制
            {'angular_speed': 0.0},
            {'publish_rate': 50.0},
            {'use_sim_time': use_sim_time}] 
            )
    

