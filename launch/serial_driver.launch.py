import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('nav_serial')
    
    #=========================================================================
    # 启动参数声明
    #=========================================================================
    
    args = [
        DeclareLaunchArgument(
            'namespace',
            default_value='red_standard_robot1',
            description='Robot namespace (e.g., red_standard_robot1)'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port device path'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Serial port baud rate'
        ),
        DeclareLaunchArgument(
            'send_rate',
            default_value='100',
            description='Send rate in Hz'
        ),
    ]
    
    #=========================================================================
    # 配置文件
    #=========================================================================
    
    config_file = os.path.join(pkg_share, 'config', 'serial_driver.yaml')
    
    #=========================================================================
    # 节点配置
    #=========================================================================
    
    serial_driver_node = Node(
        package='nav_serial',
        executable='serial_driver_node',
        name='serial_driver_node',
        output='screen',
        emulate_tty=True,  # 启用彩色输出
        parameters=[
            config_file,
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'send_rate': LaunchConfiguration('send_rate'),
            }
        ],
        remappings=[
            # 话题重映射（去掉前导斜杠以支持命名空间）
            ('cmd_vel', 'cmd_vel'),
            ('/serial/gimbal_joint_state', '/serial/gimbal_joint_state'),
            ('serial/reconnect', 'serial/reconnect'),
        ]
    )
    
    #=========================================================================
    # 带命名空间的节点组
    #=========================================================================
    
    serial_driver_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        SetRemap('/tf', 'tf'),
        SetRemap('/tf_static', 'tf_static'),
        serial_driver_node,
    ])
    
    return LaunchDescription([
        *args,
        serial_driver_group,
    ])
