# skyshard_simulation/launch/skyshard_sim.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Parametreleri Tanımla
    num_drones_arg = DeclareLaunchArgument('num_drones', default_value='5', description='Simule edilecek drone sayisi.')
    routing_method_arg = DeclareLaunchArgument('routing_method', default_value='prophet', description='Kullanilacak routing methodu (dijkstra/prophet).')
    
    # Dosya Yolları
    pkg_simulation = get_package_share_directory('skyshard_simulation')
    pkg_core = get_package_share_directory('skyshard_core')
    
    world_file = os.path.join(
        pkg_simulation, 
        'models', 
        'city_world', 
        'city_world.sdf'
    )
    
    rviz_config_file = os.path.join(
        pkg_simulation,
        'rviz',
        'skyshard_default.rviz' # Varsayılan Rviz konfigürasyonu (burada yaratılmadı, varsayıyoruz)
    )

    # 1. Gazebo Başlatma
    # ROS2 Galactic ve sonrası için Gazebo Classic veya Ignition/Fortress/Harmonic kullanılır.
    # Varsayılan Gazebo Classic (ros_ign_gazebo yerine gazebo_ros kullanıyorsak)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # 2. Rviz2 Başlatma
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_node',
        output='screen',
        arguments=['-d', rviz_config_file],
    )
    
    # 3. Router Node (skyshard_core)
    # Parametreler test senaryosuna göre ayarlanmalıdır
    router_node = Node(
        package='skyshard_core',
        executable='router_node',
        name='router_node',
        output='screen',
        parameters=[
            {'drone_count': LaunchConfiguration('num_drones')},
            {'leader_id': 1},
            {'data_size_mb': 100},
            {'redundancy_k': 3},
            {'num_shards': 10},
            {'routing_method': LaunchConfiguration('routing_method')},
            # Örnek Başlangıç Verileri (Droneların birbirine bağlı olduğunu varsayalım)
            {'node_resources': [
                "1 500 100 0.9 2.0",  # ID Storage(MB) BW(MBps) Rel(0-1) CPU(units)
                "2 750 150 0.95 1.5",
                "3 600 80 0.8 1.0",
                "4 1000 200 0.99 3.0",
                "5 400 50 0.7 0.8",
            ]},
            {'initial_links': [
                "1 2 50 5",   # U V BW(MBps) Latency(ms)
                "1 3 40 10",
                "2 3 30 15",
                "3 4 70 8",
                "4 5 10 20",
                "5 1 20 25",
            ]}
        ]
    )
    
    # 4. Scenario Manager Node (skyshard_core)
    scenario_manager_node = Node(
        package='skyshard_core',
        executable='scenario_manager',
        name='scenario_manager',
        output='screen',
        parameters=[
            {'main_drone_id': 1},
            {'link_fail_u': 2},
            {'link_fail_v': 3}
        ]
    )

    # 5. Spawn Manager Node (skyshard_simulation)
    spawn_manager_node = Node(
        package='skyshard_simulation',
        executable='spawn_manager',
        name='spawn_manager',
        output='screen',
        parameters=[
            {'num_drones': LaunchConfiguration('num_drones')}
        ]
    )
    
    return LaunchDescription([
        num_drones_arg,
        routing_method_arg,
        
        # Gazebo'yu başlat
        gazebo_launch,
        
        # ROS2 düğümlerini başlat
        router_node,
        scenario_manager_node,
        spawn_manager_node,
        
        # Rviz'i başlat (Görselleştirme için)
        rviz_node
    ])
