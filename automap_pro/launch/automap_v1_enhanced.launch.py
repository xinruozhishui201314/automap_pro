#!/usr/bin/env python3
# ══════════════════════════════════════════════════════════════════════════════
# AutoMap-Pro v2.0 V1优化版本 Launch文件
#
# 功能：
# 1. CompressedImage解压缩节点
# 2. 多相机管理器（支持Cam-head/Cam-left/Cam-right切换）
# 3. IMU在线标定器（偏置估计 + 动态噪声调整）
# 4. LiDAR-Camera外参在线标定
# 5. 增强的子图管理（自适应大小 + 质量评估）
# 6. 多会话建图（跨会话回环 + 地图合并）
# ══════════════════════════════════════════════════════════════════════════════

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ──────────────────────────────────────────────────────────────
    # Launch参数
    # ──────────────────────────────────────────────────────────────
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='automap_pro/config/system_config_M2DGR.yaml',
        description='系统配置文件路径'
    )
    
    dataset_path_arg = DeclareLaunchArgument(
        'dataset_path',
        default_value='/home/wqs/Documents/github/automap_pro/data/automap_input/M2DGR/street_03_ros2',
        description='M2DGR数据集路径'
    )
    
    active_camera_arg = DeclareLaunchArgument(
        'active_camera',
        default_value='head',
        description='活跃相机: head/left/right'
    )
    
    enable_multi_camera_arg = DeclareLaunchArgument(
        'enable_multi_camera',
        default_value='false',
        description='启用多相机模式'
    )
    
    enable_imu_calibration_arg = DeclareLaunchArgument(
        'enable_imu_calibration',
        default_value='true',
        description='启用IMU在线标定'
    )
    
    enable_extrinsic_calibration_arg = DeclareLaunchArgument(
        'enable_extrinsic_calibration',
        default_value='true',
        description='启用外参在线标定'
    )
    
    use_composable_arg = DeclareLaunchArgument(
        'use_composable',
        default_value='true',
        description='使用组合节点模式'
    )
    
    # ──────────────────────────────────────────────────────────────
    # 读取配置
    # ──────────────────────────────────────────────────────────────
    config_file = LaunchConfiguration('config_file')
    dataset_path = LaunchConfiguration('dataset_path')
    
    # ──────────────────────────────────────────────────────────────
    # V1优化组件
    # ──────────────────────────────────────────────────────────────
    
    # 1. CompressedImage解压缩节点
    image_decompressor_node = Node(
        package='image_transport',
        executable='republish',
        name='image_decompressor_head',
        arguments=['compressed', 'raw'],
        parameters=[{
            'input_topic': '/camera/head/image_raw/compressed',
            'output_topic': '/camera/head/image_raw',
        }],
        remappings=[
            ('in', '/camera/head/image_raw/compressed'),
            ('out', '/camera/head/image_raw')
        ],
        output='screen'
    )
    
    # 2. 多相机管理器
    multi_camera_manager_node = Node(
        package='automap_pro',
        executable='multi_camera_manager',
        name='multi_camera_manager',
        parameters=[{
            'active_camera': LaunchConfiguration('active_camera'),
            'enable_multi_camera': LaunchConfiguration('enable_multi_camera'),
            'calibration_file': '',
            'auto_quality_switch': 'false',
            'quality_threshold': 50.0,
            'queue_size': 10,
        }],
        output='screen'
    )
    
    # 3. IMU在线标定器
    imu_online_calibrator_node = Node(
        package='automap_pro',
        executable='imu_online_calibrator',
        name='imu_online_calibrator',
        parameters=[{
            'imu_topic': '/handsfree/imu',
            'queue_size': 1000,
            'bias_window_size': 200,
            'noise_window_size': 1000,
            'static_threshold_acc': 0.1,
            'static_threshold_gyr': 0.05,
            'min_static_samples': 50,
            'bias_update_interval': 1.0,
            'noise_update_interval': 2.0,
            'enable_auto_noise_adjust': 'true',
            'noise_adjust_factor': 1.5,
        }],
        condition=LaunchConfiguration('enable_imu_calibration'),
        output='screen'
    )
    
    # 4. LiDAR-Camera外参在线标定器
    extrinsic_online_calibrator_node = Node(
        package='automap_pro',
        executable='extrinsic_online_calibrator',
        name='extrinsic_online_calibrator',
        parameters=[{
            'camera_topic': '/camera/head/image_raw',
            'lidar_topic': '/velodyne_points',
            'camera_info_topic': '/camera/head/camera_info',
            'queue_size': 30,
            # 相机内参（从system_config读取）
            'fx': 542.993253538048,
            'fy': 541.3882904458247,
            'cx': 629.0025857364897,
            'cy': 503.71809588651786,
            'k1': -0.057963907006683066,
            'k2': -0.026465594265953234,
            'p1': 0.011980216320790046,
            'p2': -0.003041081642470451,
            # 初始外参（LiDAR到相机）
            'init_trans_x': -0.07410,
            'init_trans_y': -0.00127,
            'init_trans_z': -0.65608,
            'init_roll': 0.0,
            'init_pitch': 0.0,
            'init_yaw': 0.0,
            # 标定参数
            'enable_online_optimization': 'true',
            'calib_interval': 1.0,
            'max_iter': 100,
            'converge_threshold': 1e-6,
            'project_error_threshold': 5.0,
            'min_keypoints': 50,
            'edge_threshold_low': 50,
            'edge_threshold_high': 150,
        }],
        condition=LaunchConfiguration('enable_extrinsic_calibration'),
        output='screen'
    )
    
    # 5. 多会话管理器
    multi_session_manager_node = Node(
        package='automap_pro',
        executable='multi_session_manager',
        name='multi_session_manager',
        parameters=[{
            'session_root_dir': '/data/automap_sessions',
            'max_sessions': 100,
            'auto_save_interval': 60.0,
            'enable_cross_session_loop': 'true',
            'cross_session_search_radius': 200.0,
            'cross_session_min_overlap': 0.15,
            'auto_merge_sessions': 'false',
            'merge_threshold': 0.8,
        }],
        output='screen'
    )
    
    # ──────────────────────────────────────────────────────────────
    # 主AutoMap系统（使用组合节点）
    # ──────────────────────────────────────────────────────────────
    
    # 如果使用组合节点模式
    composable_container = ComposableNodeContainer(
        name='automap_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # AutoMap系统组件
            ComposableNode(
                package='automap_pro',
                plugin='automap_pro::AutoMapSystem',
                name='automap_system',
                parameters=[config_file],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
        condition=LaunchConfiguration('use_composable')
    )
    
    # 如果使用独立节点模式
    automap_system_node = Node(
        package='automap_pro',
        executable='automap_system',
        name='automap_system',
        parameters=[config_file],
        output='screen',
        condition=LaunchConfiguration('use_composable', default_value='false').equals('false')
    )
    
    # ──────────────────────────────────────────────────────────────
    # 数据包回放（可选）
    # ──────────────────────────────────────────────────────────────
    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', 
             os.path.join(dataset_path, 'street_03_ros2.db3'),
             '--remap', '/velodyne_points:=/velodyne_points',
             '--remap', '/handsfree/imu:=/handsfree/imu',
             '--remap', '/camera/head/image_raw/compressed:=/camera/head/image_raw/compressed',
             '--remap', '/camera/left/image_raw/compressed:=/camera/left/image_raw/compressed',
             '--remap', '/camera/right/image_raw/compressed:=/camera/right/image_raw/compressed'],
        output='screen'
    )
    
    # ──────────────────────────────────────────────────────────────
    # RViz2可视化（可选）
    # ──────────────────────────────────────────────────────────────
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(FindPackageShare('automap_pro'), 'rviz', 'automap_v1.rviz')],
        output='screen'
    )
    
    # ──────────────────────────────────────────────────────────────
    # 返回Launch描述
    # ──────────────────────────────────────────────────────────────
    return LaunchDescription([
        # 声明参数
        config_file_arg,
        dataset_path_arg,
        active_camera_arg,
        enable_multi_camera_arg,
        enable_imu_calibration_arg,
        enable_extrinsic_calibration_arg,
        use_composable_arg,
        
        # V1优化组件
        image_decompressor_node,
        multi_camera_manager_node,
        imu_online_calibrator_node,
        extrinsic_online_calibrator_node,
        multi_session_manager_node,
        
        # 主系统
        composable_container,
        automap_system_node,
        
        # 可选组件（注释掉默认不启动）
        # rosbag_play,
        # rviz2_node,
    ])
