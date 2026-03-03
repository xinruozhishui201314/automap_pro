#!/usr/bin/env python3
"""检查system_config.yaml配置与数据话题的匹配性"""

import yaml
import sys

def check_config():
    """检查配置文件"""
    print("=== 配置文件检查 ===")
    
    # 加载配置
    with open('automap_pro/config/system_config.yaml') as f:
        config = yaml.safe_load(f)
    
    # 加载数据metadata
    with open('data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2/metadata.yaml') as f:
        bag_info = yaml.safe_load(f)
    
    # 获取数据中的所有话题
    data_topics = set()
    for t in bag_info['rosbag2_bagfile_information']['topics_with_message_count']:
        data_topics.add(t['topic_metadata']['name'])
    
    # 检查配置中的话题
    sensor = config.get('sensor', {})
    print("\n--- 配置中的话题 ---")
    lidar_topic = sensor.get('lidar', {}).get('topic', 'NOT SET')
    imu_topic = sensor.get('imu', {}).get('topic', 'NOT SET')
    cam_left_topic = sensor.get('camera_left', {}).get('image_topic', 'NOT SET')
    cam_right_topic = sensor.get('camera_right', {}).get('image_topic', 'NOT SET')
    gps_topic = sensor.get('gps', {}).get('topic', 'NOT SET')
    
    print(f"LiDAR话题: {lidar_topic}")
    print(f"IMU话题: {imu_topic}")
    print(f"左相机话题: {cam_left_topic}")
    print(f"右相机话题: {cam_right_topic}")
    print(f"GPS话题: {gps_topic}")
    
    # 检查话题匹配性
    print("\n--- 话题匹配性检查 ---")
    
    critical_topics = {
        'LiDAR': lidar_topic,
        'IMU': imu_topic,
    }
    
    all_match = True
    for name, topic in critical_topics.items():
        if topic in data_topics:
            count = next(t['message_count'] for t in bag_info['rosbag2_bagfile_information']['topics_with_message_count'] 
                        if t['topic_metadata']['name'] == topic)
            print(f"✓ {name}: {topic} ({count} 条消息)")
        else:
            print(f"✗ {name}: {topic} (数据中不存在)")
            all_match = False
    
    # 检查传感器启用状态
    print("\n--- 传感器启用状态 ---")
    print(f"GPS启用: {sensor.get('gps', {}).get('enabled', False)}")
    print(f"相机启用: {sensor.get('camera', {}).get('enabled', False)}")
    
    # 系统配置
    system = config.get('system', {})
    print(f"\n--- 系统配置 ---")
    print(f"模式: {system.get('mode', 'NOT SET')}")
    print(f"输出目录: {system.get('output_dir', 'NOT SET')}")
    print(f"使用GPU: {system.get('use_gpu', False)}")
    
    # 前端配置
    frontend = config.get('frontend', {})
    print(f"\n--- 前端配置 ---")
    print(f"前端模式: {frontend.get('mode', 'NOT SET')}")
    print(f"外部前端启用: {frontend.get('external_fast_livo', {}).get('odom_topic', 'NOT SET')}")
    
    print("\n=== 配置检查完成 ===")
    return 0 if all_match else 1

if __name__ == '__main__':
    sys.exit(check_config())
