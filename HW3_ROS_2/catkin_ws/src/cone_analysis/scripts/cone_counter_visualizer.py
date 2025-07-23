#!/usr/bin/env python3
"""
锥桶统计和可视化节点
功能：
1. 订阅锥桶检测数据
2. 统计红、蓝锥桶的数量
3. 在rviz中可视化锥桶，区分颜色
"""

"""
python订阅方模版：
1. 导入必要的包
2. 初始化ROS节点
3. 创建订阅器，订阅锥桶检测话题
4. 回调函数处理接收到的锥桶数据
5. rospy.spin()循环等待消息
"""

import rospy
from fsd_common_msgs.msg import ConeDetections
from visualization_msgs.msg import Marker, MarkerArray
from collections import defaultdict # 这是什么？
# defaultdict 是Python标准库 collections 模块中的一个字典类，
# 它是普通字典(dict)的子类，但有一个特殊功能：
# 当访问不存在的键时，会自动创建该键并给它一个默认值。
# 而普通字典试图访问不存在的键时，会报错
# 默认值：
# int()  # 返回 0
# str()  # 返回 ''
# list() # 返回 []
# set()  # 返回 set()

class ConeCounterVisualizer:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('cone_counter_visualizer')
        
        # 保存消息的header信息，后面的marker需要用到
        self.msg_header = None

        # 锥桶计数器
        self.cone_counts = defaultdict(int)  # 颜色: 数量
        self.total_processed = 0
        
        # 当前帧的锥桶数据
        self.current_cones = []
        
        # 订阅锥桶检测话题
        self.cone_sub = rospy.Subscriber(
            '/perception/lidar/cone_side',
            ConeDetections,
            self.cone_detection_callback, 
            # ConeCounterVisualizer类初始化时，就要调用类的回调函数，为什么？
            # 这样可以确保第一次接收到新消息时，就会调用该回调函数处理数据
            queue_size=10
        )
        
        # 发布可视化标记
        self.marker_pub = rospy.Publisher(
            '/cone_visualization',
            MarkerArray,
            queue_size=10
        )
        
        # 发布统计信息
        self.stats_timer = rospy.Timer(rospy.Duration(2.0), self.publish_stats)
        
        rospy.loginfo("=== Cone Counter and Visualizer Started ===")
        rospy.loginfo("Subscribing to: /perception/lidar/cone_side")
        rospy.loginfo("Publishing visualization to: /cone_visualization")
        
    def cone_detection_callback(self, msg):
        """处理锥桶检测消息"""
        # 保存消息的header信息
        self.msg_header = msg.header
        
        # 清空当前帧数据
        self.current_cones = []
        
        # 处理每个检测到的锥桶
        for cone in msg.cone_detections:
            color = cone.color.data
            position = cone.position
            
            # 统计锥桶数量
            self.cone_counts[color] += 1
            self.total_processed += 1
            
            # 保存当前帧的锥桶信息
            self.current_cones.append({
                'color': color,
                'position': position,
                'pose_confidence': cone.poseConfidence.data,
                'color_confidence': cone.colorConfidence.data
            })
        
        # 发布可视化标记
        self.publish_visualization()
        
        # 打印当前帧统计
        if len(msg.cone_detections) > 0:
            rospy.loginfo(f"Frame processed: {len(msg.cone_detections)} cones detected")
    
    def publish_visualization(self):
        """发布某一帧的rviz可视化标记"""
        marker_array = MarkerArray()
        
        for i, cone in enumerate(self.current_cones): # 用当前帧的锥桶列表创建迭代器
            marker = Marker()
            marker.header.frame_id = self.msg_header.frame_id # 坐标系 
            # 如何确定坐标系：坐标系是写在Header里的
            # 查看锥桶检测消息的header信息：
            # rostopic echo /perception/lidar/cone_side -n 1，这是只看一条的指令
            # 但是Cone消息没有header字段，需要从ConeDetections消息中获取header信息
            marker.header.stamp = rospy.Time.now()
            marker.ns = "cones"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            # 设置位置 - Point类型
            # pose下有两个字段：geometry_msgs/Point position
            # 和geometry_msgs/Quaternion orientation
            # x, y, z是位置坐标
            # orientation默认为(0, 0, 0, 1.0)(四元数)
            # 但如果需要旋转，可以设置为其他值
            marker.pose.position.x = cone['position'].x
            marker.pose.position.y = cone['position'].y
            marker.pose.position.z = 0.0
            
            # 设置四元数方向（必须设置，否则可能导致显示异常）
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            # 设置大小
            marker.scale.x = 0.3  # 直径
            marker.scale.y = 0.3  # 直径
            marker.scale.z = 0.5  # 高度
            
            # 根据颜色设置标记颜色
            if cone['color'] == 'r':    # 红色锥桶
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.8
            elif cone['color'] == 'b':  # 蓝色锥桶
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 0.8
            elif cone['color'] == 'y':  # 黄色锥桶
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.8
            elif cone['color'] == 'o':  # 橙色锥桶（如果有的话）
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
                marker.color.a = 0.8
            else:  # 未知颜色
                marker.color.r = 0.5
                marker.color.g = 0.5
                marker.color.b = 0.5
                marker.color.a = 0.5
            
            # 设置生存时间
            marker.lifetime = rospy.Duration(3.0)
            
            marker_array.markers.append(marker)
        
        # 发布标记数组
        self.marker_pub.publish(marker_array)
    
    def publish_stats(self, event):
        """定期发布统计信息"""
        if self.total_processed > 0:
            rospy.loginfo("=== Cone Statistics ===")
            for color, count in self.cone_counts.items():
                color_name = {
                    'r': 'Red',
                    'b': 'Blue',
                    'y': 'Yellow', 
                    'o': 'Orange'
                }.get(color, f'Unknown({color})')
            # dict.get(key, default_value)用于获取字典中指定键的值，
            # 如果键不存在则返回默认值
                rospy.loginfo(f"{color_name} cones: {count}")
            rospy.loginfo(f"Total cones processed: {self.total_processed}")
            rospy.loginfo("=======================")
    
    def run(self):
        """运行节点"""
        rospy.loginfo("Cone Counter and Visualizer ready...")
        rospy.loginfo("Waiting for cone detection messages...")
        rospy.loginfo("To view visualization, open rviz and add MarkerArray topic: /cone_visualization")
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down Cone Counter and Visualizer...")
            # 打印最终统计
            rospy.loginfo("=== Final Statistics ===")
            for color, count in self.cone_counts.items():
                color_name = {
                    'r': 'Red',
                    'b': 'Blue',
                    'y': 'Yellow',
                    'o': 'Orange'
                }.get(color, f'Unknown({color})')
                rospy.loginfo(f"{color_name} cones: {count}")
            rospy.loginfo(f"Total cones processed: {self.total_processed}")

if __name__ == '__main__':
    try:
        node = ConeCounterVisualizer()
        node.run()
    except rospy.ROSInterruptException:
        pass
