#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
任务1: Python发布器 - 只发布线速度
按照PPT要求：创建py文件，自定义乌龟线速度消息类型并发布
重点：只关注线速度的动态变化
"""

import rospy
import math
from turtle_control_pkg.msg import TurtleVelocity

class SimpleLinearVelocityPublisher:
    def __init__(self):
        """初始化发布器节点 - 任务1专用"""
        # 初始化ROS节点
        rospy.init_node('simple_linear_publisher', anonymous=True)
        
        # 创建发布器 - 发布自定义TurtleVelocity消息
        self.linear_vel_pub = rospy.Publisher('/turtle_linear_velocity', TurtleVelocity, queue_size=10)
        
        # 从参数服务器获取基础参数
        self.base_speed = rospy.get_param('/turtle_params/trajectory/base_linear_speed', 1.0)
        self.wave_freq = rospy.get_param('/turtle_params/trajectory/wave_frequency', 0.2)
        self.wave_amp = rospy.get_param('/turtle_params/trajectory/wave_amplitude', 0.5)
        self.publish_rate = rospy.get_param('/turtle_velocity_publisher/publish_rate', 10)
        
        # 设置发布频率
        self.rate = rospy.Rate(self.publish_rate)
        self.start_time = rospy.Time.now()
        
        rospy.loginfo("=== 任务1: 简单线速度发布器已启动 ===")
        rospy.loginfo(f"基础速度: {self.base_speed} m/s")
        rospy.loginfo(f"波浪频率: {self.wave_freq} Hz")
        rospy.loginfo(f"发布频率: {self.publish_rate} Hz")
    
    def generate_linear_velocity(self):
        """生成动态变化的线速度 - 任务1的核心功能"""
        current_time = rospy.Time.now()
        elapsed_time = (current_time - self.start_time).to_sec()
        
        # 正弦波变化的线速度
        linear_velocity = self.base_speed + self.wave_amp * math.sin(elapsed_time * self.wave_freq * 2 * math.pi)
        
        # 确保速度为正值
        linear_velocity = max(0.1, linear_velocity)
        
        return linear_velocity
    
    def publish_linear_velocity(self):
        """发布线速度 - 任务1的主要工作"""
        rospy.loginfo("开始发布线速度...")
        
        while not rospy.is_shutdown():
            try:
                # 生成当前线速度
                linear_vel = self.generate_linear_velocity()
                
                # 创建并发布TurtleVelocity消息
                vel_msg = TurtleVelocity()
                vel_msg.header.stamp = rospy.Time.now()
                vel_msg.linear_x = linear_vel
                self.linear_vel_pub.publish(vel_msg)
                
                # 周期性打印状态
                if rospy.get_time() % 2 < 0.1:  # 每2秒打印一次
                    rospy.loginfo(f"[任务1] 发布线速度: {linear_vel:.3f} m/s")
                
                self.rate.sleep()
                
            except rospy.ROSInterruptException:
                rospy.loginfo("任务1发布器被中断")
                break
            except Exception as e:
                rospy.logerr(f"任务1发布时出错: {e}")
                self.rate.sleep()

def main():
    """主函数"""
    try:
        publisher = SimpleLinearVelocityPublisher()
        publisher.publish_linear_velocity()
    except rospy.ROSInterruptException:
        rospy.loginfo("任务1: 简单线速度发布器已关闭")
    except Exception as e:
        rospy.logerr(f"任务1启动失败: {e}")

if __name__ == '__main__':
    main()
