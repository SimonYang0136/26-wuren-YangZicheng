#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS节点：turtle_velocity_publisher
功能：发布自定义TurtleVelocity消息，控制乌龟运动
作者：Student
日期：2025-07-20
"""

import rospy
import math
from std_msgs.msg import Float32, Header
from turtle_control_pkg.msg import TurtleVelocity

class TurtleVelocityPublisher:
    def __init__(self):
        """初始化发布器节点"""
        # 初始化ROS节点
        rospy.init_node('turtle_velocity_publisher', anonymous=True)
        
        # 创建发布器
        self.velocity_pub = rospy.Publisher('/turtle_velocity_cmd', TurtleVelocity, queue_size=10)
        self.linear_vel_pub = rospy.Publisher('/turtle_linear_velocity', Float32, queue_size=10)
        
        # 从参数服务器获取参数
        self.publish_rate = rospy.get_param('~publish_rate', 10)
        self.max_linear_vel = rospy.get_param('/turtle_params/motion/max_linear_velocity', 2.0)
        self.max_angular_vel = rospy.get_param('/turtle_params/motion/max_angular_velocity', 2.0)
        self.trajectory_type = rospy.get_param('/turtle_params/trajectory/type', 'circular')
        
        # 内部状态变量
        self.time_start = rospy.Time.now()
        self.trajectory_progress = 0.0
        
        # 设置发布频率
        self.rate = rospy.Rate(self.publish_rate)
        
        rospy.loginfo("Turtle Velocity Publisher节点已启动")
        rospy.loginfo(f"发布频率: {self.publish_rate} Hz")
        rospy.loginfo(f"轨迹类型: {self.trajectory_type}")
    
    def generate_trajectory_velocity(self):
        """根据轨迹类型生成速度命令"""
        current_time = rospy.Time.now()
        elapsed_time = (current_time - self.time_start).to_sec()
        
        if self.trajectory_type == "circular":
            # 圆形轨迹
            linear_x = self.max_linear_vel * 0.5
            angular_z = self.max_angular_vel * 0.3
            self.trajectory_progress = (elapsed_time * 0.1) % 1.0
            
        elif self.trajectory_type == "sinusoidal":
            # 正弦波轨迹
            linear_x = self.max_linear_vel * 0.5 * (1 + 0.5 * math.sin(elapsed_time))
            angular_z = self.max_angular_vel * 0.2 * math.cos(elapsed_time * 0.5)
            self.trajectory_progress = (elapsed_time * 0.05) % 1.0
            
        elif self.trajectory_type == "figure_eight":
            # 8字形轨迹
            linear_x = self.max_linear_vel * 0.6
            angular_z = self.max_angular_vel * 0.4 * math.sin(elapsed_time * 0.5)
            self.trajectory_progress = (elapsed_time * 0.08) % 1.0
            
        else:
            # 直线运动（默认）
            linear_x = self.max_linear_vel * 0.3
            angular_z = 0.0
            self.trajectory_progress = (elapsed_time * 0.1) % 1.0
        
        return linear_x, 0.0, angular_z
    
    def create_velocity_message(self, linear_x, linear_y, angular_z):
        """创建TurtleVelocity消息"""
        msg = TurtleVelocity()
        
        # 设置header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "turtle_frame"
        
        # 设置速度
        msg.linear_x = linear_x
        msg.linear_y = linear_y
        msg.angular_z = angular_z
        
        # 计算速度大小
        msg.velocity_magnitude = math.sqrt(linear_x**2 + linear_y**2)
        
        # 安全检查
        msg.is_emergency_stop = False
        if msg.velocity_magnitude > self.max_linear_vel or abs(angular_z) > self.max_angular_vel:
            msg.is_emergency_stop = True
            rospy.logwarn("速度超出安全限制，触发紧急停止")
        
        # 轨迹信息
        msg.trajectory_type = self.trajectory_type
        msg.trajectory_progress = self.trajectory_progress
        
        return msg
    
    def publish_velocities(self):
        """发布速度消息"""
        while not rospy.is_shutdown():
            try:
                # 生成轨迹速度
                linear_x, linear_y, angular_z = self.generate_trajectory_velocity()
                
                # 创建并发布自定义消息
                velocity_msg = self.create_velocity_message(linear_x, linear_y, angular_z)
                self.velocity_pub.publish(velocity_msg)
                
                # 发布线速度（Float32消息）
                linear_vel_msg = Float32()
                linear_vel_msg.data = velocity_msg.velocity_magnitude
                self.linear_vel_pub.publish(linear_vel_msg)
                
                # 打印状态信息
                if rospy.get_time() % 2 < 0.1:  # 每2秒打印一次
                    rospy.loginfo(f"发布速度 - 线速度: {linear_x:.2f}, 角速度: {angular_z:.2f}, "
                                f"轨迹进度: {self.trajectory_progress:.2f}")
                
                self.rate.sleep()
                
            except rospy.ROSInterruptException:
                rospy.loginfo("节点被中断")
                break
            except Exception as e:
                rospy.logerr(f"发布速度时出错: {e}")
                self.rate.sleep()

def main():
    """主函数"""
    try:
        publisher = TurtleVelocityPublisher()
        publisher.publish_velocities()
    except rospy.ROSInterruptException:
        rospy.loginfo("Turtle Velocity Publisher节点已关闭")
    except Exception as e:
        rospy.logerr(f"节点启动失败: {e}")

if __name__ == '__main__':
    main()
