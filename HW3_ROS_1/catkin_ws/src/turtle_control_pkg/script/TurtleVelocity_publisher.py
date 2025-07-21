#! /usr/bin/env python

# step0.导包
import rospy
from turtle_control_pkg.msg import TurtleVelocity

# step1.初始化ROS节点
rospy.init_node('turtle_velocity_publisher')

# step2.创建发布者对象
pub = rospy.Publisher('turtle_velocity', TurtleVelocity, queue_size=10)
# 话题名称是 'turtle_velocity'，控制器将订阅这个话题来获取速度信息

# step3.组织被发布的消息，编写发布逻辑并发布消息
# 创建数据
msg = TurtleVelocity()
msg.linear_x = 1.0

# step4.设置发布频率
rate = rospy.Rate(1)

# 添加启动信息
rospy.loginfo("TurtleVelocity Publisher node started")
rospy.loginfo("Publishing velocity message: linear_x = %.2f", msg.linear_x)

# step5.发布消息
count = 0
while not rospy.is_shutdown():
    pub.publish(msg)
    count += 1
    rospy.loginfo("Published message #%d: linear_x = %.2f", count, msg.linear_x)
    rate.sleep()