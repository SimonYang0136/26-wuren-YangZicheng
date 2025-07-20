/**
 * @file custom_publisher.cpp
 * @brief 严格按照模板实现的自定义消息发布器
 * @author Student
 * @date 2025-07-20
 */

#include <ros/ros.h>                                    // ros核心头文件
#include "turtle_control_pkg/TurtleVelocity.h"         // 自定义消息头文件 (package1/messagename.h)

int main(int argc, char *argv[])
{
    //1.初始化 ROS 节点
    ros::init(argc, argv, "turtle_velocity_pub");      // nodename_pub
    
    //2.创建 ROS 句柄
    ros::NodeHandle nh;
    
    //3.创建发布者对象
    ros::Publisher pub = nh.advertise<turtle_control_pkg::TurtleVelocity>("turtle_velocity_topic", 1000);
    // 对应模板: ros::Publisher pub = nh.advertise<package1::messagename>("topicname", 1000);
    
    //4.组织被发布的消息，编写发布逻辑并发布消息
    turtle_control_pkg::TurtleVelocity velocity_msg;   // 创建消息对象
    
    // 初始化消息内容
    int count = 0;
    
    ROS_INFO("Custom Publisher Started - 自定义发布器已启动");
    
    //5.设置循环频率
    ros::Rate rate(1);                                  // 1Hz频率
    
    //6.发布消息
    while (ros::ok())
    {
        // 组织消息内容
        velocity_msg.header.stamp = ros::Time::now();
        velocity_msg.header.frame_id = "base_link";
        
        velocity_msg.linear_x = 1.0 + 0.5 * sin(count * 0.1);
        velocity_msg.linear_y = 0.0;
        velocity_msg.angular_z = 0.5 * cos(count * 0.1);
        velocity_msg.velocity_magnitude = sqrt(velocity_msg.linear_x * velocity_msg.linear_x + 
                                              velocity_msg.linear_y * velocity_msg.linear_y);
        velocity_msg.is_emergency_stop = false;
        velocity_msg.trajectory_type = "custom_template";
        velocity_msg.trajectory_progress = (count % 100) / 100.0;
        
        // 发布消息
        pub.publish(velocity_msg);                      // 对应模板: pub.publish(name);
        
        ROS_INFO("发布消息 %d: 线速度=%.2f, 角速度=%.2f", count, velocity_msg.linear_x, velocity_msg.angular_z);
        
        count++;
        rate.sleep();                                   // 休眠保持频率
        ros::spinOnce();                               // 处理回调函数
    }
    
    return 0;
}
