/**
 * @file custom_subscriber.cpp
 * @brief 严格按照模板实现的自定义消息订阅器
 * @author Student
 * @date 2025-07-20
 */

#include "ros/ros.h"                                    // ros核心头文件
#include "turtle_control_pkg/TurtleVelocity.h"         // 自定义消息头文件 (package1/messagename.h)

// 回调函数 - 对应模板: void callback(const package1::messagename::ConstPtr& name){}
void velocityCallback(const turtle_control_pkg::TurtleVelocity::ConstPtr& msg)
{
    // 处理接收到的消息
    ROS_INFO("接收到速度消息:");
    ROS_INFO("  时间戳: %f", msg->header.stamp.toSec());
    ROS_INFO("  线速度: x=%.2f, y=%.2f", msg->linear_x, msg->linear_y);
    ROS_INFO("  角速度: z=%.2f", msg->angular_z);
    ROS_INFO("  速度大小: %.2f", msg->velocity_magnitude);
    ROS_INFO("  轨迹类型: %s", msg->trajectory_type.c_str());
    ROS_INFO("  轨迹进度: %.2f%%", msg->trajectory_progress * 100.0);
    ROS_INFO("  紧急停止: %s", msg->is_emergency_stop ? "是" : "否");
    ROS_INFO("----------------------------------------");
}

int main(int argc, char *argv[])
{   
    //1.初始化 ROS 节点
    ros::init(argc, argv, "turtle_velocity_sub");      // nodename_sub
    
    //2.创建 ROS 句柄
    ros::NodeHandle nh;
    
    //3.创建订阅对象
    ros::Subscriber sub = nh.subscribe<turtle_control_pkg::TurtleVelocity>("turtle_velocity_topic", 10, velocityCallback);
    // 对应模板: ros::Subscriber sub = nh.subscribe<package1::messagename>("topicname", 10, callback);
    
    ROS_INFO("Custom Subscriber Started - 自定义订阅器已启动");
    ROS_INFO("等待接收 turtle_velocity_topic 话题上的消息...");
    
    //4.回调函数处理
    ros::spin();        // 循环等待回调函数
    
    return 0;
}
