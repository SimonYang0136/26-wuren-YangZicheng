/**
 * @file turtle_velocity_controller.cpp
 * @brief 控制器 - 订阅TurtleVelocity消息并控制小乌龟
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtle_control_pkg/TurtleVelocity.h> //别漏了

class TurtleVelocityController
{
private:
    ros::NodeHandle nh_;
    
    // 订阅器 - 接收TurtleVelocity消息
    ros::Subscriber turtle_vel_sub_;
    
    // 发布器 - 控制小乌龟
    ros::Publisher cmd_vel_pub_;
    
    // 固定角速度（可以从参数服务器读取）
    double angular_velocity_;
    
    // 当前接收到的线速度
    double current_linear_vel_;

public:
    TurtleVelocityController() : current_linear_vel_(0.0)
    {
        // 从参数服务器读取角速度，默认值0.5
        nh_.param("angular_velocity", angular_velocity_, 0.5);
        
        // 初始化订阅器 - 订阅turtle_velocity话题
        turtle_vel_sub_ = nh_.subscribe("turtle_velocity", 10, 
                                       &TurtleVelocityController::turtleVelocityCallback, this);
        
        // 初始化发布器 - 控制小乌龟
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
        
        //顺序很重要：先读参数，然后订阅器，最后发布器
        //原因：turtleVelocityCallback调用时需要用到angular_velocity_，回调函数处理完数据后才能发布控制命令

        ROS_INFO("=== TurtleVelocity Controller Started ===");
        ROS_INFO("Subscribing to topic: turtle_velocity");
        ROS_INFO("Using fixed angular velocity: %.3f rad/s", angular_velocity_);
    }
    
    /**
     * @brief 接收TurtleVelocity消息的回调函数
     */
    void turtleVelocityCallback(const turtle_control_pkg::TurtleVelocity::ConstPtr& msg)
    {
        current_linear_vel_ = msg->linear_x;
        
        // 收到消息立即组合速度并发布控制命令
        combineVelocitiesAndControl();
    }
    
    /**
     * @brief 核心功能：组合线速度和角速度，控制小乌龟
     */
    void combineVelocitiesAndControl()
    {
        geometry_msgs::Twist cmd;
        
        // 使用接收到的线速度
        cmd.linear.x = current_linear_vel_;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        
        // 使用固定角速度
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = angular_velocity_;
        
        // 发布控制命令
        cmd_vel_pub_.publish(cmd);
        
        // 打印日志信息
        static int count = 0;
        if (++count % 10 == 0)  // 每10次打印一次，避免刷屏
        {
            ROS_INFO("Turtle Control - Linear: %.3f, Angular: %.3f", 
                    current_linear_vel_, angular_velocity_);
        }
    }
    
    /**
     * @brief 运行控制器 - 纯事件驱动，收到消息就发布
     */
    void run()
    {
        ROS_INFO("Controller waiting for TurtleVelocity messages...");
        ROS_INFO("Event-driven mode: publish control commands upon receiving messages");
        
        // 纯事件驱动：只处理回调，不需要固定频率
        ros::spin();  // 等待并处理回调函数，替换 spinOnce() + rate.sleep()
    }
};

/**
 * @brief 主函数
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtle_velocity_controller");
    
    TurtleVelocityController controller;
    controller.run();
    
    return 0;
}
