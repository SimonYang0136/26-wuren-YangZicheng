/**
 * @file turtle_controller.cpp
 * @brief ROS节点：C++乌龟控制器，订阅自定义速度消息并控制turtlesim
 * @author Student
 * @date 2025-07-20
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <std_msgs/Float32.h>
#include <turtle_control_pkg/TurtleVelocity.h>
#include <cmath>

class TurtleController
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅器
    ros::Subscriber velocity_sub_;
    ros::Subscriber linear_vel_sub_;
    ros::Subscriber pose_sub_;
    
    // 发布器
    ros::Publisher cmd_vel_pub_;
    ros::Publisher status_pub_;
    
    // 参数
    double control_rate_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    double safety_margin_;
    bool enable_safety_check_;
    
    // 状态变量
    turtlesim::Pose current_pose_;
    geometry_msgs::Twist current_cmd_;
    bool pose_received_;
    bool emergency_stop_;
    
    // 控制参数
    double kp_linear_;
    double kp_angular_;
    
public:
    /**
     * @brief 构造函数
     */
    TurtleController() : private_nh_("~"), pose_received_(false), emergency_stop_(false)
    {
        // 从参数服务器加载参数
        loadParameters();
        
        // 初始化订阅器
        velocity_sub_ = nh_.subscribe("/turtle_velocity_cmd", 10, 
                                    &TurtleController::velocityCallback, this);
        linear_vel_sub_ = nh_.subscribe("/turtle_linear_velocity", 10,
                                      &TurtleController::linearVelocityCallback, this);
        pose_sub_ = nh_.subscribe("/turtle1/pose", 10,
                                &TurtleController::poseCallback, this);
        
        // 初始化发布器
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
        status_pub_ = nh_.advertise<std_msgs::Float32>("/turtle_controller_status", 10);
        
        // 初始化命令
        current_cmd_.linear.x = 0.0;
        current_cmd_.linear.y = 0.0;
        current_cmd_.linear.z = 0.0;
        current_cmd_.angular.x = 0.0;
        current_cmd_.angular.y = 0.0;
        current_cmd_.angular.z = 0.0;
        
        ROS_INFO("Turtle Controller节点已启动");
        ROS_INFO("控制频率: %.1f Hz", control_rate_);
        ROS_INFO("最大线速度: %.2f m/s", max_linear_velocity_);
        ROS_INFO("最大角速度: %.2f rad/s", max_angular_velocity_);
    }
    
    /**
     * @brief 从参数服务器加载参数
     */
    void loadParameters()
    {
        // 控制参数
        private_nh_.param("control_rate", control_rate_, 20.0);
        
        // 速度限制参数
        nh_.param("/turtle_params/motion/max_linear_velocity", max_linear_velocity_, 2.0);
        nh_.param("/turtle_params/motion/max_angular_velocity", max_angular_velocity_, 2.0);
        
        // 安全参数
        nh_.param("/turtle_params/safety/enable_boundary_check", enable_safety_check_, true);
        nh_.param("/turtle_params/safety/margin", safety_margin_, 0.5);
        
        // 控制器增益
        nh_.param("/turtle_params/control/kp_linear", kp_linear_, 1.0);
        nh_.param("/turtle_params/control/kp_angular", kp_angular_, 1.0);
        
        ROS_INFO("参数加载完成");
    }
    
    /**
     * @brief 自定义速度消息回调函数
     */
    void velocityCallback(const turtle_control_pkg::TurtleVelocity::ConstPtr& msg)
    {
        // 检查紧急停止标志
        if (msg->is_emergency_stop)
        {
            emergency_stop_ = true;
            ROS_WARN("接收到紧急停止信号");
            stopTurtle();
            return;
        }
        else
        {
            emergency_stop_ = false;
        }
        
        // 应用控制增益
        double linear_x = msg->linear_x * kp_linear_;
        double angular_z = msg->angular_z * kp_angular_;
        
        // 速度限制
        linear_x = std::max(-max_linear_velocity_, std::min(max_linear_velocity_, linear_x));
        angular_z = std::max(-max_angular_velocity_, std::min(max_angular_velocity_, angular_z));
        
        // 安全检查
        if (enable_safety_check_ && pose_received_)
        {
            if (!isSafeToMove(linear_x, angular_z))
            {
                ROS_WARN("安全检查失败，停止运动");
                stopTurtle();
                return;
            }
        }
        
        // 更新命令
        current_cmd_.linear.x = linear_x;
        current_cmd_.angular.z = angular_z;
        
        // 发布控制命令
        cmd_vel_pub_.publish(current_cmd_);
        
        // 发布状态信息
        publishStatus(msg);
        
        // 打印调试信息
        static int count = 0;
        if (++count % 50 == 0)  // 每2.5秒打印一次（假设20Hz）
        {
            ROS_INFO("轨迹: %s, 进度: %.2f%%, 线速度: %.2f, 角速度: %.2f",
                    msg->trajectory_type.c_str(),
                    msg->trajectory_progress * 100.0,
                    linear_x, angular_z);
        }
    }
    
    /**
     * @brief 线速度消息回调函数
     */
    void linearVelocityCallback(const std_msgs::Float32::ConstPtr& msg)
    {
        // 这里可以处理额外的线速度信息
        static ros::Time last_log_time = ros::Time::now();
        ros::Time now = ros::Time::now();
        
        if ((now - last_log_time).toSec() > 5.0)  // 每5秒记录一次
        {
            ROS_DEBUG("当前线速度大小: %.3f m/s", msg->data);
            last_log_time = now;
        }
    }
    
    /**
     * @brief 乌龟位姿回调函数
     */
    void poseCallback(const turtlesim::Pose::ConstPtr& msg)
    {
        current_pose_ = *msg;
        pose_received_ = true;
    }
    
    /**
     * @brief 安全检查函数
     */
    bool isSafeToMove(double linear_vel, double angular_vel)
    {
        if (!pose_received_) return false;
        
        // 检查边界条件
        double future_x = current_pose_.x + linear_vel * cos(current_pose_.theta) * 0.1;
        double future_y = current_pose_.y + linear_vel * sin(current_pose_.theta) * 0.1;
        
        // turtlesim的边界大约是0到11
        if (future_x < safety_margin_ || future_x > (11.0 - safety_margin_) ||
            future_y < safety_margin_ || future_y > (11.0 - safety_margin_))
        {
            return false;
        }
        
        return true;
    }
    
    /**
     * @brief 停止乌龟运动
     */
    void stopTurtle()
    {
        current_cmd_.linear.x = 0.0;
        current_cmd_.angular.z = 0.0;
        cmd_vel_pub_.publish(current_cmd_);
    }
    
    /**
     * @brief 发布状态信息
     */
    void publishStatus(const turtle_control_pkg::TurtleVelocity::ConstPtr& msg)
    {
        std_msgs::Float32 status_msg;
        status_msg.data = msg->velocity_magnitude;
        status_pub_.publish(status_msg);
    }
    
    /**
     * @brief 运行控制循环
     */
    void run()
    {
        ros::Rate rate(control_rate_);
        
        while (ros::ok())
        {
            ros::spinOnce();
            
            // 如果长时间没有接收到速度命令，停止乌龟
            static ros::Time last_velocity_time = ros::Time::now();
            
            ros::Time now = ros::Time::now();
            if ((now - last_velocity_time).toSec() > 2.0)  // 2秒超时
            {
                if (!emergency_stop_)
                {
                    ROS_WARN_THROTTLE(5.0, "长时间未接收到速度命令，保持停止状态");
                    stopTurtle();
                }
                last_velocity_time = now;
            }
            
            rate.sleep();
        }
    }
};

/**
 * @brief 主函数
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtle_controller");
    
    try
    {
        TurtleController controller;
        controller.run();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Turtle Controller异常: %s", e.what());
        return 1;
    }
    
    ROS_INFO("Turtle Controller节点已关闭");
    return 0;
}
