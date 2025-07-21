/**
 * @file simple_turtle_controller.cpp
 * @brief 任务3: C++控制器 - 组合任务1的线速度和任务2的角速度
 * 按照用户理解：任务1发布线速度，任务2提供角速度，任务3组合控制
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

class SimpleTurtleController
{
private:
    ros::NodeHandle nh_;
    
    // 订阅器 - 接收任务1的线速度
    ros::Subscriber linear_vel_sub_;
    
    // 发布器 - 控制小乌龟
    ros::Publisher cmd_vel_pub_;
    
    // 任务2提供的固定角速度
    double angular_velocity_;
    
    // 当前接收到的线速度
    double current_linear_vel_;

public:
    SimpleTurtleController() : current_linear_vel_(0.0)
    {
        // 任务2: 从YAML读取固定角速度
        nh_.param("/turtle_params/angular_motion/angular_velocity", angular_velocity_, 0.5);
        
        // 初始化订阅器 - 接收任务1的线速度
        linear_vel_sub_ = nh_.subscribe("/turtle_linear_velocity", 10, 
                                       &SimpleTurtleController::linearVelocityCallback, this);
        
        // 初始化发布器 - 控制小乌龟
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
        
        ROS_INFO("=== 任务3: 简单控制器启动 ===");
        ROS_INFO("接收任务1的线速度，使用任务2的角速度: %.3f rad/s", angular_velocity_);
    }
    
    /**
     * @brief 接收任务1发布的线速度
     */
    void linearVelocityCallback(const std_msgs::Float32::ConstPtr& msg)
    {
        current_linear_vel_ = msg->data;
        
        // 任务3核心：组合线速度和角速度
        combineVelocitiesAndControl();
    }
    
    /**
     * @brief 任务3核心功能：组合任务1和任务2的速度
     */
    void combineVelocitiesAndControl()
    {
        geometry_msgs::Twist cmd;
        
        // 任务1提供：动态线速度
        cmd.linear.x = current_linear_vel_;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        
        // 任务2提供：固定角速度
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = angular_velocity_;
        
        // 发布组合后的控制命令
        cmd_vel_pub_.publish(cmd);
        
        // 简单日志
        static int count = 0;
        if (++count % 20 == 0)  // 每2秒打印一次
        {
            ROS_INFO("[任务3] 线速度: %.3f (任务1) + 角速度: %.3f (任务2)", 
                    current_linear_vel_, angular_velocity_);
        }
    }
    
    /**
     * @brief 运行控制器
     */
    void run()
    {
        ros::Rate rate(20);  // 20Hz控制频率
        
        ROS_INFO("任务3等待任务1的线速度输入...");
        
        while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
    }
};

/**
 * @brief 主函数
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_turtle_controller");
    
    SimpleTurtleController controller;
    controller.run();
    
    return 0;
}
