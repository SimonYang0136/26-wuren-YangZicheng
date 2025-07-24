#include <ros/ros.h>
#include <fsd_common_msgs/ConeDetections.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <opencv2/opencv.hpp>
class Projection{
    private:
        ros::NodeHandle nh_;
        ros::Subscriber position_sub_;
        ros::Publisher image_pub_;  // 发布图像话题
        
        // 相机参数矩阵
        Eigen::Matrix<double, 3, 4> extrinsic_matrix_;
        Eigen::Matrix3d intrinsic_matrix_;
        
        // 图像尺寸
        const int IMAGE_WIDTH = 1280;
        const int IMAGE_HEIGHT = 360;

    public:
    Projection() {
        // 初始化相机参数矩阵
        extrinsic_matrix_ << 
            3.5594209875121074e-03, -9.9987761481865733e-01, -1.5234365979146680e-02, 8.9277270417879417e-02,
            1.9781062410225703e-03,  1.5241472820252011e-02, -9.9988188532544631e-01, 9.1100499695349946e-01,
            9.9999170877459420e-01,  3.5288653732390984e-03,  2.0321149683686368e-03, 1.9154049062915668e+00;
        
        intrinsic_matrix_ <<
            532.795, 0.0, 632.15,
            0.0, 532.72, -3.428,
            0.0, 0.0, 1.0;
        
        // 初始化订阅器和发布器
        position_sub_ = nh_.subscribe("/perception/lidar/cone_detections", 10, &Projection::ProjectionCallback, this);
        image_pub_ = nh_.advertise<sensor_msgs::Image>("/projected_image", 10);
        
        ROS_INFO("Projection node initialized.");
    }
    void ProjectionCallback(const fsd_common_msgs::ConeDetections::ConstPtr& msg) {
        // 创建一个白色背景的图像
        cv::Mat image(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));
        
        ROS_INFO("Received %lu cone detections", msg->cone_detections.size());
        
        // 遍历所有检测到的锥桶(cone[]本身就是一个数组)
        for (const fsd_common_msgs::Cone& cone : msg->cone_detections) {
            // 提取3D位置
            Eigen::Vector3d position(cone.position.x, cone.position.y, cone.position.z);
            
            // 投影到图像平面
            Eigen::Vector2d pixel;
            MatchInPixel(position, pixel);
            
            // 根据锥桶颜色设置绘制颜色
            cv::Scalar color = getColorFromConeColor(cone.color.data);
                
            // 在图像上绘制投影点
            cv::circle(image, cv::Point(static_cast<int>(pixel(0)), static_cast<int>(pixel(1))), 
                      8, color, -1);  // 半径8的实心圆
                
            ROS_INFO("Cone (%s) at (%.2f, %.2f, %.2f) projected to pixel (%.0f, %.0f)", 
                     cone.color.data.c_str(), position(0), position(1), position(2), pixel(0), pixel(1));
        }

        // 将图像转换为ROS消息格式并发布（移到循环外）
        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        image_pub_.publish(image_msg);
        // 关于为什么发布的是Image,而图像却转换为sensor_msgs::ImagePtr:
        // ImagePtr 等价于 boost::shared_ptr<sensor_msgs::Image>，即指向 sensor_msgs::Image 对象的智能指针
        // 而ROS 发布器的 publish() 方法可以接受智能指针类型的消息，自动解引用
    }
    void MatchInPixel(const Eigen::Vector3d& position, Eigen::Vector2d& pixel) {
        // 将世界坐标系下的点投影到图像平面
        Eigen::Vector4d cone_coordinates_in_world(position(0), position(1), position(2), 1.0);
        Eigen::Vector3d cone_coordinates_in_pixel = intrinsic_matrix_ * (extrinsic_matrix_ * cone_coordinates_in_world);
        
        // 归一化，填充输出的pixel
        pixel(0) = cone_coordinates_in_pixel(0) / cone_coordinates_in_pixel(2);
        pixel(1) = cone_coordinates_in_pixel(1) / cone_coordinates_in_pixel(2);
    }
    cv::Scalar getColorFromConeColor(const std::string& color_name) {
        if (color_name == "r") {
            return cv::Scalar(0, 0, 255);  // 红色
        } 
        else if (color_name == "b") {
            return cv::Scalar(255, 0, 0);  // 蓝色
        } 
        else {
            return cv::Scalar(0, 0, 0);  // 默认黑色
        }
    }
};

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "project_image_node");
    
    // 创建Projection对象
    Projection projection;
    
    ROS_INFO("Project Image node started.");
    
    // 进入ROS循环
    ros::spin();
    
    return 0;
}