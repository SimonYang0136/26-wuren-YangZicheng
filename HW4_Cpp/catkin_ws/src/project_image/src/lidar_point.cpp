#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<std_msgs/Header.h>


#include<pcl/point_types.h> 
// 这是PCL的点类型定义头文件
// 比如，pcl::PointXYZI表示带强度值的3D点
#include<pcl/point_cloud.h>
//这是PCL的点云数据结构定义头文件
// 比如，pcl::PointCloud<pcl::PointXYZI>表示带强度值的3D点云
#include<pcl_conversions/pcl_conversions.h> 
// ROS和PCL之间转换需要的头文件


class PointCloudProcessor {
private:
    ros::NodeHandle nh_;
    ros::Subscriber pointcloud_sub_down, pointcloud_sub_mid, pointcloud_sub_up;
    ros::Publisher processed_pub_;
    pcl::PointCloud<pcl::PointXYZI> pcl_down, pcl_mid, pcl_up, processed_pointcloud;
    
    // 添加同步标志
    bool down_received, mid_received, up_received;
    
public:
    PointCloudProcessor() : down_received(false), mid_received(false), up_received(false) {
        // 初始化列表效率更高
        
        // 初始化订阅器和发布器
        pointcloud_sub_down = nh_.subscribe("/carla/ego_vehicle/lidar_down", 10, 
            &PointCloudProcessor::pointCloudCallbackDown, this);
        pointcloud_sub_mid = nh_.subscribe("/carla/ego_vehicle/lidar_mid", 10, 
            &PointCloudProcessor::pointCloudCallbackMid, this);
        pointcloud_sub_up = nh_.subscribe("/carla/ego_vehicle/lidar_up", 10, 
            &PointCloudProcessor::pointCloudCallbackUp, this);
        processed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/lidar_points", 10);
        
        ROS_INFO("PointCloudProcessor node initialized.");
    }

    // 分别处理三个雷达的数据
    void pointCloudCallbackDown(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        pcl::fromROSMsg(*msg, pcl_down);
        down_received = true;
        ROS_INFO("Received lidar_down: %lu points", pcl_down.points.size());
        checkAndProcess();
    }

    void pointCloudCallbackMid(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        pcl::fromROSMsg(*msg, pcl_mid);
        mid_received = true;
        ROS_INFO("Received lidar_mid: %lu points", pcl_mid.points.size());
        checkAndProcess();
    }

    void pointCloudCallbackUp(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        pcl::fromROSMsg(*msg, pcl_up);
        up_received = true;
        ROS_INFO("Received lidar_up: %lu points", pcl_up.points.size());
        checkAndProcess();
    }

    // 检查是否所有数据都收到，然后处理
    void checkAndProcess() {
        if (down_received && mid_received && up_received) {
            // 重置标志
            down_received = mid_received = up_received = false;
            
            // 处理和发布
            processAndPublish();
        }
    }

    void processAndPublish() {
        // 清空结果点云
        processed_pointcloud.clear();
        
        // 正确的点云合并方式(不是a+b+c!)
        processed_pointcloud += pcl_down;
        processed_pointcloud += pcl_mid;
        processed_pointcloud += pcl_up;
        
        // 处理强度值 - 使用引用才能修改原数据
        for (auto& point : processed_pointcloud.points) {
            point.intensity = point.intensity * 255.0f + 1.0f;
        }
        
        // 创建输出消息
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(processed_pointcloud, output_msg);
        
        // 设置消息头
        output_msg.header.stamp = ros::Time::now();
        output_msg.header.frame_id = "lidar";
        
        // 发布处理后的点云
        processed_pub_.publish(output_msg);
        
        ROS_INFO("Processed and published point cloud with %lu points", 
                 processed_pointcloud.points.size());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_processor");
    
    PointCloudProcessor processor;

    ROS_INFO("Waiting for point cloud data...");
    
    ros::spin();
    
    return 0;
}
       