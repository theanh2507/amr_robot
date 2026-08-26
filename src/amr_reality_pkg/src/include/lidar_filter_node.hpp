#ifndef LIDAR_FILTER_NODE_HPP
#define LIDAR_FILTER_NODE_HPP


#include <rclcpp_components/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <string>



class LidarFilterNode : public rcpcpp::Node
{
    public:
        explicit LidarFilterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

        // dam bao doi tuong thuoc lop con se luon bi giai phong thong qua con tro lop cha, destructor lop con luon duoc goi truoc
        virtual ~LidarFilterNode() = default;
        
    private:
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;            // cho phep nhieu con tro tro chung vao 1 vung nho
        rclcpp::Subscriptions<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        double outlier_threshold_;

        void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
};

#endif