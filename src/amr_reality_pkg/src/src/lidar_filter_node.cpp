#include <lidar_filter_node.hpp>


LidarFilterNode::LidarFilterNode(const rclcpp::NodeOptions & options) 
: Node("lidar_node_filter", option)
{
    // khai bao param
    this->declare_parameter<std::string>("source_topic", "/scan");
    this->declare_parameter<std::string>("pub_topic", "/scan_filtered");
    this->declare_parameter<double>("outlier_threshold", 0.2);

    // doc param
    std::string source_topic_name = this->get_parameter("source_topic").as_string();
    std::string pub_topic_name = this->get_parameter("pub_topic").as_string();
    this->get_parameter("outlier_threshold", outlier_threshold_);

    scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(pub_topic_name, rclcpp::QoS(10));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        source_topic_name, rclcpp::QoS(10),
        std::bind(&LidarFilterNode::lidarCallback, this, std::placeholders::_1));
}

void LidarFilterNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    int nRanges = scan->ranges.size();

    if (nRanges < 3) 
    {
        scan_pub_->publish(*scan);
        return;
    }

    auto new_scan = *scan;

    for (int i = 1; i < nRanges - 1; ++i)
    {
        float prev_range = new_scan.ranges[i - 1];
        float current_range = new_scan.ranges[i];
        float next_range = new_scan.ranges[i + 1];

        bool current_valid = std::isfinite(current_range) && 
                             current_range >= new_scan.range_min && 
                             current_range <= new_scan.range_max;

        if (!current_valid) 
        {
            continue;
        }

        // kiem tra xem co tia laser nao bi nhieu, do dai lon hon outlier_threshold so voi 2 tia laser ben canh ko
        if (std::abs(current_range - prev_range) > outlier_threshold_ &&
            std::abs(current_range - next_range) > outlier_threshold_)
        {
            new_scan.ranges[i] = std::numeric_limits<float>::infinity();

            if (!new_scan.intensities.empty() && i < static_cast<int>(new_scan.intensities.size())) 
            {
                new_scan.intensities[i] = 0.0f;
            }
        }
    }

    scan_pub_->publish(new_scan);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}