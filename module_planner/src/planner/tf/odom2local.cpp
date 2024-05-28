/*
订阅：
    self_interface::msg::ego_status       ego_status_msg     (100Hz）
发布：
    geometry_msgs::msg::TransformStamped  (20Hz）
*/
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "self_interface/msg/ego_status.hpp"

class LocalFramePublisher : public rclcpp::Node
{
public:
    LocalFramePublisher() : Node("local_frame_publisher")
    {
        ego_status_subscription_ = this->create_subscription<self_interface::msg::EgoStatus>(
            "ego_status_msg", 20, std::bind(&LocalFramePublisher::publishLocalFrame, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void publishLocalFrame(const self_interface::msg::EgoStatus::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "local_frame";
        transformStamped.transform.translation.x = msg->ego_x;
        transformStamped.transform.translation.y = msg->ego_y;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation.x = 0.0; 
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = sin(msg->ego_theta) / 2;
        transformStamped.transform.rotation.w = cos(msg->ego_theta) / 2;

        tf_broadcaster_->sendTransform(transformStamped);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<self_interface::msg::EgoStatus>::SharedPtr ego_status_subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalFramePublisher>());
    rclcpp::shutdown();
    return 0;
}
