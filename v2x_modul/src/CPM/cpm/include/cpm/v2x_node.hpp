#ifndef V2X_NODE_HPP_EUIC2VFR
#define V2X_NODE_HPP_EUIC2VFR

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "autoware_perception_msgs/msg/predicted_objects.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <boost/asio/io_service.hpp>
#include "cpm/v2x_app.hpp"
#include "cpm/cpm_application.hpp"
#include "cpm/time_trigger.hpp"
#include "cpm/link_layer.hpp"
#include "cpm/ethernet_device.hpp"
#include "cpm/positioning.hpp"
#include "cpm/security.hpp"
#include "cpm/router_context.hpp"
#include <fstream>

namespace v2x
{
  class V2XNode : public rclcpp::Node
  {
  public:
    explicit V2XNode(const rclcpp::NodeOptions &node_options);
    V2XApp *app;
    void publishObjects(std::vector<CpmApplication::Object> *, int cpm_num);
    void publishCpmSenderObject(double, double, double);
    
    std::ofstream latency_log_file;

  private:
    void objectsCallback(const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg);
    void tfCallback(const tf2_msgs::msg::TFMessage::ConstSharedPtr msg);

    rclcpp::Subscription<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr objects_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Publisher<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr cpm_objects_pub_;
    rclcpp::Publisher<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr cpm_sender_pub_;

    double pos_lat_;
    double pos_lon_;
  };
}

#endif