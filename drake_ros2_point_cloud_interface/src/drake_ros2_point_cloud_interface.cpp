#include "drake_ros2_point_cloud_interface/visibility_control.h"
#include "drake_ros2_point_cloud_interface/drake_ros2_point_cloud_interface.hpp"

#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/perception/point_cloud.h>
#include <drake/perception/point_cloud_flags.h>

#include <drake_ros/core/drake_ros.h>
#include <drake_ros/core/ros_interface_system.h>
#include <drake_ros/core/ros_publisher_system.h>
#include <drake_ros/core/ros_subscriber_system.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace drake_ros2_interface {
using drake::perception::pc_flags::Fields;
using drake::perception::pc_flags::kXYZs;
using drake::perception::pc_flags::kRGBs;

class PointCloudConcatenate : public drake::systems::LeafSystem<double> {
public:
    PointCloudConcatenate() {
        this->DeclareAbstractInputPort("pc_1", *drake::AbstractValue::Make(sensor_msgs::msg::PointCloud2()));
        this->DeclareAbstractInputPort("pc_2", *drake::AbstractValue::Make(sensor_msgs::msg::PointCloud2()));

        this->DeclareAbstractOutputPort("pc_out", drake::perception::PointCloud(0, Fields(kXYZs | kRGBs)), &PointCloudConcatenate::output_callback);
    }
private:
    void output_callback(const drake::systems::Context<double>& context, drake::perception::PointCloud* output) const {
        auto pc1 = this->get_input_port(0).Eval<sensor_msgs::msg::PointCloud2>(context);
        auto pc2 = this->get_input_port(1).Eval<sensor_msgs::msg::PointCloud2>(context);

        output->Expand(output->size() + (pc1.height * pc1.width));

        /*Code needed here to copy data from pc1 and pc2 into the output point cloud*/
    }
};


PointCloudSubscriber::PointCloudSubscriber() {
    auto builder = drake::systems::DiagramBuilder<double>();

    drake_ros::core::init();
    auto interface = builder.AddSystem<drake_ros::core::RosInterfaceSystem>(std::make_unique<drake_ros::core::DrakeRos>("drake_ros2_point_cloud"));

    rclcpp::QoS qos{10};

    auto subscriber_1 = builder.AddSystem(
        drake_ros::core::RosSubscriberSystem::Make<sensor_msgs::msg::PointCloud2> (
            "/point_cloud_1",
            qos,
            interface->get_ros_interface()
        )
    );

    auto subscriber_2 = builder.AddSystem(
        drake_ros::core::RosSubscriberSystem::Make<sensor_msgs::msg::PointCloud2> (
            "/point_cloud_2",
            qos,
            interface->get_ros_interface()
        )
    );

}
}  // namespace drake_ros2_interface
