#ifndef DRAKE_ROS2_POINT_CLOUD_INTERFACE__DRAKE_ROS2_INTERFACE_HPP_
#define DRAKE_ROS2_POINT_CLOUD_INTERFACE__DRAKE_ROS2_INTERFACE_HPP_

#include "drake_ros2_point_cloud_interface/visibility_control.h"

#include <drake/systems/framework/diagram.h>
#include <drake/perception/point_cloud.h>

namespace drake_ros2_interface
{

class PointCloudSubscriber : public drake::systems::Diagram<double> {
public:
    explicit PointCloudSubscriber();

private:
};

}  // namespace drake_ros2_interface

#endif  // DRAKE_ROS2_INTERFACE__DRAKE_ROS2_INTERFACE_HPP_
