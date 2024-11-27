#ifndef DRAKE_ROS2_INTERFACE__DRAKE_ROS2_INTERFACE_HPP_
#define DRAKE_ROS2_INTERFACE__DRAKE_ROS2_INTERFACE_HPP_

#include "drake_ros2_interface/visibility_control.h"

#include <drake/systems/framework/diagram.h>

namespace drake_ros2_interface
{

using drake::systems::Diagram;

class DrakeRos2Interface : public Diagram<double> {
public:
    DrakeRos2Interface(const float publish_period, const float subscriber_sample_period=0.02);
};

}  // namespace drake_ros2_interface

#endif  // DRAKE_ROS2_INTERFACE__DRAKE_ROS2_INTERFACE_HPP_
