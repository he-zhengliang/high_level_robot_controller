#include "drake_ros2_schunk_interface/drake_ros2_schunk_interface.hpp"

#include <iostream>
#include <memory>
#include <array>

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/common/value.h>
#include <drake/systems/framework/event_status.h>

#include <drake_ros/core/drake_ros.h>
#include <drake_ros/core/ros_interface_system.h>
#include <drake_ros/core/ros_publisher_system.h>
#include <drake_ros/core/ros_subscriber_system.h>
#include <drake_ros/core/clock_system.h>

#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/header.hpp>

namespace drake_ros2_interface {

using drake::systems::DiagramBuilder;
using drake::systems::Context;
using drake::systems::LeafSystem;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::systems::DiscreteStateIndex;
using drake::systems::TriggerType;
using drake::AbstractValue;
using drake::Value;

using drake_ros::core::DrakeRos;
using drake_ros::core::RosInterfaceSystem;
using drake_ros::core::RosPublisherSystem;
using drake_ros::core::RosSubscriberSystem;

using trajectory_msgs::msg::JointTrajectory;
using trajectory_msgs::msg::JointTrajectoryPoint;
using sensor_msgs::msg::JointState;
using std_msgs::msg::Header;

namespace {

const std::vector<std::string> joint_names = {
    "Left_Hand_Thumb_Opposition",
    "Left_Hand_Thumb_Flexion",
    "Left_Hand_Index_Finger_Proximal",
    "Left_Hand_Index_Finger_Distal",
    "Left_Hand_Middle_Finger_Proximal",
    "Left_Hand_Middle_Finger_Distal",
    "Left_Hand_Finger_Spread",
    "Left_Hand_Pinky",
    "Left_Hand_Ring_Finger"
};

class SvhJointStateDecomposer : public LeafSystem<double> {
public:
    SvhJointStateDecomposer(const float subscriber_sampling_period) {
        this->DeclareAbstractInputPort("joint_state_ros", *AbstractValue::Make<JointState>());
        this->state_idx = this->DeclareDiscreteState(18);
        this->effort_idx = this->DeclareDiscreteState(9);
        this->DeclareStateOutputPort("svh_state", this->state_idx);
        this->DeclareStateOutputPort("svh_effort", this->effort_idx);
        this->DeclarePeriodicDiscreteUpdateEvent(subscriber_sampling_period, 0.0, &SvhJointStateDecomposer::calc_state_output);
        this->DeclareInitializationDiscreteUpdateEvent(&SvhJointStateDecomposer::calc_state_output);
    }

private:
    EventStatus calc_state_output(const Context<double>& context, DiscreteValues<double>* vector) const {
        const auto& input_message = this->get_input_port(0).Eval<JointState>(context);
        
        size_t joint_names_size = input_message.name.size();
        for (size_t i = 0; i < joint_names_size; i++) {
            size_t idx = estimate_postition[i];
            while (!input_message.name[i].compare(joint_names[idx])) {
                idx++;
                if (idx >= joint_names_size) {
                    idx = 0;
                }
            }
        
            vector->get_mutable_value(0)[idx] = input_message.position[i];
            vector->get_mutable_value(0)[idx+9] = input_message.velocity[i];
            vector->get_mutable_value(1)[idx] = input_message.effort[i];
        }

        return EventStatus::Succeeded();
    }

    DiscreteStateIndex state_idx;
    DiscreteStateIndex effort_idx;

    std::array<size_t, 9> estimate_postition = {3, 1, 5, 0, 8, 4, 7, 2, 6};

};

class SvhJointTrajectoryBuilder : public LeafSystem<double> {
public:
    SvhJointTrajectoryBuilder() {
        this->DeclareVectorInputPort("svh_desired_trajectory", 18);
        this->DeclareAbstractOutputPort("joint_trajectory_ros", &SvhJointTrajectoryBuilder::calc_output);
    }

private:
    void calc_output(const Context<double>& context, JointTrajectory* value) const {
        value->set__header(std_msgs::msg::Header());
        value->set__joint_names(joint_names);
        auto pt = JointTrajectoryPoint();
        pt.positions.resize(9);
        pt.velocities.resize(9);

        Eigen::Vector<double, 9>::Map(&pt.positions[0]) = this->get_input_port(0).Eval(context)(Eigen::seqN(0,9));
        Eigen::Vector<double, 9>::Map(&pt.velocities[0]) = this->get_input_port(0).Eval(context)(Eigen::seqN(9,9));

        value->set__points(std::vector<JointTrajectoryPoint>{pt});
    }
};

}

DrakeRos2Interface::DrakeRos2Interface(const float publish_period, const float subscriber_sample_period) {
    auto builder = DiagramBuilder<double>();

    drake_ros::core::init();
    auto interface = builder.AddSystem<RosInterfaceSystem>(std::make_unique<DrakeRos>("drake_ros2_interface_node"));

    rclcpp::QoS qos{10};

    builder.AddSystem<drake_ros::core::ClockSystem>();

    auto state_subscriber = builder.AddSystem(
        RosSubscriberSystem::Make<JointState>(
            "/joint_states", 
            qos, 
            interface->get_ros_interface()
        )
    );

    auto control_publisher = builder.AddSystem(
        RosPublisherSystem::Make<JointTrajectory>(
            "/left_hand/joint_trajectory", 
            qos, 
            interface->get_ros_interface(), 
            std::unordered_set<TriggerType>{TriggerType::kPeriodic}, 
            publish_period
        )
    );

    auto state_translator = builder.AddSystem<SvhJointStateDecomposer>(subscriber_sample_period);
    auto msg_builder = builder.AddSystem<SvhJointTrajectoryBuilder>();

    builder.Connect(state_subscriber->get_output_port(0), state_translator->get_input_port(0));
    builder.ExportOutput(state_translator->get_output_port(0), "svh_state");
    builder.ExportOutput(state_translator->get_output_port(1), "svh_effort");   

    builder.Connect(msg_builder->get_output_port(0), control_publisher->get_input_port(0));
    builder.ExportInput(msg_builder->get_input_port(0), "svh_desired_state");

    builder.BuildInto(this);
}

}  // namespace drake_ros2_interface
