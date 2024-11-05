#include <iostream>
#include <fstream>
#include <cstdlib>
#include <ctime>

#include "simulation/robot_diagram.hpp"
#include "simulation/abb_motion_planner.hpp"
#include "simulation/svh_motion_planner.hpp"
#include "simulation/package_path.hpp"

#include <drake/geometry/meshcat.h>
#include <drake/math/rigid_transform.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/constant_value_source.h>
#include <drake/systems/analysis/simulator.h>

#include <drake_ros/core/drake_ros.h>
#include <drake_ros/core/ros_interface_system.h>
#include <drake_ros/core/ros_publisher_system.h>
#include <drake_ros/core/ros_subscriber_system.h>
#include <drake_ros/core/geometry_conversions.h>

#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

class EndEffectorTranslator : public drake::systems::LeafSystem<double> {
public:
    explicit EndEffectorTranslator() {
        this->DeclareAbstractInputPort("ee_ros_command", *drake::AbstractValue::Make(geometry_msgs::msg::Pose()));
        this->DeclareAbstractOutputPort("ee_rigid_transform", &EndEffectorTranslator::calc_output);
    }

private:
    void calc_output(const drake::systems::Context<double>& context, drake::math::RigidTransformd* output) const {
        *output = drake_ros::core::RosPoseToRigidTransform(this->get_input_port().Eval<geometry_msgs::msg::Pose>(context));
    }
};

class JointTrajectoryTranslator : public drake::systems::LeafSystem<double> {
public:
    explicit JointTrajectoryTranslator() {
        this->DeclareAbstractInputPort("joint_trajectory_command", *drake::AbstractValue::Make(trajectory_msgs::msg::JointTrajectory()));
        this->DeclareVectorOutputPort("joint_trajectory_command_vector", 18, &JointTrajectoryTranslator::calc_output);
    }

private:
    void calc_output(const drake::systems::Context<double>& context, drake::systems::BasicVector<double>* vector) const {
        vector->SetZero();
    }
};

int main(int argc, char ** argv) {
    (void) argc;
    (void) argv;

    // Meshcat Instance
    auto meshcat = std::make_shared<drake::geometry::Meshcat>();

    // Drake DiagramBuilder to connect all the systems
    auto builder = drake::systems::DiagramBuilder<double>();

    // Drake ROS2 
    rclcpp::QoS qos{10};
    drake_ros::core::init();

    // Drake ROS2system
    auto ros_system = builder.AddSystem<drake_ros::core::RosInterfaceSystem>(std::make_unique<drake_ros::core::DrakeRos>("simulator_node"));

    // ROS2 publisher to output SVH DynamicJointStates
    auto joint_output = builder.AddSystem(drake_ros::core::RosPublisherSystem::Make<control_msgs::msg::DynamicJointState>("/dynamic_joint_states", qos, ros_system->get_ros_interface()));

    // ROS2 publisher to output ABB JointStates
    auto abb_joint_output = builder.AddSystem(drake_ros::core::RosPublisherSystem::Make<control_msgs::msg::DynamicJointState>("/abb_dynamic_joint_states", qos, ros_system->get_ros_interface()));

    // ROS2 subscriber to get the input desired trajectory
    auto joint_trajectory = builder.AddSystem(drake_ros::core::RosSubscriberSystem::Make<trajectory_msgs::msg::JointTrajectory>("/left_hand/joint_trajectory", qos, ros_system->get_ros_interface()));
    
    // ROS2 subscriber to get the desired end effector pose
    auto ee_pose = builder.AddSystem(drake_ros::core::RosSubscriberSystem::Make<geometry_msgs::msg::Pose>("/abb_irb1200/ee_pose", qos, ros_system->get_ros_interface()));

    // System to translate between ROS2 Pose messages and Drake RigidTransformd
    auto ee_trans = builder.AddSystem<EndEffectorTranslator>();
    builder.Connect(ee_pose->get_output_port(), ee_trans->get_input_port());

    // Add ABB Motion Planner which takes a pose and gives commands to the ABB to interpolate between the current point and the target point
    auto abb_motion_planner = builder.AddSystem<simulation::AbbMotionPlanner>(0.3);

    // Add Svh Motion Planner which takes a joint trajectory and converts it to a smoothly interpolated signal for the SVH motors
    auto svh_motion_planner = builder.AddSystem<simulation::SvhMotionPlanner>();
    
    // Add RobotDiagram system which contains all of the multibody simulation and low level controllers
    auto world_sdf_location = simulation::package_path::get_package_share_path("simulation") + std::string("world/world.sdf");
    auto system = builder.AddSystem<simulation::RobotDiagram>(0.001, meshcat, false, std::vector<std::string>{world_sdf_location});
    builder.Connect(abb_motion_planner->get_output_port(), system->GetInputPort("irb1200_desired_state"));

    builder.Connect(system->GetOutputPort("irb1200_state"), abb_motion_planner->GetInputPort("irb1200_estimated_state"));

    builder.Connect(ee_trans->get_output_port(), abb_motion_planner->GetInputPort("target_ee_location"));

    auto diagram = builder.Build();
    
    auto sim = drake::systems::Simulator<double>(std::move(diagram));
    sim.set_target_realtime_rate(1.0);

    while (true) {
        auto time = sim.get_context().get_time();
        auto result = sim.AdvanceTo(time + 1.0);
    }
}

    /* Test rigid transform
    auto X_WG = drake::math::RigidTransformd(
        drake::math::RotationMatrixd::MakeXRotation(M_PIf64) * drake::math::RotationMatrixd::MakeYRotation(M_PI_2f64), 
        Eigen::Vector3d{0.6, 0.05, 0.05}
    );
    */
