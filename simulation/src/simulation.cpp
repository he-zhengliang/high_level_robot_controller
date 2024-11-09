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


class DynamicJointStateMessageCreator : public drake::systems::LeafSystem<double> {
public:
    explicit DynamicJointStateMessageCreator() {
        this->DeclareVectorInputPort("svh_state", 18);
        this->DeclareVectorInputPort("svh_effort", 9);
        this->DeclareAbstractOutputPort("dynamic_joint_state_message", &DynamicJointStateMessageCreator::create_message);
    }

private:
    void create_message(const drake::systems::Context<double>& context, control_msgs::msg::DynamicJointState* message) const {
        auto state = this->get_input_port(0).Eval(context);
        auto effort = this->get_input_port(1).Eval(context);

        *message = control_msgs::msg::DynamicJointState();
        message->joint_names = names;

        message->interface_values.resize(9);
        for (int i = 0; i < 9; i++) {
            message->interface_values[i].interface_names = {"position", "velocity", "effort", "current"};
            message->interface_values[i].values = {state(i), state(i+9), effort(i), 0.0};
        }
    }

    const std::vector<std::string> names =  {
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
    auto joint_output = builder.AddSystem(
        drake_ros::core::RosPublisherSystem::Make<control_msgs::msg::DynamicJointState>(
            "/dynamic_joint_states", 
            qos, 
            ros_system->get_ros_interface(),
            {drake::systems::TriggerType::kPeriodic},
            0.02
        )
    );

    // ROS2 publisher to output ABB JointStates
    // auto abb_joint_output = builder.AddSystem(drake_ros::core::RosPublisherSystem::Make<control_msgs::msg::DynamicJointState>("/abb_dynamic_joint_states", qos, ros_system->get_ros_interface()));

    // ROS2 subscriber to get the input svh desired trajectory
    auto joint_trajectory = builder.AddSystem(drake_ros::core::RosSubscriberSystem::Make<trajectory_msgs::msg::JointTrajectory>("/left_hand/joint_trajectory", qos, ros_system->get_ros_interface()));
    
    // ROS2 subscriber to get the desired end effector pose
    auto ee_pose = builder.AddSystem(drake_ros::core::RosSubscriberSystem::Make<geometry_msgs::msg::Pose>("/abb_irb1200/ee_pose", qos, ros_system->get_ros_interface()));

    // Add ABB Motion Planner which takes a pose and gives commands to the ABB to interpolate between the current point and the target point
    auto abb_motion_planner = builder.AddSystem<simulation::AbbMotionPlanner>(0.3);

    // Add Svh Motion Planner which takes a joint trajectory and converts it to a smoothly interpolated signal for the SVH motors
    auto svh_motion_planner = builder.AddSystem<simulation::SvhMotionPlanner>();
    builder.Connect(joint_trajectory->get_output_port(), svh_motion_planner->get_input_port(0));
    
    // Add RobotDiagram system which contains all of the multibody simulation and low level controllers
    auto world_sdf_location = simulation::package_path::get_package_share_path("simulation") + std::string("world/world.sdf");
    auto system = builder.AddSystem<simulation::RobotDiagram>(0.001, meshcat, false, std::vector<std::string>{world_sdf_location});

    builder.Connect(abb_motion_planner->get_output_port(), system->GetInputPort("irb1200_desired_state"));
    builder.Connect(system->GetOutputPort("irb1200_state"), abb_motion_planner->GetInputPort("irb1200_estimated_state"));
    builder.Connect(ee_pose->get_output_port(), abb_motion_planner->GetInputPort("target_ee_location"));
    builder.Connect(svh_motion_planner->get_output_port(), system->GetInputPort("svh_desired_state"));
    builder.Connect(system->GetOutputPort("svh_state"), svh_motion_planner->get_input_port(1));

    auto djsmc = builder.AddSystem<DynamicJointStateMessageCreator>();
    builder.Connect(system->GetOutputPort("svh_state"), djsmc->get_input_port(0));
    builder.Connect(system->GetOutputPort("svh_net_actuation"), djsmc->get_input_port(1));
    builder.Connect(djsmc->get_output_port(), joint_output->get_input_port());

    auto abb_state_logger = builder.AddSystem<drake::systems::VectorLogSink<double>>(12);
    builder.Connect(system->GetOutputPort("irb1200_state"), abb_state_logger->get_input_port());
    auto abb_input_logger = builder.AddSystem<drake::systems::VectorLogSink<double>>(12);
    builder.Connect(abb_motion_planner->get_output_port(), abb_input_logger->get_input_port());
    auto svh_state_logger = builder.AddSystem<drake::systems::VectorLogSink<double>>(18);
    builder.Connect(system->GetOutputPort("svh_state"), svh_state_logger->get_input_port());
    auto svh_input_logger = builder.AddSystem<drake::systems::VectorLogSink<double>>(18);
    builder.Connect(svh_motion_planner->get_output_port(), svh_input_logger->get_input_port());

    auto diagram = builder.Build();

    {
        std::ofstream file;
        file.open("/home/alexm/diagram_log.txt");
        if (file.is_open()) {
            file << diagram->GetGraphvizString();
            file.close();
        } else {
            std::cerr << "Error opening file for writing" << std::endl;
        }
    }

    auto sim = drake::systems::Simulator<double>(std::move(diagram));
    sim.set_target_realtime_rate(1.0);

    meshcat->StartRecording();

    sim.AdvanceTo(10.0);

    meshcat->PublishRecording();


    std::ofstream file;
    file.open("/home/alexm/abb_log.txt");
    if (file.is_open()) {
        auto abb_state_log = abb_state_logger->GetLog(abb_state_logger->GetMyContextFromRoot(sim.get_context()));
        auto abb_input_log = abb_input_logger->GetLog(abb_input_logger->GetMyContextFromRoot(sim.get_context()));
        file 
        << "<abb_state>"
            << "<input_size>" << abb_state_log.get_input_size() << "</input_size>"
            << "<num_samples>" << abb_state_log.num_samples() << "</num_samples>"
            << "<sample_times>" << abb_state_log.sample_times() << "</sample_times>"
            << "<data>" << abb_state_log.data() << "</data>"
        << "</abb_state>"
        << "<abb_input>"
            << "<input_size>" << abb_input_log.get_input_size() << "</input_size>"
            << "<num_samples>" << abb_input_log.num_samples() << "</num_samples>"
            << "<sample_times>" << abb_input_log.sample_times() << "</sample_times>"
            << "<data>" << abb_input_log.data() << "</data>"
        << "</abb_input>"
        ;
        std::cout << "Wrote to file\n";
        file.close();
    }

    file.open("/home/alexm/svh_log.txt");
    if (file.is_open()) {
        auto svh_state_log = svh_state_logger->GetLog(svh_state_logger->GetMyContextFromRoot(sim.get_context()));
        auto svh_input_log = svh_input_logger->GetLog(svh_input_logger->GetMyContextFromRoot(sim.get_context()));
        file 
        << "<svh_state>"
            << "<input_size>" << svh_state_log.get_input_size() << "</input_size>"
            << "<num_samples>" << svh_state_log.num_samples() << "</num_samples>"
            << "<sample_times>" << svh_state_log.sample_times() << "</sample_times>"
            << "<data>" << svh_state_log.data() << "</data>"
        << "</svh_state>"
        << "<svh_input>"
            << "<input_size>" << svh_input_log.get_input_size() << "</input_size>"
            << "<num_samples>" << svh_input_log.num_samples() << "</num_samples>"
            << "<sample_times>" << svh_input_log.sample_times() << "</sample_times>"
            << "<data>" << svh_input_log.data() << "</data>"
        << "</svh_input>"
        ;
        std::cout << "Wrote to file\n";
        file.close();
    }


    int x;
    std::cin >> x;

    /*
    while (true) {
        auto time = sim.get_context().get_time();
        auto result = sim.AdvanceTo(time + 1.0);
    }*/
}

    /* Test rigid transform
    auto X_WG = drake::math::RigidTransformd(
        drake::math::RotationMatrixd::MakeXRotation(M_PIf64) * drake::math::RotationMatrixd::MakeYRotation(M_PI_2f64), 
        Eigen::Vector3d{0.6, 0.05, 0.05}
    );
    */
