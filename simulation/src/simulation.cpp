#include <iostream>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <unistd.h>
#include <pwd.h>

#include "simulation/robot_diagram.hpp"
#include "simulation/abb_motion_planner.hpp"
#include "simulation/svh_motion_planner.hpp"
#include "simulation/package_path.hpp"
#include "simulation/ros_message_creator.hpp"

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

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

int main() {
    // Meshcat Instance
    auto meshcat = std::make_shared<drake::geometry::Meshcat>();

    // Drake DiagramBuilder to connect all the systems
    auto builder = drake::systems::DiagramBuilder<double>();

    // Drake ROS2 Node for modelling the Schunk SVH controller
    rclcpp::QoS qos{10};
    drake_ros::core::init();
    auto ros_system = builder.AddSystem<drake_ros::core::RosInterfaceSystem>(std::make_unique<drake_ros::core::DrakeRos>("simulator_node"));

    // ROS2 publisher to output SVH JointStates  
    auto joint_output = builder.AddSystem(
        drake_ros::core::RosPublisherSystem::Make<sensor_msgs::msg::JointState>(
            "/dynamic_joint_states", 
            qos, 
            ros_system->get_ros_interface(),
            {drake::systems::TriggerType::kPeriodic},
            0.02
        )
    );

    // ROS2 publisher to output ABB JointStates (TODO: Replace with a udp publisher with abb egm Protobuf spec)
    auto abb_joint_output = builder.AddSystem(drake_ros::core::RosPublisherSystem::Make<sensor_msgs::msg::JointState>("/abb_dynamic_joint_states", qos, ros_system->get_ros_interface(), {drake::systems::TriggerType::kPeriodic}, 0.02));

    // Point cloud outputs (TODO: replace with a more accurate version?)
    auto cam0_pc_output = builder.AddSystem(drake_ros::core::RosPublisherSystem::Make<sensor_msgs::msg::PointCloud2>("/cam0_point_cloud", qos, ros_system->get_ros_interface(), {drake::systems::TriggerType::kForced}));
    auto cam1_pc_output = builder.AddSystem(drake_ros::core::RosPublisherSystem::Make<sensor_msgs::msg::PointCloud2>("/cam1_point_cloud", qos, ros_system->get_ros_interface(), {drake::systems::TriggerType::kForced}));

    // ROS2 subscriber to get the input svh desired trajectory
    auto joint_trajectory = builder.AddSystem(drake_ros::core::RosSubscriberSystem::Make<trajectory_msgs::msg::JointTrajectory>("/left_hand/joint_trajectory", qos, ros_system->get_ros_interface()));
    
    // ROS2 subscriber to get the desired end effector pose (TODO: Replace with a tcp client that messages the controller server)
    auto ee_pose = builder.AddSystem(drake_ros::core::RosSubscriberSystem::Make<geometry_msgs::msg::Pose>("/abb_irb1200/ee_pose", qos, ros_system->get_ros_interface()));

    // Add ABB Motion Planner which takes a pose and gives commands to the ABB to interpolate between the current point and the target point
    auto abb_motion_planner = builder.AddSystem<simulation::AbbMotionPlanner>("172.22.78.115");

    // Add Svh Motion Planner which takes a joint trajectory and converts it to a smoothly interpolated signal for the SVH motors
    auto svh_motion_planner = builder.AddSystem<simulation::SvhMotionPlanner>();
    builder.Connect(joint_trajectory->get_output_port(), svh_motion_planner->get_input_port(0));
    
    // Path world.sdf
    auto world_sdf_location = simulation::package_path::get_package_share_path("simulation") + std::string("world/world.sdf");

    // Add RobotDiagram system which contains all of the multibody simulation and low level controllers
    auto system = builder.AddSystem<simulation::RobotDiagram>(0.002, meshcat, false, std::vector<std::string>{world_sdf_location});

    builder.Connect(abb_motion_planner->get_output_port(), system->GetInputPort("irb1200_desired_state"));
    builder.Connect(system->GetOutputPort("irb1200_state"), abb_motion_planner->GetInputPort("irb1200_estimated_state"));
    builder.Connect(ee_pose->get_output_port(), abb_motion_planner->GetInputPort("target_ee_location"));
    builder.Connect(svh_motion_planner->get_output_port(), system->GetInputPort("svh_desired_state"));
    builder.Connect(system->GetOutputPort("svh_state"), svh_motion_planner->get_input_port(1));

    auto djsmc = builder.AddSystem<simulation::SvhJointStateMessageCreator>();
    builder.Connect(system->GetOutputPort("svh_state"), djsmc->get_input_port(0));
    builder.Connect(system->GetOutputPort("svh_net_actuation"), djsmc->get_input_port(1));
    builder.Connect(djsmc->get_output_port(), joint_output->get_input_port());

    auto abb_djsmc = builder.AddSystem<simulation::AbbJointStateMessageCreator>();
    builder.Connect(system->GetOutputPort("irb1200_state"), abb_djsmc->get_input_port(0));
    builder.Connect(abb_djsmc->get_output_port(), abb_joint_output->get_input_port(0));

    auto abb_state_logger = builder.AddSystem<drake::systems::VectorLogSink<double>>(12);
    builder.Connect(system->GetOutputPort("irb1200_state"), abb_state_logger->get_input_port());
    auto abb_input_logger = builder.AddSystem<drake::systems::VectorLogSink<double>>(12);
    builder.Connect(abb_motion_planner->get_output_port(), abb_input_logger->get_input_port());
    auto svh_state_logger = builder.AddSystem<drake::systems::VectorLogSink<double>>(18);
    builder.Connect(system->GetOutputPort("svh_state"), svh_state_logger->get_input_port());
    auto svh_input_logger = builder.AddSystem<drake::systems::VectorLogSink<double>>(18);
    builder.Connect(svh_motion_planner->get_output_port(), svh_input_logger->get_input_port());

    auto pc0 = builder.AddSystem<simulation::PointCloudMessageCreator>(false);
    auto pc1 = builder.AddSystem<simulation::PointCloudMessageCreator>(false);

    builder.Connect(system->GetOutputPort("cam0_point_cloud"), pc0->get_input_port());
    builder.Connect(system->GetOutputPort("cam1_point_cloud"), pc1->get_input_port());
    builder.Connect(pc0->get_output_port(), cam0_pc_output->get_input_port());
    builder.Connect(pc1->get_output_port(), cam1_pc_output->get_input_port());

    auto diagram = builder.Build();

    const char* homedir = getpwuid(getuid())->pw_dir;
    
    {
	char filepath[100];
	strcpy(filepath, homedir);
	strcat(filepath, "/simulation_logs/diagram_log.txt");
        std::ofstream file;
        file.open(filepath);
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

    while (true) {
        sim.AdvanceTo(sim.get_context().get_time() + 2.0);
        break;
    }
    
    drake::perception::PointCloud pc_eval_0 = system->GetOutputPort("cam0_point_cloud").Eval<drake::perception::PointCloud>(system->GetMyContextFromRoot(sim.get_context()));
    drake::perception::PointCloud pc_eval_1 = system->GetOutputPort("cam1_point_cloud").Eval<drake::perception::PointCloud>(system->GetMyContextFromRoot(sim.get_context()));
    auto pc = drake::perception::Concatenate({pc_eval_0, pc_eval_1}).VoxelizedDownSample(0.01, 8);

    meshcat->SetObject("aaah", pc);
    meshcat->SetTransform("aaah", drake::math::RigidTransformd());//system->GetOutputPort("cam0_pose").Eval<drake::math::RigidTransformd>(system->GetMyContextFromRoot(sim.get_context())));
    meshcat->SetTransform("aa", drake::math::RigidTransformd());//system->GetOutputPort("cam0_pose").Eval<drake::math::RigidTransformd>(system->GetMyContextFromRoot(sim.get_context())));
    meshcat->PublishRecording();

    {
    std::ofstream file;
    char filename[100];
    strcpy(filename, homedir);
    strcat(filename, "/simulation_logs/abb_log.txt");
    file.open(filename);
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
        std::cout << "Wrote to file '" << filename << "'\n";
        file.close();
    } else {
	std::cout << "Failed to open abb log file '" << filename << "'\n";
    }
    }

    {
    std::ofstream file;
    char filename[100];
    strcpy(filename, homedir);
    strcat(filename, "/simulation_logs/svh_log.txt");
    file.open(filename);
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
        std::cout << "Wrote to file'" << filename << "'\n";
        file.close();
    } else {
        std::cout << "Failed to open svh log file '" << filename << "'\n";
    }
    }

    std::cout << "Reached End\n";
    int x;
    std::cin >> x;
}
