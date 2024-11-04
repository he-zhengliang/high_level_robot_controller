#include <iostream>

#include "simulation/robot_diagram.hpp"
#include "simulation/abb_motion_planner.hpp"
#include "simulation/package_path.hpp"

#include <drake/geometry/meshcat.h>
#include <drake/math/rigid_transform.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/systems/analysis/simulator.h>

int main(int argc, char ** argv)
{
    (void) argc;
    (void) argv;

    auto meshcat = std::make_shared<drake::geometry::Meshcat>();

    auto builder = drake::systems::DiagramBuilder<double>();
    auto robot_diagram = builder.AddSystem<simulation::RobotDiagram>(0.001, meshcat);
    // std::cout << "\n\n" << robot_diagram->GetGraphvizString() << "\n\n" << std::endl;
    builder.ExportInput(robot_diagram->GetInputPort("svh_desired_state"), "svh_desired_state");
    builder.ExportInput(robot_diagram->GetInputPort("irb1200_desired_state"), "irb1200_desired_state");
    builder.ExportInput(robot_diagram->GetInputPort("svh_feed_forward_torque"), "svh_feed_forward_torque");
    auto diagram = builder.Build();

    std::cout << "built the diagram\n";
    auto context = diagram->CreateDefaultContext();

    // std::cout << "<Diagram>\n\n" << diagram->GetGraphvizString() << "\n\n</Diagram>\n";

    diagram->GetInputPort("svh_desired_state").FixValue(context.get(), Eigen::Vector<double, 18>::Zero());
    diagram->GetInputPort("irb1200_desired_state").FixValue(context.get(), Eigen::Vector<double, 12>::Zero());
    diagram->GetInputPort("svh_feed_forward_torque").FixValue(context.get(), Eigen::Vector<double, 9>::Zero());

    auto sim = drake::systems::Simulator<double>(std::move(diagram), std::move(context));
    meshcat->StartRecording();
    sim.Initialize();

    std::cout << "initialized the simulation\n";
    sim.AdvanceTo(1.0);
    meshcat->PublishRecording();
    int x;
    std::cin >> x;
    std::cout << x << "\n";
}
