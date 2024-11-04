#include <iostream>
#include <fstream>
#include <cstdlib>
#include <ctime>

#include "simulation/robot_diagram.hpp"
#include "simulation/abb_motion_planner.hpp"
#include "simulation/package_path.hpp"

#include <drake/geometry/meshcat.h>
#include <drake/math/rigid_transform.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/constant_value_source.h>
#include <drake/systems/analysis/simulator.h>

const double max_travel = 0.5;
const double speed = 1.0;
const double sim_time = 2 * max_travel / speed;

int main(int argc, char ** argv)
{
    (void) argc;
    (void) argv;

    // Meshcat Instance
    auto meshcat = std::make_shared<drake::geometry::Meshcat>();

    auto builder = drake::systems::DiagramBuilder<double>();

    auto world_sdf_location = simulation::package_path::get_package_share_path("simulation") + std::string("world/world.sdf");
    auto system = builder.AddSystem<simulation::RobotDiagram>(0.001, meshcat, false, std::vector<std::string>{world_sdf_location});
    std::cout << "Added the RobotDiagram\n";

    auto abb_motion_planner = builder.AddSystem<simulation::AbbMotionPlanner>();
    std::cout << "Added the AbbMotionPlanner\n";

    // builder.Connect(abb_motion_planner->get_output_port(), system->GetInputPort("irb1200_desired_state"));

    builder.Connect(system->GetOutputPort("irb1200_state"), abb_motion_planner->GetInputPort("irb1200_estimated_state"));

    builder.ExportInput(abb_motion_planner->GetInputPort("target_ee_location"), "target_ee_location");
    builder.ExportInput(system->GetInputPort("irb1200_desired_state"), "irb1200_desired_state");
    builder.ExportInput(system->GetInputPort("svh_desired_state"), "svh_desired_state");

    std::cout << "Finalized assembling the diagram\n";
    auto diagram = builder.Build();
    std::cout << "Finalized building the diagram\n";
    auto context = diagram->CreateDefaultContext();

    auto X_WG = drake::math::RigidTransformd(
        drake::math::RotationMatrixd::MakeXRotation(M_PIf64) * drake::math::RotationMatrixd::MakeYRotation(M_PI_2f64), 
        Eigen::Vector3d{0.6, 0.05, 0.05}
    );

    diagram->GetInputPort("target_ee_location").FixValue(context.get(), X_WG);
    diagram->GetInputPort("svh_desired_state").FixValue(context.get(), Eigen::Vector<double, 18>::Zero());
    diagram->GetInputPort("irb1200_desired_state").FixValue(context.get(), Eigen::Vector<double, 12>::Zero());
    
    auto sim = drake::systems::Simulator<double>(std::move(diagram), std::move(context));
    sim.Initialize();
    // sim.set_target_realtime_rate(1.0);

    meshcat->StartRecording();

    auto result = sim.AdvanceTo(sim_time);

    if (result.succeeded()) {
        std::cout << "Simulation Succeeded\n";
    } else {
        std::cout << "Simulation Failed!\n";
    }

    meshcat->PublishRecording();

    int end_input;
    std::cin >> end_input;
}
