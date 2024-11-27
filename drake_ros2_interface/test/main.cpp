#include <drake/systems/framework/diagram_builder.h>
#include "drake_ros2_interface/drake_ros2_interface.hpp"
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/context.h>
#include <drake/geometry/meshcat.h>

#include <iostream>

const double simulation_time = 10.0;
const double height = 5.0;
const double horizontal_stretch = 2.0 * height * 2.0295858 / simulation_time;

void DrawBasicLine(
    drake::geometry::Meshcat& mc, 
    const double x0, 
    const double y0, 
    const double x1, 
    const double y1,
    const std::string& name,
    const drake::geometry::Rgba& color
) {
    Eigen::Matrix<double, 3, 2> matrix;
    matrix << x0*horizontal_stretch,  x1*horizontal_stretch,
              0.0, 0.0,
              y0,  y1;
    
    mc.SetLine("/drake/2d_grid/" + name, matrix, 1.0, color);
}

void DrawLine(
    drake::geometry::Meshcat& mc,
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& y,
    const std::string& name,
    const drake::geometry::Rgba& color
) {
    Eigen::MatrixXd matrix;
    DRAKE_ASSERT(x.size() == y.size());
    DRAKE_ASSERT(x.size() > 1);
    matrix.resize(3, x.size());
    matrix(0, Eigen::all) = x * horizontal_stretch;
    matrix(1, Eigen::all).setZero();
    matrix(2, Eigen::all) = y;

    mc.SetLine("/drake/plots/" + name, matrix, 1.0, color);
}

void Create2dPlot(drake::geometry::Meshcat& mc, const double end_time) {
    mc.Set2dRenderMode(
        (drake::math::RigidTransformd)(Eigen::Vector3d{0, -1, 0}),
            0.0, end_time, 
        -height,   height
    );

    for (double i = -5.0; i < 5.1; i++) {
        if (i == 0.0) continue;
        std::string name = "horizontal_grid_" + std::to_string((int)i);
        DrawBasicLine(mc, 0.0, i, end_time, i, name, {0.7, 0.7, 0.7});
    }

    for (double i = 1.0; i < end_time + 0.1; i++) {
        std::string name = "vertical_grid_" + std::to_string((int)i);
        DrawBasicLine(mc, i, -5.0, i    , 5.0, name, {0.7, 0.7, 0.7});
    }

    DrawBasicLine(mc, 0.0, 0.0, end_time, 0.0, "horizontal_axis", {0.2, 0.2, 0.2});
    DrawBasicLine(mc, 0.0, 5.0, 0.0, -5.0, "vertical_axis", {0.2, 0.2, 0.2});   
}

int main() {
    auto builder = drake::systems::DiagramBuilder<double>();

    auto interface = builder.AddSystem<drake_ros2_interface::DrakeRos2Interface>(0.02, 0.02);
    auto input_vector = drake::systems::BasicVector<double>(18);
    input_vector.SetZero();
    auto source = builder.AddSystem<drake::systems::ConstantVectorSource<double>>(input_vector);

    auto sink_1 = builder.AddSystem<drake::systems::VectorLogSink<double>>(18);
    auto sink_2 = builder.AddSystem<drake::systems::VectorLogSink<double>>(9);

    builder.Connect(source->get_output_port(), interface->get_input_port(0));
    builder.Connect(interface->get_output_port(0), sink_1->get_input_port());
    builder.Connect(interface->get_output_port(1), sink_2->get_input_port());

    auto diagram = builder.Build();

    auto simulator = drake::systems::Simulator<double>(std::move(diagram));
    simulator.Initialize();
    simulator.set_target_realtime_rate(1.0);
    simulator.AdvanceTo(simulation_time);

    auto& context = simulator.get_context();

    auto log_1 = sink_1->GetLog(sink_1->GetMyContextFromRoot(context));
    auto log_2 = sink_2->GetLog(sink_2->GetMyContextFromRoot(context));

    auto mc = drake::geometry::Meshcat();
    Create2dPlot(mc, simulation_time);

    DrawLine(mc, log_1.sample_times(), log_1.data()(0, Eigen::all), "test", {1.0, 0.0, 1.0});

    char x;
    std::cin >> x;

    return 0;
}
