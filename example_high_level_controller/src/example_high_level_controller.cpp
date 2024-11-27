#include <iostream>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/systems/primitives/constant_value_source.h>
#include <drake/math/rigid_transform.h>
#include <drake/systems/analysis/simulator.h>
#include <abb_driver/abb_driver.hpp>

int main(int argc, char ** argv) {
  int in;
  auto builder = drake::systems::DiagramBuilder<double>();
  auto abb_driver = builder.AddSystem<controller::AbbDriver>();

  auto sink = builder.AddSystem<drake::systems::VectorLogSink<double>>(6);
  auto source = builder.AddSystem<drake::systems::ConstantValueSource<double>>(
    *drake::AbstractValue::Make(
      drake::math::RigidTransformd(
        drake::math::RotationMatrixd::MakeYRotation(M_PI_2f64),
        Eigen::Vector3d{0.6, 0.0, 0.9}
      )
    )
  );

  builder.Connect(source->get_output_port(), abb_driver->get_input_port());
  builder.Connect(abb_driver->get_output_port(), sink->get_input_port());

  std::cout << "I'm ready to build\n";
  std::cin >> in;

  auto diagram = builder.Build();

  std::cout << "I'm ready to run\n";
  std::cin >> in;

  auto sim = drake::systems::Simulator<double>(std::move(diagram));

  sim.set_target_realtime_rate(1.0);
  sim.AdvanceTo(10.0);

  return 0;
}
