#include <iostream>
#include <fstream>

#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/systems/primitives/constant_value_source.h>
#include <drake/math/rigid_transform.h>
#include <drake/systems/analysis/simulator.h>
#include <abb_driver/abb_driver.hpp>

#include <drake_ros2_interface/drake_ros2_interface.hpp>

// #define LOG_OUT

int main() {
  int in;
  auto builder = drake::systems::DiagramBuilder<double>();

  auto abb_driver = builder.AddSystem<controller::AbbDriver>();

  #ifdef LOG_OUT
  auto sink = builder.AddSystem<drake::systems::VectorLogSink<double>>(6, 0.001);
  builder.Connect(abb_driver->get_output_port(), sink->get_input_port());
  #endif

  auto source = builder.AddSystem<drake::systems::ConstantValueSource<double>>(
    *drake::AbstractValue::Make(
      drake::math::RigidTransformd(
        drake::math::RotationMatrixd(), // ::MakeYRotation(M_PI_2f64),
        Eigen::Vector3d{0.6, 0.0, 0.9}
      )
    )
  );

  builder.Connect(source->get_output_port(), abb_driver->get_input_port());

  auto diagram = builder.Build();

  std::cout << "I'm ready to run\n";
  std::cin >> in;

  auto sim = drake::systems::Simulator<double>(std::move(diagram));

  sim.set_target_realtime_rate(1.0);

  while (true) {
    sim.AdvanceTo(10.0);
  }

  #ifdef LOG_OUT
  {
    std::ofstream file;
    file.open("/home/alexm/simulation_logs/abb_network_log.txt");
    if (file.is_open()) {
      auto abb_state_log = sink->GetLog(sink->GetMyContextFromRoot(sim.get_context()));
      file 
      << "<abb_state>"
          << "<input_size>" << abb_state_log.get_input_size() << "</input_size>"
          << "<num_samples>" << abb_state_log.num_samples() << "</num_samples>"
          << "<sample_times>" << abb_state_log.sample_times() << "</sample_times>"
          << "<data>" << abb_state_log.data() << "</data>"
      << "</abb_state>"
      ;
      std::cout << "Wrote to file '" << "abb_network_log.txt" << "'\n";
      file.close();
    } else {
	    std::cout << "Failed to open abb log file '" << "abb_network_log.txt" << "'\n";
    }
  }
  #endif


  return 0;
}
