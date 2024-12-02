#include <iostream>
#include <fstream>
#include <signal.h>

#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/systems/primitives/constant_value_source.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/math/rigid_transform.h>
#include <drake/systems/analysis/simulator.h>

#include <abb_driver/abb_driver.hpp>
#include <drake_ros2_interface/drake_ros2_interface.hpp>

#define LOG_OUT

std::atomic_bool continue_sim = true;
void check_for_stop_signal() {
  int x;
  std::cin >> x;
  continue_sim = false;
}

int main() {
  int in;
  auto builder = drake::systems::DiagramBuilder<double>();

  auto abb_driver = builder.AddSystem<controller::AbbDriver>();

  auto svh_driver = builder.AddSystem<drake_ros2_interface::DrakeRos2Interface>(0.01, 0.01);

  #ifdef LOG_OUT
  auto sink = builder.AddSystem<drake::systems::VectorLogSink<double>>(6, 0.001);
  builder.Connect(abb_driver->get_output_port(), sink->get_input_port());
  auto svh_sink = builder.AddSystem<drake::systems::VectorLogSink<double>>(18, 0.001);
  builder.Connect(svh_driver->GetOutputPort("svh_state"), svh_sink->get_input_port());
  auto svh_effort_sink = builder.AddSystem<drake::systems::VectorLogSink<double>>(9, 0.001);
  builder.Connect(svh_driver->GetOutputPort("svh_effort"), svh_effort_sink->get_input_port());
  #endif

  auto source = builder.AddSystem<drake::systems::ConstantValueSource<double>>(
    *drake::AbstractValue::Make(
      drake::math::RigidTransformd(
        drake::math::RotationMatrixd(), // ::MakeYRotation(M_PI_2f64),
        Eigen::Vector3d{0.6, 0.0, 0.9}
      )
    )
  );

  drake::Vector<double, 18> svh_target;
  for (size_t i = 0; i < 9; i++) {
    svh_target[i] = 0.4;
    svh_target[i+9] = 0.0;
  }
  auto svh_source = builder.AddSystem<drake::systems::ConstantVectorSource<double>>(svh_target);

  builder.Connect(source->get_output_port(), abb_driver->get_input_port());
  builder.Connect(svh_source->get_output_port(), svh_driver->get_input_port());

  auto diagram = builder.Build();

  std::cout << "I'm ready to run\n";
  std::cin >> in;

  auto sim = drake::systems::Simulator<double>(std::move(diagram));

  sim.set_target_realtime_rate(1.0);

  std::thread stop_thread(check_for_stop_signal);

  while (continue_sim) {
    sim.AdvanceTo(sim.get_context().get_time() + 0.2);
  }

  stop_thread.join();

  #ifdef LOG_OUT
  {
    std::ofstream file;
    file.open("/home/alexm/simulation_logs/abb_network_log.txt");
    if (file.is_open()) {
      auto state_log = sink->GetLog(sink->GetMyContextFromRoot(sim.get_context()));
      file 
      << "<abb_state>"
          << "<input_size>" << state_log.get_input_size() << "</input_size>"
          << "<num_samples>" << state_log.num_samples() << "</num_samples>"
          << "<sample_times>" << state_log.sample_times() << "</sample_times>"
          << "<data>" << state_log.data() << "</data>"
      << "</abb_state>"
      ;
      std::cout << "Wrote to file '" << "abb_network_log.txt" << "'\n";
      file.close();
    } else {
	    std::cout << "Failed to open abb log file '" << "abb_network_log.txt" << "'\n";
    }
  }
  {
    std::ofstream file;
    file.open("/home/alexm/simulation_logs/svh_network_log.txt");
    if (file.is_open()) {
      auto state_log = svh_sink->GetLog(svh_sink->GetMyContextFromRoot(sim.get_context()));
      file 
      << "<abb_state>"
          << "<input_size>" << state_log.get_input_size() << "</input_size>"
          << "<num_samples>" << state_log.num_samples() << "</num_samples>"
          << "<sample_times>" << state_log.sample_times() << "</sample_times>"
          << "<data>" << state_log.data() << "</data>"
      << "</abb_state>"
      ;
      std::cout << "Wrote to file '" << "svh_network_log.txt" << "'\n";
      file.close();
    } else {
	    std::cout << "Failed to open svh log file '" << "svh_network_log.txt" << "'\n";
    }
  }
  {
    std::ofstream file;
    file.open("/home/alexm/simulation_logs/svh_effort_network_log.txt");
    if (file.is_open()) {
      auto state_log = svh_effort_sink->GetLog(svh_effort_sink->GetMyContextFromRoot(sim.get_context()));
      file 
      << "<abb_state>"
          << "<input_size>" << state_log.get_input_size() << "</input_size>"
          << "<num_samples>" << state_log.num_samples() << "</num_samples>"
          << "<sample_times>" << state_log.sample_times() << "</sample_times>"
          << "<data>" << state_log.data() << "</data>"
      << "</abb_state>"
      ;
      std::cout << "Wrote to file '" << "svh_effort_network_log.txt" << "'\n";
      file.close();
    } else {
	    std::cout << "Failed to open abb log file '" << "svh_effort_network_log.txt" << "'\n";
    }
  }
  #endif

  std::cout << "End of file" << std::endl;
  
  return 0;
}
