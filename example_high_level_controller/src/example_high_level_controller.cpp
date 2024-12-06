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

// #define LOG_OUT

std::atomic_bool continue_sim = true;
void check_for_stop_signal() {
  int x;
  std::cin >> x;
  continue_sim = false;
}

int main(int argc, char** argv) {
  int in;
  auto builder = drake::systems::DiagramBuilder<double>();

  // Create an ABB driver system
  bool simulation;
  std::string ip_addr = "";
  if (argc > 1) {
    simulation = true;
    ip_addr += std::string(argv[1]);
  } else {
    simulation = false;
  }
  auto abb_driver = builder.AddSystem<controller::AbbDriver>(simulation, ip_addr);

  auto svh_driver = builder.AddSystem<drake_ros2_interface::DrakeRos2Interface>(0.01, 0.01);

  #ifdef LOG_OUT
  auto sink = builder.AddSystem<drake::systems::VectorLogSink<double>>(6, 0.001);
  builder.Connect(abb_driver->get_output_port(), sink->get_input_port());
  auto svh_sink = builder.AddSystem<drake::systems::VectorLogSink<double>>(18, 0.001);
  builder.Connect(svh_driver->GetOutputPort("svh_state"), svh_sink->get_input_port());
  auto svh_effort_sink = builder.AddSystem<drake::systems::VectorLogSink<double>>(9, 0.001);
  builder.Connect(svh_driver->GetOutputPort("svh_effort"), svh_effort_sink->get_input_port());
  #endif

  drake::Vector<double, 18> svh_target;
  for (size_t i = 0; i < 9; i++) {
    svh_target[i] = 0.4;
    svh_target[i+9] = 0.0;
  }
  auto svh_source = builder.AddSystem<drake::systems::ConstantVectorSource<double>>(svh_target);

  builder.Connect(svh_source->get_output_port(), svh_driver->get_input_port());

  builder.ExportInput(abb_driver->get_input_port(), "abb_input");
  
  auto diagram = builder.Build();
  auto d_context = diagram->CreateDefaultContext();

  auto& fix_id = diagram->GetInputPort("abb_input").FixValue(d_context.get(), drake::math::RigidTransformd(drake::math::RollPitchYawd(-M_PIf64, -M_PI_2f64, 0), Eigen::Vector3d{0.6, 0.0, 0.9}));


  auto sim = drake::systems::Simulator<double>(std::move(diagram), std::move(d_context));

  sim.set_target_realtime_rate(1.0);

  // Start the stop signal listener thread
  // std::thread stop_thread(check_for_stop_signal);

    // Create the simulator
  std::cout << "I'm ready to run\n";
  std::cin >> in;

  // Main loop to read user input and update robot arm position
  while (continue_sim) {
    sim.AdvanceTo(sim.get_context().get_time() + 0.2);

    // Read input from user (6 values for the coordinates)
    std::cout << "Enter new target coordinates (x, y, z, roll, pitch, yaw): ";
    double x, y, z, roll, pitch, yaw;
    std::cin >> x >> y >> z >> roll >> pitch >> yaw;

    // If the input is valid, update the position
    if (std::cin.fail()) {
      std::cout << "Invalid input. Please enter six numerical values." << std::endl;
      std::cin.clear(); // Clear the error flag
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Discard invalid input
      continue; // Skip to the next iteration of the loop
    }

    // Update the rigid transform target for the robot arm
    drake::math::RigidTransformd new_target(
        drake::math::RollPitchYawd(Eigen::Vector3d{roll * M_PIf64/180, pitch * M_PIf64/180, yaw * M_PIf64/180}), Eigen::Vector3d(x, y, z));

    fix_id.GetMutableData()->set_value<drake::math::RigidTransformd>(new_target);
  }

  // Wait for the stop signal thread to finish
  // stop_thread.join();


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

  printf("reached end of program\n");
  return 0;
}
