#ifndef SIMULATION__ABB_MOTION_PLANNER_HPP
#define SIMULATION__ABB_MOTION_PLANNER_HPP

#include <drake/systems/framework/leaf_system.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/common/trajectories/path_parameterized_trajectory.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

namespace simulation {
    class AbbMotionPlanner : public drake::systems::LeafSystem<double> {
    public:
        explicit AbbMotionPlanner(const std::string controller_ip);

        ~AbbMotionPlanner();

        const drake::systems::InputPort<double>& get_target_ee_location_input_port() const;

        const drake::systems::InputPort<double>& get_estimated_state_input_port() const;

        const drake::systems::InputPort<double>& get_desired_state_output_port() const;

    private:
        void calc_motion_output(const drake::systems::Context<double>& context, drake::systems::BasicVector<double>* vector) const;
        drake::systems::EventStatus initialize_discrete(const drake::systems::Context<double>& context, drake::systems::DiscreteValues<double>* value) const;
        drake::systems::EventStatus update_trajectory(const drake::systems::Context<double>& context, drake::systems::State<double>* state) const;
        drake::systems::EventStatus udp_send(const drake::systems::Context<double>& context) const;
        
        drake::multibody::MultibodyPlant<double> plant_;

        drake::systems::AbstractStateIndex traj_index_;

        drake::systems::DiscreteStateIndex max_speed_state_index_;
        drake::systems::DiscreteStateIndex old_goal_state_index_;
        drake::systems::DiscreteStateIndex old_time_state_index_;
        
        drake::systems::InputPortIndex target_ee_location_abstract_input_port_index_;
        drake::systems::InputPortIndex estimated_state_input_port_index_;
        drake::systems::OutputPortIndex desired_state_output_port_index_;

        struct sockaddr_in udp_server_addr_;
        sockaddr* udp_server_addr_ptr_;
        int udp_sock_fd_;

        const int num_plant_states_ = 6;
    };
}


#endif