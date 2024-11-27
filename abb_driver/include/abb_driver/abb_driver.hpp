#ifndef CONTROLLER__ABB_DRIVER_HPP
#define CONTROLLER__ABB_DRIVER_HPP

#include <drake/systems/framework/leaf_system.h>
#include <atomic>
#include <thread>

namespace controller {

class AbbDriver : public drake::systems::LeafSystem<double> {
public:
    explicit AbbDriver();
    ~AbbDriver();

private:
    void state_output_callback(const drake::systems::Context<double>& context, drake::systems::BasicVector<double>* output) const;
    void udp_read();
    inline void tcp_connect();
    drake::systems::EventStatus ee_publish(const drake::systems::Context<double>& context, drake::systems::State<double>* state) const;
    drake::systems::EventStatus get_time_offset(const drake::systems::Context<double>& context, drake::systems::State<double>* state) const;

    std::mutex state_mutex_;
    Eigen::Vector<double, 6> thread_safe_state_;
    double thread_safe_time_;
    std::thread udp_thread_;

    int tcp_sockfd_;
};

}

#endif