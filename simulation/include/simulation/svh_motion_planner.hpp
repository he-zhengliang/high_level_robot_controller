#ifndef SIMULATION__SVH_MOTION_PLANNER_HPP
#define SIMULATION__SVH_MOTION_PLANNER_HPP

#include <drake/systems/framework/leaf_system.h>

namespace simulation {
class SvhMotionPlanner : public drake::systems::LeafSystem<double> {
public:
    explicit SvhMotionPlanner();

private:
    void calc_output(const drake::systems::Context<double>& context, drake::systems::BasicVector<double>* output) const;

    drake::systems::EventStatus update_trajectory(const drake::systems::Context<double>& context, drake::systems::State<double>* state) const;

    drake::systems::AbstractStateIndex sec_state_idx;
    drake::systems::AbstractStateIndex nano_state_idx;
    drake::systems::AbstractStateIndex traj_state_idx;
};
};

#endif