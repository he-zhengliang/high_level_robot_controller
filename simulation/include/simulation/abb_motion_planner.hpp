#ifndef SIMULATION__ABB_MOTION_PLANNER_HPP
#define SIMULATION__ABB_MOTION_PLANNER_HPP

#include <drake/systems/framework/leaf_system.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/common/trajectories/path_parameterized_trajectory.h>

namespace simulation {
    class AbbMotionPlanner : public drake::systems::LeafSystem<double> {
    public:
        explicit AbbMotionPlanner();
    private:
        void calc_motion_output(const drake::systems::Context<double>& context, drake::systems::BasicVector<double>* vector) const;
        drake::systems::EventStatus update_trajectory(const drake::systems::Context<double>& context, drake::systems::State<double>* state) const;

        drake::multibody::MultibodyPlant<double> plant_;

        drake::systems::AbstractStateIndex traj_index_;
        drake::systems::AbstractStateIndex target_location_index_;
        const int num_plant_states_ = 6;
    };
}


#endif