#include "simulation/svh_motion_planner.hpp"

#include <drake/common/trajectories/piecewise_polynomial.h>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <iostream>
#include <vector>
#include <unordered_map>

namespace simulation {

    namespace {
        const std::array<std::string, 9> names =  {
            "Left_Hand_Thumb_Opposition",
            "Left_Hand_Thumb_Flexion",
            "Left_Hand_Index_Finger_Proximal",
            "Left_Hand_Index_Finger_Distal",
            "Left_Hand_Middle_Finger_Proximal",
            "Left_Hand_Middle_Finger_Distal",
            "Left_Hand_Finger_Spread",
            "Left_Hand_Pinky",
            "Left_Hand_Ring_Finger"
        };
    }

    SvhMotionPlanner::SvhMotionPlanner() {
        this->DeclareAbstractInputPort("joint_trajectory", *drake::AbstractValue::Make(trajectory_msgs::msg::JointTrajectory()));
        this->DeclareVectorInputPort("current_joint_state", 18);
        this->DeclareVectorOutputPort("desired_joint_state", 18, &SvhMotionPlanner::calc_output);

        sec_state_idx = this->DeclareAbstractState(*drake::AbstractValue::Make((int32_t)(-1)));
        nano_state_idx = this->DeclareAbstractState(*drake::AbstractValue::Make((uint32_t)(0)));

        traj_state_idx = this->DeclareAbstractState(*drake::AbstractValue::Make(
            drake::trajectories::PiecewisePolynomial<double>(Eigen::Vector<double, 9>::Zero())
        ));

        this->DeclarePeriodicUnrestrictedUpdateEvent(0.01, 0.0, &SvhMotionPlanner::update_trajectory);
    }

    void SvhMotionPlanner::calc_output(const drake::systems::Context<double>& context, drake::systems::BasicVector<double>* output) const {
        auto& traj = context.get_abstract_state().get_value(traj_state_idx).get_value<drake::trajectories::PiecewisePolynomial<double>>();
        auto t = context.get_time();// - context.get_discrete_state().value()(0);
        output->get_mutable_value()(Eigen::seqN(0, 9))  = traj.value(t);
        output->get_mutable_value()(Eigen::seqN(9, 9)) = traj.EvalDerivative(t, 1);
    }

    drake::systems::EventStatus SvhMotionPlanner::update_trajectory(const drake::systems::Context<double>& context, drake::systems::State<double>* state) const {
        const auto jt = this->get_input_port(0).Eval<trajectory_msgs::msg::JointTrajectory>(context);
        size_t num_names = jt.joint_names.size();
        auto last_time_sec = context.get_abstract_state<int32_t>(sec_state_idx);
        auto last_time_nano = context.get_abstract_state<uint32_t>(nano_state_idx);
        if ((last_time_sec == jt.header.stamp.sec && last_time_nano == jt.header.stamp.nanosec) || num_names == 0) {
            return drake::systems::EventStatus::Succeeded();
        };
        state->get_mutable_abstract_state<int32_t>(sec_state_idx) = jt.header.stamp.sec;
        state->get_mutable_abstract_state<uint32_t>(nano_state_idx) = jt.header.stamp.nanosec;
        
        const auto time = context.get_time();
        const auto current_state = this->get_input_port(1).Eval(context);

        const auto num_breaks = jt.points.size() + 1;
        Eigen::VectorXd breaks;
        {
            breaks.resize(num_breaks);
            breaks(0) = time;
            size_t counter = 1;
            for (auto pt : jt.points) {
                breaks(counter) = time + static_cast<double>(pt.time_from_start.sec) + static_cast<double>(pt.time_from_start.nanosec) * 1e-9;
                counter++;
            }
        }

        Eigen::Matrix<double, 9, -1> positions;
        Eigen::Matrix<double, 9, -1> velocities;

        positions.resize(Eigen::NoChange, num_breaks);
        velocities.resize(Eigen::NoChange, num_breaks);

        positions(Eigen::all, 0) = Eigen::Vector<double, 9>::Zero();
        velocities(Eigen::all, 0) = Eigen::Vector<double, 9>::Zero();

        auto traj = context.get_abstract_state<drake::trajectories::PiecewisePolynomial<double>>(traj_state_idx);

        for (size_t i = 0; i < names.size(); i++) {
            std::cout << names[i] << "\n";

            size_t index = 0;
            bool name_not_present = false;

            while (true) {
                if (jt.joint_names[index] == names[i]) {
                    break;
                } else if (index >= num_names) {
                    name_not_present = true;
                }
                index++;
            }

            std::cout << "Index: " << index << "\n";
            std::cout << "name_not_present: " << name_not_present << "\n";

            if (!name_not_present) {
                for (size_t b = 1; b < num_breaks; b++) {
                    positions(i, b) = jt.points[b-1].positions[index];
                    velocities(i, b) = jt.points[b-1].velocities[index];
                }
            } else {
                for (size_t b = 1; b < num_breaks; b++) {
                    positions(i, b) = traj.value(breaks(b))(i);
                    velocities(i, b) = traj.value(breaks(b))(i+9);
                }
            }
        }

        std::cout << positions << "\n\n" << velocities << "\n\n";

        state->get_mutable_abstract_state<drake::trajectories::PiecewisePolynomial<double>>(traj_state_idx)
            = drake::trajectories::PiecewisePolynomial<double>::CubicHermite(breaks, positions, velocities);

        return drake::systems::EventStatus::Succeeded();
    }

    
}