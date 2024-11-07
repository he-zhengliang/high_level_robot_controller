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

        traj_state_idx = this->DeclareAbstractState(*drake::AbstractValue::Make(
            drake::trajectories::PiecewisePolynomial<double>(Eigen::Vector<double, 9>::Zero())
        ));

        this->DeclarePeriodicUnrestrictedUpdateEvent(0.01, 0.0, &SvhMotionPlanner::update_trajectory);
    }

    void SvhMotionPlanner::calc_output(const drake::systems::Context<double>& context, drake::systems::BasicVector<double>* output) const {
        auto& traj = context.get_abstract_state().get_value(traj_state_idx).get_value<drake::trajectories::PiecewisePolynomial<double>>();
        auto t = context.get_time();
        output->get_mutable_value()(Eigen::seqN(0, 9))  = traj.value(t);
        output->get_mutable_value()(Eigen::seqN(9, 9)) = traj.EvalDerivative(t, 1);
    }

    drake::systems::EventStatus SvhMotionPlanner::update_trajectory(const drake::systems::Context<double>& context, drake::systems::State<double>* state) const {
        const auto jt = this->get_input_port(0).Eval<trajectory_msgs::msg::JointTrajectory>(context);
        size_t num_names = jt.joint_names.size();
        if (num_names == 0) {
            return drake::systems::EventStatus::Succeeded();
        };

        const auto time = context.get_time();
        const auto current_state = this->get_input_port(1).Eval<Eigen::Vector<double, 18>>(context);

        /*
        
        if (traj.end_time() == INFINITY) {
            Eigen::Matrix<double, 9, 2> positions;
            positions << Eigen::Vector<double, 9>::Zero(), current_state(Eigen::seqN(0, 9));

            Eigen::Matrix<double, 9, 2> velocities;
            velocities << Eigen::Vector<double, 9>::Zero(), current_state(Eigen::seqN(9, 9));

            traj = drake::trajectories::PiecewisePolynomial<double>::CubicHermite(Eigen::Vector2d{0.0, time}, positions, velocities);
        }
        traj = drake::trajectories::PiecewisePolynomial<double>(current_state);
        */
        
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

        auto& traj = state->get_mutable_abstract_state<drake::trajectories::PiecewisePolynomial<double>>(0);

        for (size_t i = 0; i < names.size(); i++) {

            size_t index = 0;
            bool name_not_present = false;

            while (true) {
                if (jt.joint_names[index] == names[i]) {
                    break;
                } else if (index >= num_names) {
                    name_not_present = true;
                    break;
                }
            }

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

        traj = drake::trajectories::PiecewisePolynomial<double>::CubicHermite(breaks, positions, velocities);

        return drake::systems::EventStatus::Succeeded();
    }

    
}