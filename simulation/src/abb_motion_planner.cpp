#include "simulation/abb_motion_planner.hpp"
#include "simulation/package_path.hpp"

#include <iostream>

#include <drake/systems/framework/leaf_system.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/common/value.h>
#include <drake/solvers/solve.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/common/trajectories/path_parameterized_trajectory.h>
#include <drake/common/trajectories/piecewise_polynomial.h>

namespace simulation {
    AbbMotionPlanner::AbbMotionPlanner() : 
        plant_(drake::multibody::MultibodyPlant<double>(0.0))
    {
        auto parser = drake::multibody::Parser(&plant_);
        parser.AddModels(simulation::package_path::get_package_share_path("simulation") + "abb/ABB_irb1200.urdf");
        this->plant_.WeldFrames(this->plant_.world_frame(), this->plant_.GetFrameByName("base"));
        this->plant_.Finalize();

        this->target_location_index_ = this->DeclareAbstractState(*drake::AbstractValue::Make(*this->plant_.CreateDefaultContext()));

        auto temp = drake::trajectories::PathParameterizedTrajectory<double>(
            drake::trajectories::PiecewisePolynomial<double>(Eigen::Vector<double, 6>::Zero()), 
            drake::trajectories::PiecewisePolynomial<double>(Eigen::Vector<double, 1>::Zero())
        );
        this->traj_index_ = this->DeclareAbstractState(*drake::AbstractValue::Make(temp));
        
        this->DeclareAbstractInputPort("target_ee_location", *drake::AbstractValue::Make(drake::math::RigidTransformd()));
        
        this->DeclareVectorInputPort("irb1200_estimated_state", this->num_plant_states_*2);
        this->DeclareVectorOutputPort("irb1200_desired_state", this->num_plant_states_*2, &AbbMotionPlanner::calc_motion_output);

        this->DeclareInitializationUnrestrictedUpdateEvent(&AbbMotionPlanner::update_trajectory);
        this->DeclarePeriodicUnrestrictedUpdateEvent(5.0, 0.0, &AbbMotionPlanner::update_trajectory);
    };

    void AbbMotionPlanner::calc_motion_output(const drake::systems::Context<double>& context, drake::systems::BasicVector<double>* vector) const {
        auto t = context.get_time();
        auto& traj = context.get_abstract_state<drake::trajectories::PathParameterizedTrajectory<double>>(1);
        vector->get_mutable_value()(Eigen::seqN(0, 6)) = traj.value(t);
        vector->get_mutable_value()(Eigen::seqN(6, 12)) = traj.EvalDerivative(t, 1);
    };

    drake::systems::EventStatus AbbMotionPlanner::update_trajectory(const drake::systems::Context<double>& context, drake::systems::State<double>* state) const {
        /*
        auto& mutable_context = state->get_mutable_abstract_state().get_mutable_value(0).get_mutable_value<drake::systems::Context<double>>();
        this->plant_.SetPositionsAndVelocities(&mutable_context, this->GetInputPort("irb1200_estimated_state").Eval(context));
        
        auto q0 = this->plant_.GetPositions(mutable_context);
        auto initial = this->plant_.GetBodyByName("gripper_frame").body_frame().CalcPoseInWorld(mutable_context);
        auto goal = this->GetInputPort("target_ee_location").Eval<drake::math::RigidTransformd>(context);

        double speed = 1.0;
        auto distance = (goal.translation() - initial.translation()).norm();
        double end_time = distance / speed;

        Eigen::Vector<double, 6> q1 = Eigen::Vector<double, 6>::Zero();
        {
            auto prog = drake::multibody::InverseKinematics(this->plant_);

            prog.AddPositionConstraint(
                this->plant_.GetBodyByName("gripper_frame").body_frame(),
                Eigen::Vector3d::Zero(),
                this->plant_.world_frame(),
                goal.translation(),
                goal.translation()
            );

            prog.AddOrientationConstraint(
                this->plant_.world_frame(),
                drake::math::RotationMatrixd::Identity(),
                this->plant_.GetBodyByName("gripper_frame").body_frame(),
                goal.rotation(),
                0.0
            );

            auto result = drake::solvers::Solve(prog.prog());
            q1 = this->plant_.GetPositions(prog.context());

            if (!result.is_success()) {
                std::cout << "Inverse kinematics failed. Results may not be accurate\n";
            }
        }

        Eigen::Matrix<double, 6, 2> M;
        M << q0, q1;

        auto linear_traj = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(Eigen::Vector2d{0.0, 1.0}, M);
        auto time_scaling = drake::trajectories::PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
            Eigen::Vector2d{0.0, end_time},
            Eigen::RowVector2d{0.0, 1.0},
            Eigen::Vector<double, 1>::Zero(),
            Eigen::Vector<double, 1>::Zero()
        );

        auto traj = state->get_abstract_state<drake::trajectories::PathParameterizedTrajectory<double>>(1);
        traj = drake::trajectories::PathParameterizedTrajectory<double>(linear_traj, time_scaling);
        */

        return drake::systems::EventStatus::Succeeded();
    };
}