#include "simulation/abb_motion_planner.hpp"
#include "simulation/package_path.hpp"

#include <iostream>

#include <drake/systems/framework/leaf_system.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/common/value.h>
#include <drake/common/trajectories/path_parameterized_trajectory.h>
#include <drake/common/trajectories/piecewise_polynomial.h>

#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/multibody/inverse_kinematics/position_constraint.h>
#include <drake/multibody/inverse_kinematics/orientation_constraint.h>
#include <drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h>
#include <drake/solvers/solve.h>

#include <drake_ros/core/geometry_conversions.h>
#include <geometry_msgs/msg/pose.hpp>

namespace simulation {
    AbbMotionPlanner::AbbMotionPlanner(const double speed) : 
        plant_(drake::multibody::MultibodyPlant<double>(0.0)), speed_(speed)
    {
        auto parser = drake::multibody::Parser(&plant_);
        parser.AddModels(simulation::package_path::get_package_share_path("simulation") + "abb/ABB_irb1200.urdf");
        this->plant_.WeldFrames(this->plant_.world_frame(), this->plant_.GetFrameByName("base"));
        this->plant_.Finalize();
        plant_context_ = this->plant_.CreateDefaultContext();

        this->target_location_index_ = this->DeclareAbstractState(*drake::AbstractValue::Make(*plant_context_));

        /*
        auto temp = drake::trajectories::PathParameterizedTrajectory<double>(
            drake::trajectories::PiecewisePolynomial<double>(Eigen::Vector<double, 6>::Zero()), 
            drake::trajectories::PiecewisePolynomial<double>(Eigen::Vector<double, 1>::Zero())
        );*/

        auto basis = drake::math::BsplineBasis<double>(4, std::vector{0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0});

        auto temp = drake::trajectories::BsplineTrajectory<double>(basis, {
            Eigen::Vector<double, 6>::Zero(),
            Eigen::Vector<double, 6>::Zero(),
            Eigen::Vector<double, 6>::Zero(),
            Eigen::Vector<double, 6>::Zero()
        });

        this->traj_index_ = this->DeclareAbstractState(*drake::AbstractValue::Make(temp));
        this->DeclareDiscreteState(7);

        this->DeclareAbstractInputPort("target_ee_location", *drake::AbstractValue::Make(geometry_msgs::msg::Pose()));
        
        this->DeclareVectorInputPort("irb1200_estimated_state", this->num_plant_states_*2);
        this->DeclareVectorOutputPort("irb1200_desired_state", this->num_plant_states_*2, &AbbMotionPlanner::calc_motion_output);

        this->DeclareInitializationDiscreteUpdateEvent(&AbbMotionPlanner::initialize_discrete);

        this->DeclarePeriodicUnrestrictedUpdateEvent(0.01, 0.0, &AbbMotionPlanner::update_trajectory);
    };

    void AbbMotionPlanner::calc_motion_output(const drake::systems::Context<double>& context, drake::systems::BasicVector<double>* vector) const {
        auto t = context.get_time();
        auto& traj = context.get_abstract_state<drake::trajectories::BsplineTrajectory<double>>(this->traj_index_);
        vector->get_mutable_value()(Eigen::seqN(0, this->num_plant_states_)) = traj.value(t);
        vector->get_mutable_value()(Eigen::seqN(this->num_plant_states_, this->num_plant_states_)) = traj.EvalDerivative(t, 1);
    };

    drake::systems::EventStatus AbbMotionPlanner::initialize_discrete(const drake::systems::Context<double>& context, drake::systems::DiscreteValues<double>* value) const {
        value->get_mutable_value(0) = Eigen::Vector<double, 7>{0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
        return drake::systems::EventStatus::Succeeded();
    }

    drake::systems::EventStatus AbbMotionPlanner::update_trajectory(const drake::systems::Context<double>& context, drake::systems::State<double>* state) const {
        auto goal_ros2 = this->GetInputPort("target_ee_location").Eval<geometry_msgs::msg::Pose>(context);
        auto old_goal = state->get_mutable_discrete_state().get_mutable_value();
        auto new_goal = Eigen::Vector<double, 7>{goal_ros2.position.x, goal_ros2.position.y, goal_ros2.position.z, goal_ros2.orientation.w, goal_ros2.orientation.x, goal_ros2.orientation.y, goal_ros2.orientation.z};
        if (old_goal == new_goal) {
            return drake::systems::EventStatus::Succeeded();
        }
        else {
            old_goal = new_goal;
            std::cout << "Recieved new end effector command\n";
        }

        auto goal = drake_ros::core::RosPoseToRigidTransform(goal_ros2);
        
        auto& mutable_abstract_state = state->get_mutable_abstract_state();
        auto& mutable_context = mutable_abstract_state.get_mutable_value(0).get_mutable_value<drake::systems::Context<double>>();
        this->plant_.SetPositionsAndVelocities(&mutable_context, this->GetInputPort("irb1200_estimated_state").Eval(context));
        
        auto q0 = this->plant_.GetPositions(mutable_context);
        auto initial = this->plant_.GetBodyByName("gripper_frame").body_frame().CalcPoseInWorld(mutable_context);

        double speed = 0.5;
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

        const int num_knots = 5;
        auto traj_opt = drake::planning::trajectory_optimization::KinematicTrajectoryOptimization(this->plant_.num_positions(), num_knots);
        auto& prog = traj_opt.get_mutable_prog();
        traj_opt.AddDurationCost(1.0);
        traj_opt.AddPathLengthCost(1.0);
        traj_opt.AddPositionBounds(this->plant_.GetPositionLowerLimits(), this->plant_.GetPositionUpperLimits());
        traj_opt.AddVelocityBounds(this->plant_.GetVelocityLowerLimits(), this->plant_.GetVelocityUpperLimits());
        traj_opt.AddDurationConstraint(0.5, 50.0);

        auto start_constraint = std::make_shared<drake::multibody::PositionConstraint> (
            &this->plant_,
            this->plant_.world_frame(),
            initial.translation(),
            initial.translation(),
            this->plant_.GetFrameByName("gripper_frame"),
            Eigen::Vector3d{0.0, 0.0, 0.0},
            &mutable_context
        );

        traj_opt.AddPathPositionConstraint(start_constraint, 0.0);

        auto goal_constraint = std::make_shared<drake::multibody::PositionConstraint> (
            &this->plant_,
            this->plant_.world_frame(),
            goal.translation(),
            goal.translation(),
            this->plant_.GetFrameByName("gripper_frame"),
            Eigen::Vector3d{0.0, 0.0, 0.0},
            &mutable_context
        );

        traj_opt.AddPathPositionConstraint(goal_constraint, 1.0);

        auto start_orientation = std::make_shared<drake::multibody::OrientationConstraint> (
            &this->plant_,
            this->plant_.world_frame(),
            initial.rotation(),
            this->plant_.GetFrameByName("gripper_frame"),
            drake::math::RotationMatrixd::Identity(),
            0.0,
            &mutable_context
        );

        traj_opt.AddPathPositionConstraint(start_orientation, 0.0);

        auto goal_orientation = std::make_shared<drake::multibody::OrientationConstraint> (
            &this->plant_,
            this->plant_.world_frame(),
            goal.rotation(),
            this->plant_.GetFrameByName("gripper_frame"),
            drake::math::RotationMatrixd::Identity(),
            0.0, 
            &mutable_context
        );

        traj_opt.AddPathPositionConstraint(goal_orientation, 1.0);

        auto num_q = this->plant_.num_positions();
        assert(num_q == 6);

        // Maybe don't add cost
        prog.AddQuadraticErrorCost(Eigen::Matrix<double, 6, 6>::Identity(), q0, traj_opt.control_points()(Eigen::all, 5));

        traj_opt.AddPathVelocityConstraint(Eigen::Vector<double, 6>::Zero(), Eigen::Vector<double, 6>::Zero(), 0);
        traj_opt.AddPathVelocityConstraint(Eigen::Vector<double, 6>::Zero(), Eigen::Vector<double, 6>::Zero(), 1);

        std::vector<Eigen::MatrixXd> break_point_vector;
        break_point_vector.resize(num_knots);
        for (int x = 0; x < num_knots; x++) {
            break_point_vector[x].resize(6, 1);
            break_point_vector[x] = (static_cast<double>((num_knots - 1 - x)) / static_cast<double>((num_knots - 1)))*q0 + (static_cast<double>(x) / static_cast<double>((num_knots - 1))) * q1;
        }

        auto basis = drake::math::BsplineBasis<double>(4, num_knots);
        auto initial_guess = drake::trajectories::BsplineTrajectory<double>(basis, break_point_vector);

        traj_opt.SetInitialGuess(initial_guess);
        auto result = drake::solvers::Solve(prog);

        if (! result.is_success()) {            
            std::cout << "Trajopt Failed!\n";
        }

        mutable_abstract_state.get_mutable_value(1).get_mutable_value<drake::trajectories::BsplineTrajectory<double>>()
            = traj_opt.ReconstructTrajectory(result);

        return drake::systems::EventStatus::Succeeded();

        /*
        Eigen::Matrix<double, 6, 2> M;
        M << q0, q1;

        auto linear_traj = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(Eigen::Vector2d{0.0, 1.0}, M);
        auto time_scaling = drake::trajectories::PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
            Eigen::Vector2d{0.0, end_time},
            Eigen::RowVector2d{0.0, 1.0},
            Eigen::Vector<double, 1>::Zero(),
            Eigen::Vector<double, 1>::Zero()
        );

        mutable_abstract_state.get_mutable_value(1).get_mutable_value<drake::trajectories::PathParameterizedTrajectory<double>>() 
            = drake::trajectories::PathParameterizedTrajectory<double>(linear_traj, time_scaling);

        return drake::systems::EventStatus::Succeeded();
        */
    };
}