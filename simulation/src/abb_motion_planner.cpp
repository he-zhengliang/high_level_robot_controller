#include "simulation/abb_motion_planner.hpp"
#include "simulation/package_path.hpp"

#include <iostream>
#include <atomic>
#include <chrono>
#include <thread>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

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

#include "egm.pb.h"

namespace simulation {

    AbbMotionPlanner::AbbMotionPlanner(const std::string controller_ip) : 
        server_ip_string_(controller_ip),
        plant_(drake::multibody::MultibodyPlant<double>(0.0))
    {
        {
            // Add a multibody plant with a ABB irb1200 for performing IK operations
            auto parser = drake::multibody::Parser(&plant_);
            parser.AddModels(simulation::package_path::get_package_share_path("simulation") + "abb/ABB_irb1200.urdf");
            this->plant_.WeldFrames(this->plant_.world_frame(), this->plant_.GetFrameByName("base"));
            this->plant_.Finalize();
            auto context = this->plant_.CreateDefaultContext();
            auto start_position = this->plant_.GetBodyByName("gripper_frame").body_frame().CalcPoseInWorld(*context);
            this->start_rotation_inv_index_ = this->DeclareAbstractState(*drake::AbstractValue::Make(drake::math::RigidTransformd(start_position.rotation())));
            this->plant_context_index_ = this->DeclareAbstractState(*drake::AbstractValue::Make(*context));
        }

        auto traj_to_copy_to_state = drake::trajectories::PathParameterizedTrajectory<double>(
            drake::trajectories::PiecewisePolynomial<double>(Eigen::Vector<double, 6>::Zero()), 
            drake::trajectories::PiecewisePolynomial<double>(Eigen::Vector<double, 1>::Zero())
        );

        this->traj_index_ = this->DeclareAbstractState(*drake::AbstractValue::Make(traj_to_copy_to_state));
        this->ee_pose_index_ = this->DeclareAbstractState(*drake::AbstractValue::Make(drake::math::RigidTransformd()));

        this->max_speed_state_index_ = this->DeclareDiscreteState(Eigen::Vector<double, 1>{0.3});
        this->old_time_state_index_ = this->DeclareDiscreteState(1);

        auto& irb1200_estimated_state = this->DeclareVectorInputPort("irb1200_estimated_state", this->num_plant_states_*2);
        this->estimated_state_input_port_index_ = irb1200_estimated_state.get_index();

        auto& irb1200_desired_state = this->DeclareVectorOutputPort("irb1200_desired_state", this->num_plant_states_*2, &AbbMotionPlanner::calc_motion_output);
        this->desired_state_output_port_index_ = irb1200_desired_state.get_index();
        
        udp_sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_sock_fd_ < 0) {
            perror("Failed to create a socket\n");
            return;
        }

        memset(&udp_server_addr_, 0, sizeof(udp_server_addr_));
        udp_server_addr_.sin_family = AF_INET;
        udp_server_addr_.sin_addr.s_addr = inet_addr(server_ip_string_.c_str());
        udp_server_addr_.sin_port = htons(3842);

        udp_server_addr_ptr_ = reinterpret_cast<sockaddr*>(&udp_server_addr_);  

        this->DeclarePeriodicPublishEvent(0.004, 0.0, &AbbMotionPlanner::udp_send);

        tcp_sock_fd_ = socket(AF_INET, SOCK_STREAM, 0);

        if (tcp_sock_fd_ < 0) {
            perror("Creating the TCP socket failed\n");
            return;
        }

        memset(&tcp_server_addr_, 0, sizeof(tcp_server_addr_));
        tcp_server_addr_.sin_family = AF_INET;
        tcp_server_addr_.sin_addr.s_addr = inet_addr(server_ip_string_.c_str());
        tcp_server_addr_.sin_port = 5554;

        while (true) {
            if (connect(tcp_sock_fd_, reinterpret_cast<sockaddr*>(&tcp_server_addr_), sizeof(tcp_server_addr_)) < 0) {
                if (errno == ECONNREFUSED) {
                    tcp_server_addr_.sin_port++;
                    continue;
                }
                perror("Connecting to the TCP server failed\n");
                return;
            }
            else {
                break;
            }
        }

        printf("Connected to port %u\n", tcp_server_addr_.sin_port);

        this->tcp_sock_open_index_ = this->DeclareAbstractState(*drake::AbstractValue::Make(true));

        this->DeclarePerStepUnrestrictedUpdateEvent(&AbbMotionPlanner::update_trajectory);
    };

    AbbMotionPlanner::~AbbMotionPlanner() {
        close(udp_sock_fd_);
        close(tcp_sock_fd_);
    }

    void AbbMotionPlanner::calc_motion_output(const drake::systems::Context<double>& context, drake::systems::BasicVector<double>* vector) const {
        double t = context.get_time() - context.get_discrete_state(this->old_time_state_index_).value()(0);
        auto& traj = context.get_abstract_state<drake::trajectories::PathParameterizedTrajectory<double>>(this->traj_index_);
        vector->get_mutable_value()(Eigen::seqN(0, this->num_plant_states_)) = traj.value(t);
        vector->get_mutable_value()(Eigen::seqN(this->num_plant_states_, this->num_plant_states_)) = traj.EvalDerivative(t, 1);
    };

    drake::systems::EventStatus AbbMotionPlanner::udp_send(const drake::systems::Context<double>& context) const {
        Eigen::VectorXd current_position = this->get_estimated_state_input_port().Eval(context);

        abb::egm::EgmRobot udp_msg;
        {
            auto header = udp_msg.mutable_header();
            header->set_mtype(abb::egm::EgmHeader_MessageType_MSGTYPE_DATA);
            header->set_seqno(0);
            header->set_tm(floor(context.get_time()*1000));
        }
        {
            auto feedback = udp_msg.mutable_feedback();
            {
                auto sense_time = feedback->mutable_time();
                double secs, usecs;
                usecs = modf(context.get_time(), &secs);
                sense_time->set_sec((uint64_t) secs);
                sense_time->set_usec(floor(usecs * 10e6));
            }
            {
                auto joints = feedback->mutable_joints();
                for (int i = 0; i < 6; i++) {
                    joints->add_joints(current_position(i));
                }
            }
        }

        auto msg_out = udp_msg.SerializeAsString();

        sendto(udp_sock_fd_, msg_out.c_str(), msg_out.size(), 0, udp_server_addr_ptr_, sizeof(udp_server_addr_));

        return drake::systems::EventStatus::Succeeded();
    }

    drake::systems::EventStatus AbbMotionPlanner::update_trajectory(const drake::systems::Context<double>& context, drake::systems::State<double>* state) const {
        if (!context.get_abstract_state<bool>(tcp_sock_open_index_)) {
            return drake::systems::EventStatus::Succeeded();
        }

        char msg_buffer[1024] = {0};
        ssize_t recv_len = recv(tcp_sock_fd_, msg_buffer, 1024, MSG_DONTWAIT);

        if (recv_len < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            return drake::systems::EventStatus::Succeeded();
        } else if (recv_len < 0) {
            perror("failed to recieve message");
        }

        drake::math::RigidTransformd goal;
        if (recv_len < 6 && strncmp("close", msg_buffer, recv_len) == 0) {
            state->get_mutable_abstract_state<bool>(tcp_sock_open_index_) = false;
            close(tcp_sock_fd_);
            return drake::systems::EventStatus::Succeeded();
        } else if (strncmp("movej", msg_buffer, 5) == 0) {
            if (recv_len == 33) {
                float* target_ee_pose = (float*)(msg_buffer + 5);
                goal.set_translation(Eigen::Vector3d(target_ee_pose[0], target_ee_pose[1], target_ee_pose[2]) / 1000);
                goal.set_rotation(Eigen::Quaterniond(target_ee_pose[3], target_ee_pose[4], target_ee_pose[5], target_ee_pose[6]));
                std::cout << "Goal: " << goal << "\n";
            } else {
                return drake::systems::EventStatus::Succeeded();
            }
        }

        goal = goal * context.get_abstract_state<drake::math::RigidTransformd>(this->start_rotation_inv_index_);

        auto& mutable_context = state->get_mutable_abstract_state<drake::systems::Context<double>>(this->plant_context_index_);
        this->plant_.SetPositionsAndVelocities(&mutable_context, this->GetInputPort("irb1200_estimated_state").Eval(context));

        auto q0 = this->plant_.GetPositions(mutable_context);
        auto initial = this->plant_.GetBodyByName("gripper_frame").body_frame().CalcPoseInWorld(mutable_context);

        auto distance = (goal.translation() - initial.translation()).norm();
        double end_time = distance / context.get_discrete_state(this->max_speed_state_index_).value()(0);

        auto ik_prog = drake::multibody::InverseKinematics(this->plant_);

        ik_prog.AddPositionConstraint( 
            this->plant_.GetBodyByName("gripper_frame").body_frame(),
            Eigen::Vector3d::Zero(),
            this->plant_.world_frame(),
            goal.translation(),
            goal.translation()
        );

        ik_prog.AddOrientationConstraint(
            this->plant_.world_frame(),
            drake::math::RotationMatrixd::Identity(),
            this->plant_.GetBodyByName("gripper_frame").body_frame(),
            goal.rotation(),
            0.0
        );

        auto ik_result = drake::solvers::Solve(ik_prog.prog());

        if (!ik_result.is_success()) {
            std::cout << "Inverse kinematics failed. Results may not be accurate\n";
            return drake::systems::EventStatus::Succeeded(); 
        }

        auto q1 = this->plant_.GetPositions(ik_prog.context());

        Eigen::Matrix<double, 6, 2> M;
        M << q0, q1;

        auto linear_traj = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(Eigen::Vector2d{0.0, 1.0}, M);
        auto time_scaling = drake::trajectories::PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
            Eigen::Vector2d{0.0, end_time},
            Eigen::RowVector2d{0.0, 1.0},
            Eigen::Vector<double, 1>::Zero(),
            Eigen::Vector<double, 1>::Zero()
        );

        state->get_mutable_abstract_state<drake::trajectories::PathParameterizedTrajectory<double>>(this->traj_index_)
            = drake::trajectories::PathParameterizedTrajectory<double>(linear_traj, time_scaling);
        state->get_mutable_discrete_state(this->old_time_state_index_).get_mutable_value() = Eigen::Vector<double, 1>{context.get_time()};
        return drake::systems::EventStatus::Succeeded();
    };

    const drake::systems::InputPort<double>& AbbMotionPlanner::get_estimated_state_input_port() const {
        return this->get_input_port(this->estimated_state_input_port_index_);
    }

    const drake::systems::InputPort<double>& AbbMotionPlanner::get_desired_state_output_port() const {
        return this->get_input_port(this->desired_state_output_port_index_);
    }

}