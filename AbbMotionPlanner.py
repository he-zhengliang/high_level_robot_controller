from pydrake.systems.framework import LeafSystem, BasicVector, Context, State
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.trajectories import BsplineTrajectory
from pydrake.common.value import AbstractValue # type: ignore
from pydrake.math import RigidTransform, RotationMatrix, BsplineBasis
from pydrake.multibody.inverse_kinematics import InverseKinematics, PositionConstraint, OrientationConstraint
from pydrake.planning import KinematicTrajectoryOptimization
from pydrake.solvers import Solve

import numpy as np

class AbbMotionPlanner(LeafSystem):
    def __init__(self, num_knots_in_traj_opt=10):
        super().__init__()

        # Internal representation of the IRB1200 to allow for optimization
        self.plant_ = MultibodyPlant(0.0)
        parser = Parser(self.plant_)
        parser.AddModels("abb/ABB_irb1200.urdf")
        self.initial_ = None
        self.goal_ = None
        self.q1_ = None
        self.plant_.WeldFrames(self.plant_.world_frame(), self.plant_.GetFrameByName("base"))
        self.plant_.Finalize()
        self.traj = BsplineTrajectory()
        self.num_knots = num_knots_in_traj_opt
        self.DeclareAbstractState(AbstractValue.Make(self.plant_.CreateDefaultContext()))

        self.num_plant_states = self.plant_.num_multibody_states()

        self.DeclareAbstractInputPort("target_ee_location", AbstractValue.Make(RigidTransform()))
        self.DeclareVectorInputPort("irb1200_estimated_state", BasicVector(np.zeros(self.num_plant_states)))
        self.DeclareVectorOutputPort("irb1200_desired_state", BasicVector(np.zeros(self.num_plant_states)), self.CalcMotionOutput)
        self.DeclareInitializationUnrestrictedUpdateEvent(self.UpdateTrajectory)
        
    def CalcMotionOutput(self, context:Context, output:BasicVector):
        t = context.get_time()
        value = np.concatenate((self.traj.value(t), self.traj.EvalDerivative(t, 1)), axis=0)
        output.SetFrom(BasicVector(value))

    def UpdateTrajectory(self, context:Context, state:State):
        mutable_context = state.get_abstract_state().get_mutable_value(0).get_mutable_value()
        self.plant_.SetPositionsAndVelocities(mutable_context, self.GetInputPort("irb1200_estimated_state").EvalBasicVector(context).value())

        q0 = self.plant_.GetPositions(mutable_context)
        initial = self.plant_.GetBodyByName("gripper_frame").body_frame().CalcPoseInWorld(mutable_context)
        goal:RigidTransform = self.GetInputPort("target_ee_location").EvalAbstract(context).get_value()

        self.initial_ = initial
        self.goal_ = goal

        def get_final_joint_angles():
            prog_ = InverseKinematics(self.plant_)
            prog_.AddPositionConstraint(
                self.plant_.GetBodyByName("gripper_frame").body_frame(),
                np.zeros((3,1)),
                self.plant_.world_frame(),
                goal.translation(),
                goal.translation()
            )
            prog_.AddOrientationConstraint(
                self.plant_.world_frame(),
                RotationMatrix().Identity(),
                self.plant_.GetBodyByName("gripper_frame").body_frame(),
                goal.rotation(),
                0.0
            )
            
            result = Solve(prog_.prog())
            if result.is_success():
                temp_context = prog_.context()
                return self.plant_.GetPositions(temp_context)
            else:
                print("IK failed")
                return self.plant_.GetPositions(mutable_context)
            
        self.q1_ = get_final_joint_angles()

        trajopt = KinematicTrajectoryOptimization(self.plant_.num_positions(), self.num_knots)

        prog = trajopt.get_mutable_prog()
        trajopt.AddDurationCost(1.0)
        trajopt.AddPathLengthCost(1.0)
        trajopt.AddPositionBounds(self.plant_.GetPositionLowerLimits(), self.plant_.GetPositionUpperLimits())
        trajopt.AddVelocityBounds(self.plant_.GetVelocityLowerLimits(), self.plant_.GetVelocityUpperLimits())
        trajopt.AddDurationConstraint(5.0, 50.0)
        
        start_constraint = PositionConstraint(
            self.plant_,
            self.plant_.world_frame(),
            initial.translation(),
            initial.translation(),
            self.plant_.GetFrameByName("gripper_frame"),
            [0.0, 0.0, 0.0],
            mutable_context
        )
        trajopt.AddPathPositionConstraint(start_constraint, 0.0)

        goal_constraint = PositionConstraint(
            self.plant_,
            self.plant_.world_frame(),
            goal.translation(),
            goal.translation(),
            self.plant_.GetFrameByName("gripper_frame"),
            [0.0, 0.0, 0.0],
            mutable_context
        )
        trajopt.AddPathPositionConstraint(goal_constraint, 1.0)  

        initial_orientation_constraint = OrientationConstraint(
            self.plant_,
            self.plant_.world_frame(),
            initial.rotation(),
            self.plant_.GetFrameByName("gripper_frame"),
            RotationMatrix().Identity(),
            0.0,
            mutable_context
        )
        trajopt.AddPathPositionConstraint(initial_orientation_constraint, 0.0)

        goal_orientation_constraint = OrientationConstraint(
            self.plant_,
            self.plant_.world_frame(),
            goal.rotation(),
            self.plant_.GetFrameByName("gripper_frame"),
            RotationMatrix().Identity(),
            0.0,
            mutable_context
        )
        trajopt.AddPathPositionConstraint(goal_orientation_constraint, 1.0)
        
        num_q = self.plant_.num_positions()
        prog.AddQuadraticErrorCost(np.eye(num_q), q0, trajopt.control_points()[:,-1])
        trajopt.AddPathVelocityConstraint(np.zeros((num_q, 1)), np.zeros((num_q, 1)), 0)
        trajopt.AddPathVelocityConstraint(np.zeros((num_q, 1)), np.zeros((num_q, 1)), 1)

        # trajopt.AddPathAccelerationConstraint(np.zeros((num_q, 1)), np.zeros((num_q, 1)), 0)
        # trajopt.AddPathAccelerationConstraint(np.zeros((num_q, 1)), np.zeros((num_q, 1)), 1)

        prog.AddQuadraticErrorCost(np.eye(num_q), q0, trajopt.control_points()[:, 0])
        
        break_points = [np.reshape(((self.num_knots-1-x)/(self.num_knots-1))*q0 + (x/(self.num_knots-1))*self.q1_, (6,1)) for x in range(self.num_knots)]
        basis = BsplineBasis(4, self.num_knots)
        initial_guess = BsplineTrajectory(basis, break_points)
        trajopt.SetInitialGuess(initial_guess)
        result = Solve(prog)
        if not result.is_success():
            print("optimization failed")
        else:
            print("optimization succeeded")
        self.traj = trajopt.ReconstructTrajectory(result)
