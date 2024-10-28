"""@package high_level_robot_controller

This module holds an interface between ROS2 and Drake specifically for the SVH.
TODO add support for the ABB
"""

from pydrake.systems.framework import DiagramBuilder, LeafSystem, Context, BasicVector, DiscreteValues, Diagram, TriggerType
from pydrake.common.value import AbstractValue
from math import floor

import drake_ros.core
from drake_ros.core import ClockSystem
from drake_ros.core import RosInterfaceSystem
from drake_ros.core import RosPublisherSystem
from drake_ros.core import RosSubscriberSystem

from rclpy.qos import QoSProfile

from control_msgs.msg import DynamicJointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class SvhDynamicJointStateDecomposer(LeafSystem):
    """Drake System that takes a ROS DynamicJointState message and parses it into a BasicVector.

    This class is only defined to work with the Schunk SVH.
    """

    def __init__(self, subscriber_sampling_period:float):
        super().__init__()
        self.joint_names = [
            'Left_Hand_Thumb_Opposition',
            'Left_Hand_Thumb_Flexion',
            'Left_Hand_Index_Finger_Proximal',
            'Left_Hand_Index_Finger_Distal',
            'Left_Hand_Middle_Finger_Proximal',
            'Left_Hand_Middle_Finger_Distal',
            'Left_Hand_Finger_Spread',
            'Left_Hand_Pinky',
            'Left_Hand_Ring_Finger',
        ]

        self.DeclareAbstractInputPort("dynamic_joint_ros", AbstractValue.Make(DynamicJointState()))
        self.state_idx = self.DeclareDiscreteState(18)
        self.effort_idx = self.DeclareDiscreteState(9)
        self.DeclareStateOutputPort("svh_state", self.state_idx)
        self.DeclareStateOutputPort("svh_effort", self.effort_idx)
        self.DeclarePeriodicDiscreteUpdateEvent(subscriber_sampling_period, 0.0, self.calc_state_output)
        self.DeclareInitializationDiscreteUpdateEvent(self.calc_state_output)

    def calc_state_output(self, context:Context, vector:DiscreteValues):
        #Callback function that parses a ROS2 input message and fills in an output vector
        input_message:DynamicJointState = self.get_input_port(0).Eval(context)
        for joint,interface_values in zip(input_message.joint_names, input_message.interface_values):
            joint_idx = self.joint_names.index(joint)
            vector.get_mutable_vector(0).SetAtIndex(joint_idx, interface_values.values[0])
            vector.get_mutable_vector(0).SetAtIndex(joint_idx+9, interface_values.values[1])
            vector.get_mutable_vector(1).SetAtIndex(joint_idx, interface_values.values[2])

class SvhJointTrajectoryBuilder(LeafSystem):
    """Drake System that takes in a BasicVector and outputs a JointTrajectory ROS2 message for controlling the SVH.

    This class is only defined for the Schunk SVH
    """

    def __init__(self):
        super().__init__()
        self.num_positions = 9
        self.DeclareVectorInputPort("svh_desired_trajectory", 18)
        self.DeclareAbstractOutputPort("joint_trajectory_ros", lambda : AbstractValue.Make(JointTrajectory()), self.calc_output)
        self.joint_names = [
            f'Left_Hand_Thumb_Opposition',
            f'Left_Hand_Thumb_Flexion',
            f'Left_Hand_Index_Finger_Proximal',
            f'Left_Hand_Index_Finger_Distal',
            f'Left_Hand_Middle_Finger_Proximal',
            f'Left_Hand_Middle_Finger_Distal',
            f'Left_Hand_Finger_Spread',
            f'Left_Hand_Pinky',
            f'Left_Hand_Ring_Finger',
        ]

    def calc_output(self, context:Context, value:AbstractValue):
        #Callback function that builds a ROS2 JointTrajectory message.

        traj_vector:BasicVector = self.get_input_port(0).Eval(context)
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj_pt = JointTrajectoryPoint()
        #TODO Update this to send coarser points to the svh controller
        traj_pt.time_from_start = Duration(sec=0)
        traj_pt.positions = list(traj_vector[:self.num_positions])
        traj_pt.velocities = list(traj_vector[self.num_positions:])
        traj.points.append(traj_pt)
        value.SetFrom(AbstractValue.Make(traj))

class RosInterface(Diagram):
    """Wrapper class that allows for quick interfacing with the Schunk SVH
    """

    def __init__(self, publishing_period:float, subscriber_sampling_period:float=0.02):
        super().__init__()
        builder = DiagramBuilder()

        drake_ros.core.init()
        interface = builder.AddSystem(RosInterfaceSystem("drake_interface"))

        qos = QoSProfile(depth=10)

        state_subscriber = builder.AddSystem(
            RosSubscriberSystem.Make(
                DynamicJointState, 
                "/dynamic_joint_states", 
                qos, 
                interface.get_ros_interface()
            )   
        )

        state_publisher = builder.AddSystem(
            RosPublisherSystem.Make(
                JointTrajectory,
                "/left_hand/joint_trajectory",
                qos,
                interface.get_ros_interface(),
                {TriggerType.kPeriodic},
                publish_period=publishing_period
            )
        )

        traj_msg_builder = builder.AddSystem(SvhJointTrajectoryBuilder())
        builder.Connect(traj_msg_builder.get_output_port(), state_publisher.get_input_port(0))
        builder.ExportInput(traj_msg_builder.get_input_port(0))

        state_sensor = builder.AddSystem(SvhDynamicJointStateDecomposer(subscriber_sampling_period))
        builder.Connect(state_subscriber.get_output_port(0), state_sensor.get_input_port(0))
        builder.ExportOutput(state_sensor.GetOutputPort("svh_state"))
        builder.ExportOutput(state_sensor.GetOutputPort("svh_effort"))

        builder.BuildInto(self)
