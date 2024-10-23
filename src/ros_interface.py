#!/usr/bin/env python3
import sys, os
import numpy as np

import drake_ros.core
from drake_ros.core import RosInterfaceSystem
from drake_ros.core import RosPublisherSystem
from drake_ros.core import RosSubscriberSystem

from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, LeafSystem, Context
from pydrake.geometry import StartMeshcat, GeometrySet, CollisionFilterDeclaration
from pydrake.multibody.plant import MultibodyPlant, AddMultibodyPlantSceneGraph, DiscreteContactApproximation
from pydrake.multibody.parsing import Parser
from pydrake.visualization import AddDefaultVisualization

from GripperMotionController import GripperMotionController

from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import DynamicJointState

from rclpy.qos import QoSProfile

def main():
    meshcat = StartMeshcat()
    builder = DiagramBuilder()
    plant:MultibodyPlant
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.001)


    plant.set_discrete_contact_approximation(DiscreteContactApproximation.kSap)
    parser = Parser(plant)
    print(os.getcwd())
    parser.AddModels("install/high_level_robot_controller/share/high_level_robot_controller/svh/Schunk_SVH.urdf")
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"))

    svh_collision_set = GeometrySet()
    for bi in plant.GetBodyIndices(plant.GetModelInstanceByName("svh")):
        svh_collision_set.Add(plant.GetCollisionGeometriesForBody(plant.get_body(bi)))
    scene_graph.collision_filter_manager().Apply(CollisionFilterDeclaration().ExcludeWithin(svh_collision_set))

    plant.Finalize()

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

    print("Number of actuators %i", plant.get_net_actuation_output_port(plant.GetModelInstanceByName("svh")).size())

    AddDefaultVisualization(builder, meshcat)

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator_context = simulator.get_mutable_context()
    ds = np.zeros(18)
    ds[2] = 1.0
    plant.get_desired_state_input_port(plant.GetModelInstanceByName("svh")).FixValue(plant.GetMyContextFromRoot(simulator_context), ds)
    simulator.set_target_realtime_rate(1.0)

    step = 1.0
    while simulator_context.get_time() < 10000.0:
        next_time = min(
            simulator_context.get_time() + step, 10000.0,
        )

        print("Time: %f", simulator_context.get_time())
        print(state_subscriber.get_output_port(0).Eval(state_subscriber.GetMyContextFromRoot(simulator_context)))
        print(plant.GetPositionNames(plant.GetModelInstanceByName("svh")))
        print(plant.GetPositions(plant.GetMyContextFromRoot(simulator_context), plant.GetModelInstanceByName("svh")))
        print("\n")
        
        simulator.AdvanceTo(next_time)

if __name__ == '__main__':
    main()