#!/usr/bin/env python3

from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, LeafSystem, Diagram, Context, State
from pydrake.trajectories import PiecewisePolynomial
from pydrake.systems.primitives import ConstantVectorSource, VectorLogSink, TrajectorySource
from RosInterface import RosInterface

import numpy as np
from matplotlib import pyplot as plt
from random import random

class BasicController(LeafSystem):
    def __init__(self, source:TrajectorySource):
        super().__init__()
        self.trajectory = PiecewisePolynomial()
        self.source:TrajectorySource = source
        self.DeclareVectorInputPort("current_state", 18)
        self.DeclarePeriodicPublishEvent(2.5, 0.0, self.update_traj)
        self.DeclareInitializationPublishEvent(self.update_traj)

    def update_traj(self, context:Context):
        current_state = self.get_input_port(0).Eval(context)
        time = context.get_time()
        breaks = [time, time+3.0]

        samples = [[0.0, 0.0]]*9
        samples[6] = [current_state[6], 0.3+0.2*random()]
        traj = PiecewisePolynomial.FirstOrderHold(breaks, samples)
        self.source.UpdateTrajectory(traj)
        
def main():
    builder = DiagramBuilder()

    ros_interface = builder.AddSystem(RosInterface())
    state_in = builder.AddSystem(TrajectorySource(PiecewisePolynomial.FirstOrderHold([0, 0.001], [[0.0, 0.0]]*9), 1, True))#builder.AddSystem(ConstantVectorSource(np.zeros(18)))
    controller = builder.AddSystem(BasicController(state_in))

    state_out:VectorLogSink = builder.AddSystem(VectorLogSink(18))
    state_in_logger:VectorLogSink = builder.AddSystem(VectorLogSink(18))
    effort_out:VectorLogSink = builder.AddSystem(VectorLogSink(9))
    
    builder.Connect(state_in.get_output_port(0), ros_interface.get_input_port(0))
    builder.Connect(ros_interface.get_output_port(0), state_out.get_input_port(0))
    builder.Connect(ros_interface.get_output_port(1), effort_out.get_input_port(0))
    builder.Connect(state_in.get_output_port(), state_in_logger.get_input_port())
    builder.Connect(ros_interface.get_output_port(0), controller.get_input_port())

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator_context = simulator.get_mutable_context()
    simulator.set_target_realtime_rate(1.0)

    step = 0.1
    while simulator_context.get_time() < 10:
        simulator.AdvanceTo(simulator_context.get_time() + step)

    log = state_in_logger.GetLog(state_in_logger.GetMyContextFromRoot(simulator_context))

    plt.figure(1)
    for i in range(log.data().shape[0]):
        plt.plot(log.sample_times(), log.data()[i, :])
    plt.savefig("plot.png")

    plt.figure(2)
    sensor_log = state_out.GetLog(state_out.GetMyContextFromRoot(simulator_context))
    for i in range(sensor_log.data().shape[0]):
        plt.plot(sensor_log.sample_times(), sensor_log.data()[i,:])
    plt.savefig("sensor_plot.png")
if __name__ == '__main__':
    main()
