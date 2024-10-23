from pydrake.systems.framework import LeafSystem, DiagramBuilder, InputPort, OutputPort
from pydrake.systems.primitives import Multiplexer, Demultiplexer
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser

def AddSvhToPlant(plant:MultibodyPlant, parser:Parser, builder:DiagramBuilder, actuation_required=True):
    if actuation_required:
        parser.AddModels("install/high_level_robot_controller/share/high_level_robot_controller/svh/Schunk_SVH")
    else:
        parser.AddModels("install/high_level_robot_controller/share/high_level_robot_controller/svh/Schunk_SVH_no_collision.urdf")
    svh = plant.GetModelInstanceByName("svh")
    state_input:InputPort = plant.get_desired_state_input_port(svh)
    state_output:OutputPort = plant.get_state_output_port()




        
