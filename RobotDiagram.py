import numpy as np
from pydrake.multibody.plant import MultibodyPlant, DiscreteContactApproximation, AddMultibodyPlantSceneGraph, ContactModel
from pydrake.systems.sensors import RgbdSensor, CameraInfo
from pydrake.geometry import (
    CollisionFilterDeclaration, 
    GeometrySet, 
    Meshcat, 
    SceneGraph, 
    MakeRenderEngineGl, 
    RenderCameraCore, 
    ColorRenderCamera, 
    DepthRenderCamera, 
    RenderEngineGlParams, 
    ClippingRange, 
    DepthRange)
from pydrake.visualization import AddDefaultVisualization
from pydrake.math import RotationMatrix, RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.systems.controllers import InverseDynamicsController
from pydrake.systems.framework import Diagram, DiagramBuilder, System
from pydrake.perception import DepthImageToPointCloud


def get_svh_geometry_filter(plant:MultibodyPlant) -> CollisionFilterDeclaration:
    svh_collision_set = GeometrySet()
    for bi in plant.GetBodyIndices(plant.GetModelInstanceByName("svh")):
        svh_collision_set.Add(plant.GetCollisionGeometriesForBody(plant.get_body(bi)))
    return CollisionFilterDeclaration().ExcludeWithin(svh_collision_set)

def abb_inverse_dynamics_controller() -> InverseDynamicsController:
    plant = MultibodyPlant(0.001)
    parser = Parser(plant)
    parser.AddModels("abb/ABB_irb1200.urdf")
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"))
    other_plant = MultibodyPlant(0.001)
    other_plant.set_discrete_contact_approximation(DiscreteContactApproximation.kSap)
    other_parser = Parser(other_plant)
    other_parser.AddModels("svh/Schunk_SVH_no_collision.urdf")
    other_plant.Finalize()
    other_context = other_plant.CreateDefaultContext()
    svh_ = other_plant.GetModelInstanceByName("svh")
    svh_I = other_plant.CalcSpatialInertia(other_context, other_plant.world_frame(), other_plant.GetBodyIndices(svh_))
    plant.Finalize()
    context = plant.CreateDefaultContext()
    gripper = plant.GetBodyByName("gripper_frame")
    gripper.SetSpatialInertiaInBodyFrame(context, svh_I)

    Kp = np.array([10.0, 10.0, 10.0, 10.0, 20.0, 20.0])*5
    Ki = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    Kd = np.array([50.0, 50.0, 50.0, 50.0, 100.0, 100.0])*5

    return InverseDynamicsController(plant, Kp, Ki, Kd, False, context)

def get_camera_pose(camera_position:np.ndarray, focus_point:np.ndarray=np.zeros(3)):
    focus_translation = camera_position - focus_point
    base_length = np.sqrt(focus_translation[0]**2 + focus_translation[1]**2)
    if base_length < 0.0001:
        if focus_translation[2] > 0:
            x_rot_angle = np.pi / 2     
        else:
            x_rot_angle = -np.pi / 2
    else:
        x_rot_angle = np.arctan(focus_translation[2] / base_length)
        print("standard_case_x")
    if abs(focus_translation[0]) < 0.0001:
        if focus_translation[1] > 0:
            z_rotation = np.pi/2
        else:
            z_rotation = -np.pi/2
    else:
        z_rotation = np.arctan(focus_translation[1]/focus_translation[0])
        print("standard_case_z")

    camera_rot = RotationMatrix.MakeZRotation(np.pi/2+z_rotation) @ RotationMatrix.MakeXRotation(-np.pi/2 - x_rot_angle)
    return RigidTransform(camera_rot, camera_position)

class RobotDiagram(Diagram):
    def __init__(self, sim_time_step:float, meshcat:Meshcat=None, hydroelastic_contact:bool=False, models_to_add:list[str]=[]):
        super().__init__()
        builder = DiagramBuilder()
        if sim_time_step <= 0:
            raise RuntimeError("simulation time step must be a floating point number above 0 to allow for the correct model initialization")

        self.camera_pos_ = None

        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, sim_time_step)

        plant.set_discrete_contact_approximation(DiscreteContactApproximation.kSap)
        if hydroelastic_contact:
            plant.set_contact_model(ContactModel.kHydroelastic)

        parser = Parser(plant)
        for model in models_to_add:
            parser.AddModels(model)
        parser.AddModels("abb/ABB_irb1200.urdf")
        parser.AddModels("svh/Schunk_SVH.urdf")
        
        svh = plant.GetModelInstanceByName("svh")
        robot = plant.GetModelInstanceByName("irb1200")

        scene_graph.collision_filter_manager().Apply(get_svh_geometry_filter(plant))

        plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"))
        plant.WeldFrames(plant.GetFrameByName("gripper_frame"), plant.GetFrameByName("base_link"))
        plant.Finalize()

        print(scene_graph.RendererCount())
        engine = MakeRenderEngineGl(RenderEngineGlParams())
        scene_graph.AddRenderer("default_renderer", engine)

        color_camera_info = CameraInfo(1920, 1080, 1322.77924362, 1310.10452962, 1920/2-0.5, 1080/2-0.5)
        depth_camera_info = CameraInfo(1280, 720, 634.0862423, 629.433346922, 1280/2-0.5, 800/2-0.5)
        
        color_camera = ColorRenderCamera(RenderCameraCore("default_renderer", color_camera_info, ClippingRange(0.1, 10.0), RigidTransform()))
        depth_camera = DepthRenderCamera(RenderCameraCore("default_renderer", depth_camera_info, ClippingRange(0.1, 10.0), RigidTransform()), DepthRange(0.28, 10.0))
    
        camera_pos = get_camera_pose(np.array([1.0, 0.5, 1.0]), np.array([0.7, 0.0, 0.0]))
        self.camera_pos_ = camera_pos
        cam0 = builder.AddSystem(RgbdSensor(scene_graph.world_frame_id(), camera_pos, color_camera, depth_camera))
        builder.Connect(scene_graph.get_query_output_port(), cam0.get_input_port())

        im_to_pc:System = builder.AddSystem(DepthImageToPointCloud(depth_camera_info))
        builder.Connect(cam0.GetOutputPort("depth_image_32f"), im_to_pc.depth_image_input_port())
        context = im_to_pc.CreateDefaultContext()
        im_to_pc.camera_pose_input_port().FixValue(context, camera_pos)
        im_to_pc.SetDefaultContext(context)

        abb_controller = builder.AddSystem(abb_inverse_dynamics_controller())
        builder.Connect(abb_controller.get_output_port(), plant.get_actuation_input_port(robot))
        builder.Connect(plant.get_state_output_port(robot), abb_controller.get_input_port_estimated_state())
        builder.ExportInput(abb_controller.get_input_port_desired_state(), "irb1200_desired_state")

        builder.ExportInput(plant.get_desired_state_input_port(svh), "svh_desired_state")
        builder.ExportInput(plant.get_actuation_input_port(svh), "svh_feed_forward_torque")
        builder.ExportOutput(plant.get_state_output_port(robot), "irb1200_state")
        builder.ExportOutput(plant.get_state_output_port(svh), "svh_state")
        builder.ExportOutput(plant.get_net_actuation_output_port(svh), "svh_net_actuation")
        builder.ExportOutput(cam0.GetOutputPort("color_image"), "camera0_color_image")
        builder.ExportOutput(cam0.GetOutputPort("depth_image_32f"), "camera0_depth_image")
        builder.ExportOutput(im_to_pc.get_output_port(), "point_cloud")
        
        AddDefaultVisualization(builder, meshcat=meshcat)
        diagram:Diagram = builder.BuildInto(self)