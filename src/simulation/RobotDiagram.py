import numpy as np
from pydrake.multibody.plant import MultibodyPlant, DiscreteContactApproximation, AddMultibodyPlantSceneGraph, ContactModel
from pydrake.systems.sensors import RgbdSensor, CameraInfo, RgbdSensorDiscrete
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
    DepthRange
)
from pydrake.visualization import AddDefaultVisualization
from pydrake.math import RotationMatrix, RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.systems.controllers import InverseDynamicsController
from pydrake.systems.framework import Diagram, DiagramBuilder
from pydrake.perception import DepthImageToPointCloud, Fields, BaseField

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

    Kp = np.array([50.0, 50.0, 50.0, 50.0, 100.0, 100.0])
    Ki = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    Kd = np.array([250.0, 250.0, 250.0, 250.0, 500.0, 500.0])

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
    if abs(focus_translation[0]) < 0.0001:
        if focus_translation[1] > 0:
            z_rotation = np.pi/2
        else:
            z_rotation = -np.pi/2
    else:
        z_rotation = np.arctan(focus_translation[1]/focus_translation[0])

    camera_rot = RotationMatrix.MakeZRotation(np.pi/2+z_rotation) @ RotationMatrix.MakeXRotation(-np.pi/2 - x_rot_angle)
    return RigidTransform(camera_rot, camera_position)

class RobotDiagram(Diagram):
    def __init__(self, sim_time_step:float, meshcat:Meshcat=None, hydroelastic_contact:bool=False, models_to_add:list[str]=[], pc_fields=BaseField.kXYZs):
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

        engine = MakeRenderEngineGl(RenderEngineGlParams())
        scene_graph.AddRenderer("default_renderer", engine)

        camera_pix_w = 1280
        camera_pix_h = 720
        depth_camera_sensor_w = 3.896
        depth_camera_sensor_h = 2.453
        depth_camera_focal_length = 1.93
        color_camera_sensor_w = 2.7288
        color_camera_sensor_h = 1.5498
        color_camera_focal_length = 1.88

        color_camera_info = CameraInfo(
            camera_pix_w, 
            camera_pix_h, 
            color_camera_focal_length * camera_pix_w/color_camera_sensor_w, 
            color_camera_focal_length * camera_pix_h/color_camera_sensor_h, 
            camera_pix_w/2-0.5, 
            camera_pix_h/2-0.5
        )
        depth_camera_info = CameraInfo(
            camera_pix_w, 
            camera_pix_h, 
            depth_camera_focal_length * camera_pix_w/depth_camera_sensor_w, 
            depth_camera_focal_length * camera_pix_h/depth_camera_sensor_h, 
            camera_pix_w/2-0.5, 
            camera_pix_h/2-0.5
        )
        
        color_camera = ColorRenderCamera(RenderCameraCore("default_renderer", color_camera_info, ClippingRange(0.1, 10.0), RigidTransform()))
        depth_camera = DepthRenderCamera(RenderCameraCore("default_renderer", depth_camera_info, ClippingRange(0.1, 10.0), RigidTransform()), DepthRange(0.28, 10.0))
        self.camera_pose_0 = get_camera_pose(np.array([1.1559625, 0.76555, 0.7]), np.array([0.7, 0.0, 0.0]))
        self.camera_pose_1 = get_camera_pose(np.array([1.1559625, -0.76555, 0.7]), np.array([0.7, 0.0, 0.0]))

        cam0 = builder.AddSystem(RgbdSensor(scene_graph.world_frame_id(), self.camera_pose_0, color_camera, depth_camera))
        cam1 = builder.AddSystem(RgbdSensor(scene_graph.world_frame_id(), self.camera_pose_1, color_camera, depth_camera))

        builder.Connect(scene_graph.get_query_output_port(), cam0.get_input_port())
        builder.Connect(scene_graph.get_query_output_port(), cam1.get_input_port())

        im_to_pc_0 = builder.AddSystem(DepthImageToPointCloud(depth_camera_info, fields=pc_fields))
        im_to_pc_1 = builder.AddSystem(DepthImageToPointCloud(depth_camera_info, fields=pc_fields))
        builder.Connect(cam0.GetOutputPort("depth_image_32f"), im_to_pc_0.depth_image_input_port())
        builder.Connect(cam0.body_pose_in_world_output_port(), im_to_pc_0.camera_pose_input_port())
        builder.Connect(cam1.GetOutputPort("depth_image_32f"), im_to_pc_1.depth_image_input_port())
        builder.Connect(cam1.body_pose_in_world_output_port(), im_to_pc_1.camera_pose_input_port())

        if (pc_fields & BaseField.kRGBs):
            builder.Connect(cam0.GetOutputPort("color_image"), im_to_pc_0.color_image_input_port())
            builder.Connect(cam1.GetOutputPort("color_image"), im_to_pc_1.color_image_input_port())

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
        builder.ExportOutput(im_to_pc_0.get_output_port(), "point_cloud")

        builder.ExportOutput(cam1.GetOutputPort("color_image"), "camera1_color_image")
        builder.ExportOutput(cam1.GetOutputPort("depth_image_32f"), "camera1_depth_image")
        
        builder.ExportOutput(im_to_pc_0.get_output_port(), "point_cloud_0")
        builder.ExportOutput(im_to_pc_1.get_output_port(), "point_cloud_1")

        AddDefaultVisualization(builder, meshcat=meshcat)
        diagram = builder.BuildInto(self)