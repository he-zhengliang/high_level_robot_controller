#include "simulation/robot_diagram.hpp"
#include "simulation/package_path.hpp"

#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/sensors/rgbd_sensor.h>
#include <drake/systems/primitives/multiplexer.h>
#include <drake/systems/primitives/demultiplexer.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/geometry/meshcat.h>
#include <drake/visualization/visualization_config_functions.h>
#include <drake/geometry/render_gl/factory.h>
#include <drake/geometry/render/render_camera.h>
#include <drake/perception/point_cloud_flags.h>
#include <drake/perception/depth_image_to_point_cloud.h>
#include <cmath>

namespace simulation {

using drake::perception::pc_flags::BaseField;
using simulation::package_path;

namespace {
    int index(const std::vector<std::string>& list, std::string element) {
        int index = 0;
        while (!(list[index].compare(element) == 0)) 
            index++;
        return index;
    };
}

RobotDiagram::RobotDiagram(
    double sim_time_step, 
    std::shared_ptr<drake::geometry::Meshcat> meshcat,
    bool hydroelastic_contact,
    const std::vector<std::string>& models_to_add,
    BaseField pc_fields
) {
    auto builder = drake::systems::DiagramBuilder<double>();

    DRAKE_ASSERT(sim_time_step > 0);
    drake::multibody::MultibodyPlant<double>* plant;
    drake::geometry::SceneGraph<double>* scene_graph;

    std::tie(plant, scene_graph) = drake::multibody::AddMultibodyPlantSceneGraph(&builder, sim_time_step);

    plant->set_discrete_contact_approximation(drake::multibody::DiscreteContactApproximation::kSap);
    if (hydroelastic_contact) {
        plant->set_contact_model(drake::multibody::ContactModel::kHydroelastic);
    }

    auto parser = drake::multibody::Parser(plant);
    for (auto model : models_to_add) {
        parser.AddModels(model);
    }
    parser.AddModels(package_path::get_package_share_path("simulation") + "abb/ABB_irb1200.urdf");
    parser.AddModels(package_path::get_package_share_path("simulation") + "svh/Schunk_SVH.urdf");

    auto svh = plant->GetModelInstanceByName("svh");
    auto robot = plant->GetModelInstanceByName("irb1200");

    auto svh_collision_set = drake::geometry::GeometrySet();
    for (auto bi : plant->GetBodyIndices(svh)) {
        svh_collision_set.Add(plant->GetCollisionGeometriesForBody(plant->get_body(bi)));
    }
    auto collision_filter = drake::geometry::CollisionFilterDeclaration().ExcludeWithin(svh_collision_set);
    scene_graph->collision_filter_manager().Apply(collision_filter);

    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base"));
    plant->WeldFrames(plant->GetFrameByName("gripper_frame"), plant->GetFrameByName("base_link"));

    plant->Finalize();
/*
    auto engine = drake::geometry::MakeRenderEngineGl();
    scene_graph->AddRenderer("default_renderer", std::move(engine));

    int camera_pix_w = 1280;
    int camera_pix_h = 720;
    double depth_camera_sensor_w = 3.896;
    double depth_camera_sensor_h = 2.453;
    double depth_camera_focal_length = 1.93;
    double color_camera_sensor_w = 2.7288;
    double color_camera_sensor_h = 1.5498;
    double color_camera_focal_length = 1.88;

    auto color_camera_info = drake::systems::sensors::CameraInfo (
        camera_pix_w, 
        camera_pix_h, 
        color_camera_focal_length * camera_pix_w/color_camera_sensor_w, 
        color_camera_focal_length * camera_pix_h/color_camera_sensor_h, 
        camera_pix_w/2-0.5, 
        camera_pix_h/2-0.5
    );

    auto depth_camera_info = drake::systems::sensors::CameraInfo (
        camera_pix_w, 
        camera_pix_h, 
        depth_camera_focal_length * camera_pix_w/depth_camera_sensor_w, 
        depth_camera_focal_length * camera_pix_h/depth_camera_sensor_h, 
        camera_pix_w/2-0.5, 
        camera_pix_h/2-0.5
    );

    auto color_camera = drake::geometry::render::ColorRenderCamera(
        drake::geometry::render::RenderCameraCore(
            "default_renderer", 
            color_camera_info, 
            drake::geometry::render::ClippingRange(0.1, 10.0), 
            drake::math::RigidTransformd()
        )
    );

    auto depth_camera = drake::geometry::render::DepthRenderCamera(
        drake::geometry::render::RenderCameraCore(
            "default_renderer", 
            color_camera_info, 
            drake::geometry::render::ClippingRange(0.1, 10.0), 
            drake::math::RigidTransformd()
        ),
        drake::geometry::render::DepthRange(0.28, 10.0)
    );

    auto camera_pose_0 = this->get_camera_pose({1.1559625, 0.76555, 0.7}, {0.7, 0.0, 0.0});
    auto camera_pose_1 = this->get_camera_pose({1.1559625, -0.76555, 0.7}, {0.7, 0.0, 0.0});

    auto cam0 = builder.AddSystem<drake::systems::sensors::RgbdSensor>(scene_graph->world_frame_id(), camera_pose_0, color_camera, depth_camera);
    auto cam1 = builder.AddSystem<drake::systems::sensors::RgbdSensor>(scene_graph->world_frame_id(), camera_pose_1, color_camera, depth_camera);

    builder.Connect(scene_graph->get_query_output_port(), cam0->get_input_port());
    builder.Connect(scene_graph->get_query_output_port(), cam1->get_input_port());

    auto im_to_pc_0 = builder.AddSystem<drake::perception::DepthImageToPointCloud>(depth_camera_info);
    auto im_to_pc_1 = builder.AddSystem<drake::perception::DepthImageToPointCloud>(depth_camera_info);
    builder.Connect(cam0->GetOutputPort("depth_image_32f"), im_to_pc_0->depth_image_input_port());
    builder.Connect(cam0->body_pose_in_world_output_port(), im_to_pc_0->camera_pose_input_port());
    builder.Connect(cam1->GetOutputPort("depth_image_32f"), im_to_pc_1->depth_image_input_port());
    builder.Connect(cam1->body_pose_in_world_output_port(), im_to_pc_1->camera_pose_input_port());

    if (pc_fields & drake::perception::pc_flags::kRGBs) {
        builder.Connect(cam0->GetOutputPort("color_image"), im_to_pc_0->color_image_input_port());
        builder.Connect(cam1->GetOutputPort("color_image"), im_to_pc_1->color_image_input_port());
    }
*/
    auto abb_controller = RobotDiagram::abb_inverse_dynamics_controller(builder);

    builder.Connect(abb_controller->get_output_port(), plant->get_actuation_input_port(robot));
    builder.Connect(plant->get_state_output_port(robot), abb_controller->get_input_port_estimated_state());
    builder.ExportInput(abb_controller->get_input_port_desired_state(), "irb1200_desired_state");

    builder.ExportInput(plant->get_desired_state_input_port(svh), "svh_desired_state");
    builder.ExportInput(plant->get_actuation_input_port(svh), "svh_feed_forward_torque");
    builder.ExportOutput(plant->get_state_output_port(robot), "irb1200_state");

    auto names_raw = plant->GetStateNames(svh);
    std::vector<std::string> names;
    for (std::string name : names_raw) {
        if (isupper(name[10])) {
            names.push_back(name);
        }
    }

    auto demux = builder.AddSystem<drake::systems::Demultiplexer<double>>(names_raw.size(), 1);
    auto mux = builder.AddSystem<drake::systems::Multiplexer<double>>(names.size());
    builder.Connect(plant->get_state_output_port(svh), demux->get_input_port());

    for (unsigned long int i = 0; i < names.size(); i++) {
        builder.Connect(demux->get_output_port(index(names_raw, names[i])), mux->get_input_port(i));
    }

    builder.ExportOutput(mux->get_output_port(), "svh_state");
    builder.ExportOutput(plant->get_net_actuation_output_port(svh), "svh_net_actuation");

/*
    builder.ExportOutput(cam0->GetOutputPort("color_image"), "camera0_color_image");
    builder.ExportOutput(cam0->GetOutputPort("depth_image_32f"), "camera0_depth_image");
    builder.ExportOutput(im_to_pc_0->get_output_port(), "point_cloud");

    builder.ExportOutput(cam1->GetOutputPort("color_image"), "camera1_color_image");
    builder.ExportOutput(cam1->GetOutputPort("depth_image_32f"), "camera1_depth_image");
    
    builder.ExportOutput(im_to_pc_0->get_output_port(), "point_cloud_0");
    builder.ExportOutput(im_to_pc_1->get_output_port(), "point_cloud_1");
*/

    drake::visualization::AddDefaultVisualization(&builder, meshcat);

    builder.BuildInto(this);
}

drake::math::RigidTransformd RobotDiagram::get_camera_pose(const Eigen::Vector3d& camera_position, const Eigen::Vector3d& focus_point) const {
    Eigen::Vector3d focus_translation = camera_position - focus_point;
    double base_length = focus_translation(Eigen::seq(0,2)).norm();
    double x_rot_angle;
    if (abs(base_length) < 0.0001) {
        if (focus_translation(2) > 0) {
            x_rot_angle = M_PI_2f64;
        } else {
            x_rot_angle = -M_PI_2f64;
        }
    } else {
        x_rot_angle = atan(focus_translation(2)/base_length);
    }
    
    double z_rot_angle;
    if (abs(focus_translation(0)) < 0.0001) {
        if (focus_translation(1) > 0) {
            z_rot_angle = M_PI_2f64;
        } else {
            z_rot_angle = -M_PI_2f64;
        }
    } else {
        z_rot_angle = atan(focus_translation(1)/focus_translation(0));
    }

    auto camera_rot = drake::math::RotationMatrixd::MakeZRotation(M_PI_2f64 + z_rot_angle) * drake::math::RotationMatrixd::MakeXRotation(-M_PI_2f64 - x_rot_angle);

    return drake::math::RigidTransformd(camera_rot, camera_position);
}

drake::systems::controllers::InverseDynamicsController<double>* RobotDiagram::abb_inverse_dynamics_controller(drake::systems::DiagramBuilder<double>& builder)  {
    auto plant = std::make_unique<drake::multibody::MultibodyPlant<double>>(0.001);
    auto parser = drake::multibody::Parser(plant.get());
    parser.AddModels(simulation::package_path::get_package_share_path("simulation") + "abb/ABB_irb1200.urdf");
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base"));
    auto other_plant = drake::multibody::MultibodyPlant<double>(0.001);
    other_plant.set_discrete_contact_approximation(drake::multibody::DiscreteContactApproximation::kSap);
    auto other_parser = drake::multibody::Parser(&other_plant);
    other_parser.AddModels(simulation::package_path::get_package_share_path("simulation") + "svh/Schunk_SVH_no_collision.urdf");
    other_plant.Finalize();
    auto other_context = other_plant.CreateDefaultContext();
    
    auto svh_ = other_plant.GetModelInstanceByName("svh");
    auto svh_I = other_plant.CalcSpatialInertia(*other_context, other_plant.world_frame(), other_plant.GetBodyIndices(svh_));

    plant->Finalize();
    auto context = plant->CreateDefaultContext();
    auto& gripper = plant->GetBodyByName("gripper_frame");
    
    gripper.SetSpatialInertiaInBodyFrame(context.get(), svh_I);

    auto Kp = Eigen::Vector<double, 6>{50.0, 50.0, 50.0, 50.0, 100.0, 100.0};
    auto Ki = Eigen::Vector<double, 6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    auto Kd = Eigen::Vector<double, 6>{250.0, 250.0, 250.0, 250.0, 500.0, 500.0};

    return builder.AddSystem<drake::systems::controllers::InverseDynamicsController<double>>(std::move(plant), Kp, Ki, Kd, false, context.get());
}
} // namespace simulation