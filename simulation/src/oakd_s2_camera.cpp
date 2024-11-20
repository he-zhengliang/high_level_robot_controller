#include "simulation/oakd_s2_camera.hpp"

#include <drake/systems/framework/diagram_builder.h>

#include <drake/systems/sensors/rgbd_sensor.h>
#include <drake/geometry/render_gl/factory.h>
#include <drake/geometry/render/render_camera.h>
#include <drake/perception/point_cloud_flags.h>
#include <drake/perception/depth_image_to_point_cloud.h>

namespace simulation {
    OakdS2Camera::OakdS2Camera(drake::geometry::SceneGraph<double>* scene_graph, const Eigen::Vector3d& camera_position, const Eigen::Vector3d& focus_point, bool include_color_in_point_cloud) {
        drake::systems::DiagramBuilder<double> builder;
        
        auto engine = drake::geometry::MakeRenderEngineGl();
        scene_graph->AddRenderer("default_renderer", std::move(engine));
        
        constexpr double color_vfov = 55.0 * M_PI / 180.0;
        auto color_camera_info = drake::systems::sensors::CameraInfo (
            4056, 
            3040,
            color_vfov
        );

        auto depth_camera_info = drake::systems::sensors::CameraInfo (
            1280,
            800,
            color_vfov
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
                depth_camera_info,
                drake::geometry::render::ClippingRange(0.1, 10.0),
                drake::math::RigidTransformd()
            ),
            drake::geometry::render::DepthRange(0.28, 10.0)
        );

        auto camera_pose = this->get_camera_pose(camera_position, focus_point);

        auto cam = builder.AddSystem<drake::systems::sensors::RgbdSensor>(scene_graph->world_frame_id(), camera_pose, color_camera, depth_camera);

        builder.Connect(scene_graph->get_query_output_port(), cam->get_input_port());

        auto im_to_pc = builder.AddSystem<drake::perception::DepthImageToPointCloud>(depth_camera_info);

        builder.Connect(cam->GetOutputPort("depth_image_32f"), im_to_pc->depth_image_input_port());
        builder.Connect(cam->body_pose_in_world_output_port(), im_to_pc->camera_pose_input_port());

        if (include_color_in_point_cloud) {
            builder.Connect(cam->GetOutputPort("color_image"), im_to_pc->color_image_input_port());
        }

        builder.ExportOutput(cam->GetOutputPort("color_image"), "color_image");
        builder.ExportOutput(cam->GetOutputPort("depth_image_32f"), "depth_image_32f");
        builder.ExportOutput(im_to_pc->get_output_port(), "point_cloud");

        builder.BuildInto(this);
    }

    drake::math::RigidTransformd OakdS2Camera::get_camera_pose(const Eigen::Vector3d& camera_position, const Eigen::Vector3d& focus_point) {
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

}