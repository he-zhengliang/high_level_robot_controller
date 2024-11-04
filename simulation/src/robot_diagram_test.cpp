#include "simulation/robot_diagram_test.hpp"
#include "simulation/package_path.hpp"
#include "simulation/abb_motion_planner.hpp"

#include <iostream>

#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/sensors/rgbd_sensor.h>
#include <drake/systems/primitives/multiplexer.h>
#include <drake/systems/primitives/demultiplexer.h>
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
 
    RobotDiagram::RobotDiagram(
        double sim_time_step, 
        std::shared_ptr<drake::geometry::Meshcat> meshcat,
        bool hydroelastic_contact,
        const std::vector<std::string>& models_to_add,
        drake::perception::pc_flags::BaseField pc_fields
    ) {
        std::cout << "I've started something\n";
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
        parser.AddModels(simulation::package_path::get_package_share_path("simulation") + "abb/ABB_irb1200.urdf");
        parser.AddModels(package_path::get_package_share_path("simulation") + "svh/Schunk_SVH.urdf");

        auto svh = plant->GetModelInstanceByName("svh");
        auto robot = plant->GetModelInstanceByName("irb1200");

        auto svh_collision_set = drake::geometry::GeometrySet();
        for (auto bi : plant->GetBodyIndices(svh)) {
            svh_collision_set.Add(plant->GetCollisionGeometriesForBody(plant->get_body(bi)));
        }
        auto collision_filter = drake::geometry::CollisionFilterDeclaration().ExcludeWithin(svh_collision_set);
        scene_graph->collision_filter_manager().Apply(collision_filter);

        plant->WeldFrames(plant->GetFrameByName("gripper_frame"), plant->GetFrameByName("base_link"));

        plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base"));
        plant->Finalize();

        drake::visualization::AddDefaultVisualization(&builder, meshcat);

        std::cout << "I'm here\n";

        auto abb_controller = abb_inverse_dynamics_controller(builder);
        std::cout << "Abb controller outputs: " << abb_controller->get_output_port().size() << std::endl;

        builder.Connect(abb_controller->get_output_port(), plant->get_actuation_input_port(robot));
        builder.Connect(plant->get_state_output_port(robot), abb_controller->get_input_port_estimated_state());
        builder.ExportInput(abb_controller->get_input_port_desired_state(), "irb1200_desired_state");

        builder.ExportInput(plant->get_desired_state_input_port(svh), "svh_desired_state");
        builder.ExportInput(plant->get_actuation_input_port(svh), "svh_feed_forward_torque");
        builder.ExportOutput(plant->get_state_output_port(robot), "irb1200_state");

        builder.BuildInto(this);
    }

    drake::math::RigidTransformd RobotDiagram::get_camera_pose(const Eigen::Vector3d& camera_position, const Eigen::Vector3d& focus_point) const {
        return drake::math::RigidTransformd();
    }

    drake::systems::controllers::InverseDynamicsController<double>* RobotDiagram::abb_inverse_dynamics_controller(drake::systems::DiagramBuilder<double>& builder) {
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

}