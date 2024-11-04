#ifndef SIMULATION__ROBOT_DIAGRAM_HPP
#define SIMULATION__ROBOT_DIAGRAM_HPP

#include <string>
#include <vector>
#include <memory>

#include <drake/systems/framework/diagram.h>
#include <drake/geometry/meshcat.h>
#include <drake/perception/point_cloud_flags.h>
#include <drake/math/rigid_transform.h>
#include <drake/systems/controllers/inverse_dynamics_controller.h>

namespace simulation {
    class RobotDiagram : public drake::systems::Diagram<double> {
    public:
        explicit RobotDiagram(
            double sim_time_step, 
            std::shared_ptr<drake::geometry::Meshcat> meshcat,
            bool hydroelastic_contact=false,
            const std::vector<std::string>& models_to_add = {},
            drake::perception::pc_flags::BaseField pc_fields = drake::perception::pc_flags::kXYZs
        );
    private:
        drake::math::RigidTransformd get_camera_pose(const Eigen::Vector3d& camera_position, const Eigen::Vector3d& focus_point) const;
        drake::systems::controllers::InverseDynamicsController<double>* abb_inverse_dynamics_controller(drake::systems::DiagramBuilder<double>& builder);
    };
}

#endif