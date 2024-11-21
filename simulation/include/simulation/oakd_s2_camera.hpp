#include <drake/systems/framework/diagram.h>
#include <drake/geometry/scene_graph.h>

namespace simulation {

class OakdS2Camera : public drake::systems::Diagram<double> {
public:
    explicit OakdS2Camera(drake::geometry::SceneGraph<double>* scene_graph, const Eigen::Vector3d& camera_position, const Eigen::Vector3d& focus_point, bool include_color_in_point_cloud=false);
private:
    static inline drake::math::RigidTransformd get_camera_pose(const Eigen::Vector3d& camera_position, const Eigen::Vector3d& focus_point);
};

}
