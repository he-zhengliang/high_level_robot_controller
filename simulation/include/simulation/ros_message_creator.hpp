#ifndef SIMULATION__ROS_MESSAGE_CREATOR_HPP
#define SIMULATION__ROS_MESSAGE_CREATOR_HPP

#include <drake/systems/framework/leaf_system.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <drake/perception/point_cloud.h>
#include <vector>
#include <string>


namespace simulation {
class PointCloudMessageCreator : public drake::systems::LeafSystem<double> {

    /// This is possibly copying the entire point cloud. See if there is a way we can serialize a drake point cloud directly 
    /// as this will let us avoid copying the cloud
    /// 
public:
    explicit PointCloudMessageCreator(bool includes_color);

    void calc_output(const drake::systems::Context<double>& context, sensor_msgs::msg::PointCloud2* output) const;

    bool colored_;
};

class SvhJointStateMessageCreator : public drake::systems::LeafSystem<double> {
public:
    explicit SvhJointStateMessageCreator();
private:
    void create_message(const drake::systems::Context<double>& context, sensor_msgs::msg::JointState* message) const;

    const std::vector<std::string> names =  {
        "Left_Hand_Thumb_Opposition",
        "Left_Hand_Thumb_Flexion",
        "Left_Hand_Index_Finger_Proximal",
        "Left_Hand_Index_Finger_Distal",
        "Left_Hand_Middle_Finger_Proximal",
        "Left_Hand_Middle_Finger_Distal",
        "Left_Hand_Finger_Spread",
        "Left_Hand_Pinky",
        "Left_Hand_Ring_Finger"
    };
};

class AbbJointStateMessageCreator : public drake::systems::LeafSystem<double> {
public:
    explicit AbbJointStateMessageCreator();

private:
    void create_message(const drake::systems::Context<double>& context, sensor_msgs::msg::JointState* message) const;
    
    std::vector<std::string> names = {
        "1",
        "2",
        "3",
        "4",
        "5",
        "6"
    };
};

};
#endif