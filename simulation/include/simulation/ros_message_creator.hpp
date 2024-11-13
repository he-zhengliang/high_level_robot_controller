#include <drake/systems/framework/leaf_system.h>
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <vector>

class SvhDynamicJointStateMessageCreator : public drake::systems::LeafSystem<double> {
public:
    explicit SvhDynamicJointStateMessageCreator() {
        this->DeclareVectorInputPort("svh_state", 18);
        this->DeclareVectorInputPort("svh_effort", 9);
        this->DeclareAbstractOutputPort("dynamic_joint_state_message", &SvhDynamicJointStateMessageCreator::create_message);
    }

private:
    void create_message(const drake::systems::Context<double>& context, control_msgs::msg::DynamicJointState* message) const {
        auto state = this->get_input_port(0).Eval(context);
        auto effort = this->get_input_port(1).Eval(context);

        *message = control_msgs::msg::DynamicJointState();
        message->joint_names = names;

        message->interface_values.resize(9);
        for (int i = 0; i < 9; i++) {
            message->interface_values[i].interface_names = {"position", "velocity", "effort", "current"};
            message->interface_values[i].values = {state(i), state(i+9), effort(i), 0.0};
        }
    }

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

class AbbDynamicJointStateMessageCreator : public drake::systems::LeafSystem<double> {
public:
    explicit AbbDynamicJointStateMessageCreator() {
        this->DeclareVectorInputPort("abb_state", 12);
        this->DeclareAbstractOutputPort("dynamic_joint_state_message", &AbbDynamicJointStateMessageCreator::create_message);
    }

private:
    void create_message(const drake::systems::Context<double>& context, control_msgs::msg::DynamicJointState* message) const {
        auto state = this->get_input_port(0).Eval(context);

        *message = control_msgs::msg::DynamicJointState();

        message->joint_names = names;

        message->interface_values.resize(6);
        for (int i = 0; i < 6; i++) {
            message->interface_values[i].interface_names = {"position", "velocity"};
            message->interface_values[i].values = {state(i), state(i+6)};
        }
    }

    std::vector<std::string> names = {
        "1",
        "2",
        "3",
        "4",
        "5",
        "6"
    };

};
