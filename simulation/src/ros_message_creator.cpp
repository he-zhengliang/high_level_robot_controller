#include "simulation/ros_message_creator.hpp"

namespace simulation {

PointCloudMessageCreator::PointCloudMessageCreator(bool includes_color) : colored_(includes_color) {
    this->DeclareAbstractOutputPort("point_cloud_message", &PointCloudMessageCreator::calc_output);
    this->DeclareAbstractInputPort("point_cloud", *drake::AbstractValue::Make(drake::perception::PointCloud()));
}

void PointCloudMessageCreator::calc_output(const drake::systems::Context<double>& context, sensor_msgs::msg::PointCloud2* output) const {
    auto pc_in = this->get_input_port().Eval<drake::perception::PointCloud>(context);
    pc_in.xyzs();
    pc_in.rgbs();
    output->header.set__stamp(builtin_interfaces::msg::Time());
    output->header.set__frame_id("cam");
    output->set__height(1);
    output->set__width(pc_in.size());
    auto fields = sensor_msgs::msg::PointField();
    fields.name = "default";
    fields.offset = 0;
    fields.datatype = sensor_msgs::msg::PointField::FLOAT32;
    fields.count = pc_in.size();
    output->set__fields({fields});
    output->set__is_bigendian(false);
    int point_step;
    if (colored_) {
        point_step = 15;
        DRAKE_ASSERT(false);
        // The colored part needs to be confirmed
    } else {
        point_step = 3*sizeof(float);
    }

    output->set__point_step(point_step);
    int row_size = point_step * pc_in.size();
    output->set__row_step(row_size);

    output->data = std::vector<uint8_t>((uint8_t*)pc_in.xyzs().data(), (uint8_t*)pc_in.xyzs().data() + row_size);
    output->is_dense = true;
}

SvhDynamicJointStateMessageCreator::SvhDynamicJointStateMessageCreator() {
    this->DeclareVectorInputPort("svh_state", 18);
    this->DeclareVectorInputPort("svh_effort", 9);
    this->DeclareAbstractOutputPort("dynamic_joint_state_message", &SvhDynamicJointStateMessageCreator::create_message);
}

void SvhDynamicJointStateMessageCreator::create_message(const drake::systems::Context<double>& context, control_msgs::msg::DynamicJointState* message) const {
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

AbbJointStateMessageCreator::AbbJointStateMessageCreator() {
    this->DeclareVectorInputPort("abb_state", 12);
    this->DeclareAbstractOutputPort("joint_state_message", &AbbJointStateMessageCreator::create_message);
}

void AbbJointStateMessageCreator::create_message(const drake::systems::Context<double>& context, sensor_msgs::msg::JointState* message) const {
    auto state = this->get_input_port(0).Eval(context);

    *message = sensor_msgs::msg::JointState();

    message->name = names;
    message->position = std::vector<double>(state.data(), state.data() + 6 * sizeof(double));
}

};