#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <thread>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define SERVER_PORT 5555

class TempNode : public rclcpp::Node {
public:
  explicit TempNode() : Node("temp") {
    publisher_ = this->create_publisher<control_msgs::msg::DynamicJointState>("/abb_dynamic_joint_states", 10);
    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>("/abb_irb1200/ee_pose", 10, std::bind(&TempNode::subscription_callback, this, std::placeholders::_1));

    udp_thread_ = std::thread(&TempNode::udp_read, this);

    
  }

private:
  void subscription_callback(const geometry_msgs::msg::Pose::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "position (%d, %d, %d)  orientation (%d, %d, %d, %d)", msg->position.x, msg->position.y, msg->position.z, msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    
  }

  static void udp_read() {
    int sockfd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len = sizeof(client_addr);

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    if (sockfd < 0) {
      perror("Failed to create a socket");
      return;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(SERVER_PORT);

    if (bind(sockfd, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) < 0) {
      perror("Failed to bind to port");
      return;
    }

    char buffer[1024];

    while (true /*insert a thread safe stop signal here*/) {
      ssize_t recieved_bytes = recvfrom(sockfd, buffer, sizeof(buffer), 0, reinterpret_cast<sockaddr*>(&client_addr), &client_len);

      if (recieved_bytes < 0) {
        perror("Recieved an invalid message");
        continue;
      }

      // add protobuf type and stuff here
    }
  }

  rclcpp::Publisher<control_msgs::msg::DynamicJointState>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
  std::thread udp_thread_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TempNode>());
  rclcpp::shutdown();
  return 0;
}
