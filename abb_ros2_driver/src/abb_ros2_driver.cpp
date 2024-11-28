#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <thread>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <atomic>
#include <egm.pb.h>

#define UDP_SERVER_PORT 3842
#define TCP_CLIENT_PORT 5555

std::atomic_bool stop_threads = false;

class AbbRos2Driver : public rclcpp::Node {
public:
  explicit AbbRos2Driver() : Node("abb_ros2_driver") {    
    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>("/abb_irb1200/ee_pose", 10, std::bind(&AbbRos2Driver::subscription_callback, this, std::placeholders::_1));

    udp_thread_ = std::thread(&AbbRos2Driver::udp_read, this);
    RCLCPP_INFO(this->get_logger(), "Started UDP thread");

    tcp_sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in tcp_server_addr;

    memset(&tcp_server_addr, 0, sizeof(tcp_server_addr));
    tcp_server_addr.sin_family = AF_INET;
    tcp_server_addr.sin_addr.s_addr = INADDR_ANY;
    tcp_server_addr.sin_port = htons(TCP_CLIENT_PORT);

    if (bind(tcp_sockfd_, reinterpret_cast<sockaddr*>(&tcp_server_addr), sizeof(tcp_server_addr)) < 0) {
      perror("Failed to bind to port");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "TCP Port has been bound");

    if (listen(tcp_sockfd_, 5) < 0) {
      perror("listening failed");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "TCP Port has been listened to successfully");

    struct sockaddr_in tcp_client_addr;
    socklen_t tcp_client_len;
    
    tcp_connected_sockfd_ = accept(tcp_sockfd_, reinterpret_cast<sockaddr*>(&tcp_client_addr), &tcp_client_len);
    if (tcp_connected_sockfd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "TCP Connection Failed");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "TCP Port is connected");

  }

  ~AbbRos2Driver() {
    close(tcp_sockfd_);
    close(tcp_connected_sockfd_);
    stop_threads = true;
    udp_thread_.join();
  }

private:
  void subscription_callback(const geometry_msgs::msg::Pose::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "position (%f, %f, %f)  orientation (%f, %f, %f, %f)", msg->position.x, msg->position.y, msg->position.z, msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    float abb_msg_buffer[7] = {
      (float) msg->position.x, 
      (float) msg->position.y, 
      (float) msg->position.z, 
      (float) msg->orientation.w, 
      (float) msg->orientation.x, 
      (float) msg->orientation.y, 
      (float) msg->orientation.z
    };

    send(tcp_connected_sockfd_, abb_msg_buffer, sizeof(abb_msg_buffer), 0);
  }

  void udp_read() {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/abb_dynamic_joint_states", 10);
    int sockfd;
    struct sockaddr_in udp_server_addr, udp_client_addr;
    socklen_t udp_client_len = sizeof(udp_client_addr);

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    if (sockfd < 0) {
      perror("Failed to create a socket");
      return;
    }

    memset(&udp_server_addr, 0, sizeof(udp_server_addr));
    udp_server_addr.sin_family = AF_INET;
    udp_server_addr.sin_addr.s_addr = INADDR_ANY;
    udp_server_addr.sin_port = htons(UDP_SERVER_PORT);

    if (bind(sockfd, reinterpret_cast<sockaddr*>(&udp_server_addr), sizeof(udp_server_addr)) < 0) {
      perror("Failed to bind to port");
      return;
    }

    char buffer[1024];
    const std::vector<std::string> joint_names = {"1", "2", "3", "4", "5", "6"};

    while (! stop_threads) {
      ssize_t recieved_bytes = recvfrom(sockfd, buffer, sizeof(buffer), 0, reinterpret_cast<sockaddr*>(&udp_client_addr), &udp_client_len);
      if (recieved_bytes < 0) {
        perror("Recieved an invalid message");
        continue;
      }

      abb::egm::EgmRobot msg;
      msg.ParseFromArray(buffer, recieved_bytes);
      if (msg.has_feedback() && msg.feedback().has_joints()) {
        auto joint_pos = std::vector<double>(msg.feedback().joints().joints().begin(), msg.feedback().joints().joints().end());

        sensor_msgs::msg::JointState msg_out;
        msg_out.header.set__frame_id("Helolo");
        msg_out.header.set__stamp(this->now());

        msg_out.set__name(joint_names);
        msg_out.set__position(joint_pos);

        publisher_->publish(msg_out);
      }
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
  std::thread udp_thread_;

  int tcp_sockfd_;
  int tcp_connected_sockfd_;  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AbbRos2Driver>());
  rclcpp::shutdown();
  return 0;
}
