#include "abb_driver/abb_driver.hpp"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <drake/math/rigid_transform.h>

#include "egm.pb.h"

#define UDP_SERVER_PORT 3842
#define TCP_CLIENT_PORT 5555

namespace controller {
std::atomic_bool stop_threads = false;

AbbDriver::AbbDriver() : thread_safe_state_(std::atomic<Eigen::Vector<double, 6>>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0})) {
    
    this->DeclareAbstractInputPort("ee_pose", *drake::AbstractValue::Make(drake::math::RigidTransformd()));
    
    this->DeclareVectorOutputPort("position", &AbbDriver::state_output_callback);

    udp_thread_ = std::thread(&AbbDriver::udp_read, this);

    tcp_connect();
}

AbbDriver::~AbbDriver() {
    stop_threads.store(true);
    udp_thread_.join();
}

void AbbDriver::state_output_callback(const drake::systems::Context<double>& context, drake::systems::BasicVector<double>* output) const {
    output->get_mutable_value() = thread_safe_state_.load();
}

void AbbDriver::udp_read() {
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
            int num_joints = msg.feedback().joints().joints_size();
            auto joint_pos = std::vector<double>(msg.feedback().joints().joints().begin(), msg.feedback().joints().joints().end());
            thread_safe_state_.store(
                Eigen::Map<Eigen::VectorXd>((double*) msg.feedback().joints().joints().begin(), num_joints)
            );
        }
    }
}

void AbbDriver::tcp_connect() {
    
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

    if (listen(tcp_sockfd_, 5) < 0) {
      perror("listening failed");
      return;
    }

    struct sockaddr_in tcp_client_addr;
    socklen_t tcp_client_len;
    
    while (true) {
        tcp_connected_sockfd_ = accept(tcp_sockfd_, reinterpret_cast<sockaddr*>(&tcp_client_addr), &tcp_client_len);
        if (tcp_connected_sockfd_ < 0) {
            perror("connection failed!");
            continue;
        } else {
            break;
        }
    }
}

};