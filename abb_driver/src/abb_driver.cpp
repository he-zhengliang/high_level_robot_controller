#include "abb_driver/abb_driver.hpp"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <mutex>
#include <thread>
#include <chrono>

#include <drake/math/rigid_transform.h>

#include "egm.pb.h"

#define UDP_SERVER_PORT 3842
#define TCP_CLIENT_PORT 5555

namespace controller {

std::mutex state_mutex;
std::atomic_bool stop_threads = false;

AbbDriver::AbbDriver() : thread_safe_state_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0} {
    
    this->DeclareAbstractInputPort("ee_pose", *drake::AbstractValue::Make(drake::math::RigidTransformd()));
    
    this->DeclareVectorOutputPort("position", 6, &AbbDriver::state_output_callback);
    this->DeclarePeriodicPublishEvent(0.5, 0.0, &AbbDriver::periodic_publish);

    this->DeclareAbstractState(*drake::AbstractValue::Make(drake::math::RigidTransformd()));
    this->DeclarePerStepUnrestrictedUpdateEvent(&AbbDriver::ee_publish);

    udp_thread_ = std::thread(&AbbDriver::udp_read, this);

    tcp_connect();
}

AbbDriver::~AbbDriver() {
    stop_threads.store(true);
    udp_thread_.join();

    const char to_send[6] = "hello";
    send(tcp_sockfd_, to_send, 5, 0);

    std::this_thread::sleep_for(std::chrono::microseconds(500));

    close(tcp_sockfd_);
}

drake::systems::EventStatus AbbDriver::periodic_publish(const drake::systems::Context<double>& context) const {
    return drake::systems::EventStatus::Succeeded();
}

drake::systems::EventStatus AbbDriver::ee_publish(const drake::systems::Context<double>& context, drake::systems::State<double>* state) const {
    auto transform = this->get_input_port().Eval<drake::math::RigidTransformd>(context);
    if (transform.translation() == context.get_abstract_state<drake::math::RigidTransformd>(0).translation() && 
        transform.rotation().ToQuaternionAsVector4() == context.get_abstract_state<drake::math::RigidTransformd>(0).rotation().ToQuaternionAsVector4()) {
        return drake::systems::EventStatus::Succeeded();
    }

    char send_command[] = "movej";
    char data_to_send_buffer[5 + 7 * sizeof(float)];
    memcpy(data_to_send_buffer, send_command, 5);
    float float_data_to_send[7];

    for (size_t i = 0; i < 3; i++)
        float_data_to_send[i] = (float) transform.translation()[i] * 1000;
    
    for (size_t i = 0; i < 4; i++)
        float_data_to_send[i+3] = (float) transform.rotation().ToQuaternionAsVector4()[i];

    printf("Sending p:(%f, %f, %f), q:(%f, %f, %f, %f)\n", 
        float_data_to_send[0],
        float_data_to_send[1],
        float_data_to_send[2],
        float_data_to_send[3],
        float_data_to_send[4],
        float_data_to_send[5],
        float_data_to_send[6]
    );

    memcpy(data_to_send_buffer+5, float_data_to_send, 7*sizeof(float));
    send(tcp_sockfd_, (void*) data_to_send_buffer, sizeof(data_to_send_buffer), 0);

    state->get_mutable_abstract_state<drake::math::RigidTransformd>(0) = transform;

    return drake::systems::EventStatus::Succeeded();
}

void AbbDriver::state_output_callback(const drake::systems::Context<double>& context, drake::systems::BasicVector<double>* output) const {
    state_mutex.lock();
    output->get_mutable_value() = this->thread_safe_state_;
    state_mutex.unlock();
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
            state_mutex.lock();
            thread_safe_state_ = Eigen::Map<Eigen::VectorXd>((double*) msg.feedback().joints().joints().begin(), num_joints);
            state_mutex.unlock();
        }
    }
}

void AbbDriver::tcp_connect() {
    printf("Waiting for the ABB to connect over TCP");
    
    int temp_tcp_sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in tcp_server_addr;

    memset(&tcp_server_addr, 0, sizeof(tcp_server_addr));
    tcp_server_addr.sin_family = AF_INET;
    tcp_server_addr.sin_addr.s_addr = INADDR_ANY;
    tcp_server_addr.sin_port = htons(TCP_CLIENT_PORT);

    if (bind(temp_tcp_sockfd_, reinterpret_cast<sockaddr*>(&tcp_server_addr), sizeof(tcp_server_addr)) < 0) {
      perror("Failed to bind to port");
      return;
    }

    if (listen(temp_tcp_sockfd_, 5) < 0) {
      perror("listening failed");
      return;
    }

    struct sockaddr_in tcp_client_addr;
    socklen_t tcp_client_len;
    
    while (true) {
        tcp_sockfd_ = accept(temp_tcp_sockfd_, reinterpret_cast<sockaddr*>(&tcp_client_addr), &tcp_client_len);
        if (tcp_sockfd_ < 0) {
            perror("connection failed!");
            continue;
        } else {
            break;
        }
    }
    printf("TCP is connected\n");

}

};