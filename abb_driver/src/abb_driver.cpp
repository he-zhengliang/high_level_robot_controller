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

#define UDP_CLIENT_PORT 3842
#define TCP_SERVER_PORT_START 5554

namespace controller {

std::mutex state_mutex;
std::atomic_bool stop_threads = false;

AbbDriver::AbbDriver(bool connect_to_simulation, std::string simulation_ip) :
    thread_safe_state_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    thread_safe_time_(0.0) {
    
    this->DeclareAbstractInputPort("ee_pose", *drake::AbstractValue::Make(drake::math::RigidTransformd()));
    
    this->DeclareVectorOutputPort("position", 6, &AbbDriver::state_output_callback);

    this->DeclareAbstractState(*drake::AbstractValue::Make(drake::math::RigidTransformd()));

    udp_thread_ = std::thread(&AbbDriver::udp_read, this);

    tcp_sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in tcp_server_addr;

    memset(&tcp_server_addr, 0, sizeof(tcp_server_addr));
    tcp_server_addr.sin_family = AF_INET;
    if (connect_to_simulation) {
        tcp_server_addr.sin_addr.s_addr = inet_addr(simulation_ip.c_str());
    } else {
        tcp_server_addr.sin_addr.s_addr = inet_addr("192.168.125.1");
    }
    uint16_t temp_port_no = TCP_SERVER_PORT_START;
    tcp_server_addr.sin_port = htons(temp_port_no);

    while (true) {
        if (connect(tcp_sockfd_, reinterpret_cast<sockaddr*>(&tcp_server_addr), sizeof(tcp_server_addr)) < 0) {
            if (errno == ECONNREFUSED) {
                printf("Nothing at %u\n", ntohs(tcp_server_addr.sin_port));
                if (ntohs(tcp_server_addr.sin_port) > 5560) {
                    printf("Failed to find an open port\n");
                    return;
                }
                tcp_server_addr.sin_port++;
                continue;
            }
            perror("Connecting to the TCP server failed");
            return;
        }
        else {
            break;
        }
    }

    char buff[1024];
    ssize_t rec_len = recv(tcp_sockfd_, buff, 1024, 0);
    printf("%.*s\n", rec_len, buff);

    this->DeclarePerStepUnrestrictedUpdateEvent(&AbbDriver::ee_publish);
}

AbbDriver::~AbbDriver() {
    stop_threads.store(true);
    udp_thread_.join();

    const char to_send[6] = "close";
    send(tcp_sockfd_, to_send, 5, 0);

    std::this_thread::sleep_for(std::chrono::microseconds(500));

    close(tcp_sockfd_);
}

drake::systems::EventStatus AbbDriver::ee_publish(const drake::systems::Context<double>& context, drake::systems::State<double>* state) const {
    auto transform = this->get_input_port().Eval<drake::math::RigidTransformd>(context);
    auto& old_transform = context.get_abstract_state<drake::math::RigidTransformd>(0);
    if (transform.IsExactlyEqualTo(old_transform)) {
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

    send(tcp_sockfd_, data_to_send_buffer, sizeof(data_to_send_buffer), 0);

    state->get_mutable_abstract_state<drake::math::RigidTransformd>(0) = transform;

    return drake::systems::EventStatus::Succeeded();
}

void AbbDriver::state_output_callback(const drake::systems::Context<double>& context, drake::systems::BasicVector<double>* output) const {
    (void) context;

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
    udp_server_addr.sin_port = htons(UDP_CLIENT_PORT);

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
            double msg_time = ((double)msg.header().tm()) / 1000.0;
            if (msg_time > thread_safe_time_) {
                thread_safe_state_ = Eigen::Map<Eigen::VectorXd>((double*) msg.feedback().joints().joints().begin(), num_joints);
                thread_safe_time_ = msg_time;
            }
            state_mutex.unlock();
        }
    }
}

};