#define MAV_COMP_ID_OFFBOARD 191

#include <iostream>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cmath>
#include <cstring>
#include <chrono>
#include <thread>

extern "C" {
#include <common/mavlink.h>
}

constexpr const char *TARGET_IP = "127.0.0.1";
constexpr int TARGET_PORT = 14580;

int sockfd;
struct sockaddr_in dest_addr;
uint8_t system_id = 1;
uint8_t component_id = MAV_COMP_ID_OFFBOARD;

void send_heartbeat() {
    mavlink_message_t msg;
    uint8_t buf[300];

    mavlink_msg_heartbeat_pack(system_id, component_id, &msg,
                               MAV_TYPE_GCS,
                               MAV_AUTOPILOT_GENERIC,
                               MAV_MODE_MANUAL_ARMED,
                               0, MAV_STATE_ACTIVE);
    int len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(sockfd, buf, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
}

void arm() {
    mavlink_message_t msg;
    uint8_t buf[300];

    mavlink_msg_command_long_pack(system_id, component_id, &msg,
                                  1, MAV_COMP_ID_AUTOPILOT1,
                                  MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                  1, 0, 0, 0, 0, 0, 0);
    int len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(sockfd, buf, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
}

void set_offboard_mode() {
    mavlink_message_t msg;
    uint8_t buf[300];

    // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
    // OFFBOARD mode for PX4 custom_mode = 6
    mavlink_msg_command_long_pack(
        system_id,
        component_id,
        &msg,
        1, // target_system (usually 1)
        1, // target_component (usually 1)
        MAV_CMD_DO_SET_MODE,
        0, // confirmation
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, // param1: base mode flag
        6, // param2: custom mode (OFFBOARD)
        0, 0, 0, 0, 0 // unused
    );

    int len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(sockfd, buf, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
}

void send_position_ned(float x, float y, float z, float yaw_deg) {
    mavlink_message_t msg;
    uint8_t buf[300];

    uint16_t type_mask = 0b0000111111000111; // Use only position and yaw
    uint8_t coordinate_frame = MAV_FRAME_LOCAL_NED;

    mavlink_msg_set_position_target_local_ned_pack(system_id, component_id, &msg,
        static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count()),
        1, MAV_COMP_ID_AUTOPILOT1,
        coordinate_frame,
        type_mask,
        x, y, z,
        0, 0, 0,
        0, 0, 0,
        yaw_deg * M_PI / 180.0, 0);

    int len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(sockfd, buf, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
}

int main() {
    // Create UDP socket
    sockfd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(TARGET_PORT);
    inet_aton(TARGET_IP, &dest_addr.sin_addr);

    std::cout << "Sending heartbeat...\n";
    for (int i = 0; i < 5; ++i) {
        send_heartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    std::cout << "Arming...\n";
    arm();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    std::cout << "Switching to OFFBOARD...\n";
    set_offboard_mode();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    std::cout << "Sending position setpoints...\n";
    while(true) {
        send_position_ned(0, 0, -10, 0); // Go to 2m altitude
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "Done. Closing socket.\n";
    close(sockfd);
    return 0;
}
