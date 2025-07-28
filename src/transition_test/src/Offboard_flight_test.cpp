#include <iostream>
#include <chrono>
#include <vector>
#include <stdint.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_control_mode.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"  // 착륙 감지 토픽

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

// OffboardControl 노드 클래스 정의
class OffboardControl : public rclcpp::Node {
public:
    OffboardControl() : Node("test") {
        // 오도메트리 구독
        odom_sub_ = this->create_subscription<VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
            [this](const VehicleOdometry::SharedPtr msg) {
                curr_odom_ = *msg;
                has_odom_ = true;
            }
        );
        // 착륙 감지 구독
        land_sub_ = this->create_subscription<VehicleLandDetected>(
            "/fmu/out/vehicle_land_detected", 1,
            [this](const VehicleLandDetected::SharedPtr msg) {
                landed_detected_ = msg->landed;
            }
        );

        // 퍼블리셔 설정
        offboard_control_mode_publisher_ =
            this->create_publisher<OffboardControlMode>(
                "/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ =
            this->create_publisher<TrajectorySetpoint>(
                "/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ =
            this->create_publisher<VehicleCommand>(
                "/fmu/in/vehicle_command", 10);

        // 주기 타이머
        timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));
    }

private:
    // 멤버 변수
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<VehicleLandDetected>::SharedPtr land_sub_;

    VehicleOdometry curr_odom_;
    bool has_odom_ = false;
    bool landed_detected_ = false;
    bool armed_ = false;

    enum FlightMode { MULTIROTOR = 3, FIXED_WING = 4 };
    enum Mission    { FLIGHT = 0, RESCUE = 1, FINISHED = 2 };
    FlightMode flight_mode_ = MULTIROTOR;
    Mission mission_mode_ = FLIGHT;

    std::vector<std::array<float,3>> waypoints_ = {
        {0,0,-5}, {5,0,-5}, {5,0,0}
    };

    size_t wp_idx_ = 0;
    int hold_counter_ = 0;
    const int HOLD_THRESHOLD = 100;         // 100ms * 20 = 2s

    int land_counter_ = 0;
    const int LAND_DISARM_DELAY = 50;      // 100ms * 50 = 5s

    float k = 1.0f;

    void timer_callback() {
        if (!has_odom_) return;

        // Arm & 모드 설정
        if (!armed_ && mission_mode_ != FINISHED) {
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            arm();
        }
        // Offboard 모드 메시지 발행
        publish_offboard_control_mode();

        // 상태별 제어
        if (mission_mode_ == FLIGHT) {
            publish_trajectory_setpoint();
        }
        // 착륙 감지 후 지연 Disarm
        if (mission_mode_ == FINISHED && landed_detected_) {
            if (++land_counter_ > LAND_DISARM_DELAY) {
                disarm();
                land_counter_ = 0;
            }
        }
    }

    void arm() {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        RCLCPP_INFO(this->get_logger(), "Arm command sent");
        armed_ = true;
    }

    void disarm() {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
        RCLCPP_INFO(this->get_logger(), "Disarm command sent");
        armed_ = false;
    }

    void publish_offboard_control_mode() {
        OffboardControlMode msg{};
        msg.position     = true;
        msg.velocity     = (flight_mode_ == MULTIROTOR) ? false : true;
        msg.acceleration = false;
        msg.attitude     = false;
        msg.body_rate    = false;
        msg.timestamp    = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    void publish_trajectory_setpoint() {
        if (curr_odom_.timestamp == 0) return;
        auto &wp = waypoints_[wp_idx_];
        Eigen::Vector3f cur(curr_odom_.position[0], curr_odom_.position[1], curr_odom_.position[2]);
        Eigen::Vector3f tgt(wp[0], wp[1], wp[2]);
        float dist = (tgt - cur).norm();

        TrajectorySetpoint msg{};
        if (flight_mode_ == MULTIROTOR && armed_) {
        
            RCLCPP_INFO(this->get_logger(),
            	"[Multirotor] WP: %zu, Dist: %.2f", wp_idx_, dist);

            msg.position = {wp[0], wp[1], wp[2]};
            if (dist < 3.0f) {
                hold_counter_++;
                if(hold_counter_>50){
                    wp_idx_++;
                    hold_counter_= 0;
                }

            }
            if (wp_idx_ >= waypoints_.size()) {
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
                mission_mode_ = FINISHED;
                return;
            }
        }
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
    }


    void publish_vehicle_command(uint16_t cmd, float p1 = 0.0f, float p2 = 0.0f) {
        VehicleCommand m{};
        m.command         = cmd;
        m.param1          = p1;
        m.param2          = p2;
        m.target_system   = 1;
        m.target_component= 1;
        m.from_external   = true;
        m.timestamp       = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(m);
    }

    void transition(FlightMode mode) {
        if (mode == flight_mode_) return;
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION, static_cast<float>(mode));
        flight_mode_ = mode;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
