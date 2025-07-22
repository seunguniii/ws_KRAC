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
        {0,0,-7}, {100,0,-7}, {200,100,-7}, {200,-100,-7}, {100,0,-7}, {50,-50,-7}
    };
    std::vector<std::array<float,3>> tempwp_ = {
        {0,-100,-7}, {0,-100,-1.2}, {0,-100,-7}, {0,100,-7}, {0,100,-1.2}, {0,100,-5}, {0,0,-5}, {0,0,0}
    };

    size_t wp_idx_ = 0;
    int hold_counter_ = 0;
    const int HOLD_THRESHOLD = 20;         // 100ms * 20 = 2s

    int land_counter_ = 0;
    const int LAND_DISARM_DELAY = 50;      // 100ms * 50 = 5s
    
    const int WP0_HOVER_TICKS = 70;

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
        } else if (mission_mode_ == RESCUE) {
            rescue_and_return();
        }

        // 착륙 감지 후 지연 Disarm
        if (mission_mode_ == FINISHED && landed_detected_) {
            if (++land_counter_ > LAND_DISARM_DELAY) {
                disarm();
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
        Eigen::Vector3f dir = (tgt - cur).normalized();

        TrajectorySetpoint msg{};
        if (flight_mode_ == MULTIROTOR && armed_) {
        
            RCLCPP_INFO(this->get_logger(),
            	"[Multirotor] WP: %zu, Dist: %.2f", wp_idx_, dist);

        
            msg.position = {wp[0], wp[1], wp[2]};
            
                /* ── 변경된 부분 ─────────────────────────────── */
    	    int thresh = (wp_idx_ == 0) ? WP0_HOVER_TICKS : HOLD_THRESHOLD;
    	    
            if (dist < 1.0f && ++hold_counter_ > thresh) {
                hold_counter_ = 0;
                wp_idx_++;
                transition(FIXED_WING);
            }
        } else if (flight_mode_ == FIXED_WING) {
        
            RCLCPP_INFO(this->get_logger(),
            	"[Fixed-wing] WP: %zu, Dist: %.2f", wp_idx_, dist);
        
            msg.position = {wp[0], wp[1], wp[2]};
            msg.velocity = {k * dir.x(), k * dir.y(), 0.0f}; 
            
            if (dist < 10.0f) wp_idx_++;
            if (wp_idx_ >= waypoints_.size()) {
                transition(MULTIROTOR);
                mission_mode_ = RESCUE;
                wp_idx_ = 0;
                hold_counter_ = 0;
            }
        }
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
    }

    void rescue_and_return() {
        // 완료 후에도 마지막 위치 유지
        if (wp_idx_ >= tempwp_.size()) {
            TrajectorySetpoint msg{};
            auto wp_last = tempwp_.back();
            msg.position = {wp_last[0], wp_last[1], wp_last[2]};
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            trajectory_setpoint_publisher_->publish(msg);
            return;
        }
        auto &wp = tempwp_[wp_idx_];
        if (flight_mode_ == FIXED_WING) {
            transition(MULTIROTOR);
            hold_counter_ = 0;
        }
        Eigen::Vector3f cur(curr_odom_.position[0], curr_odom_.position[1], curr_odom_.position[2]);
        Eigen::Vector3f tgt(wp[0], wp[1], wp[2]);
        float dist = (tgt - cur).norm();
        const int SHORT_HOLD = HOLD_THRESHOLD;
        const int LONG_HOLD  = 70;
        int thresh = (wp[2] == -1.2f) ? LONG_HOLD : SHORT_HOLD;
        TrajectorySetpoint msg{};
        msg.position = {wp[0], wp[1], wp[2]};
        if (dist < 1.0f && ++hold_counter_ > thresh) {
            hold_counter_ = 0;
            wp_idx_++;
            if (wp_idx_ >= tempwp_.size()) mission_mode_ = FINISHED;
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
