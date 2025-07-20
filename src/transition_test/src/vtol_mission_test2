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
#include "px4_msgs/msg/vehicle_land_detected.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node {
public:
    OffboardControl() : Node("test") {
        odom_sub_ = this->create_subscription<VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
            [this](const VehicleOdometry::SharedPtr msg) {
                curr_odom_ = *msg;
                has_odom_ = true;
            });

        land_sub_ = this->create_subscription<VehicleLandDetected>(
            "/fmu/out/vehicle_land_detected", 1,
            [this](const VehicleLandDetected::SharedPtr msg) {
                landed_detected_ = msg->landed;
            });

        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));
    }

private:
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
    enum Mission { FLIGHT = 0, RESCUE = 1, FINISHED = 2 };
    FlightMode flight_mode_ = MULTIROTOR;
    Mission mission_mode_ = FLIGHT;

    std::vector<std::array<float, 3>> waypoints_ = {
        {0, 0, -25}, {100, 0, -25}, {200, 100, -25}, {200, -100, -25}, {100, 0, -25}, {50, -50, -25}};
    std::vector<std::array<float, 3>> tempwp_ = {
        {0, -100, -25}, {0, -100, -5}, {0, -100, -25}, {0, 100, -25}, {0, 100, -5}, {0, 0, -5}, {0, 0, 0}};

    size_t wp_idx_ = 0;
    int hold_counter_ = 0;
    const int HOLD_THRESHOLD = 20;

    int land_counter_ = 0;
    const int LAND_DISARM_DELAY = 50;

    float k = 1.0f;
    float current_speed_ = 0.0f;
    const float MAX_SPEED = 15.0f;
    bool wp0_hovering_ = false;
    rclcpp::Time wp0_reached_time_;

    void timer_callback() {
        if (!has_odom_) return;

        if (!armed_ && mission_mode_ != FINISHED) {
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            arm();
        }
        publish_offboard_control_mode();

        if (mission_mode_ == FLIGHT) {
            publish_trajectory_setpoint();
        } else if (mission_mode_ == RESCUE) {
            rescue_and_return();
        }

        if (mission_mode_ == FINISHED && landed_detected_) {
            if (++land_counter_ > LAND_DISARM_DELAY) disarm();
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
        msg.position = true;
        msg.velocity = (flight_mode_ == MULTIROTOR) ? false : true;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
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
            if (wp_idx_ == 0) {
                msg.position = {wp[0], wp[1], wp[2]};
                if (!wp0_hovering_) {
                    wp0_reached_time_ = this->get_clock()->now();
                    wp0_hovering_ = true;
                    RCLCPP_INFO(this->get_logger(), "[Hovering] Starting hover at WP0");
                } else {
                    auto hover_duration = this->get_clock()->now() - wp0_reached_time_;
                    if (hover_duration.seconds() >= 5.0) {
                        wp_idx_++;
                        RCLCPP_INFO(this->get_logger(), "[Hovering] Done 5s hover, move to WP1");
                    }
                }
            } else if (wp_idx_ == 1) {
                RCLCPP_INFO(this->get_logger(), "[Multirotor] WP1, Dist: %.2f", dist);

                float accel = 1.0f;
                if (current_speed_ < MAX_SPEED) current_speed_ += accel;
                msg.velocity = {current_speed_ * dir.x(), current_speed_ * dir.y(), 0.0f};

                if (current_speed_ >= MAX_SPEED) {
                    transition(FIXED_WING);
                    RCLCPP_INFO(this->get_logger(), "[Transition] Switching to FIXED_WING at 15 m/s");
                }
            }
        } else if (flight_mode_ == FIXED_WING) {
            RCLCPP_INFO(this->get_logger(), "[Fixed-wing] WP: %zu, Dist: %.2f", wp_idx_, dist);
            msg.position = {wp[0], wp[1], wp[2]};
            msg.velocity = {MAX_SPEED * dir.x(), MAX_SPEED * dir.y(), 0.0f};

            if (dist < 10.0f) wp_idx_++;
            if (wp_idx_ >= waypoints_.size()) {
                transition(MULTIROTOR);
                mission_mode_ = RESCUE;
                wp_idx_ = 0;
                hold_counter_ = 0;
                current_speed_ = 0.0f;
                wp0_hovering_ = false;
            }
        }

        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
    }

    void rescue_and_return() {
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
        const int LONG_HOLD = 70;
        int thresh = (wp[2] == -5.0f) ? LONG_HOLD : SHORT_HOLD;
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
        m.command = cmd;
        m.param1 = p1;
        m.param2 = p2;
        m.target_system = 1;
        m.target_component = 1;
        m.from_external = true;
        m.timestamp = this->get_clock()->now().nanoseconds() / 1000;
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
