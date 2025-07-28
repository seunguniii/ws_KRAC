#define PI 3.14

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
		
		land_sub_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected", rclcpp::SensorDataQoS(),
			[this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
				landed_ = msg->landed;
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
		rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_sub_;

    VehicleOdometry curr_odom_;
    bool has_odom_ = false;
    bool armed_ = false;
    bool landed_ = false;

    std::vector<std::array<float, 3>> waypoints_ = {
        {0, 0, -5}, {0, 0, -5}, {0, 0, -5}, {0, 0, -5},
		{5, 0, -5}, {-5, 0, -5}, {0, 0, -5},
		{0, -5, -5}, {0, 5, -5}, {0, 0, -5},
	};

	float yaw[10] = {0, -PI/2, PI/2, 0, 
					 0, 0, 0, 0, 0, 0};

    size_t wp_idx_ = 0;
    int hold_counter_ = 0;
    const int HOLD_THRESHOLD = 40;

    void timer_callback() {
        if (!has_odom_) return;

        if (!armed_) {
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            arm();
        }
        
        publish_offboard_control_mode();
        publish_trajectory_setpoint();
    }

    void arm() {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        RCLCPP_INFO(this->get_logger(), "Arm command sent");
        armed_ = true;
    }

	void disarm() {
		publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

		RCLCPP_INFO(this->get_logger(), "Disarm command send");
		armed_ = false;
	}

    void publish_offboard_control_mode() {
        OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
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
        Eigen::Vector3f dir = tgt - cur;
        dir.normalize();
        
        float dist = (tgt - cur).norm();
        
        const float MAX_SPEED = 0.3f;

        TrajectorySetpoint msg{};
		
		if(wp_idx_ > 9){
			publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
			if(landed_ && armed_){
				disarm();
				disarm();
				disarm();
			}
		}
		
        if (armed_) {
        	if(dist < 0.3f) hold_counter_ ++;
        	if(hold_counter_ > HOLD_THRESHOLD) {
        		wp_idx_ ++;
        		hold_counter_ = 0;
			}
		}
		
        msg.position = {tgt[0], tgt[1], tgt[2]};
        msg.velocity = {dir[0] * MAX_SPEED, dir[1] * MAX_SPEED, dir[2] * MAX_SPEED};
        msg.yaw = yaw[wp_idx_];
        msg.yawspeed = PI/6;
		
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
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
