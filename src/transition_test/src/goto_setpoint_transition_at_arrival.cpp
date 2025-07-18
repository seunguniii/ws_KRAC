#include <chrono>
#include <iostream>
#include <stdint.h>
#include <math.h>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/goto_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
//#include <px4_msgs/msg/vtol_vehicle_status.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node {
	public:
		OffboardControl() : Node("test") {
			odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
											     [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
											     curr_odom_ = *msg; has_odom_ = true;});
			offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
			trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
			vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
			goto_setpoint_publisher_ = this->create_publisher<GotoSetpoint>("/fmu/in/goto_setpoint", 10);
			
			auto timer_callback = [this]() -> void {
				if(!has_odom_) {
					RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
					return;
				}

				if (!armed_) {
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
					this->arm();
				}

				publish_offboard_control_mode();
				publish_trajectory_setpoint();

				offboard_setpoint_counter_++;
			};
			timer_ = this->create_wall_timer(100ms, timer_callback);
		};

	private:
		rclcpp::TimerBase::SharedPtr timer_;
		std::atomic<uint64_t> timestamp_;

		rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
		rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
		rclcpp::Publisher<GotoSetpoint>::SharedPtr goto_setpoint_publisher_;
		rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

		rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
		
		px4_msgs::msg::VehicleOdometry curr_odom_;
		
		enum FlightMode {
		    MULTIROTOR = 3,
		    FIXED_WING = 4
		};
		FlightMode flight_mode_ = MULTIROTOR;

		std::vector<std::array<float,3>> waypoints_ = {
			{0.0f, 0.0f, -25.0f},
			{500.0f, 0.0f, -25.0f},
			{1000.0f, 500.0f, -25.0f},
			{1000.0f, -1000.0f, -25.0f},
			{500.0f, 0.0f, -25.0f},
			{0.0f, 0.0f, -25.0f},
			{0.0f, 0.0f, 0.0f},
		};

		uint64_t offboard_setpoint_counter_ {0};
		size_t wp_idx_ {0};

		bool has_odom_ = false;
		bool armed_ = false;
		bool in_fixed_wing_mode_ = false;
		
		void arm();
		void disarm();

		void publish_offboard_control_mode();
		void publish_trajectory_setpoint();
		void rescue_mission();
		void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
		void transition(FlightMode mode = MULTIROTOR);
		void publish_vehicle_command(int command, float value);
		int hold_counter_ = 0;
		const int HOLD_THRESHOLD = 20;
};

void OffboardControl::arm() {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
	armed_ = true;
}

void OffboardControl::disarm() {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void OffboardControl::publish_offboard_control_mode() {
	OffboardControlMode msg {};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

void OffboardControl::publish_trajectory_setpoint() {
	if (curr_odom_.timestamp == 0) {
		RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
		return;
	}

	TrajectorySetpoint msg {};
	GotoSetpoint msg_fw {};

	auto &wp = waypoints_[wp_idx_];

	Eigen::Vector3f current(curr_odom_.position[0], curr_odom_.position[1], curr_odom_.position[2]);
	Eigen::Vector3f target(wp[0], wp[1], wp[2]);
	Eigen::Vector3f to_wp = target - current;
	float dist_to_wp = to_wp.norm();

	if (flight_mode_ == MULTIROTOR && armed_) {
		RCLCPP_INFO(this->get_logger(), "[Multirotor] Distance to waypoint %d: %f", wp_idx_, dist_to_wp);
		msg_fw.position = {wp[0], wp[1], wp[2]};

		if (dist_to_wp < 3.0f) {
			hold_counter_++;
			if (hold_counter_ > HOLD_THRESHOLD) {
				hold_counter_ = 0;
				if(wp_idx_ >= waypoints_.size() - 1) this->disarm();
				wp_idx_++;
				if(wp_idx_ > 0) {
					RCLCPP_INFO(this->get_logger(), "Switching to fixed wing..."); 
					if(wp_idx_ < waypoints_.size() - 3){
						flight_mode_ = FIXED_WING;
						transition(flight_mode_);
					}
					offboard_setpoint_counter_ = 0;
				}
			}
		}
	}
	
	else if (flight_mode_ == FIXED_WING) {
		RCLCPP_INFO(this->get_logger(), "[Fixed-wing] Heading to projected point. WP: %d, Dist: %.2f", wp_idx_, dist_to_wp);
		RCLCPP_INFO(this->get_logger(), "[Fixed-wing] Desired Position %.2f, %.2f, %.2f\n", wp[0], wp[1], wp[2]);

		msg_fw.position = {wp[0], wp[1], wp[2]};
		if (dist_to_wp < 10.0f) {
			flight_mode_ = MULTIROTOR;
			transition(flight_mode_);
		}
	}
	
	msg_fw.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	goto_setpoint_publisher_->publish(msg_fw);
}

void OffboardControl::rescue_mission() {
	TrajectorySetpoint msg {};
	if(flight_mode_ == FIXED_WING) {
		flight_mode_ = MULTIROTOR;
		transition(flight_mode_);
	}
	Eigen::Vector3f current(curr_odom_.position[0], curr_odom_.position[1], curr_odom_.position[2]);
	Eigen::Vector3f target(0.0f, 0.0f, -50.0f);
	Eigen::Vector3f to_wp = target - current;
	float dist_to_wp = to_wp.norm();
	msg.position = {target[0], target[1], target[2]};
	if(dist_to_wp < 0.3f) {
		RCLCPP_INFO(this->get_logger(), "Rescue mission complete. Initiating landing sequence.");
	}
}

void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2) {
	VehicleCommand msg {};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

void OffboardControl::transition(FlightMode mode) {
	if ((mode == FIXED_WING && in_fixed_wing_mode_) ||
	        (mode == MULTIROTOR && !in_fixed_wing_mode_)) {
		RCLCPP_INFO(this->get_logger(), "Already in desired flight mode, no command sent.");
		return;
	}

	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION, static_cast<float>(mode));
	in_fixed_wing_mode_ = (mode == FIXED_WING);
	std::string mode_str = (mode == FIXED_WING) ? "Fixed-Wing" : "Multicopter";
	RCLCPP_INFO(this->get_logger(), "Sent VTOL transition command: %s", mode_str.c_str());
}

int main(int argc, char *argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
