#include <iostream>
#include <chrono>
#include <vector>
#include <stdint.h>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/goto_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

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
				if(!has_odom_){
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
			{200.0f, 0.0f, -25.0f},
			{400.0f, 200.0f, -25.0f},
			{200.0f, -400.0f, -25.0f},
			{200.0f, 0.0f, -25.0f},
			{0.0f, 0.0f, -25.0f},
			{0.0f, 0.0f, 0.0f},
		};

		uint64_t offboard_setpoint_counter_ {0};
		size_t wp_idx_ {0};


		bool has_odom_ = false;
		bool armed_ = false;
		
		int hold_counter_ = 0;
		const int HOLD_THRESHOLD = 20;
		
		void arm();
		void disarm();

		void publish_offboard_control_mode();
		void publish_trajectory_setpoint();
		void rescue_mission();
		void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
		void transition(FlightMode mode = MULTIROTOR);

		void publish_vehicle_command(int command, float value);
		float L1_guidance(float vx_, float vy_, float currx_, float curry_);
};

void OffboardControl::arm() {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
	armed_ = true;
}

void OffboardControl::disarm() {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
	armed_ = false;
}

void OffboardControl::publish_offboard_control_mode() {
	OffboardControlMode msg {};
	msg.position = flight_mode_ == MULTIROTOR? true:false;
	msg.velocity = false;
	msg.acceleration = flight_mode_ == MULTIROTOR? false:true;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

float OffboardControl::L1_guidance(float vx_, float vy_, float currx_, float curry_){
	float sin_eta;
	float v_size = sqrt(vx_*vx_ + vy_*vy_);
	float L_size = sqrt(currx_*currx_ + curry_*curry_);
	
	if(v_size*L_size == 0) sin_eta = 0;
	else sin_eta = (vx_*curry_ - vy_*currx_)/(v_size*L_size);
	
	return 2*v_size*v_size/L_size*sin_eta;
}

void OffboardControl::publish_trajectory_setpoint() {
	if (curr_odom_.timestamp == 0) {
		RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
		return;
	}

	TrajectorySetpoint msg_fw {};

	auto &wp = waypoints_[wp_idx_];
	
	Eigen::Vector3f current(curr_odom_.position[0], curr_odom_.position[1], curr_odom_.position[2]);
	Eigen::Vector3f target(wp[0], wp[1], wp[2]);
	Eigen::Vector3f to_wp = target - current;
	float dist_to_wp = to_wp.norm();
	
	Eigen::Vector3f vel_frd(curr_odom_.velocity[0], curr_odom_.velocity[1], curr_odom_.velocity[2]);
	Eigen::Quaternionf q_frd(curr_odom_.q[0], curr_odom_.q[1], curr_odom_.q[2], curr_odom_.q[3]);
	Eigen::Quaternionf q_ned = q_frd.inverse();
	Eigen::Vector3f desired_wp_frd = q_ned * to_wp;
	Eigen::Vector3f acc_frd, acc_ned;

	if (flight_mode_ == MULTIROTOR && armed_) {
		RCLCPP_INFO(this->get_logger(), "[Multirotor] Distance to waypoint %ld: %f", wp_idx_, dist_to_wp);
		msg_fw.position = {wp[0], wp[1], wp[2]};

		if (dist_to_wp < 3.0f) {
			hold_counter_++;
			if (hold_counter_ > HOLD_THRESHOLD) {
				hold_counter_ = 0;
				if(wp_idx_ >= waypoints_.size() - 1) this->disarm();
				wp_idx_++;
				transition(FIXED_WING);

				offboard_setpoint_counter_ = 0;
			}
		}
	}
	
	else if (flight_mode_ == FIXED_WING) {
		RCLCPP_INFO(this->get_logger(), "[Fixed-wing] Heading to projected point. WP: %ld, Dist: %.2f", wp_idx_, dist_to_wp);
		RCLCPP_INFO(this->get_logger(), "[Fixed-wing] Desired Position %.2f, %.2f, %.2f\n", wp[0], wp[1], wp[2]);
		
		acc_frd << 0.0f,
					L1_guidance(vel_frd.y(), vel_frd.x(), desired_wp_frd.y(), desired_wp_frd.x()),
					L1_guidance(vel_frd.x(), vel_frd.z(), desired_wp_frd.x(), desired_wp_frd.z());
		
		acc_ned = q_ned * acc_frd;
		msg_fw.acceleration = {acc_ned[0], acc_ned[1], acc_ned[2]};
		
		if (dist_to_wp < 10.0f) {
			wp_idx_++;
		}
	}
	
	msg_fw.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg_fw);
}

void OffboardControl::rescue_mission() {
	TrajectorySetpoint msg {};
	if(flight_mode_ == FIXED_WING) {
		transition(MULTIROTOR);
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
	if (mode == flight_mode_) {
		RCLCPP_INFO(this->get_logger(), "Already in desired flight mode, no command sent.");
		return;
	}

	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION, static_cast<float>(mode));
	std::string mode_str = (mode == FIXED_WING) ? "Fixed-Wing" : "Multicopter"; 
	RCLCPP_INFO(this->get_logger(), "Sent VTOL transition command: %s", mode_str.c_str());
	flight_mode_ = mode;
}

int main(int argc, char *argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
