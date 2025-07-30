#include <iostream>
#include <chrono>
#include <vector>
#include <stdint.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node {
	public:
		OffboardControl() : Node("test") {
			odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
			[this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
				curr_odom_ = *msg;
				has_odom_ = true;
			});

			desired_setpoint_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/tag_fused_point", 10,
			[this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
				target_from_sub_x_ = msg->point.x;
				target_from_sub_y_ = msg->point.y;
				accurate_altitude_ = msg->point.z;
			});

			land_sub_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected", rclcpp::SensorDataQoS(),
			[this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
				landed_ = msg->landed;
			});

			offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
			trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
			vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

			auto timer_callback = [this]() -> void {
				if(!has_odom_) {
					RCLCPP_WARN(this->get_logger(), "Waiting for ...");
					return;
				}

				if (!armed_ && mission_mode_ != FINISHED) {
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
					this->arm();
				}

				publish_offboard_control_mode();

				switch (mission_mode_) {
					default:
					case FLIGHT:
						publish_trajectory_setpoint();
						break;

					case LANDING:
						land();
						break;

					case FINISHED:
						if(landed_ && armed_) {
							disarm();
							disarm();
							disarm();
						}
						if(!armed_) return;
						break;
				}

				offboard_setpoint_counter_++;
			};
			timer_ = this->create_wall_timer(100ms, timer_callback);
		};

	private:
		rclcpp::TimerBase::SharedPtr timer_;
		std::atomic<uint64_t> timestamp_;

		rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
		rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
		rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

		rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
		rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_sub_;
		rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr desired_setpoint_subscriber_;

		px4_msgs::msg::VehicleOdometry curr_odom_;

		enum Mission {
		    FLIGHT,
		    LANDING,
		    FINISHED,
		};

		bool has_odom_ = false;
		bool armed_ = false;
		bool landed_ = false;

		int hold_counter_ = 0;
		const int HOLD_THRESHOLD = 20;

		void arm();
		void disarm();

		void publish_offboard_control_mode();
		void publish_trajectory_setpoint();
		void land();
		void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

		void publish_vehicle_command(int command, float value);

		int offboard_setpoint_counter_ = 0;

		float target_from_sub_x_ = 0.0f;
		float target_from_sub_y_ = 0.0f;
		float accurate_altitude_ = 0.0f;

		float low_enough_ = -0.5f; // m
		float descent_step_ = 0.1f; // *0.5~1.0 m/s (inaccurate-calculated heuristically)

		Mission mission_mode_ = FLIGHT;
		//Eigen::Vector3f targetNED{0, 0, 0};
};

void OffboardControl::arm() {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
	armed_ = true;
}

void OffboardControl::disarm() {
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
	armed_ = false;
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

//simplified for landing simulation
void OffboardControl::publish_trajectory_setpoint() {
	if (curr_odom_.timestamp == 0) {
		RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
		return;
	}

	TrajectorySetpoint msg {};

	Eigen::Vector3f current(curr_odom_.position[0], curr_odom_.position[1], curr_odom_.position[2]);
	Eigen::Vector3f target(0, 0, -10);
	Eigen::Vector3f to_wp = target - current;
	float dist_to_wp = to_wp.norm();

	RCLCPP_INFO(this->get_logger(), "[Multirotor] Distance to waypoint: %f",  dist_to_wp);

	if (dist_to_wp < 3.0f) {
		hold_counter_++;
		if (hold_counter_ > HOLD_THRESHOLD) {
			hold_counter_ = 0;
			mission_mode_ = LANDING;
		}
	}

	msg.position = {target[0], target[1], target[2]};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::land() {
	TrajectorySetpoint msg {};
	Eigen::Quaternionf q(curr_odom_.q[0], curr_odom_.q[1], curr_odom_.q[2], curr_odom_.q[3]); //current aircraft state for frd -> ned conversion
	q.normalize(); //abs(q) == 1

	//fake altitude value, delete this line to use lidar value(published as /tag_fused_point.z) for altitude
	//refer desired_setpoint_subscriber
	//accurate_altitude_ = curr_odom_.position[2];

	Eigen::Vector3f current(curr_odom_.position[0], curr_odom_.position[1], accurate_altitude_); //current coordinate with altitude replaced with lidar value
	float k = -accurate_altitude_; //(10 - accurate_altitude_)*0.5;
	Eigen::Vector3f targetFRD(0, 0, 0);
	if(target_from_sub_x_ != 0 || target_from_sub_y_ != 0) {
		targetFRD = {target_from_sub_y_*k, target_from_sub_x_*k, 0}; //target coordinate in frd frame from /tag_fused_point
		//targetNED = current + q * targetFRD; //convert frd coordinate to ned coordinate
	}
	Eigen::Vector3f targetNED = current + q * targetFRD; //convert frd coordinate to ned coordinate

	msg.position = {targetNED[0], targetNED[1], accurate_altitude_ + descent_step_};

	RCLCPP_INFO(this->get_logger(), "[Landing] current coordinate %.3f %.3f %.3f", curr_odom_.position[0], curr_odom_.position[1], curr_odom_.position[2]);
	RCLCPP_INFO(this->get_logger(), "[Landing] subbed FRD %.3f %.3f %.3f", targetFRD[0], targetFRD[1], targetFRD[2]);
	RCLCPP_INFO(this->get_logger(), "[Landing] calc'ed NED %.3f %.3f %.3f\n", targetNED[0], targetNED[1], targetNED[2]);

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
	if(accurate_altitude_ > low_enough_) {
		mission_mode_ = FINISHED;
		publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
		publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
		publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
		RCLCPP_INFO(this->get_logger(), "[Landing] low enough at altitude %.3f. sending land command", -accurate_altitude_);
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

int main(int argc, char *argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
