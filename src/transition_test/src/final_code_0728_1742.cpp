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
#include "px4_msgs/msg/vehicle_control_mode.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node {
	public:
		OffboardControl() : Node("test") {
			//three subscribers
			odom_sub_ = this->create_subscription<VehicleOdometry>(
			    "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
				[this](const VehicleOdometry::SharedPtr msg) {
				curr_odom_ = *msg;
				has_odom_ = true;
			});

			desired_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
			    "/tag_fused_point", 10,
				[this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
				target_from_sub_x_ = msg->point.x;
				target_from_sub_y_ = msg->point.y;
				accurate_altitude_ = msg->point.z;
			});

			land_sub_ = this->create_subscription<VehicleLandDetected>(
			    "/fmu/out/vehicle_land_detected", rclcpp::SensorDataQoS(),
			[	this](const VehicleLandDetected::SharedPtr msg) {
				landed_ = msg->landed;
			});

			//three publishers
			offboard_control_mode_publisher_ =
			    this->create_publisher<OffboardControlMode>(
			    "/fmu/in/offboard_control_mode", 10);

			trajectory_setpoint_publisher_ =
			    this->create_publisher<TrajectorySetpoint>(
			    "/fmu/in/trajectory_setpoint", 10);

			vehicle_command_publisher_ =
			    this->create_publisher<VehicleCommand>(
			    "/fmu/in/vehicle_command", 10);

			timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));
		}
	//public end

	private:
		rclcpp::TimerBase::SharedPtr timer_;

		//three publishers
		rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
		rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
		rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

		//three subscribers
		rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
		rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr desired_point_sub_;
		rclcpp::Subscription<VehicleLandDetected>::SharedPtr land_sub_;

		VehicleOdometry curr_odom_;

		//mission & aircraft states
		enum FlightMode {
		    MULTIROTOR = 3,
		    FIXED_WING = 4
		};

		enum Mission {
		    FLIGHT,
		    RESCUE,
		    FINISHED
		};

		FlightMode flight_mode_ = MULTIROTOR;
		Mission mission_mode_ = FLIGHT;

		bool has_odom_ = false;
		bool landed_detected_ = false;
		bool armed_ = false;
		bool landed_ = false;

		//auto-flight
		std::vector<std::array<float,3>> waypoints_ = {
			{0,0,-25},
			{100,0,-25},
			{200,100,-25},
			{200,-100,-25},
			{100,0,-25},
			{50,-50,-25}
		};

		size_t wp_idx_ = 0;
		int hold_counter_ = 0;
		const int HOLD_THRESHOLD = 20;	// 100ms * 20 = 2s

		float k = 1.0f; //velocity constant for fixed_wing control
		
		//sub'ed coordinates for aircraft guidance
		float target_from_sub_x_ = 0.0f;
		float target_from_sub_y_ = 0.0f;
		float accurate_altitude_ = 0.0f;

		//threshold for landing
		float low_enough_ = -0.5f; // m
		float descent_step_ = 3.0f; // *5 m/s (inaccurate-calculated heuristically)
		
		void publish_vehicle_command(uint16_t cmd, float p1 = 0.0f, float p2 = 0.0f) {
			VehicleCommand m {};
			m.command         = cmd;
			m.param1          = p1;
			m.param2          = p2;
			m.target_system   = 1;
			m.target_component= 1;
			m.from_external   = true;
			m.timestamp       = this->get_clock()->now().nanoseconds() / 1000;
			vehicle_command_publisher_->publish(m);
		}

		void publish_offboard_control_mode() {
			OffboardControlMode msg {};
			msg.position     = true;
			msg.velocity     = (flight_mode_ == MULTIROTOR) ? false : true;
			msg.acceleration = false;
			msg.attitude     = false;
			msg.body_rate    = false;
			msg.timestamp    = this->get_clock()->now().nanoseconds() / 1000;
			offboard_control_mode_publisher_->publish(msg);
		}

		void arm() {
			publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
			RCLCPP_INFO(this->get_logger(), "Arm command sent");
			armed_ = true;
		}

		void disarm() {
			publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
			RCLCPP_INFO(this->get_logger(), "Disarm command sent");
			armed_ = false; //safer if change tag after checking odometry
		}

		void transition(FlightMode mode) {
			if (mode == flight_mode_) return;

			publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION, static_cast<float>(mode));
			flight_mode_ = mode;
		}

		void publish_trajectory_setpoint() {
			if (curr_odom_.timestamp == 0) return; //wait if no odometry

			TrajectorySetpoint msg {};

			auto &wp = waypoints_[wp_idx_];

			Eigen::Vector3f cur(curr_odom_.position[0], curr_odom_.position[1], curr_odom_.position[2]);
			Eigen::Vector3f tgt(wp[0], wp[1], wp[2]);
			Eigen::Vector3f dir = (tgt - cur).normalized();
			float dist = (tgt - cur).norm();

			if(armed_) {
				switch(flight_mode_) {
					case MULTIROTOR:
						RCLCPP_INFO(this->get_logger(),
						            "[Multirotor] WP: %zu, Dist: %.2f", wp_idx_, dist);

						msg.position = {wp[0], wp[1], wp[2]};

						if (dist < 3.0f && hold_counter_ > HOLD_THRESHOLD) {
							hold_counter_ = 0;
							wp_idx_++;
							transition(FIXED_WING);
						}
						break;

					case FIXED_WING:
						RCLCPP_INFO(this->get_logger(),
						            "[Fixed-wing] WP: %zu, Dist: %.2f", wp_idx_, dist);

						msg.position = {wp[0], wp[1], wp[2]};
						msg.velocity = {k * dir.x(), k * dir.y(), 0.0f};

						if (dist < 10.0f) wp_idx_++; //decrease threshold for accuracy

						if (wp_idx_ >= waypoints_.size()) {
							transition(MULTIROTOR);
							mission_mode_ = RESCUE;
							hold_counter_ = 0;
						}
						break;
				}
			}
			
			msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
			trajectory_setpoint_publisher_->publish(msg);
		}

		void rescue_and_return() {
			TrajectorySetpoint msg {};
			Eigen::Quaternionf q(curr_odom_.q[0], curr_odom_.q[1], curr_odom_.q[2], curr_odom_.q[3]); //current aircraft state for frd -> ned conversion
			q.normalize(); //abs(q) == 1

			//fake altitude value, delete this line to use lidar value(published as /tag_fused_point.z) for altitude
			//refer desired_setpoint_subscriber
			accurate_altitude_ = curr_odom_.position[2];

			Eigen::Vector3f current(curr_odom_.position[0], curr_odom_.position[1], accurate_altitude_); //current coordinate with altitude replaced with lidar value
			Eigen::Vector3f targetFRD(target_from_sub_x_, target_from_sub_y_, 0); //target coordinate in frd frame from /tag_fused_point
			Eigen::Vector3f targetNED = current + q * targetFRD; //convert frd coordinate to ned coordinate

			msg.position = {targetNED[0], targetNED[1], accurate_altitude_ + descent_step_};

			msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
			trajectory_setpoint_publisher_->publish(msg);
			if(accurate_altitude_ > low_enough_) {
				mission_mode_ = FINISHED;
				publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
				RCLCPP_INFO(this->get_logger(), "[Landing] low enough at altitude %.3f. sending land command", -accurate_altitude_);
			}
		}

		void timer_callback() {
			if (!has_odom_) return;

			if (!armed_ && mission_mode_ != FINISHED) {
				publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				arm();
			}
			publish_offboard_control_mode();

			switch(mission_mode_) {
				default:
				case FLIGHT:
					publish_trajectory_setpoint();
					break;

				case RESCUE:
					rescue_and_return();
					break;

				case FINISHED:
					if(landed_) disarm();
					break;
			}
		}
	//private end
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());
	rclcpp::shutdown();
	return 0;
}
