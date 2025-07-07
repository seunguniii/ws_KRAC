// 기본 C++ 라이브러리 및 Eigen 수학 라이브러리 포함
#include <iostream>
#include <chrono>
#include <vector>
#include <stdint.h>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ROS2 기본 헤더
#include <rclcpp/rclcpp.hpp>

// PX4 메시지 인터페이스
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/goto_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

// 클래스 정의: 오프보드 제어 노드
class OffboardControl : public rclcpp::Node {
public:
	// 생성자: 구독, 발행자, 타이머 정의
	OffboardControl() : Node("test") {
		// 차량 위치(odometry) 구독
		odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
			"/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
			[this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
				curr_odom_ = *msg;
				has_odom_ = true;
			}
		);

		// 퍼블리셔 정의
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		goto_setpoint_publisher_ = this->create_publisher<GotoSetpoint>("/fmu/in/goto_setpoint", 10);

		// 주기적으로 실행되는 타이머 콜백 (100ms 주기)
		auto timer_callback = [this]() -> void {
			if (!has_odom_) {
				RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
				return;
			}

			if (!armed_) {
				// 오프보드 모드 전환 및 arm
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);  // custom mode: PX4 manual
				this->arm();
			}

			// 오프보드 제어 메시지 및 경로 목표 전송
			publish_offboard_control_mode();
			publish_trajectory_setpoint();

			offboard_setpoint_counter_++;
		};

		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

private:
	// ROS2 타이머
	rclcpp::TimerBase::SharedPtr timer_;
	std::atomic<uint64_t> timestamp_; // 사용되지 않음

	// 퍼블리셔
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<GotoSetpoint>::SharedPtr goto_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	// 오도메트리 구독
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
	px4_msgs::msg::VehicleOdometry curr_odom_;  // 현재 위치 정보 저장

	// VTOL 모드 정의
	enum FlightMode {
		MULTIROTOR = 3,
		FIXED_WING = 4
	};
	FlightMode flight_mode_ = MULTIROTOR;  // 기본은 멀티콥터

	// 비행 경로 (웨이포인트 리스트)
	std::vector<std::array<float, 3>> waypoints_ = {
		{0.0f, 0.0f, -25.0f},
		{0.0f, 0.0f, -27.0f}, // 천이 시 고도 하강 대비 고도 약간 상승
		{100.0f, 0.0f, -25.0f},
		{200.0f, 100.0f, -25.0f},
		{200.0f, -100.0f, -25.0f},
		{100.0f, 0.0f, -25.0f}, 
		{0.0f, -100.0f, -25.0f},
		{0.0f, 0.0f, -25.0f},
		{0.0f, 0.0f, 0.0f},  // 착륙 지점
	};

	uint64_t offboard_setpoint_counter_{0};  // 반복 제어용 카운터
	size_t wp_idx_{0};  // 현재 웨이포인트 인덱스

	// 상태 변수
	bool has_odom_ = false;
	bool armed_ = false;
	int hold_counter_ = 0;
	const int HOLD_THRESHOLD = 20;

	// 내부 함수 선언
	void arm();
	void disarm();
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void rescue_mission();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void transition(FlightMode mode = MULTIROTOR);
	float k = 1;  // 속도 계수 (고정익일 때 사용)
};

// ARM 명령 전송
void OffboardControl::arm() {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command sent");
	armed_ = true;
}

// DISARM 명령 전송
void OffboardControl::disarm() {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command sent");
	armed_ = false;
}

// 오프보드 제어 모드 메시지 전송
void OffboardControl::publish_offboard_control_mode() {
	OffboardControlMode msg{};
	msg.position = true;  // 위치 기반 제어 허용
	msg.velocity = flight_mode_ == MULTIROTOR ? false : true;  // 고정익은 속도 기반
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

// 위치 목표 또는 속도 목표 전송
void OffboardControl::publish_trajectory_setpoint() {
	if (curr_odom_.timestamp == 0) {
		RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
		return;
	}

	TrajectorySetpoint msg_fw{};
	auto &wp = waypoints_[wp_idx_];

	// 현재 위치 및 타겟 위치 벡터 계산
	Eigen::Vector3f current(curr_odom_.position[0], curr_odom_.position[1], curr_odom_.position[2]);
	Eigen::Vector3f target(wp[0], wp[1], wp[2]);
	Eigen::Vector3f to_wp = target - current;
	float dist_to_wp = to_wp.norm();

	// 방향 벡터 정규화
	Eigen::Vector3f direction_v = to_wp.normalized();

	if (flight_mode_ == MULTIROTOR && armed_) {
		RCLCPP_INFO(this->get_logger(), "[Multirotor] Distance to waypoint %ld: %f", wp_idx_, dist_to_wp);
		msg_fw.position = {wp[0], wp[1], wp[2]};
	
		if (dist_to_wp < 3.0f) {
			hold_counter_++;
	
			if (hold_counter_ > HOLD_THRESHOLD) {
				hold_counter_ = 0;
	
				 // if (wp_idx_ >= waypoints_.size() - 1) {
					//this->disarm();  // 마지막이면 착륙
				//} 
	
				// wp1에서만 고정익으로 전환
				if (wp_idx_ == 1) {
					 transition(FIXED_WING);
				 }
	
				wp_idx_++;
				offboard_setpoint_counter_ = 0;
			}
		}
	}
	else if (flight_mode_ == FIXED_WING) {
		// 고정익 모드일 때: 속도 기반 제어
		RCLCPP_INFO(this->get_logger(), "[Fixed-wing] Heading to projected point. WP: %ld, Dist: %.2f", wp_idx_, dist_to_wp);
		msg_fw.position = {wp[0], wp[1], wp[2]};
		msg_fw.velocity = {k * direction_v.x(), k * direction_v.y(), 0};

		if (dist_to_wp < 10.0f) wp_idx_++;

		if (wp_idx_ == 5) {
			rescue_mission();  // 자동 구조 미션 수행
			return;  // rescue_mission 내부에서 trajectory_setpoint 보냄
		}


	}

	msg_fw.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg_fw);
}

// 구조 임무 수행 (멀티콥터 모드로 내려가서 착륙)
void OffboardControl::rescue_mission() {
	TrajectorySetpoint msg{};

	if (flight_mode_ == FIXED_WING) {
		transition(MULTIROTOR);  // 구조 시엔 회전익으로 전환
	}

	Eigen::Vector3f current(curr_odom_.position[0], curr_odom_.position[1], curr_odom_.position[2]);
	Eigen::Vector3f target(0.0f, 0.0f, -50.0f);
	Eigen::Vector3f to_wp = target - current;
	float dist_to_wp = to_wp.norm();

	msg.position = {target[0], target[1], target[2]};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);

	if (dist_to_wp < 0.3f) {
		RCLCPP_INFO(this->get_logger(), "Rescue mission complete. Initiating landing sequence.");
		// 여기다가 정밀착륙 코드 넣기
	}
}


// VehicleCommand 메시지 전송 함수
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2) {
	VehicleCommand msg{};
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

// 고정익 ↔ 회전익 모드 전환
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

// 메인 함수: ROS 2 노드 실행
int main(int argc, char *argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);  // 출력 버퍼링 제거
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());
	rclcpp::shutdown();
	return 0;
}
