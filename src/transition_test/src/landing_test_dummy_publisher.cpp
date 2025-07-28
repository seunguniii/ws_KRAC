#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

class CoordinatePublisher : public rclcpp::Node {
public:
	CoordinatePublisher(): Node("coordinate_publisher") {
		publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("tag_fused_point", 10);

		timer_ = this->create_wall_timer(
		             std::chrono::milliseconds(500),
		             std::bind(&CoordinatePublisher::publish_coordinates, this)
		         );
	}

private:
	void publish_coordinates() {
		geometry_msgs::msg::PointStamped point_msg;

		point_msg.point.x = 0;
		point_msg.point.y = 0;
		point_msg.point.z = -25;

		publisher_->publish(point_msg);
	}

	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CoordinatePublisher>());
	rclcpp::shutdown();
	return 0;
}

