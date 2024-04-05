#include "slcan_cpp/slcan.hpp"

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<slcan_node>());
	rclcpp::shutdown();
	return 0;
}
