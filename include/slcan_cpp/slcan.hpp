#pragma once

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <bit>
#include <bitset>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>

#include "can_msgs/msg/can_msg.hpp"

#include "slcan_cpp/map_data.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>

#include <thread>
#include <queue>

void Write(const int fd);

class slcan_node : public rclcpp::Node
{
	int m_fd;
	rclcpp::Subscription<can_msgs::msg::CanMsg>::SharedPtr m_sub_can;
	rclcpp::Publisher<can_msgs::msg::CanMsg>::SharedPtr m_pub_data;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_uart_fail_pub;
	rclcpp::TimerBase::SharedPtr m_recv_timer;

	std::thread m_write;

	bool open_serial_port();

	void close_serial_port();

	void send(const std::string &data);

	void reopen_sub_callback(std_msgs::msg::Bool::SharedPtr msg);

	void recv_timer_callback();

	void uart_fail_publish();

	std::string encode_data(const can_msgs::msg::CanMsg &msg);

	can_msgs::msg::CanMsg decode_data(const std::string &data);

	void sub_callback(can_msgs::msg::CanMsg::SharedPtr msg);
	
public:
	slcan_node();
	~slcan_node();

};

