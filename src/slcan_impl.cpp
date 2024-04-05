#include "slcan_cpp/slcan.hpp"

#include <chrono>

using namespace std::chrono_literals;

slcan_node::slcan_node() : rclcpp::Node("slcan_node")
{
	auto param_slcan_topic_name = rcl_interfaces::msg::ParameterDescriptor{};
	this->declare_parameter("slcan_topic_name", "slcan", param_slcan_topic_name);
	auto topic_name = get_parameter("slcan_topic_name").as_string();

	m_sub_can = this->create_subscription<can_msgs::msg::CanMsg>(
		topic_name, 10, std::bind(&slcan_node::sub_callback, this, std::placeholders::_1));
	m_pub_data = this->create_publisher<can_msgs::msg::CanMsg>("slcan_recv", 10);
	m_recv_timer = this->create_wall_timer(
		1ms, std::bind(&slcan_node::recv_timer_callback, this));
	m_uart_fail_pub = this->create_publisher<std_msgs::msg::Bool>("uart_fail", 10);

	auto is_open = this->open_serial_port();

	if(not is_open)
		return;

	RCLCPP_INFO(this->get_logger(), "config ok");

	::tcdrain(m_fd);
	send("O\r");
}

slcan_node::~slcan_node()
{
	::tcdrain(m_fd);
	send("C\r");

	this->close_serial_port();
}

bool slcan_node::open_serial_port()
{
	auto param_serial_name = rcl_interfaces::msg::ParameterDescriptor{};
	auto param_baudrate    = rcl_interfaces::msg::ParameterDescriptor{};
	param_serial_name.description = "";
	this->declare_parameter("serial_name", "/dev/ttyACM0", param_serial_name);
	this->declare_parameter("baudrate", 115200, param_baudrate);

	auto device = get_parameter("serial_name").as_string();
	auto baudrate = get_parameter("baudrate").as_int();

	RCLCPP_INFO(this->get_logger(), "device  : %s", device.c_str());
	RCLCPP_INFO(this->get_logger(), "baudrate: %ld", baudrate);

	bool is_same_rate = false;
	for(const auto& rate: baudrate_list) 
	{
		if (baudrate == rate) 
		{
			is_same_rate = true;
			break;
		}
	}

	if(!is_same_rate) 
	{
		RCLCPP_ERROR(this->get_logger(), "invalid baudrate: %ld", baudrate);
		return false;
	}

	// シリアルポートをオープン
	m_fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
	if (m_fd < 0) 
	{
		RCLCPP_ERROR(this->get_logger(), "can't open %s", device.c_str());
		return false;
	}

	// シリアル通信の設定
	struct termios serial;
	tcgetattr(m_fd, &serial);
	if (-1 == tcgetattr(m_fd, &serial)) 
	{
		close(m_fd);
		RCLCPP_ERROR(this->get_logger(), "tcgetattr error!" );
		return false;
	}

	const int UART_BAUDRATE = baudrate_map.at(baudrate);
	struct termios term;

	if (-1 == tcgetattr(m_fd, &term)) 
	{
		close(m_fd);
		RCLCPP_ERROR(this->get_logger(), "tcsetattr error!" );
		return false;
	}

	cfsetospeed(&term, UART_BAUDRATE);
	cfsetispeed(&term, UART_BAUDRATE);
	term.c_cflag &= ~CSIZE;
	term.c_cflag |= CS8;
	term.c_cflag &= ~CRTSCTS;
	term.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	term.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR);
	term.c_oflag &= ~OPOST;
	term.c_cc[VMIN] = 0;
	term.c_cc[VTIME]= 0;

	if (-1 == tcsetattr(m_fd, TCSANOW, &term)) 
	{
		close(m_fd);
		RCLCPP_ERROR(this->get_logger(), "tcsetattr error!" );
		return false;
	}
	return true;
}

void slcan_node::close_serial_port()
{
	tcdrain(m_fd);

	if(m_fd < 0)
		close(m_fd);
}

void slcan_node::send(const std::string& data)
{
	tcdrain(m_fd);
	
	int ret = ::write(m_fd, data.c_str(), data.size());
#if SHOW_SENDING_MSG
	RCLCPP_INFO(this->get_logger(), "send: %s", data.c_str());
#endif // SHOW_SENDING
}

void slcan_node::recv_timer_callback()
{
	if(m_fd < 0) 
	{
	#if SHOW_SENDING_ERROR
		RCLCPP_ERROR(this->get_logger(), "serial port is not open");
	#endif // SHOW_SENDING_ERROR

		uart_fail_publish();
		return ;
	}
	
	constexpr size_t buffer_size = 19;
	char buff[buffer_size];

	int n = read(m_fd, buff, buffer_size);
	if(n > 0)
	{
		std::string recv_data(buff, n);

		auto msg = decode_data(recv_data);

		msg.header.stamp = this->get_clock()->now();

		msg.header.frame_id = "slcan";

		m_pub_data->publish(msg);
	}
}

void slcan_node::uart_fail_publish()
{
	std_msgs::msg::Bool msg;
	msg.data = true;
	m_uart_fail_pub->publish(msg);
}

std::string slcan_node::encode_data(const can_msgs::msg::CanMsg &msg)
{
	std::string ret;

	if(msg.is_remote)
	{
		if(msg.is_extended)
			ret.push_back('R');
		else
			ret.push_back('r');
		
	}
	else
	{
		if(msg.is_extended) 
			ret.push_back('T');
		else
			ret.push_back('t');
	}
	char id[8];

	sprintf(id, "%08x", msg.id);
	ret.append(id, 8);

	auto dlc = std::to_string(msg.data.size());
	ret += dlc;

	for(const auto& s : msg.data) 
	{
		char tmp[2];
		sprintf(tmp, "%02x", s);
		ret.append(tmp, 2);
	}

	ret += "\r";
	
	return ret;
}

can_msgs::msg::CanMsg slcan_node::decode_data(const std::string &data) 
{
	bool is_remote;
	bool is_extended;

	switch(data[0])
	{
	case 'T':
	{
		is_remote = false;
		is_extended = true;
		break;
	}

	case 't':
	{
		is_remote = false;
		is_extended = false;
		break;
	}

	case 'R':
	{
		is_remote = true;
		is_extended = true;
		break;
	}

	case 'r':
	{
		is_remote = true;
		is_extended = false;
		break;
	}
	default:
	{
		RCLCPP_ERROR(this->get_logger(), "invalid command");
		return can_msgs::msg::CanMsg();
	}
	}

	can_msgs::msg::CanMsg msg;

	if(is_extended)
	{
		uint dlc = 8;
		msg.is_extended = true;
		msg.id = std::stoul(data.substr(1, dlc), nullptr, 16);
	}
	else
	{
		uint dlc = 3;
		msg.is_extended = false;
		msg.id = std::stoul(data.substr(1, dlc), nullptr, 16);
	}

	if (not is_remote)
	{
		msg.is_remote = false;
		uint dlc_place = is_extended ?  1+8 : 1+3;
		auto dlc = stoi(data.substr(dlc_place, 1), nullptr, 16);
		msg.data.resize(dlc);
		for (unsigned long i = 0; i < msg.data.size(); i++)
		{
			msg.data[i] = stoi(data.substr(dlc_place+1+2*i, 2), nullptr, 16);
		}
	}

	return msg;
}

void slcan_node::sub_callback(const can_msgs::msg::CanMsg::SharedPtr msg) 
{
	auto data = encode_data(*msg);

	send(data);

	auto data_msg = decode_data(data);

	if(msg->data.size() == 4)
	{
		uint32_t data = 0;
		for(uint32_t i = 0; i < msg->data.size(); i++)
		{
			data |= data_msg.data[i] << (8*i);
		}
		auto f = std::bit_cast<float>(data);

	#if SHOW_SUBSCRIPTION
		RCLCPP_INFO(this->get_logger(), "msg: {id: %x ,data: %f}", data_msg.id, f);
	#endif // SHOW_SUBSCRIPTION
	}
	
}
