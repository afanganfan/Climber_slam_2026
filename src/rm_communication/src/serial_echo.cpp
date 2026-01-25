#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <chrono>

using namespace std::chrono_literals;

class SerialEchoNode : public rclcpp::Node {
public:
  SerialEchoNode()
  : Node("serial_echo") {
    this->declare_parameter<std::string>("port", "/dev/ttySLAM");
    this->declare_parameter<int>("baudrate", 115200);
    this->get_parameter("port", port_);
    this->get_parameter("baudrate", baudrate_);

    serial_init();
    timer_ = this->create_wall_timer(100ms, std::bind(&SerialEchoNode::timer_cb, this));
    RCLCPP_INFO(this->get_logger(), "serial_echo node started on %s @ %d", port_.c_str(), baudrate_);
  }

  ~SerialEchoNode() {
    if (serial_port_.isOpen()) serial_port_.close();
  }

private:
  serial::Serial serial_port_;
  std::string port_;
  int baudrate_;
  rclcpp::TimerBase::SharedPtr timer_;

  void serial_init() {
    try {
      serial_port_.setPort(port_);
      serial_port_.setBaudrate(baudrate_);
      serial::Timeout to = serial::Timeout::simpleTimeout(100);
      serial_port_.setTimeout(to);
      serial_port_.open();
      if (serial_port_.isOpen()) {
        RCLCPP_INFO(this->get_logger(), "Opened serial port %s@%d", port_.c_str(), baudrate_);
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to open serial port %s", port_.c_str());
      }
    } catch (serial::IOException &e) {
      RCLCPP_ERROR(this->get_logger(), "Serial exception: %s", e.what());
    }
  }

  void timer_cb() {
    if (!serial_port_.isOpen()) {
      // try to reopen
      serial_init();
      return;
    }

    try {
      size_t available = serial_port_.available();
      if (available == 0) return;
      std::vector<uint8_t> buf(available);
      size_t n = serial_port_.read(buf.data(), buf.size());
      if (n == 0) return;

      // print raw bytes as hex and ascii
      std::ostringstream hexs;
      std::ostringstream ascii;
      for (size_t i = 0; i < n; ++i) {
        hexs << std::hex << std::setw(2) << std::setfill('0') << (int)buf[i] << " ";
        char c = static_cast<char>(buf[i]);
        if (std::isprint(static_cast<unsigned char>(c))) ascii << c;
        else ascii << '.';
      }

      RCLCPP_INFO(this->get_logger(), "Serial RX (%zu bytes): HEX=[%s] ASCII=[%s]", n, hexs.str().c_str(), ascii.str().c_str());
    } catch (serial::IOException &e) {
      RCLCPP_ERROR(this->get_logger(), "Read error: %s", e.what());
      if (serial_port_.isOpen()) serial_port_.close();
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialEchoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
