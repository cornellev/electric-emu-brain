#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h>
#include <string>
#include "std_msgs/msg/float32.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "cev_msgs/msg/sensor_collect.hpp"
#include <iostream>

#define UART_DEV "/dev/ttyUSB0"
#define BAUD_RATE B115200

class SerialHandlerNode : public rclcpp::Node {
public:
    SerialHandlerNode(): Node("serial_handler_node") {
        // Initialize serial port
        try {
            serial_port_.setPort("/dev/ttyUSB0");
            serial_port_.setBaudrate(115200);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_port_.setTimeout(timeout);
            serial_port_.open();

            if (serial_port_.isOpen()) {
                RCLCPP_INFO(this->get_logger(), "Serial port initialized successfully.");
            } else {
                RCLCPP_INFO(this->get_logger(), "Failed to open serial port.");
            }
        } catch (const serial::IOException& e) {
            RCLCPP_INFO(this->get_logger(), "Serial port error: %s", e.what());
        }

        // Subscribers
        rc_movement_sub_ =
            this->create_subscription<ackermann_msgs::msg::AckermannDrive>("rc_movement_msg", 1,
                std::bind(&SerialHandlerNode::rcMovementCallback, this, std::placeholders::_1));

        // Publisher for sensor data
        sensor_collect_pub_ = this->create_publisher<cev_msgs::msg::SensorCollect>("sensor_collect",
            1);

        // Timer for reading serial data periodically
        comm_timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
            std::bind(&SerialHandlerNode::commSerialData, this));
    }

private:
    serial::Serial serial_port_;
    rclcpp::TimerBase::SharedPtr comm_timer_;

    // Data to publish to arduino
    float steering_angle_;
    float velocity_ = 10;
    float max_velocity_ = 20;

    // Parsed serial data from arduino
    int32_t reported_timestamp;
    float reported_steering_angle;

    // Subscribed topics to send over serial
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr rc_movement_sub_;

    // Publisher for sensor data to SensorCollect message
    rclcpp::Publisher<cev_msgs::msg::SensorCollect>::SharedPtr sensor_collect_pub_;

    // Callback for the rc_movement_msg topic
    void rcMovementCallback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
        steering_angle_ = msg->steering_angle;
    }

    // Read and write serial data periodically
    void commSerialData() {
        if (serial_port_.isOpen()) {
            try {
                // Read a line (until newline character)
                std::string message = serial_port_.readline();

                RCLCPP_INFO(this->get_logger(), "Raw message received: '%s'", message.c_str());
                if (!message.empty()) {
                    RCLCPP_INFO(this->get_logger(), "Raw message received: '%s'", message.c_str());

                    // Parse the received message into reported sensor data
                    if (parseMessage(message)) {
                        RCLCPP_INFO(this->get_logger(),
                            "Parsed values - Int: %d, Float1: %f, Float2: %f", reported_timestamp,
                            0.0, reported_steering_angle);

                        // Publish the parsed data
                        publishSensorData();
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Failed to parse message: '%s'",
                            message.c_str());
                    }
                } else {
                    RCLCPP_INFO(this->get_logger(), "Message empty.");
                }
            } catch (const serial::IOException& e) {
                RCLCPP_INFO(this->get_logger(), "Serial read error: %s", e.what());
            }

            try {
                // Use ostringstream to control the precision of the float values
                std::ostringstream serial_message;
                // serial_message << std::fixed << std::setprecision(2) << velocity_ << " "
                            //    << steering_angle_ << " " << max_velocity_ << "\n";
                serial_message << steering_angle_ << "\n";

                // Send the message over the serial port
                serial_port_.write(serial_message.str());
                RCLCPP_INFO(this->get_logger(), "Serial message sent: '%s'",
                    serial_message.str().c_str());

            } catch (const serial::IOException& e) {
                RCLCPP_ERROR(this->get_logger(), "Serial write error: %s", e.what());
            }
        }
    }

    // Helper function to parse incoming serial message
    bool parseMessage(const std::string& message) {
        // Split by spaces
        size_t first_space = message.find(' ');
        size_t second_space = message.find(' ', first_space + 1);

        if (first_space == std::string::npos || second_space == std::string::npos) {
            return false;  // Parsing failed
        }

        try {
            reported_timestamp = std::stoi(message.substr(0, first_space));
            reported_steering_angle = std::stof(message.substr(second_space + 1));
            return true;
        } catch (const std::exception& e) {
            return false;  // Parsing failed
        }
    }

    // Function to publish sensor data
    void publishSensorData() {
        auto sensor_msg = cev_msgs::msg::SensorCollect();

        auto current_time = this->now().nanoseconds();
        double current_time_seconds = current_time / 1e9;
        // std::cout << current_time << std::endl;

        sensor_msg.timestamp = current_time_seconds;
        // sensor_msg.timestamp = this->now().nanoseconds() / 1e9;
        // sensor_msg.velocity = reported_velocity;
        sensor_msg.steering_angle = reported_steering_angle;

        sensor_collect_pub_->publish(sensor_msg);  // Publish the message
        RCLCPP_INFO(this->get_logger(),
            "Published sensor data: Timestamp: %f, Velocity: %f, Steering Angle: %f",
            sensor_msg.timestamp, sensor_msg.velocity, sensor_msg.steering_angle);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialHandlerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}