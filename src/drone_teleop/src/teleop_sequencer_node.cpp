#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <vector>

using namespace std::chrono_literals;

class TeleopSequencer : public rclcpp::Node
{
public:
    TeleopSequencer() : Node("teleop_sequencer")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("/teleop_cmd", 10);
        RCLCPP_INFO(this->get_logger(), "Teleop Sequencer Node Started");

        timer_ = this->create_wall_timer(1s, std::bind(&TeleopSequencer::sendNextCommand, this));
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    size_t command_index_ = 0;
    std::vector<std::pair<char, std::chrono::seconds>> sequence_ = {
        {'M', 2s},   // Set mode GUIDED
        {'T', 6s},   // Arm and Takeoff
        {'W', 10s},  // Maju pertama
        {'E', 3s},   // Yaw ke kiri 90 derajat,   // Maju lagi
        {'A', 10s},  // Maju lagi (setelah rotasi)
        {'L', 0s}    // Landing
    };

    rclcpp::Time next_command_time_;
    bool waiting_ = false;

    void sendNextCommand()
    {
        if (command_index_ >= sequence_.size()) {
            timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "Finished all commands.");
            return;
        }

        auto now = this->now();

        if (!waiting_) {
            char cmd = sequence_[command_index_].first;
            auto delay = sequence_[command_index_].second;

            std_msgs::msg::String msg;
            msg.data = std::string(1, cmd);
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Sent command: %c", cmd);

            next_command_time_ = now + rclcpp::Duration(delay);
            waiting_ = true;
        } 
        else if (now >= next_command_time_) {
            waiting_ = false;
            command_index_++;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopSequencer>());
    rclcpp::shutdown();
    return 0;
}
