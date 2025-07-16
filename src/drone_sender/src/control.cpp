#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include <chrono>

using namespace std::chrono_literals;

class TeleopDrone : public rclcpp::Node
{
public:
TeleopDrone() : Node("teleop_drone")
{
cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
"/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");  
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");  
    takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");  
    land_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");  

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(  
        "/mavros/state", 10, std::bind(&TeleopDrone::stateCallback, this, std::placeholders::_1));  

    cmd_sub_ = this->create_subscription<std_msgs::msg::String>(  
        "/teleop_cmd", 10, std::bind(&TeleopDrone::commandCallback, this, std::placeholders::_1));  

    RCLCPP_INFO(this->get_logger(), "Teleop Drone Node Started!");  
}

private:
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;

mavros_msgs::msg::State current_state_;  

void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)  
{  
    current_state_ = *msg;  
}  

void commandCallback(const std_msgs::msg::String::SharedPtr msg)  
{  
    if (msg->data.empty()) return;  
    char key = toupper(msg->data[0]);  

    if (key == 'M') {  
        setGuidedMode();  
    } else if (key == 'T') {  
        armAndTakeoff();  
    } else if (key == 'L') {  
        land();  
    } else if (key == 'Z') {  
        disarm();  
    } else if (key == 'W') {  
        publishVelocity(0.0, -1.0, 0.0, 0.0);  // Maju  
    } else if (key == 'E') {  
        publishVelocity(0.0, 0.0, 0.0, 0.5);  // Stop  
    }else if (key == 'A') {  
        publishVelocity(1.0, 0.0, 0.0, 0.0);  //  
    } else if (key == 'F') {
        publishVelocity(0.0, 0.0, -0.2, 0.0);
    } else if (key == 'R') {
        publishVelocity(0.0, 0.0, 0.2, 0.0);  // Naik
    } else if (key == 'D') {
        publishVelocity(-1.0, 0.0, 0.0, 0.0);  // Mundur
    } else {  
        RCLCPP_WARN(this->get_logger(), "Unknown command: %c", key);  
    }  
}  

void publishVelocity(double x, double y, double z, double yaw = 0.0)  
{  
    geometry_msgs::msg::Twist vel_msg;  
    vel_msg.linear.x = x;  
    vel_msg.linear.y = y;  
    vel_msg.linear.z = z;
    vel_msg.angular.z = yaw;
    cmd_vel_pub_->publish(vel_msg);  
    RCLCPP_INFO(this->get_logger(), "Publishing velocity x:%.2f y:%.2f z:%.2f", x, y, z);  
}  

void setGuidedMode()  
{  
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();  
    request->custom_mode = "GUIDED";  

    while (!set_mode_client_->wait_for_service(1s)) {  
        RCLCPP_WARN(this->get_logger(), "Waiting for /set_mode service...");  
    }  

    auto result = set_mode_client_->async_send_request(request);  
    RCLCPP_INFO(this->get_logger(), "GUIDED mode request sent.");  
}  

void armAndTakeoff()  
{  
    auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();  
    arm_request->value = true;  

    while (!arming_client_->wait_for_service(1s)) {  
        RCLCPP_WARN(this->get_logger(), "Waiting for arming service...");  
    }  

    arming_client_->async_send_request(arm_request);  
    RCLCPP_INFO(this->get_logger(), "Arming request sent.");  

    rclcpp::sleep_for(1s);  

    auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();  
    takeoff_request->altitude = 1.0;  

    while (!takeoff_client_->wait_for_service(1s)) {  
        RCLCPP_WARN(this->get_logger(), "Waiting for takeoff service...");  
    }  

    takeoff_client_->async_send_request(takeoff_request);  
    RCLCPP_INFO(this->get_logger(), "Takeoff request sent.");  
}  

void land()  
{  
    auto land_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();  

    while (!land_client_->wait_for_service(1s)) {  
        RCLCPP_WARN(this->get_logger(), "Waiting for land service...");  
    }  

    land_client_->async_send_request(land_request);  
    RCLCPP_INFO(this->get_logger(), "Land request sent.");  
}  

void disarm()  
{  
    auto disarm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();  
    disarm_request->value = false;  

    while (!arming_client_->wait_for_service(1s)) {  
        RCLCPP_WARN(this->get_logger(), "Waiting for disarm service...");  
    }  

    arming_client_->async_send_request(disarm_request);  
    RCLCPP_INFO(this->get_logger(), "Disarm request sent.");  
}

};

int main(int argc, char **argv)
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<TeleopDrone>());
rclcpp::shutdown();
return 0;
}
