#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <fstream>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class DroneController: public rclcpp::Node 
{
    public:
        DroneController(): Node("drone_controller")
        {
            arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
            set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
            takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");
            land_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/land");
        }

        bool setMode(const std::string &mode)
        {
            auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            request->custom_mode = mode;
            auto result = set_mode_client_->async_send_request(request);
            // Return true if successful, false otherwise.
        }

        bool arm()
        {
            auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            request->value = true;
            auto result = arming_client_->async_send_request(request);
            // Return true if successful, false otherwise.
        }

        bool takeoff(float altitude)
        {
            auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
            request->altitude = altitude;
            auto result = takeoff_client_->async_send_request(request);
            // Return true if successful, false otherwise.
        }

        bool land()
        {
            auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
            auto result = land_client_->async_send_request(request);
            // Return true if successful, false otherwise.
        }

    private:
        rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
        rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
        rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
        rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        std::string filename_;

        void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
        {
            
            tf2::Matrix3x3 m(q);
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneController>());
    rclcpp::shutdown();
    return 0;
}