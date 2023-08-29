#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

class DroneController: public rclcpp::Node 
{
    public:
        DroneController(): Node("drone_controller")
        {

        }
    private:
        rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
        rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
        rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
        rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
};