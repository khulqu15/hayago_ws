#include <fstream>
#include <vector>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class DroneController: public rclcpp::Node 
{
    public:
        struct PoseData {
            double x, y, z;
            double roll, pitch, yaw;
        };
        
        DroneController(): Node("drone_controller")
        {
            arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
            set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
            takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");
            land_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/land");
            pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("mavros/local_position/pose", 10, std::bind(&DroneController::poseCallback, this, std::placeholders::_1));
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
            if(result.get()->success)
            {
                RCLCPP_INFO(this->get_logger(), "ARMING!");
                return true;
            } 
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Arming failed: %s", result.get()->result_text.c_str());
                return false;
            } 
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
            saveToCSV();
            // Return true if successful, false otherwise.
        }

    private:
        rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
        rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
        rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
        rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        std::vector<PoseData> pose_data_;

        void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            PoseData data;
            data.x = msg->pose.position.x;
            data.y = msg->pose.position.y;
            data.z = msg->pose.position.z;
            tf2::Quaternion q(
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z,
                msg->pose.orientation.w
            );
            tf2::Matrix3x3 m(q);
            m.getRPY(data.roll, data.pitch, data.yaw);
            pose_data_.push_back(data);
        }

        void saveToCSV()
        {
            auto now = std::chrono::system_clock::now();
            std::time_t end_time = std::chrono::system_clock::to_time_t(now);
            std::string filename = std::to_string(end_time) + "_flight.csv";
            std::ofstream file(filename);
            if(file.is_open())
            {
                file << "x,y,z,roll,pitch,yaw\n";
                for(const auto &data: pose_data_)
                    file << data.x << "," << data.y << "," << data.z << "," << data.roll << "," << data.pitch << "," << data.yaw << "\n";
                file.close();
            } else
                RCLCPP_ERROR(this->get_logger(), "CSV FILE CANNOT OPEN!");
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneController>());
    rclcpp::shutdown();
    return 0;
}