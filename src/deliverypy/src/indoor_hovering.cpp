#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class HAYAGOControl: public rclcpp::Node
{
    public:
        HAYAGOControl(): Node("hayago_control")
        {
            trajectory_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
            vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
            odometry_subscription_ = this->create_subscription<VehicleOdometry>(
                "/fmu/out/vehicle_odometry", 10, 
                std::bind(&HAYAGOControl::odometryCallback, this, std::placeholders::_1)
            );
            takeoff_setpoint_.position = {0.0, 0.0, -5.0};
            takeoff_setpoint_.yaw = 0.0;
            hover_setpoint_ = takeoff_setpoint_;
            landing_setpoint_.position = {0.0, 0.0, 0.0};
            landing_setpoint_.yaw = 0.0;
        }

        void odometryCallback(const VehicleOdometry::SharedPtr msg)
        {
            std::cout << "Position (x, y, z): " << msg->position[0] << ", " << msg->position[1] << ", " << msg->position[3] << std::endl;

            double roll, pitch, yaw;
            tf2::Quaternion q(
                msg->q[0],
                msg->q[1],
                msg->q[2],
                msg->q[3]
            );
            tf2::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);
            std::cout << "Orientation (roll, pitch, yaw): " << roll << ", " << pitch << ", " << yaw << std::endl;
        }

        void arm() 
        {
            sendVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        }

        void takeoff()
        {
            trajectory_publisher_->publish(takeoff_setpoint_);
        }

        void hover()
        {
            trajectory_publisher_->publish(hover_setpoint_);
        }

        void land() 
        {
            trajectory_publisher_->publish(landing_setpoint_);
        }

        void setModeMANUAL() 
        {
            sendVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 1);
        }
        
        void setModeGUIDED()
        {
            sendVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4);
        }

        void setModeMANUAL() 
        {
            sendVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 5);
        }

        void setModeSTABILIZED()
        {
            sendVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 8);
        }

        void setModeTAKEOFF()
        {
            sendVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 10);
        }

        void setModeLAND()
        {
            sendVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 11);
        }

        void setModeRTL()
        {
            sendVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 12);
        }
        
        void setModeRTGS()
        {
            sendVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 13);
        }

    private:
        TrajectorySetpoint takeoff_setpoint_;
        TrajectorySetpoint hover_setpoint_;
        TrajectorySetpoint landing_setpoint_;
        rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_publisher_;
        rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
        rclcpp::Subscription<VehicleOdometry>::SharedPtr odometry_subscription_;

        void sendVehicleCommand(uint16_t command, float param1, float param2 = 0.0)
        {
            VehicleCommand msg;
            msg.param1 = param1;
            msg.param2 = param2;
            msg.command = command;
            msg.target_system = 1;
            msg.target_component = 1;
            msg.source_system = 1;
            msg.source_component = 1;
            msg.from_external = true;
            vehicle_command_publisher_->publish(msg);
        }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HAYAGOControl>();

    node->setModeGUIDED();
    node->arm();
    node->takeoff();
    std::this_thread::sleep_for(5s);
    node->hover();
    std::this_thread::sleep_for(5s);
    node->land();
    node->setModeLAND();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}