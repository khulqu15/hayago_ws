import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint

class DroneController(Node):

    def __init__(self):
        super().__init__('drone_controller')
        self.publisher_ = self.create_publisher(TrajectorySetpoint, '/TrajectorySetpoint_pub', 10)
        self.timer_period = 5  # detik
        self.timer = self.create_timer(self.timer_period, self.move_drone)
        self.state = 0

    def move_drone(self):
        msg = TrajectorySetpoint()

        if self.state == 0:
            msg.z = 1.0  # Takeoff 1 meter
        elif self.state == 1:
            msg.x = 0.3  # Maju ke depan 30 cm
        elif self.state == 2:
            msg.y = -0.3  # Ke samping kanan 30 cm (Asumsi y positif ke kiri)
        elif self.state == 3:
            msg.x = -0.3  # Ke belakang 30 cm
        elif self.state == 4:
            msg.y = 0.3  # Ke kiri 30 cm
        else:
            self.timer.cancel()  # Matikan timer setelah semua perintah

        self.publisher_.publish(msg)
        self.state += 1

def main(args=None):
    rclpy.init(args=args)
    drone_controller = DroneController()
    rclpy.spin(drone_controller)
    drone_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
