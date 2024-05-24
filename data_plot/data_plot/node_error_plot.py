import rclpy
from rclpy.node import Node
from self_interface.msg import ControlError
import matplotlib.pyplot as plt

# 订阅control_error数据
class node_error_plot(Node):
    def __init__(self):
        super().__init__('node_error_plot')
        self.subscription_ego_status = self.create_subscription(
            ControlError,
            'control_error',
            self.control_error_callback,
            10)
        self.data_control_error = {'lateral_err': [], 'yaw_err': [], 'dot_yaw_err': [], 'vel_err': [], 'time': []}
        self.start_time = None  # Initial time set to None
        self.count = 0
        self.fig, (self.ax1, self.ax2, self.ax3, self.ax4) = plt.subplots(4, 1)  # Create a figure with four subplots
        self.fig.tight_layout(pad=1.0)  # Adjust spacing between subplots
        plt.ion()  # Turn on interactive mode

    def control_error_callback(self, msg):
        if self.start_time is None:
            self.start_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9  # Get the first timestamp as the start time
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9 - self.start_time

        self.data_control_error['lateral_err'].append(msg.lateral_err)
        self.data_control_error['yaw_err'].append(msg.yaw_err)  
        self.data_control_error['dot_yaw_err'].append(msg.dot_yaw_err)
        self.data_control_error['vel_err'].append(msg.vel_err)
        self.data_control_error['time'].append(current_time)

        self.update_plot()
        self.count += 1

                
    def update_plot(self):
        self.ax1.clear()  # First subplot
        self.ax1.plot(self.data_control_error['time'], self.data_control_error['lateral_err'], label='lateral_err')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Value')
        self.ax1.set_title('lateral_err')
        self.ax1.legend()

        self.ax2.clear()  # Second subplot
        self.ax2.plot(self.data_control_error['time'], self.data_control_error['yaw_err'], label='yaw_err')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Value')
        self.ax2.set_title('yaw_err')
        self.ax2.legend()

        self.ax3.clear()  # Third subplot
        self.ax3.plot(self.data_control_error['time'], self.data_control_error['dot_yaw_err'], label='dot_yaw_err')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Value')
        self.ax3.set_title('dot_yaw_err')
        self.ax3.legend()

        self.ax4.clear()  
        self.ax4.plot(self.data_control_error['time'], self.data_control_error['vel_err'], label='vel_err')
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('Value')
        self.ax4.set_title('vel_err')
        self.ax4.legend()

        plt.pause(0.02)

def main(args=None):
    rclpy.init(args=args)
    data_subscriber = node_error_plot()
    rclpy.spin(data_subscriber)
    data_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
