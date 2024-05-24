import rclpy
from rclpy.node import Node
from self_interface.msg import EgoStatus
from self_interface.msg import ChassisCmd  # Import ChassisCmd message
import matplotlib.pyplot as plt
from math import degrees

# 订阅ego_status_msg数据，chassis_cmd数据
class node_data_plot(Node):
    def __init__(self):
        super().__init__('node_data_plot')
        self.subscription_ego_status = self.create_subscription(
            EgoStatus,
            'ego_status_msg',
            self.ego_status_callback,
            10)
        self.subscription_chassis = self.create_subscription(
            ChassisCmd,
            'chassis_cmd',
            self.chassis_callback,
            10)
        self.data_ego_status = {'act_velocity': [], 'ang_velocity': [], 'act_curvature': [], 'act_Yaw': [], 'time': []}
        self.data_chassis = {'cmd_velocity': [], 'cmd_curvature': [], 'cmd_center_steer_req': [], 'time': []}
        self.start_time = None  # Initial time set to None
        self.count = 0
        self.fig, (self.ax1, self.ax2, self.ax3, self.ax4) = plt.subplots(4, 1)  # Create a figure with four subplots
        self.fig.tight_layout(pad=1.0)  # Adjust spacing between subplots
        plt.ion()  # Turn on interactive mode
        self.steer_mod_req = None  # Initialize steer_mod_req state

    def ego_status_callback(self, msg):
        if self.start_time is None:
            self.start_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9  # Get the first timestamp as the start time
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9 - self.start_time

        self.data_ego_status['act_velocity'].append(msg.ego_velocity)
        self.data_ego_status['ang_velocity'].append(msg.ego_w)  
        self.data_ego_status['act_Yaw'].append(degrees(msg.ego_theta))
        self.data_ego_status['act_curvature'].append(msg.ego_curvature)
        self.data_ego_status['time'].append(current_time)

        if self.steer_mod_req == 1:
            self.update_plot1()
        elif self.steer_mod_req == 0:
            self.update_plot2()
        self.count += 1

    def chassis_callback(self, msg):
        if self.start_time is None:
            self.start_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9  # Get the first timestamp as the start time
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9 - self.start_time

        self.data_chassis['cmd_velocity'].append(msg.vel_req)
        self.data_chassis['cmd_curvature'].append(1 / msg.turn_radius_req)
        if msg.steer_mod_req == 0:
            center_steer_req = msg.center_steer_req
        else:
            center_steer_req = 0
        self.data_chassis['cmd_center_steer_req'].append(center_steer_req)
        self.data_chassis['time'].append(current_time)

        self.steer_mod_req = msg.steer_mod_req  # Update steer_mod_req state

        self.count += 1

                
    def update_plot1(self):
        self.ax4.clear()

        self.ax1.clear()  # First subplot
        self.ax1.plot(self.data_ego_status['time'], self.data_ego_status['act_velocity'], label='act_velocity')
        self.ax1.plot(self.data_chassis['time'], self.data_chassis['cmd_velocity'], label='cmd_velocity', linestyle='--')  # Dashed line
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Value')
        self.ax1.set_title('cmd_velocity & act_velocity')
        self.ax1.legend()

        self.ax2.clear()  # Second subplot
        self.ax2.plot(self.data_ego_status['time'], self.data_ego_status['ang_velocity'], label='ang_velocity')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Value')
        self.ax2.set_title('ang_velocity')
        self.ax2.legend()

        self.ax3.clear()  # Third subplot
        self.ax3.plot(self.data_ego_status['time'], self.data_ego_status['act_curvature'], label='act_curvature')
        self.ax3.plot(self.data_chassis['time'], self.data_chassis['cmd_curvature'], label='cmd_curvature', linestyle='--')  # Dashed line
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Value')
        self.ax3.set_title('cmd_curvature & act_curvature')
        self.ax3.legend()

        plt.pause(0.02)

    def update_plot2(self):
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()

        self.ax4.clear()  # Fourth subplot
        self.ax4.plot(self.data_ego_status['time'], self.data_ego_status['act_Yaw'], label='act_Yaw')
        self.ax4.plot(self.data_chassis['time'], self.data_chassis['cmd_center_steer_req'], label='cmd_center_steer_req', linestyle='--')  # Dashed line
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('Value')
        self.ax4.set_title('act_Yaw & cmd_center_steer_req')
        self.ax4.legend()

        plt.pause(0.02)

def main(args=None):
    rclpy.init(args=args)
    data_subscriber = node_data_plot()
    rclpy.spin(data_subscriber)
    data_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
