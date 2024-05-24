import rclpy
from rclpy.node import Node
import csv
import os
from datetime import datetime

from self_interface.msg import EgoStatus
from self_interface.msg import ChassisCmd
from self_interface.msg import ControlError

import configparser

class TopicRecorderNode(Node):
    def __init__(self):
        super().__init__('topic_recorder')
        self.read_data_record_ini()
        if self.use_record2csv:
            self.subscription1 = self.create_subscription(
                EgoStatus,
                'ego_status_msg',  
                self.callback1,
                10)
            self.subscription1  # 防止subscription被垃圾收集器回收

            self.subscription2 = self.create_subscription(
                ChassisCmd,
                'chassis_cmd',  
                self.callback2,
                10)
            self.subscription2  # 防止subscription被垃圾收集器回收

            self.subscription3 = self.create_subscription(
                ControlError,
                'control_error',  
                self.callback3,
                10)
            self.subscription3  # 防止subscription被垃圾收集器回收


            # 获取当前时间并构建CSV文件路径
            current_time = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
            folder_path = '/home/dengjia/DengJia_ws/src/data_plot/csv_recorder' 
            csv_filename1 = os.path.join(folder_path,  f'{current_time}_EgoStatus.csv')
            csv_filename2 = os.path.join(folder_path,  f'{current_time}_ChassisCmd.csv')
            csv_filename3 = os.path.join(folder_path,  f'{current_time}_ControlError.csv')
            
            # 创建CSV文件并打开以进行写入
            self.csv_file1 = open(csv_filename1, 'w')
            self.csv_writer1 = csv.writer(self.csv_file1)
            self.csv_writer1.writerow(['real_time', 'header_timestamp', 'ego_velocity(m/s)', 'ego_curvature(1/m)'])

            self.csv_file2 = open(csv_filename2, 'w')
            self.csv_writer2 = csv.writer(self.csv_file2)
            self.csv_writer2.writerow(['real_time', 'header_timestamp', 'cmd_velocity(m/s)','cmd_curvature(1/m)'])

            self.csv_file3 = open(csv_filename3, 'w')
            self.csv_writer3 = csv.writer(self.csv_file3)
            self.csv_writer3.writerow(['real_time', 'header_timestamp', 'lateral_err(m)','yaw_err(rad)','vel_err(m/s)'])

    use_record2csv = False
    def read_data_record_ini(self):
        # 创建 ConfigParser 对象
        config = configparser.ConfigParser()
        # 读取配置文件
        file_path = "/home/dengjia/DengJia_ws/src/data_plot/data_plot/data_record.ini"
        config.read(file_path)
        # 获取配置文件中的值
        self.use_record2csv = config.getboolean('data_record', 'use_record2csv')

    def callback1(self, msg):
        # 获取消息头中的时间戳，并将其转换为实时时间
        header_timestamp_sec = msg.header.stamp.sec
        header_timestamp_nanosec = msg.header.stamp.nanosec
        real_time = self.get_real_time(header_timestamp_sec, header_timestamp_nanosec)
        header_timestamp = f"{header_timestamp_sec}{header_timestamp_nanosec:09d}"
        self.csv_writer1.writerow([real_time, 
                                   header_timestamp, 
                                   msg.ego_velocity,
                                   msg.ego_curvature
                                   ])
        # self.get_logger().info(f'Recorded: {msg.ego_velocity}')

    def callback2(self, msg):
        header_timestamp_sec = msg.header.stamp.sec
        header_timestamp_nanosec = msg.header.stamp.nanosec
        real_time = self.get_real_time(header_timestamp_sec, header_timestamp_nanosec)
        header_timestamp = f"{header_timestamp_sec}{header_timestamp_nanosec:09d}"
        cmd_curvature= 0.0
        if -400 < msg.turn_radius_req < 400:
            cmd_curvature = 1.0 / msg.turn_radius_req
        self.csv_writer2.writerow([real_time, 
                                   header_timestamp, 
                                   msg.vel_req, 
                                   cmd_curvature
                                   ])
    
    def callback3(self, msg):
        header_timestamp_sec = msg.header.stamp.sec
        header_timestamp_nanosec = msg.header.stamp.nanosec
        real_time = self.get_real_time(header_timestamp_sec, header_timestamp_nanosec)
        header_timestamp = f"{header_timestamp_sec}{header_timestamp_nanosec:09d}"
        self.csv_writer3.writerow([real_time, 
                                   header_timestamp, 
                                   msg.lateral_err, 
                                   msg.yaw_err,
                                   msg.vel_err
                                   ])

    def get_real_time(self, sec, nanosec):
        # 根据消息头中的时间戳计算实时时间，并格式化为 '%H:%M:%S.%f'
        timestamp = sec + nanosec / 1e9  # 将纳秒转换为秒
        return datetime.fromtimestamp(timestamp).strftime('%H:%M:%S.%f')

    def close(self):
        # 关闭CSV文件
        self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)
    node = TopicRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('正在关闭...')
    finally:
        node.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
