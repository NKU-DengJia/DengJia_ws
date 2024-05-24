import csv
import matplotlib.pyplot as plt
from datetime import datetime
import sys

# 设置全局字体大小
plt.rcParams.update({'font.size': 14})

def plot_csv_velocity(csv_filename, label_, ax):
    time = []
    velocity = []

    with open(csv_filename, 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        next(csv_reader)  # Skip header row
        for row in csv_reader:
            time_str = row[0] 
            time.append(datetime.strptime(time_str, '%H:%M:%S.%f'))
            velocity.append(float(row[2]))  # Assuming velocity is in the second column

    ax.plot(time, velocity, label=label_)
    ax.set_title('Velocity (m/s)')
    ax.set_xlabel('Time')
    ax.set_ylabel('Velocity (m/s)')
    ax.grid(True)

def plot_csv_curvature(csv_filename, label_, ax):
    time = []
    curvature = []

    with open(csv_filename, 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        next(csv_reader)  # Skip header row
        for row in csv_reader:
            time_str = row[0]  
            time.append(datetime.strptime(time_str, '%H:%M:%S.%f'))
            curvature.append(float(row[3]))  # Assuming curvature is in the second column

    ax.plot(time, curvature, label=label_)
    ax.set_title('Curvature (1/m)')
    ax.set_xlabel('Time')
    ax.set_ylabel('Curvature (1/m)')
    ax.grid(True)

# if __name__ == '__main__':
#     if len(sys.argv) != 2:
#         print("Usage: python3 ./000.py 2024-05-11_16-37-32")
#         sys.exit(1)

#     _time = sys.argv[1]

#     ego_csv_filename = f'/home/dengjia/DengJia_ws/src/data_plot/csv_recorder/{_time}_EgoStatus.csv'
#     chassis_csv_filename = f'/home/dengjia/DengJia_ws/src/data_plot/csv_recorder/{_time}_ChassisCmd.csv'
#     controlErr_csv_filename = f'/home/dengjia/DengJia_ws/src/data_plot/csv_recorder/{_time}_ControlError.csv'

#     # 设置图的大小
#     fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 10))

#     plot_csv_velocity(ego_csv_filename, 'Ego Status', ax1)
#     plot_csv_velocity(chassis_csv_filename, 'Chassis Command', ax1)

#     plot_csv_curvature(ego_csv_filename, 'Ego Status', ax2)
#     plot_csv_curvature(chassis_csv_filename, 'Chassis Command', ax2)

#     ax1.legend()
#     ax2.legend()

#     plt.tight_layout()
#     plt.show()


def plot_csv_control_error(csv_filename, label_, ax):
    time = []
    column_3 = []
    column_4 = []
    column_5 = []

    with open(csv_filename, 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        next(csv_reader)  # Skip header row
        for row in csv_reader:
            time_str = row[0]
            time.append(datetime.strptime(time_str, '%H:%M:%S.%f'))
            column_3.append(float(row[2]))  # Assuming column 3 is the 3rd column
            column_4.append(float(row[3]))  # Assuming column 4 is the 4th column
            column_5.append(float(row[4]))  # Assuming column 5 is the 5th column

    ax[0].plot(time, column_3, label=f'{label_} - lateral_err(m)')
    ax[1].plot(time, column_4, label=f'{label_} - yaw_err(rad)')
    ax[2].plot(time, column_5, label=f'{label_} - vel_err(m/s)')
    ax[0].set_title('Control Error - lateral_err(m)')
    ax[1].set_title('Control Error - yaw_err(rad)')
    ax[2].set_title('Control Error - vel_err(m/s)')
    for axis in ax:
        axis.set_xlabel('Time')
        axis.set_ylabel('Value')
        axis.grid(True)

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 ./000.py 2024-05-11_16-37-32")
        sys.exit(1)

    _time = sys.argv[1]

    ego_csv_filename = f'/home/dengjia/DengJia_ws/src/data_plot/csv_recorder/{_time}_EgoStatus.csv'
    chassis_csv_filename = f'/home/dengjia/DengJia_ws/src/data_plot/csv_recorder/{_time}_ChassisCmd.csv'
    controlErr_csv_filename = f'/home/dengjia/DengJia_ws/src/data_plot/csv_recorder/{_time}_ControlError.csv'

    # 设置图的大小
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 15))

    plot_csv_velocity(ego_csv_filename, 'Ego Status', ax1)
    plot_csv_velocity(chassis_csv_filename, 'Chassis Command', ax1)

    plot_csv_curvature(ego_csv_filename, 'Ego Status', ax2)
    plot_csv_curvature(chassis_csv_filename, 'Chassis Command', ax2)

    # Add new plot for Control Error using three subplots
    fig, ax3 = plt.subplots(3, 1, figsize=(15, 15))  # Create a new figure with three subplots
    plot_csv_control_error(controlErr_csv_filename, 'Control Error', ax3)

    ax1.legend()
    ax2.legend()
    ax3[0].legend()
    ax3[1].legend()
    ax3[2].legend()

    plt.tight_layout()
    plt.show()

