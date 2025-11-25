import matplotlib.pyplot as plt
import rosbag2_py
import numpy as np
from rclpy.serialization import deserialize_message
from px4_msgs.msg import VehicleLocalPosition, ManualControlSetpoint, ActuatorOutputs, VehicleAttitude

#bag file path
BAG_PATH = '/home/user/ros2_ws/src/bag_files/data/'

def get_yaw_from_quaternion(q):
    """
    Convert quaternion array [w, x, y, z] to Yaw (radians)
    """
    w, x, y, z = q[0], q[1], q[2], q[3]
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_rad = np.arctan2(t3, t4)
    return yaw_rad

def read_bag_data(bag_path):
    print(f"Opening bag: {bag_path}")
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Could not open as sqlite3, trying mcap... ({e})")
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
        reader.open(storage_options, converter_options)

    # Data Containers
    pos_data = {'time': [], 'x': [], 'y': [], 'z_ned': [], 'z_enu': [], 'vx': [], 'vy': [], 'vz': []}
    att_data = {'time': [], 'yaw': []}
    rc_data = {'time': [], 'throttle': []}
    act_data = {'time': [], 'm1': [], 'm2': [], 'm3': [], 'm4': []}

    topics_found = set()

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        topics_found.add(topic)
        time_sec = t / 1e9 

        if topic == '/fmu/out/vehicle_local_position':
            msg = deserialize_message(data, VehicleLocalPosition)
            pos_data['time'].append(time_sec)
            pos_data['x'].append(msg.x)
            pos_data['y'].append(msg.y)
            pos_data['z_ned'].append(msg.z)   # Raw PX4 (Down+)
            pos_data['z_enu'].append(-msg.z)  # Converted (Up+)
            pos_data['vx'].append(msg.vx)
            pos_data['vy'].append(msg.vy)
            pos_data['vz'].append(msg.vz)

        elif topic == '/fmu/out/vehicle_attitude':
            msg = deserialize_message(data, VehicleAttitude)
            att_data['time'].append(time_sec)
            att_data['yaw'].append(get_yaw_from_quaternion(msg.q))

        elif topic == '/fmu/out/manual_control_setpoint':
            msg = deserialize_message(data, ManualControlSetpoint)
            rc_data['time'].append(time_sec)
            # In PX4, 'z' or 'throttle' is the vertical control
            rc_data['throttle'].append(msg.throttle) 

        elif topic == '/fmu/out/actuator_outputs':
            msg = deserialize_message(data, ActuatorOutputs)
            act_data['time'].append(time_sec)
            # Assuming a Quadcopter (4 motors)
            if len(msg.output) >= 4:
                act_data['m1'].append(msg.output[0])
                act_data['m2'].append(msg.output[1])
                act_data['m3'].append(msg.output[2])
                act_data['m4'].append(msg.output[3])

    print(f"Topics found in bag: {topics_found}")
    return pos_data, att_data, rc_data, act_data

def normalize_time(data_dict, start_time):
    """Helper to zero-out time arrays"""
    if 'time' in data_dict and len(data_dict['time']) > 0:
        data_dict['time'] = np.array(data_dict['time']) - start_time
    return data_dict

def plot_assignment_results(pos, att, rc, act):
    # Find global start time to sync all plots
    start_candidates = []
    if pos['time']: start_candidates.append(pos['time'][0])
    if att['time']: start_candidates.append(att['time'][0])
    if rc['time']: start_candidates.append(rc['time'][0])
    if act['time']: start_candidates.append(act['time'][0])
    
    if not start_candidates:
        print("CRITICAL ERROR: No data found in bag.")
        return

    t0 = min(start_candidates)
    
    # Normalize all times
    pos = normalize_time(pos, t0)
    att = normalize_time(att, t0)
    rc = normalize_time(rc, t0)
    act = normalize_time(act, t0)

    
    # ASSIGNMENT POINT 1b: Actuator Outputs
    fig1 = plt.figure(num=1, figsize=(10, 6))
    plt.title("Assignment 1(b): Actuator Outputs")
    if len(act['time']) > 0:
        plt.plot(act['time'], act['m1'], label='Motor 1', alpha=0.8)
        plt.plot(act['time'], act['m2'], label='Motor 2', alpha=0.8)
        plt.plot(act['time'], act['m3'], label='Motor 3', alpha=0.8)
        plt.plot(act['time'], act['m4'], label='Motor 4', alpha=0.8)
        plt.ylabel('Output (PWM or Normalized)')
        plt.xlabel('Time [s]')
        plt.legend()
        plt.grid(True)
    else:
        plt.text(0.5, 0.5, "No Actuator Data Found", ha='center')

    
    # ASSIGNMENT POINT 2b: Force Land Logic Check
    # (Altitude ENU + Manual Control Setpoint)
    fig2, (ax2_1, ax2_2) = plt.subplots(2, 1, num=2, sharex=True, figsize=(10, 8))
    fig2.suptitle("Assignment 2(b): Force Land & Manual Override Analysis")

    # Plot Altitude (ENU)
    if len(pos['time']) > 0:
        ax2_1.plot(pos['time'], pos['z_enu'], 'b-', label='Altitude (ENU)')
        ax2_1.axhline(y=20, color='r', linestyle='--', label='Threshold (20m)')
        ax2_1.set_ylabel('Altitude [m]')
        ax2_1.grid(True)
        ax2_1.legend(loc='upper right')
    
    # Plot Manual Control (Throttle)
    if len(rc['time']) > 0:
        ax2_2.plot(rc['time'], rc['throttle'], 'g-', label='Pilot Throttle Setpoint')
        ax2_2.set_ylabel('Stick Input [0-1]')
        ax2_2.set_xlabel('Time [s]')
        ax2_2.grid(True)
        ax2_2.legend(loc='upper right')
    else:
        ax2_2.text(0.5, 0.5, "No ManualControlSetpoint Data", ha='center')

    
    # ASSIGNMENT POINT 3b: Trajectory Planner Analysis
    # (XY Path, Altitude, Yaw, Velocity, Acceleration)
    fig3 = plt.figure(num=3, figsize=(12, 10))
    fig3.suptitle("Assignment 3(b): Offboard Trajectory Analysis")

    # 3b.1: Trajectory on XY Plane
    ax3_1 = fig3.add_subplot(2, 2, 1)
    if len(pos['time']) > 0:
        ax3_1.plot(pos['y'], pos['x'], 'b-', linewidth=2) # X is North, Y is East
        ax3_1.plot(pos['y'][0], pos['x'][0], 'go', label='Start')
        ax3_1.plot(pos['y'][-1], pos['x'][-1], 'rx', label='End')
        ax3_1.set_title("Trajectory (XY Plane)")
        ax3_1.set_xlabel("East (Y) [m]")
        ax3_1.set_ylabel("North (X) [m]")
        ax3_1.axis('equal')
        ax3_1.grid(True)
        ax3_1.legend()

    # 3b.2: Altitude
    ax3_2 = fig3.add_subplot(2, 2, 2)
    if len(pos['time']) > 0:
        ax3_2.plot(pos['time'], pos['z_enu'], 'k-')
        ax3_2.set_title("Altitude Profile")
        ax3_2.set_ylabel("Height [m]")
        ax3_2.set_xlabel("Time [s]")
        ax3_2.grid(True)

    # 3b.3: Yaw (from Attitude msg)
    ax3_3 = fig3.add_subplot(2, 2, 3)
    if len(att['time']) > 0:
        ax3_3.plot(att['time'], np.degrees(att['yaw']), 'm-')
        ax3_3.set_title("Yaw Profile")
        ax3_3.set_ylabel("Yaw [deg]")
        ax3_3.set_xlabel("Time [s]")
        ax3_3.grid(True)
    else:
        ax3_3.text(0.5, 0.5, "No Attitude Data", ha='center')

    # 3b.4: Velocity & Acceleration
    ax3_4 = fig3.add_subplot(2, 2, 4)
    if len(pos['time']) > 0:
        # Calculate Velocity Magnitude (Speed)
        vel_mag = np.sqrt(np.array(pos['vx'])**2 + np.array(pos['vy'])**2 + np.array(pos['vz'])**2)
        
        # Calculate Acceleration (Derivative of Velocity)
        # Gradient calculates differences between adjacent elements
        acc_x = np.gradient(pos['vx'], pos['time'])
        acc_y = np.gradient(pos['vy'], pos['time'])
        acc_z = np.gradient(pos['vz'], pos['time'])
        acc_mag = np.sqrt(acc_x**2 + acc_y**2 + acc_z**2)

        ax3_4.plot(pos['time'], vel_mag, 'b-', label='Velocity [m/s]')
        ax3_4.plot(pos['time'], acc_mag, 'r-', label='Accel [m/s^2]', alpha=0.7)
        ax3_4.set_title("Dynamics")
        ax3_4.set_xlabel("Time [s]")
        ax3_4.legend()
        ax3_4.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    p, a, r, ac = read_bag_data(BAG_PATH)
    plot_assignment_results(p, a, r, ac)
