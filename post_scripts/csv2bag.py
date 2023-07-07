#!/usr/bin/env python3
# Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
#
# All rights reserved.
#
# This software is licensed under the terms of the BSD-2-Clause-License with
# no commercial use allowed, the full terms of which are made available
# in the LICENSE file. No license in patents is granted.
#
# You can contact the author at <christian.brommer@ieee.org>


# The script reads all sensor files from one experiment and generates a rosbag file for it.
# NOTE: All timestamps are offset by 1e-9 seconds because this is the minimal time for ROS headers

import os
import csv
import rosbag
import rospy

from sensor_msgs.msg import Imu, NavSatFix, MagneticField, FluidPressure, Range, CameraInfo
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseStamped, PoseWithCovarianceStamped
from insane_msgs.msg import MotorSpeed, TagDistance

import sys

# Sanitize python path to ensure the import of cv2 for python3
# in the presence of ROS Python 2.7 paths
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
    import cv2 as cv

    sys.path.append(ros_path)
else:
    import cv2 as cv

from cv_bridge import CvBridge

bagfile_name = "insane_dataset.bag"
min_ros_hdr_time = 1e-9


def write_imu_msg(csv_file, topic_name, bag, sensor_name, seq):
    print('Writing ' + sensor_name + ' data to Bag...')

    imu = Imu()  # IMU message

    with open(csv_file, newline='') as csvfile:
        reader = csv.DictReader(csvfile, skipinitialspace=True)
        for seq_count, row in enumerate(reader, start=seq):
            # Update time for message and wall time
            time = rospy.Time.from_sec(float(row['t']) + min_ros_hdr_time)

            # Write IMU message
            imu.header.seq = seq_count
            imu.header.stamp = time
            imu.linear_acceleration.x = float(row['a_x'])
            imu.linear_acceleration.y = float(row['a_y'])
            imu.linear_acceleration.z = float(row['a_z'])
            imu.angular_velocity.x = float(row['w_x'])
            imu.angular_velocity.y = float(row['w_y'])
            imu.angular_velocity.z = float(row['w_z'])

            # Record message into the bag file
            bag.write(topic_name, imu, time)
    print('...done')
    return


def write_mag_msg(csv_file, topic_name, bag, sensor_name, seq):
    print('Writing ' + sensor_name + ' data to Bag...')

    mag_msg = MagneticField()

    with open(csv_file, newline='') as csvfile:
        reader = csv.DictReader(csvfile, skipinitialspace=True)
        for seq_count, row in enumerate(reader, start=seq):
            # Update time for message and wall time
            time = rospy.Time.from_sec(float(row['t']) + min_ros_hdr_time)

            # Write Timed Position message
            mag_msg.header.seq = seq_count
            mag_msg.header.stamp = time
            mag_msg.header.frame_id = 'world'  # World frame
            mag_msg.magnetic_field.x = float(row['cart_x'])
            mag_msg.magnetic_field.y = float(row['cart_y'])
            mag_msg.magnetic_field.z = float(row['cart_z'])

            # Record message into the bag file
            bag.write(topic_name, mag_msg, time)
    print('...done')
    return


def write_baro_msg(csv_file, topic_name, bag, sensor_name, seq):
    print('Writing ' + sensor_name + ' data to Bag...')

    baro_msg = FluidPressure()

    with open(csv_file, newline='') as csvfile:
        reader = csv.DictReader(csvfile, skipinitialspace=True)
        for seq_count, row in enumerate(reader, start=seq):
            # Update time for message and wall time
            time = rospy.Time.from_sec(float(row['t']) + min_ros_hdr_time)

            # Write timed baro message
            baro_msg.header.seq = seq_count
            baro_msg.header.stamp = time
            baro_msg.header.frame_id = 'world'  # World frame
            baro_msg.fluid_pressure = float(row['p'])

            # Record message into the bag file
            bag.write(topic_name, baro_msg, time)
    print('...done')
    return


def write_uwb_msg(csv_file, topic_name, bag, sensor_name, seq):
    print('Writing ' + sensor_name + ' data to Bag...')

    uwb_msg = TagDistance()

    with open(csv_file, newline='') as csvfile:
        reader = csv.DictReader(csvfile, skipinitialspace=True)
        for seq_count, row in enumerate(reader, start=seq):
            # Update time for message and wall time
            time = rospy.Time.from_sec(float(row['t']) + min_ros_hdr_time)

            # Write Timed Position message
            uwb_msg.header.seq = seq_count
            uwb_msg.header.stamp = time
            uwb_msg.valid[0] = bool(int(row['valid_m1']))
            uwb_msg.distance[0] = float(row['dist_m1'])
            uwb_msg.valid[1] = bool(int(row['valid_m2']))
            uwb_msg.distance[1] = float(row['dist_m2'])
            uwb_msg.valid[2] = bool(int(row['valid_m3']))
            uwb_msg.distance[2] = float(row['dist_m3'])
            uwb_msg.valid[3] = False
            uwb_msg.distance[3] = 0

            # Record message into the bag file
            bag.write(topic_name, uwb_msg, time)
    print('...done')
    return


def write_rpm_msg(csv_file, topic_name, bag, sensor_name, seq):
    print('Writing ' + sensor_name + ' data to Bag...')

    rpm_msg = MotorSpeed()

    with open(csv_file, newline='') as csvfile:
        reader = csv.DictReader(csvfile, skipinitialspace=True)
        for seq_count, row in enumerate(reader, start=seq):
            # Update time for message and wall time
            time = rospy.Time.from_sec(float(row['t']) + min_ros_hdr_time)

            # Write Timed Position message
            rpm_msg.header.seq = seq_count
            rpm_msg.header.stamp = time
            rpm_msg.rpm[0] = float(row['motor1'])
            rpm_msg.rpm[1] = float(row['motor2'])
            rpm_msg.rpm[2] = float(row['motor3'])
            rpm_msg.rpm[3] = float(row['motor4'])

            # Record message into the bag file
            bag.write(topic_name, rpm_msg, time)
    print('...done')
    return


def write_lrf_msg(csv_file, topic_name, bag, sensor_name, seq):
    print('Writing ' + sensor_name + ' data to Bag...')

    lrf_msg = Range()

    with open(csv_file, newline='') as csvfile:
        reader = csv.DictReader(csvfile, skipinitialspace=True)
        for seq_count, row in enumerate(reader, start=seq):
            # Update time for message and wall time
            time = rospy.Time.from_sec(float(row['t']) + min_ros_hdr_time)

            # Write Timed Position message
            lrf_msg.header.seq = seq_count
            lrf_msg.header.stamp = time
            lrf_msg.header.frame_id = 'world'  # World frame
            lrf_msg.radiation_type = Range().INFRARED

            lrf_msg.range = float(row['range'])

            # Record message into the bag file
            bag.write(topic_name, lrf_msg, time)
    print('...done')
    return


def write_gps_vel_msg(csv_file, topic_name, bag, sensor_name, seq):
    print('Writing ' + sensor_name + ' data to Bag...')

    gps_vel_msg = TwistWithCovarianceStamped()

    with open(csv_file, newline='') as csvfile:
        reader = csv.DictReader(csvfile, skipinitialspace=True)
        for seq_count, row in enumerate(reader, start=seq):
            # Update time for message and wall time
            time = rospy.Time.from_sec(float(row['t']) + min_ros_hdr_time)

            # Write Velocity message
            gps_vel_msg.header.seq = seq_count
            gps_vel_msg.header.stamp = time
            gps_vel_msg.header.frame_id = 'world'  # World frame
            gps_vel_msg.twist.twist.linear.x = float(row['v_x'])
            gps_vel_msg.twist.twist.linear.y = float(row['v_y'])
            gps_vel_msg.twist.twist.linear.z = float(row['v_z'])

            # Record message into the bag file
            bag.write(topic_name, gps_vel_msg, time)

    print('...done')
    return


def write_gps_coord_msg(csv_file, topic_name, bag, sensor_name, seq):
    print('Writing ' + sensor_name + ' data to Bag...')

    gps_coord_msg = NavSatFix()

    with open(csv_file, newline='') as csvfile:
        reader = csv.DictReader(csvfile, skipinitialspace=True)
        for seq_count, row in enumerate(reader, start=seq):
            # Update time for message and wall time
            time = rospy.Time.from_sec(float(row['t']) + min_ros_hdr_time)

            # Write Coordinate message
            gps_coord_msg.header.seq = seq_count
            gps_coord_msg.header.stamp = time
            gps_coord_msg.header.frame_id = 'world'  # World frame
            gps_coord_msg.latitude = float(row['lat'])
            gps_coord_msg.longitude = float(row['long'])
            gps_coord_msg.altitude = float(row['alt'])

            gps_coord_msg.position_covariance[0] = float(row['cov_p_x'])
            gps_coord_msg.position_covariance[4] = float(row['cov_p_y'])
            gps_coord_msg.position_covariance[5] = float(row['cov_p_z'])

            # Record message into the bag file
            bag.write(topic_name, gps_coord_msg, time)

    print('...done')
    return


def write_gps_enu_msg(csv_file, topic_name, bag, sensor_name, seq):
    print('Writing ' + sensor_name + ' data to Bag...')

    gps_enu_msg = PoseWithCovarianceStamped()

    with open(csv_file, newline='') as csvfile:
        reader = csv.DictReader(csvfile, skipinitialspace=True)
        for seq_count, row in enumerate(reader, start=seq):
            # Update time for message and wall time
            time = rospy.Time.from_sec(float(row['t']) + min_ros_hdr_time)

            # Write Position message
            gps_enu_msg.header.seq = seq_count
            gps_enu_msg.header.stamp = time
            gps_enu_msg.header.frame_id = 'world'  # World frame
            gps_enu_msg.pose.pose.position.x = float(row['p_x'])
            gps_enu_msg.pose.pose.position.y = float(row['p_y'])
            gps_enu_msg.pose.pose.position.z = float(row['p_z'])

            gps_enu_msg.pose.covariance[0] = float(row['cov_p_x'])
            gps_enu_msg.pose.covariance[7] = float(row['cov_p_y'])
            gps_enu_msg.pose.covariance[14] = float(row['cov_p_z'])

            # Record message into the bag file
            bag.write(topic_name, gps_enu_msg, time)

    print('...done')
    return


def write_pose_msg(csv_file, topic_name, bag, sensor_name, seq):
    print('Writing ' + sensor_name + ' data to Bag...')

    pose_msg = PoseStamped()

    with open(csv_file, newline='') as csvfile:
        reader = csv.DictReader(csvfile, skipinitialspace=True)
        for seq_count, row in enumerate(reader, start=seq):
            # Update time for message and wall time
            time = rospy.Time.from_sec(float(row['t']) + min_ros_hdr_time)

            # Write Position message
            pose_msg.header.seq = seq_count
            pose_msg.header.stamp = time
            pose_msg.header.frame_id = 'world'  # World frame
            pose_msg.pose.position.x = float(row['p_x'])
            pose_msg.pose.position.y = float(row['p_y'])
            pose_msg.pose.position.z = float(row['p_z'])

            pose_msg.pose.orientation.w = float(row['q_w'])
            pose_msg.pose.orientation.x = float(row['q_x'])
            pose_msg.pose.orientation.y = float(row['q_y'])
            pose_msg.pose.orientation.z = float(row['q_z'])

            # Record message into the bag file
            bag.write(topic_name, pose_msg, time)
        print('...done')
    return


def write_img_msg(csv_file, topic_name, bag, sensor_name, seq):
    print('Writing ' + sensor_name + ' data to Bag...')
    
    img_folder = "img"
    cv_bridge = CvBridge()

    with open(csv_file, newline='') as csvfile:
        reader = csv.DictReader(csvfile, skipinitialspace=True)
        for seq_count, row in enumerate(reader, start=seq):
            # Update time for message and wall time
            time = rospy.Time.from_sec(float(row['t[ns]']) + min_ros_hdr_time)

            root_path = os.path.join(os.path.split(csv_file)[0], img_folder)
            if not os.path.exists(root_path):
                print("Image file path " + root_path + "does not exist")
                return

            filename = str(row['filename']) + ".png"
            image_file = os.path.join(root_path, filename)
            cv_img = cv.imread(image_file, cv.IMREAD_GRAYSCALE)

            # Write Timed Position message
            image_msg = cv_bridge.cv2_to_imgmsg(cv_img, encoding="passthrough")
            image_msg.header.seq = seq_count
            image_msg.header.stamp = time
            image_msg.header.frame_id = 'world'  # World frame

            # Record message into the bag file
            bag.write(topic_name + "/image_raw", image_msg, time)

            # Add empty camera info msg
            cam_info_msg = CameraInfo()
            cam_info_msg.header.seq = seq
            cam_info_msg.header.stamp = time

            bag.write(topic_name + "/camera_info", cam_info_msg, time)
    print('...done')
    return


def write_img_stereo_msg(csv_file, topic_name, bag, sensor_name, seq):
    print('Writing ' + sensor_name + ' data to Bag...')
    
    img_folder = "img"
    cv_bridge = CvBridge()

    with open(csv_file, newline='') as csvfile:
        reader = csv.DictReader(csvfile, skipinitialspace=True)
        for seq_count, row in enumerate(reader, start=seq):
            # Update time for message and wall time
            time = rospy.Time.from_sec(float(row['t[ns]']) + min_ros_hdr_time)

            root_path = os.path.join(os.path.split(csv_file)[0], img_folder)
            if not os.path.exists(root_path):
                print("Image file path " + root_path + "does not exist")
                return

            filename = str(row['filename']) + ".png"
            image_file = os.path.join(root_path, filename)
            cv_img = cv.imread(image_file, cv.IMREAD_GRAYSCALE)

            # Write Timed Position message
            image_msg = cv_bridge.cv2_to_imgmsg(cv_img, encoding="passthrough")
            image_msg.header.seq = seq_count
            image_msg.header.stamp = time
            image_msg.header.frame_id = 'world'  # World frame

            if filename.find("c1") != -1:
                pre_topic_name = topic_name + "1"
            else:
                pre_topic_name = topic_name + "2"

            # Record message into the bag file
            bag.write(pre_topic_name + "/image_raw", image_msg, time)

            # Add empty camera info msg
            cam_info_msg = CameraInfo()
            cam_info_msg.header.seq = seq
            cam_info_msg.header.stamp = time

            bag.write(pre_topic_name + "/camera_info", cam_info_msg, time)
    print('...done')
    return


class data_module:
    def __init__(self, sensor_name, filename, topic, export_fun):
        self.name = sensor_name
        self.found = False
        self.filename = filename
        self.topic = topic
        self.export_fun = export_fun
        self.path = ""

    def __str__(self):
        return "Name: " + self.name + " Topic: " + self.topic + " File: " + self.filename


def main():
    # Search for csv sensor files in current directory
    dir_list = dict()
    current_path = os.getcwd()
    for root, dirs, files in os.walk(current_path):
        for name in files:
            if name.endswith(".csv"):
                if 'name' in dir_list:
                    print("Warning: " + name + " was found previously")
                    continue

                dir_list[name] = root

    # Define sensor modules and associated csv files for export
    sensor_list = []

    sensor_list.append(data_module("PX4_IMU", "px4_imu.csv", "/mavros/imu/data_raw", write_imu_msg))
    sensor_list.append(data_module("PX4_mag", "px4_mag.csv", "/mavros/imu/mag", write_mag_msg))
    sensor_list.append(data_module("PX4_baro", "px4_baro.csv", "/mavros/imu/static_pressure", write_baro_msg))
    sensor_list.append(data_module("PX4_GPS_coord", "px4_gps.csv", "/mavros/global_position/raw/fix", write_gps_coord_msg))
    sensor_list.append(data_module("PX4_GPS_vel", "px4_gps_vel_data.csv", "/mavros/global_position/raw/gps_vel", write_gps_vel_msg))
    sensor_list.append(data_module("PX4_RPM", "px4_rpm.csv", "/mavros/motor_speeds/speed", write_rpm_msg))
    sensor_list.append(data_module("LSM_IMU", "lsm_imu.csv", "/imu_lsm9ds1/imu", write_imu_msg))
    sensor_list.append(data_module("LSM_mag", "lsm_mag.csv", "/imu_lsm9ds1/mag", write_mag_msg))
    sensor_list.append(data_module("UWB_range", "uwb_range.csv", "/uwb_trek/tagDistance_raw", write_uwb_msg))
    sensor_list.append(data_module("RTK1_coord", "rtk_gps1.csv", "/rtk_gps_1/fix", write_gps_coord_msg))
    sensor_list.append(data_module("RTK1_enu", "rtk_gps1.csv", "/rtk_gps_1/enu", write_gps_enu_msg))
    sensor_list.append(data_module("RTK1_vel", "rtk_gps1.csv", "/rtk_gps_1/fix_velocity", write_gps_vel_msg))
    sensor_list.append(data_module("RTK2_coord", "rtk_gps2.csv", "/rtk_gps_2/fix", write_gps_coord_msg))
    sensor_list.append(data_module("RTK2_enu", "rtk_gps2.csv", "/rtk_gps_2/enu", write_gps_enu_msg))
    sensor_list.append(data_module("RTK2_vel", "rtk_gps2.csv", "/rtk_gps_2/fix_velocity", write_gps_vel_msg))
    sensor_list.append(data_module("MoCap", "mocap_vehicle_data.csv", "/vrpn_client/raw_transform", write_pose_msg))
    sensor_list.append(data_module("RS_IMU", "rs_imu.csv", "/realsense/imu", write_imu_msg))
    sensor_list.append(data_module("RS_odom", "rs_odom.csv", "/realsense/odom/sample", write_pose_msg))
    sensor_list.append(data_module("LRF", "lrf_range.csv", "/lidar_lite/range", write_lrf_msg))
    sensor_list.append(data_module("Nav_cam", "nav_cam_timestamps.csv", "/nav_cam", write_img_msg))
    sensor_list.append(data_module("RS_stereo_cam", "stereo_cam_timestamps.csv", "/realsense/fisheye", write_img_stereo_msg))
    sensor_list.append(data_module("ground_truth", "ground_truth_8hz.csv", "/gt", write_pose_msg))
    sensor_list.append(data_module("ground_truth_interpolated", "ground_truth_80hz.csv", "/gt_inter", write_pose_msg))

    # Check if files for the defined sensors exist and associate the file path
    not_found_sensors = []
    print("Found file for: ")
    for sensor in sensor_list:
        if sensor.filename in dir_list:
            print('\t', end='')
            print(sensor)
            root_dir = dir_list[sensor.filename]
            file_name = sensor.filename
            sensor.path = os.path.join(root_dir, sensor.filename)
            sensor.found = True
        else:
            not_found_sensors.append(sensor.filename)
            sensor.found = False

    # Print not found files
    if len(not_found_sensors) > 0:
        print("Did not find files for:")
        for filename in not_found_sensors:
            print("\t- " + filename)

    # Open file for rosbag and write data for sensor modules whose data was present
    with rosbag.Bag(os.path.join(current_path, bagfile_name), 'w') as bagfile:
        # Write all sensors with associated files
        for sensor in sensor_list:
            if sensor.found:
                sensor.export_fun(sensor.path, sensor.topic, bagfile, sensor.name, 1)


if __name__ == "__main__":
    main()
