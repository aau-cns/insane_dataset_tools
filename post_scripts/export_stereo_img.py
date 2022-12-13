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


# System imports
import sys
import os
import argparse
import rosbag
import re
import numpy as np
import yaml
from tqdm import tqdm

# Sanitize python path to ensure the import of cv2 for python3
# in the presence of ROS Python 2.7 paths
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
    import cv2 as cv

    sys.path.append(ros_path)
else:
    import cv2 as cv

# ROS imports
from cv_bridge import CvBridge

# ROS message imports
from sensor_msgs.msg import Image

# Parameter and Options
raw_img_export_dir = 'export/rs_img/'
time_info_filename = 'export/time_info.yaml'
raw_img_info_filename = 'export/img_rs_timestamps.csv'
in_cam1_topic_name = '/realsense/fisheye1/image_raw'
in_cam2_topic_name = '/realsense/fisheye2/image_raw'
cam1_img_postfix = '_c1'
cam2_img_postfix = '_c2'

do_crop = True

# Read YAML file and extract parameter
f_yaml = open(time_info_filename, "r")
yaml_data = yaml.full_load(f_yaml)
# Read time parameters
t_crop_begin = yaml_data['t_begin']
t_crop_end = yaml_data['t_end']
t_trailing_span = 0  # 3 # time befor and after the data time span in which images are still included

if do_crop:
    print("Crop data at: [%f, %f] " % (t_crop_begin, t_crop_end))

# Raw image options
looseless_compression_factor = 9  # 0-9, 9 being highest compression


def main(argv):
    # Create RAW image folder and info file
    if not os.path.exists(raw_img_export_dir):
        try:
            os.mkdir(raw_img_export_dir)
        except OSError:
            print("Creation of the image export directory %s failed" % raw_img_export_dir)
        else:
            print("Successfully created the image export directory %s " % raw_img_export_dir)
    img_info_file = open(raw_img_info_filename, 'w')
    img_info_file.write("#img_no, t[ns], filename\n")

    cv_bridge = CvBridge()

    # Read ROS topics
    n_img = 0
    n_file = 0

    parser = argparse.ArgumentParser()
    parser.add_argument('input_file', nargs='+', help='input bagfiles e.g. ./stereo_img/*.bag')
    # parser.add_argument('output_file', nargs=1, help='output bagfile')

    num_files = len(parser.parse_args().input_file)
    print("Got %i Files" % num_files)
    input_files = tqdm(parser.parse_args().input_file)

    for in_bagfile_name in input_files:
        n_file = n_file + 1
        print('Processing [' + str(n_file) + '/' + str(num_files) + ']: ' + in_bagfile_name)

        # Read bagfile
        input_bagfile = rosbag.Bag(in_bagfile_name)

        # Print Bag Stats
        topic_count1 = input_bagfile.get_message_count(in_cam1_topic_name)
        topic_count2 = input_bagfile.get_message_count(in_cam2_topic_name)
        print("Processing %i and %i images\n" % (topic_count1, topic_count2))

        # Main data extraction loop
        for topic, img_msg_in, t in input_bagfile.read_messages(topics=[in_cam1_topic_name, in_cam2_topic_name]):

            if do_crop:
                if img_msg_in.header.stamp.to_sec() < t_crop_begin - t_trailing_span or img_msg_in.header.stamp.to_sec() > t_crop_end + t_trailing_span:
                    continue

            n_img = n_img + 1
            img_cv_in = cv_bridge.imgmsg_to_cv2(img_msg_in, desired_encoding='passthrough')

            # Export raw image data
            if topic == in_cam1_topic_name:
                img_postfix = cam1_img_postfix

            elif topic == in_cam2_topic_name:
                img_postfix = cam2_img_postfix

            raw_img_file_name = raw_img_export_dir + '/' + str(n_img) + img_postfix + '.png'
            img_info_file.write(
                str(n_img) + ', ' + "{:.25f}".format(img_msg_in.header.stamp.to_sec()) + ', ' + str(n_img) + str(
                    img_postfix) + "\n")
            cv.imwrite(raw_img_file_name, img_cv_in, [cv.IMWRITE_PNG_COMPRESSION, looseless_compression_factor])

    print("Read %i images" % n_img)


if __name__ == "__main__":
    main(sys.argv[1:])
