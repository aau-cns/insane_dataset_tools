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


# For the parameter: 'write' means writing the data to a bagfile,
# 'export' means exporting the raw data to .png images
#
# Note: We are not using the ROI given by 'getOptimalNewCameraMatrix'
#
# @reference: https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html
# @reference: https://docs.opencv.org/master/d7/d4d/tutorial_py_thresholding.html

# System imports
import sys
import os
import argparse
import re
import numpy as np
import yaml
from tqdm import tqdm
from scipy.spatial.transform import Rotation as R

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
import rosbag
# import roslib
# import rospy

# ROS message imports
from sensor_msgs.msg import Image
from fiducial_msgs.msg import FiducialTransform
from fiducial_msgs.msg import FiducialTransformArray

# Parameter and Options
raw_img_export_dir = './export/'
raw_img_info_filename = 'img_timestamps.csv'
in_cam_topic_name = '/mission_cam/image_raw'
out_cam_undist_topic_name = '/mission_cam/image_undistort'

tag_info_yaml_file = './tag_info.yaml'
out_marker_tf_topic_name = '/marker_detections'
out_marker_img_topic_name = '/mission_cam/image_marker'
out_thr_img_topic_name = '/mission_cam/image_thr'
out_marker_img_append_name = '_marker_img'

# Raw image options
export_raw_images = False
looseless_compression_factor = 9  # 0-9, 9 being highest compression

# Undistorted image options
keep_all_pixel = False
export_undist_images = False
write_undistort_images = False

# Marker export options
detect_tags = True
write_tag_images = False  # only if detect_tags = True
write_marker_pose = True  # only if detect_tags = True
write_tag_thrs_images = False
export_tag_detect_images = False  # Export images with tag detection to export dir for debugging
export_tag_thrs_images = False  # Export threshold images to export dir for debugging

# Image Parameter / Calibration
imageSize = (2056, 1542)

# Set Distortion coefficients and intrinsic camera matrix

# Camera matrix
# pinhole camera model (pinhole)
# intrinsics vector(kalibr): [fu fv pu pv])
# intrinsics vector(OpenCV): [fx fy cx cy])
f_x = 1141.1779469749947
f_y = 1141.3215000994876
c_x = 1046.039612036861
c_y = 758.6326219885912

cameraMatrix = np.array([[f_x, 0, c_x], [0, f_y, c_y], [0, 0, 1]])

# Distortion coefficients
# radial-tangential (radtan)
# (distortion_coeffs: [k1 k2 r1 r2])
distCoeffs = np.array([[-0.25487708571221745, 0.05606960044697385, 0.00018224272830655975,
    9.05900966852277e-05]])

# Set Aruco Marker information
aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_7X7_250)

# Get default parameter
aruco_params = cv.aruco.DetectorParameters_create()

# Adapt parameter
aruco_params.adaptiveThreshWinSizeMin = 141  # default 3
aruco_params.adaptiveThreshWinSizeMax = 251  # default 23
aruco_params.adaptiveThreshWinSizeStep = 10  # default 10
aruco_params.adaptiveThreshConstant = 7  # default 7
# Note that this step is optional and it only makes sense if the positions of the marker corners have to be accurate,
# for instance for pose estimation. It is usually a time-consuming step and therefore is disabled by default.
# CORNER_REFINE_APRILTAG, CORNER_REFINE_CONTOUR, CORNER_REFINE_NONE, CORNER_REFINE_SUBPIX

aruco_params.cornerRefinementMethod = cv.aruco.CORNER_REFINE_CONTOUR
# This value can be lower than default because undistorted images are used

aruco_params.polygonalApproxAccuracyRate = 0.005


## Outdoor Parameter
# aruco_params.adaptiveThreshWinSizeMin = 31 #141  # default 3
# aruco_params.adaptiveThreshWinSizeMax = 61 #251  # default 23
# aruco_params.adaptiveThreshWinSizeStep = 10  # default 10
# aruco_params.adaptiveThreshConstant = 7      # default 7
## Note that this step is optional and it only makes sense if the positions of the marker corners have to be accurate,
## for instance for pose estimation. It is usually a time-consuming step and therefore is disabled by default.
## CORNER_REFINE_APRILTAG, CORNER_REFINE_CONTOUR, CORNER_REFINE_NONE, CORNER_REFINE_SUBPIX

# aruco_params.cornerRefinementMethod = cv.aruco.CORNER_REFINE_CONTOUR
## This value can be lower than default because undistorted images are used

##aruco_params.polygonalApproxAccuracyRate = 0.005

##aruco_params.errorCorrectionRate = 1.2
# aruco_params.maxErroneousBitsInBorderRate = 50
# aruco_params.perspectiveRemovePixelPerCell = 1
##aruco_params.perspectiveRemoveIgnoredMarginPerCell = 0.5


def main(argv):
    if detect_tags:
        # Read YAML file and extract parameter
        f_yaml = open(tag_info_yaml_file, "r")
        yaml_data = yaml.full_load(f_yaml)
        # Read tag_size information
        marker_size_dict = yaml_data['marker_size'][0]
        print("Marker ID/Size Parameter: ", marker_size_dict)

    # Create RAW image folder and info file
    if export_raw_images or export_tag_thrs_images:
        if not os.path.exists(raw_img_export_dir):
            try:
                os.mkdir(raw_img_export_dir)
            except OSError:
                print("Creation of the image export directory %s failed" % raw_img_export_dir)
            else:
                print("Successfully created the image export directory %s " % raw_img_export_dir)
        img_info_file = open(raw_img_export_dir + raw_img_info_filename, 'w')
        img_info_file.write("#img_no, t[ns]\n")

    # Read Calibration and generate camera matrix
    if keep_all_pixel:
        newCameraMatrix, validPixROI = cv.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize)
    else:
        newCameraMatrix, validPixROI = cv.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 0, imageSize)

    newCameraMatrix = cameraMatrix
    mapx, mapy = cv.initUndistortRectifyMap(cameraMatrix, distCoeffs, None, newCameraMatrix, imageSize, 5)

    # Get file name arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('input_file', nargs='+', help='input bagfiles')
    parser.add_argument('output_file', nargs=1, default='tag_detections.bag', help='output bagfile')

    num_files = len(parser.parse_args().input_file)
    print("Got %i Files" % num_files)

    # Create new bagfiles for undistorted images and tag detections
    out_bagfile_name = parser.parse_args().output_file[0]
    out_bagfile = rosbag.Bag(out_bagfile_name, 'w')

    if write_tag_images:
        # Open dedicated bagfile for images with marker detections
        # Remove possible .bag postfix
        out_bagfile_marker_img_name = re.sub('\.bag.*', '', out_bagfile_name)
        # Append post fix
        out_bagfile_marker_img_name = out_bagfile_marker_img_name + out_marker_img_append_name + '.bag'
        # Open new bagfile
        out_bagfile_marker_img = rosbag.Bag(out_bagfile_marker_img_name, 'w')

    cv_bridge = CvBridge()

    # Read ROS topics
    n_img = 0
    n_file = 0
    input_files = tqdm(parser.parse_args().input_file)

    for in_bagfile_name in input_files:
        n_file = n_file + 1
        print('Processing [' + str(n_file) + '/' + str(num_files) + ']: ' + in_bagfile_name)

        # Check that we got a valid bagfile
        # TODO

        # Read bagfile
        input_bagfile = rosbag.Bag(in_bagfile_name)

        # Print Bag Stats
        topic_count = input_bagfile.get_message_count(in_cam_topic_name)
        print("Processing %i images\n" % topic_count)

        # Main data extraction loop
        for topic, img_msg_in, t in input_bagfile.read_messages(topics=in_cam_topic_name):
            n_img = n_img + 1
            img_cv_in = cv_bridge.imgmsg_to_cv2(img_msg_in, desired_encoding='passthrough')

            # Export raw image data
            if export_raw_images:
                # img_msg_in.header.stamp
                raw_img_file_name = raw_img_export_dir + '/' + str(n_img) + '.png'
                cv.imwrite(raw_img_file_name, img_cv_in, [cv.IMWRITE_PNG_COMPRESSION, looseless_compression_factor])
                img_info_file.write(str(n_img) + ', ' + str(img_msg_in.header.stamp) + "\n")

            # Undistort Image
            # h, w = img_cv_in.shape[:2]
            img_cv_undist_out = cv.remap(img_cv_in, mapx, mapy, cv.INTER_LINEAR)

            if export_undist_images:
                cv.imwrite(raw_img_export_dir + '/' + str(n_img) + '_undist' + '.png', img_cv_undist_out)

            # Write undistorted image to bagfile, maintain msg header information
            if write_undistort_images:
                img_msg_out = cv_bridge.cv2_to_imgmsg(img_cv_undist_out, encoding="mono8")
                # img_msg_out.header = img_msg_in.header
                out_bagfile.write(out_cam_undist_topic_name, img_msg_out, img_msg_in.header.stamp)

            # Detect Aruco Marker and write the image depending on settings
            if detect_tags:
                corners, ids, rejected = cv.aruco.detectMarkers(img_cv_undist_out, aruco_dict, parameters=aruco_params)

                # Draw and store marker detections
                if write_tag_images or export_tag_detect_images:
                    img_marker = cv.aruco.drawDetectedMarkers(cv.cvtColor(img_cv_undist_out, cv.COLOR_GRAY2RGB),
                                                              corners, ids)
                    if write_tag_images:
                        img_marker_msg_out = cv_bridge.cv2_to_imgmsg(img_marker, encoding="rgb8")
                        img_marker_msg_out.header = img_msg_in.header
                        out_bagfile_marker_img.write(out_marker_img_topic_name, img_marker_msg_out,
                                                     img_msg_in.header.stamp)
                    if export_tag_detect_images:
                        cv.imwrite(raw_img_export_dir + '/' + str(n_img) + '_mark' + '.png', img_marker)

                if write_tag_thrs_images or export_tag_thrs_images:
                    thr_img = cv.adaptiveThreshold(img_cv_undist_out, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, \
                                                   cv.THRESH_BINARY_INV, aruco_params.adaptiveThreshWinSizeMin,
                                                   aruco_params.adaptiveThreshConstant)
                    if export_tag_thrs_images:
                        cv.imwrite(raw_img_export_dir + '/' + str(n_img) + '_thr' + '.png', thr_img)

                    if write_tag_thrs_images:
                        img_thr_msg_out = cv_bridge.cv2_to_imgmsg(thr_img, encoding="mono8")
                        img_thr_msg_out.header = img_msg_in.header
                        out_bagfile_marker_img.write(out_thr_img_topic_name, img_thr_msg_out,
                                                     img_msg_in.header.stamp)

                # Write marker poses
                if write_marker_pose:
                    zeroDistCoeffs = np.array([[0.0, 0.0, 0.0, 0.0]])  # zero distortion coefficients

                    if ids is not None:  # Check if current image has marker detections
                        pose = []
                        # Write poses for tags in current image
                        for k in range(len(ids)):

                            if ids.item(k) in marker_size_dict:
                                current_id = ids.item(k)
                                current_corners = corners[k]
                                current_marker_size = marker_size_dict[current_id]

                                rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(current_corners, current_marker_size,
                                                                                   newCameraMatrix, zeroDistCoeffs)

                                rmat, _ = cv.Rodrigues(rvec)

                                # The returned value is in scalar-last (x, y, z, w) format
                                quat = R.from_matrix(rmat).as_quat()

                                current_pose = FiducialTransform()
                                current_pose.fiducial_id = current_id
                                current_pose.transform.translation.x = tvec.item(0)
                                current_pose.transform.translation.y = tvec.item(1)
                                current_pose.transform.translation.z = tvec.item(2)

                                current_pose.transform.rotation.x = quat.item(0)
                                current_pose.transform.rotation.y = quat.item(1)
                                current_pose.transform.rotation.z = quat.item(2)
                                current_pose.transform.rotation.w = quat.item(3)

                                pose.append(current_pose)

                            else:
                                print('Marker size for ID: ', ids.item(k),
                                      'is not defined - Not generating a pose for it!')
                                continue

                        # Check if transforms exist and write them
                        if pose:
                            # Generate message for the camera poses w.r.t. the marker frame
                            marker_transform = FiducialTransformArray()
                            marker_transform.header = img_msg_in.header
                            # Map pose/transform array
                            marker_transform.transforms = pose
                            out_bagfile.write(out_marker_tf_topic_name, marker_transform, marker_transform.header.stamp)

    print("Read %i images" % n_img)

    # Close all files
    out_bagfile.close()

    if write_tag_images:
        out_bagfile_marker_img.close()
    if export_raw_images:
        img_info_file.close()


if __name__ == "__main__":
    main(sys.argv[1:])
