## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import time

def convert_depth_to_phys_coord_using_realsense(x, y, depth, intrinsics): 
    _intrinsics = rs.intrinsics()
    # _intrinsics.width = cameraInfo.width
    # _intrinsics.height = cameraInfo.height
    # _intrinsics.ppx = cameraInfo.K[2]
    # _intrinsics.ppy = cameraInfo.K[5]
    # _intrinsics.fx = cameraInfo.K[0]
    # _intrinsics.fy = cameraInfo.K[4]
    #_intrinsics.model = cameraInfo.distortion_model
    # _intrinsics.model = rs.distortion.none
    # _intrinsics.coeffs = [i for i in cameraInfo.D]
    result = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth)
    #result[0]: right, result[1]: down, result[2]: forward
    return result
    # return result[2], -result[0], -result[1]

    # X = (x - intrinsics.ppx)/intrinsics.fx * depth
    # Y = (y - intrinsics.ppy)/intrinsics.fy * depth
    # return X, Y, depth


# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device.hardware_reset()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:

    start_time = time.time()

    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # print(depth_frame.profile.as_video_stream_profile().intrinsics)

        # print("--- %s seconds ---" % (time.time() - start_time))
        start_time = time.time()

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())



        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        print(convert_depth_to_phys_coord_using_realsense(320, 240, depth_image[320][240], depth_frame.profile.as_video_stream_profile().intrinsics))
        # print("dim:")
        # print(depth_image.shape)
        # print(color_image[0][0])

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)

finally:

    # Stop streaming
    pipeline.stop()