## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2




def convert_depth_frame_to_pointcloud(depth_image, camera_intrinsics):
	"""
	Convert the depthmap to a 3D point cloud
	Parameters:
	-----------
	depth_frame 	 	 : rs.frame()
						   The depth_frame containing the depth map
	camera_intrinsics : The intrinsic values of the imager in whose coordinate system the depth_frame is computed
	Return:
	----------
	x : array
		The x values of the pointcloud in meters
	y : array
		The y values of the pointcloud in meters
	z : array
		The z values of the pointcloud in meters
	"""
	
	[height, width] = depth_image.shape

	nx = np.linspace(0, width - 1, width)
	ny = np.linspace(0, height - 1, height)
	u, v = np.meshgrid(nx, ny)
	x = (u - camera_intrinsics.ppx) / camera_intrinsics.fx
	y = (v - camera_intrinsics.ppy) / camera_intrinsics.fy

	z = depth_image / 1000
	x = np.multiply(x, z)
	y = np.multiply(y, z)

	# x = x[np.nonzero(z)]
	# y = y[np.nonzero(z)]
	# z = z[np.nonzero(z)]
    
	return np.dstack((x, y, z))


cv2.namedWindow('image', cv2.WINDOW_NORMAL)
cv2.resizeWindow("image", 800, 500)


def nothing(x):
    pass


cv2.createTrackbar('HMin', 'image', 0, 255, nothing)
cv2.createTrackbar('SMin', 'image', 0, 255, nothing)
cv2.createTrackbar('VMin', 'image', 0, 255, nothing)
cv2.createTrackbar('HMax', 'image', 0, 255, nothing)
cv2.createTrackbar('SMax', 'image', 0, 255, nothing)
cv2.createTrackbar('VMax', 'image', 0, 255, nothing)

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
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
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 2
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

# lower_red = np.array([217, 0, 93])
# upper_red = np.array([0, 0, 59])

blue_lower = np.array([100, 150, 0], np.uint8)
blue_upper = np.array([140, 255, 255], np.uint8)

lower_brown = np.array([1, 7, 110])
upper_brown = np.array([55, 76, 206])

hMin = 59
sMin = 0
vMin = 166

hMax = 126
sMax = 40
vMax = 226

# hMin = 0
# sMin = 0
# vMin = 97

# hMax = 159
# sMax = 51
# vMax = 162

# Set default value for Max HSV trackbars
cv2.setTrackbarPos('HMin', 'image', hMin)
cv2.setTrackbarPos('SMin', 'image', sMin)
cv2.setTrackbarPos('VMin', 'image', vMin)
cv2.setTrackbarPos('HMax', 'image', hMax)
cv2.setTrackbarPos('SMax', 'image', sMax)
cv2.setTrackbarPos('VMax', 'image', vMax)

lower_gray = np.array([hMin, sMin, vMin])
upper_gray = np.array([hMax, sMax, vMax])


# Streaming loop
try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        hMin = cv2.getTrackbarPos('HMin', 'image')
        sMin = cv2.getTrackbarPos('SMin', 'image')
        vMin = cv2.getTrackbarPos('VMin', 'image')
        hMax = cv2.getTrackbarPos('HMax', 'image')
        sMax = cv2.getTrackbarPos('SMax', 'image')
        vMax = cv2.getTrackbarPos('VMax', 'image')

        # Set minimum and maximum HSV values to display
        lower_gray = np.array([hMin, sMin, vMin])
        upper_gray = np.array([hMax, sMax, vMax])

        lower_gray = np.array([0, 0, 168])
        upper_gray = np.array([172, 111, 255])

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
        color_image = np.where((depth_image_3d > clipping_distance), 0, color_image)

        color_image = cv2.GaussianBlur(color_image, (5,5), 0)

        color_HSV_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        mask_gray = cv2.inRange(color_HSV_image, lower_gray, upper_gray)
        mask_wood = cv2.inRange(color_HSV_image, lower_brown, upper_brown)

        pc = convert_depth_frame_to_pointcloud(depth_image, aligned_depth_frame.profile.as_video_stream_profile().intrinsics)


        white = np.full(color_image.shape, (255, 255, 255), np.uint8)
        filtered = cv2.bitwise_and(white, white, mask=mask_gray)
        filtered = cv2.bitwise_and(filtered, cv2.bitwise_not(white, white, mask=mask_wood))
        filtered = cv2.cvtColor(filtered, cv2.COLOR_RGB2GRAY)

        connectivity = 8 # or whatever you prefer
        n_components, output, stats, centroids = cv2.connectedComponentsWithStats(filtered, connectivity, cv2.CV_32S)

        stored_components = []

        for i in range(1, n_components+1):
            pts = np.where(output == i)
            if len(pts[0]) < 1000:
                output[pts] = 0
            else:
                stored_components.append(i)

        result = np.full(color_image.shape, (0, 0, 0), np.uint8)
        for i in stored_components:
            result[output == i] = [255, 255, 255]

        result = cv2.morphologyEx(result, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        result = cv2.morphologyEx(result, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))

        result_thres = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        contours, hierarchy = cv2.findContours(result_thres, 1, 2)

        for cnt in contours:
            if len(cnt) < 5:
                continue

            convexHull = cv2.convexHull(cnt)
            ellipse = cv2.fitEllipse(cnt)

            # result = cv2.ellipse(result, ellipse, (0,255,0), 2)

            contour_mask = np.zeros((color_image.shape[0], color_image.shape[1]), np.uint8)
            contour_mask = cv2.drawContours(contour_mask, convexHull, -1, (255, 255, 255), 2)

            ellipse_mask = np.zeros((color_image.shape[0], color_image.shape[1]), np.uint8)
            ellipse_mask = cv2.ellipse(ellipse_mask, ellipse, (255, 255, 255), 2)

            intersection = cv2.bitwise_and(contour_mask, ellipse_mask)

            num_intersection = cv2.countNonZero(intersection)
            num_contour = cv2.countNonZero(contour_mask)
            measure = num_intersection / num_contour

            centroid = ellipse[0]
            bounds = ellipse[1]

            if measure > 0.90 and bounds[0] > 20 and bounds[1] > 20:
                result = cv2.circle(result, (int(centroid[0]), int(centroid[1])), radius=7, color=(0, 0, 255), thickness=-1)
                result = cv2.ellipse(result, ellipse, (0, measure*255, 255 - measure*255), 2)

                if centroid[1] < pc.shape[0] and centroid[0] < pc.shape[1]:
                    depthpoint = pc[int(centroid[1])][int(centroid[0])]
                    text = np.array2string(depthpoint, precision=2, separator=',', suppress_small=True)

                    result = cv2.putText(
                        img = result,
                        text = text,
                        org = (int(centroid[0]) + 15, int(centroid[1]) + 5),
                        fontFace = cv2.FONT_HERSHEY_DUPLEX,
                        fontScale = 0.6,
                        color = (125, 246, 55),
                        thickness = 2
                    )



        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((result, depth_colormap))

        
        cv2.imshow('image', images)
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()