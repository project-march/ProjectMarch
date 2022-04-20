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


cv2.createTrackbar('HMin', 'image', 0, 179, nothing)
cv2.createTrackbar('SMin', 'image', 0, 255, nothing)
cv2.createTrackbar('VMin', 'image', 0, 255, nothing)
cv2.createTrackbar('HMax', 'image', 0, 179, nothing)
cv2.createTrackbar('SMax', 'image', 0, 255, nothing)
cv2.createTrackbar('VMax', 'image', 0, 255, nothing)

# Set default value for Max HSV trackbars
cv2.setTrackbarPos('HMax', 'image', 179)
cv2.setTrackbarPos('SMax', 'image', 255)
cv2.setTrackbarPos('VMax', 'image', 255)

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
clipping_distance_in_meters = 100 
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

# lower_red = np.array([217, 0, 93])
# upper_red = np.array([0, 0, 59])

lower_gray = np.array([59,0,166])
upper_gray = np.array([126,40,226])

blue_lower=np.array([100,150,0],np.uint8)
blue_upper=np.array([140,255,255],np.uint8)

lower_brown = np.array([1, 7, 110])
upper_brown = np.array([55, 76, 206])

hMin = sMin = vMin = hMax = sMax = vMax = 0
phMin = psMin = pvMin = phMax = psMax = pvMax = 0


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
        lower = np.array([hMin, sMin, vMin])
        upper = np.array([hMax, sMax, vMax])

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        color_HSV_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        mask_gray = cv2.inRange(color_HSV_image, lower_gray, upper_gray)
        mask_wood = cv2.inRange(color_HSV_image, lower_brown, upper_brown)

        pc = convert_depth_frame_to_pointcloud(depth_image, aligned_depth_frame.profile.as_video_stream_profile().intrinsics)
        # print(pc.shape)

        # Remove background - Set pixels further than clipping_distance to grey
        # grey_color = 153
        # depth_image_3d = np.dstack((depth_image, depth_image, depth_image)) #depth image is 1 channel, color is 3 channels
        # bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
        # bg_removed = color_image

        white = np.full(color_image.shape, (255, 255, 255), np.uint8)

        # img = np.full((100, 100, 3), color, np.uint8)

        filtered = cv2.bitwise_and(white, white, mask=mask_gray)
        filtered = cv2.bitwise_and(filtered, cv2.bitwise_not(white, white, mask=mask_wood))
        filtered = cv2.cvtColor(filtered, cv2.COLOR_RGB2GRAY)
        # filtered = cv2.bitwise_not(filtered, color_image, mask=mask_wood)
        # bg_removed = red_red

        connectivity = 4 # or whatever you prefer
        n_components, output, stats, centroids = cv2.connectedComponentsWithStats(filtered, connectivity, cv2.CV_32S)
        sizes = stats[:, -1]

        stored_components = []

        for i in range(1, n_components+1):
            pts = np.where(output == i)
            if len(pts[0]) < 3000:
                output[pts] = 0
            else:
                stored_components.append(i)


        result = np.full(color_image.shape, (0, 0, 0), np.uint8)

        result[output == 0] = [0, 0, 0]

        for i in stored_components:
            result[output == i] = [255, 255, 255]
            result = cv2.circle(result, (int(centroids[i][0]), int(centroids[i][1])), radius=7, color=(0, 0, 255), thickness=-1)

            # print(centroids[i])
            print(pc[int(centroids[i][1])][int(centroids[i][0])])



        # Render images:
        #   depth align to color on left
        #   depth on right
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