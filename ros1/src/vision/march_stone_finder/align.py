
import pyrealsense2 as rs
import numpy as np
import cv2


def convert_depth_frame_to_pointcloud(depth_image, camera_intrinsics):
    [height, width] = depth_image.shape

    nx = np.linspace(0, width - 1, width)
    ny = np.linspace(0, height - 1, height)
    u, v = np.meshgrid(nx, ny)
    x = (u - camera_intrinsics.ppx) / camera_intrinsics.fx
    y = (v - camera_intrinsics.ppy) / camera_intrinsics.fy

    z = depth_image / 1000
    x = np.multiply(x, z)
    y = np.multiply(y, z)

    return np.dstack((x, y, z))


cv2.namedWindow('image', cv2.WINDOW_NORMAL)
cv2.resizeWindow("image", 800, 500)


cv2.createTrackbar('h_min', 'image', 0, 255, lambda _: None)
cv2.createTrackbar('s_min', 'image', 0, 255, lambda _: None)
cv2.createTrackbar('v_min', 'image', 0, 255, lambda _: None)
cv2.createTrackbar('h_max', 'image', 0, 255, lambda _: None)
cv2.createTrackbar('s_max', 'image', 0, 255, lambda _: None)
cv2.createTrackbar('v_max', 'image', 0, 255, lambda _: None)

width = 640
height = 480
dimensions = (height, width)
ellipse_similarity_threshold = 0.90
min_ellipse_size = 20

pipeline = rs.pipeline()
config = rs.config()
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device.hardware_reset()
device_product_line = str(device.get_info(rs.camera_info.product_line))

config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)

profile = pipeline.start(config)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

clipping_distance_in_meters = 2
clipping_distance = clipping_distance_in_meters / depth_scale

align_to = rs.stream.color
align = rs.align(align_to)

lower_brown = np.array([1, 7, 110])
upper_brown = np.array([55, 76, 206])

h_min = 59
s_min = 0
v_min = 166
h_max = 126
s_max = 40
v_max = 226

# Set default value for Max HSV trackbars
cv2.setTrackbarPos('h_min', 'image', h_min)
cv2.setTrackbarPos('s_min', 'image', s_min)
cv2.setTrackbarPos('v_min', 'image', v_min)
cv2.setTrackbarPos('h_max', 'image', h_max)
cv2.setTrackbarPos('s_max', 'image', s_max)
cv2.setTrackbarPos('v_max', 'image', v_max)

lower_gray = np.array([h_min, s_min, v_min])
upper_gray = np.array([h_max, s_max, v_max])

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            continue

        h_min = cv2.getTrackbarPos('h_min', 'image')
        s_min = cv2.getTrackbarPos('s_min', 'image')
        v_min = cv2.getTrackbarPos('v_min', 'image')
        h_max = cv2.getTrackbarPos('h_max', 'image')
        s_max = cv2.getTrackbarPos('s_max', 'image')
        v_max = cv2.getTrackbarPos('v_max', 'image')

        lower_gray = np.array([0, 0, 168])
        upper_gray = np.array([172, 111, 255])

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        pc = convert_depth_frame_to_pointcloud(depth_image, aligned_depth_frame.profile.as_video_stream_profile().intrinsics)

        depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
        color_image = np.where((depth_image_3d > clipping_distance), 0, color_image)

        color_image = cv2.GaussianBlur(color_image, (5, 5), 0)
        color_hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        mask_gray = cv2.inRange(color_hsv_image, lower_gray, upper_gray)
        mask_wood = cv2.inRange(color_hsv_image, lower_brown, upper_brown)
        white = np.full(dimensions, 255, np.uint8)
        filtered = cv2.bitwise_and(white, white, mask=mask_gray)
        filtered = cv2.bitwise_and(filtered, cv2.bitwise_not(white, white, mask=mask_wood))

        connectivity = 8
        n_components, output, stats, centroids = cv2.connectedComponentsWithStats(filtered, connectivity, cv2.CV_32S)

        stored_components = []
        result = np.full(color_image.shape, (0, 0, 0), np.uint8)

        for i in range(1, n_components + 1):
            pts = np.where(output == i)
            if len(pts[0]) >= 1000:
                result[output == i] = [255, 255, 255]

        result = cv2.morphologyEx(result, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        result = cv2.morphologyEx(result, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))

        result_thres = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        contours, hierarchy = cv2.findContours(result_thres, 1, 2)

        for contour in contours:
            if len(contour) < 5:
                continue

            convex_hull = cv2.convexHull(contour)
            ellipse = cv2.fitEllipse(contour)

            contour_mask = np.zeros(dimensions, np.uint8)
            contour_mask = cv2.drawContours(contour_mask, convex_hull, -1, (255, 255, 255), 2)

            ellipse_mask = np.zeros(dimensions, np.uint8)
            ellipse_mask = cv2.ellipse(ellipse_mask, ellipse, (255, 255, 255), 2)

            intersection = cv2.bitwise_and(contour_mask, ellipse_mask)

            num_intersection = cv2.countNonZero(intersection)
            num_contour = cv2.countNonZero(contour_mask)
            measure = num_intersection / num_contour

            centroid = ellipse[0]
            x_pixel = int(centroid[0])
            y_pixel = int(centroid[1])
            bounds = ellipse[1]

            if measure > ellipse_similarity_threshold and bounds[0] > min_ellipse_size and bounds[1] > min_ellipse_size:
                result = cv2.circle(result, (x_pixel, y_pixel), radius=7, color=(0, 0, 255), thickness=-1)
                result = cv2.ellipse(result, ellipse, (0, measure * 255, 255 - measure * 255), 2)

                if centroid[1] < pc.shape[0] and centroid[0] < pc.shape[1]:
                    depthpoint = pc[y_pixel][x_pixel]
                    text = np.array2string(depthpoint, precision=2, separator=',', suppress_small=True)

                    result = cv2.putText(
                        img=result,
                        text=text,
                        org=(x_pixel + 15, y_pixel + 7),
                        fontFace=cv2.FONT_HERSHEY_DUPLEX,
                        fontScale=0.6,
                        color=(125, 246, 55),
                        thickness=2
                    )

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((result, depth_colormap))

        cv2.imshow('image', images)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()
