import cv2
import numpy as np
import pyrealsense2 as rs
import apriltag

# ################### modified by Rong Zhiyi ####################

# ################## embedded in a config file ##################
# # TAG_ID = 'E280-1160-6000-0209-F811-48C3' # white, apriltag
# TAG_ID = 'E280-1160-6000-0209-F811-5C03' # brown, aruco
# PHASE_LOAD_NUM = 4
TAG_SIZE = 0.05
# # antenna params
# NUM_ANTENNA = 4
# # WAVE_LENGTH = 0.164624707 * 2 # tuned
# WAVE_LENGTH = 0.162924707 * 2 # provided
# # camera params
F_X, F_Y = 649.376, 649.376 # focal_length
C_X, C_Y = 648.137, 353.517 # principal_point
# # structural params
# X_OFFSET = 0.032
# Y_OFFSET = 0.085
# Z_OFFSET = 0.06
# HALF_SIDE = 0.125
# # candidates generator params
# RADIUS = 0.05 # metre
# NUM_CANDIDATES = 100
# # starting point used for manual-grid-calibration
# # location_initial = np.array([-0.5, 0, 1])
# # P controller params
# KP_linear = 0.08
# KP_angular = 0.008
# ####################### saving data ###########################
# phase_dist_file = open('phase_dist.csv', 'a+')
# phase_dist_writer = csv.writer(phase_dist_file)
# # traj_gt_file = open('traj_gt.csv', 'a+')
# # traj_gt_writer = csv.writer(traj_gt_file)
# traj_pd_file = open('traj_pd.csv', 'a+')
# traj_pd_writer = csv.writer(traj_pd_file)


# def camera_init():
#     pipeline = rs.pipeline()
#     config = rs.config()
#     # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30) # depth
#     config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30) # color
#     pipeline.start(config)
#     tag_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))

#     return pipeline, tag_detector

    
# def tag_detect(pipeline, tag_detector):
#     frames = pipeline.wait_for_frames()
#     # depth_frame = frames.get_depth_frame()
#     color_frame = frames.get_color_frame()
#     # depth_image = np.asanyarray(depth_frame.get_data()) # numpy.ndarray
#     color_image = np.asanyarray(color_frame.get_data())
#     gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    
#     # show images, only for illustration
#     cv2.imshow('RealSense', color_image)
#     # key = cv2.waitKey(1)
#     tags_list = tag_detector.detect(gray)
#     for tags in tags_list:
#         # add selectioin on tags (environment disturbance exists)
#         # if tags.tag_id == 0:
#         #     pose, _, _ = tag_detector.detection_pose(tags, camera_params=(F_X, F_Y, C_X, C_Y), tag_size=TAG_SIZE)
#         #     tag_camera_position = pose[:3, 3]
#         #     # rotation = pose[:3, :3]

#         print(tag)

# if __name__ == '__main__':

#     pipeline, tag_detector = camera_init()
#     while(1):
#         tag_detect(pipeline, tag_detector)
    





def main():
    # Configure the RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start the RealSense pipeline
    pipeline.start(config)

    # Create an AprilTag detector
    detector = apriltag.Detector(apriltag.DetectorOptions(families='tag16h5'))

    try:
        while True:
            # Wait for a new frame from the RealSense camera
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            if not color_frame:
                continue

            # Convert the RealSense frame to a numpy array
            color_image = np.asanyarray(color_frame.get_data())

            # Convert the color image to grayscale
            gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            # Detect AprilTags in the grayscale image
            tags_list = detector.detect(gray_image)

            # Draw AprilTag markers on the color image
            for tag in tags_list:
                # detection.draw(color_image)
                # print(tag)
                pose, _, _ = detector.detection_pose(tag, camera_params=(F_X, F_Y, C_X, C_Y), tag_size=TAG_SIZE)
                # tag_camera_position = pose[:3, 3]
                print(pose)
                # rotation = pose[:3, :3]

            # Display the annotated color image
            cv2.imshow("AprilTag Detection", color_image)

            # Exit the program if the 'q' key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Clean up
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
