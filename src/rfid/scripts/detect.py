import cv2
import numpy as np
import pyrealsense2 as rs
import apriltag

TAG_SIZE = 0.05
F_X, F_Y = 649.376, 649.376 # focal_length
C_X, C_Y = 648.137, 353.517 # principal_point

video_path = '1.mp4'



def main():
    # Configure the RealSense pipeline
    cap = cv2.VideoCapture(video_path)

# Check if the video file was opened successfully
    if not cap.isOpened():
        print("Error: Could not open video file.")
        exit()
        # Create an AprilTag detector
    detector = apriltag.Detector(apriltag.DetectorOptions(families='tag16h5'))

    try:
        while True:
            # Wait for a new frame from the RealSense camera
            ret, frames = cap.read()

            # Convert the color image to grayscale
            gray_image = cv2.cvtColor(frames, cv2.COLOR_BGR2GRAY)

            # Detect AprilTags in the grayscale image
            tags_list = detector.detect(gray_image)

            # Draw AprilTag markers on the color image
            for tag in tags_list:
                # detection.draw(color_image)
                # print(tag)
                pose, _, _ = detector.detection_pose(tag, camera_params=(F_X, F_Y, C_X, C_Y), tag_size=TAG_SIZE)
                tag_camera_position = pose[:3, 3]
                print(tag_camera_position)
                # rotation = pose[:3, :3]

            # Display the annotated color image
            cv2.imshow("AprilTag Detection", frames)

            # Exit the program if the 'q' key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Clean up
      
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
