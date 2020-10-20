# Multi_camera_calibration
## Dependencies
Allows easy calibration of cameras using a chessboard checkerboard calibration target
The use of this library requires
-cv2
-numpy
-tqdm 
-rospy

## Usage
  ```roslaunch multi_camera_calibration chessboard_capture.launch```
Subsribes to ros image topics from left and right camera presents a live video feed published in ```/image_pair```. The program captures checkerboard pattern visible in both left and right camera. The number of images to be captured, delay between capturing images, rows, columns and square-size of the checkerboard pattern can specified in the launch file



