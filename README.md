# Multi_camera_calibration
## Dependencies
Allows easy calibration of cameras using a chessboard checkerboard calibration target
The use of this library requires
- cv2
- numpy
- tqdm 
- rospy

## Usage
  ```roslaunch multi_camera_calibration chessboard_capture.launch```
Subsribes to ros image topics from left and right camera presents a live video feed published in ```/image_pair```. The program captures checkerboard pattern visible in both left and right camera. The number of images to be captured, folder to save images, delay between capturing images, rows, columns and square-size of the checkerboard pattern can specified in the launch file. Images are only captured if detected pattern is visible in both images

### Subscribed Topics
- camera1 (sensor_msgs/Image) 
  - raw left image topic, for stereo cameras 
- camera2 (sensor_msgs/Image) 
  - raw right image topic, for stereo cameras

### Published Topics
- /image_pair (sensor_msgs/Image) 
  - live video feed with detected checkerboard patterns displayed

After capturing images,  the stereo calibration routine can be run the calibration parameters are saved in pickle format
```rosrun multi_camera_calibration run_calibration.py <filename>.pickle```





