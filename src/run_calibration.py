#!/usr/bin/env python

import rospy
import sys
from calibration import StereoCalibration

def main():
    rospy.init_node('calibration', anonymous=True)
    args = rospy.myargv(argv=sys.argv)
    if len(args)!=2:
        print ("[ERROR]: please provide <output>.pickle")
        sys.exit(1)

    output_file = args[1]

    rows = rospy.get_param("/chessboard_capture/rows")
    columns = rospy.get_param("chessboard_capture/columns")
    square_size = rospy.get_param("chessboard_capture/square-size")
    calibration_images = rospy.get_param("chessboard_capture/calibration_images")

    s = StereoCalibration(calibration_images, rows, columns, square_size)
    s.calibrate(output_file)
    

if __name__ == '__main__':
    main()
