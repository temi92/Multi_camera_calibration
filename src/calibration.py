import cv2
import numpy as np
import time
import os
from tqdm import tqdm
import pickle
class CalibrationException(Exception):
    pass

class MonoCalibration(object):
    def __init__(self, image_size, camera_position, rows, columns, square_size):
        self.image_size = image_size
        self.pattern_size = (rows, columns)
        self.square_size = square_size
        self.obj_points = []
        self.img_points = []
        self.camera_position = camera_position
      


    def _object_points(self):
        corner_coordinates = np.zeros((np.prod(self.pattern_size),3), np.float32)
        corner_coordinates[:, :2] = np.indices((self.pattern_size)).T.reshape(-1, 2)
        corner_coordinates *= self.square_size
        return corner_coordinates

    def _get_corners(self, img):
            """
            Get corners for a particular chessboard for an image
            """
            #ensure img is in gray scale format
            if len(img.shape) == 3 and img.shape[2] == 3:
                mono = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            else:
                mono = img
            found, corners = cv2.findChessboardCorners(mono,self.pattern_size)
            

            if found:
                cv2.cornerSubPix(mono, corners, (11, 11), (-1, -1),
                                (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS,
                                30, 0.01))
            else:
                raise CalibrationException("no checkerboard found")
                
                                    
            return corners

    def collect_corners(self, image_files):
        for file_name in tqdm(image_files, desc="detecting corner points in %s images." %self.camera_position):
            self.obj_points.append(self._object_points())
            im = cv2.imread(file_name, 0)
            corners = self._get_corners(im).reshape(-1, 2)
            self.img_points.append(corners)
    
    def calibrate(self):
        """
        perform monocular camera calibration using the object points and detected image points.
        """

        # If FIX_ASPECT_RATIO flag set, enforce focal lengths have 1/1 ratio
        intrinsics_in = np.eye(3, dtype=np.float64)


        rms_error, self.intrinsics, dist_coeffs, self.rvecs,self.tvecs = cv2.calibrateCamera(
                       self.obj_points, self.img_points,
                       self.image_size,
                       intrinsics_in,
                       None,
                       flags = 0)
        
        assert rms_error < 1.0, "[ERROR] Calibration RMS error < 1.0 (%i). Re-try image capture." % (rms_error)
        print("[OK] Calibration successful %s RMS error=" %(self.camera_position) + str(rms_error))

        self.distortion = dist_coeffs.flat[:8].reshape(-1, 1)
        self.reprojectionError()
                
    def report(self):
        print("K =", np.ravel(self.intrinsics).tolist())
        print ("dist_coeff = ", np.ravel(self.distortion))

       

    def reprojectionError(self):
        mean_error = 0
        for i in xrange(len(self.obj_points)):
            imgpoints2, _ = cv2.projectPoints(self.obj_points[i], self.rvecs[i], self.tvecs[i], self.intrinsics, self.distortion)
            error = cv2.norm(self.img_points[i], imgpoints2.reshape(-1,2), cv2.NORM_L2) / len(imgpoints2)
            mean_error +=error
        print("Re-projection Error: {}".format(mean_error / len(self.obj_points)))

class StereoCalibration(object):
    
    camera_positions = ["left camera", "right_camera"]


    def __init__(self, image_paths, *args, **kwargs):
        files = StereoCalibration.find_files(image_paths)
       
        self.left_images = files[0]
        self.right_images = files[1]
        assert (len(self.left_images)== len(self.right_images))
         #get image size..
        img = cv2.imread(self.left_images[0])
        height, width = img.shape[:2]
        image_size = (width, height)
        self.image_size = image_size

        self.l = MonoCalibration(image_size, self.camera_positions[0], *args, **kwargs)
        self.r = MonoCalibration(image_size, self.camera_positions[1], *args, **kwargs)

    def calibrate(self, filename):
        self.l.collect_corners(self.left_images)
        self.l.calibrate()
        self.r.collect_corners(self.right_images)
        self.r.calibrate()

        print("[INFO] calibrating stereo pair...")
        #flags = (cv2.CALIB_FIX_ASPECT_RATIO + cv2.CALIB_ZERO_TANGENT_DIST +
                 #cv2.CALIB_SAME_FOCAL_LENGTH)
        #flags = (cv2.CALIB_USE_INTRINSIC_GUESS)
        flags = (cv2.CALIB_FIX_INTRINSIC)
        
        criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS,
                    100, 1e-5)


       
        rms, intrinsics_left, dist_left, intrinsics_right, dist_right, rot, trans, _, _ = cv2.stereoCalibrate(self.l.obj_points, self.l.img_points, 
                            self.r.img_points, self.l.intrinsics,
                            self.l.distortion, self.r.intrinsics, 
                            self.r.distortion, self.image_size, 
                            None, None, 
                            criteria=criteria, flags=flags)

        assert rms < 1.0, "[ERROR] Calibration STEREO RMS error < 1.0 (%i). Re-try image capture." % (rms)
        print("[OK] STEREO Calibration successful RMS error: %.4f" %(rms))

        print ("[INFO] rms stero %.4f" %rms)

        print("T =", np.ravel(trans).tolist())
        print("R =", np.ravel(rot).tolist())
        
        #save calibration results.
        dist_pickle = {}
        dist_pickle["left_intrinsic"] = intrinsics_left
        dist_pickle["left_dist"] = dist_left
        dist_pickle["right_intrinsic"] = intrinsics_right
        dist_pickle["right_dist"] = dist_right
        dist_pickle["rotation_matrix"] = rot
        dist_pickle["trans_vec"] = trans

        pickle_file = open(filename, "wb")
        pickle.dump(dist_pickle, pickle_file)
        pickle_file.close()

       

        
                                   
    @staticmethod
    def find_files(folder):
    
        left_images = [i for i in os.listdir(folder) if i.startswith("left")]
        left_images.sort()
        left_images = [os.path.join(folder, filename) for filename in left_images]
        right_images = [i for i in os.listdir(folder) if i.startswith("right")]
        right_images.sort()
        right_images = [os.path.join(folder, filename) for filename in right_images]
        return left_images, right_images
    
