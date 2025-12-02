REQUIREMENTS = '''
opencv-python==4.8.0
numpy
PyYAML
'''

#CODE = '''
from pathlib import Path
import yaml
import rospy
from sensor_msgs.msg import CameraInfo, Image
from image_geometry import PinholeCameraModel
import cv2 as cv
#import tf2_geometry_msgs  
#import tf2_ros 
#from geometry_msgs.msg import PointStamped, Point
import numpy as np
from scipy.spatial.transform import Rotation as R

rospy.init_node("task_K")


def find_point(path: Path):
    img = cv.imread(str(path))
    M = cv.moments(img) 
    if M["m00"] != 0:
        x = int(M["m10"] / M["m00"])
        y = int(M["m01"] / M["m00"])
    return x, y


def get_cam_info(path: Path):
    with open(str(path), "r") as f:
        data = yaml.safe_load(f)

    cam_info = CameraInfo()
    cam_info.header.frame_id = data["header"]["frame_id"]
    cam_info.width = data["width"]
    cam_info.height = data["height"]
    cam_info.distortion_model = data["distortion_model"]
    cam_info.D = data["D"]
    cam_info.K = data["K"]
    cam_info.R = data["R"]
    cam_info.P = data["P"]

    return cam_info


def transform(path: Path):
    with open(str(path), "r") as f:
        data = yaml.safe_load(f)

    x = data["transform"]["translation"]["x"]
    y = data["transform"]["translation"]["y"]
    z = data["transform"]["translation"]["z"]

    qx = data["transform"]["rotation"]["x"]
    qy = data["transform"]["rotation"]["y"]
    qz = data["transform"]["rotation"]["z"]
    qw = data["transform"]["rotation"]["w"]

    T = np.array([x, y, z])

    rot = R.from_quat([qx, qy, qz, qw])
    R_mat = rot.as_matrix()
    

    return R_mat, T, z


def get_marker_coordinates_in_aruco_map(
    image_path: Path, 
    camera_info: Path, 
    camera_aruco_map_transform: Path
) -> tuple[float, float, float]:
    global cords
    camera_info = get_cam_info(camera_info)
    cam = PinholeCameraModel()
    cam.fromCameraInfo(camera_info)

    x, y = find_point(image_path)
    ray = cam.projectPixelTo3dRay((x, y))

    R_mat, T, z = transform(camera_aruco_map_transform)
    point_cam = np.array(ray) * z

    point = R_mat @ point_cam + T

    print(tuple(point.tolist()))

    return tuple(point.tolist())
#'''

#ANSWER = (1.2, 4.5, 0.0)

if __name__ == '__main__':
    path = "/home/clover/task_K/"
    img_path = path + "image.png"
    cam_path = path + "camera_info.yml"
    trans_path = path + "transform.yml"
    get_marker_coordinates_in_aruco_map(img_path, cam_path, trans_path)
