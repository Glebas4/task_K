REQUIREMENTS = '''
opencv-python==4.8.0
numpy==1.23.5
PyYAML
scipy==1.3.0
'''


CODE = '''
from pathlib import Path
import yaml
import numpy as np
import cv2 as cv
from scipy.spatial.transform import Rotation as R


def load_camera_info(path: Path):
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    K = np.array(data["K"]).reshape(3, 3)
    return K


def load_transform(path: Path):
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    t = np.array([
        data["transform"]["translation"]["x"],
        data["transform"]["translation"]["y"],
        data["transform"]["translation"]["z"]
    ])

    q = [
        data["transform"]["rotation"]["x"],
        data["transform"]["rotation"]["y"],
        data["transform"]["rotation"]["z"],
        data["transform"]["rotation"]["w"]
    ]

    R_mat = R.from_quat(q).as_matrix()

    return R_mat, t


def find_centroid(path: Path):
    img = cv.imread(str(path))
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    M = cv.moments(gray)
    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]
    return cx, cy


def get_marker_coordinates_in_aruco_map(
    image_path: Path,
    camera_info: Path,
    camera_aruco_map_transform: Path
) -> tuple[float, float, float]:

    cx, cy = find_centroid(image_path)
    K = load_camera_info(camera_info)
    R_cam, t_cam = load_transform(camera_aruco_map_transform)

    # луч
    fx = K[0, 0]
    fy = K[1, 1]
    cx0 = K[0, 2]
    cy0 = K[1, 2]

    x = (cx - cx0) / fx
    y = (cy - cy0) / fy
    ray = np.array([x, y, 1.0])

    # камера находится на высоте t_cam[2]
    Z_cam = t_cam[2]

    # точка в координатах камеры при Z_world = 0
    s = -Z_cam / (R_cam[2] @ ray)
    p_cam = ray * s

    # мировые координаты
    p_world = R_cam @ p_cam + t_cam

    return (
        float(round(p_world[0], 2)),
        float(round(p_world[1], 2)),
        float(round(p_world[2], 2)),
    )
'''

ANSWER = (2.08, 7.58, 0.0)
