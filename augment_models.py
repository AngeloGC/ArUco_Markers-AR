import cv2
import cv2.aruco as aruco
import math

from resources.camera_calibration import *
from resources.objloader import *


def load_aug_models(path):
    """
    :param path: folder in which all the 3D models with ids are stored
    :return: dictionary with key as the id and values as the augment model
    """
    my_list = os.listdir(path)
    no_of_models = len(my_list)
    print("Total Number of Models Detected:", no_of_models)

    models_dict = {}
    for model_path in my_list:
        obj = OBJ(f'{path}/{model_path}', swapyz=True)
        key = int(model_path.split('_')[0])
        models_dict[key] = obj

    return models_dict


def find_aruco_markers(img, marker_size=6, total_markers=250, draw_bbox=True):
    """
    :param img: image in which to find the aruco images
    :param marker_size: the size of the images
    :param total_markers: total number of images that compose the dictionary
    :param draw_bbox: flag to draw bbox around images detected
    :return: bounding boxes and id numbers of images detected
    """
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = getattr(aruco, f'DICT_{marker_size}X{marker_size}_{total_markers}')
    aruco_dict = aruco.Dictionary_get(key)
    bboxes, ids, _ = aruco.detectMarkers(img_gray, aruco_dict)

    if draw_bbox:
        aruco.drawDetectedMarkers(img, bboxes)

    return [bboxes, ids]


def projection_matrix(camera_parameters, homography):
    """
    :param camera_parameters: camera matrix calculated from camera calibration
    :param homography: homography matrix needed to augment
    :return: projection matrix needed to augment 3D points
    """
    # Compute rotation along the x and y axis as well as the translation
    rot_and_transl = np.dot(np.linalg.inv(camera_parameters), -homography)
    col_1 = rot_and_transl[:, 0]
    col_2 = rot_and_transl[:, 1]
    col_3 = rot_and_transl[:, 2]

    # Normalise vectors
    l = math.sqrt(np.linalg.norm(col_1) * np.linalg.norm(col_2))
    rot_1 = col_1 / l
    rot_2 = col_2 / l
    translation = col_3 / l

    # Compute the orthonormal basis
    c = rot_1 + rot_2
    p = np.cross(rot_1, rot_2)
    d = np.cross(c, p)
    rot_1 = np.dot(c / np.linalg.norm(c, 2) + d / np.linalg.norm(d, 2), 1 / math.sqrt(2))
    rot_2 = np.dot(c / np.linalg.norm(c, 2) - d / np.linalg.norm(d, 2), 1 / math.sqrt(2))
    rot_3 = np.cross(rot_1, rot_2)

    # Finally, compute the 3D projection matrix from the model to the current frame
    projection = np.stack((rot_1, rot_2, rot_3, translation)).T

    return np.dot(camera_parameters, projection)


def render(img, obj, projection, model, scale, bgr_model_color):
    """
    :param img: the final image on which to draw
    :param obj: the model that will be overlapped on the marker
    :param projection: projection matrix needed to augment 3D points
    :param model: image used for homography
    :param scale: scale factor to modify the size of the 3D model
    :param bgr_model_color: color for 3D model
    :return: image with the augment model overlaid
    """
    vertices = obj.vertices
    scale_matrix = np.eye(3) * scale
    h, w, _ = model.shape

    for face_vertices in obj.faces:
        points = np.array([vertices[vertex - 1] for vertex in face_vertices])
        points = np.dot(points, scale_matrix)
        points = np.array([[p[0] + w / 2, p[1] + h / 2, p[2]] for p in points])
        dst = cv2.perspectiveTransform(points.reshape(-1, 1, 3), projection)
        imgpts = np.int32(dst)
        cv2.fillConvexPoly(img, imgpts, bgr_model_color)

    return img


def augment_aruco_3d(bbox, id, img, homography_image, camera_matrix,
                     obj, scale=1.0, bgr_model_color=(255, 255, 255), draw_id=False):
    """
    :param bbox: the four corner points of the box
    :param id: market id of the corresponding box used only for display
    :param img: the final image on which to draw
    :param homography_image: image used for homography
    :param camera_matrix: camera matrix calculated from camera calibration
    :param obj: the model that will be overlapped on the marker
    :param scale: scale factor to modify the size of the 3D model
    :param bgr_model_color: color for 3D model
    :param draw_id: flag to display the id of the detected images
    :return: image with the augment model overlaid
    """

    tl = int(bbox[0][0][0]), int(bbox[0][0][1])
    tr = int(bbox[0][1][0]), int(bbox[0][1][1])
    br = int(bbox[0][2][0]), int(bbox[0][2][1])
    bl = int(bbox[0][3][0]), int(bbox[0][3][1])

    h, w, _ = homography_image.shape
    bbox_pts = np.array([tl, tr, br, bl])
    aruco_pts = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    matrix, _ = cv2.findHomography(aruco_pts, bbox_pts)
    projection = projection_matrix(camera_matrix, matrix)
    img_out = render(img, obj, projection, homography_image, scale, bgr_model_color)

    if draw_id:
        cv2.putText(img_out, str(id), tl, cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 255), 2)

    return img_out


def main():
    ###############################################################
    # Code to change for your convenience

    models_path = 'models'
    calibration_path = 'chessboard/PC_camera'
    homography_image_path = 'other/23.jpg'

    scales_dict = {23: 0.4, 62: 0.2}
    colors_dict = {23: (153, 153, 153), 62: (255, 0, 0)}
    chessboard_dimensions = (9, 7)
    camera_resolution = (1920, 1080)
    camera = 0
    ###############################################################

    homography_image = cv2.imread(homography_image_path)
    cap = cv2.VideoCapture(camera, cv2.CAP_DSHOW)
    models_dict = load_aug_models(models_path)
    camera_matrix = camera_calibration(chessboard_dimensions, camera_resolution, calibration_path)

    while True:
        _, img = cap.read()
        aruco_found = find_aruco_markers(img)

        if len(aruco_found[0]) != 0:
            for bbox, id in zip(aruco_found[0], aruco_found[1]):
                if int(id) in models_dict.keys():
                    img = augment_aruco_3d(bbox, id, img, homography_image, camera_matrix,
                                           models_dict[int(id)], scales_dict[int(id)], colors_dict[int(id)])

        cv2.imshow("Camera", img)
        cv2.waitKey(1)

        if not cv2.getWindowProperty("Camera", cv2.WND_PROP_VISIBLE):
            break


if __name__ == "__main__":
    main()
