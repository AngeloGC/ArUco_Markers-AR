import cv2
import cv2.aruco as aruco
import numpy as np
import os


def load_aug_images(path):
    """
    :param path: folder in which all the marker images with ids are stored
    :return: dictionary with key as the id and values as the augment image
    """
    my_list = os.listdir(path)
    no_of_markers = len(my_list)
    print("Total Number of Markers Detected:", no_of_markers)

    images_dict = {}
    for img_path in my_list:
        key = int(img_path.split('_')[0])
        img_aug = cv2.imread(f'{path}/{img_path}')
        images_dict[key] = img_aug

    return images_dict


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


def augment_aruco(bbox, id, img, aug_img, draw_id=False):
    """
    :param bbox: the four corner points of the box
    :param id: market id of the corresponding box used only for display
    :param img: the final image on which to draw
    :param aug_img: the image that will be overlapped on the marker
    :param draw_id: flag to display the id of the detected images
    :return: image with the augment image overlaid
    """
    tl = int(bbox[0][0][0]), int(bbox[0][0][1])
    tr = int(bbox[0][1][0]), int(bbox[0][1][1])
    br = int(bbox[0][2][0]), int(bbox[0][2][1])
    bl = int(bbox[0][3][0]), int(bbox[0][3][1])

    h, w, _ = aug_img.shape
    bbox_pts = np.array([tl, tr, br, bl])
    aug_img_pts = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    matrix, _ = cv2.findHomography(aug_img_pts, bbox_pts)
    img_out = cv2.warpPerspective(aug_img, matrix, (img.shape[1], img.shape[0]))
    cv2.fillConvexPoly(img, bbox_pts, (0, 0, 0))
    img_out = img + img_out

    if draw_id:
        cv2.putText(img_out, str(id), tl, cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 255), 2)

    return img_out


def main():
    ###############################################################
    # Code to change for your convenience

    images_path = 'images'
    camera = 0
    ###############################################################

    images_dict = load_aug_images(images_path)
    cap = cv2.VideoCapture(camera, cv2.CAP_DSHOW)

    while True:
        _, img = cap.read()
        aruco_found = find_aruco_markers(img)

        if len(aruco_found[0]) != 0:
            for bbox, id in zip(aruco_found[0], aruco_found[1]):
                if int(id) in images_dict.keys():
                    img = augment_aruco(bbox, id, img, images_dict[int(id)])

        cv2.imshow("Camera", img)
        cv2.waitKey(1)

        if not cv2.getWindowProperty("Camera", cv2.WND_PROP_VISIBLE):
            break


if __name__ == "__main__":
    main()
