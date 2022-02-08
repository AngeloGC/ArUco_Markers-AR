from resources.camera_calibration import *


def main():
    ###############################################################
    # Code to change for your convenience

    calibration_path = 'chessboard/PC_camera'
    chessboard_dimensions = (9, 7)
    camera_resolution = (1920, 1080)
    ###############################################################

    camera_calibration(chessboard_dimensions, camera_resolution, calibration_path, True)


if __name__ == "__main__":
    main()
