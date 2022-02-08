# Augmented Reality using ArUco Markers

This repository contains a project developed in *Python* that allows you to project images, videos and 3D models on the well-known ArUco markers. This project contains sample images and models to make the program work, but you can add others if you wish.

## Installation

To make the project work, the libraries used must be installed. To do this you can use the package manager [pip](https://pip.pypa.io/en/stable/).

The commands to install the libraries are the following:

* ***OpenCV Full Package:***

```bash
pip install opencv-contrib-python
```

* ***NumPy:***

```bash
pip install numpy
```

* ***Math:***

```bash
pip install python-math
```

* ***Os:***

```bash
pip install os-sys
```

It is necessary to install *OpenCV Full Package* because it contains the *aruco* module used to work with the markers. If you have installed the version of *OpenCV* that only contains the main modules, you must uninstall it and then install the *Full Package* version. To do this you can use the following command:

```bash
pip uninstall opencv-python
```

## Usage

At the root of the project there are four files that you can execute to carry out a given function. In the *main* header of each file you can find a box containing the different variables that the program uses and that the user can modify for its use. 

The variables to be modified for each file and their corresponding use are listed below:

### augment_images.py

Projects images onto ArUco markers.

```bash
    ###############################################################
    # Code to change for your convenience

    images_path = 'images'
    camera = 0
    ###############################################################
```

* **images_path:** Path to the folder where the different images to be projected on the ArUco markers are stored. Each of the images must be named with the identifier of the ArUco marker on which you want to project the image, followed by "_" and optionally a name describing the image.

* **camera:** Positive number that identifies the camera connected to the PC that you want to use for the execution. If only one camera is connected, it should be left at 0. Otherwise, it should be changed to the number that identifies the camera to be used.

### augment_videos.py

Projects a video onto ArUco markers.

```bash
    ###############################################################
    # Code to change for your convenience

    video_path = 'videos/video.mp4'
    arucos_list = [23, 62]
    camera = 0
    ###############################################################
```

* **video_path:** Path of the video to be projected.

* **arucos_list:** Identifiers of the ArUco markers on which you want the video to be played.

* **camera:** Positive number that identifies the camera connected to the PC that you want to use for the execution. If only one camera is connected, it should be left at 0. Otherwise, it should be changed to the number that identifies the camera to be used.

### augment_models.py

Projects 3D models onto ArUco markers.

```bash
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
```

* **models_path:** Path to the folder where the different 3D models to be projected on the ArUco markers are stored. Each of the models must be named with the identifier of the ArUco marker on which you want to project the model, followed by "_" and optionally a name describing the model. They must be in *.obj* format.

* **calibration_path:** Path to the folder where the different images of a checkerboard taken for camera calibration are stored.

* **homography_image_path:** Path of any image used to calculate the homography matrix needed to calculate the projection matrix.

* **scales_dict:** Dictionary which stores the scale for each 3D model. The keys must store the identifier of the ArUco marker that projects the 3D model, and the values must store the desired scale for that model.

* **colors_dict:** Dictionary which stores the color for each 3D model. The keys must store the identifier of the ArUco marker that projects the 3D model, and the values must store the desired color for that model. The color format is in *BGR*.

* **chessboard_dimensions:** Dimensions of the chessboard used in camera calibration.

* **camera_resolution:** Resolution of the camera used.

* **camera:** Positive number that identifies the camera connected to the PC that you want to use for the execution. If only one camera is connected, it should be left at 0. Otherwise, it should be changed to the number that identifies the camera to be used.

### show_patterns.py

Displays the calculated patterns of each image used in the camera calibration process.

```bash
    ###############################################################
    # Code to change for your convenience

    calibration_path = 'chessboard/PC_camera'
    chessboard_dimensions = (9, 7)
    camera_resolution = (1920, 1080)
    ###############################################################
```

* **calibration_path:** Path to the folder where the different images of a checkerboard taken for camera calibration are stored.

* **chessboard_dimensions:** Dimensions of the chessboard used in camera calibration.

* **camera_resolution:** Resolution of the camera used.
