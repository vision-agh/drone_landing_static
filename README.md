# drone_landing_static
Software model - "A Vision Based Hardware-software Real-time Control System for the Autonomous Landing of an UAV"


# Abstract

In this paper we present a vision based hardware-software control system enabling the autonomous landing of a multirotor unmanned aerial vehicle (UAV). It allows for the detection of a marked landing pad in real-time for a 1280 x 720 @ 60 fps video stream. In addition, a LiDAR sensor is used to measure the altitude above ground. A heterogeneous Zynq SoC device is used as the computing platform. The solution was tested on a number of sequences and the landing pad was detected with 96% accuracy. This research shows that a reprogrammable heterogeneous computing system is a good solution for UAVs because it enables real-time data stream processing with relatively low energy consumption.


# Instruction

The OpenCV library in version 4.1.0 was used in this project, please make sure you have it installed and configured properly in order to run the application.

In the dataset directory there are three sets of test images. In `test_set1` there are 32 images with the size of the marker determined manually and placed in the `test_set1_data.csv` file. In `test_set2` there are 18 images with the altitude data from Pixhawk controller and placed in the `test_set2_data.csv` file. In `test_set3` there are 25 images with the altitude data obtained with LIDAR and placed in the `test_set3_data.csv` file. All 75 images are in HD resolution (parameters `WIDTH 1280`, `HEIGHT 720`).

The size of the windows for thresholding can be configured, but for now only 128 x 128 was used (parameters `pSX 128`, `pSY 128`).

If you use a dataset with many images, you may want to skip some frames from the first one (parameter `START_FRAME 0`) or some subsequent frames (parameter `STEP 1`).

More test sets will be added later and synchronized with the data from LIDAR. The mounting of the camera is now changed so there are no visible parts of the drone in the image - that is done for the newest test set (and it will be the standard for the next sets). For the older ones (`test_set1` and `test_set2`) you can use the provided mask (parameter `UAV_MASK 1`) to avoid the possible problems with marker detection.

If you use the altitude data from Pixhawk or LIDAR, you may want to average the values, but do it only if the differences between subsequent frames and measurements are small. Otherwise process each value separately (parameter `AVERAGE 0`).

You can choose between three provided test sets or use your own set (with our marker to detect it correctly, the marker is provided in the repository). Put a full or relative path to the directory with images (`PATH`). The size of the marker determined manually or the altitude data from either Pixhawk or LIDAR is read from the .csv file (`PATH_HEIGHT`).

If you use the marker size in pixels instead of the altitude, use the `test_set1_data.csv` file as an example with the header `(Frame_ID, Circle size [pix])` - it is the default mode. If you use the altitude data from Pixhawk, put it in metres into the file as in the `test_set2_data.csv` file and put "Pixhawk" in the header `(Frame_ID Pixhawk, Altitude [m])`. If you use the altitude data from LIDAR, put it in metres into the file as in the `test_set3_data.csv` file and put "LIDAR" in the header `(Frame_ID LIDAR, Altitude [m])`.

The logs with the number of objects from CCL, the position of the marker and its offset from the centre of the image along with its orientation are written to the .csv file (`PATH_LOGS`). The mask used to remove visible parts of the drone from the image is read from the .png file (`PATH_MASK`).

If you have any questions about the application, feel free to contact us.


# Versions

01.04.2020 - Version 1.0

07.07.2020 - Version 2.0
