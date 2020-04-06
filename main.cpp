/*
    MIT License

    Copyright (c) 2020
    Krzysztof Blachut, Hubert Szolc, Mateusz Wasala, Tomasz Kryjak, Marek Gorgon
    email: {kblachut, szolc, wasala, tomasz.kryjak, mago} @agh.edu.pl
    Computer Vision Laboratory
    Department of Automatic Control and Robotics
    Faculty of Electrical Engineering, Automatics, Computer Science and Biomedical Engineering
    AGH University of Science and Technology, Krakow, Poland

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

/*  Versions:
    01.04.2020 - Version 1.0

*/

/*
    The OpenCV library in version 4.1.0 was used in this project, please make sure you have it installed
    and configured properly in order to run the application.

    In the dataset directory there are two sets of test images. In test_set1 there are 32 images
    with the size of the marker determined manually and placed in the test_set1_data.csv file.
    In test_set2 there are 18 images with the altitude data from Pixhawk controller and placed
    in the test_set2_data.csv file. All 50 images are in HD resolution (parameters WIDTH 1280, HEIGHT 720).

    The size of the windows for thresholding can be configured, but for now only 128x128 was used
    (parameters pSX 128, pSY 128).

    If you use a dataset with many images, you may want to skip some frames from the first one
    (parameter START_FRAME 0) or some subsequent frames (parameter STEP 1).

    New test sets will be added later and synchronized with the data from LIDAR. The mounting
    of the camera will be changed so as not to have visible parts of the drone in the image.
    For now please use the provided mask (parameter UAV_MASK 1) to avoid possible problems
    with marker detection.

    If you use the altitude data from Pixhawk or LIDAR, you may want to average the values,
    but do it only if the differences between subsequent frames and measurements are small.
    Otherwise process each value separately (parameter AVERAGE 0).

    You can choose between two provided test sets or use your own set (with our marker to detect
    it correctly, the marker is provided in the repository). Put a full or relative path to the 
    directory with images (PATH). The size of the marker determined manually or the altitude data 
    from either Pixhawk or LIDAR is read from the .csv file (PATH_HEIGHT).

    If you use the marker size in pixels instead of the altitude, use the test_set1_data.csv
    file as an example with the header (Frame_ID, Circle size [pix]) - it is the default mode.
    If you use the altitude data from Pixhawk, put it in metres into the file
    as in the test_set2_data.csv file and put "Pixhawk" in the header (Frame_ID Pixhawk, Altitude [m]).
    If you use the altitude data from LIDAR, put it in metres into the file
    as in the test_set2_data.csv file and put "LIDAR" in the header (Frame_ID LIDAR, Altitude [m]).

    The logs with the number of objects from CCL, the position of the marker and its offset
    from the centre of the image along with its orientation are written to the .csv file (PATH_LOGS).
    The mask used to remove visible parts of the drone from the image is read from the .png file (PATH_MASK).

    If you have any questions about the application, feel free to contact us.
*/

#include <iostream>
#include <dirent.h>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <opencv2/opencv.hpp>

// -------------------------------------------------------------------------------------------------------------------
// CONSTANT PARAMETERS
// -------------------------------------------------------------------------------------------------------------------

#define M_PI 3.14159265358979323846

// The size of the input image
#define WIDTH 1280
#define HEIGHT 720

// The size of windows for thresholding
#define pSX 128
#define pSY 128

// Start frame and step
#define START_FRAME 0
#define STEP 1

// Mask visible parts of the drone
#define USE_MASK 1

// Average last 5 values from LIDAR or Pixhawk
#define AVERAGE 0

// Path to a directory containing a dataset
const std::string PATH = "dataset/test_set1/";
const std::string PATH_HEIGHT = "dataset/test_set1_data.csv";
const std::string PATH_LOGS = "dataset/test_set1_logs.csv";

#if USE_MASK
const std::string PATH_MASK = "uav_mask.png";
#endif

// -------------------------------------------------------------------------------------------------------------------
int main()
{
    // Open directory with a dataset
    DIR *directory = opendir(PATH.c_str());
    dirent *fileName;
    std::vector<std::string> listOfFiles;

    // Load names of all files
    while ((fileName = readdir(directory)))
    {
        std::string tempName = std::string(fileName->d_name);
        listOfFiles.push_back(tempName);
    }
    closedir(directory);
    std::sort(listOfFiles.begin(), listOfFiles.end());

    // Initialize output file
    std::ofstream outputFile(PATH_LOGS);
    if (outputFile.is_open())
        outputFile << "Number of objects" << ',' << " X centre [pix]" << ','
                   << " Y centre [pix]" << ',' << " X distance to centre [pix]" << ','
                   << " Y distance to centre [pix]" << ',' << " X distance to centre [cm]" << ','
                   << " Y distance to centre [cm]" << ',' << " Orientation angle [deg]" << std::endl;

    // Count the images and initialize tables with the size of a circle on consecutive frames
    int nimages = listOfFiles.end() - listOfFiles.begin() - 2;
    int table_circles[nimages];
    int table_squares[nimages];

    // -------------------------------------------------------------------------------------------------------------------
    // ALTITUDE DATA PROCESSING
    // -------------------------------------------------------------------------------------------------------------------

    // Open file with altitude data
    std::ifstream inputFile;
    inputFile.open(PATH_HEIGHT);
    if (inputFile.is_open())
    {
        std::string header;
        getline(inputFile, header);
        std::string altitude;
        double alt[nimages];
        std::string filename;

        // Use altitude data from LIDAR
        if (header.find("LIDAR") != std::string::npos)
        {
            for (int i = 0; i < nimages; i++)
            {
                getline(inputFile, filename, ',');
                inputFile >> altitude;
                alt[i] = std::stod(altitude) * 100 - 20; // Subtract 20 cm as it's the difference between position of LIDAR and camera in our case
                alt[i] = std::max(std::min(alt[i], 300.0), 0.0);
#if AVERAGE
                if (i >= 4)
                    alt[i] = (alt[i] + alt[i - 1] + alt[i - 2] + alt[i - 3] + alt[i - 4]) / 5;
#endif
                // Convert altitude to expected circle size
                table_circles[i] = int(3.43375604675693e-15 * pow(alt[i], 8) - 4.77097482299868e-12 * pow(alt[i], 7) +
                                   2.79264529276849e-09 * pow(alt[i], 6) - 8.96208749876710e-07 * pow(alt[i], 5) + 0.000172002819190201 * pow(alt[i], 4) -
                                   0.0202309024343306 * pow(alt[i], 3) + 1.44107699084859 * pow(alt[i], 2) - 59.7115579325702 * alt[i] + 1345.58241758463);
            }
        }

        // Use altitude data from Pixhawk
        if (header.find("Pixhawk") != std::string::npos)
        {
            for (int i = 0; i < nimages; i++)
            {
                getline(inputFile, filename, ',');
                inputFile >> altitude;
                alt[i] = std::stod(altitude) * 100;
                alt[i] = std::max(std::min(alt[i], 300.0), 0.0);
#if AVERAGE
                if (i >= 4)
                    alt[i] = (alt[i] + alt[i - 1] + alt[i - 2] + alt[i - 3] + alt[i - 4]) / 5;
#endif
                // Convert altitude to expected circle size
                table_circles[i] = int(- 0.000110976007441677 * pow(alt[i], 3) + 0.0433422982169812 * pow(alt[i], 2) - 6.67322230364540 * alt[i] + 481.517071405531);
            }
        }

		// Use circle size in pixels (default mode)
		else
        {
            for (int i = 0; i < nimages; i++)
            {
                getline(inputFile, filename, ',');
                inputFile >> altitude;
                alt[i] = std::stod(altitude);
                table_circles[i] = int(alt[i]);
            }
        }

    }
    inputFile.close();

    for (int i = 0; i < nimages; i++)
        table_squares[i] = int(table_circles[i] / 5);

    double centre[2] = {0, 0};
    double diff_to_centre[2] = {0, 0};
    double distance_to_centre[2] = {0, 0};
    double angle = 0;

#if USE_MASK
    // Read a mask to remove UAV parts from the image
    cv::Mat mask = cv::imread(PATH_MASK);
    cv::cvtColor(mask, mask, cv::COLOR_BGR2GRAY);
#endif

    // -------------------------------------------------------------------------------------------------------------------
    // CONSECUTIVE FRAMES PROCESSING
    // -------------------------------------------------------------------------------------------------------------------
    for (std::vector<std::string>::iterator it = listOfFiles.begin() + 2 + START_FRAME; it < listOfFiles.end(); it = it + STEP)
    {
        // Load image
        cv::Mat image = cv::imread(PATH + (*it));

        // Display debug info
        std::cout << "Processing file: " << (*it);

        int circle_size = table_circles[it - listOfFiles.begin() - 2];
        int square_size = table_squares[it - listOfFiles.begin() - 2];

        // Conversion to grey and Gaussian blur
        cv::Mat grey = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
        cv::cvtColor(image, grey, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(grey, grey, cv::Size(5, 5), 0.65);

        // Global thresholding
        cv::Mat IB_G = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
        double minG, maxG;
        cv::minMaxLoc(grey, &minG, &maxG);
        unsigned int thrG = (maxG - minG) * 0.3 + minG;
        for (int j = 0; j < WIDTH; j++)
        {
            for (int i = 0; i < HEIGHT; i++)
            {
                IB_G.at<uchar>(i, j) = 255 * (grey.at<uchar>(i, j) < thrG);
            }
        }

        // Local thresholding
        cv::Mat IB_C = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
        int cSize = 7; // Size of the context
        for (int jj = cSize; jj < WIDTH - cSize; jj++)
        {
            for (int ii = cSize; ii < HEIGHT - cSize; ii++)
            {
                cv::Mat patch = grey(cv::Range(ii - cSize, ii + cSize), cv::Range(jj - cSize, jj + cSize));
                unsigned int thrC = (cv::mean(cv::mean(patch).val[0])).val[0];
                IB_C.at<uchar>(ii, jj) = 255 * (grey.at<uchar>(ii, jj) < thrC);
            }
        }

        // Thresholding in windows
        cv::Mat IB = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
        cv::Mat thresholds = cv::Mat::zeros(floor(HEIGHT / pSX) + 1, floor(WIDTH / pSY) + 1, CV_8UC1);

        for (int jj = 0; jj < WIDTH; jj = jj + pSY)
        {
            for (int ii = 0; ii < HEIGHT; ii = ii + pSX)
            {
                cv::Mat patch = grey(cv::Range(ii, std::min(ii + pSX - 1, HEIGHT)), cv::Range(jj, std::min(jj + pSY - 1, WIDTH)));
                double minP, maxP;
                cv::minMaxLoc(patch, &minP, &maxP);
                unsigned int thrW = (maxP - minP) * 0.25 + minP;
                thresholds.at<uchar>(floor(ii / pSX), floor(jj / pSY)) = thrW;

                for (int j = jj; j < std::min(jj + pSY, WIDTH); j++)
                {
                    for (int i = ii; i < std::min(ii + pSX, HEIGHT); i++)
                    {
                        IB.at<uchar>(i, j) = 255 * (patch.at<uchar>(i % 128, j % 128) < thrW);
                    }
                }
            }
        }

        // Interpolation of image thresholded in windows
        cv::Mat IB_L = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
        int iT, jT;
        double dX1, dX2, dY1, dY2;
        int th11, th12, th21, th22;
        double th1, th2, th;

        for (int jj = 0; jj < WIDTH; jj++)
        {
            for (int ii = 0; ii < HEIGHT; ii++)
            {
                // Corners of the image
                if (ii < pSX / 2 && jj < pSY / 2 || ii > (HEIGHT - pSX / 2) && jj < pSY / 2 || ii < pSX / 2 && jj > (WIDTH - pSY / 2) || ii > (HEIGHT - pSX / 2) && jj > (WIDTH - pSY / 2))
                {
                    iT = floor(ii / pSX);
                    jT = floor(jj / pSY);
                    th11 = thresholds.at<uchar>(iT, jT);
                    th12 = thresholds.at<uchar>(iT, jT);
                    th21 = thresholds.at<uchar>(iT, jT);
                    th22 = thresholds.at<uchar>(iT, jT);
                }

                // Horizontal borders of the image
                if (ii > pSX / 2 && ii <= (HEIGHT - pSX / 2) && jj < pSY / 2 || ii > pSX / 2 && ii <= (HEIGHT - pSX / 2) && jj > (WIDTH - pSY / 2))
                {
                    iT = floor((ii - pSX / 2) / pSX);
                    jT = floor((jj - pSY / 2) / pSY);
                    th11 = thresholds.at<uchar>(iT, jT);
                    th12 = thresholds.at<uchar>(iT + 1, jT);
                    th21 = thresholds.at<uchar>(iT, jT);
                    th22 = thresholds.at<uchar>(iT + 1, jT);
                }

                // Vertical borders of the image
                if (jj > pSY / 2 && jj <= (WIDTH - pSY / 2) && ii < pSX / 2 || jj > pSY / 2 && jj <= (WIDTH - pSY / 2) && ii > (HEIGHT - pSX / 2))
                {
                    iT = floor((ii - pSX / 2) / pSX);
                    jT = floor((jj - pSY / 2) / pSY);
                    th11 = thresholds.at<uchar>(iT, jT);
                    th12 = thresholds.at<uchar>(iT, jT);
                    th21 = thresholds.at<uchar>(iT, jT + 1);
                    th22 = thresholds.at<uchar>(iT, jT + 1);
                }

                // Inside of image
                if (ii >= pSX / 2 && ii < (HEIGHT - pSX / 2) && jj >= pSY / 2 && jj < (WIDTH - pSY / 2))
                {
                    iT = floor((ii - pSX / 2) / pSX);
                    jT = floor((jj - pSY / 2) / pSY);
                    th11 = thresholds.at<uchar>(iT, jT);
                    th12 = thresholds.at<uchar>(iT + 1, jT);
                    th21 = thresholds.at<uchar>(iT, jT + 1);
                    th22 = thresholds.at<uchar>(iT + 1, jT + 1);
                }

                dX1 = ii - pSX / 2 - (iT - 0) * pSX;
                dX2 = (iT + 1) * pSX - (ii - pSX / 2);
                dY1 = jj - pSY / 2 - (jT - 0) * pSY;
                dY2 = (jT + 1) * pSY - (jj - pSY / 2);
                th1 = th11 * (dX2 / pSX) + th12 * (dX1 / pSX);
                th2 = th21 * (dX2 / pSX) + th22 * (dX1 / pSX);
                th = th1 * (dY2 / pSY) + th2 * (dY1 / pSY);
                IB_L.at<uchar>(ii, jj) = 255 * (grey.at<uchar>(ii, jj) < th);
            }
        }

        // Erosion 3x3
        cv::Mat IB_E = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
        cv::erode(IB_L, IB_E, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

        // Median filter 5x5
        cv::Mat IB_M = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
        cv::medianBlur(IB_E, IB_M, 5);

        // Dilation 3x3
        cv::Mat IB_D = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
        cv::dilate(IB_M, IB_D, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

#if USE_MASK
        // Use the UAV mask
        for (int jj = 0; jj < WIDTH; jj++)
        {
            for (int ii = 0; ii < HEIGHT; ii++)
            {
                if (mask.at<uchar>(ii, jj) == 0)
                    IB_D.at<uchar>(ii, jj) = 0;
            }
        }
#endif

        // Connected Component Labelling (CCL)
        // Stats: leftmost(x), topmost(y), horizontal size, vertical size, total area
        cv::Mat IB_S, stats, cent;
        cv::connectedComponentsWithStats(IB_D, IB_S, stats, cent, 8, CV_32S, cv::CCL_DEFAULT);
        cv::normalize(IB_S, IB_S, 0, 255, cv::NORM_MINMAX, CV_8UC1, cv::Mat());

        // Convert binary image to 3-channel image
        cv::Mat IB_VIS = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
        cv::Mat channel[3];
        split(IB_VIS, channel);
        channel[0] = IB_D;
        channel[1] = IB_D;
        channel[2] = IB_D;
        merge(channel, 3, IB_VIS);

        // Temporary vectors for centroids (cent) of circles (c), squares (s), rectangles (r)
        std::vector<std::vector<double>> centc;
        std::vector<std::vector<double>> cents;
        std::vector<std::vector<double>> centr;
        // Temporary vectors for bounding boxes (bb) of circles (c), squares (s), rectangles (r)
        std::vector<std::vector<double>> bbc;
        std::vector<std::vector<double>> bbs;
        std::vector<std::vector<double>> bbr;

        // Save to a file the number of objects
        if (outputFile.is_open())
            outputFile << stats.rows << ',';

        // Parameters based on bounding box and area of the object
        double bboxRatio;
        double bboxAreaRatio;
        double meanSquareSize;
        double bboxSize;

        // -------------------------------------------------------------------------------------------------------------------
        // OBJECT PARAMETER ANALYSIS
        // -------------------------------------------------------------------------------------------------------------------
        // Iterate through all detected objects without background
        for (int i = 1; i < stats.rows; i++)
        {
            bboxRatio = (double)std::max(stats.at<int>(i, 2), stats.at<int>(i, 3)) / (double)std::min(stats.at<int>(i, 2), stats.at<int>(i, 3));
            bboxAreaRatio = (double)stats.at<int>(i, 2) * (double)stats.at<int>(i, 3) / (double)stats.at<int>(i, 4);
            meanSquareSize = ((double)stats.at<int>(i, 2) + (double)stats.at<int>(i, 3)) / 2;
            bboxSize = (double)stats.at<int>(i, 2) * (double)stats.at<int>(i, 3);

            /** Detect big circle
             * @param bboxRatio         bounding box ratio is smaller than 1.15 (the shape of bbox is always similar to square)
             * @param bboxAreaRatio     bounding box area ratio is bigger than 2.75 (as it's rather a ring than a circle, many pixels inside bbox don't belong to the object)
             * @param meanSquareSize    mean square size of bounding box is bigger than 0.5 * circle_size (the average of height and width of bbox can't be too small at certain altitude)
             * @param meanSquareSize    mean square size of bounding box is smaller than 1.5 * circle_size (the average of height and width of bbox can't be too big at certain altitude)
             */
            /** Detect small circle
             * @param bboxRatio             bounding box ratio is smaller than 1.15 (the shape of bbox is always similar to square)
             * @param circle_size           circle size is bigger than 150 (the altitude is lower than a certain threshold)
             * @param circle_size           circle size is smaller than 500 (the altitude is higher than a certain threshold)
             * @param bboxAreaRatio         bounding box area ratio is bigger than 1.55 (no more than 65% of pixels inside bbox belong to the object)
             * @param bboxAreaRatio         bounding box area ratio is smaller than 2.0 (at least 50% of pixels inside bbox belong to the object)
             * @param meanSquareSize        mean square size of bounding box is smaller than 1.1 * square_size (the average of height and width of bbox can't be too big at certain altitude)
             * @param stats.at<int>(i, 4)   area of the object is smaller than 0.04 * circle_size * circle_size (the area of object can't be too big at certain altitude)
             * @param stats.at<int>(i, 4)   area of the object is bigger than 0.005 * circle_size * circle_size (the area of object can't be too small at certain altitude)
             or
             * @param bboxRatio             bounding box ratio is smaller than 1.15 (the shape of bbox is always similar to square)
             * @param circle_size           circle size is bigger than 500 (the altitude is lower than a certain threshold)
             * @param bboxAreaRatio         bounding box area ratio is bigger than 1.5 (no more than 67% of pixels inside bbox belong to the object)
             * @param bboxAreaRatio         bounding box area ratio is smaller than 2.0 (at least 50% of pixels inside bbox belong to the object)
             * @param stats.at<int>(i, 4)   area of the object is bigger than 0.01 * circle_size * circle_size (the area of object can't be too small at certain altitude)
             */

            if (bboxRatio < 1.15 && ((bboxAreaRatio > 2.75 && meanSquareSize > 0.5 * circle_size && meanSquareSize < 1.5 * circle_size) ||
                                    (circle_size > 150 && circle_size < 500 && bboxAreaRatio > 1.55 && bboxAreaRatio < 2.0 && meanSquareSize < 1.1 * square_size &&
                                     stats.at<int>(i, 4) < 0.04 * circle_size * circle_size && stats.at<int>(i, 4) > 0.005 * circle_size * circle_size) ||
                                     (circle_size >= 500 && bboxAreaRatio > 1.5 && bboxAreaRatio < 2.0 && stats.at<int>(i, 4) > 0.01 * circle_size * circle_size)))
            {
                // Save the bounding box and centroid
                std::vector<double> temp_bbox;
                std::vector<double> temp_cent;
                for (int j = 0; j < 4; j++)
                    temp_bbox.push_back(stats.at<int>(i, j));
                for (int j = 0; j < 2; j++)
                    temp_cent.push_back(stats.at<int>(i, j) + stats.at<int>(i, j + 2) / 2);
                bbc.push_back(temp_bbox);
                centc.push_back(temp_cent);
            }

            /** Detect square
             * @param bboxRatio             bounding box ratio is smaller than 1.25 (the shape of bbox is similar to square even if it's rotated)
             * @param bboxAreaRatio         bounding box area ratio is smaller than 2.25 (at least 44% of pixels inside bbox belong to the object)
             * @param bboxSize              bounding box size is smaller than 3.0 * square_size * square_size (the bbox can't be too big at certain altitude)
             * @param stats.at<int>(i, 4)   area of the object is bigger than 0.4 * square_size * square_size (the area of object can't be too small at certain altitude)
             * @param stats.at<int>(i, 4)   area of the object is smaller than 1.7 * square_size * square_size (the area of object can't be too small at certain altitude)
             */
            else if (bboxRatio < 1.25 && bboxAreaRatio < 2.25 && bboxSize < 3.0 * square_size * square_size && stats.at<int>(i, 4) > 0.4 * square_size * square_size && stats.at<int>(i, 4) < 1.7 * square_size * square_size)
            {
                // Save the bounding box and centroid
                std::vector<double> temp_bbox;
                std::vector<double> temp_cent;
                for (int j = 0; j < 4; j++)
                    temp_bbox.push_back(stats.at<int>(i, j));
                for (int j = 0; j < 2; j++)
                    temp_cent.push_back(stats.at<int>(i, j) + stats.at<int>(i, j + 2) / 2);
                bbs.push_back(temp_bbox);
                cents.push_back(temp_cent);
            }

            /** Detect rectangle
             * @param bboxRatio             bounding box ratio equals at least 1.0 (the shape of bbox can be similar to square when it's rotated)
             * @param bboxRatio             bounding box ratio is smaller than 2.4 (the difference between height and width of bbox can't be too big)
             * @param bboxAreaRatio         bounding box area ratio is smaller than 2.2 (at least 45% of pixels inside bbox belong to the object)
             * @param bboxSize              bounding box size is bigger than 1.3 * square_size * square_size (the bbox can't be too small at certain altitude)
             * @param stats.at<int>(i, 4)   area of the object is bigger than 0.4 * square_size * square_size (the area of object can't be too small at certain altitude)
             */
            else if (bboxRatio >= 1.0 && bboxRatio < 2.4 && bboxAreaRatio < 2.2 && bboxSize > 1.3 * square_size * square_size && stats.at<int>(i, 4) > 0.4 * square_size * square_size)
            {
                //Save the bounding box and centroid
                std::vector<double> temp_bbox;
                std::vector<double> temp_cent;
                for (int j = 0; j < 4; j++)
                    temp_bbox.push_back(stats.at<int>(i, j));
                for (int j = 0; j < 2; j++)
                    temp_cent.push_back(stats.at<int>(i, j) + stats.at<int>(i, j + 2) / 2);
                bbr.push_back(temp_bbox);
                centr.push_back(temp_cent);
            }
        }

        // Iterate through all detected circles, squares and rectangles
        if ((!bbc.empty()) && (!cents.empty()) && (!centr.empty()))
        {
            for (int i = 0; i < bbc.size(); i++)
            {
                for (int j = 0; j < cents.size(); j++)
                {
                    for (int k = 0; k < centr.size(); k++)
                    {
                        // Check if the centroids of the figures are inside the image
                        if (centc[i][0] > 0 && centc[i][1] > 0 && centc[i][0] < WIDTH && centc[i][1] < HEIGHT &&
							cents[j][0] > 0 && cents[j][1] > 0 && cents[j][0] < WIDTH && cents[j][1] < HEIGHT &&
							centr[k][0] > 0 && centr[k][1] > 0 && centr[k][0] < WIDTH && centr[k][1] < HEIGHT)
                        {
                            // Calculate the distance between the centroids of a square and a rectangle
                            double dist = (cents[j][0] - centr[k][0]) * (cents[j][0] - centr[k][0]) + (cents[j][1] - centr[k][1]) * (cents[j][1] - centr[k][1]);

                            // Check if there is a square and a rectangle inside a circle
                            if (bbc[i][0] < cents[j][0] && bbc[i][0] < centr[k][0] &&
                                bbc[i][1] < cents[j][1] && bbc[i][1] < centr[k][1] &&
                                bbc[i][0] + bbc[i][2] > cents[j][0] && bbc[i][0] + bbc[i][2] > centr[k][0] &&
                                bbc[i][1] + bbc[i][3] > cents[j][1] && bbc[i][1] + bbc[i][3] > centr[k][1])
                            {
                                // Draw the bounding box and the centroid of the circle
                                cv::Rect rectc(bbc[i][0], bbc[i][1], bbc[i][2], bbc[i][3]);
                                cv::rectangle(IB_VIS, rectc, cv::Scalar(0, 0, 255));
                                cv::Point pc;
                                pc.x = centc[i][0];
                                pc.y = centc[i][1];
                                if (pc.x > 0 && pc.y > 0)
                                    cv::circle(IB_VIS, pc, 2, cv::Scalar(0, 0, 255), 2);

                                // Draw the bounding box and the centroid of the square
                                cv::Rect rects(bbs[j][0], bbs[j][1], bbs[j][2], bbs[j][3]);
                                cv::rectangle(IB_VIS, rects, cv::Scalar(0, 255, 0));
                                cv::Point ps;
                                ps.x = cents[j][0];
                                ps.y = cents[j][1];
                                if (ps.x > 0 && ps.y > 0)
                                    cv::circle(IB_VIS, ps, 2, cv::Scalar(0, 255, 0), 2);

                                // Draw the bounding box and the centroid of the rectangle
                                cv::Rect rectr(bbr[k][0], bbr[k][1], bbr[k][2], bbr[k][3]);
                                cv::rectangle(IB_VIS, rectr, cv::Scalar(255, 0, 0));
                                cv::Point pr;
                                pr.x = centr[k][0];
                                pr.y = centr[k][1];
                                if (pr.x > 0 && pr.y > 0)
                                    cv::circle(IB_VIS, pr, 2, cv::Scalar(255, 0, 0), 2);

                                // If the distance is between 20 or 30% (based on the altitude) to 65% of the circle size, draw the line and calculate the position and orientation
                                if (sqrt(dist) < circle_size * 0.65 && ((sqrt(dist) > circle_size * 0.3) || (sqrt(dist) > circle_size * 0.2 && circle_size > 200)))
                                {
                                    cv::Point p1, p2;
                                    p1.x = cents[j][0];
                                    p1.y = cents[j][1];
                                    p2.x = centr[k][0];
                                    p2.y = centr[k][1];
                                    cv::line(IB_VIS, p1, p2, cv::Scalar(0, 255, 255), 2);

                                    // Calculate the position as the average of square's and rectangle's centroid
                                    cv::Point pos;
                                    pos.x = (p1.x + p2.x) / 2;
                                    pos.y = (p1.y + p2.y) / 2;
                                    centre[0] = pos.x;
                                    centre[1] = pos.y;
                                    if (pos.x > 0 && pos.y > 0)
                                        cv::circle(IB_VIS, pos, 2, cv::Scalar(0, 0, 255), 2);
                                    // Calculate the angle, rotate it 90 degrees left (so now 0 degrees is up) and convert to degrees
                                    angle = (atan2(cents[j][1] - centr[k][1], cents[j][0] - centr[k][0]) + M_PI / 2) * 180 / M_PI;
                                    angle = int(angle);
                                }
                            }

                            // Draw the bounding box and the centroid of the small circle if it's between square and rectangle
                            if (circle_size > 150 && sqrt(dist) < circle_size * 0.65 &&
                                centc[i][0] >= std::min(cents[j][0], centr[k][0]) &&
                                centc[i][0] <= std::max(cents[j][0], centr[k][0]) &&
                                centc[i][1] >= std::min(cents[j][1], centr[k][1]) &&
                                centc[i][1] <= std::max(cents[j][1], centr[k][1]))
                            {
                                cv::Rect rectc(bbc[i][0], bbc[i][1], bbc[i][2], bbc[i][3]);
                                cv::rectangle(IB_VIS, rectc, cv::Scalar(0, 0, 255));
                                cv::Point pc;
                                pc.x = centc[i][0];
                                pc.y = centc[i][1];
                                if (pc.x > 0 && pc.y > 0)
                                    cv::circle(IB_VIS, pc, 2, cv::Scalar(0, 0, 255), 2);
                            }
                        }
                    }
                }
            }
        }

        // Save the centre and draw the bounding box and the centroid of the small circle if we're very close to the ground
        if (!bbc.empty() && circle_size > 500)
        {
            for (int i = 0; i < bbc.size(); i++)
            {
                cv::Rect rectc(bbc[i][0], bbc[i][1], bbc[i][2], bbc[i][3]);
                cv::rectangle(IB_VIS, rectc, cv::Scalar(0, 0, 255));
                cv::Point pc;
                pc.x = centc[i][0];
                pc.y = centc[i][1];
                if (pc.x > 0 && pc.y > 0)
                    cv::circle(IB_VIS, pc, 2, cv::Scalar(0, 0, 255), 2);
                centre[0] = pc.x;
                centre[1] = pc.y;
            }
        }

        // Calculate the distance from the centre of the line to the centre of the image (in pixels and centimetres)
        diff_to_centre[0] = centre[0] - WIDTH / 2;
        diff_to_centre[1] = centre[1] - HEIGHT / 2;
        distance_to_centre[0] = (25.0 / circle_size) * diff_to_centre[0];
        distance_to_centre[1] = (25.0 / circle_size) * diff_to_centre[1];
        diff_to_centre[0] = int(diff_to_centre[0]);
        diff_to_centre[1] = int(diff_to_centre[1]);

        // Save to a file the distance from centre in pixels and centimetres
        if (outputFile.is_open())
            outputFile << (int)centre[0] << ',' << (int)centre[1] << ',' << diff_to_centre[0] << ',' << diff_to_centre[1] << ',' << distance_to_centre[0] << ',' << distance_to_centre[1] << ',';

        // Save to a file the orientation angle in degrees
        if (outputFile.is_open())
        {
            outputFile << angle;
            outputFile << '\n';
        }

        cv::namedWindow("Detection result", cv::WINDOW_NORMAL);
        cv::imshow("Detection result", IB_VIS);
        //cv::imwrite("detection_result.png", IB_VIS);
        std::cout <<" - done." << std::endl;
        cv::waitKey(1);
    }

    outputFile.close();
    return 0;
}
