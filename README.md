# SFND 3D Object Tracking

Welcome to the final project of the camera course. By completing all the lessons, you now have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, you know how to detect objects in an image using the YOLO deep-learning framework. And finally, you know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we already have accomplished and what's still missing.

<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, you will implement the missing parts in the schematic. To do this, you will complete four major tasks: 
1. First, you will develop a way to match 3D objects over time by using keypoint correspondences. 
2. Second, you will compute the TTC based on Lidar measurements. 
3. You will then proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, you will conduct various tests with the framework. Your goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. In the last course of this Nanodegree, you will learn about the Kalman filter, which is a great way to combine the two independent TTC measurements into an improved version which is much more reliable than a single sensor alone can be. But before we think about such things, let us focus on your final project in the camera course. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.

## Report Items

### Match 3D Objects


In the `matchBoundingBox()` function, a table named `matchCountMap` is created with a size of (number of bounding boxes in the previous frame x number of bounding boxes in the current frame). The function iterates through the bounding boxes of the previous and current frames, keeping track of keypoints and their corresponding bounding boxes in each frame. The table is updated to reflect the number of keypoints in each bounding box between frames, and this process is implemented in the `camFusion_Student.cpp` file.

Subsequently, the function identifies the best match for the bounding box in the previous frame within the current frame. This is achieved by selecting the bounding box ID with the highest number of matches, which corresponds to the maximum element in each row of the matchCountMap. The implementation for this step can be found in the `camFusion_Student.cpp` file. Finally, the function returns the best-matched bounding boxes.

## Compute Lidar-based TTC

The formula used for Time-to-Collision (TTC) Lidar is consistent with the classroom discussion. The primary challenge involves handling noisy measurements and outliers. To address this issue, the boxplot technique is employed to determine the Inter-Quartile Range (IQR) of the 3D Lidar points. The lower and upper threshold limits are set as Q1 - 1.2 * IQR and Q3 + 1.2 * IQR, respectively.

Upon performing outlier removal using this approach, it was observed that utilizing the mean x value of the Lidar points in TTC calculations yields more accurate results compared to using the x minimum of the points. This adjustment proves beneficial, especially when dealing with faulty measurements near the object edges. The implementation of this process can be found in the `computeTTC_Lidar()` function.

For improved modularity, a `util.cpp` file has been created, housing both the outlier removal function and separate functions for calculating IQR for Lidar and Camera measurements.

## Associate Keypoint Correspondences with Bounding Boxes

The described functionality is implemented in the `clusterKptMatchesWithROI()` function. A vector of pairs, `distMap`, is created to store the indices of keypoints within the bounding box of the current frame that have been matched with keypoints in the same bounding box in the previous frame as the first part of the pair. The second part of the vector of pairs stores the Euclidean distance between the matches.

Following this, outliers present in the `distMap` are removed using the Inter-Quartile Range (IQR) technique, similar to the Lidar case. This outlier removal process is implemented in the util.cpp file. The indices of the inliers are then added to the kptMatches of the respective bounding box in every frame.

> The outlier removal of TTC Lidar used IQR. The function that removes is `removeOutliersLidar()` within `util.cpp`.

## Compute Camera-based TTC

For camera TTC computation, the process involves computing distance ratios between all matched keypoints within the bounding boxes. The median of the distance ratios is chosen for the camera TTC calculation to effectively handle any potential outliers.

## Performance Evaluation 1

From the Lidar top view perspective, the x_min (minimum x-coordinate) consistently decreases; however, the difference in x_min between consecutive frames does not decrease at the same rate. Consequently, the TTC does not exhibit a consistent decreasing trend across all frames, as might be anticipated. It's worth noting that some instances of inaccuracies in the TTC values may be attributed to noisy measurements originating from the Lidar.

An instance where the Lidar TTC estimation becomes implausible is when the Lidar registers points that are evidently not situated on the vehicle but are much closer than expected. This discrepancy may arise from airborne dust particles introducing noise into the Lidar scans, resulting in an erroneously low TTC. To address this issue, a more robust approach involves mitigating the impact of outliers by employing an average distance instead of relying solely on the closest point.

The provided table utilizes the AKAZE and BRIEF detector and descriptor combination for keypoints in the image processing pipeline.

| Frame | TTC Lidar | TTC Camera |
|-------|-----------|------------|
| 0-1   | 13.144    | 14.6617    |
| 1-2   | 12.9549   | 15.4843    |
| 2-3   | 16.3229   | 14.9118    |
| 3-4   | 15.9883   | 15.1336    |
| 4-5   | 11.7422   | 15.3667    |
| 5-6   | 15.1703   | 15.5464    |
| 6-7   | 11.3822   | 15.7052    |
| 7-8   | 12.9842   | 14.8902    |
| 8-9   | 13.0407   | 15.6456    |
| 9-10  | 12.4117   | 12.0318    |
| 10-11 | 11.6781   | 12.9694    |
| 11-12 | 10.5236   | 12.4144    |
| 12-13 | 9.3932    | 10.2317    |
| 13-14 | 9.54557   | 10.1498    |
| 14-15 | 8.62937   | 10.4327    |
| 15-16 | 8.67236   | 10.14      |
| 16-17 | 11.5561   | 9.72712    |
| 17-18 | 8.04705   | 9.66427    |

## Performance Evaluation 2

The following table displays the Time-to-Collision (TTC) values for Camera when utilizing the FAST and BRIEF detector and descriptor combination. It's evident that the TTC estimates are not as accurate as expected. Despite this, it's worth noting that this detector-descriptor combination is the fastest in terms of computation.

In comparison to the TTC camera estimates using the AKAZE and BRIEF descriptor combination, as illustrated in the FP.5 section, AKAZE and BRIEF outperforms FAST and BRIEF by providing more accurate camera TTC estimates.

All result data are available [here](./data/Performance Evaluation.csv)

| Frame | TTC Lidar | TTC Camera |
|-------|-----------|------------|
| 0-1   | 13.144    | 12.5923    |
| 1-2   | 12.9549   | 11.3472    |
| 2-3   | 16.3229   | 13.2228    |
| 3-4   | 15.9883   | 13.2296    |
| 4-5   | 11.7422   | -inf       |
| 5-6   | 15.1703   | 13.5856    |
| 6-7   | 11.3822   | 12.7507    |
| 7-8   | 12.9842   | 12.2852    |
| 8-9   | 13.0407   | 13.6469    |
| 9-10  | 12.4117   | 13.4681    |
| 10-11 | 11.6781   | 14.3636    |
| 11-12 | 10.5236   | 10.8819    |
| 12-13 | 9.3932    | 12.4052    |
| 13-14 | 9.54557   | 11.5337    |
| 14-15 | 8.62937   | 11.8945    |
| 15-16 | 8.67236   | 13.659     |
| 16-17 | 11.5561   | 7.8014     |
| 17-18 | 8.04705   | 11.9485    |

## Running Script

<img src="./images/results ttc camera and ttc lidar.png" width="779" height="414" />
