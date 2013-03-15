Relative Pose Estimation Package
==========

This package contains some widely used relative pose estimation algorithm, which include the following algorithm. APIs of the algorithms all follows OpenCV data type. In addition, the RANSAC framework code is from OpenCV library. These algorithms accept feature point correspondences detected from images. This is the same with the well-known OpenCV function `cv::findFundamentalMat()`. Meanwhile, focal length and principle point (pp) have to be also passed to the functions. 

The four-point algorithm is related with this paper: 

A 4-point Algorithm for Relative Pose Estimation of a Calibrated Camera with a Known Relative Rotation Angle, submitted to the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2013.


Four-point algorithm (Numerical solver)
----------

A numerical solver for the 4-point relative pose estimation algorithm. Returns two 3xn matrices. Each column of the matrices is a possible corresponding solution of rotation vector and translation vector. 

* **Folder**: four-point-numerical/

* **API**:  `void findPose4pt_numerical(cv::InputArray points1, cv::InputArray points2, 
              double angle, double focal, cv::Point2d pp, 
              cv::OutputArray rvecs, cv::OutputArray tvecs, 
              int method, double prob, double threshold, cv::OutputArray _mask); `

* **Dependency**: OpenCV 2.4, GSL Library

* **Remarks**: When running the compiled code, some may have problems like this "symbol lookup error: /usr/lib/libgsl.so.0: undefined symbol: cblas\_dnrm2". This problem is caused by binutils-gold linker. To solve it, run `apt-get remove binutils-gold in terminal. 

Four-point algorithm (Groebner basis solver)
----------

A Groebner basis based solver for the 4-point relative pose estimation algorithm. Returns two 3xn matrices. Each column of the matrices is a possible corresponding solution of rotation vector and translation vector. 

* **Folder**: four-point-groebner/

* **API**:  `void findPose4pt_groebner(cv::InputArray points1, cv::InputArray points2, 
              double angle, double focal, cv::Point2d pp, 
              cv::OutputArray rvecs, cv::OutputArray tvecs, 
              int method, double prob, double threshold, cv::OutputArray _mask); `

* **Dependency**: OpenCV 2.4, Eigen (Contained in this package)

* **Remarks**: The code uses SparseQR, a new function from Eigen repo to computer QR decomposition for sparse matrix. You can also try using SPQRSupport from Eigen to call the suitesparse library for QR decomposition. See the code comment for details. 

One-point algorithm 
----------

A simple 1-point algorithm implementation. Returns two 3xn matrices. Each column of the matrices is a possible corresponding solution of rotation vector and translation vector. This is based on the paper: 

D. Scaramuzza, “1-point-ransac structure from motion for vehicle-mounted cameras by exploiting non-holonomic constraints,” International journal of computer vision, vol. 95, no. 1, pp. 74–85, 2011.


* **Folder**: one-point/

* **API**: `void findPose1pt(cv::InputArray points1, cv::InputArray points2, 
              double focal, cv::Point2d pp, 
              cv::OutputArray rvec, cv::OutputArray tvec, 
              int method, double prob, double threshold, cv::OutputArray _mask); `

* **Dependency**: OpenCV 2.4

Five-point algorithm 
----------

A implementation of the famous 5-point algorithm. Returns the estimated essential matrix. It is based on solver in the paper: 

D. Nister, “An efficient solution to the five-point relative pose problem,” Pattern Analysis and Machine Intelligence, IEEE Transactions on, vol. 26, no. 6, pp. 756–770, 2004. 

The code is based on the Matlab implementation from http://www.vis.uky.edu/~stewe/FIVEPOINT/. 

* **Folder**: five-point-nister/

* **API**: `Mat findEssentialMat( InputArray points1, InputArray points2, double focal = 1.0, Point2d pp = Point2d(0, 0), 
					int method = CV_RANSAC, 
					double prob = 0.999, double threshold = 1, OutputArray mask = noArray() ); `

* **Dependency**: OpenCV 2.4

Small demo and compilation
----------

You can compile the whole package like this: 

    {path}$mkdir build
    {path}$cd build
    {path}/build$cmake ..
    {path}/build$make

`demo.cpp` is a small demo which show how to call the APIs. Each sub-module is independent from each other. Check the `CMakeLists.txt` in each folder and see how they can be used. 
