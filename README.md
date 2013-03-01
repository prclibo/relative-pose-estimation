==========
Relative Pose Estimation Package
==========

This package contains some widely used relative pose estimation algorithm, which include the following algorithm. APIs of the algorithms all follows OpenCV data type. In addition, the RANSAC framework code is from OpenCV library. 

Four-point algorithm (Numerical solver)
==========

A numerical solver for the 4-point relative pose estimation algorithm. 

Folder: four-point-numerical/

API:  void findPose4pt\_numerical(cv::InputArray points1, cv::InputArray points2, 
              double angle, double focal, cv::Point2d pp, 
              cv::OutputArray rvecs, cv::OutputArray tvecs, 
              int method, double prob, double threshold, cv::OutputArray _mask); 

Dependency: OpenCV 2.4, GSL Library

Remarks: When running the compiled code, some may have problems like this "symbol lookup error: /usr/lib/libgsl.so.0: undefined symbol: cblas\_dnrm2". This problem is caused by binutils-gold linker. To solve it, run apt-get remove binutils-gold in terminal. 

Four-point algorithm (Groebner basis solver)
==========

A Groebner basis based solver for the 4-point relative pose estimation algorithm. 

Folder: four-point-groebner/

API:  void findPose4pt\_groebner(cv::InputArray points1, cv::InputArray points2, 
              double angle, double focal, cv::Point2d pp, 
              cv::OutputArray rvecs, cv::OutputArray tvecs, 
              int method, double prob, double threshold, cv::OutputArray _mask); 

Dependency: OpenCV 2.4, Eigen (Contained in this package)

Remarks: The code uses SparseQR, a new function from Eigen repo to computer QR decomposition for sparse matrix. You can also try using SPQRSupport from Eigen to call the suitesparse library for QR decomposition. See the code comment for details. 

One-point algorithm 
==========

A simple 1-point algorithm implementation. Based on the paper: 
D. Scaramuzza, “1-point-ransac structure from motion for vehicle-mounted cameras by exploiting non-holonomic constraints,” International journal of computer vision, vol. 95, no. 1, pp. 74–85, 2011.

Folder: one-point/

API: void findPose1pt(cv::InputArray points1, cv::InputArray points2, 
              double focal, cv::Point2d pp, 
              cv::OutputArray rvec, cv::OutputArray tvec, 
              int method, double prob, double threshold, cv::OutputArray _mask); 

Dependency: OpenCV 2.4

Five-point algorithm (Nister's solver)
==========

A implementation of the famous 5-point algorithm. It is based on solver in the paper: 
D. Nister, “An efficient solution to the five-point relative pose problem,” Pattern Analysis and Machine Intelligence, IEEE Transactions on, vol. 26, no. 6, pp. 756–770, 2004. 
The code also refers to the Matlab implementation from http://www.vis.uky.edu/~stewe/FIVEPOINT/. 

Folder: five-point-nister/

API: Mat findEssentialMat( InputArray points1, InputArray points2, double focal = 1.0, Point2d pp = Point2d(0, 0), 
					int method = CV_RANSAC, 
					double prob = 0.999, double threshold = 1, OutputArray mask = noArray() ); 

Dependency: OpenCV 2.4


Five-point algorithm (Groebner basis solver)
==========

Another implementation of the famous 5-point algorithm. It is based on solver in the paper: 
H. Stewenius, C. Engels, and D. Nister, “Recent developments on direct relative orientation,” ISPRS Journal of Photogrammetry and Remote Sensing, vol. 60, no. 4, pp. 284–294, 2006.
The code also refers to the Matlab implementation from http://www.vis.uky.edu/~stewe/FIVEPOINT/. 

Folder: five-point-groebner/

API: Mat findEssentialMat( InputArray points1, InputArray points2, double focal = 1.0, Point2d pp = Point2d(0, 0), 
					int method = CV_RANSAC, 
					double prob = 0.999, double threshold = 1, OutputArray mask = noArray() ); 

Dependency: OpenCV 2.4, Eigen (Contained in the package)

