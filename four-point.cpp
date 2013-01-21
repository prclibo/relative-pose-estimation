#include <opencv2/opencv.hpp>
#include "four-point.hpp"

using namespace cv; 

void four_point(cv::InputArray _points1, cv::InputArray _points2, 
                double angle, double focal, cv::Point2d pp, 
                cv::OutputArrayOfArrays rvecs, cv::OutputArrayOfArrays tvecs)
{
    Mat points1, points2; 
	_points1.getMat().copyTo(points1); 
	_points2.getMat().copyTo(points2); 

	int npoints = points1.checkVector(2);
    CV_Assert( npoints == 4 && points2.checkVector(2) == npoints &&
				              points1.type() == points2.type());

	if (points1.channels() > 1)
	{
		points1 = points1.reshape(1, npoints); 
		points2 = points2.reshape(1, npoints); 
	}
	points1.convertTo(points1, CV_64F); 
	points2.convertTo(points2, CV_64F); 

	points1.col(0) = (points1.col(0) - pp.x) / focal; 
	points2.col(0) = (points2.col(0) - pp.x) / focal; 
	points1.col(1) = (points1.col(1) - pp.y) / focal; 
	points2.col(1) = (points2.col(1) - pp.y) / focal; 

    points1 = points1.t(); 
    points2 = points2.t(); 
    std::cout << points1 << std::endl; 
    std::cout << points2 << std::endl; 

    CV_Assert(points1.isContinuous() && points2.isContinuous()); 
    double *x1 = points1.ptr<double>(0); 
    double *y1 = points1.ptr<double>(1); 
    double *x2 = points2.ptr<double>(0); 
    double *y2 = points2.ptr<double>(1); 

    double k1 = cos(angle); 
    double k2 = 1.0 - k1; 
    double k3 = sin(angle); 
    
    double a[56], b[56]; 
    four_point_helper(k1, k2, k3, x1, y1, x2, y2, a, b); 
    
    std::cout << Mat(1, 56, CV_64F, a) << std::endl; 
    std::cout << Mat(1, 56, CV_64F, b) << std::endl; 


}


int main()
{
    Mat rvec = (Mat_<float>(3, 1) << 0.1, 0.2, 0.3);    
    Mat rmat, tvec = (Mat_<float>(3, 1) << 3, 2, 1); 
    Rodrigues(rvec, rmat); 
    std::cout << "Expected:" << std::endl; 
    std::cout << rmat << std::endl; 
    std::cout << tvec << std::endl; 

    Mat Xs = (Mat_<float>(4, 3) << 10, 23, 34, 
                                    -10, 12, 32, 
                                    -4, 22, 11, 
                                    15, -10, 21); 
    std::cout << Xs << std::endl; 

    Mat x1s = Xs.t(); 
    Mat x2s = rmat * Xs.t(); 
    x2s.col(0) += tvec; 
    x2s.col(1) += tvec; 
    x2s.col(2) += tvec; 
    x2s.col(3) += tvec; 
    
    x1s.row(0) /= x1s.row(2); 
    x1s.row(1) /= x1s.row(2); 
    x1s.row(2) /= x1s.row(2); 
    
    x2s.row(0) /= x2s.row(2); 
    x2s.row(1) /= x2s.row(2); 
    x2s.row(2) /= x2s.row(2); 
    
    x1s = x1s.t(); 
    x2s = x2s.t(); 

    x1s = x1s.colRange(0, 2) * 1.0; 
    x2s = x2s.colRange(0, 2) * 1.0; 

    std::vector<Mat> rvecs, tvecs; 
    four_point(x1s, x2s,norm(rvec),  1.0, Point2d(0, 0), rvecs, tvecs); 

}
