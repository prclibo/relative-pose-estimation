#include <opencv2/opencv.hpp>
void four_point_groebner(cv::InputArray _points1, cv::InputArray _points2, 
                double angle, double focal, cv::Point2d pp, 
                cv::OutputArray _rvecs, cv::OutputArray _tvecs); 
