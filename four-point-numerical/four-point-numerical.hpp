#ifndef FOUR_POINT_HPP
#define FOUR_POINT_HPP

#include <opencv2/opencv.hpp>

void findPose4pt_numerical(cv::InputArray points1, cv::InputArray points2, 
              double angle, double focal, cv::Point2d pp, 
              cv::OutputArray rvecs, cv::OutputArray tvecs, 
              int method, double prob, double threshold, cv::OutputArray _mask); 

void four_point_numerical(cv::InputArray points1, cv::InputArray points2, 
                double angle, double focal, cv::Point2d pp, 
                cv::OutputArray rvecs, cv::OutputArray tvecs); 

#endif
