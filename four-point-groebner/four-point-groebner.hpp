#ifndef FOUR_POINT_GROEBNER_HPP
#define FOUR_POINT_GROEBNER_HPP

#include <opencv2/opencv.hpp>
void four_point_groebner(cv::InputArray _points1, cv::InputArray _points2, 
                double angle, double focal, cv::Point2d pp, 
                cv::OutputArray _rvecs, cv::OutputArray _tvecs); 
void findPose4pt_groebner(cv::InputArray points1, cv::InputArray points2, 
              double angle, double focal, cv::Point2d pp, 
              cv::OutputArray rvecs, cv::OutputArray tvecs, 
              int method, double prob, double threshold, cv::OutputArray _mask); 

#endif
