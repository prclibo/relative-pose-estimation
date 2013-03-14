/*  
    Four point algorithm using groebner solver
    Copyright (C) 2013, Bo Li

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see http://www.gnu.org/licenses.
*/

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
