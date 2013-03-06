#include <algorithm>
#include "four-point-numerical.hpp"
#include "five-point.hpp"

using namespace cv; 

int main()
{
    double angle_bound = CV_PI / 4; 

    double nearest_dist = 10; 
    double baseline_dev = 0.05; 
    double baseline = -1; 
    double depth = 10; 

    double focal = 300; 
    double bound_2d = 175; 

    vector<vector<double> > t_angle_4pt, t_angle_5pt;     
    
    int count = 0; 
    int N = 50000; 
    for (int i = 0; i < N; i++)
    {    
        RNG rng(i); 
        Mat rvec(3, 1, CV_64F), tvec(3, 1, CV_64F), cvec(3, 1, CV_64F); 
        rng.fill(rvec, RNG::UNIFORM, -angle_bound, angle_bound); 
        rng.fill(cvec, RNG::UNIFORM, -baseline_dev, baseline_dev); 
        
        normalize(cvec, cvec); 

        Mat rmat; 
        Rodrigues(rvec, rmat); 

        tvec = -rmat * cvec; 

        Mat K = (Mat_<double>(3, 3) << focal, 0, 0, 0, focal, 0, 0, 0, 1); 
        
        Mat Xs(5, 3, CV_64F); 
        rng.fill(Xs, RNG::UNIFORM, -bound_2d, bound_2d); 
        Xs.col(2) = focal; 


        Mat ds(5, 1, CV_64F); 
        rng.fill(ds, RNG::UNIFORM, nearest_dist, nearest_dist + depth); 
        for (int j = 0; j < Xs.rows; j++)
            Xs.row(j) *= ds.at<double>(j) / Xs.at<double>(j, 2); 
        

        Mat x1s = K * Xs.t(); 
        Mat x2s = rmat * Xs.t(); 
        for (int j = 0; j < x2s.cols; j++) x2s.col(j) += tvec; 
        x2s = K * x2s; 
    
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


        Mat rvecs_4pt, tvecs_4pt, rvecs_4pt_noise, tvecs_4pt_noise; 
        four_point_numerical(x1s.rowRange(0, 4), x2s.rowRange(0, 4), norm(rvec), focal, Point2d(0, 0), rvecs_4pt, tvecs_4pt); 

        tvec /= norm(tvec); 
        
        int k; 
        int ind; double min_d = 100; 
        for (k = 0; k < rvecs_4pt.cols; k++)
        {
            if (norm(rvecs_4pt.col(k), rvec) < min_d) 
            {
                ind = k; 
                min_d = norm(rvecs_4pt.col(k), rvec); 
            } 
        }
        if (min_d > 0.1) 
        {
            std::cout << "===" << std::endl; 
            std::cout << x1s << std::endl << x2s << std::endl; 
            std::cout << min_d << std::endl; 
            std::cout << rvec << rvecs_4pt.col(ind) << std::endl; 
            std::cout << tvec << tvecs_4pt.col(ind) << std::endl; 
        } 
        else count++; 
            

    }
    std::cout << count << " " << N << std::endl; 

}
