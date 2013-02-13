#include <opencv2/opencv.hpp>
#include "one-point.hpp"
using namespace cv; 


int main()
{
    RNG rng; 
    double focal = 300; 

    double theta = 0.1; 
    double rho = 1; 

    Mat rmat = (Mat_<double>(3, 3) << cos(theta), sin(theta), 0, -sin(theta), cos(theta), 0, 0, 0, 1); 
    Mat tvec = (Mat_<double>(3, 1) << -cos(theta / 2.0), sin(theta / 2.0), 0); 

    
    Mat Xs(5, 3, CV_64F); 
    rng.fill(Xs, RNG::UNIFORM, -1, 1); 
            
    Mat K = (Mat_<double>(3, 3) << focal, 0, 0, 0, focal, 0, 0, 0, 1); 
    Mat x1s = K * Xs.t(); 
    Mat x2s = rmat * Xs.t(); 
    for (int j = 0; j < x2s.cols; j++) x2s.col(j) += tvec; 
    x2s = K * x2s; 
    std::cout << tvec << std::endl; 
            
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
            
            Mat noise1(x1s.size(), CV_64F), noise2(x2s.size(), CV_64F); 
            rng.fill(noise1, RNG::NORMAL, 0, 1); 
            Mat x1s_noise = x1s + noise1; 
            rng.fill(noise2, RNG::NORMAL, 0, 1); 
            Mat x2s_noise = x2s + noise2; 
    
    Mat rres, tres; 
    findPose_1pt(x1s_noise, x2s_noise, focal, Point2d(0, 0), rres, tres, CV_RANSAC, 0.99, 1, noArray()); 
    std::cout << tres << std::endl; 
    

}


