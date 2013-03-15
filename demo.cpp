#include <algorithm>
#include <opencv2/opencv.hpp>
#include "four-point-numerical/four-point-numerical.hpp"
#include "four-point-groebner/four-point-groebner.hpp"
#include "five-point-nister/five-point.hpp"

using namespace cv; 

int main()
{
    double N = 50; 
    double bound_2d = 5; 

    double focal = 300; 
    Point2d pp(0, 0); 
    
    Mat rvec = (cv::Mat_<double>(3, 1) << 0.1, 0.2, 0.3); 
    Mat tvec = (cv::Mat_<double>(3, 1) << 0.4, 0.5, 0.6); 
    normalize(tvec, tvec); 
    std::cout << "Expected rvec: " << rvec << std::endl; 
    std::cout << "Expected tvec: " << tvec << std::endl; 


    Mat rmat; 
    Rodrigues(rvec, rmat); 

    Mat K = (Mat_<double>(3, 3) << focal, 0, pp.x, 0, focal, pp.y, 0, 0, 1); 
    
    RNG rng; 
    Mat Xs(N, 3, CV_64F); 
    rng.fill(Xs, RNG::UNIFORM, -bound_2d, bound_2d); 

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

    
    Mat rvecs_4pt_nm, tvecs_4pt_nm; 
    findPose4pt_numerical(x1s, x2s, norm(rvec), focal, pp, rvecs_4pt_nm, tvecs_4pt_nm, CV_RANSAC, 0.99, 1, noArray() ); 
    std::cout << "=====================================================" << std::endl; 
    std::cout << "4-pt-nm rvec: " << std::endl; 
    std::cout << rvecs_4pt_nm.t() << std::endl; 
    std::cout << "4-pt-nm tvec: " << std::endl; 
    std::cout << tvecs_4pt_nm.t() << std::endl; 

    Mat rvecs_4pt_gb, tvecs_4pt_gb; 
    findPose4pt_groebner(x1s, x2s, norm(rvec), focal, pp, rvecs_4pt_gb, tvecs_4pt_gb, CV_RANSAC, 0.99, 1, noArray() ); 
    std::cout << "=====================================================" << std::endl; 
    std::cout << "4-pt-gb rvec: " << std::endl; 
    std::cout << rvecs_4pt_gb.t() << std::endl; 
    std::cout << "4-pt-gb tvec: " << std::endl; 
    std::cout << tvecs_4pt_gb.t() << std::endl; 

    Mat E = findEssentialMat(x1s, x2s, focal, pp, CV_RANSAC, 0.99, 1, noArray() ); 
    std::cout << "=====================================================" << std::endl; 
    Mat R1_5pt, R2_5pt, tvec_5pt, rvec1_5pt, rvec2_5pt; 
    decomposeEssentialMat(E, R1_5pt, R2_5pt, tvec_5pt); 
    Rodrigues(R1_5pt, rvec1_5pt); 
    Rodrigues(R2_5pt, rvec2_5pt); 
    std::cout << "5-pt-nister rvec: " << std::endl; 
    std::cout << rvec1_5pt << std::endl; 
    std::cout << rvec2_5pt << std::endl; 
    std::cout << "5-pt-nister tvec: " << std::endl; 
    std::cout << tvec_5pt << std::endl; 
    std::cout << -tvec_5pt << std::endl; 


}
