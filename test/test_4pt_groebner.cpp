#include <algorithm>
#include "four-point-groebner.hpp"
#include "five-point.hpp"

using namespace cv; 

int main()
{
    Mat rvec = (Mat_<double>(3, 1) << 1, 2, 3); 
    Mat tvec = (Mat_<double>(3, 1) << 1, 1, 1); 

        
    Mat Xs = (Mat_<double>(4, 3) << 10, 23, 34,
                                   -10, 12, 32,
                                    -4, 22, 11,
                                   15, -10, 21); 
    Mat rmat; 
    Rodrigues(rvec, rmat); 
    tvec /= tvec.at<double>(2); 

    std::cout << rmat << std::endl; 
    std::cout << tvec << std::endl; 

            Mat x1s = Xs.t(); 
            Mat x2s = rmat * Xs.t(); 
            for (int j = 0; j < x2s.cols; j++) x2s.col(j) += tvec; 
            x2s = x2s; 
        
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

            std::cout << x1s << std::endl; 
            std::cout << x2s << std::endl; 
    
            Mat rvecs_4pt, tvecs_4pt; 
            four_point_groebner(x1s.rowRange(0, 4), x2s.rowRange(0, 4), norm(rvec), 1.0, Point2d(0, 0), rvecs_4pt, tvecs_4pt); 
            std::cout << rvecs_4pt.t() << std::endl; 
            std::cout << tvecs_4pt.t() << std::endl; 

    
}
