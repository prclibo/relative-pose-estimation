#include "four-point.hpp"

using namespace cv; 

int main()
{
    double bound = 100.0; 

    RNG rng; 
    for (int i = 0; i < 1000; i++)
    {
        Mat rvec(3, 1, CV_64F), tvec(3, 1, CV_64F); 
        rng.fill(rvec, RNG::UNIFORM, -CV_PI, CV_PI); 
        rng.fill(tvec, RNG::UNIFORM, -bound, bound); 

        rvec *= fmod(norm(rvec), 2.0 * CV_PI) / norm(rvec); 

        Mat rmat; 
        Rodrigues(rvec, rmat); 

        double focal = 300; 
        Mat K = (Mat_<double>(3, 3) << focal, 0, 0, 0, focal, 0, 0, 0, 1); 
        
        Mat Xs(4, 3, CV_64F); 
        rng.fill(Xs, RNG::UNIFORM, -bound, bound); 

        Mat x1s = K * Xs.t(); 
        Mat x2s = rmat * Xs.t(); 
        x2s.col(0) += tvec; 
        x2s.col(1) += tvec; 
        x2s.col(2) += tvec; 
        x2s.col(3) += tvec; 
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

        Mat noise(x1s.size(), CV_64F); 
        randn(noise, 0, 1.0); 
        Mat x1s_noise = x1s + noise; 
        randn(noise, 0, 1.0); 
        Mat x2s_noise = x2s + noise; 
    
        std::vector<Mat> rvecs, tvecs, rvecs_noise, tvecs_noise; 
        four_point(x1s, x2s, norm(rvec), focal, Point2d(0, 0), rvecs, tvecs); 
        four_point(x1s_noise, x2s_noise, norm(rvec), focal, Point2d(0, 0), rvecs_noise, tvecs_noise); 

        tvec /= norm(tvec); 
        int j; 
        std::cout << "===== " << i << " =====" << std::endl; 
        for (j = 0; j < rvecs.size(); j++)
        {
            if (norm(rvec, rvecs[j]) + norm(tvec, tvecs[j]) < 1e-5)
            {
                break; 
            }
        }
        if (j < rvecs.size()) 
        {
            double dist = 1e100; 
            int index; 
            for (int k = 0; k < rvecs.size(); k++)
                if (norm(rvec, rvecs[k]) + norm(rvec, rvecs[k]) < dist)
                {
                    dist = norm(rvec, rvecs[k]) + norm(rvec, rvecs[k]); 
                    index = k; 
                }
            std::cout << rvecs[index] << " " << tvecs[index] << std::endl; 
            std::cout << rvecs_noise[index] << " " << tvecs_noise[index] << std::endl; 

/*            std::cout << "=========" << i << "========================" << std::endl; 
            std::cout << rvec << tvec << std::endl; 
            std::cout << Xs << std::endl; 
            std::cout << "--" << std::endl; 
            for (int k = 0; k < rvecs.size(); k++)
                std::cout << rvecs[k] << tvecs[k] << std::endl; 
*/
        }
    }
}
