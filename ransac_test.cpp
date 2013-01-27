#include "four-point.hpp"

using namespace cv; 

int main()
{
    double bound = 100.0; 
    int ninlier = 100; 
    int noutlier = 120; 


    RNG rng; 
    vector<vector<double> > t_angle;     
    for (double sigma = 0.5; sigma <= 0.5; sigma += 0.1)
    {
        std::cout << "sigma = " << sigma << std::endl; 
        t_angle.push_back(vector<double>());         
        for (int i = 0; i < 1000; i++)
        {    
            std::cout << "==" << i << "==" << std::endl; 
            Mat rvec(3, 1, CV_64F), tvec(3, 1, CV_64F); 
            rng.fill(rvec, RNG::UNIFORM, -CV_PI, CV_PI); 
            rng.fill(tvec, RNG::UNIFORM, -bound, bound); 
    
            rvec *= fmod(norm(rvec), CV_PI) / norm(rvec); 
    
            Mat rmat; 
            Rodrigues(rvec, rmat); 
    
            double focal = 300; 
            Mat K = (Mat_<double>(3, 3) << focal, 0, 0, 0, focal, 0, 0, 0, 1); 
            
            Mat Xs(ninlier, 3, CV_64F); 
            rng.fill(Xs, RNG::UNIFORM, -bound, bound); 
    
            Mat x1s = K * Xs.t(); 
            Mat x2s = rmat * Xs.t(); 
            for (int i = 0; i < x2s.cols; i++) x2s.col(i) += tvec; 
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

            x1s.resize(ninlier + noutlier); 
            x2s.resize(ninlier + noutlier); 

            Mat o1s(noutlier, 2, CV_64F), o2s(noutlier, 2, CV_64F); 
            rng.fill(o1s, RNG::UNIFORM, -bound, bound); 
            rng.fill(o2s, RNG::UNIFORM, -bound, bound); 

            x1s.rowRange(ninlier, ninlier + noutlier) = o1s * 1.0; 
            x2s.rowRange(ninlier, ninlier + noutlier) = o2s * 1.0; 
    
            Mat noise(x1s.size(), CV_64F); 
            randn(noise, 0, sigma); 
            Mat x1s_noise = x1s + noise; 
            randn(noise, 0, sigma); 
            Mat x2s_noise = x2s + noise; 
        
            std::vector<Mat> rvecs, tvecs, rvecs_noise, tvecs_noise; 
            Mat mask; 
            findPose(x1s_noise, x2s_noise, norm(rvec), focal, Point2d(0, 0), rvecs, tvecs, CV_RANSAC, 0.99, 1.0, mask); 
    
            tvec /= norm(tvec); 
            
            
            double dist = 1e100; 
            int index; 
            for (int k = 0; k < rvecs.size(); k++)
                if (norm(rvec, rvecs[k]) + norm(tvec, tvecs[k]) < dist)
                {
                    dist = norm(rvec, rvecs[k]) + norm(tvec, tvecs[k]); 
    
                    index = k; 
                }
            std::cout << rvec << tvec << std::endl; 
            std::cout << rvecs[index] << tvecs[index] << std::endl; 
            t_angle.back().push_back(acos(tvecs[index].dot(tvec))); 
        }
        Scalar mean, dev; 
        meanStdDev(t_angle.back(), mean, dev); 
        std::cout << t_angle.back().size() << Mat(mean) << Mat(dev) << std::endl; ; 
    }
}
