#include <algorithm>
#include <iomanip>
#include "four-point.hpp"
#include "five-point.hpp"

using namespace cv; 

int main()
{
    double angle_bound = CV_PI / 18; 

    double nearest_dist = 10; 
    double baseline_dev = 0.001; 
    double baseline = 1; 
    double depth = 10; 

    double focal = 300; 
    double bound_2d = 175; 

    int N = 50; 

    RNG rng(2); 

    std::cout << "h = figure, hold on" << std::endl; 

//    for (double odo_sigma = 0; odo_sigma < M_PI / 180 * 3; odo_sigma += M_PI / 180 * 0.5)
    for (double odo_sigma = 0; odo_sigma <= 0.06; odo_sigma += 0.02)
    {
    
        vector<vector<double> > t_angle_4pt, t_angle_5pt;     
        
        std::cout << "t_err_4pt = 0; " << std::endl;  
        std::cout << "t_err_5pt = 0; " << std::endl;  
        std::cout << "px_sigma = 0; " << std::endl;  
    
        for (double stdd = 0.1; stdd <= 1.0; stdd += 0.1)
        {
            double px_sigma = stdd; 
    
            std::cout << "px_sigma(end + 1) = " << px_sigma << "; " << std::endl; 
            t_angle_4pt.push_back(vector<double>());         
            t_angle_5pt.push_back(vector<double>());         
            for (int i = 0; i < 200; i++)
            {    
                Mat rvec(3, 1, CV_64F), tvec(3, 1, CV_64F), cvec(3, 1, CV_64F); 
                rng.fill(rvec, RNG::UNIFORM, -angle_bound, angle_bound); 
                rng.fill(cvec, RNG::UNIFORM, -baseline, baseline); 
                
                normalize(cvec, cvec); 
                rvec *= fmod(norm(rvec), CV_PI) / norm(rvec); 
    
                Mat rmat; 
                Rodrigues(rvec, rmat); 
    
                tvec = -rmat * cvec; 
    
                Mat K = (Mat_<double>(3, 3) << focal, 0, 0, 0, focal, 0, 0, 0, 1); 
                
                Mat Xs(N, 3, CV_64F); 
                rng.fill(Xs, RNG::UNIFORM, -bound_2d, bound_2d); 
                Xs.col(2) = focal; 
    
    
                Mat ds(N, 1, CV_64F); 
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
    
        
                Mat noise1(x1s.size(), CV_64F), noise2(x2s.size(), CV_64F); 
                rng.fill(noise1, RNG::NORMAL, 0, px_sigma); 
                Mat x1s_noise = x1s + noise1; 
                rng.fill(noise2, RNG::NORMAL, 0, px_sigma); 
                Mat x2s_noise = x2s + noise2; 

//                double angle = norm(rvec) + rng.gaussian(odo_sigma); 
                double angle = norm(rvec) * (1.0 + rng.gaussian(odo_sigma)); 
    
                std::vector<Mat> rvecs_4pt, tvecs_4pt, rvecs_4pt_noise, tvecs_4pt_noise; 
                findPose_4pt(x1s_noise, x2s_noise, angle, focal, cv::Point2d(0, 0), rvecs_4pt_noise, tvecs_4pt_noise, CV_RANSAC, 0.99, 1, cv::noArray()); 
        
                tvec /= norm(tvec); 
                
                if (rvecs_4pt_noise.empty()) 
                {
                    continue; 
                }
                
                double min_dist = 1e100; 
                int index; 
                for (int k = 0; k < rvecs_4pt_noise.size(); k++)
                    if (norm(rvec, rvecs_4pt_noise[k]) + norm(tvec, tvecs_4pt_noise[k]) < min_dist)
                    {
                        min_dist = norm(rvec, rvecs_4pt_noise[k]) + norm(tvec, tvecs_4pt_noise[k]); 
        
                        index = k; 
                    }
                double temp = tvecs_4pt_noise[index].dot(tvec); 
                t_angle_4pt.back().push_back(acos(temp > 1 ? 1 : temp)); 
    
                /*******************************************************/
                Mat E = findEssentialMat(x1s_noise, x2s_noise, focal); 
                Mat rvec_5pt_noise, tvec_5pt_noise; 
                min_dist = 1e10; 
                int j; 
                for (j = 0; j < E.rows / 3; j++)
                {
                    Mat R1, R2, t; 
                    decomposeEssentialMat(E.rowRange(j * 3, j * 3 + 3), R1, R2, t); 
                    Mat r1, r2; 
                    Rodrigues(R1, r1); 
                    Rodrigues(R2, r2); 
        
                    double dist1 = norm(r1, rvec) + norm(t, tvec); 
                    double dist2 = norm(r1, rvec) + norm(-t, tvec); 
                    double dist3 = norm(r2, rvec) + norm(t, tvec); 
                    double dist4 = norm(r2, rvec) + norm(-t, tvec); 
    
                    if (dist1 < min_dist)
                    {
                        rvec_5pt_noise = r1 * 1.0; 
                        tvec_5pt_noise = t * 1.0; 
                        min_dist = dist1; 
                    }
                    if (dist2 < min_dist)
                    {
                        rvec_5pt_noise = r1 * 1.0; 
                        tvec_5pt_noise = -t * 1.0; 
                        min_dist = dist2; 
                    }
    
                    if (dist3 < min_dist)
                    {
                        rvec_5pt_noise = r2 * 1.0; 
                        tvec_5pt_noise = t * 1.0; 
                        min_dist = dist3;  
                    }
                    if (dist4 < min_dist)
                    {
                        rvec_5pt_noise = r2 * 1.0; 
                        tvec_5pt_noise = -t * 1.0; 
                        min_dist = dist4; 
                    }
                }
    
                if (j == 0)
                {
                    continue; 
                }
                else
                {               
                    temp = tvec_5pt_noise.dot(tvec); 
                    t_angle_5pt.back().push_back(acos(temp > 1 ? 1 : temp)); 
    
    
                }
    
    
            }
            Scalar mean, dev; 
            meanStdDev(t_angle_4pt.back(), mean, dev); 
            std::cout << "t_err_4pt(end + 1) = " << mean[0] << "; " << std::endl;  
            meanStdDev(t_angle_5pt.back(), mean, dev); 
            std::cout << "t_err_5pt(end + 1) = " << mean[0] << "; " << std::endl;  
    
        }
    
        std::cout << "t_err_4pt = t_err_4pt * 180 / pi; " << std::endl; 
        std::cout << "h_4pt = plot(0:0.1:1, t_err_4pt, '-o', 'linewidth', 3, " << "'color', [" << odo_sigma * 10 << ", " << odo_sigma * 1.0 << ", 1])" << std::endl; ; hold on" << std::endl; 
        std::cout << std::setprecision(1) << "text(1.02, t_err_4pt(end), '\\sigma_{o}=" << odo_sigma << "')" << std::endl; 
        std::cout << std::setprecision(6); 
    }
    std::cout << "t_err_5pt = t_err_5pt * 180 / pi; " << std::endl; 
    std::cout << "h_5pt = plot(0:0.1:1, t_err_5pt, '-xr', 'markersize', 12, 'linewidth', 4); hold on" << std::endl; 
    std::cout << "legend([h_4pt, h_5pt], '4-pt w/ varying odo noise', '5-pt', 'Location', 'Northwest')" << std::endl; 
    std::cout << "xlabel('Noise (px)')" << std::endl; 
    std::cout << "ylabel('Translation error (deg)')" << std::endl; 

}
