#include "four-point.hpp"
#include "five-point/five-point.hpp"

using namespace cv; 

int main()
{
    double angle_bound = CV_PI / 4; 

    double nearest_dist = 1; 
    double baseline_dev = 0.01; 
    double baseline = 1; 
    double depth = 5; 

    double focal = 300; 
    double bound_2d = 175; 

    RNG rng; 
    vector<vector<double> > t_angle_4pt, t_angle_5pt;     
    

    std::cout << "h = figure, " << std::endl; 
    for (double odo_sigma = 0; odo_sigma < M_PI / 180.0 * 1.5; odo_sigma += M_PI / 180.0 * 3)
    {
        std::cout << "t_err_4pt = []; " << std::endl;  
        std::cout << "t_err_5pt = []; " << std::endl;  
        std::cout << "px_sigma = []; " << std::endl;  

        for (double px_sigma = 0; px_sigma <= 1.0; px_sigma += 0.1)
        {
            std::cout << "px_sigma(end + 1) = " << px_sigma << "; " << std::endl; 
            t_angle_4pt.push_back(vector<double>());         
            t_angle_5pt.push_back(vector<double>());         
            for (int i = 0; i < 1000; i++)
            {    
                Mat rvec(3, 1, CV_64F), tvec(3, 1, CV_64F), cvec(3, 1, CV_64F); 
                rng.fill(rvec, RNG::UNIFORM, -angle_bound, angle_bound); 
                rng.fill(cvec, RNG::UNIFORM, -baseline_dev, baseline_dev); 
                
                cvec.at<double>(2) = baseline;  
    
                rvec *= fmod(norm(rvec), CV_PI) / norm(rvec); 
    
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
    
        
                Mat noise(x1s.size(), CV_64F); 
                rng.fill(noise, RNG::NORMAL, 0, px_sigma); 
                Mat x1s_noise = x1s + noise; 
                rng.fill(noise, RNG::NORMAL, 0, px_sigma); 
                Mat x2s_noise = x2s + noise; 
            
                std::vector<Mat> rvecs_4pt, tvecs_4pt, rvecs_4pt_noise, tvecs_4pt_noise; 
                double angle = norm(rvec) + rng.gaussian(odo_sigma); 
    
                four_point(x1s.rowRange(0, 4), x2s.rowRange(0, 4), norm(rvec), focal, Point2d(0, 0), rvecs_4pt, tvecs_4pt); 
                four_point(x1s_noise.rowRange(0, 4), x2s_noise.rowRange(0, 4), angle, focal, Point2d(0, 0), rvecs_4pt_noise, tvecs_4pt_noise); 
        
                tvec /= norm(tvec); 
                
                if (rvecs_4pt_noise.empty()) 
                {
    //                t_angle_4pt.back().push_back(CV_PI/2); 
                    continue; 
                }
                
                int k; 
                for (k = 0; k < rvecs_4pt.size(); k++)
                {
                    if (norm(rvecs_4pt[k], rvec) + norm(tvecs_4pt[k], tvec) < 1e-5) break; 
                }
                if (k >= rvecs_4pt.size()) continue; 
    
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
    //                std::cout << r1 << r2 << t << std::endl; 
                    if (norm(r1, rvec) < min_dist)
                    {
                        rvec_5pt_noise = r1 * 1.0; 
                        tvec_5pt_noise = norm(t, tvec) < norm(-t, tvec) ? t * 1.0 : -t * 1.0; 
                        min_dist = norm(r1, rvec); 
                    }
                    if (norm(r2, rvec) < min_dist)
                    {
                        rvec_5pt_noise = r2 * 1.0; 
                        tvec_5pt_noise = norm(t, tvec) < norm(-t, tvec) ? t * 1.0 : -t * 1.0; 
                        min_dist = norm(r2, rvec); 
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
    }

    std::cout << "t_err_4pt = t_err_4pt * 180 / pi; " << std::endl; 
    std::cout << "t_err_5pt = t_err_5pt * 180 / pi; " << std::endl; 
    std::cout << "plot(0:0.1:1, t_err_4pt, '-o'), hold on" << std::endl; 
    std::cout << "plot(0:0.1:1, t_err_5pt, '-x'), hold on" << std::endl; 
    std::cout << "legend('4-pt', '5-pt', 'Location', 'Northwest')" << std::endl; 
    std::cout << "set(h, 'Position', [0, 0, 300, 200])" << std::endl; ; 
}
