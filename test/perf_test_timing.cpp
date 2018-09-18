#include <algorithm>
#include "four-point-numerical.hpp"
#include "four-point-groebner.hpp"
#include "five-point.hpp"
#include <time.h>

using namespace cv; 

double markTime()
{
    static timespec tp_prev; 
    timespec tp_curr; 
    clock_gettime(CLOCK_REALTIME, &tp_curr);

    double diff = static_cast<double>(tp_curr.tv_sec - tp_prev.tv_sec) +
                  static_cast<double>(tp_curr.tv_nsec - tp_prev.tv_nsec) / 1000000000.0;
    tp_prev = tp_curr; 
    return diff; 
}

int main()
{
    double angle_bound = CV_PI / 6; 

    double nearest_dist = 10; 
    double baseline_dev = 0.05; 
    double baseline = -1; 
    double depth = 10; 

    double focal = 300; 
    double bound_2d = 175; 

    RNG rng; 
    vector<vector<double> > t_angle_4pt, t_angle_4pt_gb, t_angle_5pt; 
    vector<double> compTime_4pt_nm, compTime_4pt_gb, compTime_5pt; 
    
    for (double stdd = 0; stdd <= 0; stdd += 0.1)
    {
        double sigma = stdd; 

        t_angle_4pt.push_back(vector<double>());         
        t_angle_4pt_gb.push_back(vector<double>());         
        t_angle_5pt.push_back(vector<double>());         


        for (int i = 0; i < 100; i++)
        {    
            std::cerr << i << std::endl;
            Mat rvec(3, 1, CV_64F), tvec(3, 1, CV_64F), cvec(3, 1, CV_64F); 
            rng.fill(rvec, RNG::UNIFORM, -angle_bound, angle_bound); 
            rng.fill(cvec, RNG::UNIFORM, -baseline, baseline); 
            
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

    
            Mat noise1(x1s.size(), CV_64F), noise2(x2s.size(), CV_64F); 
            rng.fill(noise1, RNG::NORMAL, 0, sigma); 
            Mat x1s_noise = x1s + noise1; 
            rng.fill(noise2, RNG::NORMAL, 0, sigma); 
            Mat x2s_noise = x2s + noise2; 

            // std::vector<Mat> rvecs_4pt, tvecs_4pt, rvecs_4pt_noise, tvecs_4pt_noise; 
            Mat rvecs_4pt_noise, tvecs_4pt_noise;
  
            markTime(); 
            four_point_numerical(
                    x1s_noise.rowRange(0, 4), x2s_noise.rowRange(0, 4),
                    norm(rvec), focal, Point2d(0, 0),
                    rvecs_4pt_noise, tvecs_4pt_noise); 
            compTime_4pt_nm.push_back( markTime() ); 
    
            tvec /= norm(tvec); 
            
            if (rvecs_4pt_noise.empty()) 
            {
//                t_angle_4pt.back().push_back(CV_PI/2); 
                continue; 
            }
            
            double min_dist = 1e100; 
            int index; 
            for (int k = 0; k < rvecs_4pt_noise.cols; k++)
                if (norm(rvec, rvecs_4pt_noise.col(k)) +
                        norm(tvec, tvecs_4pt_noise.col(k)) < min_dist)
                {
                    min_dist = norm(rvec, rvecs_4pt_noise.col(k)) + norm(tvec, tvecs_4pt_noise.col(k)); 
    
                    index = k; 
                }
            double temp = tvecs_4pt_noise.col(index).dot(tvec); 
            t_angle_4pt.back().push_back(acos(temp > 1 ? 1 : temp)); 

            /*******************************************************/
            Mat rvecs_4pt_noise_gb, tvecs_4pt_noise_gb; 
            markTime(); 
            four_point_groebner(x1s_noise.rowRange(0, 4), x2s_noise.rowRange(0, 4), norm(rvec), focal, Point2d(0, 0), rvecs_4pt_noise_gb, tvecs_4pt_noise_gb); 
            compTime_4pt_gb.push_back( markTime() ); 

            if (rvecs_4pt_noise_gb.cols == 0) continue; 

            min_dist = 1e100; 
            index = -1; 
            for (int k = 0; k < rvecs_4pt_noise_gb.cols; k++)            
                if (norm(rvec, rvecs_4pt_noise_gb.col(k)) + norm(tvec, tvecs_4pt_noise_gb.col(k)) < min_dist)
                {
                    min_dist = norm(rvec, rvecs_4pt_noise_gb.col(k) + norm(tvec, tvecs_4pt_noise_gb.col(k))); 
                    index = k; 
                }            
            temp = Mat(tvecs_4pt_noise_gb.col(index)).dot(tvec); 
            t_angle_4pt_gb.back().push_back(acos(temp > 1 ? 1 : temp)); 

//            std::cout << t_angle_4pt.back().back() << " " << t_angle_4pt_gb.back().back() << std::endl; 
            /*******************************************************/
            markTime(); 
            Mat E = findEssentialMat(x1s_noise, x2s_noise, focal); 
            compTime_5pt.push_back( markTime() ); 
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


//                std::cout << tvec << std::endl; 
//                std::cout << tvec_5pt_noise << std::endl; 
//                std::cout << t_angle_5pt.back().back() << std::endl; 

            }


        }
    }
    std::cout << "compTime_4pt_nm = " << Mat(compTime_4pt_nm) << "; " << std::endl; 
    std::cout << "compTime_4pt_gb = " << Mat(compTime_4pt_gb) << "; " << std::endl; 
    std::cout << "compTime_5pt = " << Mat(compTime_5pt) << "; " << std::endl; 
    std::cout << "compTime = [compTime_4pt_gb, compTime_4pt_nm, compTime_5pt]; " << std::endl; 
    std::cout << "f = figure; " << std::endl; 
    std::cout << "h = bar([min(compTime); mean(compTime); max(compTime)]); " << std::endl; 
    std::cout << "set(h(1), 'FaceColor', [0.5, 0.5, 1]); " << std::endl; 
    std::cout << "set(h(1), 'EdgeColor', [0.5, 0.5, 1])" << std::endl; 
    std::cout << "set(h(2), 'FaceColor', [0, 0, 1]); " << std::endl; 
    std::cout << "set(h(2), 'EdgeColor', [0, 0, 1]); " << std::endl; 
    std::cout << "set(h(3), 'FaceColor', [1, 0, 0]); " << std::endl; 
    std::cout << "set(h(3), 'EdgeColor', [1, 0, 0]); " << std::endl; 
    std::cout << "set(gca, 'xticklabel', {'Min', 'Mean', 'Max'}); " << std::endl; 
    std::cout << "ylabel('sec'); " << std::endl; 
    std::cout << "legend('4-pt gb', '4-pt nm', '5-pt', 'location', 'NorthEastOutside'); " << std::endl; 
    std::cout << "set(f, 'position', [0, 0, 560, 260]); " << std::endl; 
}
