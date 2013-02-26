#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "five-point.hpp"
#include "four-point-numerical.hpp"
#include "one-point.hpp"
#include "selectPose.hpp"

#include "sparse_graph/SparseGraph.h"
#include "sparse_graph/Odometer.h"

using namespace cv; 
using namespace vcharge; 
using namespace Eigen; 

int main()
{
    Matrix4d H_c2o;  

    // Right 
/*    H_c2o << -0.999296531113, -0.00851557878687,  0.0365229766604, 1.88439902007, 
            -0.0342714966818, 0.602816314663,  -0.79714362275,  -1.0024505215, 
            -0.015228506867, -0.797834554086,  -0.602684093771,  0.914422547537, 
            0, 0, 0, 1; */

    // Rear
/*    H_c2o << -0.003413036949,  0.0961438750952,  -0.995361595834,  -0.683308998207, 
             0.999677909435,  -0.0247037361153,  -0.00581401832829,  -0.00202029098963, 
             -0.0251481324547,  -0.995060842714,  -0.0960285932961,  0.784801620532, 
             0, 0, 0, 1; */
    // Front
    H_c2o << -0.0125024998363,  0.12005696458,  0.992688275721,  3.40940859273, 
            -0.999754671718,  -0.0196534643182,  -0.0102145835687,  0.00208103690043, 
            0.0182834317083, -0.992572449041,  0.120273228645,  0.592218078391, 
            0, 0, 0, 1; 


    SparseGraph sg; 
    sg.readFromFile("../../v-charge-data/long_loop/frames.xml"); 
//    sg.readFromFile("../../v-charge-data/more_loop/frames.xml"); 


    int ci = 0; 

    FrameSegment fs0 = sg.frameSegments(ci).front(); 

    Mat pose_4pt_odo = Mat::eye(4, 4, CV_64F); 
    Mat pose_4pt_gps = Mat::eye(4, 4, CV_64F); 
    Mat pose_5pt = Mat::eye(4, 4, CV_64F); 
    Mat pose_1pt = Mat::eye(4, 4, CV_64F); 

    std::cout << "clear" << std::endl; 
    int n = 0; 
    int start = 0; 
    for (int s = 0; s < sg.frameSegments(ci).size(); s++)
    {
        FrameSegment fs = sg.frameSegments(ci).at(s); 
        std::cout << fs.size() << std::endl; 
        for (int i = 0; i < fs.size(); i++)
        {
            if (s == 0 && i == 0) continue; 

            Matrix4d pose_gps_prev, pose_odo_prev; 
            double angle_odo; 
            if (s > 0 && i == 0)
            {
                pose_gps_prev = sg.frameSegments(ci).at(s - 1).back()->gps_ins()->pose();  
                pose_odo_prev = sg.frameSegments(ci).at(s - 1).back()->odometer()->pose(); 
                angle_odo = fs.at(i)->odometer()->yaw() - sg.frameSegments(ci).at(s - 1).back()->odometer()->yaw(); 
                std::cout << fs.at(i)->odometer()->yaw() << " - " << sg.frameSegments(ci).at(s - 1).back()->odometer()->yaw(); 
//                std::cout << "=" << angle_odo << std::endl; 
            }
            else
            {
                pose_gps_prev = fs.at(i - 1)->gps_ins()->pose();  
                pose_odo_prev = fs.at(i - 1)->odometer()->pose(); 
                angle_odo = fs.at(i)->odometer()->yaw() - fs.at(i - 1)->odometer()->yaw(); 
                std::cout << fs.at(i)->odometer()->yaw() << " - " << fs.at(i - 1)->odometer()->yaw(); 
//                std::cout << "=" << angle_odo << std::endl; 
            } 
//            continue; 

            Matrix4d pose_gps_init = fs0.at(0)->gps_ins()->pose(); 
            Matrix4d pose_odo_init = fs0.at(0)->odometer()->pose();             

            Matrix4d pose_gps_curr = fs.at(i)->gps_ins()->pose();  
            Matrix4d pose_odo_curr = fs.at(i)->odometer()->pose(); 

            pose_gps_init.block<3, 1>(0, 3) -= fs0.at(0)->gps_ins()->translation(); 
            pose_odo_init.block<3, 1>(0, 3) -= fs0.at(0)->odometer()->pose().block<3, 1>(0, 3); 
            pose_gps_prev.block<3, 1>(0, 3) -= fs0.at(0)->gps_ins()->translation(); 
            pose_odo_prev.block<3, 1>(0, 3) -= fs0.at(0)->odometer()->pose().block<3, 1>(0, 3); 
            pose_gps_curr.block<3, 1>(0, 3) -= fs0.at(0)->gps_ins()->translation(); 
            pose_odo_curr.block<3, 1>(0, 3) -= fs0.at(0)->odometer()->pose().block<3, 1>(0, 3); 


            Matrix4d H_o_prev = pose_odo_prev.inverse() * pose_odo_init; 
            Matrix4d H_gps_prev = pose_gps_prev.inverse() * pose_gps_init; 
            Matrix4d H_cfo_prev = H_c2o.inverse() * H_o_prev * H_c2o; 
            Matrix4d H_cfg_prev = H_c2o.inverse() * H_gps_prev * H_c2o; 
    
            Matrix4d H_gps= pose_gps_curr.inverse() * pose_gps_init; 
            Matrix4d H_o = pose_odo_curr.inverse() * pose_odo_init;  
            Matrix4d H_cfo = H_c2o.inverse() * H_o * H_c2o; 
            Matrix4d H_cfg = H_c2o.inverse() * H_gps * H_c2o; 

            Matrix4d H_cfg_rel = H_cfg * H_cfg_prev.inverse(); 
            Matrix4d H_cfo_rel = H_cfo * H_cfo_prev.inverse(); 
            
            
            Mat R_cfg_cv, r_cfg_cv, t_cfg_cv, R_cfo_cv, r_cfo_cv; 
            Matrix3d R_cfg = H_cfg_rel.block<3, 3>(0, 0); 
            Matrix3d R_cfo = H_cfo_rel.block<3, 3>(0, 0); 
            Vector3d t_cfg = H_cfg_rel.block<3, 1>(0, 3); 
            eigen2cv(R_cfg, R_cfg_cv); 
            eigen2cv(R_cfo, R_cfo_cv); 
            eigen2cv(t_cfg, t_cfg_cv); 
    
            normalize(t_cfg_cv, t_cfg_cv); 
            Rodrigues(R_cfg_cv, r_cfg_cv); 
            Rodrigues(R_cfo_cv, r_cfo_cv); 

            double angle_gps = norm(r_cfg_cv) * (r_cfg_cv.at<double>(1) < 0 ? 1 : -1); 
    
            Vector3d C_cfo = H_cfo.inverse().block<3, 1>(0, 3); 
            Vector3d C_cfg = H_cfg.inverse().block<3, 1>(0, 3); 
    //        C_cfo = H_o.inverse().block<3, 1>(0, 3); 
            
    
            double length = t_cfg.norm(); 
    
            const std::vector<Point2DFeaturePtr> & features = fs.at(i)->features2D(); 
    
            std::vector<Point2f> points1, points2; 
            for (int j = 0; j < features.size(); j++)
            {
                const Point2DFeatureConstPtr & pt = features.at(j); 
                if (pt->prevMatches().empty()) continue; 
    
                const Point2DFeatureConstPtr & ptPrev = pt->prevMatches().at(0); 
    
                points1.push_back(ptPrev->keypoint().pt); 
                points2.push_back(pt->keypoint().pt); 
            }
    
            int select; 
            /*******************************************************/
            Mat mask; 
            Mat E_5pt = findEssentialMat(points1, points2, 300, Point2d(640, 400), CV_RANSAC, 0.999, 1, mask); 
    //        std::cout << countNonZero(mask) << " / " << mask.total() << "   "; 
            Mat R1_5pt, R2_5pt, t_5pt; 
            decomposeEssentialMat(E_5pt, R1_5pt, R2_5pt, t_5pt); 
            
            Mat r1_5pt, r2_5pt; 
            Rodrigues(R1_5pt, r1_5pt); 
            Rodrigues(R2_5pt, r2_5pt); 

/*            Mat rvec_5pt(3, 4, CV_64F), tvec_5pt(3, 4, CV_64F); 
            rvec_5pt.col(0) = r1_5pt * 1.0; 
            rvec_5pt.col(1) = r1_5pt * 1.0; 
            rvec_5pt.col(2) = r2_5pt * 1.0; 
            rvec_5pt.col(3) = r2_5pt * 1.0; 

            tvec_5pt.col(0) = t_5pt * 1.0; 
            tvec_5pt.col(1) = -t_5pt * 1.0; 
            tvec_5pt.col(2) = t_5pt * 1.0; 
            tvec_5pt.col(3) = -t_5pt * 1.0; */
    
            Mat R_5pt; 
            if (norm(R1_5pt, R_cfg_cv) < norm(R2_5pt, R_cfg_cv)) R_5pt = R1_5pt * 1.0; 
            else R_5pt = R2_5pt * 1.0; 
    
            if (norm(-t_5pt, t_cfg_cv) < norm(t_5pt, t_cfg_cv)) t_5pt = -t_5pt; 
    
/*            select = selectPose(points1, points2, 300, Point2d(640, 400), rvec_5pt, tvec_5pt); 
            t_5pt = tvec_5pt.col(select) * 1.0; 
            R_5pt = select > 1 ? R2_5pt * 1.0 : R1_5pt * 1.0; */
            
            /*******************************************************/
            Mat rvecs_4pt_odo, tvecs_4pt_odo; 
            findPose4pt_numerical(points1, points2, angle_odo, 300, Point2d(640, 400), rvecs_4pt_odo, tvecs_4pt_odo, CV_RANSAC, 0.99, 1, mask); 
            std::cout << angle_odo << std::endl; 
    //        std::cout << countNonZero(mask) << " / " << mask.total() << "   "; 
            Mat R_4pt_odo, t_4pt_odo; 
            if (angle_odo * rvecs_4pt_odo.at<double>(1, 0) > 0)
            {
//                std::cout << "refer = " << rvecs_4pt_odo.col(0) << std::endl; 
//                std::cout << "real = " << rvecs_4pt_odo.col(0) << std::endl; 
                Rodrigues(rvecs_4pt_odo.col(0), R_4pt_odo); 
            }
            else 
            {
//                std::cout << "refer = " << -rvecs_4pt_odo.col(0) << std::endl; 
//                std::cout << "real = " << rvecs_4pt_odo.col(0) << std::endl; 
                Rodrigues(-rvecs_4pt_odo.col(0), R_4pt_odo); 
            }
            

            if (norm(-tvecs_4pt_odo.col(0), t_cfg_cv) < norm(tvecs_4pt_odo.col(0), t_cfg_cv))
                t_4pt_odo = -tvecs_4pt_odo.col(0) * 1.0; 
            else
                t_4pt_odo = tvecs_4pt_odo.col(0) * 1.0; 

/*            select = selectPose(points1, points2, 300, Point2d(640, 400), rvecs_4pt_odo, tvecs_4pt_odo, mask); 
            t_4pt_odo = tvecs_4pt_odo.col(select) * 1.0;
            Rodrigues(rvecs_4pt_odo.col(select), R_4pt_odo); 
            std::cout << "est = " << rvecs_4pt_odo.col(select) << std::endl; */
    
            /*******************************************************/
            Mat rvecs_4pt_gps, tvecs_4pt_gps; 
            findPose4pt_numerical(points1, points2, angle_gps, 300, Point2d(640, 400), rvecs_4pt_gps, tvecs_4pt_gps, CV_RANSAC, 0.99, 1, mask); 
    //        std::cout << countNonZero(mask) << " / " << mask.total() << "   "; 
            Mat R_4pt_gps, t_4pt_gps; 
            if (1 || angle_gps * rvecs_4pt_odo.at<double>(1, 0) > 0)
                Rodrigues(rvecs_4pt_gps.col(0), R_4pt_gps); 
            else 
                Rodrigues(-rvecs_4pt_gps.col(0), R_4pt_gps); 
            

            if (norm(-tvecs_4pt_gps.col(0), t_cfg_cv) < norm(tvecs_4pt_gps.col(0), t_cfg_cv))
                t_4pt_gps = -tvecs_4pt_gps.col(0) * 1.0; 
            else
                t_4pt_gps = tvecs_4pt_gps.col(0) * 1.0; 
    
/*            select = selectPose(points1, points2, 300, Point2d(640, 400), rvecs_4pt_gps, tvecs_4pt_gps); 
            t_4pt_gps = tvecs_4pt_gps.col(select) * 1.0;
            Rodrigues(rvecs_4pt_gps.col(select), R_4pt_gps); */
            /*******************************************************/
            Mat rvecs_1pt, tvecs_1pt; 
            findPose1pt(points1, points2, 300, Point2d(640, 400), rvecs_1pt, tvecs_1pt, CV_RANSAC, 0.99, 3, mask); 
            Mat R_1pt, t_1pt; 
            Rodrigues(rvecs_1pt.col(0), R_1pt); 
            if (norm(tvecs_1pt.col(0), t_cfg_cv) < norm(tvecs_1pt.col(1), t_cfg_cv))
                t_1pt = tvecs_1pt.col(0) * 1.0; 
            else 
                t_1pt = tvecs_1pt.col(1) * 1.0; 
    
    
            /*******************************************************/
            t_4pt_odo *= length; 
            t_4pt_gps *= length; 
            t_5pt *= length; 
            t_1pt *= length; 
    
            Mat H_4pt_odo = Mat::eye(4, 4, CV_64F); 
            Mat H_4pt_gps = Mat::eye(4, 4, CV_64F); 
            Mat H_5pt = Mat::eye(4, 4, CV_64F); 
            Mat H_1pt = Mat::eye(4, 4, CV_64F); 
    
            H_5pt(Range(0, 3), Range(0, 3)) = R_5pt * 1.0; 
            H_5pt(Range(0, 3), Range(3, 4)) = t_5pt * 1.0; 
    
            H_4pt_odo(Range(0, 3), Range(0, 3)) = R_4pt_odo * 1.0; 
            H_4pt_odo(Range(0, 3), Range(3, 4)) = t_4pt_odo * 1.0; 
    
            H_4pt_gps(Range(0, 3), Range(0, 3)) = R_4pt_gps * 1.0; 
            H_4pt_gps(Range(0, 3), Range(3, 4)) = t_4pt_gps * 1.0; 
    
            H_1pt(Range(0, 3), Range(0, 3)) = R_1pt * 1.0; 
            H_1pt(Range(0, 3), Range(3, 4)) = t_1pt * 1.0; 
    
            pose_5pt = H_5pt * pose_5pt; 
            pose_4pt_odo = H_4pt_odo * pose_4pt_odo; 
            pose_4pt_gps = H_4pt_gps * pose_4pt_gps; 
            pose_1pt = H_1pt * pose_1pt; 
            
            if (n == start)
            {
                eigen2cv(H_cfo, pose_4pt_odo); 
                eigen2cv(H_cfo, pose_4pt_gps); 
                eigen2cv(H_cfo, pose_5pt); 
                eigen2cv(H_cfo, pose_1pt); 
            }        
    
    
            Mat C_5pt = Mat(pose_5pt.inv())(Range(0, 3), Range(3, 4)) * 1.0; 
            Mat C_4pt_odo = Mat(pose_4pt_odo.inv())(Range(0, 3), Range(3, 4)) * 1.0; 
            Mat C_4pt_gps = Mat(pose_4pt_gps.inv())(Range(0, 3), Range(3, 4)) * 1.0; 
            Mat C_1pt = Mat(pose_1pt.inv())(Range(0, 3), Range(3, 4)) * 1.0; 
    
            std::cout << "c_cfo(" << n + 1 - start << ", :) = [" << C_cfo.transpose() << "]; " << std::endl; 
            std::cout << "c_cfg(" << n + 1 - start << ", :) = [" << C_cfg.transpose() << "]; " << std::endl; 
            std::cout << "c_4pt_odo(" << n + 1 - start << ", :) = " << C_4pt_odo.t() << "; " << std::endl; 
            std::cout << "c_4pt_gps(" << n + 1 - start << ", :) = " << C_4pt_gps.t() << "; " << std::endl; 
            std::cout << "c_5pt(" << n + 1 - start << ", :) = " << C_5pt.t() << "; " << std::endl; 
            std::cout << "c_1pt(" << n + 1 - start << ", :) = " << C_1pt.t() << "; " << std::endl; 
            n++; 
        }
    }
    std::cout << "h = figure; " << std::endl; 
    std::cout << "plot(c_cfg(:, 3), -c_cfg(:, 1), '-', 'color', [1, 1, 1] * 0.7, 'linewidth', 10, 'markersize', 2), hold on" << std::endl; 
    std::cout << "plot(c_cfo(:, 3), c_cfo(:, 1), '-', 'color', [1, 1, 1] * 0.4, 'linewidth', 5, 'markersize', 2), hold on" << std::endl; 
    std::cout << "plot(c_1pt(:, 3), c_1pt(:, 1), '-.g', 'linewidth', 3, 'markersize', 2), hold on" << std::endl; 
    std::cout << "plot(c_4pt_odo(:, 3), c_4pt_odo(:, 1), '-b', 'linewidth', 3, 'markersize', 2), hold on" << std::endl; 
    std::cout << "plot(c_4pt_gps(:, 3), c_4pt_gps(:, 1), '-m', 'linewidth', 3, 'markersize', 2), hold on" << std::endl; 
    std::cout << "plot(c_5pt(:, 3), c_5pt(:, 1), '--r', 'linewidth', 3, 'markersize', 2), hold on" << std::endl; 
    std::cout << "axis equal" << std::endl; 
    std::cout << "set(h, 'Position', [0, 0, 550, 400])" << std::endl; 
    std::cout << "legend('GPS', 'Odo', '1-pt', '4-pt Odo', '4-pt GPS', '5-pt', 'Location', 'Northwest')" << std::endl; 
    std::cout << "text(c_cfo(1, 3) - 1.5, c_cfo(1, 1) -0.5, 'Start point')" << std::endl; 
    std::cout << "xlabel('(m)')" << std::endl; 
    std::cout << "ylabel('(m)')" << std::endl; 
}

