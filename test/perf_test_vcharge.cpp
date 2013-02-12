#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "five-point/five-point.hpp"
#include "four-point.hpp"

#include "sparse_graph/SparseGraph.h"
#include "sparse_graph/Odometer.h"

using namespace cv; 
using namespace vcharge; 
using namespace Eigen; 

int main()
{
    Matrix4d H_c2o;  
    H_c2o << 0.0154394200723,  0.0875650715775,  0.996039147096,  3.42017159758, 
             -0.999856042858,  0.00836277405312,  0.0147633861499, 0.000534201531086, 
             -0.00703689337029,  -0.996123698267,  0.0876815823416,  0.595195403879, 
             0, 0, 0, 1;     

    SparseGraph sg; 
    sg.readFromFile("../../v-charge-data/sparse_graph/frames.xml"); 
//    sg.readFromFile("../../v-charge-data/data_new_new/frames.xml"); 

    FrameSegment fs = sg.frameSegments(0).front(); 

    Mat pose_4pt = Mat::eye(4, 4, CV_64F); 
    Mat pose_5pt = Mat::eye(4, 4, CV_64F); 

    for (int i = 1; i < fs.size(); i++)
    {
        Matrix4d H_o_prev = fs.at(i - 1)->odometer()->pose().inverse() * fs.at(0)->odometer()->pose(); 
        Matrix4d H_gt_prev = H_c2o.inverse() * H_o_prev * H_c2o; 

        Matrix4d H_o = fs.at(i)->odometer()->pose().inverse() * fs.at(0)->odometer()->pose(); 
        Matrix4d H_gt = H_c2o.inverse() * H_o * H_c2o; 

        Matrix4d H_gt_rel = H_gt * H_gt_prev.inverse(); 
        
        Mat R_gt_cv, r_gt_cv, t_gt_cv; 
        Matrix3d R_gt = H_gt_rel.block<3, 3>(0, 0); 
        Vector3d t_gt = H_gt_rel.block<3, 1>(0, 3); 
        eigen2cv(R_gt, R_gt_cv); 
        eigen2cv(t_gt, t_gt_cv); 

        normalize(t_gt_cv, t_gt_cv); 
        Rodrigues(R_gt_cv, r_gt_cv); 


        Vector3d C_gt = H_gt.inverse().block<3, 1>(0, 3); 
//        C_gt = H_o.inverse().block<3, 1>(0, 3); 

        double angle = fs.at(i)->odometer()->yaw() - fs.at(i - 1)->odometer()->yaw(); 

        double length = t_gt.norm(); 

        const std::vector<Point2DFeaturePtr> & features = fs.at(i)->features2D(); 

        std::vector<Point2f> points1, points2; 
        for (int j = 0; j < features.size(); j++)
        {
            const Point2DFeatureConstPtr & pt = features.at(j); 
            if (pt->prevMatches().empty() || pt->bestPrevMatchIdx() < 0) continue; 

            const Point2DFeatureConstPtr & ptPrev = pt->prevMatch(); 

            points1.push_back(ptPrev->keypoint().pt); 
            points2.push_back(pt->keypoint().pt); 
        }

        /*******************************************************/
        Mat mask; 
        Mat E_5pt = findEssentialMat(points1, points2, 300, Point2d(640, 400), CV_RANSAC, 0.99, 1, mask); 
//        std::cout << countNonZero(mask) << " / " << mask.total() << "   "; 
        Mat R1_5pt, R2_5pt, t_5pt; 
        decomposeEssentialMat(E_5pt, R1_5pt, R2_5pt, t_5pt); 
        
        Mat r1_5pt, r2_5pt; 
        Rodrigues(R1_5pt, r1_5pt); 
        Rodrigues(R2_5pt, r2_5pt); 

        Mat R_5pt; 
        if (norm(R1_5pt, R_gt_cv) < norm(R2_5pt, R_gt_cv)) R_5pt = R1_5pt * 1.0; 
        else R_5pt = R2_5pt * 1.0; 

        if (norm(-t_5pt, t_gt_cv) < norm(t_5pt, t_gt_cv)) t_5pt = -t_5pt; 

        
        /*******************************************************/
        std::vector<Mat> rvecs_4pt, tvecs_4pt; 
        findPose(points1, points2, angle, 300, Point2d(640, 400), rvecs_4pt, tvecs_4pt, CV_RANSAC, 0.99, 1, mask); 
//        std::cout << countNonZero(mask) << " / " << mask.total() << "   "; 
        Mat R_4pt, t_4pt; 
        Rodrigues(rvecs_4pt.front(), R_4pt); 
        if (norm(-tvecs_4pt.front(), t_gt_cv) < norm(tvecs_4pt.front(), t_gt_cv))
            t_4pt = -tvecs_4pt.front(); 
        else
            t_4pt = tvecs_4pt.front(); 

        /*******************************************************/

        t_4pt *= length; 
        t_5pt *= length; 

        Mat H_4pt = Mat::eye(4, 4, CV_64F); 
        Mat H_5pt = Mat::eye(4, 4, CV_64F); 

        H_5pt(Range(0, 3), Range(0, 3)) = R_5pt * 1.0; 
        H_5pt(Range(0, 3), Range(3, 4)) = t_5pt * 1.0; 

        H_4pt(Range(0, 3), Range(0, 3)) = R_4pt * 1.0; 
        H_4pt(Range(0, 3), Range(3, 4)) = t_4pt * 1.0; 

        pose_5pt = H_5pt * pose_5pt; 
        pose_4pt = H_4pt * pose_4pt; 

        Mat C_5pt = Mat(pose_5pt.inv())(Range(0, 3), Range(3, 4)) * 1.0; 
        Mat C_4pt = Mat(pose_4pt.inv())(Range(0, 3), Range(3, 4)) * 1.0; 

        std::cout << "c_gt(" << i + 1 << ", :) = [" << C_gt.transpose() << "]; " << std::endl; 
        std::cout << "c4_gt(" << i + 1 << ", :) = " << C_4pt.t() << "; " << std::endl; 
        std::cout << "c5_gt(" << i + 1 << ", :) = " << C_5pt.t() << "; " << std::endl; 
    }
}

