#include <opencv2/opencv.hpp>

using namespace cv; 

int selectPose(cv::InputArray _points1, cv::InputArray _points2, 
                double focal, cv::Point2d pp, 
                cv::InputArray _rvec, cv::InputArray _tvec, cv::InputArray _mask)
{
    Mat points1, points2; 
	_points1.getMat().copyTo(points1); 
	_points2.getMat().copyTo(points2); 

	int npoints = points1.checkVector(2);
    CV_Assert( npoints >= 1 && points2.checkVector(2) == npoints &&
				              points1.type() == points2.type());

	if (points1.channels() > 1)
	{
		points1 = points1.reshape(1, npoints); 
		points2 = points2.reshape(1, npoints); 
	}
	points1.convertTo(points1, CV_64F); 
	points2.convertTo(points2, CV_64F); 

	points1.col(0) = (points1.col(0) - pp.x) / focal; 
	points2.col(0) = (points2.col(0) - pp.x) / focal; 
	points1.col(1) = (points1.col(1) - pp.y) / focal; 
	points2.col(1) = (points2.col(1) - pp.y) / focal; 
	
    Mat rvec, tvec; 
    _rvec.getMat().copyTo(rvec); 
    _tvec.getMat().copyTo(tvec); 
    
    if (rvec.channels() > 1) rvec.reshape(1, 3); 
    if (tvec.channels() > 1) tvec.reshape(1, 3); 
    rvec.convertTo(rvec, CV_64F); 
    tvec.convertTo(tvec, CV_64F); 


    int good_count = 0; 
    int good_index = 0; 
    double min_dist = 1.0; 
    double max_dist = 10.0; 
    Mat mask0; 
    _mask.getMat().copyTo(mask0); 
    for (size_t i = 0; i < rvec.cols; i++)
    {
        Mat rmat; 
        Rodrigues(rvec.col(i), rmat); 

        Mat P0 = Mat::eye(3, 4, rmat.type()); 
        Mat P1 = Mat(3, 4, rmat.type()); 
        P1(Range::all(), Range(0, 3)) = rmat * 1.0; 
        P1.col(3) = tvec.col(i) * 1.0; 
        
        Mat Q; 
        triangulatePoints(P0, P1, points1.t(), points2.t(), Q); 
        for (size_t j = 0; j < 4; j++) Q.row(j) /= Q.row(3); 
	    
        Mat mask; 
        mask = (Q.row(2) > min_dist) & (Q.row(2) < max_dist) & mask0; 
//        std::cout << Q.row(2) << std::endl; 

        Q = P1 * Q; 
//        std::cout << Q.row(2) << std::endl; 
        mask = (Q.row(2) > min_dist) & (Q.row(2) < max_dist) & mask; 
        
        if (countNonZero(mask) > good_count)
        {
            good_count = countNonZero(mask); 
            good_index = i; 
        }                      
    
        Mat _tvec = tvec.col(i) * 1.0; 
        double * t = _tvec.ptr<double>(); 
        Mat tskew = (Mat_<double>(3, 3) << 0, -t[2], t[1], t[2], 0, -t[0], -t[1], t[0], 0); 

        Mat E = tskew * rmat; 
        double error = 0; 
        int count = 0; 
        for (int j = 0; j < npoints; j++)
        {
            if (mask.at<uchar>(j) == 0) continue; 

            Mat x1 = (Mat_<double>(3, 1) << points1.at<double>(j, 0), points1.at<double>(j, 1), 1.0); 
            Mat x2 = (Mat_<double>(3, 1) << points2.at<double>(j, 0), points2.at<double>(j, 1), 1.0); 
            double x2tEx1 = x2.dot(E * x1); 
            Mat Ex1 = E * x1; 
            Mat Etx2 = E * x2; 
            double a = Ex1.at<double>(0) * Ex1.at<double>(0); 
            double b = Ex1.at<double>(1) * Ex1.at<double>(1); 
            double c = Etx2.at<double>(0) * Etx2.at<double>(0); 
            double d = Etx2.at<double>(0) * Etx2.at<double>(0); 
    
            error += sqrt(x2tEx1 * x2tEx1 / (a + b + c + d)); 
            count += sqrt(x2tEx1 * x2tEx1 / (a + b + c + d)) < 1.0 / 300; 
        }
        std::cout << "...." << rvec.col(i) << countNonZero(mask) << " " << error << " " << count << std::endl; 
    }
        
    return good_index;         

}
