#include <opencv2/opencv.hpp>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_multiroots.h>
#include "four-point.hpp"

using namespace cv; 


int four_point_f(const gsl_vector * t, void * ab, gsl_vector * f)
{
    double * a = (double*)ab; 
    double * b = a + 56; 

    double t0 = gsl_vector_get(t, 0); 
    double t1 = gsl_vector_get(t, 1); 

    if (t0 < 0.0) t0 = 0.0; 
    if (t0 > 1.0) t0 = 1.0; 
    if (t1 < 0.0) t0 = 0.0; 
    if (t1 > 1.0) t1 = 1.0; 

    double theta = acos(2.0 * t0 - 1.0); 
    double psi = 2.0 * CV_PI * t1; 
    double rx = sin(theta) * cos(psi); 
    double ry = sin(theta) * sin(psi); 
    double rz = cos(theta); 

    double f0 = a[0]+a[1]*rx*rx*rx*rx*rx+a[2]*rx*rx*rx*rx*ry+a[3]*rx*rx*rx*rx*rz+a[4]*rx*rx*rx*ry*ry+a[5]*rx*rx*rx*ry*rz+a[6]*rx*rx*rx*rz*rz+a[7]*rx*rx*ry*ry*ry+a[8]*rx*rx*ry*ry*rz+a[9]*rx*rx*ry*rz*rz+a[10]*rx*rx*rz*rz*rz+a[11]*rx*ry*ry*ry*ry+a[12]*rx*ry*ry*ry*rz+a[13]*rx*ry*ry*rz*rz+a[14]*rx*ry*rz*rz*rz+a[15]*rx*rz*rz*rz*rz+a[16]*ry*ry*ry*ry*ry+a[17]*ry*ry*ry*ry*rz+a[18]*ry*ry*ry*rz*rz+a[19]*ry*ry*rz*rz*rz+a[20]*ry*rz*rz*rz*rz+a[21]*rz*rz*rz*rz*rz+a[22]*rx*rx*rx*rx+a[23]*rx*rx*rx*ry+a[24]*rx*rx*rx*rz+a[25]*rx*rx*ry*ry+a[26]*rx*rx*ry*rz+a[27]*rx*rx*rz*rz+a[28]*rx*ry*ry*ry+a[29]*rx*ry*ry*rz+a[30]*rx*ry*rz*rz+a[31]*rx*rz*rz*rz+a[32]*ry*ry*ry*ry+a[33]*ry*ry*ry*rz+a[34]*ry*ry*rz*rz+a[35]*ry*rz*rz*rz+a[36]*rz*rz*rz*rz+a[37]*rx*rx*rx+a[38]*rx*rx*ry+a[39]*rx*rx*rz+a[40]*rx*ry*ry+a[41]*rx*ry*rz+a[42]*rx*rz*rz+a[43]*ry*ry*ry+a[44]*ry*ry*rz+a[45]*ry*rz*rz+a[46]*rz*rz*rz+a[47]*rx*rx+a[48]*rx*ry+a[49]*rx*rz+a[50]*ry*ry+a[51]*ry*rz+a[52]*rz*rz+a[53]*rx+a[54]*ry+a[55]*rz;
    double f1 = b[0]+b[1]*rx*rx*rx*rx*rx+b[2]*rx*rx*rx*rx*ry+b[3]*rx*rx*rx*rx*rz+b[4]*rx*rx*rx*ry*ry+b[5]*rx*rx*rx*ry*rz+b[6]*rx*rx*rx*rz*rz+b[7]*rx*rx*ry*ry*ry+b[8]*rx*rx*ry*ry*rz+b[9]*rx*rx*ry*rz*rz+b[10]*rx*rx*rz*rz*rz+b[11]*rx*ry*ry*ry*ry+b[12]*rx*ry*ry*ry*rz+b[13]*rx*ry*ry*rz*rz+b[14]*rx*ry*rz*rz*rz+b[15]*rx*rz*rz*rz*rz+b[16]*ry*ry*ry*ry*ry+b[17]*ry*ry*ry*ry*rz+b[18]*ry*ry*ry*rz*rz+b[19]*ry*ry*rz*rz*rz+b[20]*ry*rz*rz*rz*rz+b[21]*rz*rz*rz*rz*rz+b[22]*rx*rx*rx*rx+b[23]*rx*rx*rx*ry+b[24]*rx*rx*rx*rz+b[25]*rx*rx*ry*ry+b[26]*rx*rx*ry*rz+b[27]*rx*rx*rz*rz+b[28]*rx*ry*ry*ry+b[29]*rx*ry*ry*rz+b[30]*rx*ry*rz*rz+b[31]*rx*rz*rz*rz+b[32]*ry*ry*ry*ry+b[33]*ry*ry*ry*rz+b[34]*ry*ry*rz*rz+b[35]*ry*rz*rz*rz+b[36]*rz*rz*rz*rz+b[37]*rx*rx*rx+b[38]*rx*rx*ry+b[39]*rx*rx*rz+b[40]*rx*ry*ry+b[41]*rx*ry*rz+b[42]*rx*rz*rz+b[43]*ry*ry*ry+b[44]*ry*ry*rz+b[45]*ry*rz*rz+b[46]*rz*rz*rz+b[47]*rx*rx+b[48]*rx*ry+b[49]*rx*rz+b[50]*ry*ry+b[51]*ry*rz+b[52]*rz*rz+b[53]*rx+b[54]*ry+b[55]*rz;

    gsl_vector_set(f, 0, f0); 
    gsl_vector_set(f, 1, f1); 
    
    return GSL_SUCCESS; 
    
}

int four_point_df(const gsl_vector * t, void * ab, gsl_matrix * J)
{
    double * a = (double*)ab; 
    double * b = a + 56; 

    double t0 = gsl_vector_get(t, 0); 
    double t1 = gsl_vector_get(t, 1); 

    if (t0 < 0.0) t0 = 0.0; 
    if (t0 > 1.0) t0 = 1.0; 
    if (t1 < 0.0) t0 = 0.0; 
    if (t1 > 1.0) t1 = 1.0; 

    double theta = acos(2.0 * t0 - 1.0); 
    double psi = 2.0 * CV_PI * t1; 
    double rx = sin(theta) * cos(psi); 
    double ry = sin(theta) * sin(psi); 
    double rz = cos(theta); 

    double df0_drx = 5.0*rx*rx*rx*rx*a[1]+4.0*rx*rx*rx*ry*a[2]+4.0*rx*rx*rx*rz*a[3]+3.0*rx*rx*ry*ry*a[4]+3.0*rx*rx*ry*rz*a[5]+3.0*rx*rx*rz*rz*a[6]+2.0*rx*ry*ry*ry*a[7]+2.0*rx*ry*ry*rz*a[8]+2.0*rx*ry*rz*rz*a[9]+2.0*rx*rz*rz*rz*a[10]+ry*ry*ry*ry*a[11]+ry*ry*ry*rz*a[12]+ry*ry*rz*rz*a[13]+ry*rz*rz*rz*a[14]+rz*rz*rz*rz*a[15]+4.0*rx*rx*rx*a[22]+3.0*rx*rx*ry*a[23]+3.0*rx*rx*rz*a[24]+2.0*rx*ry*ry*a[25]+2.0*rx*ry*rz*a[26]+2.0*rx*rz*rz*a[27]+ry*ry*ry*a[28]+ry*ry*rz*a[29]+ry*rz*rz*a[30]+rz*rz*rz*a[31]+3.0*rx*rx*a[37]+2.0*rx*ry*a[38]+2.0*rx*rz*a[39]+ry*ry*a[40]+ry*rz*a[41]+rz*rz*a[42]+2.0*rx*a[47]+ry*a[48]+rz*a[49]+a[53]; 
    double df0_dry = rx*rx*rx*rx*a[2]+2.0*rx*rx*rx*ry*a[4]+rx*rx*rx*rz*a[5]+3.0*rx*rx*ry*ry*a[7]+2.0*rx*rx*ry*rz*a[8]+rx*rx*rz*rz*a[9]+4.0*rx*ry*ry*ry*a[11]+3.0*rx*ry*ry*rz*a[12]+2.0*rx*ry*rz*rz*a[13]+rx*rz*rz*rz*a[14]+5.0*ry*ry*ry*ry*a[16]+4.0*ry*ry*ry*rz*a[17]+3.0*ry*ry*rz*rz*a[18]+2.0*ry*rz*rz*rz*a[19]+rz*rz*rz*rz*a[20]+rx*rx*rx*a[23]+2.0*rx*rx*ry*a[25]+rx*rx*rz*a[26]+3.0*rx*ry*ry*a[28]+2.0*rx*ry*rz*a[29]+rx*rz*rz*a[30]+4.0*ry*ry*ry*a[32]+3.0*ry*ry*rz*a[33]+2.0*ry*rz*rz*a[34]+rz*rz*rz*a[35]+rx*rx*a[38]+2.0*rx*ry*a[40]+rx*rz*a[41]+3.0*ry*ry*a[43]+2.0*ry*rz*a[44]+rz*rz*a[45]+rx*a[48]+2.0*ry*a[50]+rz*a[51]+a[54]; 
    double df0_drz = rx*rx*rx*rx*a[3]+rx*rx*rx*ry*a[5]+2.0*rx*rx*rx*rz*a[6]+rx*rx*ry*ry*a[8]+2.0*rx*rx*ry*rz*a[9]+3.0*rx*rx*rz*rz*a[10]+rx*ry*ry*ry*a[12]+2.0*rx*ry*ry*rz*a[13]+3.0*rx*ry*rz*rz*a[14]+4.0*rx*rz*rz*rz*a[15]+ry*ry*ry*ry*a[17]+2.0*ry*ry*ry*rz*a[18]+3.0*ry*ry*rz*rz*a[19]+4.0*ry*rz*rz*rz*a[20]+5.0*rz*rz*rz*rz*a[21]+rx*rx*rx*a[24]+rx*rx*ry*a[26]+2.0*rx*rx*rz*a[27]+rx*ry*ry*a[29]+2.0*rx*ry*rz*a[30]+3.0*rx*rz*rz*a[31]+ry*ry*ry*a[33]+2.0*ry*ry*rz*a[34]+3.0*ry*rz*rz*a[35]+4.0*rz*rz*rz*a[36]+rx*rx*a[39]+rx*ry*a[41]+2.0*rx*rz*a[42]+ry*ry*a[44]+2.0*ry*rz*a[45]+3.0*rz*rz*a[46]+rx*a[49]+ry*a[51]+2.0*rz*a[52]+a[55]; 
    double df1_drx = 5.0*rx*rx*rx*rx*b[1]+4.0*rx*rx*rx*ry*b[2]+4.0*rx*rx*rx*rz*b[3]+3.0*rx*rx*ry*ry*b[4]+3.0*rx*rx*ry*rz*b[5]+3.0*rx*rx*rz*rz*b[6]+2.0*rx*ry*ry*ry*b[7]+2.0*rx*ry*ry*rz*b[8]+2.0*rx*ry*rz*rz*b[9]+2.0*rx*rz*rz*rz*b[10]+ry*ry*ry*ry*b[11]+ry*ry*ry*rz*b[12]+ry*ry*rz*rz*b[13]+ry*rz*rz*rz*b[14]+rz*rz*rz*rz*b[15]+4.0*rx*rx*rx*b[22]+3.0*rx*rx*ry*b[23]+3.0*rx*rx*rz*b[24]+2.0*rx*ry*ry*b[25]+2.0*rx*ry*rz*b[26]+2.0*rx*rz*rz*b[27]+ry*ry*ry*b[28]+ry*ry*rz*b[29]+ry*rz*rz*b[30]+rz*rz*rz*b[31]+3.0*rx*rx*b[37]+2.0*rx*ry*b[38]+2.0*rx*rz*b[39]+ry*ry*b[40]+ry*rz*b[41]+rz*rz*b[42]+2.0*rx*b[47]+ry*b[48]+rz*b[49]+b[53]; 
    double df1_dry = rx*rx*rx*rx*b[2]+2.0*rx*rx*rx*ry*b[4]+rx*rx*rx*rz*b[5]+3.0*rx*rx*ry*ry*b[7]+2.0*rx*rx*ry*rz*b[8]+rx*rx*rz*rz*b[9]+4.0*rx*ry*ry*ry*b[11]+3.0*rx*ry*ry*rz*b[12]+2.0*rx*ry*rz*rz*b[13]+rx*rz*rz*rz*b[14]+5.0*ry*ry*ry*ry*b[16]+4.0*ry*ry*ry*rz*b[17]+3.0*ry*ry*rz*rz*b[18]+2.0*ry*rz*rz*rz*b[19]+rz*rz*rz*rz*b[20]+rx*rx*rx*b[23]+2.0*rx*rx*ry*b[25]+rx*rx*rz*b[26]+3.0*rx*ry*ry*b[28]+2.0*rx*ry*rz*b[29]+rx*rz*rz*b[30]+4.0*ry*ry*ry*b[32]+3.0*ry*ry*rz*b[33]+2.0*ry*rz*rz*b[34]+rz*rz*rz*b[35]+rx*rx*b[38]+2.0*rx*ry*b[40]+rx*rz*b[41]+3.0*ry*ry*b[43]+2.0*ry*rz*b[44]+rz*rz*b[45]+rx*b[48]+2.0*ry*b[50]+rz*b[51]+b[54]; 
    double df1_drz = rx*rx*rx*rx*b[3]+rx*rx*rx*ry*b[5]+2.0*rx*rx*rx*rz*b[6]+rx*rx*ry*ry*b[8]+2.0*rx*rx*ry*rz*b[9]+3.0*rx*rx*rz*rz*b[10]+rx*ry*ry*ry*b[12]+2.0*rx*ry*ry*rz*b[13]+3.0*rx*ry*rz*rz*b[14]+4.0*rx*rz*rz*rz*b[15]+ry*ry*ry*ry*b[17]+2.0*ry*ry*ry*rz*b[18]+3.0*ry*ry*rz*rz*b[19]+4.0*ry*rz*rz*rz*b[20]+5.0*rz*rz*rz*rz*b[21]+rx*rx*rx*b[24]+rx*rx*ry*b[26]+2.0*rx*rx*rz*b[27]+rx*ry*ry*b[29]+2.0*rx*ry*rz*b[30]+3.0*rx*rz*rz*b[31]+ry*ry*ry*b[33]+2.0*ry*ry*rz*b[34]+3.0*ry*rz*rz*b[35]+4.0*rz*rz*rz*b[36]+rx*rx*b[39]+rx*ry*b[41]+2.0*rx*rz*b[42]+ry*ry*b[44]+2.0*ry*rz*b[45]+3.0*rz*rz*b[46]+rx*b[49]+ry*b[51]+2.0*rz*b[52]+b[55]; 

    double drx_dt0 = cos(theta) * cos(psi) * (-2.0) / sqrt(1.0 - (2.0 * t0 - 1.0) * (2.0 * t0 - 1.0)); 
    double dry_dt0 = cos(theta) * sin(psi) * (-2.0) / sqrt(1.0 - (2.0 * t0 - 1.0) * (2.0 * t0 - 1.0)); 
    double drz_dt0 = -sin(theta) * (-2.0) / sqrt(1.0 - (2.0 * t0 - 1.0) * (2.0 * t0 - 1.0)); 
    double drx_dt1 = sin(theta) * (-sin(psi)) * 2.0 * CV_PI; 
    double dry_dt1 = sin(theta) * cos(psi) * 2.0 * CV_PI; 
    double drz_dt1 = 0.0; 

    double df0_dt0 = df0_drx * drx_dt0 + df0_dry * dry_dt0 + df0_drz * drz_dt0; 
    double df0_dt1 = df0_drx * drx_dt1 + df0_dry * dry_dt1 + df0_drz * drz_dt1; 
    double df1_dt0 = df1_drx * drx_dt0 + df1_dry * dry_dt0 + df1_drz * drz_dt0; 
    double df1_dt1 = df1_drx * drx_dt1 + df1_dry * dry_dt1 + df1_drz * drz_dt1; 

    gsl_matrix_set(J, 0, 0, df0_dt0); 
    gsl_matrix_set(J, 0, 1, df0_dt1); 
    gsl_matrix_set(J, 1, 0, df1_dt0); 
    gsl_matrix_set(J, 1, 1, df1_dt1); 

    return GSL_SUCCESS; 
}

int four_point_fdf(const gsl_vector * t, void * ab, gsl_vector * f, gsl_matrix * J)
{
    four_point_f(t, ab, f); 
    four_point_df(t, ab, J); 

    return GSL_SUCCESS; 
}

void solve_roots(double a[56], double b[56])
{
    int n_samples = 10; 
    int n_iters = 200; 

    // Sample
    vector<double> t0s, t1s; 
    double t0, t1; 
    for (t0 = 0; t0 <= 1.0; t0 += 1.0 / (n_samples - 1.0))
        for (t1 = 0; t1 <= 1.0; t1 += 1.0 / (n_samples - 1.0))
        {
            t0s.push_back(t0); 
            t1s.push_back(t1); 
        }

    // Solve
    double ab[56 * 2]; 
    memcpy(ab, a, sizeof(double) * 56); 
    memcpy(ab + 56, b, sizeof(double) * 56); 
    gsl_multiroot_function_fdf f = {&four_point_f, &four_point_df, &four_point_fdf, 2, (void*)ab};     

    const gsl_multiroot_fdfsolver_type *solver_type; 
    gsl_multiroot_fdfsolver * solver; 

    solver_type = gsl_multiroot_fdfsolver_hybridsj; 
    solver = gsl_multiroot_fdfsolver_alloc(solver_type, 2); 

    for (int i = 0; i < t0s.size(); i++)
    {
        gsl_vector *t = gsl_vector_alloc(2); 
        gsl_vector_set(t, 0, t0s[i]); 
        gsl_vector_set(t, 1, t1s[i]); 
        
        gsl_multiroot_fdfsolver_set(solver, &f, t); 
        
        int iter = 0, status; 
        while (iter++ < n_iters)
        {
            status = gsl_multiroot_fdfsolver_iterate(solver); 
            if (status) break; 
            status = gsl_multiroot_test_residual(solver->f, 1e-15); 
            if (status != GSL_CONTINUE) break;
            
            printf ("iter = %3u x = % .3f % .3f "
               "f(x) = % .3e % .3e\n",
               iter,
               gsl_vector_get (solver->x, 0),
               gsl_vector_get (solver->x, 1),
               gsl_vector_get (solver->f, 0),
               gsl_vector_get (solver->f, 1));
        }                
        printf ("status = %s\n", gsl_strerror (status));
        gsl_vector_free(t); 
    }

    gsl_multiroot_fdfsolver_free(solver); 

}


void four_point(cv::InputArray _points1, cv::InputArray _points2, 
                double angle, double focal, cv::Point2d pp, 
                cv::OutputArrayOfArrays rvecs, cv::OutputArrayOfArrays tvecs)
{
    Mat points1, points2; 
	_points1.getMat().copyTo(points1); 
	_points2.getMat().copyTo(points2); 

	int npoints = points1.checkVector(2);
    CV_Assert( npoints == 4 && points2.checkVector(2) == npoints &&
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

    points1 = points1.t(); 
    points2 = points2.t(); 
    std::cout << points1 << std::endl; 
    std::cout << points2 << std::endl; 

    CV_Assert(points1.isContinuous() && points2.isContinuous()); 
    double *x1 = points1.ptr<double>(0); 
    double *y1 = points1.ptr<double>(1); 
    double *x2 = points2.ptr<double>(0); 
    double *y2 = points2.ptr<double>(1); 

    double k1 = cos(angle); 
    double k2 = 1.0 - k1; 
    double k3 = sin(angle); 
    
    double a[56], b[56]; 
    four_point_helper(k1, k2, k3, x1, y1, x2, y2, a, b); 
    
    std::cout << Mat(1, 56, CV_64F, a) << std::endl; 
    std::cout << Mat(1, 56, CV_64F, b) << std::endl; 
    
    solve_roots(a, b); 


}

int main()
{
    Mat rvec = (Mat_<float>(3, 1) << 0.1, 0.2, 0.3);    
    Mat rmat, tvec = (Mat_<float>(3, 1) << 3, 2, 1); 
    Rodrigues(rvec, rmat); 
    std::cout << "Expected:" << std::endl; 
    std::cout << rmat << std::endl; 
    std::cout << tvec << std::endl; 

    Mat Xs = (Mat_<float>(4, 3) << 10, 23, 34, 
                                    -10, 12, 32, 
                                    -4, 22, 11, 
                                    15, -10, 21); 
    std::cout << Xs << std::endl; 

    Mat x1s = Xs.t(); 
    Mat x2s = rmat * Xs.t(); 
    x2s.col(0) += tvec; 
    x2s.col(1) += tvec; 
    x2s.col(2) += tvec; 
    x2s.col(3) += tvec; 
    
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

    std::vector<Mat> rvecs, tvecs; 
    four_point(x1s, x2s,norm(rvec),  1.0, Point2d(0, 0), rvecs, tvecs); 

}
