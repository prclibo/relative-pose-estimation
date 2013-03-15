/*  Copyright (c) 2013, Bo Li, prclibo@gmail.com
    All rights reserved.
    
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
          notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
          notice, this list of conditions and the following disclaimer in the
          documentation and/or other materials provided with the distribution.
        * Neither the name of the copyright holder nor the
          names of its contributors may be used to endorse or promote products
          derived from this software without specific prior written permission.
    
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <opencv2/opencv.hpp>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_multiroots.h>
#include "four-point-numerical.hpp"
#include "four-point-numerical-helper.hpp"
#include "_modelest.h"

using namespace cv; 

void truncate(double & t)
{
    if (t > 1.0) t = fmod(t, 1.0); 
    if (t < 0.0) t = fmod(t, 1.0) + 1.0; 
}

int four_point_f(const gsl_vector * t, void * ab, gsl_vector * f)
{
    double * a = (double*)ab; 
    double * b = a + 56; 

    double t0 = gsl_vector_get(t, 0); 
    double t1 = gsl_vector_get(t, 1); 

    truncate(t0); 
    truncate(t1); 

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

    truncate(t0); 
    truncate(t1); 

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

void solve_roots(const vector<double> & a, const vector<double> & b, vector<double> & rx, vector<double> & ry, vector<double> & rz)
{
    int n_samples = 10; 
    int n_iters = 200; 
    double residual_threshold = 1e-4; 
    double delta_threshold = 1e-14; 
    double same_root_threshold = 1e-2; 

    // Sample initial solutions
    vector<double> t0_initial, t1_initial; 
    for (double t0 = 0; t0 <= 1.0; t0 += 1.0 / (n_samples - 1.0))
        for (double t1 = 0; t1 <= 1.0; t1 += 1.0 / (n_samples - 1.0))
        {
            t0_initial.push_back(t0); 
            t1_initial.push_back(t1); 
        }


    // Refine solutions
    double ab[56 * 2]; 
    memcpy(ab, &a[0], sizeof(double) * 56); 
    memcpy(ab + 56, &b[0], sizeof(double) * 56); 
    gsl_multiroot_function_fdf f = {&four_point_f, &four_point_df, &four_point_fdf, 2, (void*)ab};     

    
    const gsl_multiroot_fdfsolver_type *solver_type; 
    gsl_multiroot_fdfsolver * solver; 

    solver_type = gsl_multiroot_fdfsolver_hybridj; 
    solver = gsl_multiroot_fdfsolver_alloc(solver_type, 2); 

    rx.clear(); 
    ry.clear(); 
    rz.clear(); 
    for (int i = 0; i < t0_initial.size(); i++)
    {
        gsl_vector *t = gsl_vector_alloc(2); 
        gsl_vector_set(t, 0, t0_initial[i]); 
        gsl_vector_set(t, 1, t1_initial[i]); 
        
        gsl_multiroot_fdfsolver_set(solver, &f, t); 
        
        int iter = 0, iterate_status, residual_status, delta_status; 
        while (iter++ < n_iters)
        {
            iterate_status = gsl_multiroot_fdfsolver_iterate(solver); 
            residual_status = gsl_multiroot_test_residual(solver->f, residual_threshold); 
            delta_status = gsl_multiroot_test_delta(solver->dx, solver->x, delta_threshold, 0); 
            if (residual_status == GSL_SUCCESS && delta_status == GSL_SUCCESS) break;
            if (iterate_status) break; 
            
        }
//        printf ("status = %s\n", gsl_strerror (status));
//        printf("f = %e %e\n", gsl_vector_get(solver->f, 0), gsl_vector_get(solver->f, 1)); 
//        std::cout << gsl_strerror(iterate_status) << "\n " << gsl_strerror(residual_status) << "\n " << gsl_strerror(delta_status) << std::endl; 
        if (//iterate_status == GSL_SUCCESS && 
            residual_status == GSL_SUCCESS &&  
            delta_status == GSL_SUCCESS )
        {
            double t0, t1; 
            int j, k; 

            t0 = gsl_vector_get(solver->x, 0);  
            t1 = gsl_vector_get(solver->x, 1);  
            truncate(t0); 
            truncate(t1); 

            double theta = acos(2.0 * t0 - 1.0); 
            double psi = 2.0 * CV_PI * t1; 
            double rx_cur = sin(theta) * cos(psi); 
            double ry_cur = sin(theta) * sin(psi); 
            double rz_cur = cos(theta); 

            for (j = 0; j < rx.size(); j++)
            {
                if ((rx[j] - rx_cur) * (rx[j] - rx_cur) + 
                    (ry[j] - ry_cur) * (ry[j] - ry_cur) + 
                    (rz[j] - rz_cur) * (rz[j] - rz_cur) < 
                    same_root_threshold * same_root_threshold)
                    break; 
            }

            if (j >= rx.size()) 
            {
                rx.push_back(rx_cur); 
                ry.push_back(ry_cur); 
                rz.push_back(rz_cur); 
//        std::cout << theta << " " << psi << " " << rx.back() << " " << ry.back() << " " << rz.back() << std::endl ;
            }
        } 
        


        gsl_vector_free(t); 
    }

    gsl_multiroot_fdfsolver_free(solver); 


}


void four_point_numerical(cv::InputArray _points1, cv::InputArray _points2, 
                double angle, double focal, cv::Point2d pp, 
                cv::OutputArray _rvecs, cv::OutputArray _tvecs)
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

    CV_Assert(points1.isContinuous() && points2.isContinuous()); 
    double *x1 = points1.ptr<double>(0); 
    double *y1 = points1.ptr<double>(1); 
    double *x2 = points2.ptr<double>(0); 
    double *y2 = points2.ptr<double>(1); 

    double k1 = cos(angle); 
    double k2 = 1.0 - k1; 
    double k3 = sin(angle); 
    
    vector<double> a(56), b(56); 
    four_point_get_ab(k1, k2, k3, x1, y1, x2, y2, &a[0],&b[0]); 
    
    vector<double> rx, ry, rz; 
    solve_roots(a, b, rx, ry, rz); 
    
       
    int n = rx.size(); 
    _rvecs.create(3, n * 2, CV_64F, -1, true); 
    _tvecs.create(3, n * 2, CV_64F, -1, true); 

    double m[12]; 
    Mat M(4, 3, CV_64F, m); 
    for (int i = 0; i < rx.size(); i++)
    {
        Mat rvec; 
        rvec = (Mat_<double>(3, 1) << rx[i], ry[i], rz[i]); 
        rvec *= angle; 

        Mat tvec; 
        four_point_get_M(k1, k2, k3, x1, y1, x2, y2, rx[i], ry[i], rz[i], m); 
        SVD::solveZ(M, tvec); 

        _rvecs.getMat().col(i * 2) = rvec * 1.0; 
        _tvecs.getMat().col(i * 2) = tvec * 1.0; 
        _rvecs.getMat().col(i * 2 + 1) = rvec * 1.0; 
        _tvecs.getMat().col(i * 2 + 1) = -tvec * 1.0; 

/*        rvec = (Mat_<double>(3, 1) << -rx[i], -ry[i], -rz[i]); 
        rvec *= angle; 

        four_point_get_M(k1, k2, k3, x1, y1, x2, y2, -rx[i], -ry[i], -rz[i], m); 
        SVD::solveZ(M, tvec); 

        _rvecs.getMat().col(i * 4 + 2) = rvec * 1.0; 
        _tvecs.getMat().col(i * 4 + 2) = tvec * 1.0; 
        _rvecs.getMat().col(i * 4 + 3) = rvec * 1.0; 
        _tvecs.getMat().col(i * 4 + 3) = -tvec * 1.0; 
*/


    }

}



class CvFourPointEstimator : public CvModelEstimator2
{
    double angle; 
public:
    CvFourPointEstimator( double _angle ); 
    virtual int runKernel( const CvMat* m1, const CvMat* m2, CvMat* model ); 
protected: 
    virtual void computeReprojError( const CvMat* m1, const CvMat* m2,
                                     const CvMat* model, CvMat* error );
}; 

CvFourPointEstimator::CvFourPointEstimator( double _angle )
: CvModelEstimator2( 4, cvSize(6, 1),  400 ), 
  angle( _angle ) 
{
}


// Notice to keep compatibility with opencv ransac, q1 and q2 have
// to be of 1 row x n col x 2 channel. 
int CvFourPointEstimator::runKernel( const CvMat* q1, const CvMat* q2, CvMat* _rvec_tvec )
{
	Mat Q1 = Mat(q1).reshape(1, q1->cols); 
	Mat Q2 = Mat(q2).reshape(1, q2->cols); 

    Mat rvecs, tvecs; 
    four_point_numerical(Q1, Q2, angle, 1.0, Point2d(0, 0), rvecs, tvecs); 
    rvecs = rvecs.t(); 
    tvecs = tvecs.t(); 
    double * rt = _rvec_tvec->data.db; 

    for (int i = 0; i < rvecs.rows; i++)
    {
        memcpy(rt + i * 6, rvecs.ptr<double>(i), 3 * sizeof(double)); 
        memcpy(rt + i * 6 + 3, tvecs.ptr<double>(i), 3 * sizeof(double)); 
    }
 
    return rvecs.rows; 

}


// Same as the runKernel, m1 and m2 should be
// 1 row x n col x 2 channels. 
// And also, error has to be of CV_32FC1. 
void CvFourPointEstimator::computeReprojError( const CvMat* m1, const CvMat* m2,
                                     const CvMat* model, CvMat* error )
{
    Mat X1(m1), X2(m2); 
    int n = X1.cols; 
    X1 = X1.reshape(1, n); 
    X2 = X2.reshape(1, n); 

    X1.convertTo(X1, CV_64F); 
    X2.convertTo(X2, CV_64F); 

    Mat rvec_tvec(model); 
    Mat rvec = rvec_tvec.colRange(0, 3) * 1.0; 
    Mat tvec = rvec_tvec.colRange(3, 6) * 1.0; 

    Mat rmat; 
    Rodrigues(rvec, rmat); 

    double * t = tvec.ptr<double>(); 
    Mat tskew = (Mat_<double>(3, 3) << 0, -t[2], t[1], t[2], 0, -t[0], -t[1], t[0], 0); 

    Mat E = tskew * rmat; 
    for (int i = 0; i < n; i++)
    {
        Mat x1 = (Mat_<double>(3, 1) << X1.at<double>(i, 0), X1.at<double>(i, 1), 1.0); 
        Mat x2 = (Mat_<double>(3, 1) << X2.at<double>(i, 0), X2.at<double>(i, 1), 1.0); 
        double x2tEx1 = x2.dot(E * x1); 
        Mat Ex1 = E * x1; 
        Mat Etx2 = E * x2; 
        double a = Ex1.at<double>(0) * Ex1.at<double>(0); 
        double b = Ex1.at<double>(1) * Ex1.at<double>(1); 
        double c = Etx2.at<double>(0) * Etx2.at<double>(0); 
        double d = Etx2.at<double>(0) * Etx2.at<double>(0); 

        error->data.fl[i] = x2tEx1 * x2tEx1 / (a + b + c + d); 
    }

}    


void findPose4pt_numerical(cv::InputArray _points1, cv::InputArray _points2, 
              double angle, double focal, cv::Point2d pp, 
              cv::OutputArray _rvecs, cv::OutputArray _tvecs, 
              int method, double prob, double threshold, OutputArray _mask) 
{
	Mat points1, points2; 
	_points1.getMat().copyTo(points1); 
	_points2.getMat().copyTo(points2); 

	int npoints = points1.checkVector(2);
    CV_Assert( npoints >= 4 && points2.checkVector(2) == npoints &&
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
	
	// Reshape data to fit opencv ransac function
	points1 = points1.reshape(2, 1); 
	points2 = points2.reshape(2, 1); 

	Mat rvec_tvec(1, 6, CV_64F); 
    CvFourPointEstimator estimator(angle); 

	CvMat p1 = points1; 
	CvMat p2 = points2; 
	CvMat _rvec_tvec = rvec_tvec; 
	CvMat* tempMask = cvCreateMat(1, npoints, CV_8U); 
	
	assert(npoints >= 4); 
	threshold /= focal; 
    int count = 1; 
    if (npoints == 4)
    {
        four_point_numerical(_points1, _points2, angle, focal, pp, _rvecs, _tvecs); 
        Mat(tempMask).setTo(true); 
    }
    else 
    {
        if (method == CV_RANSAC)
    	{
    		estimator.runRANSAC(&p1, &p2, &_rvec_tvec, tempMask, threshold, prob); 
    	}
    	else
    	{
    		estimator.runLMeDS(&p1, &p2, &_rvec_tvec, tempMask, prob); 
    	}
    
        if (_mask.needed())
        {
        	_mask.create(1, npoints, CV_8U, -1, true); 
        	Mat mask = _mask.getMat(); 
        	Mat(tempMask).copyTo(mask); 
        }
    

        if (false && fabs(angle) < CV_PI / 180.0 * 2.0)
        {
            _rvecs.create(3, 4, CV_64F, -1, true); 
            _tvecs.create(3, 4, CV_64F, -1, true); 
    
            _rvecs.getMat().col(0) = rvec_tvec.colRange(0, 3).t() * 1.0; 
            _tvecs.getMat().col(0) = rvec_tvec.colRange(3, 6).t() * 1.0; 
    
            _rvecs.getMat().col(1) = rvec_tvec.colRange(0, 3).t() * 1.0; 
            _tvecs.getMat().col(1) = -rvec_tvec.colRange(3, 6).t() * 1.0; 
    
            Mat rvec = rvec_tvec.colRange(0, 3).t() * 1.0; 
            Mat tvec = rvec_tvec.colRange(3, 6).t() * 1.0; 
            Mat rmat1, rmat2; 
            Rodrigues(rvec, rmat1); 
            Rodrigues(-rvec, rmat2); 
            tvec = rmat2 * rmat1.t() * tvec; 

            _rvecs.getMat().col(2) = -rvec * 1.0; 
            _tvecs.getMat().col(2) = tvec * 1.0; 
    
            _rvecs.getMat().col(3) = -rvec * 1.0; 
            _tvecs.getMat().col(3) = -tvec * 1.0; 
        }
        else 
        {
            _rvecs.create(3, 2, CV_64F, -1, true); 
            _tvecs.create(3, 2, CV_64F, -1, true); 
    
            _rvecs.getMat().col(0) = rvec_tvec.colRange(0, 3).t() * 1.0; 
            _tvecs.getMat().col(0) = rvec_tvec.colRange(3, 6).t() * 1.0; 
    
            _rvecs.getMat().col(1) = rvec_tvec.colRange(0, 3).t() * 1.0; 
            _tvecs.getMat().col(1) = -rvec_tvec.colRange(3, 6).t() * 1.0; 

        }
    }


}
