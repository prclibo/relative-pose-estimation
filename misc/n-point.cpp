#include "precomp.hpp"
#include "_modelest.h"
#include "five-point.hpp"
#include <iostream>
#include <complex>

using namespace cv; 
using namespace std; 

class CvEMEstimator : public CvModelEstimator2
{
public:
    CvEMEstimator(); 
    virtual int runKernel( const CvMat* m1, const CvMat* m2, CvMat* model ); 
    virtual int run5Point( const CvMat* _q1, const CvMat* _q2, CvMat* _ematrix ); 
protected: 
	bool reliable( const CvMat* m1, const CvMat* m2, const CvMat* model ); 
    virtual void calibrated_fivepoint_helper( double *eet, double* at ); 
    virtual void computeReprojError( const CvMat* m1, const CvMat* m2,
                                     const CvMat* model, CvMat* error );
}; 


Mat findPose( InputArray _points1, InputArray _points2, double focal, Point2d pp, 
              int method, double prob, double threshold, OutputArray _mask, 
              double angle, ) 



