#include "calibration_common.h"

using namespace std;

#include <iostream>
typedef CvPoint2D32f CvPoint2f;


double computeError(const CvMat* F,
						const CvMat* rgb_corners,
						const CvMat* ir_corners, 
						const CvMat* rgb_distortion,
						const CvMat* ir_distortion, 
						const CvMat* rgb_intrinsics, 
						const CvMat* ir_intrinsics)
{
	
	int point_cnt = rgb_corners->height;
	
	CvMat * rgb_2 = cvCreateMat(point_cnt, 1, CV_32FC2);
	CvMat * ir_2 = cvCreateMat(point_cnt, 1, CV_32FC2);
	
	
	float x,y;
	
	// copy into CV_32FC2 array
	// again, there has to be a smarter way...
	for (int i = 0; i < point_cnt; ++i)
	{
		x = cvGetReal2D(rgb_corners, i, 0);
		y = cvGetReal2D(rgb_corners, i, 1);
		
		cvSet2D(rgb_2, i, 0, cvScalar(x,y,0));
		
		x = cvGetReal2D(ir_corners, i, 0);
		y = cvGetReal2D(ir_corners, i, 1);
		
		cvSet2D(ir_2, i, 0, cvScalar(x,y,0));		
	}
	
	
	
	// undistorted image points
	CvMat* rgb_corrected = cvCreateMat(point_cnt, 1, CV_32FC2);
	CvMat* ir_corrected  = cvCreateMat(point_cnt, 1, CV_32FC2);
	
	cvUndistortPoints(rgb_2, rgb_corrected, rgb_intrinsics, rgb_distortion,NULL,NULL);
	cvUndistortPoints(ir_2, ir_corrected, ir_intrinsics, ir_distortion,NULL,NULL);
	
	
	// epilines in the two images
	CvMat* lines_in_rgb = cvCreateMat(point_cnt, 3, CV_32FC1);
	CvMat* lines_in_ir = cvCreateMat(point_cnt, 3, CV_32FC1);
	
	// one point in the rgb image produces on line in the ir image and vice versa
    cvComputeCorrespondEpilines(rgb_corrected, 2, F, lines_in_ir);
	cvComputeCorrespondEpilines(ir_corrected, 1, F, lines_in_rgb);
	
	double err = 0;
	for (int i=0; i<point_cnt; i++)
	{
		err += fabs(cvGet2D(rgb_corrected, i, 0).val[0]*cvGet2D(lines_in_rgb, i, 0).val[0]+
					cvGet2D(rgb_corrected, i, 0).val[1]*cvGet2D(lines_in_rgb, i, 1).val[0]+
					cvGet2D(lines_in_rgb, i, 2).val[0]);
		
		err += fabs(cvGet2D(ir_corrected, i, 0).val[0]*cvGet2D(lines_in_rgb, i, 0).val[0]+
					cvGet2D(ir_corrected, i, 0).val[1]*cvGet2D(lines_in_rgb, i, 1).val[0]+
					cvGet2D(lines_in_rgb, i, 2).val[0]);
		
	}
	
	return err / (2*point_cnt);
 
	
	return 0;
}
