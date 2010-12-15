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
	
	
	cvUndistortPoints(rgb_2, rgb_corrected, rgb_intrinsics, rgb_distortion,NULL,rgb_intrinsics);
	cvUndistortPoints(ir_2, ir_corrected, ir_intrinsics, ir_distortion,NULL,ir_intrinsics);
	

	
	// epilines in the two images
	CvMat* lines_in_rgb = cvCreateMat(point_cnt, 3, CV_32FC1);
	CvMat* lines_in_ir = cvCreateMat(point_cnt, 3, CV_32FC1);
	
	// one point in the rgb image produces on line in the ir image and vice versa
	cvComputeCorrespondEpilines(ir_2, 1, F, lines_in_rgb);
	cvComputeCorrespondEpilines(rgb_2, 2, F, lines_in_ir);
	
	
	/*
	char filename_rgb[100];
	char filename_int[100];
	char prefix[] = "/src/openFrameworks/addons/kinect/ofxKinect/bin/grab1";
	
	
	int cnt = 0;
	
	// sprintf(filename_rgb,"%s/rgb_%02d.jpg",prefix, cnt);
	sprintf(filename_rgb,"%s/view%04d/color.png",prefix, i);
	IplImage* rgb = cvLoadImage(filename_rgb, 1);
	
	sprintf(filename_int,"%s/view%04d/intensity.png",prefix, i);
	// sprintf(filename_int,"%s/ir_%02d.jpg",prefix, cnt);
	IplImage* ir = cvLoadImage(filename_int, 1);
	
	IplImage* cl = cvCloneImage(rgb);
	
	 for (int p_ = 1; p_<70; p_ += 11)
	 {
		 int p = p_+cnt*70;
	 
		 float a = cvGetReal2D(lines_in_rgb, p, 0);
		 float b = cvGetReal2D(lines_in_rgb, p, 1);
		 float c = cvGetReal2D(lines_in_rgb, p, 2);
	 
	 
		 // a*x+b*y+c=0  <=>   y = -(a*x+c)/b 
		 float y_1 = -(a*0+c)/b;          
		 float y_2 = -(a*640+c)/b;
		 cvLine(cl, cvPoint(0,y_1), cvPoint(640, y_2),CV_RGB(255,0,0), 2);
		 
		 float x = cvGet2D(rgb_corrected, p, 0).val[0];
		 float y = cvGet2D(rgb_corrected, p, 0).val[1];
		 cvCircle(cl, cvPoint(x,y), 2, CV_RGB(0,255,0), 2);
		 
		 // float x_ = cvGet2D(rgb_2, p, 0).val[0];
		 // float y_ = cvGet2D(rgb_2, p, 0).val[1];
		 //	 cvCircle(cl, cvPoint(x_,y_), 4, CV_RGB(0,255,255), 1);
		 
		 
		 
	 }
	 
	cvNamedWindow("ir", 1);
	cvShowImage("ir", ir);
	
	cvNamedWindow("rgb", 1);
	cvShowImage("rgb", cl);
	
	sprintf(filename_rgb,"%s/rgb_%02d_epi.jpg",prefix, cnt);
	
		cvSaveImage(filename_rgb, cl);
	
	cvWaitKey(0);
	*/
	
	
	double total_err = 0;
	double e1,e2;
	
	int image_cnt = point_cnt / 70;
	
	for (int k=0; k<image_cnt; k++)
	{
		double img_error_rgb = 0;
		double img_error_ir = 0;
		
		for (int j=0; j<70; j++)
		{
			
			int i = k*70+j;
			
			e1 = fabs(cvGet2D(rgb_corrected, i, 0).val[0]*cvGet2D(lines_in_rgb, i, 0).val[0]+
					  cvGet2D(rgb_corrected, i, 0).val[1]*cvGet2D(lines_in_rgb, i, 1).val[0]+
					  cvGet2D(lines_in_rgb, i, 2).val[0]);
			
			
			e2 = fabs(cvGet2D(ir_corrected, i, 0).val[0]*cvGet2D(lines_in_ir, i, 0).val[0]+
					  cvGet2D(ir_corrected, i, 0).val[1]*cvGet2D(lines_in_ir, i, 1).val[0]+
					  cvGet2D(lines_in_ir, i, 2).val[0]);
		
			//	cout << "err : " << e1 << " " << e2 << endl;
			
			img_error_rgb += e1;
			img_error_ir  += e2;
			
			total_err += (e1+e2);
		}
		
		/*
		 // show error for each image
		cout << k << endl;
		cout << img_error_rgb/70 << endl;
		cout << img_error_ir/70 << endl << endl;
		*/
	}
	
	return total_err / (2*point_cnt);
	
}
