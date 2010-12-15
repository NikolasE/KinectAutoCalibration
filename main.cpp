#include <OpenCV/OpenCV.h>

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <fstream>
#include <vector>

#include "calibration_common.h"

using namespace std;

int main (int argc, char * const argv[]) {
	
// activate to see the refined chessboard corners
// #define SHOW_CORNERS	
	
	
// activate to compute stereo calibration
#define STEREO_CALIBRATION	
	
// activate to load calibration for one camera and show undistorted images
// #define VIEW_UNDISTORT
	
	
	// theses values depend on the used chessboard
	const int corner_cnt_x = 10;
	const int corner_cnt_y = 7;
	const float square_size = 2.5;
	const char prefix[] = "YOUR_PATH/bin/grab1";
	const int image_cnt = 90;
	

	
	int good_picture_cnt = 0;
	
	const int corner_cnt = corner_cnt_x*corner_cnt_y;
	
	
	cout << "image_cnt: " << image_cnt << endl;
	
	char filename_rgb[200];
	char filename_int[200];
	
	CvMat *camera_matrix_ir;
	CvMat* distortion_coeffs_ir;
	
	CvMat *camera_matrix_rgb;
	CvMat* distortion_coeffs_rgb;
	
	
	int N = image_cnt*7*10;
	
	CvMat* object_points = cvCreateMat(N, 3, CV_32FC1);
	CvMat* point_counts = cvCreateMat(image_cnt, 1, CV_32SC1);
	
	CvMat* image_points_ir = cvCreateMat(N, 2, CV_32FC1);
	CvMat* image_points_rgb = cvCreateMat(N, 2, CV_32FC1);
	
	
	camera_matrix_rgb = cvCreateMat(3,3, CV_32FC1);
	distortion_coeffs_rgb = cvCreateMat(5,1, CV_32FC1);
	
	
	camera_matrix_ir = cvCreateMat(3,3, CV_32FC1);
	distortion_coeffs_ir = cvCreateMat(5,1, CV_32FC1);
	
	for (int i=0; i< image_cnt; i++)
	{
		cout << "cnt: " << i << endl;
		
		sprintf(filename_rgb,"%s/view%04d/color.png",prefix, i);
		IplImage* rgb_img = cvLoadImage(filename_rgb, 1);
		
		sprintf(filename_int,"%s/view%04d/intensity.png",prefix, i);
		IplImage* intensity_img = cvLoadImage(filename_int, 1);
		
		
		if (rgb_img == NULL)
		{
			cout << "no image named "<< filename_rgb << " found " << endl;
			continue;
		}
		
		if (intensity_img == NULL)
		{
			cout << "no image named "<< filename_int << " found " << endl;
			continue;
		}
		
		
		
		int corner_count;
		
		CvPoint2D32f corners_rgb[corner_cnt];
		CvPoint2D32f corners_int[corner_cnt];
		
		IplImage* bw = cvCreateImage(cvGetSize(rgb_img),rgb_img->depth,1);
		
		// find and correct corners on RGB image
		cvCvtColor(rgb_img, bw, CV_BGR2GRAY);
		if (cvFindChessboardCorners(bw, cvSize(corner_cnt_x,corner_cnt_y), corners_rgb, &corner_count) == 0)
		{
			cout << "no corners on " << filename_rgb << endl;
			continue;
		}
		cvFindCornerSubPix(bw, corners_rgb, corner_count, cvSize(7,7), cvSize(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));
		
		
		// find and correct corners on raw IR image
		cvCvtColor( intensity_img, bw, CV_BGR2GRAY);
		if (cvFindChessboardCorners(bw, cvSize(corner_cnt_x,corner_cnt_y), corners_int, &corner_count) == 0)
		{
			cout << "no corners on " << filename_int << endl;
			continue;
		}
		cvFindCornerSubPix(bw, corners_int, corner_count, cvSize(7,7), cvSize(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));
		
		
		
		// copy Points into calibration arrays:
		for (int j=0; j<corner_cnt; j++)
		{	
			cvSetReal2D(object_points, good_picture_cnt*corner_cnt+j,0, j%corner_cnt_x*square_size);
			cvSetReal2D(object_points, good_picture_cnt*corner_cnt+j,1, j/corner_cnt_x*square_size);
			cvSetReal2D(object_points, good_picture_cnt*corner_cnt+j,2, 0);
			
			cvSetReal2D(image_points_ir, good_picture_cnt*corner_cnt+j,0, corners_int[j].x); 
			cvSetReal2D(image_points_ir, good_picture_cnt*corner_cnt+j,1, corners_int[j].y);
			
			cvSetReal2D(image_points_rgb, good_picture_cnt*corner_cnt+j,0, corners_rgb[j].x); 
			cvSetReal2D(image_points_rgb, good_picture_cnt*corner_cnt+j,1, corners_rgb[j].y);
		}
		
		cvSetReal1D(point_counts, i, corner_count);
		
		good_picture_cnt++;
		
		
		
#ifdef SHOW_CORNERS		
		cvNamedWindow("color.png");
		cvNamedWindow("ir.png");

		cvDrawChessboardCorners(rgb_img, cvSize(10,7), corners_rgb, corner_count,1);
		cvShowImage("color.png", rgb_img);
		
		cvDrawChessboardCorners(intensity_img, cvSize(10,7), corners_int, corner_count,1);
		cvShowImage("ir.png", intensity_img );
		
		
		cvWaitKey(100);
#endif
		
		
	}
	
	
	int found_corner_cnt = good_picture_cnt*corner_cnt;
	
	// some images probably have not all corners, so remove the unused space
	CvMat* object_points_small = cvCreateMat(found_corner_cnt, 3, CV_32FC1);
	CvMat* image_points_ir_small = cvCreateMat(found_corner_cnt, 2, CV_32FC1);
	CvMat* image_points_rgb_small = cvCreateMat(found_corner_cnt, 2, CV_32FC1);
	
	CvMat* point_counts_small = cvCreateMat(good_picture_cnt, 1, CV_32SC1);
	
	// TODO: there has to be a smarter way...
	
	for (int h=0; h < found_corner_cnt; h++)
	{
		float v;
		for (int w=0; w<2; w++)
		{
			v = cvGetReal2D(image_points_ir, h, w);
			cvSet2D(image_points_ir_small, h, w,cvScalarAll(v));
			
			v = cvGetReal2D(image_points_rgb, h, w);
			cvSet2D(image_points_rgb_small, h, w,cvScalarAll(v));
			
			v = cvGetReal2D(object_points, h, w);
			cvSet2D(object_points_small, h, w, cvScalarAll(v));
		}
		// z entry of points on the calibration board:
		cvSet2D(object_points_small, h,2, cvScalarAll(0));
	}
	
	// number of found corners on each image
	// images without the complete number of points are not processed
	for (int h=0; h < good_picture_cnt; h++)
		cvSetReal1D(point_counts_small, h, corner_cnt);
	
	// calibrate both cameras:
	
	// cameras are pretty good, fourth parameter is for fisheye-cams only 
	cvSet1D(distortion_coeffs_ir, 4, cvScalarAll(0));
	
	cvCalibrateCamera2(object_points_small, image_points_ir_small, point_counts_small, cvSize(640,480), 
					   camera_matrix_ir, distortion_coeffs_ir,NULL,NULL,CV_CALIB_FIX_K3 );
	
	cvSave("Intrinsics_IR.xml",camera_matrix_ir); 
	cvSave("Distortion_IR.xml",distortion_coeffs_ir);
	
	cvSet1D(distortion_coeffs_rgb, 4, cvScalarAll(0));
	
	cvCalibrateCamera2(object_points_small, image_points_rgb_small, point_counts_small, cvSize(640,480), 
					   camera_matrix_rgb, distortion_coeffs_rgb,NULL,NULL,CV_CALIB_FIX_K3 );
	
	cvSave("Intrinsics_RGB.xml",camera_matrix_rgb); 
	cvSave("Distortion_RGB.xml",distortion_coeffs_rgb);
	
	
#ifdef STEREO_CALIBRATION	
	
	// cout << "Starting Stereo Calibration" << endl;
	CvMat* R = cvCreateMat(3, 3, CV_32FC1);
	CvMat* T = cvCreateMat(3, 1, CV_32FC1);
	CvMat* E = cvCreateMat(3,3,CV_64F);
	CvMat* F = cvCreateMat(3,3,CV_64F);
	
	
	// the core function:
	cvStereoCalibrate(object_points_small, 
					  image_points_ir_small, 
					  image_points_rgb_small, 
					  point_counts_small, 
					  camera_matrix_ir, distortion_coeffs_ir, 
					  camera_matrix_rgb, distortion_coeffs_rgb, 
					  cvSize(640,480), 
					  R,T,E,F, 
					  cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5), CV_CALIB_FIX_INTRINSIC);
	
	
	cout << "R:" << endl;
	for (int i=0; i<3; i++)
	{
		for (int j=0; j<3; j++)
			cout << cvGet2D(R, i, j).val[0] << " ";
		cout << endl;
	}
	
	cout << "T:   ";
	for (int i=0; i<3; i++)
	{
		cout << cvGet1D(T, i).val[0] << " ";
	}
	cout << endl;
	
	
	
	// compute calibration errors: (currently only rgb)
	double err = computeError(F,
							  image_points_rgb_small,
							  image_points_ir_small, 
							  distortion_coeffs_rgb,
							  distortion_coeffs_ir, 
							  camera_matrix_rgb, 
							  camera_matrix_ir);
	
	cout << "Error: " << err << endl;
	
	
	
	cvSave("kinect_R.xml", R);
	cvSave("kinect_T.xml", T);
	
	
#endif
 
	
	
#ifdef VIEW_UNDISTORT
	
	
	IplImage* map_x = cvCreateImage( cvSize(640,480), IPL_DEPTH_32F, 1 ); 
	IplImage* map_y = cvCreateImage( cvSize(640,480), IPL_DEPTH_32F, 1 ); 
	
	
	CvMat *	camera_matrix = (CvMat*)cvLoad("Intrinsics_RGB.xml"); 
	CvMat *	distortion_coeffs = (CvMat*)cvLoad("Distortion_RGB.xml"); 
	
	
	
	cvNamedWindow("undistorted");
	cvNamedWindow("original");
	
	char filename[100];
	
	
	cvInitUndistortMap(camera_matrix, distortion_coeffs, map_x, map_y);
	
	
	for (int i=0; i<image_cnt; i++)
	{
		sprintf(filename,"%s/rgb_%02d.jpg",prefix, i);
		
		
		IplImage* img = cvLoadImage(filename, 1);
		IplImage* img_undistorted = cvCloneImage(img);
		
		cvRemap(img, img_undistorted, map_x, map_y, CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS, CV_RGB(255,0,0));
		
		cvShowImage("original", img);
		cvShowImage("undistorted", img_undistorted);
		
		
		sprintf(filename,"%s/rgb_DIST_%02d.jpg",prefix, i);
		cvSaveImage(filename, img_undistorted);
		return 0;
		
		cvWaitKey( 500 );
	}
	
	
	
#endif
	
	return 0;
	
	
}
