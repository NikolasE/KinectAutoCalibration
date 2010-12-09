#include <OpenCV/OpenCV.h>

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

int main (int argc, char * const argv[]) {
	
	
// activate to get intrinsic parameters and distortion for both cameras	
#define CALIBRATION
	
// activate to get stereo calibration (needs CALIBRATION to be activated)
#define STEREO_CALIBRATION	
	
// load calibration for one camera and show undistorted images
//#define VIEW_CALIBRATION
	
	
	// theses values depend on the used chessboard
	const int corner_cnt_x = 10;
	const int corner_cnt_y = 7;
	const float square_size = 2.5;
	
	const int corner_cnt = corner_cnt_x*corner_cnt_y;
	
	
	// number of image pairs in img-folder
	int image_cnt = 3;
	
	
	
	char filename[200];
	CvMat *camera_matrix_ir;
	CvMat* distortion_coeffs_ir;
	
	CvMat *camera_matrix_rgb;
	CvMat* distortion_coeffs_rgb;
	
	/*
	 // show images pairwise
	 for (int i=20; i<image_cnt; i++)
	 {
	 sprintf(filename,"img/ir_%02d.jpg",i);
	 IplImage* ir = cvLoadImage(filename, 1);
	 
	 sprintf(filename,"img/RGB_%02d.jpg",i);
	 IplImage* rgb = cvLoadImage(filename, 1);
	 
	 cvNamedWindow("ir");
	 cvNamedWindow("rgb");
	 
	 cvShowImage("ir",ir);
	 cvShowImage("rgb", rgb);
	 
	 cvWaitKey(500);
	 }
	 
	 return 0;
	 */
	
	
	
#ifdef CALIBRATION
	
	IplImage* img;
	
	
	int N = image_cnt*7*10;
	
	CvMat* object_points = cvCreateMat(N, 3, CV_32FC1);
	CvMat* point_counts = cvCreateMat(image_cnt, 1, CV_32SC1);
	
	CvMat* image_points_ir = cvCreateMat(N, 2, CV_32FC1);
	CvMat* image_points_rgb = cvCreateMat(N, 2, CV_32FC1);
	
	
	camera_matrix_rgb = cvCreateMat(3,3, CV_32FC1);
	distortion_coeffs_rgb = cvCreateMat(5,1, CV_32FC1);
	
	
	camera_matrix_ir = cvCreateMat(3,3, CV_32FC1);
	distortion_coeffs_ir = cvCreateMat(5,1, CV_32FC1);
	
	
	// mode 0 for infrared images
	// mode 1 for rgb images
	for (int mode = 0; mode < 2; mode++)
	{
		
		for (int i=0; i<image_cnt; i++)
		{
			if (mode == 0)
				sprintf(filename,"img/ir_%02d.jpg",i);
			else
				sprintf(filename,"img/RGB_%02d.jpg",i);
			
			
			img = cvLoadImage(filename, 1);
			
			if (img == NULL)
			{
				// should not happen if images are correct
				cout << "null: " << filename << endl;
				continue;
			}
			
			CvPoint2D32f corners[corner_cnt];
			int corner_count;
			
			IplImage* bw = cvCreateImage(cvGetSize(img),img->depth,1);
			cvCvtColor(img, bw, CV_BGR2GRAY);
			
			
			int found = cvFindChessboardCorners(bw, cvSize(corner_cnt_x,corner_cnt_y), corners, &corner_count);
			//	assert(found>0 && corner_count == 7*10);
			
			if (found == 0)
			{
				// should not happen, if images are correct
				cout << "no corners on " << filename << endl;
			}
			
			// refinement of cornercoordinates
			cvFindCornerSubPix(bw, corners, corner_count, cvSize(7,7), cvSize(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));
			
			
			if (mode == 0)
			{
				// copy Points into calibration array:
				for (int j=0; j<corner_cnt; j++)
				{	
					cvSetReal2D(object_points, i*corner_cnt+j,0, j%10*square_size); // width and height of small square: 2.5cm
					cvSetReal2D(object_points, i*corner_cnt+j,1, j/10*square_size);
					cvSetReal2D(object_points, i*corner_cnt+j,2, 0);
					
					cvSetReal2D(image_points_ir, i*corner_cnt+j,0, corners[j].x); 
					cvSetReal2D(image_points_ir, i*corner_cnt+j,1, corners[j].y);
					
				}
				
			}
			else
			{
				for (int j=0; j<corner_cnt; j++)
					
				{	
					cvSetReal2D(image_points_rgb, i*corner_cnt+j,0, corners[j].x); 
					cvSetReal2D(image_points_rgb, i*corner_cnt+j,1, corners[j].y);
				}
				
			}	
			
			cvSetReal1D(point_counts, i, corner_count);
			// activate to see the refined chessboard corners
			/*
			 
			 cvNamedWindow("color.png");
			 cvDrawChessboardCorners(img, cvSize(10,7), corners, corner_count, found);
			 cvShowImage("color.png", img);
			 cvWaitKey(500);
			 */	
		}
		
		
		
		if (mode == 0)
		{
			
			// cameras are pretty good, fourth parameter is only for fisheye-cams
			cvSet1D(distortion_coeffs_ir, 4, cvScalarAll(0));
			
			cvCalibrateCamera2(object_points, image_points_ir, point_counts, cvGetSize(img), 
							   camera_matrix_ir, distortion_coeffs_ir,NULL,NULL,CV_CALIB_FIX_K3 );
			
			cvSave("Intrinsics_IR.xml",camera_matrix_ir); 
			cvSave("Distortion_IR.xml",distortion_coeffs_ir);
			
			cout << "infraRed: calibrated" << endl;
		}
		else
		{
			
			cvSet1D(distortion_coeffs_rgb, 4, cvScalarAll(0));
			
			cvCalibrateCamera2(object_points, image_points_rgb, point_counts, cvGetSize(img), 
							   camera_matrix_rgb, distortion_coeffs_rgb,NULL,NULL,CV_CALIB_FIX_K3 );
			
			cvSave("Intrinsics_RGB.xml",camera_matrix_rgb); 
			cvSave("Distortion_RGB.xml",distortion_coeffs_rgb);
			cout << "RGB: calibrated" << endl;
			
		}
		
	}
#endif
	
	
	
#ifdef STEREO_CALIBRATION	
	
	cout << "Starting Stereo Calibration" << endl;
	CvMat* R = cvCreateMat(3, 3, CV_32FC1);
	CvMat* T = cvCreateMat(3, 1, CV_32FC1);
	
	
	// the core function:
	cvStereoCalibrate(object_points, 
					  image_points_ir, 
					  image_points_rgb, 
					  point_counts, 
					  camera_matrix_ir, distortion_coeffs_ir, 
					  camera_matrix_rgb, distortion_coeffs_rgb, 
					  cvSize(640,480), R,T,NULL,NULL, cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5), CV_CALIB_FIX_INTRINSIC);
	
	cout << "R:" << endl;
	for (int i=0; i<3; i++)
	{
		for (int j=0; j<3; j++)
			cout << cvGet2D(R, i, j).val[0] << " ";
		cout << endl;
	}
	
	cout << "T:" << endl;
	for (int i=0; i<3; i++)
	{
		cout << cvGet1D(T, i).val[0] << " ";
	}
	cout << endl;
	
	cvSave("kinect_R.xml", R);
	cvSave("kinect_T.xml", T);
	
	
	
#endif
	
	
	
	
	
	
	
	
	
	
	
#ifdef VIEW_CALIBRATION
	
	
	IplImage* map_x = cvCreateImage( cvSize(640,480), IPL_DEPTH_32F, 1 ); 
	IplImage* map_y = cvCreateImage( cvSize(640,480), IPL_DEPTH_32F, 1 ); 
	
	
	CvMat *	camera_matrix = (CvMat*)cvLoad("Intrinsics_RGB.xml"); 
	CvMat *	distortion_coeffs = (CvMat*)cvLoad("Distortion_RGB.xml"); 
	
	
	cvInitUndistortMap(camera_matrix, distortion_coeffs, map_x, map_y);
	
	cvNamedWindow("undistorted");
	cvNamedWindow("original");
	
	for (int i=0; i<image_cnt; i++)
	{
		sprintf(filename, "img/rgb_%02d.jpg",i);
		
		
		IplImage* img = cvLoadImage(filename, 1);
		IplImage* img_undistorted = cvCloneImage(img);
		
		cvRemap(img, img_undistorted, map_x, map_y,CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS, CV_RGB(255,0,0));
		
		cvShowImage("original", img);
		cvShowImage("undistorted", img_undistorted);
		
		cvWaitKey(300);
	}
	
	
	
#endif
	
	return 0;
	
	
}
