#include <OpenCV/OpenCV.h>

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

int main (int argc, char * const argv[]) {
    
	
// 1: compute and save intrinsics and distortion
// 0: load matrices and show images
	
	int image_cnt = 46;
	char filename[200];
	
#if 0	
	
	
	
	IplImage* img;
	
	cvNamedWindow("color.png");
	
	int N = image_cnt*7*10;
	
	CvMat* object_points = cvCreateMat(N, 3, CV_32FC1);
	CvMat* image_points = cvCreateMat(N, 2, CV_32FC1);
	CvMat* point_counts = cvCreateMat(image_cnt, 1, CV_32SC1);
	
	CvMat* camera_matrix = cvCreateMat(3,3, CV_32FC1);
	CvMat* distortion_coeffs = cvCreateMat(5,1, CV_32FC1);
	
	
	for (int i=0; i<image_cnt; i++)
	{
		sprintf(filename, "grab1/view%04d/raw/color.png",i);
		
		
		img = cvLoadImage(filename, 1);
		
		CvPoint2D32f corners[70];
		int corner_count;
		
		IplImage* bw = cvCreateImage(cvSize(640,480),img->depth,1);
		cvCvtColor(img, bw, CV_BGR2GRAY);
		
		int found = cvFindChessboardCorners(img, cvSize(10,7), corners, &corner_count);
		assert(found>0 && corner_count == 7*10);
		
		cvFindCornerSubPix(bw, corners, corner_count, cvSize(5,5), cvSize(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS || CV_TERMCRIT_ITER, 50, 0.05));
		
		
		// copy Points into calibration array:
		for (int j=0; j<7*10; j++)
		{	
			cvSetReal2D(object_points, i*7*10+j,0, j%10*2.5); // width and height of small square: 2.5cm
			cvSetReal2D(object_points, i*7*10+j,1, j/10*2.5);
			cvSetReal2D(object_points, i*7*10+j,2, 0);
			
			cvSetReal2D(image_points, i*7*10+j,0, corners[j].x); 
			cvSetReal2D(image_points, i*7*10+j,1, corners[j].y);
			
		}
		cvSetReal1D(point_counts, i, corner_count);
		
		
		
		
		
		cvDrawChessboardCorners(img, cvSize(10,7), corners, corner_count, found);
		// cout << found << endl;
		if (found == 0)
			cout << filename << endl;
		
		cvShowImage("color.png", img);
		
		cvWaitKey(1);
		
	}
	
	cvSet1D(distortion_coeffs, 4, cvScalarAll(0));
	
	cvCalibrateCamera2(object_points, image_points, point_counts, cvGetSize(img), 
					   camera_matrix, distortion_coeffs,NULL,NULL,CV_CALIB_FIX_K3 );
	
	cout << "camera matrix: " << endl;
	for (int i=0; i<3; i++)
	{
		for (int j=0; j<3; j++)
		{
			cout << cvGet2D(camera_matrix, i, j).val[0] << " " ;
		}
		cout << endl;
	}
	
	
	cout << "distortion: " << endl;
	for (int i=0; i<5; i++)
		cout << cvGet1D(distortion_coeffs, i).val[0] << " ";
	cout << endl;
	
	
	cvSave("Intrinsics_RGB.xml",camera_matrix); 
	cvSave("Distortion_RGB.xml",distortion_coeffs);
	
#else
	
	CvMat *camera_matrix = (CvMat*)cvLoad("Intrinsics_RGB.xml"); 
	CvMat *distortion_coeffs = (CvMat*)cvLoad("Distortion_RGB.xml"); 
	
	
	IplImage* map_x = cvCreateImage( cvSize(640,480), IPL_DEPTH_32F, 1 ); 
	IplImage* map_y = cvCreateImage( cvSize(640,480), IPL_DEPTH_32F, 1 ); 

	
	cvInitUndistortMap(camera_matrix, distortion_coeffs, map_x, map_y);
	
	cvNamedWindow("undistorted");
//	cvNamedWindow("original");
	
	for (int i=0; i<image_cnt; i++)
	{
		sprintf(filename, "grab1/view%04d/raw/color.png",i);
		
		
		IplImage* img = cvLoadImage(filename, 1);
		IplImage* img_undistorted = cvCloneImage(img);
	
		cvRemap(img, img_undistorted, map_x, map_y,CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS, CV_RGB(255,0,0));
	
//		cvShowImage("original", img);
		cvShowImage("undistorted", img_undistorted);
		
		cvWaitKey(1000);
	}
#endif
	
	return 0;
	
	
}
