#ifndef CALIBRATION_COMMON_H
#define CALIBRATION_COMMON_H

#include <vector>
#include <OpenCV/OpenCV.h>


using namespace std;


double computeError(const CvMat* F,
					const CvMat* rgb_corners,
					const CvMat* ir_corners, 
					const CvMat* rgb_distortion,
					const CvMat* ir_distortion, 
					const CvMat* rgb_intrinsics, 
					const CvMat* ir_intrinsics);

#endif // CALIBRATION_COMMON_H
