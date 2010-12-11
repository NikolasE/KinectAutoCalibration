//
// Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2009
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef CALIBRATION_COMMON_H
#define CALIBRATION_COMMON_H

#include <vector>
#include <OpenCV/OpenCV.h>


using namespace std;

//TODO: write points as matrices

double computeError(const CvMat* F,
					const CvMat* rgb_corners,
					const CvMat* ir_corners, 
					const CvMat* rgb_distortion,
					const CvMat* ir_distortion, 
					const CvMat* rgb_intrinsics, 
					const CvMat* ir_intrinsics);

#endif // CALIBRATION_COMMON_H
