#ifndef _LASER_TRANSFORMATOR_H_
#define _LASER_TRANSFORMATOR_H_

#include <assert.h>
#include <vector>
#include <cstdlib>
#include <stdint.h>
#include <math_stuff/transformation3.hh>
#include <math_stuff/transformation2.hh>

using namespace std;

///add imu + laser -> transform into 3d corrdiantes for the front laser points
///and 3d coordiantes for the mirror points
struct LaserTransformator{
	void calculate3dLaserPoints(const vector<double>& ranges, const vector<double>& angles, vector<DVector3>& points, const Quaternion<double> rotation, const bool noIMU_YAW = true);
	
	void calculate2dLaserPoints(const vector<double>& ranges, const vector<double>& angles, vector<DVector2>& points, const Quaternion<double> rotation, const bool noIMU_YAW = true);
	
	
	void calculate3dMirrorPoints(const vector<double>& ranges, const vector<double>& angles, vector<DVector3>& points, const Quaternion<double> rotation, const double dist_laser_mirror, vector<DVector3>& mirror, const bool noIMU_YAW = true);
	
	void calculate3dMirrorPoints(const vector<double>& ranges, const vector<double>& angles, vector<DVector3>& points, const Quaternion<double> rotation, const double dist_laser_mirror, const bool noIMU_YAW = true);
};
#endif // _LASER_TRANSFORMATOR_H_
