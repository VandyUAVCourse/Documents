#include "lasertransformator.h"


void LaserTransformator::calculate3dLaserPoints(const vector<double>& ranges, const vector<double>& angles, vector<DVector3>& points, const Quaternion<double> rotation, const bool noIMU_YAW){
	/// calculate 3d points of the laser
	points.resize(ranges.size());
	assert (ranges.size() == angles.size());
	DVector3 rot = rotation.toAngles();
	
	const double cr = cos(rot.x());
	const double sr = sin(rot.x());
	const double cp = cos(rot.y());
	const double sp = sin(rot.y());
	const double cy = cos(rot.z());
	const double sy = sin(rot.z());
	for (uint i=0; i<ranges.size(); i++){
		const double& alpha = angles[i];
		const double& r = ranges[i];
		///laser
		points[i] = DVector3(cos(alpha) * r, sin(alpha) * r, 0);
		///roll
		points[i] = DVector3( points[i].x() , 
									 cr * points[i].y(), 
									 sr * points[i].y());
		///pitch
		
		points[i] = DVector3( cp * points[i].x() + sp * points[i].z(),
								    points[i].y(),
									 cp * points[i].z() - sp * points[i].x());
		if (noIMU_YAW)
			continue;
		///yaw
		points[i] = DVector3( cy * points[i].x() - sy * points[i].y(),
									 sy * points[i].x() + cy * points[i].y(),
									 points[i].z());
	}
	
}

void LaserTransformator::calculate2dLaserPoints(const vector<double>& ranges, const vector<double>& angles, vector<DVector2>& points, const Quaternion<double> rotation, const bool noIMU_YAW){
	/// calculate 2d projection of the laser
	points.resize(ranges.size());
	assert (ranges.size() == angles.size());
	vector<DVector3> points3d(ranges.size());
	calculate3dLaserPoints(ranges, angles, points3d, rotation, noIMU_YAW);
	assert (points3d.size() == points.size());
	for (uint i=0; i<points3d.size(); i++)
		points[i] = DVector2(points3d[i].x(), points3d[i].y());
}

void LaserTransformator::calculate3dMirrorPoints(const vector<double>& ranges, const vector<double>& angles, vector<DVector3>& points, const Quaternion<double> rotation, const double dist_laser_mirror, vector<DVector3>& mirror, const bool noIMU_YAW){
	/// calculate 3d points of the laser via the mirror
	
	points.resize(ranges.size());
	mirror.resize(ranges.size());
	assert (ranges.size() == angles.size());
	Transformation3<double> t_imu;
	if (noIMU_YAW){
		DVector3 rot = rotation.toAngles();
		t_imu = Transformation3<double>(DVector3(0,0,0), Quaternion<double>(rot.x(), rot.y(), 0));
	} else {
		t_imu = Transformation3<double>(DVector3(0,0,0), rotation);
	}
	const Transformation3<double> t_mirror(DVector3(dist_laser_mirror,0,0),Quaternion<double>(0,M_PI/2.,0));
	
	for (uint i=0; i<ranges.size(); i++){
		Transformation3<double> t_r(DVector3(ranges[i] - dist_laser_mirror, 0, 0), Quaternion<double>(0,0,0));
		Transformation3<double> t_laser (DVector3(0,0,0), Quaternion<double>(0,0,angles[i]));
		Transformation3<double> t_uptomirror = t_imu * t_laser * t_mirror;
		mirror[i] = t_uptomirror.translationVector;
		points[i] = (t_uptomirror * t_r).translationVector;
	}
}

void LaserTransformator::calculate3dMirrorPoints(const vector<double>& ranges, const vector<double>& angles, vector<DVector3>& points, const Quaternion<double> rotation, const double dist_laser_mirror, const bool noIMU_YAW){
	/// calculate 3d points of the laser via the mirror
	
	points.resize(ranges.size());
	assert (ranges.size() == angles.size());
	Transformation3<double> t_imu;
	if (noIMU_YAW){
		DVector3 rot = rotation.toAngles();
		t_imu = Transformation3<double>(DVector3(0,0,0), Quaternion<double>(rot.x(), rot.y(), 0));
	} else {
		t_imu = Transformation3<double>(DVector3(0,0,0), rotation);
	}
	const Transformation3<double> t_mirror(DVector3(dist_laser_mirror,0,0),Quaternion<double>(0,M_PI/2.,0));
	
	for (uint i=0; i<ranges.size(); i++){
		Transformation3<double> t_r(DVector3(ranges[i] - dist_laser_mirror, 0, 0), Quaternion<double>(0,0,0));
		Transformation3<double> t_laser (DVector3(0,0,0), Quaternion<double>(0,0,angles[i]));
		points[i] = (t_imu * t_laser * t_mirror * t_r).translationVector;
	}
}

