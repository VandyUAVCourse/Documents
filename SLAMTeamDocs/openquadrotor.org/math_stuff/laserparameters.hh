#ifndef LASERPARAMETERS_HH
#define LASERPARAMETERS_HH

#include "transformation2.hh"
#include <vector>


typedef Pose2<double> DPose2;
typedef Transformation2<double> DTransformation2;

struct LaserParameters{
  LaserParameters(int type, int beams, double firstBeamAngle, double angularStep, double maxRange, double accuracy, int remissionMode);
  LaserParameters(int beams, double firstBeamAngle, double angularStep, double maxRange);
  DTransformation2 laserPose;
  int type;
  double firstBeamAngle;
  double fov;
  double angularStep;
  double accuracy;
  int remissionMode;
  std::vector<DTransformation2> beams;
  double maxRange;
};

#endif
