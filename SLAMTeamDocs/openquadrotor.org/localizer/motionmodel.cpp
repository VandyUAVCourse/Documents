#include "motionmodel.hh"


void MotionModel::prepareSampling(const DPose2& motion, double deltaT, double magnitude){
 wx  = ff*fabs(motion.x()) + fs*fabs(motion.y()) + fr*fabs(motion.theta()) + sx  * deltaT;
 wy  = fs*fabs(motion.x()) + ss*fabs(motion.y()) + sr*fabs(motion.theta()) + sy  * deltaT;
 wth = fr*fabs(motion.x()) + sr*fabs(motion.y()) + rr*fabs(motion.theta()) + sth * deltaT;
 wx*=magnitude;
 wy*=magnitude;
 wth*=magnitude;
}

DPose2 MotionModel::sampleMotion(const DPose2& motion) const {
 DPose2 p=motion;
 p.x()    = motion.x() + triangularSample(wx,0.);
 p.y()    = motion.y() + triangularSample(wy,0.);
 p.theta()= motion.theta() + triangularSample(wth,0.);
 p.theta()= atan2(sin(p.theta()), cos(p.theta()));
 return p;
}

