#ifndef __LOGUTILS_HH__
#define __LOGUTILS_HH__

#include "transformation2.hh"
#include "transformation3.hh"
#include "laserparameters.hh"
#include <iostream>
#include <istream>
#include <fstream>
#include <sstream>
#include <list>



  enum Type{RobotLaserType=0, RawLaserType=1, ImuType=2 };

  struct Data{
    Type type;
    double timestamp;
    std::string tag;
    std::string line;
    virtual void write (std::ostream& os);
    virtual ~Data(){}
  };

  struct RawLaser: public Data{
    std::vector<double> ranges;
    LaserParameters laserParams;
    RawLaser();
    virtual void write (std::ostream& os);
    void sickFilter(int window=1, double acceptanceThreshold=1., double incidenceAngle=0.05);
    void crop(double distance);
    std::vector<DVector2> cartesian();
  };


  struct RobotLaser: public RawLaser{
    DPose2 odomPose, laserPose;
    double laser_tv, laser_rv, forward_safety_dist, side_safty_dist, turn_axis;
    virtual void write (std::ostream& os);
    RobotLaser();
  };
  

  struct IMUMeasurement: public Data{
    IMUMeasurement(){type=ImuType;}
    double ax, ay, az; //accelerations 
    double q0, q1, q2, q3; // //overall heading
    double mx, my, mz; // magnetic field
    double gx, gy, gz; //gyros
    double yaw;
  };
  
  Data* readLogLine(std::istream& log);

  void boundaries(DVector2& min, DVector2& max, DPose2 offset, double maxrange, const RobotLaser& scan);

  /** helper function for retrieving the yaw from a quaternion */
  double getYaw(double qw, double qx, double qy, double qz);


#endif
