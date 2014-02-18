#include "logutils.hh"
#include <iomanip>


  using namespace std;

  void Data::write(std::ostream& os){
    os << line;
  }
  
  RawLaser::RawLaser() : laserParams(0, 180, -M_PI/2, M_PI/180., 50.,0.1, 0){
    type=RawLaserType;
  }
  
  
  RobotLaser::RobotLaser(){
    Data::type=RobotLaserType;
  }

  void RawLaser::write(std::ostream& os){
    os << ios::fixed;
    os << tag << " " << laserParams.type  << " " 
       << laserParams.firstBeamAngle << " " 
       << laserParams.fov << " "
       << laserParams.angularStep << " " 
       << laserParams.maxRange << " "
       << laserParams.accuracy << " " 
       << laserParams.remissionMode << " "; 
    
    os << ranges.size() << " ";
    
    for (int i=0; i< (int)ranges.size(); i++){
      os << setprecision(2);
      os << ranges[i] << " ";
    }
    os << " 0 ";
    os << setprecision(6);
    os << timestamp << " x " << timestamp;
  }
  
  void RobotLaser::write(std::ostream& os){
    os << ios::fixed;
    os << tag << " " << laserParams.type  << " " 
       << laserParams.firstBeamAngle << " " 
       << laserParams.fov << " "
       << laserParams.angularStep << " " 
       << laserParams.maxRange << " "
       << laserParams.accuracy << " " 
       << laserParams.remissionMode << " "; 
    os << ranges.size() << " ";
    
    for (int i=0; i< (int)ranges.size(); i++){
      os << setprecision(2);
      os << ranges[i] << " ";
    }
    os << " 0 ";

    
    os << setprecision(6);

    os << laserPose.x() << " " << laserPose.y() << " " << laserPose.theta() << " ";
    os << odomPose.x() << " " << odomPose.y() << " " << odomPose.theta() << " ";
    os << laser_tv  << " " 
       << laser_rv  << " " 
       << forward_safety_dist << " " 
       << side_safty_dist  << " " 
       << turn_axis << " ";
    os << timestamp << " x " << timestamp << endl;

  }

  void RawLaser::sickFilter(int window, double acceptanceThreshold, double incidenceAngle){
    double cii=cos(incidenceAngle);
    if (!(ranges.size()==180 || 
	  ranges.size()==181 ||
	  ranges.size()==360 ||
	  ranges.size()==361 )){
      return;
    } 
    std::vector<DVector2> coords(ranges.size());
    //compute the catresian coords
    
    std::vector<double> newRanges(ranges.size(), laserParams.maxRange);
    
    for (int i=0; i<(int)ranges.size(); i++){
      DVector2 p(ranges[i],0);
      coords[i]=laserParams.beams[i]*p;
    }
    
    int okCount=0;
    for (int i=window; i<(int)ranges.size()-window; i++){
      if (ranges[i]>=laserParams.maxRange)
	continue;
      if (ranges[i-window]>=laserParams.maxRange)
	continue;
      if (ranges[i+window]>=laserParams.maxRange)
      continue;
      
      DVector2 tangent=coords[i+window]-coords[i-window];
      double m=sqrt(tangent*tangent);
      if (m> acceptanceThreshold){
	newRanges[i]=ranges[i];
	okCount++;
	continue;
      }
      
      tangent=tangent*(1./m);
      DVector2 normal(-tangent.y(),tangent.x());
      DVector2 incident(laserParams.beams[i].rotationMatrix[0][0],
			laserParams.beams[i].rotationMatrix[0][1]);
      if (fabs(normal*incident)<cii){
      newRanges[i]=ranges[i];
      okCount++;
      }
    }
    cerr << "F: " << okCount << "/" << ranges.size() << endl;
    ranges=newRanges;
  }


  void RawLaser::crop(double distance){
    for (int i=0; i<(int)ranges.size(); i++){
      if (ranges[i]>distance)
	ranges[i]=laserParams.maxRange;
    }
  }

  std::vector<DVector2> RawLaser::cartesian(){
    std::vector<DVector2> coords;
    //compute the catresian coords
    
    for (int i=0; i<(int)ranges.size(); i++){
      DVector2 p(ranges[i],0);
      if (ranges[i]<laserParams.maxRange)
      coords.push_back(laserParams.beams[i]*p);
    }
    return coords;
  }


  Data* readLogLine(istream& log){
#define BUFSIZE 20000
    char line[BUFSIZE];
    log.getline(line, BUFSIZE);
    istringstream ss(line);
    string tag;
    ss>> tag;
    
    int type;
    double angle, fov, res, maxrange, acc;
    int remission_mode;
    bool newlaser=false;
  
    bool robotLaserFound=false;
    bool imuFound=false;
    bool rawLaserFound=false;
    
    if (tag=="ROBOTLASER1" || tag=="ROBOTLASER2"){
      newlaser=true;
      ss >> type >> angle >> fov >> res >> maxrange >> acc >> remission_mode;
      robotLaserFound=true;
    } 

    if (tag=="RAWLASER1" || tag=="RAWLASER2"){
      newlaser=true;
      ss >> type >> angle >> fov >> res >> maxrange >> acc >> remission_mode;
    } 

    if (tag=="IMU")
    imuFound=true;
    
    if (tag=="FLASER"){
      tag="ROBOTLASER1";
      robotLaserFound=true;
    }
    
    RawLaser *pscan=0;
  
    if (rawLaserFound)
      pscan=new RawLaser;
    if (robotLaserFound)
      pscan=new RobotLaser;

    if (robotLaserFound || rawLaserFound) {
      RawLaser& scan(*pscan);
      int beams;
      ss >> beams;
      //cerr << "# beams=" << beams << endl;
      
    if (newlaser){
      scan.laserParams=LaserParameters(type, beams, angle, res, maxrange, acc, remission_mode);      
    } else {
      if (beams==180 ||  beams==181){
	scan.laserParams=LaserParameters(0, beams, -M_PI/2, M_PI/180., 50., 0.1, 0);      
      } else if (beams==360 || beams ==361){
	scan.laserParams=LaserParameters(0, beams, -M_PI/2, M_PI/360., 50., 0.1, 0);      
      } else {
	cerr << "unknown scanner model" << endl;
	return false;
      }
    }
    
  
    scan.ranges.resize(beams);
    for (int i=0; i<beams; i++){
      ss >> scan.ranges[i];
    }
    
    if (newlaser){
      int rems;
      ss >> rems;
      for (int i=0; i<rems; i++){
	double z;
	ss >> z;
      }
      //cerr << "#remissions=" << rems << endl;
    }
    
    if (robotLaserFound){
      RobotLaser& scan(* dynamic_cast<RobotLaser*>(pscan) );
      double x,y,theta;
      ss >> x >> y >> theta;
      scan.laserPose=DPose2(x,y,theta);
      ss >> x >> y >> theta;
      scan.odomPose=DPose2(x,y,theta);
      
      DTransformation2 globalLaserPose(scan.laserPose);
      DTransformation2 globalRobotPose(scan.odomPose);
      DTransformation2 laserPoseOnRobot=globalRobotPose.inv()*globalLaserPose;

      //DPose2 lgp=globalLaserPose.toPoseType();
      //DPose2 rgp=globalRobotPose.toPoseType();
      //DPose2 lpr=laserPoseOnRobot.toPoseType();

      //cerr << "globalRobotPose= " << rgp.x() << " " << rgp.y() << " " <<rgp.theta() << endl;
      //cerr << "globalLaserPose= " << lgp.x() << " " << lgp.y() << " " <<lgp.theta() << endl;
      //cerr << "laserPoseOnRobot=" << lpr.x() << " " << lpr.y() << " " <<lpr.theta() << endl;

      scan.laserParams.laserPose=laserPoseOnRobot;

      if (newlaser)
	ss >> scan.laser_tv >>  scan.laser_rv >>  scan.forward_safety_dist >> scan.side_safty_dist >> scan.turn_axis;
      else 
	scan.laser_tv =  scan.laser_rv =  scan.forward_safety_dist = scan.side_safty_dist = scan.turn_axis =0.0;
    }
    ss >> scan.timestamp;
    string h;
    ss >> h;
    ss >> scan.timestamp;
    pscan->line=line;
    pscan->tag=tag;
    return pscan;
    } 
    if (imuFound){
      IMUMeasurement *pimu=new IMUMeasurement;
      IMUMeasurement& imu(*pimu);
      log >> imu.ax >> imu.ay >> imu.az >> imu.q0 >> imu.q1 >> imu.q2 >> imu.q3 >> imu.mx >> imu.my >> imu.mz >> imu.gx >> imu.gy >> imu.gz;
      imu.yaw = getYaw(imu.q0, imu.q1, imu.q2, imu.q3);
      pimu->line=line;
      pimu->tag=tag;
      return pimu;
    }

    Data* d=new Data();
    d->line=line;
    d->tag=tag;
    return d;
  }


  void boundaries(DVector2& min, DVector2& max, DPose2 offset, double maxrange, const RobotLaser& scan){
    DTransformation2 tp=DTransformation2(offset)*DTransformation2(scan.laserPose)*DTransformation2(scan.laserParams.laserPose);
    
    //robot pose;
    DVector2 rp=tp.translation();
    min.x()=min.x()<rp.x()?min.x():rp.x();
    min.y()=min.y()<rp.y()?min.y():rp.y();
    max.x()=max.x()>rp.x()?max.x():rp.x();
    max.y()=max.y()>rp.y()?max.y():rp.y();
    for (int i=0; i<(int)scan.ranges.size(); i++){
      double r=scan.ranges[i];
      if (r>=maxrange)
      continue;
      
      DVector2 bp(r,0);
      bp=scan.laserParams.beams[i]*bp;
      bp=tp*bp;;
      min.x()=min.x()<bp.x()?min.x():bp.x();
      min.y()=min.y()<bp.y()?min.y():bp.y();
      max.x()=max.x()>bp.x()?max.x():bp.x();
      max.y()=max.y()>bp.y()?max.y():bp.y();
    }
  }
  
  double getYaw(double qw, double qx, double qy, double qz){
    typedef Vector3<double> DVector3;
    typedef Quaternion<double> DQuaternion;
    
    DQuaternion q(qw, qx, qy, qz);
    DVector3 angles=q.toAngles();
    return angles.yaw();
  } 
  
