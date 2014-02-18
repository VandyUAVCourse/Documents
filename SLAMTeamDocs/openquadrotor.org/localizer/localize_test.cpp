#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <math_stuff/logutils.hh>
#include "localizer.hh"
#include <cstdio>
#include <sys/time.h>
#include <math_stuff/transformation3.hh>

using namespace std;


// this dumps the status of the localizer in a pgm image
void dumpStatus(char* filename, Localizer& l, LocalizeMap& background){
  LocalizeMap image(background);
  for (int i=0; i<(int)l.particles.size(); i++){
	 IVector2 p=l.localizeMap.world2map(DVector2(l.particles[i].pose.x(), l.particles[i].pose.y()));
	 image.cell(p).occupancy=1.;//l.particles[i].weight;
	 image.cell(p).visited=1;
  }

  IVector2Vector ib;
  l.remapScan(ib, l.bestPose()); 
  for (int i=0; i<(int)ib.size(); i++){
	 IVector2 p=ib[i];
	 if (l.localizeMap.isInside(p)){
		image.cell(p).occupancy=0;
		image.cell(p).visited=1;
	 }
  }

  ofstream os(filename);
  image.saveToPGM(os);
  os.close();
}



int main (int argc, const char**argv){
  if (argc < 2){
	 cerr << "usage: localize_test <map_filename> <logfile>" << endl;
	 return 0;
  }

  ifstream is (argv[1]);
  if (! is){
	 cerr << "file " << argv[1] << " not found" << endl;
	 return 0;
  }

  cerr << "loading localize map.... ";
  LocalizeMap om;
  is >> om;
  cerr << "Done!" << endl;

  cerr << " Offset=" << om.offset.x() << " " << om.offset.y() << endl;
  cerr << " Size=" <<   om.size.x() << " " << om.size.y() << endl;
  cerr << " Resolution=" << om.resolution << endl;
  cerr << " North=" << om.north() << endl;


  bool use_odom=true;
  
  // the default parameters are ok
  LocalizerParameters params;
  // this encapsulates a laser configuration
  //LaserParameters laserParams(0, 180, -M_PI/2, M_PI/180., 50, 0.1, 0);
  // this encapsulates a IMU configuration
  IMUParameters imuParams;

  // a trivial motion model 
  MotionModel motionModel;
  
  //the localizer....
  Localizer localizer;

  double forcedMaxrange=20;
	 
  //BEGIN: alex 66.carmen.log
  motionModel.ff=0.3;
  motionModel.fs=0.3;
  motionModel.fr=0.3;
  motionModel.ss=0.3;
  motionModel.sr=0.3;
  motionModel.rr=0.3;

  motionModel.sx=1;
  motionModel.sy=1;
  motionModel.sth=0.5;

  params.distanceMapThreshold=10;
  params.dynamicRestart=false; //was true
  params.minWeight=0.001;
  params.maxLocalizationRange=forcedMaxrange;
  int particles=1000;
  //END: alex
  
  
  localizer.params=&params;
  localizer.motionModel=&motionModel;
  
  //should be initialized with a map and a number of particles
  cerr << "Initializing... ";
  localizer.init(om, particles);
  cerr << "Done!" << endl;

  //after initialization it recomputes the distance map,
  //according to the given parameters (you may change the distance threshold


  //these dumps are for seeing the shit we are working on
  ofstream os("localizer_ogrid.pgm");
  localizer.localizeMap.saveToPGM(os);
  os.close();

  os.open("localizer_dgrid.pgm");
  localizer.localizeMap.saveDistanceMapToPGM(os,true);
  os.close();

  os.open("localizer.lmap");
  os << localizer.localizeMap;
  os.close();



  //this is my naive log reading :-).
  //I *do not want* any dependency at this level
  std::vector<double> ranges;
  DPose2 pose;
  ifstream log(argv[2]);

  bool firstRead=false;
  DPose2 oldPose;

#define BUFSIZE 20000



  int count=0;
  LocalizeMap image(localizer.localizeMap);
  double ts=0;

  struct timeval tv, ntv;
  while (log){
	 Data* logRecord=readLogLine(log); // here we read a line from the log;

	 // FOR_PATRIK if you replace the reading of the log with the processing of a CARMEN message, you are nearly done :-).
	 // all informations are already in
	 // you should just output the particles

	 ts+=0.1;

	 bool updated=false;

	 RobotLaser* scanWithPose=dynamic_cast<RobotLaser*>(logRecord); // attempt to cast it to a robotlaser record;
	 if (scanWithPose){ // if it goes well then do something
		pose=scanWithPose->odomPose;
		//cerr << "  robot pose=" << pose.x() << " " << pose.y() << " " << pose.theta()<< endl;

		if (! firstRead){
	oldPose=pose;
	firstRead=true;
	updated=true;
		} else {
	//in readings are the readings of the laser
	//in pose the current pose
	//in oldpose the previous pose
	//we compute the motion between oldpose and pose,
	//and we update the filter.
	
	gettimeofday(&tv, 0);
	
	if (use_odom){
		 DTransformation2 told(oldPose);
		 DTransformation2 tnew(pose);
		 DTransformation2 tdelta=told.inv()*tnew;
		 DPose2 dpose=tdelta.toPoseType();
		 localizer.updateMotion(dpose);
		 oldPose=pose;
		 updated=localizer.updateObservation(scanWithPose->ranges, scanWithPose->laserParams);
	} else {
		 updated=localizer.updateObservation(scanWithPose->ranges, scanWithPose->laserParams, ts, true);
	}
	
	gettimeofday(&ntv, 0);
		}
	 }
	 
	 //if an update happens, we dump the image and print some bullshits
	 if (updated){
		double dt=(ntv.tv_sec-tv.tv_sec)*1e3+(ntv.tv_usec-tv.tv_usec)*1e-3;
		cerr << "!(" << dt << ")";
		DPose2 mean;
		CovarianceMatrix covariance;
		bool isBounded;
		bool isLocalized=localizer.hasConverged(mean, covariance, isBounded);
		cerr << "[" << (isBounded?"B":"N") << (isLocalized?"L":"U") <<"]:f="<< localizer.observationFitting() << endl; 
		count++;
		char dumpName[1000];
		sprintf(dumpName, "ldump-%05d.pgm", count);
		dumpStatus(dumpName, localizer, image);
	 }else {
		cerr <<".";
	 }

	 if (logRecord)
		delete logRecord;
  }
}


