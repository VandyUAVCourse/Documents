#include <iostream>
#include <fstream>
#include <cstring>
#include <assert.h>
#include <values.h>
#include <deque>
#include <sys/time.h>
#include <math_stuff/logutils.hh>

#include "cmatcher.hh"

using namespace std;

int main(int argc, char** argv){
  double croppingDistance=std::numeric_limits<double>::max();
  double icroppingDistance=std::numeric_limits<double>::max();
  
  double linearUpdate=0.3;
  double angularUpdate=0.2;
  double resolution=0.05;
  int historyLength=100;
  
  


  double linearDistance=0;
  double angularDistance=0;
  const char* filename=0;

  if (argc<2)
    return 0;
  
  
  int c=1;
  while (! filename && c<argc){
    if (! strcmp(argv[c],"-linearUpdate")){
      c++;
      linearUpdate=atof(argv[c]);
      c++;
    } else if (! strcmp(argv[c],"-angularUpdate")){
      c++;
      angularUpdate=atof(argv[c]);
      c++;
    } else if (! strcmp(argv[c],"-crop")){
      c++;
      croppingDistance=atof(argv[c]);
      c++;
    } else if (! strcmp(argv[c],"-icrop")){
      c++;
      icroppingDistance=atof(argv[c]);
      c++;
    } else if (! strcmp(argv[c],"-history")){
      c++;
      historyLength=atoi(argv[c]);
      c++;
    } else if (! strcmp(argv[c],"-resolution")){
      c++;
      resolution=atof(argv[c]);
      c++;
    } else
      filename=argv[c];
  }
  ifstream is(filename);
  if (! is)
    return 0;

  bool first=true;
  CMatcher cmatcher(resolution , 
			(M_PI/720) /*double angularResolution=*/, 
			resolution*4 /*fillingDistance*/, 
			0.4  /*minScore*/, 
			100 /*workspaceRadius*/);
  cmatcher.matchingCroppingDistance=croppingDistance;
  cmatcher.integrationCroppingDistance=icroppingDistance;
  cmatcher.matchingSparsificationDistance=resolution;

  std::deque<RobotLaser> previousScans;
  DTransformation2 previousCorrectedPose(0.,0.,0.);
  DTransformation2 previousUncorrectedPose(0.,0.,0.);
  ofstream path("path.dat");
  ofstream odom("odom.dat");

  ofstream correctedLog("corrected.clf");

  //load the log and compute the normals
  while (is){
    Data* logRecord=readLogLine(is);
    if (!logRecord)
      continue;
    RobotLaser* laser=dynamic_cast<RobotLaser*>(logRecord);
    if (!logRecord){
      delete logRecord;
      continue;
    }
    DTransformation2 currentPose=previousCorrectedPose;

    if (! first){
      DTransformation2 delta=previousUncorrectedPose.inv()*DTransformation2(laser->odomPose);
      currentPose=currentPose*delta;
      DVector2 translation=delta.translation();
      double rotation=delta.rotation();
      linearDistance+=sqrt(translation*translation);
      angularDistance+=fabs(rotation);
      if (linearDistance<linearUpdate && angularDistance<angularUpdate){
	delete laser;
	continue;
      } else {
	linearDistance=0;
	angularDistance=0;
      }
      DPose2 compilerWeakness=currentPose.toPoseType();
      cmatcher.scanMatch(*laser, compilerWeakness);
      
      if (cmatcher.results().size()){
	currentPose=cmatcher.results()[0]->pose;
      }
    }
    RobotLaser corrected(*laser);
    corrected.odomPose=currentPose.toPoseType();
    corrected.laserPose=(currentPose*DTransformation2(corrected.laserParams.laserPose)).toPoseType();
    previousScans.push_back(corrected);
    if ((int)previousScans.size()>historyLength){
      previousScans.pop_front();
    }

    DPose2 p = currentPose.toPoseType();
    cerr << "  q.size=" << previousScans.size() << "\t  #results=" << cmatcher.results().size() << "\t pose=" << p.x() << " " << p.y() << " " << p.theta() << endl;
    cmatcher.clear();
    previousCorrectedPose=currentPose;
    previousUncorrectedPose=laser->odomPose;
    
    corrected.write(correctedLog);
    corrected.write(cout);
    cout << flush;
    first=false;
    
    bool changeReference=true;
    std::deque<RobotLaser>::reverse_iterator sit=previousScans.rbegin();
    while(sit!=previousScans.rend()){
      cmatcher.integrateScan(*sit,sit->odomPose,changeReference);
      changeReference=false;
      sit++;
    }
    
  }
  correctedLog.close();

}
