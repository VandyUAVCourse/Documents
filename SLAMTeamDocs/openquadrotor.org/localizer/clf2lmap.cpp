#include <istream>
#include <fstream>
#include <sstream>
#include <list>

#include "localizemap.hh"
#include "pointmap.hh"
#include <math_stuff/logutils.hh>
#include "localizer.hh"
#include <math_stuff/gridlinetraversal.hh>
#include <cstring>
using namespace std;

const char *message[]={
  "clf2lmap: builds a localization map out of a carmen log file",
  "usage clf2lmap [options] <clf_file> <lmap_file>",
  "options:",
  " -pmap <int> <string>                 : generates a pointMap in which every cell should contain at least <int> points",
  "                                        ouput written in <string>",
  " -debug                               : dumps images \"omap.pgm\" and \"dmap.pgm\" showing the generated",
  "                                        gridmap and the distance map",
  " -res    <meters>                     : the resolution of a grid cell",
  "                                        default [0.1m]",
  " -maxrange    <meters>                : the maximum range of the scanner",
  " -usablerange <meters>                : the maximum usable range of the scanner",
  " -border      <meters>                : add a border of +- x meters to the generated map",
  " -offset <x_meters> <y_meters> <theta_radians>",
  "                                       : the offset of the robot when taking the initial scan",
  "                                        default (0, 0, 0)",
  " -full <occval>                       : if the occupancy of a cell is above this value",
  "                                        it will be considered full in the generation of the distance map",
  " -dmapDist <meters>                   : distance up to which propagate the distances in the distance map",
  " -filt <incidence> <distance>         : filter beams whose incidence is smaller than <incidence>",
  "                                        and where the distance between consecutive readings is "
  "                                        smaller than <distance>",
  " -zerooffset                          : 0,0 is in the lower left corner of the map",
  0
};

void integrateScan(PointMap& pmap, LocalizeMap& lmap, DPose2 offset, double maxrange, double usableRange, const RobotLaser& scan){
  DTransformation2 tp=DTransformation2(offset)*DTransformation2(scan.laserPose)*DTransformation2(scan.laserParams.laserPose);
  
  //robot pose;
  DVector2 rp=tp.translation();
  IVector2 start=lmap.world2map(rp);
  for (int i=0; i<(int)scan.ranges.size(); i++){
    double r=scan.ranges[i];
    if (r>=maxrange)
      continue;
    
    bool cropped=false;
    if (r>usableRange){
      r=usableRange;
      cropped=true;
    }
    static GridLineTraversalLine line;
    DVector2 bp(r,0);
    bp=scan.laserParams.beams[i]*bp;
    bp=tp*bp;;

    IVector2 end=lmap.world2map(bp);

    GridLineTraversal::gridLine(start, end, &line);
    for (int i=0; i<line.num_points; i++){
      lmap.cell(line.points[i]).distance+=1.;
    }
    if (! cropped){
      lmap.cell(end).occupancy+=1.;
      pmap.cell(end)+=bp;
    }
  }
}

int main (int argc, const char ** argv){

  if (argc<2){
    const char**v=message;
    while (*v){
      cout << *v << endl;
      v++;
    }
    return 0;
  }
  double maxrange=20;
  double usablerange=20;
  double resolution=0.1;
  double off_x=0., off_y=0., off_theta=0.;
  bool debug=false;
  double occFactor=5;
  double distance_threshold=LOCALIZE_MAP_DISTANCE_THRESHOLD;
  double full_threshold=LOCALIZE_MAP_DISTANCE_FULL_CELL_THRESHOLD;
  const char* logfile=0;
  const char* mapfile=0;
  const char* pmapfile=0;
  int pmapPoints=1;
  double border=10;
  bool doSickFilter=false;
  double filterIncidence=2.*M_PI/180.;
  double filterDistance =1.;
  bool zerooffset=false;
  int c=1;
  while (c<argc){
    if (!strcmp(argv[c],"-zerooffset")){
       zerooffset=true;
       c++;
    } else if (!strcmp(argv[c],"-debug")){
      debug=true;
      c++;
    } else if (!strcmp(argv[c],"-pmap")){
      c++;
      pmapPoints = atoi (argv[c]);
      c++;
      pmapfile=argv[c];
      c++;
    } else if (!strcmp(argv[c],"-res")){
      c++;
      resolution=atof(argv[c]);
      c++;
    } else 
    if (!strcmp(argv[c],"-offset")){
      c++;
      off_x=atof(argv[c]);
      c++;
      off_y=atof(argv[c]);
      c++;
      off_theta=atof(argv[c]);
      c++;
    } else
    if (!strcmp(argv[c],"-occFactor")){
      c++;
      occFactor=atof(argv[c]);
      c++;
    } else
    if (!strcmp(argv[c],"-maxrange")){
      c++;
      maxrange=atof(argv[c]);
      c++;
    } else
    if (!strcmp(argv[c],"-usablerange")){
      c++;
      usablerange=atof(argv[c]);
      c++;
    } else
    if (!strcmp(argv[c],"-border")){
      c++;
      usablerange=atof(argv[c]);
      c++;
    } else
    if (!strcmp(argv[c],"-full")){
      c++;
      full_threshold=atof(argv[c]);
      c++;
    } else
    if (!strcmp(argv[c],"-filt")){
      c++;
      filterIncidence=atof(argv[c]);
      filterIncidence*=M_PI/180;
      c++;
      filterDistance=atof(argv[c]);
      c++;
      doSickFilter=true;
    } else
    if (!strcmp(argv[c],"-dmapDist")){
      c++;
      distance_threshold=atof(argv[c]);
      c++;
    } else
    if (! logfile){
      logfile=argv[c];
      c++;
    } else {
      mapfile=argv[c];
      break;
    }
  }
  

  if (! logfile){
    cout << "pgm2lmap: image file not specified" << endl;
    cout << "          Run the program without arguments for the help banner.";
    return 0;
  }

  if (! logfile){
    cout << "pgm2lmap: map file not specified" << endl;
    cout << "          Run the program without arguments for the help banner.";
    return 0;
  }

  cout << "clf2lmap, generating map from file "<< logfile << " to file " << mapfile << endl;
  cout << "Parameters: " << endl;
  cout << " resolution=" << resolution << endl;
  cout << " offset    =" << off_x << " " << off_y << off_theta << endl;
  cout << " full      =" << full_threshold<< endl;
  cout << " dmapDist  =" << distance_threshold<< endl;
  cout << " maxrange  =" << maxrange << endl;
  cout << " usablerange  =" << usablerange  << endl;
  cout << " occFactor  =" << occFactor  << endl;
  cout << " filter     =" << filterIncidence  <<  " " << filterDistance << endl;
  cout << " zerooffset =" << zerooffset << endl;



  DPose2 offset(off_x,off_y,off_theta);
  
  ifstream is(logfile);
  if (! is)
    return 0;
  RobotLaser scan;
  std::list<Data*> log;
  while (is){
    Data* data=readLogLine(is);
    if (data){
      log.push_back(data);
      cerr << "*";
    }
  }
  
  cerr << "computing boundaries.." << endl;
  DVector2 min(MAXDOUBLE,MAXDOUBLE), max(-MAXDOUBLE, -MAXDOUBLE);
  for ( std::list<Data*>::iterator it=log.begin(); it!=log.end(); it++){
    RobotLaser * pscan= dynamic_cast<RobotLaser *>(*it);
    if (! pscan)
      continue;
    RobotLaser& scan(*pscan); 
    if (doSickFilter){
      pscan->sickFilter(1,filterDistance,filterIncidence);
    }
    boundaries(min, max, offset, maxrange, scan);
  }
  cerr << "Boundaries= " << min.x() << " " << min.y() << " " << max.x() << " " << max.y() << endl;
  
  min=min-DVector2(border, border);
  max=max+DVector2(border, border);
  cerr << "Extended Boundaries= " << min.x() << " " << min.y() << " " << max.x() << " " << max.y() << endl;


  DVector2 dsize=max-min;
  IVector2 isize((int)(dsize.x()/resolution), (int)(dsize.y()/resolution));
  LocalizeMapCell unknown;
  unknown.occupancy=0;
  unknown.distance=0;
  unknown.visited=false;
  cerr << "Allocating map, size=" << isize.x() << " x " << isize.y() << endl;
  LocalizeMap lmap(isize, resolution, min, unknown);

  PointMapCell unknownP;
  PointMap pointMap(isize, resolution, min, unknownP);

  cerr << "integrating scans ... " << endl;
  
  double snorth=0, cnorth=1;
  bool takeImu=false;
  bool foundIMU=false;
  DTransformation2 previousPose;
  for ( std::list<Data*>::const_iterator it=log.begin(); it!=log.end(); it++){
    const RobotLaser * pscan= dynamic_cast<RobotLaser *>(*it);
    if (pscan){
      const RobotLaser& scan(*pscan); 
      integrateScan(pointMap, lmap, offset, maxrange, usablerange, scan);
      cerr << "*";
      takeImu=true;
      previousPose=DTransformation2(offset)*DTransformation2(scan.laserPose);
      continue;
    }
    const IMUMeasurement * pimu= dynamic_cast<IMUMeasurement *>(*it);
    if (pimu&&takeImu){
      foundIMU=true;
      const IMUMeasurement& imu(*pimu);
      DPose2 p=previousPose.toPoseType();
      double robotHeading=p.theta();
      double delta=robotHeading-imu.yaw;
      snorth+=sin(delta);
      cnorth+=cos(delta);
      takeImu=false;
      continue;
    }
  }

  cerr << "generating occupancy map " << endl;
  for (int x=0; x<lmap.size.x(); x++)
    for (int y=0; y<lmap.size.y(); y++){
      IVector2 p(x,y);
      double occ=occFactor*lmap.cell(p).occupancy;
      double visit=occ+lmap.cell(p).distance;
      if (visit<=0){
	lmap.cell(p).visited=false;
	lmap.cell(p).distance=0;
	lmap.cell(p).occupancy=0;
      } else {
	lmap.cell(p).occupancy=occ/visit;
	lmap.cell(p).distance=0;
	lmap.cell(p).visited=true;
      }
    }
  
  if (zerooffset){
    cerr << "Zerooffset selected" << endl;
    lmap.offset=DVector2(0.,0.);
  }
  if (pmapfile){
    if (zerooffset){
      cerr << "Zerooffset selected" << endl;
      pointMap.offset=DVector2(0.,0.);
    }
    ofstream os(pmapfile);
    cerr << "generating point map on file " << pmapfile << endl;
    for (int x=0; x<pointMap.size.x(); x++)
      for (int y=0; y<pointMap.size.y(); y++){
	IVector2 p(x,y);
	PointMapCell& c=pointMap.cell(p);
	if (c.count>=pmapPoints) {
	  DVector2 k=c;
	  os << k.x() << " " << k.y() << endl;
	}
      }
    os.close();
  }
  if (foundIMU==true)
    lmap.north()=atan2(snorth, cnorth);
  lmap.distanceMap(distance_threshold, full_threshold);
  ofstream os (mapfile);
  os << lmap;
  os.close();
  if (debug){
    os.open("clf2lmap_omap.pgm");
    lmap.saveToPGM(os, full_threshold);
    os.close();
    os.open("clf2lmap_dmap.pgm");
    lmap.saveDistanceMapToPGM(os, false);
    os.close();
  }
  return 0;
}
