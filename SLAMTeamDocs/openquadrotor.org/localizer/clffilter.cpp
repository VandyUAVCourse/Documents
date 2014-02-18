#include "localizemap.hh"
#include <math_stuff/gridlinetraversal.hh>
#include <math_stuff/logutils.hh>
#include "localizer.hh"
#include <istream>
#include <fstream>
#include <sstream>
#include <list>
#include <cstring>
using namespace std;

const char *message[]={
  "clffilter: filters the spurious readgins due to the sick interpolation",
  "usage clffilter <clf_file>",
  "options:",
  " -crop <distance>                     : crop the scans at distance <crop>",
  " -filt <incidence> <distance>         : filter beams whose incidence is smaller than <incidence>",
  " -genScans                            : generats a gnuplot file for each scan",
  0
};


int main (int argc, const char ** argv){

  if (argc<2){
    const char**v=message;
    while (*v){
      cout << *v << endl;
      v++;
    }
    return 0;
  }
  double filterIncidence=2.*M_PI/180.;
  double filterDistance =1.;
  double crop=-1;
  bool   genScans=0;
  const char* logfile=0;
  int c=1;
  while (c<argc){
    if (!strcmp(argv[c],"-filt")){
      c++;
      filterIncidence=atof(argv[c]);
      filterIncidence*=M_PI/180;
      c++;
      filterDistance=atof(argv[c]);
      c++;
    } else if (!strcmp(argv[c],"-genScans")){
      genScans=true;
      c++;
    } else if (!strcmp(argv[c],"-crop")){
      c++;
      crop=atof(argv[c]);
      c++;
    } else if (! logfile){
      logfile=argv[c];
      c++;
      break;
    }
  }

  if (! logfile){
    cout << "pgm2lmap: image file not specified" << endl;
    cout << "          Run the program without arguments for the help banner.";
    return 0;
  }

  cout << "clffilt, filering log "<< logfile;
  cout << "Parameters: " << endl;
  cout << " filter     =" << filterIncidence  <<  " " << filterDistance << endl;


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
  
  int j=0;
  for ( std::list<Data*>::iterator it=log.begin(); it!=log.end(); it++){
    RobotLaser * pscan= dynamic_cast<RobotLaser *>(*it);
    if (pscan) {
      pscan->sickFilter(1,filterDistance,filterIncidence);
      if (crop>0){
	pscan->crop(crop);
      }
      if (genScans){
	char filename[1024];
	sprintf(filename, "scan-%05d.dat", j);
	ofstream scanStream(filename);
	std::vector<DVector2> cartesian=pscan->cartesian();
	scanStream << "#scan " << j << endl;
	scanStream << "#LaserPose " << pscan->laserPose.x() << " " << pscan->laserPose.y() << " " << pscan->laserPose.theta() << endl;
	
	for (int i=0; i<(int)cartesian.size(); i++){
	  scanStream << cartesian[i].x() << " " << cartesian[i].y() << endl;
	}
	scanStream.close();
	j++;
      }
    }

    (*it)->write(cout);
    cout << endl;
  }

  return 0;
}
