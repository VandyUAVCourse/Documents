#include "localizemap.hh"
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

int main (int argc, const char**argv){
  if (argv<0){
    cerr << "usage charmap_test <filename>" << endl;
    return 0;
  }

  ifstream is (argv[1]);
  if (! is){
    cerr << "file " << argv[1] << " not found" << endl;
    return 0;
  }

  LocalizeMap gm;
  
  cerr << "loadFromPGM" << endl;
  gm.loadFromPGM(is);
  cerr << "done" << endl;
  
  cerr << "filling Visited && computing distanceMap" << endl;
  gm.fillVisited();
  gm.distanceMap(5.);
  cerr << "done" << endl;

  ofstream os;
  cerr << "saveToPGM" << endl;
  os.open("gmt.pgm");
  gm.saveToPGM(os);
  os.close();
  cerr << "done" << endl;

  os.open("gmt_distance.pgm");
  gm.saveDistanceMapToPGM(os, false);
  os.close();
  
  

  cerr << "saveAsMap" << endl;
  os.open("gmt.lmap");
  os << gm;
  os.close();

  LocalizeMap nm;
  cerr << "done" << endl; 

  cerr << "loadFromMap" << endl;
  ifstream ms("gmt.lmap");
  ms >> nm;
  cerr << "done" << endl; 
  
  os.open("gmt2.pgm");
  nm.saveToPGM(os);
  os.close();

  os.open("gmt2_distancemMap.pgm");
  nm.saveDistanceMapToPGM(os);
  os.close();

  os.open("gmt2_distancemMap_free.pgm");
  nm.saveDistanceMapToPGM(os, false);
  os.close();
}
