#include "localizemap.hh"
#include <iostream>
#include <fstream>
#include <cstring>

using namespace std;


const char *message[]={
  "pgm2lmap: builds a localization map out of a pgm image",
  "usage pgm2lmap [options] <pgm_file> <lmap_file>",
  "options:",
  " -debug                               : dumps images showing the loaded gridmap and the distance map",
  "                                        in the file \"ogrid.pgm\" and \"dgrid.pgm\" respectively",
  " -res    <meters>                     : the resolution of a grid cell",
  "                                        default [0.1m]",
  " -offset <x_meters> <y_meters>        : the offset of the cell in 0,0, in world coordinates",
  "                                        default (0, 0)",
  " -unknown <occval_min> <occval_max>   : if the occupancy value is within this range,",
  "                                        the cell is considered as unknown",
  " -full <occval>                       : if the occupancy of a cell is above this value",
  "                                        it will be considered full in the generation of the distance map",
  " -dmapDist <meters>                   : distance up to which propagating the distances in the distance map",
  0
};

              
int main (int argc, const char**argv){
  if (argc<2){
    const char**v=message;
    while (*v){
      cout << *v << endl;
      v++;
    }
    return 0;
  }
  
  const char* pgmfile=0;
  const char* mapfile=0;
  double resolution=0.1;
  double off_x=0., off_y=0.;
  bool debug=false;
  double unknown_min=LOCALIZE_MAP_EMPTY_CELL_THRESHOLD;
  double unknown_max=LOCALIZE_MAP_FULL_CELL_THRESHOLD;
  double distance_threshold=LOCALIZE_MAP_DISTANCE_THRESHOLD;
  double full_threshold=LOCALIZE_MAP_DISTANCE_FULL_CELL_THRESHOLD;
  
  int c=1;
  while (c<argc){
    if (!strcmp(argv[c],"-debug")){
      debug=true;
      c++;
    } else
    if (!strcmp(argv[c],"-res")){
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
    } else
    if (!strcmp(argv[c],"-unknown")){
      c++;
      unknown_min=atof(argv[c]);
      c++;
      unknown_max=atof(argv[c]);
      c++;
    } else
    if (!strcmp(argv[c],"-full")){
      c++;
      full_threshold=atof(argv[c]);
      c++;
    } else
    if (!strcmp(argv[c],"-dmapDist")){
      c++;
      distance_threshold=atof(argv[c]);
      c++;
    } else
    if (! pgmfile){
      pgmfile=argv[c];
      c++;
    } else {
      mapfile=argv[c];
      break;
    }
  }

  if (! pgmfile){
    cout << "pgm2lmap: image file not specified" << endl;
    cout << "          Run the program without arguments for the help banner.";
    return 0;
  }

  if (! mapfile){
    cout << "pgm2lmap: map file not specified" << endl;
    cout << "          Run the program without arguments for the help banner.";
    return 0;
  }

  cout << "pgm2lmap, generating map from file "<< pgmfile << " to file " << mapfile << endl;
  cout << "Parameters: " << endl;
  cout << " resolution=" << resolution << endl;
  cout << " offset    =" << off_x << " " << off_y << endl;
  cout << " unknown   =" << unknown_min << " " << unknown_max << endl;
  cout << " full      =" << full_threshold<< endl;
  cout << " dmapDist  =" << distance_threshold<< endl;

  ifstream is (pgmfile);
  if (! is){
    cout << "file " << argv[1] << " not found" << endl;
    return 0;
  }

  LocalizeMap gm;
  cout << "loading from pgm... ";
  gm.loadFromPGM(is);
  cout << "done" << endl;
  gm.resolution=resolution;
  gm.offset.x()=off_x;
  gm.offset.y()=off_y;

  
  cout << "filling Visited && computing distanceMap" << endl;
  gm.fillVisited(unknown_max, unknown_min);
  gm.distanceMap(distance_threshold, full_threshold);
  cout << "done" << endl;

  ofstream os;
  
  if (debug){
    cout << "saving output to \"pgm2lmap_ogrid.pgm\"... ";
    os.open("pgm2lmap_ogrid.pgm");
    gm.saveToPGM(os);
    os.close();
    cout << "done" << endl;

    cout << "saving distance map to \"pgm2lmap_dgrid.pgm\"... ";
    os.open("pgm2lmap_dgrid.pgm");
    gm.saveDistanceMapToPGM(os, false);
    os.close();
    cout << "done" << endl;
  }
  

  cout << "saving file \"" << mapfile <<"\"... "  << endl;
  os.open(mapfile);
  os << gm;
  os.close();
  cout << "done" << endl;
  return 0;
}

