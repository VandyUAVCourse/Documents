#include <iostream>
#include <iomanip>
#include <cmath>
#include <assert.h>
#include <values.h>
#include "localizemap.hh"

using namespace std;



LocalizeMapCell::LocalizeMapCell(float _occupancy, float _distance, bool _visited){
  occupancy=_occupancy;
  distance=_distance;
  visited=_visited;
  cost=MAXDOUBLE;
  parent=false;
}

LocalizeMapCell::LocalizeMapCell(unsigned char g){
  occupancy=(float)(float(255.-g)/255.);
  visited=0;
  distance=0;
  cost=MAXDOUBLE;
  parent=false;
}


LocalizeMap::LocalizeMap(const IVector2& size, double resolution, const DVector2& offset, LocalizeMapCell& unknown)
  :GridMap<LocalizeMapCell>(size, resolution, offset, unknown)
{
}

LocalizeMap::LocalizeMap()
  :GridMap<LocalizeMapCell>()
{
}

LocalizeMap::LocalizeMap(const LocalizeMap& m):GridMap<LocalizeMapCell>((const GridMap<LocalizeMapCell>&)m)
{}

LocalizeMap& LocalizeMap::operator=(const LocalizeMap& m){
  GridMap<LocalizeMapCell>& gm=*this;
  gm=(GridMap<LocalizeMapCell>)m;
  return *this;
}

LocalizeMap LocalizeMap::resize(const DVector2 min, DVector2 max, LocalizeMapCell& unknownCell){
  LocalizeMap resized;
  GridMap<LocalizeMapCell>* presized=(GridMap<LocalizeMapCell>*) &resized;
  GridMap<LocalizeMapCell>* pcurrent=(GridMap<LocalizeMapCell>*) this;
  *presized=pcurrent->resize(min, max, unknownCell);
  return resized;
}

std::istream& operator >> (std::istream& is, LocalizeMap & gm){
  string s;
  is >> s;
  if (s!="LOCALIZEMAP")
    return is;
  is >> s;
  IVector2 size;
  if (s!="SIZE")
    return is;
  is >> size.x() >> size.y();
  is >> s;
  if (s!="RESOLUTION")
    return is;
  double resolution;
  is >> resolution;
  is >> s;
  DVector2 offset;
  if (s!="OFFSET")
    return is;
  is >> offset.x() >> offset.y();
  cerr << "Off=" <<  offset.x()   << " " <<  offset.y() << endl;
  is >> s;
  if (s!="NORTH") {
    cerr << "old map format, exiting" << endl;
    exit(0);
    return is;
  } else 
    is >> gm.north();
  
  LocalizeMapCell unknown;
  gm=LocalizeMap(size, resolution, offset, unknown);
  for (int x=0; x<gm.size.x(); x++){
    for (int y=0; y<gm.size.y(); y++){
      is >> gm.cell(IVector2(x,y)).occupancy >> gm.cell(IVector2(x,y)).distance >> gm.cell(IVector2(x,y)).visited ;
      if (! is)
	return is;
    }
  }
  return is;
}

std::ostream& operator << (std::ostream& os, LocalizeMap & gm){
  os << "LOCALIZEMAP" << endl;
  os << "SIZE " << gm.size.x() << " " << gm.size.y() << endl;
  os << "RESOLUTION " << gm.resolution << endl;
  os << "OFFSET " << gm.offset.x() << " " << gm.offset.y() << endl;
  os << "NORTH " << gm._north << endl;
  os.setf(ios::fixed);
  os << setprecision(2);
  for (int x=0; x<gm.size.x(); x++){
    for (int y=0; y<gm.size.y(); y++){
      os << gm.cell(IVector2(x,y)).occupancy << " ";
      os << gm.cell(IVector2(x,y)).distance  << " ";
      os << gm.cell(IVector2(x,y)).visited   << " ";
    }
    os << endl;
  }
  return os;
}


void LocalizeMap::loadFromPGM(std::istream& is){
  //read the pgm header
  string s;
  is >> s;
  bool plain=false;
  if (! is || (s != "P5" && s!= "P2")){
    cerr << "ERROR: magic number=" << s << endl;
    return;
  }
  if( s== "P2")
    plain=true;

  int height=0, width=0, grayval=0;

  double resolution=0.1;
  double xmin=0.;
  double ymin=0.;
  while (height==0 || width==0 || grayval==0){
    is >> s;
    if (! s.length())
      continue;
    if (s.at(0)=='#'){
      if (s=="#resolution"){
	is >> resolution;
      } else
      if (s=="#xmin"){
	is >> xmin;
      } else
      if (s=="#ymin"){
	is >> ymin;
      } else {
	char buf[1024];
	is.getline(buf,1024);
      }
    } else {
      if (width==0){
	width=atoi(s.c_str());
	//cerr << __PRETTY_FUNCTION__ << ":width=" << s << endl;
      } else if (height==0){
	height=atoi(s.c_str());
	//cerr << __PRETTY_FUNCTION__ << ":height=" << s << endl;
      } else { 
	grayval=atoi(s.c_str());
	//cerr << __PRETTY_FUNCTION__ << ":bpp=" << s << endl;
      }

    }
  }

  int bpp=1;
  if (grayval>255)
    bpp=2;
  //cerr << "IMAGE_SIZE=" << width << " " << height << endl;

  DVector2 offset(xmin, ymin);
  LocalizeMapCell unknownCell;
  *this=LocalizeMap(IVector2(width,height), resolution, offset, unknownCell);
  for (int i=1; i<=height; i++)
    for (int x=0; x<width; x++){
      char c;
      unsigned short q;
      int y=height-i;
      if (bpp==1){
	if (! plain){
	  is.read(&c,1);
	} else {
	  is >> q;
	  c=(unsigned char) q;
	}
      } else {
	if (! plain){
	  is.read(((char*) (&q)) ,2);
	} else {
	  is >> q;
	}
	c=(unsigned char)(255*((double)q/(double)grayval));
      }
      this->cell(IVector2(x,y))=LocalizeMapCell((unsigned char)c);
      if (c != 127)
         this->cell(IVector2(x,y)).visited = 1;
    }
}

void LocalizeMap::saveToPGM(std::ostream& os){
  os << "P5" << endl;
  os << "#resolution " << this->resolution << endl;
  os << "#xmin "       << this->offset.x() << endl;
  os << "#ymin "       << this->offset.y() << endl;
  os << this->size.x() << " " << this->size.y() << " " << 255 << endl;
  
  int height=this->size.y();
  int width=this->size.x();
  for (int i=1; i<=height; i++)
    for (int x=0; x<width; x++){
      int y=height-i;
      unsigned char c=(unsigned char) this->cell(IVector2(x,y));
      os.put(c);
    }
}

void LocalizeMap::saveToPPM(std::ostream& os, const std::vector<IVector2>& beams, const IVector2& bestPose){
	os << "P6" << endl;
	os << "#resolution " << this->resolution << endl;
	os << "#xmin "       << this->offset.x() << endl;
	os << "#ymin "       << this->offset.y() << endl;
	os << this->size.x() << " " << this->size.y() << " " << 255 << endl;
	int height=this->size.y();
	int width=this->size.x();
	unsigned char map[width][height][3];
	for (int x=0; x<width; x++)
		for (int y=0; y<height; y++)
			map[x][y][0] = map[x][y][1] = map[x][y][2] = (unsigned char) this->cell(IVector2(x,y));
	
	int kx = 1;
	int ky = 1;
	
	for (uint i=0; i<beams.size(); i++){
		const IVector2& p	= beams[i];
		for (int dx = -kx; dx <= kx; dx++)
			for (int dy = -ky; dy <= ky; dy++){
				if (p.x()+dx >= 0 && p.x()+dx < width && p.y()+dy >= 0 && p.y()+dy < height){
					map[p.x()+dx][p.y()+dy][0] = (unsigned char) 255;
					map[p.x()+dx][p.y()+dy][1] = (unsigned char) 0;
					map[p.x()+dx][p.y()+dy][2] = (unsigned char) 0;
				}
			}
	}
	
	for (int dx = -kx; dx <= kx; dx++)
		for (int dy = -ky; dy <= ky; dy++){
			if (bestPose.x()+dx >= 0 && bestPose.x()+dx < width && bestPose.y()+dy >= 0 && bestPose.y()+dy < height){
				map[bestPose.x()+dx][bestPose.y()+dy][0] = (unsigned char) 0;
				map[bestPose.x()+dx][bestPose.y()+dy][1] = (unsigned char) 255;
				map[bestPose.x()+dx][bestPose.y()+dy][2] = (unsigned char) 0;
			}
		}
	
	for (int i=1; i<=height; i++)
		for (int x=0; x<width; x++){
		int y=height-i;
		unsigned char c1= map[x][y][0];
		unsigned char c2= map[x][y][1];
		unsigned char c3= map[x][y][2];
		os.put(c1);
		os.put(c2);
		os.put(c3);	
	}
}

  void LocalizeMap::saveToPGM(std::ostream& os, double fullThreshold){
  os << "P5" << endl;
  os << "#resolution " << this->resolution << endl;
  os << "#xmin "       << this->offset.x() << endl;
  os << "#ymin "       << this->offset.y() << endl;
  os << this->size.x() << " " << this->size.y() << " " << 255 << endl;
  
  int height=this->size.y();
  int width=this->size.x();
  for (int i=1; i<=height; i++)
    for (int x=0; x<width; x++){
      int y=height-i;
      unsigned char c=0;
      LocalizeMapCell& curr=this->cell(IVector2(x,y));
      if (! curr.visited)
	c=200;
      else {
	if (curr.occupancy<fullThreshold)
	  c=255;
      }
      os.put(c);
    }
}


  void LocalizeMap::saveDistanceMapToPGM(std::ostream& os, bool showUnknown){
  float maxDistance=0.;
  float minDistance=MAXDOUBLE;
  for (int x=0; x<this->size.x(); x++){
    for (int y=0; y<this->size.y(); y++){
      float d=this->cell(IVector2(x,y)).distance;
      maxDistance=d>maxDistance?d:maxDistance;
      minDistance=d<minDistance?d:minDistance;
    }
  }
  assert (maxDistance>0.);

  os << "P5" << endl;
  os << "#MAP TYPE = distance map" <<  endl;
  os << "#maxdistance " << maxDistance << endl;
  os << "#resolution " << this->resolution << endl;
  os << "#xmin "       << this->offset.x() << endl;
  os << "#ymin "       << this->offset.y() << endl;
  os << this->size.x() << " " << this->size.y() << " " << 255 << endl;
  
  int height=this->size.y();
  int width=this->size.x();
  cerr.setf(ios::fixed);
  cerr << setprecision(2);

  for (int i=1; i<=height; i++)
    for (int x=0; x<width; x++){
      int y=height-i;
      float f=255*(this->cell(IVector2(x,y)).distance/maxDistance);
      char c=(unsigned char) f;

      if (!showUnknown && ! this->cell(IVector2(x,y)).visited){
	c=255;
      }
      os.put(c);
    }
}



void LocalizeMap::fillVisited(float fullThreshold, float emptyThreshold){
  for (int x=0; x<this->size.x(); x++){
    for (int y=0; y<this->size.y(); y++){
      float c=this->cell(IVector2(x,y));
      assert(c<=1.);
      if (c>fullThreshold || c<=emptyThreshold){
	this->cell(IVector2(x,y)).visited=true;
      } else {
	this->cell(IVector2(x,y)).visited=false;
      }
    }
  }
}

void LocalizeMap::distanceMap(float distanceThreshold, float fullThreshold){
  for (int x=0; x<this->size.x(); x++){
    for (int y=0; y<this->size.y(); y++){
      float c=this->cell(IVector2(x,y)).occupancy;
      if (c>fullThreshold){
	this->cell(IVector2(x,y)).distance=0;
      } else {
	this->cell(IVector2(x,y)).distance=distanceThreshold;
      }
    }
  }
  double ds=sqrt(2)*this->resolution;
  double ls=this->resolution;
  
  for (int x=2; x<this->size.x()-1; x++)
    for (int y=2; y<this->size.y()-1; y++){
      float mval=distanceThreshold;
      mval=mval<this->cell(IVector2(x-1,y-1)).distance + ds ? mval:  this->cell(IVector2(x-1,y-1)).distance + ds;
      mval=mval<this->cell(IVector2(x-1,y)).distance   + ls ? mval:  this->cell(IVector2(x-1,y)).distance   + ls;
      mval=mval<this->cell(IVector2(x-1,y+1)).distance + ds ? mval:  this->cell(IVector2(x-1,y+1)).distance + ds;
      mval=mval<this->cell(IVector2(x,y-1)).distance   + ls ? mval:  this->cell(IVector2(x,y-1)).distance   + ls;
      mval=mval<this->cell(IVector2(x,y+1)).distance   + ls ? mval:  this->cell(IVector2(x,y+1)).distance   + ls;
      mval=mval<this->cell(IVector2(x+1,y-1)).distance + ds ? mval:  this->cell(IVector2(x+1,y-1)).distance + ds;
      mval=mval<this->cell(IVector2(x+1,y)).distance   + ls ? mval:  this->cell(IVector2(x+1,y)).distance   + ls;
      mval=mval<this->cell(IVector2(x+1,y+1)).distance + ds ? mval:  this->cell(IVector2(x+1,y+1)).distance + ds;
      mval=mval<distanceThreshold?mval:distanceThreshold;
      this->cell(IVector2(x,y)).distance=this->cell(IVector2(x,y)).distance<mval ? this->cell(IVector2(x,y)).distance : mval;
    }
  for (int x=this->size.x()-2; x>1; x--)
    for (int y=this->size.y()-2; y>1; y--){
      float mval=distanceThreshold;
      mval=mval<this->cell(IVector2(x-1,y-1)).distance + ds ? mval:  this->cell(IVector2(x-1,y-1)).distance + ds;
      mval=mval<this->cell(IVector2(x-1,y)).distance   + ls ? mval:  this->cell(IVector2(x-1,y)).distance   + ls;
      mval=mval<this->cell(IVector2(x-1,y+1)).distance + ds ? mval:  this->cell(IVector2(x-1,y+1)).distance + ds;
      mval=mval<this->cell(IVector2(x,y-1)).distance   + ls ? mval:  this->cell(IVector2(x,y-1)).distance   + ls;
      mval=mval<this->cell(IVector2(x,y+1)).distance   + ls ? mval:  this->cell(IVector2(x,y+1)).distance   + ls;
      mval=mval<this->cell(IVector2(x+1,y-1)).distance + ds ? mval:  this->cell(IVector2(x+1,y-1)).distance + ds;
      mval=mval<this->cell(IVector2(x+1,y)).distance   + ls ? mval:  this->cell(IVector2(x+1,y)).distance   + ls;
      mval=mval<this->cell(IVector2(x+1,y+1)).distance + ds ? mval:  this->cell(IVector2(x+1,y+1)).distance + ds;
      mval=mval<distanceThreshold?mval:distanceThreshold;
      this->cell(IVector2(x,y)).distance=this->cell(IVector2(x,y)).distance<mval ? this->cell(IVector2(x,y)).distance : mval;
    }
}

