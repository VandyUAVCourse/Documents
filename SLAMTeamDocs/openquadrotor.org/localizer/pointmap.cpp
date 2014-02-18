#include <iostream>
#include <iomanip>
#include <cmath>
#include <assert.h>
#include <values.h>
#include "pointmap.hh"

using namespace std;

PointMapCell::PointMapCell(const DVector2& _accumulator, int _count){
  accumulator=_accumulator;
  count=_count;
}



PointMap::PointMap(const IVector2& size, double resolution, const DVector2& offset, PointMapCell& unknown)
  :GridMap<PointMapCell>(size, resolution, offset, unknown)
{
}

PointMap::PointMap()
  :GridMap<PointMapCell>()
{
}

PointMap::PointMap(const PointMap& m):GridMap<PointMapCell>((const GridMap<PointMapCell>&)m)
{}

PointMap& PointMap::operator=(const PointMap& m){
  GridMap<PointMapCell>& gm=*this;
  gm=(GridMap<PointMapCell>)m;
  return *this;
}

PointMap PointMap::resize(const DVector2 min, DVector2 max, PointMapCell& unknownCell){
  PointMap resized;
  GridMap<PointMapCell>* presized=(GridMap<PointMapCell>*) &resized;
  GridMap<PointMapCell>* pcurrent=(GridMap<PointMapCell>*) this;
  *presized=pcurrent->resize(min, max, unknownCell);
  return resized;
}




void PointMap::saveToGnuplot(std::ostream& os){
  os << "# PointMap" << endl;
  int height=this->size.y();
  int width=this->size.x();
  for (int y=0; y<height; y++)
    for (int x=0; x<width; x++){
      PointMapCell& c=this->cell(IVector2(x,y));
      if (c.count){
	DVector2 p=c;
	os << p.x() << " " << p.y() << endl;
      }
	
    }
}

