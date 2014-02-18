#ifndef GRIDMAP_HH
#define GRIDMAP_HH

#include "transformation2.hh"
#include <iostream>


/**Generic grid map class.
Supports copy constuction, assignment, resize, and indicidual cell access.*/
template <class T>
struct GridMap{
  typedef T CellType;
  /**the resolution of the grid map*/
  double resolution;
  /**the size of the grid map in cells*/
  IVector2 size;
  /**the location of the cell (0,0) in world coordinates*/
  DVector2 offset;

  /**Mapping function between world coordinates and map cells.
   @param wp: the world point
   @returns the indices of the cell corresponding to wp in the map
  */
  inline IVector2 world2map(const DVector2& wp) const {
    return IVector2(lrint((wp.x()-offset.x())/resolution),
		    lrint((wp.y()-offset.y())/resolution));
  }

  /**Mapping function between map cells and world coordinates.
   @param mp: the map cell
   @returns the coordinates of the cell mp in world coordinates
  */
  DVector2  map2world(const IVector2& mp){
    return DVector2((double) offset.x()+(resolution*(double)mp.x()),
		    (double) offset.y()+(resolution*(double)mp.y()));
  }


  /**Boudary check
     @param mp: the cell to be checked
     @returns true if a cell is in the map, false otherwise
  */
  bool isInside(const IVector2& mp){
    return mp.x()>0 && mp.y()>=0 && mp.x()<size.x() && mp.y()<size.y();
  }


  /**Cell accessor method. Returns the cell located at the indices passed as argument.
     @param p: the cell indices
     @returns the cell at indices p in the map
  */
  inline T& cell(const IVector2& p){
    return cells[p.x()][p.y()];
  }
  
  /**Const cell accessor method. Returns the cell located at the indices passed as argument.
     @param p: the cell indices
     @returns the cell at indices p in the map
  */
  inline const T& cell(const IVector2& p) const{
    return cells[p.x()][p.y()];
  }


  /**Constructs a map of a  given size, offset and resolution. The mapped is filled with unknown cells.
  @param  size: the size in cells
  @param resolution: the resolution of the map
  @param offset: the location of the cell 0,0 in world coordinates
  @param unknownCell: the cell value to be used for filling the map
  */
  GridMap(const IVector2& size, const double& resolution, const DVector2& offset, const T& unknownCell=T());

  /**Constructs an empty map*/
  GridMap();

  
  /**Copy constructor*/
  GridMap(const GridMap<T>& m);

  /**Assignment operator*/
  GridMap<T>& operator=(const GridMap<T>& m);

  /**Destructor*/
  ~GridMap();

  /**Resize operator
     It resizes the map so that the minimum represented world value will be in min and the maximum in max.
     Uninitialized cells will be padded with unknownval.
     @param min: the lower left corner in world coordinates
     @param max: the upper right corner in world coordinates
     @param unknownCell: the value to be used for padding new cells
  */
  GridMap<T> resize(const DVector2 min, DVector2 max, T& unknownCell);
  
  ///by Slawomir
  inline void setCells (const T& unknownCell){
	  for (int i=0; i<size.x(); i++){
		  for (int j=0; j<size.y(); j++){
			  cells[i][j]=unknownCell;
		  }
	  }
  };
  
  inline void setOffset(const DVector2& off){
	  offset = off;
  };
  
  ///end by Slawomir

protected:
  T** cells;
  void clear();
  void alloc(const IVector2& size);
};

template <class T>
void GridMap<T>::alloc(const IVector2& s){
  size=s;
  if (s*s==0){
    cells=0;
    return;
  }

  cells = new T*[s.x()];
  for (int i=0; i<s.x(); i++){
    cells[i]=new T[s.y()];
  }
}

template <class T>
void GridMap<T>::clear(){
  for (int i=0; i<size.x(); i++){
    delete [] cells[i];
  }
  delete [] cells;
  size=IVector2(0,0);
  cells=0;
}

template <class T>
GridMap<T>::GridMap(){
  cells=0;
  size=IVector2(0,0);
  resolution=0.1;
  offset=DVector2(0.,0.);
}

template <class T>
GridMap<T>::GridMap(const IVector2& s, const double& res, const DVector2& off, const T& unknownCell){
  size=s;
  resolution=res;
  offset=off;
  alloc(s);
  for (int i=0; i<size.x(); i++){
    for (int j=0; j<size.y(); j++){
      cells[i][j]=unknownCell;
    }
  }
}

template <class T>
GridMap<T> GridMap<T>::resize(const DVector2 min, DVector2 max, T& unknownCell){
  IVector2 newSize((int)((max.x()-min.x())/resolution), (int) ((max.y()-min.y())/resolution));
  GridMap<T> newMap(newSize, resolution, min, unknownCell);
  for (int i=0; i<newSize.x(); i++)
    for (int j=0; j<newSize.y(); j++){
      IVector2 c=world2map(newMap.map2world(IVector2(i,j)));
      if (isInside(c)){
	newMap.cells[i][j]=cells[c.x()][c.y()];
      }
    }
  return newMap;
}

template <class T>
GridMap<T>::GridMap(const GridMap<T>& m){
  size=m.size;
  resolution=m.resolution;
  offset=m.offset;
  alloc(size);
  for (int i=0; i<size.x(); i++){
    for (int j=0; j<size.y(); j++){
      cells[i][j]=m.cells[i][j];
    }
  }
}

template <class T>
GridMap<T>& GridMap<T>::GridMap::operator=(const GridMap<T>& m){

  if(size.x()!= m.size.x() || size.y()!= m.size.y()){
    clear();
    alloc(m.size);
  }

  size=m.size;
  resolution=m.resolution;
  offset=m.offset;

  for (int i=0; i<size.x(); i++){
    for (int j=0; j<size.y(); j++){
      cells[i][j]=m.cells[i][j];
    }
  }
  return *this;
}

template <class T>
GridMap<T>::~GridMap(){
  clear();
}


#endif
