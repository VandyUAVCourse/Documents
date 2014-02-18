#ifndef __POINTMAP_HXX__
#define __POINTMAP_HXX__

#include <math_stuff/gridmap.hh>
#include <iostream>

/**Cell of a point map.
It contains the vector sum of the endpoints falling in the cell.
*/
struct PointMapCell{
  DVector2 accumulator;
  int count;
  /**constructs a cell initialized with  the given parameters.
   @param accumulator: initial value of the accumulator
   @param count: number of points that fall in the cell
  */
  PointMapCell(const DVector2& accumulator=DVector2(0,0), int count=0);
  
  /**conversion to point type value*/
  inline operator DVector2() const {
    if (count)
      return (accumulator*(1./(double)count));
    return accumulator;
  };

  inline PointMapCell& operator += (const DVector2& p) {
    accumulator=accumulator+p;
    count++;
    return *this;
  };

};

/**Grid map of localize cells*/
struct PointMap : public GridMap<PointMapCell>{

  /**Constructs an empty map*/
  PointMap();

  /**Constructs a map of a  given size, offset and resolution. The mapped is filled with unknown cells.
  @param  size: the size in cells
  @param resolution: the resolution of the map
  @param offset: the location of the cell 0,0 in world coordinates
  */
  PointMap(const IVector2& size, double resolution, const DVector2& offset, PointMapCell& unknown);
  
  /**Copy constructor*/
  PointMap(const PointMap& m);

  /**Assignment operator*/
  PointMap& operator=(const PointMap& m);

  /**Resize operator
     It resizes the map so that the minimum represented world value will be in min and the maximum in max.
     Uninitialized cells will be padded with unknownval.
     @param min: the lower left corner in world coordinates
     @param max: the upper right corner in world coordinates
     @param unknownCell: the value to be used for padding new cells
  */
  PointMap resize(const DVector2 min, DVector2 max, PointMapCell& unknownCell);

  /**Saves a map in  a gnuplot like format;
     @param os: the stream pointing to the output file
  */
  
  void saveToGnuplot(std::ostream& os);
  

protected:
};



#endif
