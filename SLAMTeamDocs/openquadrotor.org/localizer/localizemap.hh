#ifndef LOCALIZEMAP_HXX
#define LOCALIZEMAP_HXX

#include <math_stuff/gridmap.hh>
#include <iostream>
#include <vector>
#include <cstdlib>

#define LOCALIZE_MAP_UNKOWN_GRAYVAL     127
#define LOCALIZE_MAP_FULL_CELL_THRESHOLD     ((255-LOCALIZE_MAP_UNKOWN_GRAYVAL+1)/255.)
#define LOCALIZE_MAP_EMPTY_CELL_THRESHOLD    ((255-LOCALIZE_MAP_UNKOWN_GRAYVAL-1)/255.)
#define LOCALIZE_MAP_DISTANCE_THRESHOLD 5.
#define LOCALIZE_MAP_DISTANCE_FULL_CELL_THRESHOLD 0.51



/**Cell of a localize map.
It contains the occupancy value (between 0. and 1.),
The distance  from the closest obstacle, and a flag which
tells if a cell has been visited.
*/
struct LocalizeMapCell{

  /**constructs a cell initialized with  the given parameters.
   @param occupancy: the occupancy of a cell
   @param distance: the value of the distanceMap
   @param visited: true if a cell is visited
  */
  LocalizeMapCell(float occupancy=0, float distance=0, bool visited=false);

  /**Constructs a cell from a gray value. Dark -> occupied, White -> free.
     The cells are marked by default as non visited.
   */
  LocalizeMapCell(unsigned char g);
  /**the occupancy value of a cell*/
  float occupancy;
  /**the distance from the closest object*/
  float distance;
  /**true if the cell is not unknown*/
  bool visited;

  /**path planning effective cost*/
  double cost;
  /**path planning heuristic cost*/
  double hcost;

  //parent in the path
  LocalizeMapCell* parent;
  // cell map coordinates
  int ix, iy;

  /**conversion to gray value*/
  inline operator unsigned char() const {
    if (visited){
      unsigned char c=(unsigned char)(255*(1-occupancy));
      if (c==LOCALIZE_MAP_UNKOWN_GRAYVAL)
	return c-1;
      return c;
    }
    return 
      LOCALIZE_MAP_UNKOWN_GRAYVAL;
  };
  /**conversion to occupancy value*/
  inline operator float() const {
    return occupancy;
  };

};

/**Grid map of localize cells*/
struct LocalizeMap : public GridMap<LocalizeMapCell>{

  /**Constructs an empty map*/
  LocalizeMap();

  /**Constructs a map of a  given size, offset and resolution. The mapped is filled with unknown cells.
  @param  size: the size in cells
  @param resolution: the resolution of the map
  @param offset: the location of the cell 0,0 in world coordinates
  */
  LocalizeMap(const IVector2& size, double resolution, const DVector2& offset, LocalizeMapCell& unknown);
  
  /**Copy constructor*/
  LocalizeMap(const LocalizeMap& m);

  /**Assignment operator*/
  LocalizeMap& operator=(const LocalizeMap& m);

  /**Resize operator
     It resizes the map so that the minimum represented world value will be in min and the maximum in max.
     Uninitialized cells will be padded with unknownval.
     @param min: the lower left corner in world coordinates
     @param max: the upper right corner in world coordinates
     @param unknownCell: the value to be used for padding new cells
  */
  LocalizeMap resize(const DVector2 min, DVector2 max, LocalizeMapCell& unknownCell);

  /**Loads a map from a pgm file.
     It reads a tag "#resolution" in the pgm comments for determining the resolution.
     If sucha tag is not found the default resolution is set to 0.1m/cell.
     The resolution can be changed after load by altering the member resolution in the grid map.
     @param is: the stream pointing to the pgm file
  */
  
  void loadFromPGM(std::istream& is);
  
  /**saves a map to a pgm file.
     In the comments section of the PGM file all the additional map parameters are added,
     so that subsequent a loading of the map will restore the resolution and the offset
     @param os: the stream pointing to the pgm file
  */
  void saveToPGM(std::ostream& os);
  
  /**saves a map to a ppm file.
  In the comments section of the PPM file all the additional map parameters are added,
  so that subsequent a loading of the map will restore the resolution and the offset
  @param os: the stream pointing to the ppm file
	*/
  void saveToPPM(std::ostream& os, const std::vector<IVector2>& beams, const IVector2& bestPose);
  
  /**saves a map to a pgm file.
     In the comments section of the PGM file all the additional map parameters are added,
     so that subsequent a loading of the map will restore the resolution and the offset
     @param os: the stream pointing to the pgm file
  */
  void saveToPGM(std::ostream& os, double fullThreshold);

  /**saves the distance map to a pgm file.
     @param os: the stream pointing to the pgm file
     @param bool showUnknown: if true it shows the distance map of the unknown cells
  */
  void saveDistanceMapToPGM(std::ostream& os, bool showUnknown=true);

  
  /**fills the visited member of each cell in a localize map based on the occupancy value.
     If the value of a cell is lower than @param fullThreshold or higher than
     @param  emptyThreshold the cell is considered visited, 
     otherwise it is considered unknown.
  */
  void fillVisited( 
		   float fullThreshold=LOCALIZE_MAP_FULL_CELL_THRESHOLD,         
		   float emptyThreshold=LOCALIZE_MAP_EMPTY_CELL_THRESHOLD);
  
  /**computes the distance map, by overwriting the values of distance in the map
     passed as argument.
     @param gm: the map.
     @param distanceThreshold: the threshold at which sthe distance propagation is stopped
     @param fullThreshold: values of occupancy above this will be considered as occupied,
     values below as free.
  */
  void distanceMap(float distanceThreshold=LOCALIZE_MAP_DISTANCE_THRESHOLD, 
		 float fullThreshold=LOCALIZE_MAP_DISTANCE_FULL_CELL_THRESHOLD);


  inline double& north() {return _north;}
  inline const double& north() const {return _north;}

  friend std::istream& operator >> (std::istream& is, LocalizeMap & gm);
  friend std::ostream& operator << (std::ostream& os, LocalizeMap & gm);

protected:
  /**north of the map*/
  double _north;
};


/**Loads a map from a stream.
The file is in plain text format.
See the source for the format specification.
@param is: the stream used for input
@param gm: the loaded grid map
*/
std::istream& operator >> (std::istream& is, LocalizeMap & gm);

/**Saves a map to a stream.
@param os: the stream used for writing
@param gm: the loaded grid map
*/
std::ostream& operator << (std::ostream& os, LocalizeMap & gm);

#endif
